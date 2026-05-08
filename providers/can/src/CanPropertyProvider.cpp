#define LOG_TAG "GborgesVHAL-CAN"

#include "CanPropertyProvider.h"

#include "CanFrameCodec.h"

#include <utils/Log.h>
#include <utils/SystemClock.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>

namespace android::hardware::automotive::vehicle::gborges::can {

namespace {

constexpr auto kReconnectInterval = std::chrono::seconds(2);
constexpr int  kRecvTimeoutMs     = 200;

::ndk::ScopedAStatus statusFromException(binder_exception_t ex, const char* msg) {
    return ::ndk::ScopedAStatus::fromExceptionCodeWithMessage(ex, msg);
}

}  // namespace

CanPropertyProvider::CanPropertyProvider(CanProviderConfig cfg)
    : mConfig(std::move(cfg)) {}

CanPropertyProvider::~CanPropertyProvider() {
    (void)stop();
}

std::vector<PropIdAreaId> CanPropertyProvider::claimedProperties() const {
    return mConfig.claims;
}

void CanPropertyProvider::setUpdateCallback(PropertyUpdate cb) {
    std::lock_guard lg(mCbLock);
    mUpdateCb = std::move(cb);
}

bool CanPropertyProvider::tryOpenSocket() {
    int prev = mFd.exchange(-1);
    if (prev >= 0) ::close(prev);

    int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (fd < 0) {
        ALOGE("socket(PF_CAN, SOCK_RAW, CAN_RAW) failed: %s", strerror(errno));
        return false;
    }

    struct ifreq ifr;
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, mConfig.busName.c_str(), IFNAMSIZ - 1);
    if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
        ALOGW("ioctl(SIOCGIFINDEX, %s): %s — bus not up yet?",
              mConfig.busName.c_str(), strerror(errno));
        ::close(fd);
        return false;
    }

    struct sockaddr_can addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
        ALOGW("bind(%s): %s", mConfig.busName.c_str(), strerror(errno));
        ::close(fd);
        return false;
    }

    struct timeval tv;
    tv.tv_sec  = kRecvTimeoutMs / 1000;
    tv.tv_usec = (kRecvTimeoutMs % 1000) * 1000;
    if (::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        ALOGW("setsockopt(SO_RCVTIMEO) on %s: %s — recv will block",
              mConfig.busName.c_str(), strerror(errno));
    }

    mFd.store(fd);
    installRxFilter();
    ALOGI("CAN provider bound to %s (fd=%d, claims=%zu, rxIds=%zu)",
          mConfig.busName.c_str(), fd, mConfig.claims.size(),
          mConfig.rxByCanId.size());
    return true;
}

void CanPropertyProvider::installRxFilter() {
    const int fd = mFd.load();
    if (fd < 0) return;
    if (mConfig.rxByCanId.empty()) {
        // Drop-all filter so the kernel doesn't wake recv on traffic we ignore.
        struct can_filter none = {.can_id = 0, .can_mask = CAN_EFF_MASK | CAN_EFF_FLAG};
        if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &none, sizeof(none)) < 0) {
            ALOGW("setsockopt(CAN_RAW_FILTER, drop-all): %s", strerror(errno));
        }
        return;
    }

    std::vector<struct can_filter> filters;
    filters.reserve(mConfig.rxByCanId.size());
    for (const auto& [packed, _] : mConfig.rxByCanId) {
        struct can_filter f;
        const bool extended = (packed & 0x80000000u) != 0;
        f.can_id   = (packed & 0x1FFFFFFFu) | (extended ? CAN_EFF_FLAG : 0);
        f.can_mask = (extended ? CAN_EFF_MASK : CAN_SFF_MASK) | CAN_EFF_FLAG;
        filters.push_back(f);
    }
    if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
                     static_cast<socklen_t>(filters.size() * sizeof(struct can_filter))) < 0) {
        ALOGW("setsockopt(CAN_RAW_FILTER, n=%zu): %s",
              filters.size(), strerror(errno));
    }
}

::ndk::ScopedAStatus CanPropertyProvider::start() {
    if (mRunning.exchange(true)) {
        return ::ndk::ScopedAStatus::ok();
    }
    if (!tryOpenSocket()) {
        ALOGW("CAN provider start: bus %s not ready; will keep retrying",
              mConfig.busName.c_str());
    }
    mRecvThread = std::thread(&CanPropertyProvider::recvLoop, this);
    ALOGI("CAN provider started (bus=%s, claims=%zu)",
          mConfig.busName.c_str(), mConfig.claims.size());
    return ::ndk::ScopedAStatus::ok();
}

::ndk::ScopedAStatus CanPropertyProvider::stop() {
    if (!mRunning.exchange(false)) {
        return ::ndk::ScopedAStatus::ok();
    }
    if (mRecvThread.joinable()) {
        mRecvThread.join();
    }
    int fd = mFd.exchange(-1);
    if (fd >= 0) ::close(fd);
    ALOGI("CAN provider stopped");
    return ::ndk::ScopedAStatus::ok();
}

void CanPropertyProvider::recvLoop() {
    while (mRunning.load()) {
        int fd = mFd.load();
        if (fd < 0) {
            if (!tryOpenSocket()) {
                std::this_thread::sleep_for(kReconnectInterval);
                continue;
            }
            fd = mFd.load();
        }

        struct can_frame frame;
        std::memset(&frame, 0, sizeof(frame));
        ssize_t n = ::read(fd, &frame, sizeof(frame));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
                continue;
            }
            ALOGW("read(%s): %s — reopening", mConfig.busName.c_str(), strerror(errno));
            int prev = mFd.exchange(-1);
            if (prev >= 0) ::close(prev);
            std::this_thread::sleep_for(kReconnectInterval);
            continue;
        }
        if (n != static_cast<ssize_t>(sizeof(frame))) {
            ALOGW("short read on %s: %zd bytes", mConfig.busName.c_str(), n);
            continue;
        }
        dispatchFrame(frame);
    }
}

void CanPropertyProvider::dispatchFrame(const struct can_frame& f) {
    const bool extended = (f.can_id & CAN_EFF_FLAG) != 0;
    const uint32_t rawId = f.can_id & (extended ? CAN_EFF_MASK : CAN_SFF_MASK);
    const uint32_t key   = CanProviderConfig::packId(rawId, extended);

    auto it = mConfig.rxByCanId.find(key);
    if (it == mConfig.rxByCanId.end()) return;

    PropertyUpdate cb;
    {
        std::lock_guard lg(mCbLock);
        cb = mUpdateCb;
    }
    if (!cb) return;

    for (size_t idx : it->second) {
        const pb::PropertyMapping& m = mConfig.mappings[idx];
        const pb::Signal& s          = m.rx();
        const double physical        = CanFrameCodec::decode(f, s);

        aidlvhal::VehiclePropValue v;
        v.prop      = m.prop_id();
        v.areaId    = m.area_id();
        v.timestamp = ::android::elapsedRealtimeNano();
        v.status    = aidlvhal::VehiclePropertyStatus::AVAILABLE;
        packValueByType(s, physical, &v);

        ALOGI("rx can_id=0x%x ext=%d dlc=%u prop=0x%x area=%d phys=%.4f",
              rawId, extended ? 1 : 0, f.can_dlc, v.prop, v.areaId, physical);
        cb(std::move(v));
    }
}

::ndk::ScopedAStatus CanPropertyProvider::writeValue(const aidlvhal::VehiclePropValue& value) {
    if (!mRunning.load()) {
        return statusFromException(EX_ILLEGAL_STATE, "provider not running");
    }
    auto it = mConfig.txByProp.find(PropIdAreaId{value.prop, value.areaId});
    if (it == mConfig.txByProp.end()) {
        return statusFromException(EX_UNSUPPORTED_OPERATION, "no tx mapping for property");
    }
    const pb::PropertyMapping& m = mConfig.mappings[it->second];
    const pb::Signal& s          = m.tx();

    double physical = 0.0;
    if (!unpackValueAsDouble(s, value, &physical)) {
        ALOGW("writeValue prop=0x%x area=%d: value missing scalar matching value_type=%d",
              value.prop, value.areaId, static_cast<int>(s.value_type()));
        return statusFromException(EX_ILLEGAL_ARGUMENT, "value type mismatch");
    }

    const uint32_t key = CanProviderConfig::packId(s.can_id(), s.extended());
    int fd = mFd.load();
    if (fd < 0) {
        return statusFromException(EX_TRANSACTION_FAILED, "CAN socket not bound");
    }

    struct can_frame frame;
    {
        std::lock_guard lg(mTxLock);
        auto& cached = mTxFrameCache[key];
        if (cached.can_id == 0 && cached.can_dlc == 0) {
            std::memset(&cached, 0, sizeof(cached));
            cached.can_id = (s.can_id() & (s.extended() ? CAN_EFF_MASK : CAN_SFF_MASK))
                            | (s.extended() ? CAN_EFF_FLAG : 0u);
            cached.can_dlc = static_cast<__u8>(s.dlc() == 0 ? 8 : std::min<uint32_t>(s.dlc(), 8));
        }
        if (!CanFrameCodec::encode(cached, s, physical)) {
            return statusFromException(EX_ILLEGAL_ARGUMENT, "signal length out of range");
        }
        frame = cached;
    }

    ssize_t n = ::write(fd, &frame, sizeof(frame));
    if (n != static_cast<ssize_t>(sizeof(frame))) {
        ALOGW("write(can_id=0x%x): %s", s.can_id(), strerror(errno));
        return statusFromException(EX_TRANSACTION_FAILED, "CAN write failed");
    }
    ALOGI("tx can_id=0x%x ext=%d prop=0x%x area=%d phys=%.4f",
          s.can_id(), s.extended() ? 1 : 0, value.prop, value.areaId, physical);
    return ::ndk::ScopedAStatus::ok();
}

void CanPropertyProvider::packValueByType(const pb::Signal& s,
                                          double physical,
                                          aidlvhal::VehiclePropValue* v) {
    auto& rv = v->value;
    rv.int32Values.clear();
    rv.int64Values.clear();
    rv.floatValues.clear();
    switch (s.value_type()) {
        case pb::VT_INT32:
            rv.int32Values = {static_cast<int32_t>(std::llround(physical))};
            break;
        case pb::VT_INT64:
            rv.int64Values = {static_cast<int64_t>(std::llround(physical))};
            break;
        case pb::VT_FLOAT:
            rv.floatValues = {static_cast<float>(physical)};
            break;
        case pb::VT_BOOLEAN:
            rv.int32Values = {physical != 0.0 ? 1 : 0};
            break;
        default:
            rv.floatValues = {static_cast<float>(physical)};
            break;
    }
}

bool CanPropertyProvider::unpackValueAsDouble(const pb::Signal& s,
                                              const aidlvhal::VehiclePropValue& v,
                                              double* out) {
    const auto& rv = v.value;
    switch (s.value_type()) {
        case pb::VT_INT32:
        case pb::VT_BOOLEAN:
            if (rv.int32Values.empty()) return false;
            *out = static_cast<double>(rv.int32Values[0]);
            return true;
        case pb::VT_INT64:
            if (rv.int64Values.empty()) return false;
            *out = static_cast<double>(rv.int64Values[0]);
            return true;
        case pb::VT_FLOAT:
            if (rv.floatValues.empty()) return false;
            *out = static_cast<double>(rv.floatValues[0]);
            return true;
        default:
            return false;
    }
}

}  // namespace android::hardware::automotive::vehicle::gborges::can
