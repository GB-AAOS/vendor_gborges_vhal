#pragma once

#include "CanProviderConfig.h"
#include "IPropertyProvider.h"

#include <linux/can.h>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace android::hardware::automotive::vehicle::gborges::can {

// SocketCAN-backed provider: decodes inbound frames into property updates
// and encodes outbound writes back to frames. One recv thread per provider.
class CanPropertyProvider : public IPropertyProvider {
  public:
    explicit CanPropertyProvider(CanProviderConfig cfg);
    ~CanPropertyProvider() override;

    std::string name() const override { return "can"; }
    std::vector<PropIdAreaId> claimedProperties() const override;
    ProviderFlags flags() const override { return mConfig.flags; }
    ::ndk::ScopedAStatus start() override;
    ::ndk::ScopedAStatus stop() override;
    ::ndk::ScopedAStatus writeValue(const aidlvhal::VehiclePropValue& value) override;
    void setUpdateCallback(PropertyUpdate cb) override;

  private:
    bool tryOpenSocket();
    void installRxFilter();
    void recvLoop();
    void dispatchFrame(const struct can_frame& f);

    static void packValueByType(const pb::Signal& s,
                                double physical,
                                aidlvhal::VehiclePropValue* v);
    static bool unpackValueAsDouble(const pb::Signal& s,
                                    const aidlvhal::VehiclePropValue& v,
                                    double* out);

    CanProviderConfig mConfig;

    std::atomic<bool> mRunning{false};
    std::atomic<int>  mFd{-1};
    std::thread       mRecvThread;

    // Per-can_id frame cache so multiple tx signals sharing an ID accumulate.
    mutable std::mutex mTxLock;
    std::unordered_map<uint32_t, struct can_frame> mTxFrameCache;

    mutable std::mutex mCbLock;
    PropertyUpdate mUpdateCb;
};

}  // namespace android::hardware::automotive::vehicle::gborges::can
