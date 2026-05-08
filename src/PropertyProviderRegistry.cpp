#define LOG_TAG "GborgesVHAL-Registry"

#include "PropertyProviderRegistry.h"

#include <utils/Log.h>

#include <array>

#ifndef GBORGES_VHAL_STRICT_PROVIDERS
#define GBORGES_VHAL_STRICT_PROVIDERS 0
#endif

namespace android::hardware::automotive::vehicle::gborges {

namespace {
::ndk::ScopedAStatus statusFromException(binder_exception_t ex, const char* msg) {
    return ::ndk::ScopedAStatus::fromExceptionCodeWithMessage(ex, msg);
}
}  // namespace

PropertyProviderRegistry::~PropertyProviderRegistry() {
    stopAll();
}

::ndk::ScopedAStatus PropertyProviderRegistry::registerProvider(
        std::unique_ptr<IPropertyProvider> provider) {
    if (!provider) {
        return statusFromException(EX_ILLEGAL_ARGUMENT, "null provider");
    }
    auto claims = provider->claimedProperties();
    {
        std::lock_guard lg(mLock);
        for (const auto& key : claims) {
            if (mIndex.count(key)) {
                ALOGE("provider '%s' claims (propId=0x%x, areaId=0x%x) already owned",
                      provider->name().c_str(), key.propId, key.areaId);
                return statusFromException(EX_ILLEGAL_ARGUMENT, "property already claimed");
            }
        }

        ProviderEntry entry;
        entry.flags = provider->flags();
        if (hasFlag(entry.flags, ProviderFlags::WRITE)) {
            for (const auto& key : provider->listenedSignals()) {
                entry.listened.insert(key);
            }
        }

        if (mOnUpdate) {
            auto upstream = mOnUpdate;
            ProviderFlags flags = entry.flags;
            const std::string nameForLog = provider->name();
            provider->setUpdateCallback([upstream, flags, nameForLog](
                                                aidlvhal::VehiclePropValue v) {
                if (!hasFlag(flags, ProviderFlags::READ)) {
                    ALOGV("dropping update from '%s' (READ=0): prop=0x%x",
                          nameForLog.c_str(), v.prop);
                    return;
                }
                upstream(std::move(v), flags);
            });
        }

        IPropertyProvider* raw = provider.get();
        for (const auto& key : claims) {
            mIndex[key] = raw;
        }
        for (const auto& key : entry.listened) {
            mFanOutIndex[key].push_back(raw);
        }
        if (hasFlag(entry.flags, ProviderFlags::WRITE) &&
            hasFlag(entry.flags, ProviderFlags::ACCEPT_ALL_WRITES)) {
            mBroadcastWriters.push_back(raw);
        }

        ALOGI("registered provider '%s' for %zu claims, %zu listened (flags=0x%x)",
              provider->name().c_str(), claims.size(), entry.listened.size(),
              static_cast<unsigned>(entry.flags));
        entry.ptr = std::move(provider);
        mProviders.push_back(std::move(entry));
    }
    return ::ndk::ScopedAStatus::ok();
}

void PropertyProviderRegistry::setOnUpdate(OnUpdate cb) {
    std::lock_guard lg(mLock);
    mOnUpdate = std::move(cb);
    auto upstream = mOnUpdate;
    for (auto& e : mProviders) {
        ProviderFlags flags = e.flags;
        const std::string nameForLog = e.ptr->name();
        e.ptr->setUpdateCallback([upstream, flags, nameForLog](
                                         aidlvhal::VehiclePropValue v) {
            if (!hasFlag(flags, ProviderFlags::READ)) {
                ALOGV("dropping update from '%s' (READ=0): prop=0x%x",
                      nameForLog.c_str(), v.prop);
                return;
            }
            upstream(std::move(v), flags);
        });
    }
}

::ndk::ScopedAStatus PropertyProviderRegistry::startAll() {
    bool anyRequiredFailed = false;
    {
        std::lock_guard lg(mLock);
        for (auto& e : mProviders) {
            auto st = e.ptr->start();
            if (!st.isOk()) {
                ALOGE("provider '%s' failed to start: %s",
                      e.ptr->name().c_str(), st.getDescription().c_str());
                if (hasFlag(e.flags, ProviderFlags::REQUIRED)) {
                    anyRequiredFailed = true;
                }
            }
        }
    }
#if GBORGES_VHAL_STRICT_PROVIDERS
    if (anyRequiredFailed) {
        return statusFromException(EX_SERVICE_SPECIFIC, "required provider startup failed");
    }
#else
    (void)anyRequiredFailed;
#endif
    return ::ndk::ScopedAStatus::ok();
}

void PropertyProviderRegistry::stopAll() {
    std::lock_guard lg(mLock);
    for (auto& e : mProviders) {
        (void)e.ptr->stop();
    }
}

IPropertyProvider* PropertyProviderRegistry::providerFor(int32_t propId, int32_t areaId) const {
    std::lock_guard lg(mLock);
    auto it = mIndex.find(PropIdAreaId{propId, areaId});
    return it == mIndex.end() ? nullptr : it->second;
}

bool PropertyProviderRegistry::isOwned(int32_t propId, int32_t areaId) const {
    std::lock_guard lg(mLock);
    return mIndex.count(PropIdAreaId{propId, areaId}) > 0;
}

::ndk::ScopedAStatus PropertyProviderRegistry::writeValue(const aidlvhal::VehiclePropValue& value) {
    // Stack-allocated dedup buffer; keeps the hot path alloc-free for the
    // common case (1–3 recipients per write).
    std::array<IPropertyProvider*, 8> recipients{};
    size_t n = 0;
    auto pushUnique = [&](IPropertyProvider* p) {
        if (!p || n == recipients.size()) return;
        for (size_t i = 0; i < n; ++i) {
            if (recipients[i] == p) return;
        }
        recipients[n++] = p;
    };

    auto findEntry = [&](IPropertyProvider* p) -> ProviderEntry* {
        for (auto& e : mProviders) {
            if (e.ptr.get() == p) return &e;
        }
        return nullptr;
    };

    {
        std::lock_guard lg(mLock);
        const PropIdAreaId key{value.prop, value.areaId};

        if (auto it = mIndex.find(key); it != mIndex.end()) {
            if (auto* e = findEntry(it->second);
                e && hasFlag(e->flags, ProviderFlags::WRITE)) {
                pushUnique(it->second);
            }
        }
        if (auto it = mFanOutIndex.find(key); it != mFanOutIndex.end()) {
            for (IPropertyProvider* p : it->second) {
                if (auto* e = findEntry(p);
                    e && hasFlag(e->flags, ProviderFlags::WRITE)) {
                    pushUnique(p);
                }
            }
        }
        for (IPropertyProvider* p : mBroadcastWriters) {
            pushUnique(p);
        }
    }

    if (n == 0) {
        // No registered recipient. Match the historical behaviour for
        // non-owned writes: silently OK (cache update still happens upstream).
        return ::ndk::ScopedAStatus::ok();
    }
    for (size_t i = 0; i < n; ++i) {
        (void)recipients[i]->writeValue(value);
    }
    return ::ndk::ScopedAStatus::ok();
}

void PropertyProviderRegistry::onSubscribe(int32_t propId, int32_t areaId, float sampleRateHz) {
    if (auto* p = providerFor(propId, areaId)) {
        p->onSubscribe(propId, areaId, sampleRateHz);
    }
}

void PropertyProviderRegistry::onUnsubscribe(int32_t propId, int32_t areaId) {
    if (auto* p = providerFor(propId, areaId)) {
        p->onUnsubscribe(propId, areaId);
    }
}

size_t PropertyProviderRegistry::providerCount() const {
    std::lock_guard lg(mLock);
    return mProviders.size();
}

}  // namespace android::hardware::automotive::vehicle::gborges
