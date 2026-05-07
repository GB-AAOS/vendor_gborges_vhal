#define LOG_TAG "GborgesVHAL-Registry"

#include "PropertyProviderRegistry.h"

#include <utils/Log.h>

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
        if (mOnUpdate) {
            auto upstream = mOnUpdate;
            provider->setUpdateCallback([upstream](aidlvhal::VehiclePropValue v) {
                upstream(std::move(v));
            });
        }
        IPropertyProvider* raw = provider.get();
        for (const auto& key : claims) {
            mIndex[key] = raw;
        }
        ALOGI("registered provider '%s' for %zu properties",
              provider->name().c_str(), claims.size());
        mProviders.push_back(std::move(provider));
    }
    return ::ndk::ScopedAStatus::ok();
}

void PropertyProviderRegistry::setOnUpdate(IPropertyProvider::PropertyUpdate cb) {
    std::lock_guard lg(mLock);
    mOnUpdate = std::move(cb);
    auto upstream = mOnUpdate;
    for (auto& p : mProviders) {
        p->setUpdateCallback([upstream](aidlvhal::VehiclePropValue v) {
            upstream(std::move(v));
        });
    }
}

::ndk::ScopedAStatus PropertyProviderRegistry::startAll() {
    std::lock_guard lg(mLock);
    for (auto& p : mProviders) {
        auto st = p->start();
        if (!st.isOk()) {
            ALOGE("provider '%s' failed to start: %s",
                  p->name().c_str(), st.getDescription().c_str());
            return st;
        }
    }
    return ::ndk::ScopedAStatus::ok();
}

void PropertyProviderRegistry::stopAll() {
    std::lock_guard lg(mLock);
    for (auto& p : mProviders) {
        (void)p->stop();
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
    IPropertyProvider* p;
    {
        std::lock_guard lg(mLock);
        auto it = mIndex.find(PropIdAreaId{value.prop, value.areaId});
        if (it == mIndex.end()) {
            return statusFromException(EX_ILLEGAL_ARGUMENT, "no provider for property");
        }
        p = it->second;
    }
    return p->writeValue(value);
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
