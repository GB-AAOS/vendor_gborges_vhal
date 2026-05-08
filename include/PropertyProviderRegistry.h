#pragma once

#include "IPropertyProvider.h"

#include <memory>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace android::hardware::automotive::vehicle::gborges {

class PropertyProviderRegistry {
  public:
    // Update sink: receives a parsed value plus the originating provider's
    // flags so VehicleHardware can decide whether to validate or shortcut.
    using OnUpdate = std::function<void(aidlvhal::VehiclePropValue, ProviderFlags)>;

    PropertyProviderRegistry() = default;
    ~PropertyProviderRegistry();

    PropertyProviderRegistry(const PropertyProviderRegistry&) = delete;
    PropertyProviderRegistry& operator=(const PropertyProviderRegistry&) = delete;

    // Returns ILLEGAL_ARGUMENT if any claimed (propId, areaId) is already owned.
    ::ndk::ScopedAStatus registerProvider(std::unique_ptr<IPropertyProvider> provider);

    void setOnUpdate(OnUpdate cb);

    ::ndk::ScopedAStatus startAll();
    void stopAll();

    IPropertyProvider* providerFor(int32_t propId, int32_t areaId) const;
    bool isOwned(int32_t propId, int32_t areaId) const;

    ::ndk::ScopedAStatus writeValue(const aidlvhal::VehiclePropValue& value);
    void onSubscribe(int32_t propId, int32_t areaId, float sampleRateHz);
    void onUnsubscribe(int32_t propId, int32_t areaId);

    size_t providerCount() const;

  private:
    // Per-provider state snapshotted at registration so the writeValue hot
    // path doesn't re-call provider->flags() / provider->listenedSignals().
    struct ProviderEntry {
        std::unique_ptr<IPropertyProvider> ptr;
        ProviderFlags flags;
        std::unordered_set<PropIdAreaId, PropIdAreaIdHash> listened;
    };

    mutable std::mutex mLock;
    std::vector<ProviderEntry> mProviders;

    // Owner index: claims → unique provider (existing semantics).
    std::unordered_map<PropIdAreaId, IPropertyProvider*, PropIdAreaIdHash> mIndex;

    // Listener index: prop key → providers that have it in listenedSignals().
    std::unordered_map<PropIdAreaId, std::vector<IPropertyProvider*>,
                       PropIdAreaIdHash> mFanOutIndex;

    // Providers with WRITE | ACCEPT_ALL_WRITES. Pre-filtered at registration.
    std::vector<IPropertyProvider*> mBroadcastWriters;

    OnUpdate mOnUpdate;
};

}  // namespace android::hardware::automotive::vehicle::gborges
