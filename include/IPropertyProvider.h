#pragma once

#include <VehicleHalTypes.h>
#include <VehicleUtils.h>

#include <android/binder_auto_utils.h>

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace android::hardware::automotive::vehicle::gborges {

namespace aidlvhal = ::aidl::android::hardware::automotive::vehicle;

// Per-provider behaviour switches; bits apply uniformly to every claim.
enum class ProviderFlags : uint32_t {
    NONE = 0,

    // Run propId-only type/cardinality and areaId-membership checks on values
    // flowing in from this provider before they hit the VHAL cache.
    VALIDATE_INBOUND = 1u << 0,

    // VHAL → provider gate. Forward setValue() into provider->writeValue()
    // for owned, listened, or ACCEPT_ALL_WRITES props. Clear to mute fully.
    WRITE = 1u << 1,

    // Provider → VHAL gate. Propagate the provider's emissions through
    // VehicleHardware::onProviderUpdate to the cache and AIDL subscribers.
    READ = 1u << 2,

    // With WRITE: provider accepts every setValue regardless of claims or
    // listenedSignals. Clear = accept only the (claims ∪ listened) subset.
    ACCEPT_ALL_WRITES = 1u << 3,

    // Marks the provider as load-bearing. Under strict build mode
    // (-DGBORGES_VHAL_STRICT_PROVIDERS=1) a start() failure aborts boot;
    // under soft mode (default) it is logged and ignored.
    REQUIRED = 1u << 4,

    DEFAULT = VALIDATE_INBOUND | WRITE | READ,
};

constexpr ProviderFlags operator|(ProviderFlags a, ProviderFlags b) {
    return static_cast<ProviderFlags>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b));
}
constexpr ProviderFlags operator&(ProviderFlags a, ProviderFlags b) {
    return static_cast<ProviderFlags>(static_cast<uint32_t>(a) & static_cast<uint32_t>(b));
}
constexpr ProviderFlags operator~(ProviderFlags a) {
    return static_cast<ProviderFlags>(~static_cast<uint32_t>(a));
}
constexpr ProviderFlags& operator|=(ProviderFlags& a, ProviderFlags b) { return a = a | b; }
constexpr ProviderFlags& operator&=(ProviderFlags& a, ProviderFlags b) { return a = a & b; }

constexpr bool hasFlag(ProviderFlags set, ProviderFlags bit) {
    return (set & bit) == bit;
}

class IPropertyProvider {
  public:
    using PropertyUpdate = std::function<void(aidlvhal::VehiclePropValue)>;

    virtual ~IPropertyProvider() = default;

    virtual std::string name() const = 0;
    virtual std::vector<PropIdAreaId> claimedProperties() const = 0;
    virtual ProviderFlags flags() const { return ProviderFlags::DEFAULT; }

    // Extra (prop, area) keys this provider wants writeValue() forwarded for,
    // beyond its claims. Snapshotted once at registration; never re-queried.
    virtual std::vector<PropIdAreaId> listenedSignals() const { return {}; }

    virtual ::ndk::ScopedAStatus start() = 0;
    virtual ::ndk::ScopedAStatus stop() = 0;

    virtual ::ndk::ScopedAStatus writeValue(const aidlvhal::VehiclePropValue& value) = 0;

    virtual void onSubscribe(int32_t /*propId*/, int32_t /*areaId*/, float /*sampleRateHz*/) {}
    virtual void onUnsubscribe(int32_t /*propId*/, int32_t /*areaId*/) {}

    virtual void setUpdateCallback(PropertyUpdate cb) = 0;
};

}  // namespace android::hardware::automotive::vehicle::gborges
