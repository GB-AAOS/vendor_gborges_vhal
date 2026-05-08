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
    NONE             = 0,
    VALIDATE_INBOUND = 1u << 0,  // Type/areaId-check inbound values before they reach the cache.
    WRITE            = 1u << 1,  // Forward VHAL setValue() to the provider's writeValue().
    READ             = 1u << 2,  // Propagate provider updates into the VHAL cache and subscribers.

    DEFAULT          = VALIDATE_INBOUND | WRITE | READ,
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

    virtual ::ndk::ScopedAStatus start() = 0;
    virtual ::ndk::ScopedAStatus stop() = 0;

    virtual ::ndk::ScopedAStatus writeValue(const aidlvhal::VehiclePropValue& value) = 0;

    virtual void onSubscribe(int32_t /*propId*/, int32_t /*areaId*/, float /*sampleRateHz*/) {}
    virtual void onUnsubscribe(int32_t /*propId*/, int32_t /*areaId*/) {}

    virtual void setUpdateCallback(PropertyUpdate cb) = 0;
};

}  // namespace android::hardware::automotive::vehicle::gborges
