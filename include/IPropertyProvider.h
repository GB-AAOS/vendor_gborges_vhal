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

// Per-provider behaviour switches, packed as a bitfield. `Default` keeps the
// safe behaviour; opt out only when the provider semantics warrant it.
//
//   ValidateInbound — if set, values arriving from this provider via the
//                     update callback are type/areaId-checked in
//                     VehicleHardware::onProviderUpdate, and the propId-only
//                     fast-path runs inside the provider itself when
//                     applicable. Clear this only when the provider is the
//                     authoritative source of truth (e.g. a CAN provider
//                     mirroring a real bus where the frame format is the
//                     contract and the VHAL is just a shadow). Applies to
//                     all of the provider's claims uniformly.
enum class ProviderFlags : uint32_t {
    None            = 0,
    ValidateInbound = 1u << 0,

    Default         = ValidateInbound,
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
    virtual ProviderFlags flags() const { return ProviderFlags::Default; }

    virtual ::ndk::ScopedAStatus start() = 0;
    virtual ::ndk::ScopedAStatus stop() = 0;

    virtual ::ndk::ScopedAStatus writeValue(const aidlvhal::VehiclePropValue& value) = 0;

    virtual void onSubscribe(int32_t /*propId*/, int32_t /*areaId*/, float /*sampleRateHz*/) {}
    virtual void onUnsubscribe(int32_t /*propId*/, int32_t /*areaId*/) {}

    virtual void setUpdateCallback(PropertyUpdate cb) = 0;
};

}  // namespace android::hardware::automotive::vehicle::gborges
