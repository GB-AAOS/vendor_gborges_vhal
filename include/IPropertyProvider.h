#pragma once

#include <VehicleHalTypes.h>
#include <VehicleUtils.h>

#include <android/binder_auto_utils.h>

#include <functional>
#include <string>
#include <vector>

namespace android::hardware::automotive::vehicle::gborges {

namespace aidlvhal = ::aidl::android::hardware::automotive::vehicle;

class IPropertyProvider {
  public:
    using PropertyUpdate = std::function<void(aidlvhal::VehiclePropValue)>;

    virtual ~IPropertyProvider() = default;

    virtual std::string name() const = 0;
    virtual std::vector<PropIdAreaId> claimedProperties() const = 0;

    virtual ::ndk::ScopedAStatus start() = 0;
    virtual ::ndk::ScopedAStatus stop() = 0;

    virtual ::ndk::ScopedAStatus writeValue(const aidlvhal::VehiclePropValue& value) = 0;

    virtual void onSubscribe(int32_t /*propId*/, int32_t /*areaId*/, float /*sampleRateHz*/) {}
    virtual void onUnsubscribe(int32_t /*propId*/, int32_t /*areaId*/) {}

    virtual void setUpdateCallback(PropertyUpdate cb) = 0;
};

}  // namespace android::hardware::automotive::vehicle::gborges
