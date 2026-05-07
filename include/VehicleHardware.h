#pragma once

#include "PropertyProviderRegistry.h"

#include <IVehicleHardware.h>
#include <VehicleHalTypes.h>
#include <VehicleUtils.h>

#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace android::hardware::automotive::vehicle::gborges {

namespace aidlvhal = ::aidl::android::hardware::automotive::vehicle;

class VehicleHardware : public IVehicleHardware {
  public:
    VehicleHardware();
    ~VehicleHardware() override = default;

    std::vector<aidlvhal::VehiclePropConfig> getAllPropertyConfigs() const override;

    aidlvhal::StatusCode setValues(
            std::shared_ptr<const SetValuesCallback> callback,
            const std::vector<aidlvhal::SetValueRequest>& requests) override;

    aidlvhal::StatusCode getValues(
            std::shared_ptr<const GetValuesCallback> callback,
            const std::vector<aidlvhal::GetValueRequest>& requests) const override;

    DumpResult dump(const std::vector<std::string>& options) override;
    aidlvhal::StatusCode checkHealth() override;

    void registerOnPropertyChangeEvent(
            std::unique_ptr<const PropertyChangeCallback> callback) override;
    void registerOnPropertySetErrorEvent(
            std::unique_ptr<const PropertySetErrorCallback> callback) override;

    // Must be called before AServiceManager_addService() — the registry is
    // not safe for registerProvider() once binder threads can call us.
    PropertyProviderRegistry& providerRegistry() { return mRegistry; }
    ::ndk::ScopedAStatus startProviders();

  private:
    // The single place to extend when adding properties.
    void seedProperties();

    void addGlobalProp(int32_t propId,
                       aidlvhal::VehiclePropertyAccess access,
                       aidlvhal::VehiclePropertyChangeMode changeMode,
                       aidlvhal::RawPropValues defaultValue,
                       float minSampleRate = 0.0f,
                       float maxSampleRate = 0.0f,
                       bool supportVariableUpdateRate = true);

    void addAreaProp(int32_t propId,
                     int32_t areaId,
                     aidlvhal::VehiclePropertyAccess access,
                     aidlvhal::VehiclePropertyChangeMode changeMode,
                     aidlvhal::RawPropValues defaultValue);

    void onProviderUpdate(aidlvhal::VehiclePropValue value);

    mutable std::mutex mLock;
    std::unordered_map<int32_t, aidlvhal::VehiclePropConfig> mConfigs;
    std::unordered_map<PropIdAreaId, aidlvhal::VehiclePropValue, PropIdAreaIdHash> mValues;

    PropertyProviderRegistry mRegistry;

    // Set once during init (before binder threadpool starts), then read-only.
    std::unique_ptr<const PropertyChangeCallback> mOnPropertyChangeCallback;
    std::unique_ptr<const PropertySetErrorCallback> mOnPropertySetErrorCallback;
};

}  // namespace android::hardware::automotive::vehicle::gborges
