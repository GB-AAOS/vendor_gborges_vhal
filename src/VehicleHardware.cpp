#define LOG_TAG "GborgesVHAL"

#include "VehicleHardware.h"

#include "PropertyValidation.h"

#include <PropertyUtils.h>

#include <utils/Log.h>
#include <utils/SystemClock.h>

namespace android::hardware::automotive::vehicle::gborges {

namespace {
using aidlvhal::FuelType;
using aidlvhal::GetValueRequest;
using aidlvhal::GetValueResult;
using aidlvhal::RawPropValues;
using aidlvhal::SetValueRequest;
using aidlvhal::SetValueResult;
using aidlvhal::StatusCode;
using aidlvhal::VehicleAreaConfig;
using aidlvhal::VehicleGear;
using aidlvhal::VehicleIgnitionState;
using aidlvhal::VehicleProperty;
using aidlvhal::VehiclePropConfig;
using aidlvhal::VehiclePropertyAccess;
using aidlvhal::VehiclePropertyChangeMode;
using aidlvhal::VehiclePropertyStatus;
using aidlvhal::VehiclePropValue;
}  // namespace

VehicleHardware::VehicleHardware() {
    seedProperties();
    mRegistry.setOnUpdate([this](aidlvhal::VehiclePropValue v, ProviderFlags f) {
        onProviderUpdate(std::move(v), f);
    });
}

::ndk::ScopedAStatus VehicleHardware::startProviders() {
    return mRegistry.startAll();
}

void VehicleHardware::onProviderUpdate(VehiclePropValue value, ProviderFlags flags) {
    if (hasFlag(flags, ProviderFlags::VALIDATE_INBOUND)) {
        VehiclePropConfig cfg;
        {
            std::lock_guard lg(mLock);
            auto it = mConfigs.find(value.prop);
            if (it == mConfigs.end()) {
                ALOGW("rejecting provider update: unknown prop=0x%x", value.prop);
                return;
            }
            cfg = it->second;
        }
        std::string err;
        if (!validatePropertyWrite(value, cfg, &err)) {
            ALOGW("rejecting provider update: prop=0x%x area=%d: %s",
                  value.prop, value.areaId, err.c_str());
            return;
        }
    }
    if (value.timestamp == 0) {
        value.timestamp = elapsedRealtimeNano();
    }
    {
        std::lock_guard lg(mLock);
        mValues[PropIdAreaId{value.prop, value.areaId}] = value;
    }
    if (mOnPropertyChangeCallback) {
        std::vector<VehiclePropValue> batch;
        batch.push_back(std::move(value));
        (*mOnPropertyChangeCallback)(std::move(batch));
    }
}

void VehicleHardware::addGlobalProp(int32_t propId,
                                    VehiclePropertyAccess access,
                                    VehiclePropertyChangeMode changeMode,
                                    RawPropValues defaultValue,
                                    float minSampleRate,
                                    float maxSampleRate,
                                    bool supportVariableUpdateRate) {
    VehicleAreaConfig area = {
            .areaId = 0,
            .access = access,
            .supportVariableUpdateRate = supportVariableUpdateRate,
    };
    VehiclePropConfig cfg = {
            .prop = propId,
            .access = access,
            .changeMode = changeMode,
            .areaConfigs = {std::move(area)},
            .minSampleRate = minSampleRate,
            .maxSampleRate = maxSampleRate,
    };
    mConfigs[propId] = std::move(cfg);

    VehiclePropValue val = {
            .timestamp = elapsedRealtimeNano(),
            .areaId = 0,
            .prop = propId,
            .status = VehiclePropertyStatus::AVAILABLE,
            .value = std::move(defaultValue),
    };
    mValues[PropIdAreaId{propId, 0}] = std::move(val);
}

void VehicleHardware::addAreaProp(int32_t propId,
                                  int32_t areaId,
                                  VehiclePropertyAccess access,
                                  VehiclePropertyChangeMode changeMode,
                                  RawPropValues defaultValue) {
    VehicleAreaConfig area = {
            .areaId = areaId,
            .access = access,
            .supportVariableUpdateRate = true,
    };
    VehiclePropConfig cfg = {
            .prop = propId,
            .access = access,
            .changeMode = changeMode,
            .areaConfigs = {std::move(area)},
    };
    mConfigs[propId] = std::move(cfg);

    VehiclePropValue val = {
            .timestamp = elapsedRealtimeNano(),
            .areaId = areaId,
            .prop = propId,
            .status = VehiclePropertyStatus::AVAILABLE,
            .value = std::move(defaultValue),
    };
    mValues[PropIdAreaId{propId, areaId}] = std::move(val);
}

void VehicleHardware::seedProperties() {
    using PA = VehiclePropertyAccess;
    using PCM = VehiclePropertyChangeMode;

    // Vehicle identity (STATIC).
    addGlobalProp(toInt(VehicleProperty::INFO_VIN), PA::READ, PCM::STATIC,
                  RawPropValues{.stringValue = "GBORGES000000001"});
    addGlobalProp(toInt(VehicleProperty::INFO_MAKE), PA::READ, PCM::STATIC,
                  RawPropValues{.stringValue = "gborges"});
    addGlobalProp(toInt(VehicleProperty::INFO_MODEL), PA::READ, PCM::STATIC,
                  RawPropValues{.stringValue = "gbrpi"});
    addGlobalProp(toInt(VehicleProperty::INFO_MODEL_YEAR), PA::READ, PCM::STATIC,
                  RawPropValues{.int32Values = {2026}});
    addGlobalProp(toInt(VehicleProperty::INFO_FUEL_TYPE), PA::READ, PCM::STATIC,
                  RawPropValues{.int32Values = {toInt(FuelType::FUEL_TYPE_UNLEADED)}});
    addGlobalProp(toInt(VehicleProperty::INFO_FUEL_CAPACITY), PA::READ, PCM::STATIC,
                  RawPropValues{.floatValues = {50000.0f}});
    addGlobalProp(toInt(VehicleProperty::INFO_DRIVER_SEAT), PA::READ, PCM::STATIC,
                  RawPropValues{.int32Values = {SEAT_1_LEFT}});

    // Driving state — required by CarDrivingStateService.
    addGlobalProp(toInt(VehicleProperty::PERF_VEHICLE_SPEED), PA::READ, PCM::CONTINUOUS,
                  RawPropValues{.floatValues = {0.0f}},
                  /* minSampleRate */ 1.0f, /* maxSampleRate */ 10.0f);
    addGlobalProp(toInt(VehicleProperty::GEAR_SELECTION), PA::READ, PCM::ON_CHANGE,
                  RawPropValues{.int32Values = {toInt(VehicleGear::GEAR_PARK)}});
    addGlobalProp(toInt(VehicleProperty::PARKING_BRAKE_ON), PA::READ, PCM::ON_CHANGE,
                  RawPropValues{.int32Values = {1}});
    addGlobalProp(toInt(VehicleProperty::IGNITION_STATE), PA::READ, PCM::ON_CHANGE,
                  RawPropValues{.int32Values = {toInt(VehicleIgnitionState::ON)}});

    // Quiet SystemUI / HVAC chatter.
    addGlobalProp(toInt(VehicleProperty::NIGHT_MODE), PA::READ, PCM::ON_CHANGE,
                  RawPropValues{.int32Values = {0}});
    addAreaProp(toInt(VehicleProperty::HVAC_POWER_ON), HVAC_ALL, PA::READ_WRITE,
                PCM::ON_CHANGE, RawPropValues{.int32Values = {1}});

    // Display brightness (0–100). CarPowerService writes this.
    addGlobalProp(toInt(VehicleProperty::DISPLAY_BRIGHTNESS), PA::READ_WRITE,
                  PCM::ON_CHANGE, RawPropValues{.int32Values = {100}});

    // VHAL liveness — DefaultVehicleHal pushes its own heartbeat values
    addGlobalProp(toInt(VehicleProperty::VHAL_HEARTBEAT), PA::READ, PCM::ON_CHANGE,
                  RawPropValues{.int64Values = {0}},
                  /* minSampleRate */ 0.0f, /* maxSampleRate */ 0.0f,
                  /* supportVariableUpdateRate */ false);
}

std::vector<aidlvhal::VehiclePropConfig> VehicleHardware::getAllPropertyConfigs() const {
    std::lock_guard lg(mLock);
    std::vector<VehiclePropConfig> out;
    out.reserve(mConfigs.size());
    for (const auto& [_, cfg] : mConfigs) {
        out.push_back(cfg);
    }
    return out;
}

StatusCode VehicleHardware::getValues(
        std::shared_ptr<const GetValuesCallback> callback,
        const std::vector<GetValueRequest>& requests) const {
    std::vector<GetValueResult> results;
    results.reserve(requests.size());
    {
        std::lock_guard lg(mLock);
        for (const auto& req : requests) {
            GetValueResult r{.requestId = req.requestId};
            auto it = mValues.find(PropIdAreaId{req.prop.prop, req.prop.areaId});
            if (it == mValues.end()) {
                r.status = StatusCode::INVALID_ARG;
            } else {
                r.status = StatusCode::OK;
                r.prop = it->second;
            }
            results.push_back(std::move(r));
        }
    }
    (*callback)(std::move(results));
    return StatusCode::OK;
}

StatusCode VehicleHardware::setValues(
        std::shared_ptr<const SetValuesCallback> callback,
        const std::vector<SetValueRequest>& requests) {
    std::vector<SetValueResult> results;
    std::vector<VehiclePropValue> changed;
    results.reserve(requests.size());
    changed.reserve(requests.size());
    for (const auto& req : requests) {
        SetValueResult r{.requestId = req.requestId, .status = StatusCode::OK};

        VehiclePropValue stored = req.value;
        stored.timestamp = elapsedRealtimeNano();
        stored.status = VehiclePropertyStatus::AVAILABLE;

        VehiclePropConfig cfg;
        bool exists;
        {
            std::lock_guard lg(mLock);
            auto it = mConfigs.find(stored.prop);
            exists = it != mConfigs.end();
            if (exists) cfg = it->second;
        }
        if (!exists) {
            r.status = StatusCode::INVALID_ARG;
            results.push_back(std::move(r));
            continue;
        }

        std::string verr;
        if (!validatePropertyWrite(stored, cfg, &verr)) {
            ALOGW("rejecting setValues: prop=0x%x area=%d: %s",
                  stored.prop, stored.areaId, verr.c_str());
            r.status = StatusCode::INVALID_ARG;
            results.push_back(std::move(r));
            continue;
        }

        // Fan out to providers (owner + listeners + ACCEPT_ALL_WRITES).
        // The registry silently returns OK when there are no recipients,
        // so calling unconditionally is cheap on cache-only props.
        const bool owned = mRegistry.isOwned(stored.prop, stored.areaId);
        if (auto st = mRegistry.writeValue(stored); !st.isOk()) {
            ALOGW("provider writeValue failed for prop=0x%x area=%d: %s",
                  stored.prop, stored.areaId, st.getDescription().c_str());
        }

        {
            std::lock_guard lg(mLock);
            mValues[PropIdAreaId{stored.prop, stored.areaId}] = stored;
        }

        std::string vsum;
        if (!stored.value.int32Values.empty()) {
            vsum = "int32=" + std::to_string(stored.value.int32Values[0]);
        } else if (!stored.value.int64Values.empty()) {
            vsum = "int64=" + std::to_string(stored.value.int64Values[0]);
        } else if (!stored.value.floatValues.empty()) {
            vsum = "float=" + std::to_string(stored.value.floatValues[0]);
        } else if (!stored.value.stringValue.empty()) {
            vsum = "string=" + stored.value.stringValue;
        } else {
            vsum = "(no scalar)";
        }
        ALOGI("setValue prop=0x%x area=%d %s %s",
              stored.prop, stored.areaId,
              owned ? "provider" : "cache",
              vsum.c_str());

        changed.push_back(stored);
        results.push_back(std::move(r));
    }
    (*callback)(std::move(results));
    if (!changed.empty() && mOnPropertyChangeCallback) {
        (*mOnPropertyChangeCallback)(std::move(changed));
    }
    return StatusCode::OK;
}

DumpResult VehicleHardware::dump(const std::vector<std::string>& /*options*/) {
    std::lock_guard lg(mLock);
    DumpResult r;
    r.callerShouldDumpState = true;
    r.buffer = "gborges VehicleHardware: " + std::to_string(mConfigs.size()) +
               " properties, " + std::to_string(mRegistry.providerCount()) +
               " providers\n";
    return r;
}

StatusCode VehicleHardware::checkHealth() {
    return StatusCode::OK;
}

void VehicleHardware::registerOnPropertyChangeEvent(
        std::unique_ptr<const PropertyChangeCallback> callback) {
    mOnPropertyChangeCallback = std::move(callback);
}

void VehicleHardware::registerOnPropertySetErrorEvent(
        std::unique_ptr<const PropertySetErrorCallback> callback) {
    mOnPropertySetErrorCallback = std::move(callback);
}

}  // namespace android::hardware::automotive::vehicle::gborges
