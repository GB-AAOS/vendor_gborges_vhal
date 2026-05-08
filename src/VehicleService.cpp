#define LOG_TAG "GborgesVehicleService"

#include "CanProviderConfig.h"
#include "CanPropertyProvider.h"
#include "MqttPropertyProvider.h"
#include "VehicleHardware.h"

#include <DefaultVehicleHal.h>
#include <PropertyUtils.h>
#include <VehicleHalTypes.h>

#include <android/binder_manager.h>
#include <android/binder_process.h>
#include <utils/Log.h>

namespace {

namespace aidlvhal = ::aidl::android::hardware::automotive::vehicle;
using ::android::hardware::automotive::vehicle::PropIdAreaId;
using ::android::hardware::automotive::vehicle::toInt;
using ::android::hardware::automotive::vehicle::gborges::VehicleHardware;
using ::android::hardware::automotive::vehicle::gborges::can::CanProviderConfig;
using ::android::hardware::automotive::vehicle::gborges::can::CanPropertyProvider;
using ::android::hardware::automotive::vehicle::gborges::mqtt::MqttPropertyProvider;

constexpr const char* kCanConfigPath = "/system/etc/gborges/vhal_can_props.pb";

std::vector<PropIdAreaId> mqttClaims() {
    using P = aidlvhal::VehicleProperty;
    return {
            {toInt(P::PERF_VEHICLE_SPEED), 0},
            {toInt(P::IGNITION_STATE),     0},
            {toInt(P::GEAR_SELECTION),     0},
            {toInt(P::PARKING_BRAKE_ON),   0},
            {toInt(P::NIGHT_MODE),         0},
            {toInt(P::HVAC_POWER_ON),      ::android::hardware::automotive::vehicle::HVAC_ALL},
            {toInt(P::DISPLAY_BRIGHTNESS), 0},
    };
}

}  // namespace

int main(int /*argc*/, char* /*argv*/[]) {
    ALOGI("Starting gborges VHAL...");
    if (!ABinderProcess_setThreadPoolMaxThreadCount(4)) {
        ALOGE("failed to set thread pool max thread count");
        return 1;
    }
    ABinderProcess_startThreadPool();

    auto hardware = std::make_unique<VehicleHardware>();

    MqttPropertyProvider::Config mqttCfg;
    mqttCfg.brokerUrl = "mqtt-tcp://127.0.0.1:1883";
    mqttCfg.clientId = "gborges-vhal";
    mqttCfg.topicPrefix = "gborges/vhal";
    mqttCfg.claimedProperties = mqttClaims();

    auto mqttProvider = std::make_unique<MqttPropertyProvider>(std::move(mqttCfg));
    auto regSt = hardware->providerRegistry().registerProvider(std::move(mqttProvider));
    if (!regSt.isOk()) {
        ALOGE("MQTT provider registration failed: %s", regSt.getDescription().c_str());
        return 1;
    }

    // CAN provider is opt-in: missing/empty textproto = no-op (non-fatal).
    if (auto canCfg = CanProviderConfig::loadFromFile(kCanConfigPath)) {
        if (canCfg->claims.empty()) {
            ALOGI("CAN provider disabled: %s has no property mappings", kCanConfigPath);
        } else {
            auto canProvider = std::make_unique<CanPropertyProvider>(std::move(*canCfg));
            auto canRegSt = hardware->providerRegistry().registerProvider(std::move(canProvider));
            if (!canRegSt.isOk()) {
                ALOGE("CAN provider registration failed: %s",
                      canRegSt.getDescription().c_str());
            }
        }
    } else {
        ALOGI("CAN provider disabled (no/invalid config at %s)", kCanConfigPath);
    }

    auto startSt = hardware->startProviders();
    if (!startSt.isOk()) {
        // Don't fail boot for transient provider issues (e.g. broker not yet
        // up); NNG will reconnect once the broker comes online.
        ALOGE("provider startup failed: %s", startSt.getDescription().c_str());
    }

    auto vhal = ::ndk::SharedRefBase::make<
            ::android::hardware::automotive::vehicle::DefaultVehicleHal>(std::move(hardware));

    binder_exception_t err = AServiceManager_addService(
            vhal->asBinder().get(),
            "android.hardware.automotive.vehicle.IVehicle/default");
    if (err != EX_NONE) {
        ALOGE("failed to register IVehicle/default, exception: %d", err);
        return 1;
    }

    ALOGI("gborges VHAL ready");
    ABinderProcess_joinThreadPool();
    return 0;
}
