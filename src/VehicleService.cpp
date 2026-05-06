#define LOG_TAG "GborgesVehicleService"

#include "VehicleHardware.h"

#include <DefaultVehicleHal.h>

#include <android/binder_manager.h>
#include <android/binder_process.h>
#include <utils/Log.h>

using ::android::hardware::automotive::vehicle::DefaultVehicleHal;
using ::android::hardware::automotive::vehicle::gborges::VehicleHardware;

int main(int /*argc*/, char* /*argv*/[]) {
    ALOGI("Starting gborges VHAL...");
    if (!ABinderProcess_setThreadPoolMaxThreadCount(4)) {
        ALOGE("failed to set thread pool max thread count");
        return 1;
    }
    ABinderProcess_startThreadPool();

    auto hardware = std::make_unique<VehicleHardware>();
    auto vhal = ::ndk::SharedRefBase::make<DefaultVehicleHal>(std::move(hardware));

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
