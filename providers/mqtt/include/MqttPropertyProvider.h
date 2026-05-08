#pragma once

#include "IPropertyProvider.h"

#include <nng/nng.h>

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace android::hardware::automotive::vehicle::gborges::mqtt {

class MqttPropertyProvider : public IPropertyProvider {
  public:
    struct Config {
        std::string brokerUrl       = "mqtt-tcp://127.0.0.1:1883";
        std::string clientId        = "gborges-vhal";
        std::string topicPrefix     = "gborges/vhal";
        int         keepAliveSecs   = 60;
        int         qos             = 0;
        bool        retainPublishes = true;
        // Broker mirrors every VHAL setValue, not just the props it claims.
        ProviderFlags flags         = ProviderFlags::DEFAULT
                                    | ProviderFlags::ACCEPT_ALL_WRITES;
        std::vector<PropIdAreaId> claimedProperties;
    };

    explicit MqttPropertyProvider(Config cfg);
    ~MqttPropertyProvider() override;

    std::string name() const override { return "mqtt"; }
    std::vector<PropIdAreaId> claimedProperties() const override;
    ProviderFlags flags() const override { return mConfig.flags; }
    ::ndk::ScopedAStatus start() override;
    ::ndk::ScopedAStatus stop() override;
    ::ndk::ScopedAStatus writeValue(const aidlvhal::VehiclePropValue& value) override;
    void setUpdateCallback(PropertyUpdate cb) override;

  private:
    void recvLoop();
    void handleIncoming(const std::string& topic,
                        const uint8_t* payload,
                        size_t payloadLen);

    Config mConfig;

    nng_socket mSocket = NNG_SOCKET_INITIALIZER;
    nng_dialer mDialer = NNG_DIALER_INITIALIZER;

    std::atomic<bool> mRunning{false};
    std::thread mRecvThread;

    mutable std::mutex mCbLock;
    PropertyUpdate mUpdateCb;
};

}  // namespace android::hardware::automotive::vehicle::gborges::mqtt
