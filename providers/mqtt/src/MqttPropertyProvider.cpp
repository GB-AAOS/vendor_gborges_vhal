#define LOG_TAG "GborgesVHAL-MQTT"

#include "MqttPropertyProvider.h"
#include "PropertyTopicCodec.h"

#include <utils/Log.h>
#include <utils/SystemClock.h>

#include <nng/mqtt/mqtt_client.h>
#include <nng/nng.h>
#include <nng/protocol/mqtt/mqtt.h>

#include <cstring>

namespace android::hardware::automotive::vehicle::gborges::mqtt {

namespace {
namespace codec_ns = ::android::hardware::automotive::vehicle::gborges::mqtt::codec;

::ndk::ScopedAStatus statusFromException(binder_exception_t ex, const char* msg) {
    return ::ndk::ScopedAStatus::fromExceptionCodeWithMessage(ex, msg);
}

}  // namespace

MqttPropertyProvider::MqttPropertyProvider(Config cfg) : mConfig(std::move(cfg)) {}

MqttPropertyProvider::~MqttPropertyProvider() {
    (void)stop();
}

std::vector<PropIdAreaId> MqttPropertyProvider::claimedProperties() const {
    return mConfig.claimedProperties;
}

void MqttPropertyProvider::setUpdateCallback(PropertyUpdate cb) {
    std::lock_guard lg(mCbLock);
    mUpdateCb = std::move(cb);
}

::ndk::ScopedAStatus MqttPropertyProvider::start() {
    if (mRunning.exchange(true)) {
        return ::ndk::ScopedAStatus::ok();
    }

    int rv = nng_mqtt_client_open(&mSocket);
    if (rv != 0) {
        ALOGE("nng_mqtt_client_open failed: %s", nng_strerror(rv));
        mRunning = false;
        return statusFromException(EX_TRANSACTION_FAILED, "nng_mqtt_client_open");
    }

    rv = nng_dialer_create(&mDialer, mSocket, mConfig.brokerUrl.c_str());
    if (rv != 0) {
        ALOGE("nng_dialer_create(%s) failed: %s",
              mConfig.brokerUrl.c_str(), nng_strerror(rv));
        nng_close(mSocket);
        mRunning = false;
        return statusFromException(EX_TRANSACTION_FAILED, "nng_dialer_create");
    }

    nng_msg* connmsg = nullptr;
    nng_mqtt_msg_alloc(&connmsg, 0);
    nng_mqtt_msg_set_packet_type(connmsg, NNG_MQTT_CONNECT);
    nng_mqtt_msg_set_connect_proto_version(connmsg, 4);
    nng_mqtt_msg_set_connect_keep_alive(connmsg, mConfig.keepAliveSecs);
    nng_mqtt_msg_set_connect_clean_session(connmsg, true);
    nng_mqtt_msg_set_connect_client_id(connmsg, mConfig.clientId.c_str());
    nng_dialer_set_ptr(mDialer, NNG_OPT_MQTT_CONNMSG, connmsg);

    rv = nng_dialer_start(mDialer, NNG_FLAG_NONBLOCK);
    if (rv != 0) {
        // Non-fatal: NNG retries with exponential backoff up to
        // NNG_OPT_MQTT_RECONNECT_BACKOFF_MAX.
        ALOGW("nng_dialer_start: %s (will reconnect)", nng_strerror(rv));
    }
    ALOGI("MQTT provider started, broker=%s clientId=%s prefix=%s claimed=%zu",
          mConfig.brokerUrl.c_str(), mConfig.clientId.c_str(),
          mConfig.topicPrefix.c_str(), mConfig.claimedProperties.size());

    mRecvThread = std::thread(&MqttPropertyProvider::recvLoop, this);
    return ::ndk::ScopedAStatus::ok();
}

::ndk::ScopedAStatus MqttPropertyProvider::stop() {
    if (!mRunning.exchange(false)) {
        return ::ndk::ScopedAStatus::ok();
    }
    // nng_close unblocks any nng_recvmsg in mRecvThread.
    if (mSocket.id != 0) {
        nng_close(mSocket);
        mSocket = NNG_SOCKET_INITIALIZER;
    }
    if (mRecvThread.joinable()) {
        mRecvThread.join();
    }
    ALOGI("MQTT provider stopped");
    return ::ndk::ScopedAStatus::ok();
}

::ndk::ScopedAStatus MqttPropertyProvider::writeValue(const aidlvhal::VehiclePropValue& value) {
    if (!mRunning.load()) {
        return statusFromException(EX_ILLEGAL_STATE, "provider not running");
    }
    std::string topic   = codec_ns::readTopicFor(value.prop, value.areaId, mConfig.topicPrefix);
    std::string payload = codec_ns::encode(value);

    nng_msg* msg = nullptr;
    nng_mqtt_msg_alloc(&msg, 0);
    nng_mqtt_msg_set_packet_type(msg, NNG_MQTT_PUBLISH);
    nng_mqtt_msg_set_publish_topic(msg, topic.c_str());
    nng_mqtt_msg_set_publish_qos(msg, mConfig.qos);
    nng_mqtt_msg_set_publish_retain(msg, mConfig.retainPublishes);
    nng_mqtt_msg_set_publish_payload(
            msg,
            reinterpret_cast<uint8_t*>(payload.data()),
            payload.size());

    int rv = nng_sendmsg(mSocket, msg, NNG_FLAG_NONBLOCK);
    if (rv != 0) {
        ALOGW("publish failed (%s) topic=%s", nng_strerror(rv), topic.c_str());
        nng_msg_free(msg);
        return statusFromException(EX_TRANSACTION_FAILED, nng_strerror(rv));
    }
    return ::ndk::ScopedAStatus::ok();
}

void MqttPropertyProvider::recvLoop() {
    // Safe to subscribe before CONNACK: NNG queues the SUBSCRIBE and replays
    // it on every (re)connect.
    std::string filter = codec_ns::cmdSubscribeFilter(mConfig.topicPrefix);
    nng_mqtt_topic_qos sub = {
            .qos = static_cast<uint8_t>(mConfig.qos),
            .topic = {
                    .buf    = reinterpret_cast<uint8_t*>(filter.data()),
                    .length = static_cast<uint32_t>(filter.size()),
            },
    };
    int rv = nng_mqtt_subscribe(mSocket, &sub, 1, nullptr);
    if (rv != 0) {
        ALOGW("subscribe(%s) returned %s — will retry on reconnect",
              filter.c_str(), nng_strerror(rv));
    } else {
        ALOGI("subscribed to %s", filter.c_str());
    }

    while (mRunning.load()) {
        nng_msg* msg = nullptr;
        rv = nng_recvmsg(mSocket, &msg, 0);
        if (rv != 0) {
            if (!mRunning.load()) break;  // socket closed by stop()
            ALOGW("nng_recvmsg: %s", nng_strerror(rv));
            continue;
        }
        uint32_t topicLen = 0;
        const char* topicPtr =
                nng_mqtt_msg_get_publish_topic(msg, &topicLen);
        uint32_t payloadLen = 0;
        uint8_t* payloadPtr =
                nng_mqtt_msg_get_publish_payload(msg, &payloadLen);
        if (topicPtr && topicLen > 0) {
            std::string topic(topicPtr, topicLen);
            handleIncoming(topic, payloadPtr, payloadLen);
        }
        nng_msg_free(msg);
    }
}

void MqttPropertyProvider::handleIncoming(const std::string& topic,
                                          const uint8_t* payload,
                                          size_t payloadLen) {
    int32_t propId = 0;
    int32_t areaId = 0;
    bool isCmd = false;
    if (!codec_ns::parseTopic(topic, mConfig.topicPrefix, &propId, &areaId, &isCmd)) {
        ALOGW("ignoring unparseable topic: %s", topic.c_str());
        return;
    }
    if (!isCmd) {
        // Provider only subscribes to .../cmd to avoid echo loops with its
        // own publishes; ignore anything else if the broker delivers it.
        return;
    }

    aidlvhal::VehiclePropValue v;
    v.prop = propId;
    v.areaId = areaId;
    v.timestamp = ::android::elapsedRealtimeNano();
    std::string_view sv(reinterpret_cast<const char*>(payload), payloadLen);
    if (!codec_ns::decode(sv, &v)) {
        ALOGW("decode failed for topic=%s", topic.c_str());
        return;
    }
    if (v.timestamp == 0) {
        v.timestamp = ::android::elapsedRealtimeNano();
    }

    PropertyUpdate cb;
    {
        std::lock_guard lg(mCbLock);
        cb = mUpdateCb;
    }
    if (cb) cb(std::move(v));
}

}  // namespace android::hardware::automotive::vehicle::gborges::mqtt
