#pragma once

#include <VehicleHalTypes.h>

#include <optional>
#include <string>
#include <string_view>

namespace android::hardware::automotive::vehicle::gborges::mqtt::codec {

namespace aidlvhal = ::aidl::android::hardware::automotive::vehicle;

// Topic shape: <prefix>/0x<propIdHex>/<areaId> for reads, …/cmd for writes.
std::string readTopicFor(int32_t propId, int32_t areaId, std::string_view prefix);
std::string cmdTopicFor(int32_t propId, int32_t areaId, std::string_view prefix);
std::string cmdSubscribeFilter(std::string_view prefix);

bool parseTopic(std::string_view topic,
                std::string_view prefix,
                int32_t* propId,
                int32_t* areaId,
                bool* isCmd);

// JSON envelope: { timestamp:<ns>, status:"AVAILABLE|UNAVAILABLE|ERROR",
// areaId:<int>, <one of> int32Values|int64Values|floatValues:[...]
// | stringValue:"..." | byteValues:"<hex>" }.
std::string encode(const aidlvhal::VehiclePropValue& v);

// Caller pre-fills out->prop and out->areaId from the topic; decode fills
// timestamp, status, value.
bool decode(std::string_view payload, aidlvhal::VehiclePropValue* out);

}  // namespace android::hardware::automotive::vehicle::gborges::mqtt::codec
