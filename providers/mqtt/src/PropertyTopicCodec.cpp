#include "PropertyTopicCodec.h"

#include <json/json.h>

#include <charconv>
#include <cstdio>
#include <sstream>

namespace android::hardware::automotive::vehicle::gborges::mqtt::codec {

namespace {

const char* statusToString(aidlvhal::VehiclePropertyStatus s) {
    using S = aidlvhal::VehiclePropertyStatus;
    switch (s) {
        case S::AVAILABLE:   return "AVAILABLE";
        case S::UNAVAILABLE: return "UNAVAILABLE";
        case S::ERROR:       return "ERROR";
        default:             return "UNAVAILABLE";  // NOT_AVAILABLE_* variants
    }
}

aidlvhal::VehiclePropertyStatus statusFromString(const std::string& s) {
    using S = aidlvhal::VehiclePropertyStatus;
    if (s == "UNAVAILABLE") return S::UNAVAILABLE;
    if (s == "ERROR")       return S::ERROR;
    return S::AVAILABLE;
}

}  // namespace

std::string readTopicFor(int32_t propId, int32_t areaId, std::string_view prefix) {
    char buf[128];
    std::snprintf(buf, sizeof(buf), "%.*s/0x%x/%d",
                  static_cast<int>(prefix.size()), prefix.data(), propId, areaId);
    return std::string(buf);
}

std::string cmdTopicFor(int32_t propId, int32_t areaId, std::string_view prefix) {
    return readTopicFor(propId, areaId, prefix) + "/cmd";
}

std::string cmdSubscribeFilter(std::string_view prefix) {
    return std::string(prefix) + "/+/+/cmd";
}

bool parseTopic(std::string_view topic,
                std::string_view prefix,
                int32_t* propId,
                int32_t* areaId,
                bool* isCmd) {
    if (topic.size() <= prefix.size() + 1) return false;
    if (topic.substr(0, prefix.size()) != prefix) return false;
    if (topic[prefix.size()] != '/') return false;

    std::string_view rest = topic.substr(prefix.size() + 1);
    auto firstSlash = rest.find('/');
    if (firstSlash == std::string_view::npos) return false;
    std::string_view propStr = rest.substr(0, firstSlash);
    std::string_view tail    = rest.substr(firstSlash + 1);

    if (propStr.size() >= 2 && propStr[0] == '0' && (propStr[1] == 'x' || propStr[1] == 'X')) {
        propStr = propStr.substr(2);
    }
    uint32_t pid = 0;
    auto [pEnd, pEc] = std::from_chars(propStr.data(),
                                       propStr.data() + propStr.size(),
                                       pid, /*base=*/16);
    if (pEc != std::errc{} || pEnd != propStr.data() + propStr.size()) return false;

    auto secondSlash = tail.find('/');
    std::string_view areaStr = (secondSlash == std::string_view::npos)
                                       ? tail
                                       : tail.substr(0, secondSlash);
    int32_t aid = 0;
    auto [aEnd, aEc] = std::from_chars(areaStr.data(),
                                       areaStr.data() + areaStr.size(),
                                       aid, /*base=*/10);
    if (aEc != std::errc{} || aEnd != areaStr.data() + areaStr.size()) return false;

    bool cmd = false;
    if (secondSlash != std::string_view::npos) {
        if (tail.substr(secondSlash + 1) != "cmd") return false;
        cmd = true;
    }

    *propId = static_cast<int32_t>(pid);
    *areaId = aid;
    *isCmd  = cmd;
    return true;
}

std::string encode(const aidlvhal::VehiclePropValue& v) {
    Json::Value root(Json::objectValue);
    root["timestamp"] = static_cast<Json::Int64>(v.timestamp);
    root["status"]    = statusToString(v.status);
    root["areaId"]    = v.areaId;

    const auto& rv = v.value;
    if (!rv.int32Values.empty()) {
        Json::Value arr(Json::arrayValue);
        for (auto x : rv.int32Values) arr.append(static_cast<Json::Int>(x));
        root["int32Values"] = std::move(arr);
    }
    if (!rv.int64Values.empty()) {
        Json::Value arr(Json::arrayValue);
        for (auto x : rv.int64Values) arr.append(static_cast<Json::Int64>(x));
        root["int64Values"] = std::move(arr);
    }
    if (!rv.floatValues.empty()) {
        Json::Value arr(Json::arrayValue);
        for (auto x : rv.floatValues) arr.append(static_cast<double>(x));
        root["floatValues"] = std::move(arr);
    }
    if (!rv.stringValue.empty()) {
        root["stringValue"] = rv.stringValue;
    }
    if (!rv.byteValues.empty()) {
        // Hex over base64 to avoid pulling a base64 dep for small payloads.
        std::string hex;
        hex.reserve(rv.byteValues.size() * 2);
        static const char kHex[] = "0123456789abcdef";
        for (auto b : rv.byteValues) {
            hex.push_back(kHex[(b >> 4) & 0xf]);
            hex.push_back(kHex[b & 0xf]);
        }
        root["byteValues"] = std::move(hex);
    }

    Json::StreamWriterBuilder w;
    w["indentation"] = "";
    return Json::writeString(w, root);
}

bool decode(std::string_view payload, aidlvhal::VehiclePropValue* out) {
    Json::CharReaderBuilder rb;
    Json::Value root;
    std::string errs;
    std::istringstream in{std::string(payload)};
    if (!Json::parseFromStream(rb, in, &root, &errs)) return false;
    if (!root.isObject()) return false;

    if (root.isMember("timestamp")) {
        out->timestamp = root["timestamp"].asInt64();
    }
    if (root.isMember("status")) {
        out->status = statusFromString(root["status"].asString());
    } else {
        out->status = aidlvhal::VehiclePropertyStatus::AVAILABLE;
    }

    auto& rv = out->value;
    rv = {};
    if (root.isMember("int32Values") && root["int32Values"].isArray()) {
        for (const auto& x : root["int32Values"]) {
            rv.int32Values.push_back(x.asInt());
        }
    }
    if (root.isMember("int64Values") && root["int64Values"].isArray()) {
        for (const auto& x : root["int64Values"]) {
            rv.int64Values.push_back(x.asInt64());
        }
    }
    if (root.isMember("floatValues") && root["floatValues"].isArray()) {
        for (const auto& x : root["floatValues"]) {
            rv.floatValues.push_back(static_cast<float>(x.asDouble()));
        }
    }
    if (root.isMember("stringValue")) {
        rv.stringValue = root["stringValue"].asString();
    }
    if (root.isMember("byteValues") && root["byteValues"].isString()) {
        const std::string& hex = root["byteValues"].asString();
        if (hex.size() % 2 == 0) {
            rv.byteValues.reserve(hex.size() / 2);
            auto hexv = [](char c) -> int {
                if (c >= '0' && c <= '9') return c - '0';
                if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
                if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
                return -1;
            };
            for (size_t i = 0; i < hex.size(); i += 2) {
                int hi = hexv(hex[i]), lo = hexv(hex[i + 1]);
                if (hi < 0 || lo < 0) return false;
                rv.byteValues.push_back(static_cast<uint8_t>((hi << 4) | lo));
            }
        }
    }
    return true;
}

}  // namespace android::hardware::automotive::vehicle::gborges::mqtt::codec
