#pragma once

#include <VehicleHalTypes.h>
#include <VehicleUtils.h>

#include <cstdio>
#include <cstring>
#include <string>

namespace android::hardware::automotive::vehicle::gborges {

namespace aidlvhal = ::aidl::android::hardware::automotive::vehicle;

// Lightweight, propId-only type/cardinality check. Used by providers that
// have no access to the prop's full VehiclePropConfig (e.g. the MQTT provider
// receiving a /cmd before VehicleHardware can look up mConfigs). MIXED is
// permissive here — only the config-aware check below validates its
// structure.
inline bool validatePropertyType(int32_t propId,
                                 const aidlvhal::RawPropValues& v,
                                 std::string* err) {
    using T = aidlvhal::VehiclePropertyType;
    const T type = ::android::hardware::automotive::vehicle::getPropType(propId);

    auto fail = [&](const char* reason) {
        if (err) *err = reason;
        return false;
    };

    const size_t i32 = v.int32Values.size();
    const size_t i64 = v.int64Values.size();
    const size_t flt = v.floatValues.size();
    const size_t byt = v.byteValues.size();
    const bool   str = !v.stringValue.empty();

    auto onlyI32 = [&] { return i64 == 0 && flt == 0 && byt == 0 && !str; };
    auto onlyI64 = [&] { return i32 == 0 && flt == 0 && byt == 0 && !str; };
    auto onlyFlt = [&] { return i32 == 0 && i64 == 0 && byt == 0 && !str; };
    auto onlyStr = [&] { return i32 == 0 && i64 == 0 && flt == 0 && byt == 0 && str; };
    auto onlyByt = [&] { return i32 == 0 && i64 == 0 && flt == 0 && byt > 0 && !str; };

    switch (type) {
        case T::BOOLEAN:
            if (!onlyI32() || i32 != 1) return fail("BOOLEAN: expected exactly one int32Values element");
            return true;
        case T::INT32:
            if (!onlyI32() || i32 != 1) return fail("INT32: expected exactly one int32Values element");
            return true;
        case T::INT32_VEC:
            if (!onlyI32() || i32 == 0) return fail("INT32_VEC: expected >=1 int32Values element");
            return true;
        case T::INT64:
            if (!onlyI64() || i64 != 1) return fail("INT64: expected exactly one int64Values element");
            return true;
        case T::INT64_VEC:
            if (!onlyI64() || i64 == 0) return fail("INT64_VEC: expected >=1 int64Values element");
            return true;
        case T::FLOAT:
            if (!onlyFlt() || flt != 1) return fail("FLOAT: expected exactly one floatValues element");
            return true;
        case T::FLOAT_VEC:
            if (!onlyFlt() || flt == 0) return fail("FLOAT_VEC: expected >=1 floatValues element");
            return true;
        case T::STRING:
            if (!onlyStr()) return fail("STRING: expected only stringValue");
            return true;
        case T::BYTES:
            if (!onlyByt()) return fail("BYTES: expected only byteValues");
            return true;
        case T::MIXED:
            return true;
        default:
            return fail("unknown property type");
    }
}

// Config-aware check. Used by VehicleHardware where mConfigs is available.
// Wraps upstream checkPropValue (type + cardinality + MIXED structure) and
// adds an areaId-vs-areaConfigs membership check.
inline bool validatePropertyWrite(const aidlvhal::VehiclePropValue& value,
                                  const aidlvhal::VehiclePropConfig& cfg,
                                  std::string* err) {
    auto res = ::android::hardware::automotive::vehicle::checkPropValue(value, &cfg);
    if (!res.ok()) {
        if (err) *err = res.error().message();
        return false;
    }
    for (const auto& a : cfg.areaConfigs) {
        if (a.areaId == value.areaId) return true;
    }
    if (err) {
        char buf[96];
        std::snprintf(buf, sizeof(buf),
                      "areaId=%d not declared on prop=0x%x",
                      value.areaId, value.prop);
        *err = buf;
    }
    return false;
}

}  // namespace android::hardware::automotive::vehicle::gborges
