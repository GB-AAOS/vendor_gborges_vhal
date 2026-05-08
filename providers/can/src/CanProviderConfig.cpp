#define LOG_TAG "GborgesVHAL-CAN"

#include "CanProviderConfig.h"

#include <google/protobuf/text_format.h>

#include <utils/Log.h>

#include <fstream>
#include <sstream>
#include <unordered_set>

namespace android::hardware::automotive::vehicle::gborges::can {

namespace {

bool readFile(const std::string& path, std::string* out) {
    std::ifstream in(path);
    if (!in.is_open()) return false;
    std::ostringstream ss;
    ss << in.rdbuf();
    *out = ss.str();
    return true;
}

}  // namespace

uint32_t CanProviderConfig::packId(uint32_t canId, bool extended) {
    constexpr uint32_t EFF = 0x80000000u;
    return (extended ? EFF : 0u) | (canId & 0x1FFFFFFFu);
}

std::optional<CanProviderConfig> CanProviderConfig::loadFromFile(const std::string& path) {
    std::string text;
    if (!readFile(path, &text)) {
        ALOGI("CAN config: %s not found or unreadable", path.c_str());
        return std::nullopt;
    }

    pb::CanProviderConfig parsed;
    if (!::google::protobuf::TextFormat::ParseFromString(text, &parsed)) {
        ALOGE("CAN config: failed to parse %s as textproto", path.c_str());
        return std::nullopt;
    }

    CanProviderConfig cfg;
    if (!parsed.bus_name().empty()) {
        cfg.busName = parsed.bus_name();
    }

    cfg.mappings.reserve(parsed.properties_size());
    std::unordered_set<uint64_t> seenClaims;

    for (int i = 0; i < parsed.properties_size(); ++i) {
        const pb::PropertyMapping& m = parsed.properties(i);
        const bool hasRx = m.has_rx();
        const bool hasTx = m.has_tx();
        if (!hasRx && !hasTx) {
            ALOGW("CAN config: prop=0x%x area=%d has neither rx nor tx; skipping",
                  m.prop_id(), m.area_id());
            continue;
        }
        const size_t idx = cfg.mappings.size();
        cfg.mappings.push_back(m);

        if (hasRx) {
            const uint32_t key = packId(m.rx().can_id(), m.rx().extended());
            cfg.rxByCanId[key].push_back(idx);
        }
        if (hasTx) {
            const PropIdAreaId k{m.prop_id(), m.area_id()};
            auto [it, inserted] = cfg.txByProp.emplace(k, idx);
            if (!inserted) {
                ALOGW("CAN config: duplicate tx for prop=0x%x area=%d; "
                      "keeping first, ignoring later",
                      m.prop_id(), m.area_id());
            }
        }

        const uint64_t claimKey = (static_cast<uint64_t>(static_cast<uint32_t>(m.prop_id())) << 32)
                                  | static_cast<uint32_t>(m.area_id());
        if (seenClaims.insert(claimKey).second) {
            cfg.claims.push_back(PropIdAreaId{m.prop_id(), m.area_id()});
        }
    }

    ALOGI("CAN config: loaded %s — bus=%s mappings=%zu claims=%zu rxIds=%zu txProps=%zu",
          path.c_str(), cfg.busName.c_str(),
          cfg.mappings.size(), cfg.claims.size(),
          cfg.rxByCanId.size(), cfg.txByProp.size());
    return cfg;
}

}  // namespace android::hardware::automotive::vehicle::gborges::can
