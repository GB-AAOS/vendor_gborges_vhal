#pragma once

#include "IPropertyProvider.h"
#include "proto/can_provider.pb.h"

#include <VehicleUtils.h>

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace android::hardware::automotive::vehicle::gborges::can {

namespace pb = ::android::hardware::automotive::vehicle::gborges::proto;

// Parsed + indexed view of vhal_can_props.pb. Immutable after load.
struct CanProviderConfig {
    std::string busName = "can0";
    ProviderFlags flags = ProviderFlags::DEFAULT;

    std::vector<pb::PropertyMapping> mappings;

    // (extended<<31 | can_id) → indices into `mappings` whose rx matches.
    std::unordered_map<uint32_t, std::vector<size_t>> rxByCanId;

    // (prop_id, area_id) → index into `mappings` for tx routing.
    std::unordered_map<PropIdAreaId, size_t, PropIdAreaIdHash> txByProp;

    std::vector<PropIdAreaId> claims;

    // std::nullopt on missing/unreadable/unparseable file (errors are logged).
    static std::optional<CanProviderConfig> loadFromFile(const std::string& path);

    static uint32_t packId(uint32_t canId, bool extended);
};

}  // namespace android::hardware::automotive::vehicle::gborges::can
