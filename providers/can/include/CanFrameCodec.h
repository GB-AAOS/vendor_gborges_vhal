#pragma once

#include "proto/can_provider.pb.h"

#include <linux/can.h>

#include <cstdint>

namespace android::hardware::automotive::vehicle::gborges::can {

namespace pb = ::android::hardware::automotive::vehicle::gborges::proto;

// DBC-compatible bit pack/unpack for one Signal in a classic-CAN frame.
class CanFrameCodec {
  public:
    // Returns raw_bits * scale + offset, sign-extended when is_signed.
    static double decode(const struct can_frame& f, const pb::Signal& s);

    // ORs the encoded bits into f.data so siblings on the same can_id stack.
    static bool encode(struct can_frame& f, const pb::Signal& s, double physical);
};

}  // namespace android::hardware::automotive::vehicle::gborges::can
