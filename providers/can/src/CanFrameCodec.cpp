#include "CanFrameCodec.h"

#include <cmath>
#include <cstdint>

namespace android::hardware::automotive::vehicle::gborges::can {

namespace {

inline int getBit(const uint8_t* data, int bitNum) {
    if (bitNum < 0 || bitNum >= 64) return 0;
    return (data[bitNum / 8] >> (bitNum % 8)) & 0x1;
}

inline void setBit(uint8_t* data, int bitNum, int value) {
    if (bitNum < 0 || bitNum >= 64) return;
    const uint8_t mask = static_cast<uint8_t>(1u << (bitNum % 8));
    if (value & 1) {
        data[bitNum / 8] |= mask;
    } else {
        data[bitNum / 8] &= static_cast<uint8_t>(~mask);
    }
}

uint64_t lengthMask(uint32_t length) {
    if (length >= 64) return ~uint64_t{0};
    return (uint64_t{1} << length) - 1;
}

int64_t signExtend(uint64_t raw, uint32_t length) {
    if (length == 0 || length > 64) return 0;
    if (length == 64) return static_cast<int64_t>(raw);
    const uint64_t signBit = uint64_t{1} << (length - 1);
    if (raw & signBit) {
        return static_cast<int64_t>(raw | (~uint64_t{0} << length));
    }
    return static_cast<int64_t>(raw);
}

uint64_t extractRaw(const uint8_t* data, const pb::Signal& s) {
    uint64_t raw = 0;
    if (s.byte_order() == pb::BO_LITTLE_ENDIAN) {
        for (uint32_t i = 0; i < s.length_bits(); ++i) {
            const int bit = getBit(data, static_cast<int>(s.start_bit() + i));
            raw |= (static_cast<uint64_t>(bit) << i);
        }
    } else {
        // Motorola: start_bit is the MSB; walk bits "down" within the byte,
        // then jump to bit 7 of the next byte at the byte boundary.
        int bitNum = static_cast<int>(s.start_bit());
        for (uint32_t i = 0; i < s.length_bits(); ++i) {
            const int bit = getBit(data, bitNum);
            raw = (raw << 1) | static_cast<uint64_t>(bit);
            if (bitNum % 8 == 0) {
                bitNum += 15;  // (next byte * 8) + 7  ==  bitNum + 15 when bit%8==0
            } else {
                bitNum -= 1;
            }
        }
    }
    return raw;
}

void insertRaw(uint8_t* data, const pb::Signal& s, uint64_t raw) {
    if (s.byte_order() == pb::BO_LITTLE_ENDIAN) {
        for (uint32_t i = 0; i < s.length_bits(); ++i) {
            setBit(data, static_cast<int>(s.start_bit() + i),
                   static_cast<int>((raw >> i) & 1u));
        }
    } else {
        int bitNum = static_cast<int>(s.start_bit());
        for (int i = static_cast<int>(s.length_bits()) - 1; i >= 0; --i) {
            setBit(data, bitNum, static_cast<int>((raw >> i) & 1u));
            if (bitNum % 8 == 0) {
                bitNum += 15;
            } else {
                bitNum -= 1;
            }
        }
    }
}

}  // namespace

double CanFrameCodec::decode(const struct can_frame& f, const pb::Signal& s) {
    if (s.length_bits() == 0 || s.length_bits() > 64) return 0.0;
    const uint64_t raw = extractRaw(f.data, s);
    const int64_t  v   = s.is_signed() ? signExtend(raw, s.length_bits())
                                       : static_cast<int64_t>(raw);
    const double scale = (s.scale() != 0.0) ? s.scale() : 1.0;
    return static_cast<double>(v) * scale + s.offset();
}

bool CanFrameCodec::encode(struct can_frame& f, const pb::Signal& s, double physical) {
    if (s.length_bits() == 0 || s.length_bits() > 64) return false;
    const double scale = (s.scale() != 0.0) ? s.scale() : 1.0;
    const double rawF  = (physical - s.offset()) / scale;
    int64_t raw        = static_cast<int64_t>(std::llround(rawF));

    int64_t minV;
    int64_t maxV;
    if (s.is_signed()) {
        if (s.length_bits() == 64) {
            minV = INT64_MIN;
            maxV = INT64_MAX;
        } else {
            maxV = (int64_t{1} << (s.length_bits() - 1)) - 1;
            minV = -(int64_t{1} << (s.length_bits() - 1));
        }
    } else {
        minV = 0;
        maxV = (s.length_bits() == 64) ? INT64_MAX
                                       : static_cast<int64_t>((uint64_t{1} << s.length_bits()) - 1);
    }
    if (raw > maxV) raw = maxV;
    if (raw < minV) raw = minV;

    const uint64_t uraw = static_cast<uint64_t>(raw) & lengthMask(s.length_bits());
    insertRaw(f.data, s, uraw);
    return true;
}

}  // namespace android::hardware::automotive::vehicle::gborges::can
