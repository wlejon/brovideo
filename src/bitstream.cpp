// Keyframe detection: Annex B NAL types for H.264 / HEVC, OBU and frame
// headers for AV1, the kind byte for Raw.
#include "brovideo/bitstream.h"

#include <cstddef>

namespace brovideo {

namespace {

// Calls f(nal) for each Annex B NAL unit (the bytes after a start code);
// stops early when f returns true. Returns whether any call did.
template <class F>
bool for_each_nal(std::span<const uint8_t> b, F f) {
    const size_t n = b.size();
    size_t i = 0;
    // Find the first start code.
    auto next_start = [&](size_t from) -> size_t {
        for (size_t j = from; j + 2 < n; ++j) {
            if (b[j] == 0 && b[j + 1] == 0 && b[j + 2] == 1) return j;
        }
        return n;
    };
    i = next_start(0);
    while (i < n) {
        const size_t begin = i + 3;
        size_t end = next_start(begin);
        const size_t following = end;
        // A four-byte start code's leading zero (and trailing_zero bytes)
        // belong to no NAL.
        while (end > begin && b[end - 1] == 0) --end;
        if (end > begin && f(b.subspan(begin, end - begin))) return true;
        i = following;
    }
    return false;
}

bool h264_key(std::span<const uint8_t> b) {
    return for_each_nal(b, [](std::span<const uint8_t> nal) { return (nal[0] & 0x1F) == 5; });
}

bool hevc_key(std::span<const uint8_t> b) {
    return for_each_nal(b, [](std::span<const uint8_t> nal) {
        const int type = (nal[0] >> 1) & 0x3F;
        return type >= 16 && type <= 21;  // BLA_W_LP .. CRA_NUT (IRAP)
    });
}

bool read_leb128(std::span<const uint8_t> b, size_t& pos, uint64_t& value) {
    value = 0;
    for (int i = 0; i < 8; ++i) {
        if (pos >= b.size()) return false;
        const uint8_t byte = b[pos++];
        value |= uint64_t(byte & 0x7F) << (7 * i);
        if (!(byte & 0x80)) return true;
    }
    return false;
}

bool av1_key(std::span<const uint8_t> b) {
    constexpr int kSequenceHeader = 1, kFrameHeader = 3, kFrame = 6;
    bool have_sequence = false;
    bool reduced_still = false;
    size_t pos = 0;
    while (pos < b.size()) {
        const uint8_t h = b[pos++];
        const int type = (h >> 3) & 0x0F;
        const bool extension = (h & 0x04) != 0;
        const bool has_size = (h & 0x02) != 0;
        if (extension) ++pos;
        uint64_t size = 0;
        if (has_size) {
            if (!read_leb128(b, pos, size)) return false;
        } else {
            size = pos <= b.size() ? b.size() - pos : 0;
        }
        if (pos > b.size() || size > b.size() - pos) return false;
        if (size > 0) {
            const uint8_t first = b[pos];
            if (type == kSequenceHeader) {
                have_sequence = true;
                // seq_profile (3), still_picture (1), reduced_still_picture_header (1)
                reduced_still = (first & 0x08) != 0;
            } else if ((type == kFrameHeader || type == kFrame) && have_sequence) {
                if (reduced_still) return true;
                // show_existing_frame (1), then frame_type (2): 0 is KEY_FRAME.
                if (first & 0x80) return false;
                return ((first >> 5) & 0x03) == 0;
            }
        }
        pos += size_t(size);
    }
    return false;
}

}  // namespace

bool is_keyframe(Codec codec, std::span<const uint8_t> packet) noexcept {
    switch (codec) {
        case Codec::Raw: return !packet.empty() && packet[0] == 0;  // kind 0: intra (docs/raw.md)
        case Codec::H264: return h264_key(packet);
        case Codec::HEVC: return hevc_key(packet);
        case Codec::AV1: return av1_key(packet);
    }
    return false;
}

}  // namespace brovideo
