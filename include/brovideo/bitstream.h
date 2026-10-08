#pragma once
// Just enough bitstream parsing to know whether a packet can start
// decoding. Decoders use it to report a lost sync (hardware decoders conceal
// a predicted frame with no reference instead of failing it); a consumer can
// use it to tell keyframes in a stream it did not encode.

#include "brovideo/codec.h"

#include <cstdint>
#include <span>

namespace brovideo {

// True when the packet holds a random access point a decoder can start at:
// an H.264 IDR slice, an HEVC IRAP picture (IDR, CRA, BLA), an AV1 temporal
// unit with a sequence header and a key frame, a Raw intra packet.
[[nodiscard]] bool is_keyframe(Codec codec, std::span<const uint8_t> packet) noexcept;

}  // namespace brovideo
