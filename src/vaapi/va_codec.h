#pragma once
// The per-codec half of the VA-API encoder. The common half (va_encoder.cpp)
// owns the display, the surfaces, the colour conversion, the rate control and
// the picture sequence; a CodecImpl turns one picture of that sequence into
// the codec's parameter buffers and packed headers.
//
// The sequence is low-latency and the same for every codec: a keyframe (IDR /
// KEY_FRAME), then predicted frames, each referencing only the picture before
// it, until the next keyframe. Two reconstructed surfaces alternate: the
// current picture and its one reference.

#include "vaapi/va_device.h"
#include "vaapi/va_util.h"

#include <memory>
#include <string>
#include <vector>

namespace brovideo::vaapi {

// Fixed for the life of one encoder.
struct StreamParams {
    uint32_t width = 0;         // the visible picture
    uint32_t height = 0;
    uint32_t coded_width = 0;   // the surfaces: the visible size aligned up to the codec's block
    uint32_t coded_height = 0;
    uint32_t fps = 60;
    uint32_t bitrate_bps = 0;
    CodecCaps caps;
};

// One picture.
struct PictureParams {
    bool keyframe = false;
    uint64_t index_in_gop = 0;  // 0 for a keyframe, then 1, 2, ... for its predicted frames
    uint64_t keyframe_count = 0;  // keyframes before this one in the stream (0 for the first)
    VASurfaceID recon = VA_INVALID_SURFACE;  // reconstructed picture (the next one's reference)
    VASurfaceID ref = VA_INVALID_SURFACE;    // the previous reconstructed picture; invalid on a keyframe
    VABufferID coded = VA_INVALID_ID;
};

class CodecImpl {
public:
    virtual ~CodecImpl() = default;
    // The VAConfigAttribEncPackedHeaders value to create the config with
    // (VA_ENC_PACKED_HEADER_NONE: the driver writes every header).
    [[nodiscard]] virtual uint32_t packed_headers() const = 0;
    // The sequence-level buffers of a keyframe (sequence parameters). The
    // common encoder adds the rate-control misc buffers right after them.
    virtual bool add_sequence(BufferList& bufs, const PictureParams& pic, std::string* err) = 0;
    // The picture's own buffers: picture parameters, packed headers, slice
    // or tile-group parameters.
    virtual bool add_picture(BufferList& bufs, const PictureParams& pic, std::string* err) = 0;
    // Completes the coded data into the packet the stream carries (AV1: the
    // temporal delimiter the driver does not write).
    virtual void finish_packet(std::vector<uint8_t>& /*data*/) {}
};

// The codecs this backend implements (and so may report), in the factory's order.
const std::vector<Codec>& supported_codecs();
// VA profiles to try for `codec`, best first.
std::vector<VAProfile> codec_profiles(Codec codec);
// The coded-size alignment `codec` needs (the encoder pads to it with the VPP pass).
uint32_t codec_alignment(Codec codec);

std::unique_ptr<CodecImpl> make_h264(const StreamParams&);
std::unique_ptr<CodecImpl> make_hevc(const StreamParams&);
std::unique_ptr<CodecImpl> make_av1(const StreamParams&);

}  // namespace brovideo::vaapi
