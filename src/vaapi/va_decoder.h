#pragma once
// The VA-API hardware decoder: the common surface/context manager
// (va_decoder.cpp) and the per-codec bitstream parsers and VA buffer builders
// (va_dec_h264.cpp, va_dec_hevc.cpp, va_dec_av1.cpp).

#include "brovideo/decoder.h"
#include "vaapi/va_device.h"
#include "vaapi/va_util.h"

#include <functional>
#include <memory>
#include <span>
#include <string>

namespace brovideo::vaapi {

// Geometry and surface produced by one decoded access unit / temporal unit.
struct DecodedUnit {
    bool has_output = false;  // false when the unit had no displayable frame
    VASurfaceID surface = VA_INVALID_SURFACE;
    int surface_idx = -1;
    uint32_t coded_width = 0;
    uint32_t coded_height = 0;
    uint32_t vis_x = 0;
    uint32_t vis_y = 0;
    uint32_t vis_w = 0;
    uint32_t vis_h = 0;
};

// Callback interface from a codec parser back to VaDecoder to allocate a target
// surface (recreating the VA context if coded dimensions changed) and submit
// one decoded picture to VA-API.
class DecodeSession {
public:
    virtual ~DecodeSession() = default;
    // Ensures the VA context and surface pool are open at coded_w x coded_h and
    // returns a free surface index (or -1 on error).
    virtual int acquire_surface(uint32_t coded_w, uint32_t coded_h, VASurfaceID& out_id, std::string* err) = 0;
    virtual void ref_surface(int idx) = 0;
    virtual void unref_surface(int idx) = 0;
    [[nodiscard]] virtual VASurfaceID surface_id(int idx) const = 0;
    // Runs vaBeginPicture(target), populate(BufferList), vaRenderPicture, vaEndPicture.
    virtual bool submit_picture(VASurfaceID target, const std::function<bool(BufferList&, std::string*)>& populate,
                                std::string* err) = 0;
};

class DecodeCodec {
public:
    virtual ~DecodeCodec() = default;
    virtual void reset() = 0;
    virtual bool decode_packet(std::span<const uint8_t> packet, DecodeSession& session, DecodedUnit& out,
                               std::string* err) = 0;
};

std::unique_ptr<DecodeCodec> make_h264_decoder();
std::unique_ptr<DecodeCodec> make_hevc_decoder();
std::unique_ptr<DecodeCodec> make_av1_decoder();

std::unique_ptr<Decoder> create_va_decoder(const DecoderConfig& config, const Capability& cap, std::string* err);

}  // namespace brovideo::vaapi
