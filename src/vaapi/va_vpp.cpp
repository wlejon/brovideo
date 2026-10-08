#include "vaapi/va_vpp.h"

#include "vaapi/va_util.h"

#include <va/va_vpp.h>

namespace brovideo::vaapi {

bool Converter::init(VADisplay dpy, uint32_t width, uint32_t height, std::string* err) {
    reset();
    dpy_ = dpy;
    if (!va_check(vaCreateConfig(dpy, VAProfileNone, VAEntrypointVideoProc, nullptr, 0, &config_),
                  "vaCreateConfig (VideoProc)", err)) {
        return false;
    }
    return va_check(vaCreateContext(dpy, config_, int(width), int(height), VA_PROGRESSIVE, nullptr, 0, &context_),
                    "vaCreateContext (VideoProc)", err);
}

void Converter::reset() {
    if (context_ != VA_INVALID_ID) vaDestroyContext(dpy_, context_);
    if (config_ != VA_INVALID_ID) vaDestroyConfig(dpy_, config_);
    context_ = VA_INVALID_ID;
    config_ = VA_INVALID_ID;
}

bool Converter::convert(VASurfaceID src, uint32_t src_w, uint32_t src_h, VASurfaceID dst, uint32_t out_w,
                        uint32_t out_h, std::string* err) {
    const VARectangle in_rect{0, 0, uint16_t(src_w), uint16_t(src_h)};
    const VARectangle out_rect{0, 0, uint16_t(out_w), uint16_t(out_h)};
    VAProcPipelineParameterBuffer p{};
    p.surface = src;
    p.surface_region = &in_rect;
    // The RGB input is tagged BT.709 too (not sRGB): the pass must apply
    // the YCbCr matrix only. Tagged sRGB, the driver also converts the
    // transfer function, lifting every mid-tone (+5 code values on average).
    p.surface_color_standard = VAProcColorStandardBT709;
    p.input_color_properties.colour_primaries = 1;
    p.input_color_properties.transfer_characteristics = 1;
    p.input_color_properties.matrix_coefficients = 0;  // RGB (identity)
    p.output_region = &out_rect;
    p.output_background_color = 0xff000000;  // opaque black (ARGB) in any padding
    p.output_color_standard = VAProcColorStandardBT709;
    p.filter_flags = VA_FILTER_SCALING_DEFAULT;
    p.input_color_properties.color_range = VA_SOURCE_RANGE_FULL;
    p.output_color_properties.color_range = VA_SOURCE_RANGE_REDUCED;
    p.output_color_properties.colour_primaries = 1;
    p.output_color_properties.transfer_characteristics = 1;
    p.output_color_properties.matrix_coefficients = 1;
    // Centre-sited chroma: the driver then averages each 2x2 block instead
    // of point-sampling one pixel (1.3 dB RGB PSNR on the test pattern).
    // The encoders signal it (H.264/HEVC chroma_sample_loc_type 1).
    p.output_color_properties.chroma_sample_location = VA_CHROMA_SITING_VERTICAL_CENTER | VA_CHROMA_SITING_HORIZONTAL_CENTER;

    if (!va_check(vaBeginPicture(dpy_, context_, dst), "vaBeginPicture (VideoProc)", err)) return false;
    BufferList bufs(dpy_, context_);
    bool ok = bufs.add(VAProcPipelineParameterBufferType, p, err) && bufs.render(err);
    // EndPicture even after a failure, so the context is not left mid-picture.
    const VAStatus end = vaEndPicture(dpy_, context_);
    if (ok) ok = va_check(end, "vaEndPicture (VideoProc)", err);
    bufs.clear();
    return ok && va_check(vaSyncSurface(dpy_, dst), "vaSyncSurface (VideoProc)", err);
}

}  // namespace brovideo::vaapi
