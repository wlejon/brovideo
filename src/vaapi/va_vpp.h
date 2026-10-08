#pragma once
// The VideoProc pass: an RGB source surface (imported dmabuf or CPU upload)
// into the encoder's NV12 input surface, BT.709 limited range, scaled to the
// visible size at the top left of the coded surface. The colour conversion
// happens here, on the GPU; there is none on the CPU.

#include <va/va.h>

#include <string>

namespace brovideo::vaapi {

class Converter {
public:
    Converter() = default;
    ~Converter() { reset(); }
    Converter(const Converter&) = delete;
    Converter& operator=(const Converter&) = delete;

    bool init(VADisplay dpy, uint32_t width, uint32_t height, std::string* err);
    void reset();
    // Converts `src` (src_w x src_h) into `dst` (out_w x out_h at the origin)
    // and waits until the copy is complete, so the source may be released.
    bool convert(VASurfaceID src, uint32_t src_w, uint32_t src_h, VASurfaceID dst, uint32_t out_w, uint32_t out_h,
                 std::string* err);

private:
    VADisplay dpy_ = nullptr;
    VAConfigID config_ = VA_INVALID_ID;
    VAContextID context_ = VA_INVALID_ID;
};

}  // namespace brovideo::vaapi
