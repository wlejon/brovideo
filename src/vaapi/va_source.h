#pragma once
// Where a frame's pixels come from: a dmabuf imported as a VA surface (no
// copy), or a CPU frame uploaded into a surface the encoder owns. Either way
// the result is an RGB surface the VPP pass converts.

#include "brovideo/frame.h"

#include <va/va.h>

#include <string>

namespace brovideo::vaapi {

// Waits for the frame's acquire fence (a sync_file, through brodmabuf);
// true at once for -1. The fd stays the caller's. False (with *err) when it
// has not signalled within `timeout_ms`.
bool wait_fence(int fd, int timeout_ms, std::string* err);

// A dmabuf imported through DRM PRIME 2 (with its modifier). The surface
// refers to the caller's memory; it is destroyed with this object, before
// the frame is released.
class ImportedSurface {
public:
    ImportedSurface() = default;
    ~ImportedSurface() { reset(); }
    ImportedSurface(const ImportedSurface&) = delete;
    ImportedSurface& operator=(const ImportedSurface&) = delete;

    bool import(VADisplay dpy, const Frame& frame, std::string* err);
    void reset();
    [[nodiscard]] VASurfaceID id() const { return id_; }

private:
    VADisplay dpy_ = nullptr;
    VASurfaceID id_ = VA_INVALID_SURFACE;
};

// An RGBA / BGRA surface CPU frames are copied into, kept for the encoder's
// life (remade when the size or the byte order changes).
class UploadSurface {
public:
    UploadSurface() = default;
    ~UploadSurface() { reset(); }
    UploadSurface(const UploadSurface&) = delete;
    UploadSurface& operator=(const UploadSurface&) = delete;

    // Copies the frame's rows into the surface. When this returns, the
    // frame's memory is no longer needed.
    bool upload(VADisplay dpy, const Frame& frame, std::string* err);
    void reset();
    [[nodiscard]] VASurfaceID id() const { return id_; }

private:
    bool create(VADisplay dpy, uint32_t width, uint32_t height, uint32_t fourcc, std::string* err);

    VADisplay dpy_ = nullptr;
    VASurfaceID id_ = VA_INVALID_SURFACE;
    VAImage image_{};            // staging image when the surface cannot be mapped directly
    bool derive_ = false;        // map the surface itself (vaDeriveImage) instead
    uint32_t width_ = 0;
    uint32_t height_ = 0;
    uint32_t fourcc_ = 0;        // VA_FOURCC_RGBA or VA_FOURCC_BGRA
};

// The VA fourcc of a supported dmabuf DRM format, 0 if unsupported.
uint32_t va_fourcc_for_drm(uint32_t drm_format);

}  // namespace brovideo::vaapi
