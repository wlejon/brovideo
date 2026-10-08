#include "vaapi/va_source.h"

#include "vaapi/va_device.h"
#include "vaapi/va_util.h"

#include <brodmabuf/sync.h>
#include <drm_fourcc.h>
#include <va/va_drmcommon.h>

#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstring>

namespace brovideo::vaapi {

bool wait_fence(int fd, int timeout_ms, std::string* err) {
    if (fd < 0) return true;
    const int own = ::dup(fd);  // brodmabuf's SyncFile owns its fd; the frame's stays the caller's
    if (own < 0) {
        if (err) *err = std::string("acquire fence: dup: ") + std::strerror(errno);
        return false;
    }
    const brodmabuf::SyncFile fence{brodmabuf::UniqueFd(own)};
    // SyncFile::wait is false both on a timeout and when a signal interrupts
    // the poll, so wait out the remainder until the deadline has passed.
    const auto until = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    for (;;) {
        const auto left =
            std::chrono::duration_cast<std::chrono::milliseconds>(until - std::chrono::steady_clock::now()).count();
        if (fence.wait(int(left > 0 ? left : 0))) return true;
        if (left <= 0) break;
    }
    if (err) *err = "acquire fence did not signal within " + std::to_string(timeout_ms) + " ms";
    return false;
}

uint32_t va_fourcc_for_drm(uint32_t drm_format) {
    switch (drm_format) {
        case DRM_FORMAT_XRGB8888: return VA_FOURCC_BGRX;  // memory: B G R X
        case DRM_FORMAT_ARGB8888: return VA_FOURCC_BGRA;
        case DRM_FORMAT_XBGR8888: return VA_FOURCC_RGBX;  // memory: R G B X
        case DRM_FORMAT_ABGR8888: return VA_FOURCC_RGBA;
        default: return 0;
    }
}

namespace {

// Two fds on one dmabuf (a caller may hand a dup per plane) are one object.
bool same_buffer(int a, int b) {
    if (a == b) return true;
    struct stat sa {}, sb {};
    return fstat(a, &sa) == 0 && fstat(b, &sb) == 0 && sa.st_dev == sb.st_dev && sa.st_ino == sb.st_ino;
}

}  // namespace

bool ImportedSurface::import(VADisplay dpy, const Frame& frame, std::string* err) {
    reset();
    const uint32_t fourcc = va_fourcc_for_drm(frame.drm_format);
    if (!fourcc) {
        if (err) {
            char name[5] = {char(frame.drm_format), char(frame.drm_format >> 8), char(frame.drm_format >> 16),
                            char(frame.drm_format >> 24), 0};
            *err = std::string("unsupported dmabuf format ") + name;
        }
        return false;
    }
    if (frame.plane_count > 4) {
        if (err) *err = "dmabuf has more than 4 planes";
        return false;
    }
    VADRMPRIMESurfaceDescriptor desc{};
    desc.fourcc = fourcc;
    desc.width = frame.width;
    desc.height = frame.height;
    desc.num_layers = 1;
    desc.layers[0].drm_format = frame.drm_format;
    desc.layers[0].num_planes = frame.plane_count;
    // One object per distinct fd: a modifier with metadata planes (AMD DCC)
    // usually puts them all in the one buffer.
    for (uint32_t i = 0; i < frame.plane_count; ++i) {
        const DmabufPlane& pl = frame.planes[i];
        if (pl.fd < 0) {
            if (err) *err = "dmabuf plane " + std::to_string(i) + " has no fd";
            return false;
        }
        uint32_t obj = 0;
        while (obj < desc.num_objects && !same_buffer(desc.objects[obj].fd, pl.fd)) ++obj;
        if (obj == desc.num_objects) {
            const off_t size = ::lseek(pl.fd, 0, SEEK_END);
            ::lseek(pl.fd, 0, SEEK_SET);
            desc.objects[obj].fd = pl.fd;
            desc.objects[obj].size = size > 0 ? uint32_t(size) : 0;
            desc.objects[obj].drm_format_modifier = frame.modifier;
            ++desc.num_objects;
        }
        desc.layers[0].object_index[i] = obj;
        desc.layers[0].offset[i] = pl.offset;
        desc.layers[0].pitch[i] = pl.pitch;
    }
    VASurfaceAttrib attrs[2]{};
    attrs[0].type = VASurfaceAttribMemoryType;
    attrs[0].flags = VA_SURFACE_ATTRIB_SETTABLE;
    attrs[0].value.type = VAGenericValueTypeInteger;
    attrs[0].value.value.i = VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2;
    attrs[1].type = VASurfaceAttribExternalBufferDescriptor;
    attrs[1].flags = VA_SURFACE_ATTRIB_SETTABLE;
    attrs[1].value.type = VAGenericValueTypePointer;
    attrs[1].value.value.p = &desc;
    VASurfaceID id = VA_INVALID_SURFACE;
    const VAStatus s = vaCreateSurfaces(dpy, VA_RT_FORMAT_RGB32, frame.width, frame.height, &id, 1, attrs, 2);
    if (!va_check(s, "dmabuf import (vaCreateSurfaces, DRM PRIME 2)", err)) return false;
    dpy_ = dpy;
    id_ = id;
    return true;
}

void ImportedSurface::reset() {
    if (id_ != VA_INVALID_SURFACE) vaDestroySurfaces(dpy_, &id_, 1);
    id_ = VA_INVALID_SURFACE;
}

bool UploadSurface::create(VADisplay dpy, uint32_t width, uint32_t height, uint32_t fourcc, std::string* err) {
    reset();
    VASurfaceAttrib attr{};
    attr.type = VASurfaceAttribPixelFormat;
    attr.flags = VA_SURFACE_ATTRIB_SETTABLE;
    attr.value.type = VAGenericValueTypeInteger;
    attr.value.value.i = int(fourcc);  // the CPU frame's own byte order: no swizzle on the CPU
    VASurfaceID id = VA_INVALID_SURFACE;
    if (!va_check(vaCreateSurfaces(dpy, VA_RT_FORMAT_RGB32, width, height, &id, 1, &attr, 1),
                  "vaCreateSurfaces (RGB upload)", err)) {
        return false;
    }
    dpy_ = dpy;
    id_ = id;
    width_ = width;
    height_ = height;
    fourcc_ = fourcc;
    // Mapping the surface itself saves a copy; where the driver cannot,
    // a staging image is copied in with vaPutImage.
    VAImage img{};
    if (vaDeriveImage(dpy, id, &img) == VA_STATUS_SUCCESS) {
        derive_ = img.format.fourcc == fourcc;
        vaDestroyImage(dpy, img.image_id);
    }
    if (!derive_) {
        const bool bgra = fourcc == VA_FOURCC_BGRA;
        VAImageFormat fmt{};
        fmt.fourcc = fourcc;
        fmt.byte_order = VA_LSB_FIRST;
        fmt.bits_per_pixel = 32;
        fmt.depth = 32;
        fmt.red_mask = bgra ? 0x00ff0000 : 0x000000ff;
        fmt.green_mask = 0x0000ff00;
        fmt.blue_mask = bgra ? 0x000000ff : 0x00ff0000;
        fmt.alpha_mask = 0xff000000;
        if (!va_check(vaCreateImage(dpy, &fmt, int(width), int(height), &image_), "vaCreateImage (RGB)", err)) {
            return false;
        }
    }
    debug_log("cpu upload %ux%u %s via %s", width, height, fourcc == VA_FOURCC_BGRA ? "BGRA" : "RGBA",
              derive_ ? "vaDeriveImage" : "vaPutImage");
    return true;
}

bool UploadSurface::upload(VADisplay dpy, const Frame& frame, std::string* err) {
    if (!frame.cpu) {
        if (err) *err = "CPU frame has no pixels";
        return false;
    }
    uint32_t fourcc = 0;
    switch (frame.cpu_format) {
        case PixelFormat::RGBA8: fourcc = VA_FOURCC_RGBA; break;
        case PixelFormat::BGRA8: fourcc = VA_FOURCC_BGRA; break;
        default:
            if (err) *err = "CPU frames must be RGBA8 or BGRA8";
            return false;
    }
    if (id_ == VA_INVALID_SURFACE || dpy_ != dpy || width_ != frame.width || height_ != frame.height ||
        fourcc_ != fourcc) {
        if (!create(dpy, frame.width, frame.height, fourcc, err)) return false;
    }
    VAImage img = image_;
    if (derive_ && !va_check(vaDeriveImage(dpy, id_, &img), "vaDeriveImage", err)) return false;
    void* map = nullptr;
    bool ok = va_check(vaMapBuffer(dpy, img.buf, &map), "vaMapBuffer (upload)", err);
    if (ok) {
        auto* dst = static_cast<uint8_t*>(map) + img.offsets[0];
        const size_t row = size_t(frame.width) * 4;
        const size_t src_stride = frame.cpu_row_bytes();
        for (uint32_t y = 0; y < frame.height; ++y) {
            std::memcpy(dst + size_t(y) * img.pitches[0], frame.cpu + y * src_stride, row);
        }
        ok = va_check(vaUnmapBuffer(dpy, img.buf), "vaUnmapBuffer (upload)", err);
    }
    if (derive_) {
        vaDestroyImage(dpy, img.image_id);
    } else if (ok) {
        ok = va_check(vaPutImage(dpy, id_, image_.image_id, 0, 0, frame.width, frame.height, 0, 0, frame.width,
                                 frame.height),
                      "vaPutImage", err);
    }
    return ok;
}

void UploadSurface::reset() {
    if (image_.image_id != VA_INVALID_ID && image_.buf != 0 && dpy_) vaDestroyImage(dpy_, image_.image_id);
    image_ = VAImage{};
    image_.image_id = VA_INVALID_ID;
    if (id_ != VA_INVALID_SURFACE) vaDestroySurfaces(dpy_, &id_, 1);
    id_ = VA_INVALID_SURFACE;
    derive_ = false;
    fourcc_ = 0;
}

}  // namespace brovideo::vaapi
