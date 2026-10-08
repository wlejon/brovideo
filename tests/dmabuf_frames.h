#pragma once
// dmabuf frames for the VA-API tests, allocated through brodmabuf's GBM as a
// compositor allocates its scanout buffers: XRGB8888 / ARGB8888, linear or
// with GBM's pick of the primary plane's modifiers (tiled, possibly
// compressed). Pixels are written through a GBM mapping, which detiles; the
// acquire fence is a real sync_file exported from the buffer.

#include "brovideo/frame.h"

#include <brodmabuf/gbm.h>
#include <brodmabuf/sync.h>
#include <drm_fourcc.h>
#include <fcntl.h>
#include <unistd.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#include <algorithm>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace testkit {

// The modifiers the display's first primary plane accepts for `format`, as
// a compositor would allocate its scanout buffers from (IN_FORMATS). Empty
// when no card node can be read.
inline std::vector<uint64_t> scanout_modifiers(uint32_t format) {
    std::vector<uint64_t> mods;
    for (int i = 0; i < 16 && mods.empty(); ++i) {
        const std::string path = "/dev/dri/card" + std::to_string(i);
        const int fd = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
        if (fd < 0) continue;
        drmSetClientCap(fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1);
        if (drmModePlaneResPtr res = drmModeGetPlaneResources(fd)) {
            for (uint32_t p = 0; p < res->count_planes && mods.empty(); ++p) {
                drmModeObjectPropertiesPtr props = drmModeObjectGetProperties(fd, res->planes[p], DRM_MODE_OBJECT_PLANE);
                if (!props) continue;
                bool primary = false;
                uint32_t blob_id = 0;
                for (uint32_t k = 0; k < props->count_props; ++k) {
                    drmModePropertyPtr prop = drmModeGetProperty(fd, props->props[k]);
                    if (!prop) continue;
                    if (!std::strcmp(prop->name, "type")) primary = props->prop_values[k] == DRM_PLANE_TYPE_PRIMARY;
                    if (!std::strcmp(prop->name, "IN_FORMATS")) blob_id = uint32_t(props->prop_values[k]);
                    drmModeFreeProperty(prop);
                }
                drmModeFreeObjectProperties(props);
                if (!primary || !blob_id) continue;
                if (drmModePropertyBlobPtr blob = drmModeGetPropertyBlob(fd, blob_id)) {
                    drmModeFormatModifierIterator it{};
                    while (drmModeFormatModifierBlobIterNext(blob, &it)) {
                        if (it.fmt == format) mods.push_back(it.mod);
                    }
                    drmModeFreePropertyBlob(blob);
                }
            }
            drmModeFreePlaneResources(res);
        }
        ::close(fd);
    }
    return mods;
}

// One GBM buffer and the Frame describing it (the plane fds are owned here).
class DmabufFrame {
public:
    // A rendered scanout buffer with GBM's pick of `modifiers` (as a
    // compositor allocates); an empty list means DRM_FORMAT_MOD_LINEAR.
    bool alloc(brodmabuf::GbmDevice& dev, uint32_t w, uint32_t h, uint32_t format,
               const std::vector<uint64_t>& modifiers, std::string* err) {
        const std::vector<uint64_t> mods = modifiers.empty() ? std::vector<uint64_t>{DRM_FORMAT_MOD_LINEAR} : modifiers;
        auto bo = dev.create_buffer_with_modifiers(w, h, format, mods, GBM_BO_USE_RENDERING | GBM_BO_USE_SCANOUT);
        if (!bo.ok()) bo = dev.create_buffer_with_modifiers(w, h, format, mods, GBM_BO_USE_RENDERING);
        if (!bo.ok()) {
            *err = "GBM allocation: " + std::string(bo.error_message());
            return false;
        }
        bo_ = std::move(bo.value());
        auto attrs = bo_->export_dmabuf();
        if (!attrs.ok()) {
            *err = "GBM export: " + std::string(attrs.error_message());
            return false;
        }
        attrs_ = std::move(attrs.value());
        frame.width = w;
        frame.height = h;
        frame.drm_format = format;
        frame.modifier = attrs_.modifier;
        frame.plane_count = uint32_t(std::min<size_t>(attrs_.plane_count(), 4));
        for (uint32_t i = 0; i < frame.plane_count; ++i) {
            frame.planes[i].fd = attrs_.planes[i].fd.get();
            frame.planes[i].offset = attrs_.planes[i].offset;
            frame.planes[i].pitch = attrs_.planes[i].stride;
        }
        return true;
    }

    // Writes an RGBA picture (R G B A bytes) as the buffer's format.
    bool fill(const uint8_t* rgba) {
        auto map = bo_->map(0, 0, frame.width, frame.height, GBM_BO_TRANSFER_WRITE);
        if (!map || !map->valid()) return false;
        const bool bgr = frame.drm_format == DRM_FORMAT_XRGB8888 || frame.drm_format == DRM_FORMAT_ARGB8888;
        for (uint32_t y = 0; y < frame.height; ++y) {
            uint8_t* dst = static_cast<uint8_t*>(map->data()) + size_t(y) * map->stride();
            const uint8_t* src = rgba + size_t(y) * frame.width * 4;
            for (uint32_t x = 0; x < frame.width; ++x) {
                dst[x * 4 + 0] = bgr ? src[x * 4 + 2] : src[x * 4 + 0];
                dst[x * 4 + 1] = src[x * 4 + 1];
                dst[x * 4 + 2] = bgr ? src[x * 4 + 0] : src[x * 4 + 2];
                dst[x * 4 + 3] = 255;
            }
        }
        return true;
    }

    // A sync_file that signals when every write to the buffer is done; an
    // invalid fd when the kernel cannot export one.
    brodmabuf::UniqueFd export_fence() const {
        auto fence = brodmabuf::export_dmabuf_sync_file(frame.planes[0].fd, false);
        return fence.ok() ? std::move(fence.value()) : brodmabuf::UniqueFd();
    }

    brovideo::Frame frame;

private:
    std::unique_ptr<brodmabuf::GbmBuffer> bo_;
    brodmabuf::DmaBufAttributes attrs_;
};

}  // namespace testkit
