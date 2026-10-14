// The VA-API hardware decoder: surface pool, VA context management, CPU NV12
// readout (vaDeriveImage / vaGetImage), and DRM PRIME 2 DMA-BUF export
// (vaExportSurfaceHandle).
#include "vaapi/va_decoder.h"

#include "brovideo/bitstream.h"
#include "vaapi/va_codec.h"

#include <drm_fourcc.h>
#include <va/va_drmcommon.h>

#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <vector>

namespace brovideo::vaapi {
namespace {

constexpr int kNumDecodeSurfaces = 20;

uint32_t align_up(uint32_t v, uint32_t a) { return (v + a - 1) / a * a; }

struct SurfacePool {
    std::shared_ptr<Display> display;
    std::vector<VASurfaceID> ids;
    std::vector<int> refs;
    uint32_t width = 0;
    uint32_t height = 0;

    ~SurfacePool() {
        if (display && !ids.empty()) {
            vaDestroySurfaces(display->get(), ids.data(), int(ids.size()));
        }
    }
};

struct DmabufHold {
    std::shared_ptr<SurfacePool> pool;
    int surface_idx = -1;
    std::vector<int> fds;

    DmabufHold(std::shared_ptr<SurfacePool> p, int idx, const VADRMPRIMESurfaceDescriptor& desc)
        : pool(std::move(p)), surface_idx(idx) {
        if (pool && surface_idx >= 0 && size_t(surface_idx) < pool->refs.size()) {
            ++pool->refs[size_t(surface_idx)];
        }
        for (uint32_t i = 0; i < desc.num_objects; ++i) {
            if (desc.objects[i].fd >= 0) fds.push_back(desc.objects[i].fd);
        }
    }

    ~DmabufHold() {
        for (int fd : fds) {
            if (fd >= 0) ::close(fd);
        }
        if (pool && surface_idx >= 0 && size_t(surface_idx) < pool->refs.size()) {
            --pool->refs[size_t(surface_idx)];
        }
    }
};

std::unique_ptr<DecodeCodec> make_codec_parser(Codec codec) {
    switch (codec) {
        case Codec::H264: return make_h264_decoder();
        case Codec::HEVC: return make_hevc_decoder();
        case Codec::AV1: return make_av1_decoder();
        default: return nullptr;
    }
}

class VaDecoder final : public Decoder, private DecodeSession {
public:
    VaDecoder(Codec codec, PictureMemory output, const NodeCaps& node, const CodecCaps& caps,
              std::shared_ptr<Display> display, std::unique_ptr<DecodeCodec> parser)
        : Decoder(codec),
          output_(output),
          node_path_(node.path),
          driver_(node.driver),
          caps_(caps),
          display_(std::move(display)),
          parser_(std::move(parser)) {
        image_.image_id = VA_INVALID_ID;
    }

    ~VaDecoder() override {
        if (parser_) parser_->reset();
        close_context();
    }

    std::string describe() const override {
        std::string s = std::string("VA-API ") + codec_name(codec());
        if (display_) s += ", " + display_->path() + " (" + driver_ + "; " + display_->vendor() + ")";
        return s;
    }

    bool hardware() const override { return true; }

    bool decode(std::span<const uint8_t> packet, int64_t pts_ns, Picture& out, std::string* err) override {
        out.ready = false;
        if (packet.empty()) {
            if (err) *err = "empty packet";
            return false;
        }
        const bool key = is_keyframe(codec(), packet);
        if (!synced_ && !key) {
            if (err) *err = "lost sync: a predicted frame with no keyframe before it";
            return false;
        }

        DecodedUnit unit{};
        if (!parser_->decode_packet(packet, *this, unit, err)) {
            reset_after_error();
            out.ready = false;
            out.gpu = GpuPicture{};
            return false;
        }
        if (key) synced_ = true;
        if (!unit.has_output) return true;

        bool ok = false;
        if (output_ == PictureMemory::Dmabuf) {
            ok = export_dmabuf(unit, out, err);
        } else {
            ok = read_cpu(unit, out, err);
        }
        if (!ok) {
            reset_after_error();
            out.ready = false;
            out.gpu = GpuPicture{};
            return false;
        }
        out.pts_ns = pts_ns;
        out.ready = true;
        return true;
    }

private:
    void reset_after_error() {
        synced_ = false;
        if (parser_) parser_->reset();
    }

    void destroy_image() {
        if (image_.image_id != VA_INVALID_ID && display_) {
            vaDestroyImage(display_->get(), image_.image_id);
        }
        image_ = VAImage{};
        image_.image_id = VA_INVALID_ID;
        image_w_ = 0;
        image_h_ = 0;
    }

    void close_context() {
        destroy_image();
        if (display_) {
            VADisplay dpy = display_->get();
            if (context_ != VA_INVALID_ID) vaDestroyContext(dpy, context_);
            if (config_id_ != VA_INVALID_ID) vaDestroyConfig(dpy, config_id_);
        }
        context_ = VA_INVALID_ID;
        config_id_ = VA_INVALID_ID;
        pool_.reset();
    }

    bool ensure_pool(uint32_t coded_w, uint32_t coded_h, std::string* err) {
        const uint32_t align = std::max(codec_alignment(codec()), std::max(caps_.align_width, caps_.align_height));
        const uint32_t alloc_w = align_up(coded_w, std::max(16u, caps_.align_width));
        const uint32_t alloc_h = align_up(coded_h, std::max(16u, caps_.align_height));
        (void)align;
        if ((caps_.max_width && alloc_w > caps_.max_width) || (caps_.max_height && alloc_h > caps_.max_height)) {
            if (err) {
                *err = std::to_string(coded_w) + "x" + std::to_string(coded_h) + " exceeds the " +
                       codec_name(codec()) + " decoder's maximum " + std::to_string(caps_.max_width) + "x" +
                       std::to_string(caps_.max_height);
            }
            return false;
        }
        if (pool_ && pool_->width == alloc_w && pool_->height == alloc_h && context_ != VA_INVALID_ID) {
            return true;
        }

        close_context();
        VADisplay dpy = display_->get();
        VAConfigAttrib attr{VAConfigAttribRTFormat, VA_RT_FORMAT_YUV420};
        if (!va_check(vaCreateConfig(dpy, caps_.profile, VAEntrypointVLD, &attr, 1, &config_id_),
                      "vaCreateConfig (decode)", err)) {
            return false;
        }

        auto pool = std::make_shared<SurfacePool>();
        pool->display = display_;
        pool->width = alloc_w;
        pool->height = alloc_h;
        pool->ids.resize(kNumDecodeSurfaces, VA_INVALID_SURFACE);
        pool->refs.assign(kNumDecodeSurfaces, 0);

        VASurfaceAttrib fmt{};
        fmt.type = VASurfaceAttribPixelFormat;
        fmt.flags = VA_SURFACE_ATTRIB_SETTABLE;
        fmt.value.type = VAGenericValueTypeInteger;
        fmt.value.value.i = VA_FOURCC_NV12;
        if (!va_check(vaCreateSurfaces(dpy, VA_RT_FORMAT_YUV420, alloc_w, alloc_h, pool->ids.data(),
                                       kNumDecodeSurfaces, &fmt, 1),
                      "vaCreateSurfaces (decode NV12)", err)) {
            pool->ids.clear();
            close_context();
            return false;
        }
        if (!va_check(vaCreateContext(dpy, config_id_, int(alloc_w), int(alloc_h), VA_PROGRESSIVE, pool->ids.data(),
                                      kNumDecodeSurfaces, &context_),
                      "vaCreateContext (decode)", err)) {
            close_context();
            return false;
        }
        pool_ = std::move(pool);
        debug_log("%s decoder on %s (%s): %ux%u (surfaces %ux%u)", codec_name(codec()), node_path_.c_str(),
                  display_->vendor().c_str(), coded_w, coded_h, alloc_w, alloc_h);
        return true;
    }

    int acquire_surface(uint32_t coded_w, uint32_t coded_h, VASurfaceID& out_id, std::string* err) override {
        if (!ensure_pool(coded_w, coded_h, err)) return -1;
        for (size_t i = 0; i < pool_->ids.size(); ++i) {
            const size_t idx = (next_surface_ + i) % pool_->ids.size();
            if (pool_->refs[idx] == 0) {
                next_surface_ = (idx + 1) % pool_->ids.size();
                out_id = pool_->ids[idx];
                return int(idx);
            }
        }
        if (err) *err = "all VA decode surfaces are in use";
        return -1;
    }

    void ref_surface(int idx) override {
        if (pool_ && idx >= 0 && size_t(idx) < pool_->refs.size()) ++pool_->refs[size_t(idx)];
    }

    void unref_surface(int idx) override {
        if (pool_ && idx >= 0 && size_t(idx) < pool_->refs.size() && pool_->refs[size_t(idx)] > 0) {
            --pool_->refs[size_t(idx)];
        }
    }

    VASurfaceID surface_id(int idx) const override {
        if (pool_ && idx >= 0 && size_t(idx) < pool_->ids.size()) return pool_->ids[size_t(idx)];
        return VA_INVALID_SURFACE;
    }

    bool submit_picture(VASurfaceID target, const std::function<bool(BufferList&, std::string*)>& populate,
                        std::string* err) override {
        VADisplay dpy = display_->get();
        if (!va_check(vaBeginPicture(dpy, context_, target), "vaBeginPicture (decode)", err)) return false;
        BufferList bufs(dpy, context_);
        bool ok = populate(bufs, err) && bufs.render(err);
        const VAStatus end = vaEndPicture(dpy, context_);
        if (ok) ok = va_check(end, "vaEndPicture (decode)", err);
        bufs.clear();
        return ok;
    }

    bool read_cpu(const DecodedUnit& unit, Picture& out, std::string* err) {
        VADisplay dpy = display_->get();
        if (!va_check(vaSyncSurface(dpy, unit.surface), "vaSyncSurface (decode)", err)) return false;

        VAImage img{};
        img.image_id = VA_INVALID_ID;
        bool derived = false;
        if (vaDeriveImage(dpy, unit.surface, &img) == VA_STATUS_SUCCESS) {
            if (img.format.fourcc == VA_FOURCC_NV12) {
                derived = true;
            } else {
                vaDestroyImage(dpy, img.image_id);
                img.image_id = VA_INVALID_ID;
            }
        }
        if (!derived) {
            if (image_.image_id == VA_INVALID_ID || image_w_ != pool_->width || image_h_ != pool_->height) {
                destroy_image();
                VAImageFormat fmt{};
                fmt.fourcc = VA_FOURCC_NV12;
                fmt.byte_order = VA_LSB_FIRST;
                fmt.bits_per_pixel = 12;
                if (!va_check(vaCreateImage(dpy, &fmt, int(pool_->width), int(pool_->height), &image_),
                              "vaCreateImage (decode NV12)", err)) {
                    return false;
                }
                image_w_ = pool_->width;
                image_h_ = pool_->height;
            }
            if (!va_check(vaGetImage(dpy, unit.surface, 0, 0, pool_->width, pool_->height, image_.image_id),
                          "vaGetImage (decode NV12)", err)) {
                return false;
            }
            img = image_;
        }

        void* map = nullptr;
        if (!va_check(vaMapBuffer(dpy, img.buf, &map), "vaMapBuffer (decode NV12)", err)) {
            if (derived) vaDestroyImage(dpy, img.image_id);
            return false;
        }

        const uint32_t w = unit.vis_w;
        const uint32_t h = unit.vis_h;
        const uint32_t stride = (w + 1u) & ~1u;
        const uint32_t chroma_rows = (h + 1u) / 2u;
        out.width = w;
        out.height = h;
        out.format = PixelFormat::NV12;
        out.memory = PictureMemory::Cpu;
        out.gpu = GpuPicture{};
        out.stride = stride;
        out.uv_offset = stride * h;
        out.data.resize(size_t(stride) * (h + chroma_rows));

        const auto* base = static_cast<const uint8_t*>(map);
        const uint8_t* y_src = base + img.offsets[0];
        const uint8_t* uv_src = base + img.offsets[1];
        const size_t y_pitch = img.pitches[0];
        const size_t uv_pitch = img.pitches[1];
        const size_t x0 = unit.vis_x & ~1u;
        const size_t y0 = unit.vis_y & ~1u;
        const size_t copy_y = std::min<size_t>(w, y_pitch > x0 ? y_pitch - x0 : 0);
        const size_t copy_uv = std::min<size_t>(stride, uv_pitch > x0 ? uv_pitch - x0 : 0);

        for (uint32_t r = 0; r < h; ++r) {
            std::memcpy(out.data.data() + size_t(r) * stride, y_src + (y0 + r) * y_pitch + x0, copy_y);
        }
        uint8_t* uv_dst = out.data.data() + out.uv_offset;
        for (uint32_t r = 0; r < chroma_rows; ++r) {
            std::memcpy(uv_dst + size_t(r) * stride, uv_src + (y0 / 2 + r) * uv_pitch + x0, copy_uv);
        }

        vaUnmapBuffer(dpy, img.buf);
        if (derived) vaDestroyImage(dpy, img.image_id);
        return true;
    }

    bool export_dmabuf(const DecodedUnit& unit, Picture& out, std::string* err) {
        VADisplay dpy = display_->get();
        if (!va_check(vaSyncSurface(dpy, unit.surface), "vaSyncSurface (decode dmabuf)", err)) return false;

        VADRMPRIMESurfaceDescriptor desc{};
        VAStatus s = vaExportSurfaceHandle(dpy, unit.surface, VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2,
                                           VA_EXPORT_SURFACE_READ_ONLY | VA_EXPORT_SURFACE_COMPOSED_LAYERS, &desc);
        if (s != VA_STATUS_SUCCESS) {
            s = vaExportSurfaceHandle(dpy, unit.surface, VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2,
                                      VA_EXPORT_SURFACE_READ_ONLY | VA_EXPORT_SURFACE_SEPARATE_LAYERS, &desc);
        }
        if (!va_check(s, "vaExportSurfaceHandle (decode dmabuf)", err)) return false;

        auto hold = std::make_shared<DmabufHold>(pool_, unit.surface_idx, desc);
        out.width = unit.vis_w;
        out.height = unit.vis_h;
        out.format = PixelFormat::NV12;
        out.memory = PictureMemory::Dmabuf;
        out.stride = 0;
        out.uv_offset = 0;
        out.data.clear();
        out.gpu = GpuPicture{};
        out.gpu.x = unit.vis_x;
        out.gpu.y = unit.vis_y;
        out.gpu.drm_format = DRM_FORMAT_NV12;
        out.gpu.modifier = desc.num_objects > 0 ? desc.objects[0].drm_format_modifier : DRM_FORMAT_MOD_LINEAR;
        out.gpu.coded_width = desc.width;
        out.gpu.coded_height = desc.height;
        uint32_t plane_idx = 0;
        for (uint32_t l = 0; l < desc.num_layers; ++l) {
            for (uint32_t p = 0; p < desc.layers[l].num_planes && plane_idx < 4; ++p) {
                const uint32_t obj = desc.layers[l].object_index[p];
                out.gpu.planes[plane_idx].fd = (obj < desc.num_objects) ? desc.objects[obj].fd : -1;
                out.gpu.planes[plane_idx].offset = desc.layers[l].offset[p];
                out.gpu.planes[plane_idx].pitch = desc.layers[l].pitch[p];
                ++plane_idx;
            }
        }
        out.gpu.plane_count = plane_idx;
        out.gpu.keepalive = std::move(hold);
        return true;
    }

    PictureMemory output_ = PictureMemory::Cpu;
    std::string node_path_;
    std::string driver_;
    CodecCaps caps_;
    std::shared_ptr<Display> display_;
    std::unique_ptr<DecodeCodec> parser_;
    std::shared_ptr<SurfacePool> pool_;
    VAConfigID config_id_ = VA_INVALID_ID;
    VAContextID context_ = VA_INVALID_ID;
    VAImage image_{};
    uint32_t image_w_ = 0;
    uint32_t image_h_ = 0;
    size_t next_surface_ = 0;
    bool synced_ = false;
};

}  // namespace

std::unique_ptr<Decoder> create_va_decoder(const DecoderConfig& config, const Capability&, std::string* err) {
    if (config.output != PictureMemory::Cpu && config.output != PictureMemory::Dmabuf) {
        if (err) *err = "VA-API decoder only supports Cpu and Dmabuf picture output";
        return nullptr;
    }
    const auto& nodes = probe_nodes();
    const NodeCaps* chosen = nullptr;
    const CodecCaps* caps = nullptr;
    for (const NodeCaps& n : nodes) {
        if (const CodecCaps* c = n.find_decoder(config.codec)) {
            chosen = &n;
            caps = c;
            break;
        }
    }
    if (!chosen || !caps) {
        if (err) *err = std::string("no VA device can decode ") + codec_name(config.codec);
        return nullptr;
    }
    auto parser = make_codec_parser(config.codec);
    if (!parser) {
        if (err) *err = std::string(codec_name(config.codec)) + " is not implemented by the VA-API decoder";
        return nullptr;
    }
    std::unique_ptr<Display> raw_dpy = Display::open(chosen->path, err);
    if (!raw_dpy) return nullptr;
    std::shared_ptr<Display> display(std::move(raw_dpy));
    return std::make_unique<VaDecoder>(config.codec, config.output, *chosen, *caps, std::move(display),
                                       std::move(parser));
}

}  // namespace brovideo::vaapi
