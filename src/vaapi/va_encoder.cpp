// The VA-API encoder: the common half (see va_codec.h for the split). One
// VADisplay per encoder, opened on the first frame so a dmabuf frame can pick
// the render node of the device that exported it.
//
// Per frame: wait on the acquire fence, import the dmabuf (or upload the CPU
// frame), convert it with VideoProc into the NV12 input surface, wait for that
// copy, release the frame, then encode and read the coded buffer.
#include "backend.h"
#include "vaapi/va_codec.h"
#include "vaapi/va_decoder.h"
#include "vaapi/va_source.h"
#include "vaapi/va_vpp.h"

#include <algorithm>
#include <cstring>
#include <functional>

namespace brovideo::vaapi {

// AV1 has no cropping: radeonsi encodes the frame at its surface alignment
// (64x16; 1080 lines become 1082) and keeps the visible size only as
// render_size, so a decoder crops to the configured size. See docs/design.md.
const std::vector<Codec>& supported_codecs() {
    static const std::vector<Codec> codecs{Codec::H264, Codec::HEVC, Codec::AV1};
    return codecs;
}

std::vector<VAProfile> codec_profiles(Codec codec) {
    switch (codec) {
        case Codec::H264: return {VAProfileH264High, VAProfileH264Main, VAProfileH264ConstrainedBaseline};
        case Codec::HEVC: return {VAProfileHEVCMain};
        case Codec::AV1: return {VAProfileAV1Profile0};
        default: return {};
    }
}

uint32_t codec_alignment(Codec codec) {
    switch (codec) {
        case Codec::AV1: return 64;  // whole superblocks
        default: return 16;          // macroblocks; HEVC's minimum coding block is 8
    }
}

namespace {

constexpr int kFenceTimeoutMs = 1000;

uint32_t align_up(uint32_t v, uint32_t a) { return (v + a - 1) / a * a; }

uint32_t bitrate_bps(const EncoderConfig& c) {
    return uint32_t(std::min<uint64_t>(uint64_t(c.bitrate_kbps) * 1000, 0xffffffffu));
}

std::unique_ptr<CodecImpl> make_impl(Codec codec, const StreamParams& sp) {
    switch (codec) {
        case Codec::H264: return make_h264(sp);
        case Codec::HEVC: return make_hevc(sp);
        case Codec::AV1: return make_av1(sp);
        default: return nullptr;
    }
}

class VaEncoder final : public Encoder {
public:
    explicit VaEncoder(const EncoderConfig& cfg) : Encoder(cfg) {}
    ~VaEncoder() override { close_session(); }

    EncoderInfo info() const override {
        EncoderInfo i;
        i.backend = Backend::VaApi;
        i.hardware = true;
        i.description = std::string("VA-API ") + codec_name(config_.codec);
        if (display_) i.description += ", " + display_->path() + " (" + driver_ + "; " + display_->vendor() + ")";
        return i;
    }

protected:
    bool encode_frame(const Frame& frame, bool keyframe, const std::function<void()>& release, Packet& out,
                      std::string* err) override {
        if (frame.width == 0 || frame.height == 0) return fail(err, "frame is empty");
        if (!display_ && !open_session(frame, err)) return false;

        // The source surface, then the conversion into the encoder's input.
        VASurfaceID src = VA_INVALID_SURFACE;
        ImportedSurface imported;
        if (frame.is_cpu()) {
            if (!upload_.upload(display_->get(), frame, err)) return false;
            release();  // the pixels are in the upload surface now
            src = upload_.id();
        } else {
            if (!wait_fence(frame.acquire_fence_fd, kFenceTimeoutMs, err)) return false;
            if (!imported.import(display_->get(), frame, err)) return false;
            src = imported.id();
        }
        if (!vpp_.convert(src, frame.width, frame.height, input_, sp_.width, sp_.height, err)) return false;
        imported.reset();
        release();
        return encode_picture(keyframe, out, err);
    }

    // The input surface keeps the last converted picture until the next
    // frame's conversion overwrites it.
    bool repeats() const override { return true; }
    bool encode_repeat_frame(bool keyframe, Packet& out, std::string* err) override {
        if (!display_) return false;
        return encode_picture(keyframe, out, err);
    }

    bool apply_config(const EncoderConfig& next, std::string*) override {
        if (!display_) return true;  // nothing open yet: the first frame opens with `next`
        if (next.width != config_.width || next.height != config_.height || next.fps != config_.fps) {
            close_session();  // new surfaces and context, opened by the next frame
            return true;
        }
        if (next.bitrate_kbps != config_.bitrate_kbps) {
            // The sequence parameters (and the H.264/HEVC/AV1 level) carry
            // the bitrate, and the rate-control buffers go with every
            // keyframe: a new CodecImpl and the keyframe reconfigure() forces
            // are all a bitrate change needs.
            sp_.bitrate_bps = bitrate_bps(next);
            impl_ = make_impl(next.codec, sp_);
        }
        return true;
    }

private:
    static bool fail(std::string* err, const std::string& what) {
        if (err) *err = what;
        return false;
    }

    const NodeCaps* pick_node(const Frame& frame) const {
        const auto& nodes = probe_nodes();
        const NodeCaps* first = nullptr;
        for (const NodeCaps& n : nodes) {
            if (n.find(config_.codec) && !first) first = &n;
        }
        if (!frame.is_cpu() && frame.planes[0].fd >= 0) {
            const std::string exporter = dmabuf_exporter(frame.planes[0].fd);
            for (const NodeCaps& n : nodes) {
                if (n.find(config_.codec) && !exporter.empty() && n.driver == exporter) return &n;
            }
        }
        return first;
    }

    bool open_session(const Frame& frame, std::string* err) {
        const Codec codec = config_.codec;
        const NodeCaps* node = pick_node(frame);
        if (!node) return fail(err, std::string("no VA device can encode ") + codec_name(codec));
        const CodecCaps& caps = *node->find(codec);
        const uint32_t align = codec_alignment(codec);
        sp_.width = align_up(config_.width, 2);
        sp_.height = align_up(config_.height, 2);
        sp_.coded_width = align_up(sp_.width, std::max(align, caps.align_width));
        sp_.coded_height = align_up(sp_.height, std::max(align, caps.align_height));
        sp_.fps = config_.fps ? config_.fps : 60;
        sp_.bitrate_bps = bitrate_bps(config_);
        sp_.caps = caps;
        if ((caps.max_width && sp_.coded_width > caps.max_width) ||
            (caps.max_height && sp_.coded_height > caps.max_height)) {
            return fail(err, std::to_string(config_.width) + "x" + std::to_string(config_.height) + " exceeds the " +
                                 codec_name(codec) + " encoder's maximum " + std::to_string(caps.max_width) + "x" +
                                 std::to_string(caps.max_height));
        }
        impl_ = make_impl(codec, sp_);
        if (!impl_) return fail(err, std::string(codec_name(codec)) + " is not implemented by the VA-API backend");

        display_ = Display::open(node->path, err);
        if (!display_) return false;
        driver_ = node->driver;
        VADisplay dpy = display_->get();
        debug_log("%s encoder on %s (%s): %ux%u coded %ux%u, %u fps, %u bps", codec_name(codec), node->path.c_str(),
                  display_->vendor().c_str(), sp_.width, sp_.height, sp_.coded_width, sp_.coded_height, sp_.fps,
                  sp_.bitrate_bps);

        std::vector<VAConfigAttrib> attrs = {{VAConfigAttribRTFormat, VA_RT_FORMAT_YUV420},
                                             {VAConfigAttribRateControl, VA_RC_CBR}};
        if (caps.packed_headers) attrs.push_back({VAConfigAttribEncPackedHeaders, impl_->packed_headers()});
        if (!va_check(vaCreateConfig(dpy, caps.profile, caps.entrypoint, attrs.data(), int(attrs.size()), &config_id_),
                      "vaCreateConfig (encode)", err)) {
            return close_session(), false;
        }
        VASurfaceAttrib fmt{};
        fmt.type = VASurfaceAttribPixelFormat;
        fmt.flags = VA_SURFACE_ATTRIB_SETTABLE;
        fmt.value.type = VAGenericValueTypeInteger;
        fmt.value.value.i = VA_FOURCC_NV12;
        VASurfaceID surfaces[3];
        if (!va_check(vaCreateSurfaces(dpy, VA_RT_FORMAT_YUV420, sp_.coded_width, sp_.coded_height, surfaces, 3, &fmt, 1),
                      "vaCreateSurfaces (NV12)", err)) {
            return close_session(), false;
        }
        input_ = surfaces[0];
        recon_[0] = surfaces[1];
        recon_[1] = surfaces[2];
        if (!va_check(vaCreateContext(dpy, config_id_, int(sp_.coded_width), int(sp_.coded_height), VA_PROGRESSIVE,
                                      surfaces, 3, &context_),
                      "vaCreateContext (encode)", err)) {
            return close_session(), false;
        }
        // A coded picture is far below the raw size at any sane bitrate;
        // the raw size bounds even a worst-case keyframe.
        const uint32_t coded_size = sp_.coded_width * sp_.coded_height * 3 / 2 + (1u << 16);
        if (!va_check(vaCreateBuffer(dpy, context_, VAEncCodedBufferType, coded_size, 1, nullptr, &coded_),
                      "vaCreateBuffer (coded)", err)) {
            return close_session(), false;
        }
        if (!vpp_.init(dpy, sp_.coded_width, sp_.coded_height, err)) return close_session(), false;
        index_in_gop_ = 0;
        keyframe_count_ = 0;
        current_ = 0;
        return true;
    }

    void close_session() {
        if (!display_) return;
        VADisplay dpy = display_->get();
        vpp_.reset();
        upload_.reset();
        if (coded_ != VA_INVALID_ID) vaDestroyBuffer(dpy, coded_);
        if (context_ != VA_INVALID_ID) vaDestroyContext(dpy, context_);
        VASurfaceID surfaces[3] = {input_, recon_[0], recon_[1]};
        for (VASurfaceID s : surfaces) {
            if (s != VA_INVALID_SURFACE) vaDestroySurfaces(dpy, &s, 1);
        }
        if (config_id_ != VA_INVALID_ID) vaDestroyConfig(dpy, config_id_);
        coded_ = VA_INVALID_ID;
        context_ = VA_INVALID_ID;
        config_id_ = VA_INVALID_ID;
        input_ = recon_[0] = recon_[1] = VA_INVALID_SURFACE;
        impl_.reset();
        display_.reset();
    }

    bool add_rate_control(BufferList& bufs, std::string* err) const {
        VAEncMiscParameterRateControl rc{};
        rc.bits_per_second = sp_.bitrate_bps;
        rc.target_percentage = 100;
        rc.window_size = 1000;
        // No filler: padding a quiet desktop up to the bitrate is wasted
        // bandwidth, and on VCN 4 firmware (ENC 1.24) an AV1 frame that needs
        // more than about 35 KB of padding OBU hangs the encode ring. Mesa
        // enables filler in CBR unless the application turns it off.
        rc.rc_flags.bits.disable_bit_stuffing = 1;
        VAEncMiscParameterHRD hrd{};
        // 50 ms of buffer (three frames at 60 fps): what a frame may borrow
        // from the ones after it. A big change (a window opening, a scene
        // switch) is one packet that must cross the link before it can be
        // shown, so the buffer bounds that spike: with half a second,
        // desktop-like content at 20 Mbit/s made packets of up to 450 kB
        // that took 4-5 ms to arrive; with 50 ms they stay near 110 kB
        // (1.5 ms), mean latency unchanged, at the cost of a softer first
        // frame after the change that sharpens over the next few.
        hrd.buffer_size = uint32_t(uint64_t(sp_.bitrate_bps) * 50 / 1000);
        hrd.initial_buffer_fullness = hrd.buffer_size / 2;
        VAEncMiscParameterFrameRate fr{};
        fr.framerate = sp_.fps;
        return bufs.add_misc(VAEncMiscParameterTypeRateControl, rc, err) &&
               bufs.add_misc(VAEncMiscParameterTypeHRD, hrd, err) &&
               bufs.add_misc(VAEncMiscParameterTypeFrameRate, fr, err);
    }

    bool encode_picture(bool keyframe, Packet& out, std::string* err) {
        VADisplay dpy = display_->get();
        if (keyframe) index_in_gop_ = 0;
        PictureParams pic;
        pic.keyframe = keyframe;
        pic.index_in_gop = index_in_gop_;
        pic.keyframe_count = keyframe_count_;
        pic.recon = recon_[current_];
        pic.ref = keyframe ? VA_INVALID_SURFACE : recon_[current_ ^ 1];
        pic.coded = coded_;

        if (!va_check(vaBeginPicture(dpy, context_, input_), "vaBeginPicture (encode)", err)) return false;
        BufferList bufs(dpy, context_);
        bool ok = true;
        if (keyframe) ok = impl_->add_sequence(bufs, pic, err) && add_rate_control(bufs, err);
        ok = ok && impl_->add_picture(bufs, pic, err) && bufs.render(err);
        const VAStatus end = vaEndPicture(dpy, context_);
        if (ok) ok = va_check(end, "vaEndPicture (encode)", err);
        bufs.clear();
        if (!ok) return false;
        if (!va_check(vaSyncSurface(dpy, input_), "vaSyncSurface (encode)", err)) return false;

        void* map = nullptr;
        if (!va_check(vaMapBuffer(dpy, coded_, &map), "vaMapBuffer (coded)", err)) return false;
        for (auto* seg = static_cast<VACodedBufferSegment*>(map); seg; seg = static_cast<VACodedBufferSegment*>(seg->next)) {
            if (seg->status & VA_CODED_BUF_STATUS_SLICE_OVERFLOW_MASK) {
                vaUnmapBuffer(dpy, coded_);
                return fail(err, "coded buffer overflow");
            }
            const auto* bytes = static_cast<const uint8_t*>(seg->buf);
            out.data.insert(out.data.end(), bytes, bytes + seg->size);
        }
        vaUnmapBuffer(dpy, coded_);
        if (out.data.empty()) return fail(err, "the encoder produced no data");
        impl_->finish_packet(out.data);

        out.keyframe = keyframe;
        if (keyframe) ++keyframe_count_;
        ++index_in_gop_;
        current_ ^= 1;
        return true;
    }

    StreamParams sp_;
    std::unique_ptr<CodecImpl> impl_;
    std::unique_ptr<Display> display_;
    std::string driver_;
    VAConfigID config_id_ = VA_INVALID_ID;
    VAContextID context_ = VA_INVALID_ID;
    VASurfaceID input_ = VA_INVALID_SURFACE;
    VASurfaceID recon_[2] = {VA_INVALID_SURFACE, VA_INVALID_SURFACE};
    VABufferID coded_ = VA_INVALID_ID;
    Converter vpp_;
    UploadSurface upload_;
    uint64_t index_in_gop_ = 0;
    uint64_t keyframe_count_ = 0;
    unsigned current_ = 0;
};

// ---- the backend's factory entry points ------------------------------------------

detail::ProbeResult probe() {
    detail::ProbeResult r;
    const auto& nodes = probe_nodes();
    for (Codec c : supported_codecs()) {
        bool any = false;
        for (const NodeCaps& n : nodes) {
            const CodecCaps* cc = n.find(c);
            if (!cc) continue;
            any = true;
            Capability cap;
            cap.codec = c;
            cap.direction = Direction::Encode;
            cap.backend = Backend::VaApi;
            cap.hardware = true;
            cap.max_width = cc->max_width;
            cap.max_height = cc->max_height;
            cap.device = n.path + (n.driver.empty() ? "" : " (" + n.driver + ")");
            r.caps.push_back(std::move(cap));
        }
        if (!any) {
            r.missing.push_back({c, Direction::Encode,
                                 nodes.empty() ? "VA-API: no render node with a VA driver that has VideoProc and an "
                                                 "encoder ($BROVIDEO_VAAPI_DEBUG=1 says why)"
                                               : std::string("VA-API: no render node's driver encodes ") +
                                                     codec_name(c)});
        }

        bool any_dec = false;
        for (const NodeCaps& n : nodes) {
            const CodecCaps* dc = n.find_decoder(c);
            if (!dc) continue;
            any_dec = true;
            Capability cap;
            cap.codec = c;
            cap.direction = Direction::Decode;
            cap.backend = Backend::VaApi;
            cap.hardware = true;
            cap.max_width = dc->max_width;
            cap.max_height = dc->max_height;
            cap.device = n.path + (n.driver.empty() ? "" : " (" + n.driver + ")");
            r.caps.push_back(std::move(cap));
        }
        if (!any_dec) {
            r.missing.push_back({c, Direction::Decode,
                                 nodes.empty() ? "VA-API: no render node with a VA driver ($BROVIDEO_VAAPI_DEBUG=1 says why)"
                                               : std::string("VA-API: no render node's driver decodes ") +
                                                     codec_name(c)});
        }
    }
    return r;
}

std::unique_ptr<Encoder> create(const EncoderConfig& config, const Capability&, std::string*) {
    // The node is chosen on the first frame (the one that exported a dmabuf
    // frame, if it can encode the codec), so the capability only says one exists.
    return std::make_unique<VaEncoder>(config);
}

}  // namespace
}  // namespace brovideo::vaapi

namespace brovideo::detail {

const BackendOps& vaapi_backend() {
    static const BackendOps ops{Backend::VaApi, vaapi::probe, vaapi::create, vaapi::create_va_decoder};
    return ops;
}

}  // namespace brovideo::detail
