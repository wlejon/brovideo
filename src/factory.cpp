// The codec names, the capability probe and the factory: the single place
// backends register. A backend that is not compiled in (or finds no
// hardware) is simply absent from the capabilities.
#include "backend.h"

#include <algorithm>
#include <cstdlib>
#include <mutex>

namespace brovideo {

const char* codec_name(Codec c) noexcept {
    switch (c) {
        case Codec::Raw: return "raw";
        case Codec::H264: return "h264";
        case Codec::HEVC: return "hevc";
        case Codec::AV1: return "av1";
    }
    return "unknown";
}

std::optional<Codec> parse_codec(std::string_view name) noexcept {
    for (Codec c : all_codecs()) {
        if (name == codec_name(c)) return c;
    }
    if (name == "h265") return Codec::HEVC;
    return std::nullopt;
}

const std::vector<Codec>& all_codecs() {
    static const std::vector<Codec> all{Codec::Raw, Codec::H264, Codec::HEVC, Codec::AV1};
    return all;
}

const char* backend_name(Backend b) noexcept {
    switch (b) {
        case Backend::Raw: return "raw";
        case Backend::VaApi: return "vaapi";
        case Backend::MediaFoundation: return "mediafoundation";
        case Backend::VulkanVideo: return "vulkanvideo";
        case Backend::VideoToolbox: return "videotoolbox";
    }
    return "unknown";
}

namespace {

const char* direction_word(Direction d) { return d == Direction::Encode ? "encoding" : "decoding"; }

// Hardware backends first, so the best capability for a codec comes first.
const std::vector<const detail::BackendOps*>& backends() {
    static const std::vector<const detail::BackendOps*> list{
#if defined(BROVIDEO_HAVE_VAAPI)
        &detail::vaapi_backend(),
#endif
#if defined(BROVIDEO_HAVE_MF)
        &detail::mf_backend(),
#endif
        &detail::raw_backend(),
    };
    return list;
}

struct Probe {
    std::vector<Capability> caps;
    std::vector<detail::Missing> missing;
};

const Probe& probe() {
    static Probe p;
    static std::once_flag once;
    std::call_once(once, [] {
        for (const detail::BackendOps* b : backends()) {
            detail::ProbeResult r = b->probe();
            for (Capability& c : r.caps) p.caps.push_back(std::move(c));
            for (detail::Missing& m : r.missing) p.missing.push_back(std::move(m));
        }
        // Within one codec and direction: hardware first, the backends'
        // order otherwise.
        std::stable_sort(p.caps.begin(), p.caps.end(), [](const Capability& a, const Capability& b) {
            if (a.codec != b.codec) return a.codec < b.codec;
            if (a.direction != b.direction) return a.direction < b.direction;
            return a.hardware && !b.hardware;
        });
    });
    return p;
}

const detail::BackendOps* ops_for(Backend b) {
    for (const detail::BackendOps* o : backends()) {
        if (o->backend == b) return o;
    }
    return nullptr;
}

// $BROVIDEO_HARDWARE=0 turns Hardware::Prefer into Forbid.
bool hardware_allowed() {
    const char* v = std::getenv("BROVIDEO_HARDWARE");
    return !(v && v[0] == '0');
}

}  // namespace

const std::vector<Capability>& capabilities() { return probe().caps; }

std::vector<Codec> codecs(Direction direction) {
    std::vector<Codec> out;
    for (Codec c : all_codecs()) {
        if (supported(c, direction)) out.push_back(c);
    }
    return out;
}

bool supported(Codec codec, Direction direction) { return best(codec, direction) != nullptr; }

const Capability* best(Codec codec, Direction direction) {
    for (const Capability& c : capabilities()) {
        if (c.codec == codec && c.direction == direction) return &c;
    }
    return nullptr;
}

std::string unavailable_reason(Codec codec, Direction direction) {
    if (supported(codec, direction)) return {};
    std::string why;
    for (const detail::Missing& m : probe().missing) {
        if (m.codec == codec && m.direction == direction && !m.why.empty()) why += (why.empty() ? "" : "; ") + m.why;
    }
    if (!why.empty()) return why;
    return std::string("nothing in this build implements ") + codec_name(codec) + " " + direction_word(direction);
}

std::unique_ptr<Encoder> create_encoder(const EncoderConfig& config, std::string* err) {
    if (config.width == 0 || config.height == 0) {
        if (err) *err = "encoder size is empty";
        return nullptr;
    }
    std::string last;
    for (const Capability& c : capabilities()) {
        if (c.codec != config.codec || c.direction != Direction::Encode) continue;
        if (!c.hardware && !config.allow_software) continue;
        const detail::BackendOps* ops = ops_for(c.backend);
        if (!ops || !ops->create_encoder) continue;
        if (auto e = ops->create_encoder(config, c, &last)) return e;
    }
    if (err) {
        if (!last.empty()) {
            *err = last;
        } else if (supported(config.codec, Direction::Encode)) {
            *err = std::string("no hardware ") + codec_name(config.codec) + " encoder here";
        } else {
            *err = std::string(codec_name(config.codec)) + " encoding is not available here: " +
                   unavailable_reason(config.codec, Direction::Encode);
        }
    }
    return nullptr;
}

std::unique_ptr<Decoder> create_decoder(const DecoderConfig& config, std::string* err) {
    Hardware hw = config.hardware;
    if (hw == Hardware::Prefer && !hardware_allowed()) hw = Hardware::Forbid;
    std::string last;
    for (const Capability& c : capabilities()) {
        if (c.codec != config.codec || c.direction != Direction::Decode) continue;
        if (hw == Hardware::Require && !c.hardware) continue;
        if (hw == Hardware::Forbid && c.hardware) continue;
        const detail::BackendOps* ops = ops_for(c.backend);
        if (!ops || !ops->create_decoder) continue;
        if (auto d = ops->create_decoder(config, c, &last)) return d;
    }
    if (err) {
        if (!last.empty()) {
            *err = last;
        } else if (supported(config.codec, Direction::Decode)) {
            *err = std::string("no ") + (hw == Hardware::Require ? "hardware " : "software ") +
                   codec_name(config.codec) + " decoder here";
        } else {
            *err = std::string(codec_name(config.codec)) + " decoding is not available here: " +
                   unavailable_reason(config.codec, Direction::Decode);
        }
    }
    return nullptr;
}

// ---- Encoder -------------------------------------------------------------------

bool Encoder::encode(const Frame& frame, bool force_keyframe, const std::function<void()>& release, Packet& out,
                     std::string* err) {
    bool released = false;
    const std::function<void()> once = [&] {
        if (!released) {
            released = true;
            if (release) release();
        }
    };
    const bool requested = keyframe_requested_.exchange(false, std::memory_order_relaxed);
    const bool keyframe = force_keyframe || requested || need_keyframe_;
    out.data.clear();
    out.keyframe = false;
    out.pts_ns = frame.pts_ns;
    const bool ok = encode_frame(frame, keyframe, once, out, err);
    once();
    if (!ok) {
        out.data.clear();
        need_keyframe_ = true;  // the reference state is unknown now
        has_picture_ = false;
        return false;
    }
    out.keyframe = out.keyframe || keyframe;  // a backend may make one on its own
    out.pts_ns = frame.pts_ns;
    need_keyframe_ = false;
    has_picture_ = true;
    last_pts_ns_ = frame.pts_ns;
    return true;
}

bool Encoder::encode_repeat(bool force_keyframe, Packet& out, std::string* err) {
    out.data.clear();
    out.keyframe = false;
    out.pts_ns = last_pts_ns_;
    if (!can_repeat()) return false;
    const bool requested = keyframe_requested_.exchange(false, std::memory_order_relaxed);
    const bool keyframe = force_keyframe || requested || need_keyframe_;
    if (!encode_repeat_frame(keyframe, out, err)) {
        out.data.clear();
        need_keyframe_ = true;
        has_picture_ = false;
        if (err && err->empty()) *err = "the repeat encode failed";
        return false;
    }
    out.keyframe = out.keyframe || keyframe;
    out.pts_ns = last_pts_ns_;
    need_keyframe_ = false;
    return true;
}

bool Encoder::reconfigure(const EncoderConfig& config, std::string* err) {
    if (config.codec != config_.codec) {
        if (err) *err = "reconfigure cannot change the codec";
        return false;
    }
    if (config.width == 0 || config.height == 0) {
        if (err) *err = "encoder size is empty";
        return false;
    }
    if (!apply_config(config, err)) return false;
    config_ = config;
    need_keyframe_ = true;
    has_picture_ = false;  // the kept picture is the old size (or gone with the session)
    return true;
}

}  // namespace brovideo
