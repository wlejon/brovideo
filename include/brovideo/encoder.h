#pragma once
// Encoders: frames in, packets out.

#include "brovideo/codec.h"
#include "brovideo/frame.h"

#include <atomic>
#include <functional>
#include <memory>
#include <string>

namespace brovideo {

// What an encoder is made for. Every encoder makes the same low-latency
// stream: a keyframe, then predicted frames each referencing only the
// picture before it, no B-frames, no reordering, one packet per frame out
// of every encode() call, constant bitrate.
struct EncoderConfig {
    Codec codec = Codec::H264;
    uint32_t width = 0;        // the picture size; a frame of another size is scaled to it (Raw: refused)
    uint32_t height = 0;
    uint32_t fps = 60;         // for rate control: bits per frame = bitrate / fps
    uint32_t bitrate_kbps = 20000;
    bool allow_software = true;  // false: a hardware encoder or none
};

// What is encoding, for a consumer to show.
struct EncoderInfo {
    Backend backend = Backend::Raw;
    bool hardware = false;
    // e.g. "VA-API hevc, /dev/dri/renderD128 (amdgpu; Mesa Gallium driver ...)"; the device is
    // known once the first frame has chosen it.
    std::string description;
};

class Encoder {
public:
    virtual ~Encoder() = default;
    Encoder(const Encoder&) = delete;
    Encoder& operator=(const Encoder&) = delete;

    // Encodes one frame into `out`. Calls `release` exactly once, as soon as
    // the frame's memory is no longer read (before the encode itself
    // finishes; a dmabuf frame after its acquire fence signalled and the
    // GPU copied it), on success and on failure alike. The packet is a
    // keyframe when it is the encoder's first, after reconfigure() or a
    // failure, when `force_keyframe` is set, or when request_keyframe() was
    // called since the last encode. False (with *err) on failure; the next
    // encode starts again with a keyframe.
    bool encode(const Frame& frame, bool force_keyframe, const std::function<void()>& release, Packet& out,
                std::string* err);

    // Encodes the last frame's picture again, from the encoder's own copy of
    // it (no source frame), into `out` with that frame's pts. A predicted
    // repeat codes what the picture before it got wrong: the first frame
    // after a big change, which rate control leaves soft, sharpens over a
    // few repeats, and a picture already at the bitrate's quality comes out
    // as a near-empty packet. For a source that sends frames only when
    // something changed (a compositor holding an unchanged screen), so a
    // still picture does not stay soft, and a keyframe can be had without a
    // new frame. The keyframe rules are encode()'s. False with *err unset
    // when there is no picture to repeat (none since a failure or
    // reconfigure(), or the backend keeps none: repeats() false); false with
    // *err when the encode failed, as encode().
    bool encode_repeat(bool force_keyframe, Packet& out, std::string* err);
    [[nodiscard]] bool can_repeat() const noexcept { return has_picture_ && repeats(); }

    // The next encode() makes a keyframe. Any thread.
    void request_keyframe() noexcept { keyframe_requested_.store(true, std::memory_order_relaxed); }

    // Changes the bitrate, frame rate or size (not the codec) from the next
    // frame on, which is a keyframe. False (with *err, the encoder unchanged)
    // when the backend cannot: the caller then makes a new encoder.
    bool reconfigure(const EncoderConfig& config, std::string* err);

    [[nodiscard]] const EncoderConfig& config() const noexcept { return config_; }
    [[nodiscard]] virtual EncoderInfo info() const = 0;

protected:
    explicit Encoder(const EncoderConfig& config) : config_(config) {}
    // The backend's half: `keyframe` is decided; `release` must run exactly
    // once (the base guarantees it when the backend forgets).
    virtual bool encode_frame(const Frame& frame, bool keyframe, const std::function<void()>& release, Packet& out,
                              std::string* err) = 0;
    // Called with the new config before config_ changes; false refuses it.
    virtual bool apply_config(const EncoderConfig& next, std::string* err) = 0;
    // A backend that keeps the last picture (its encoder input) overrides
    // both: encode_repeat_frame codes that picture again.
    [[nodiscard]] virtual bool repeats() const { return false; }
    virtual bool encode_repeat_frame(bool /*keyframe*/, Packet& /*out*/, std::string* /*err*/) { return false; }

    EncoderConfig config_;

private:
    std::atomic<bool> keyframe_requested_{false};
    bool need_keyframe_ = true;
    bool has_picture_ = false;  // a frame was encoded since the last failure or reconfigure()
    int64_t last_pts_ns_ = 0;
};

// An encoder for config.codec: the best capability the probe found
// (hardware first). Null with *err when none can do it.
std::unique_ptr<Encoder> create_encoder(const EncoderConfig& config, std::string* err);

}  // namespace brovideo
