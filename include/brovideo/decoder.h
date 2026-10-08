#pragma once
// Decoders: packets in, pictures out.

#include "brovideo/codec.h"
#include "brovideo/frame.h"

#include <cstdint>
#include <memory>
#include <span>
#include <string>

namespace brovideo {

enum class Hardware : uint8_t {
    Prefer = 0,   // hardware where the probe saw it work, else software
    Require = 1,  // hardware or nothing
    Forbid = 2,   // software only
};

struct DecoderConfig {
    Codec codec = Codec::H264;
    Hardware hardware = Hardware::Prefer;
    // Gpu: pictures stay on the GPU where the decoder can hand them over
    // (Media Foundation with D3D11: PictureMemory::D3D11) and come to CPU
    // memory otherwise; check Picture::memory. Cpu: always CPU memory.
    PictureMemory output = PictureMemory::Cpu;
};

class Decoder {
public:
    virtual ~Decoder() = default;
    Decoder(const Decoder&) = delete;
    Decoder& operator=(const Decoder&) = delete;

    // Decodes one packet (one whole picture: an access unit, a temporal
    // unit, a Raw packet). False (with *err) when the bitstream is bad or
    // the decoder lost sync (a predicted frame with no keyframe before it,
    // or after an earlier failure): the decoder takes nothing but a
    // keyframe until it has one again. `out.ready` is false when the decoder
    // needs more input before it has a picture. `pts_ns` comes back on the
    // picture this packet makes.
    virtual bool decode(std::span<const uint8_t> packet, int64_t pts_ns, Picture& out, std::string* err) = 0;
    bool decode(std::span<const uint8_t> packet, Picture& out, std::string* err) {
        return decode(packet, 0, out, err);
    }

    // What is decoding, for a consumer to show: e.g. "Media Foundation
    // hevc, hardware (D3D11)".
    [[nodiscard]] virtual std::string describe() const = 0;
    [[nodiscard]] virtual bool hardware() const = 0;
    [[nodiscard]] Codec codec() const noexcept { return codec_; }

protected:
    explicit Decoder(Codec codec) : codec_(codec) {}

private:
    Codec codec_;
};

// A decoder for config.codec: the best capability the probe found that
// config.hardware allows. Null with *err (saying why the codec is missing
// here) when none can do it. $BROVIDEO_HARDWARE=0 in the environment makes
// Prefer act as Forbid.
std::unique_ptr<Decoder> create_decoder(const DecoderConfig& config, std::string* err);

}  // namespace brovideo
