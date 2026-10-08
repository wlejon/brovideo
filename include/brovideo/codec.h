#pragma once
// Codecs, the backends that implement them, and the capability probe: what
// this build and this machine can encode and decode, and whether in hardware.

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace brovideo {

// The values are stable (a consumer may put them on a wire or in a file);
// new codecs only ever take new values.
enum class Codec : uint8_t {
    Raw = 0,   // built in everywhere: RGBA, run-length coded, XOR-delta predicted frames (docs/raw.md)
    H264 = 1,  // Annex B
    HEVC = 2,  // Annex B
    AV1 = 3,   // a temporal unit of OBUs per packet (low-overhead bitstream format)
};

[[nodiscard]] const char* codec_name(Codec) noexcept;  // "raw", "h264", "hevc", "av1"
// The inverse of codec_name; also takes "h265".
[[nodiscard]] std::optional<Codec> parse_codec(std::string_view name) noexcept;
// Every codec this version of the library defines, Raw first.
[[nodiscard]] const std::vector<Codec>& all_codecs();

enum class Direction : uint8_t { Encode, Decode };

// Where an encoder or decoder comes from. Backends that are not built leave
// no trace beyond their name here.
enum class Backend : uint8_t {
    Raw = 0,              // the built-in Raw codec (CPU)
    VaApi = 1,            // Linux: libva (encode today; decode is a later backend)
    MediaFoundation = 2,  // Windows: the system decoder MFTs, D3D11 / DXVA when the machine allows
    VulkanVideo = 3,      // reserved
    VideoToolbox = 4,     // reserved
};

[[nodiscard]] const char* backend_name(Backend) noexcept;  // "raw", "vaapi", "mediafoundation", ...

// One thing this machine can do: encode or decode one codec through one
// backend.
struct Capability {
    Codec codec = Codec::Raw;
    Direction direction = Direction::Encode;
    Backend backend = Backend::Raw;
    bool hardware = false;      // runs on a GPU / fixed-function block (false: CPU)
    uint32_t max_width = 0;     // 0: not reported
    uint32_t max_height = 0;
    std::string device;         // what does it, e.g. "/dev/dri/renderD128 (amdgpu)"; may be empty
};

// Everything this build and machine can do, best first for each codec and
// direction. The first call probes (opens devices, decodes a test picture;
// up to about a second on Windows) and the result is kept for the process:
// a codec is listed only when it really works here, not merely when a
// driver claims it.
[[nodiscard]] const std::vector<Capability>& capabilities();
// The codecs `direction` is possible for, Raw first, without duplicates.
[[nodiscard]] std::vector<Codec> codecs(Direction direction);
[[nodiscard]] bool supported(Codec codec, Direction direction);
// The best capability for codec / direction, or null.
[[nodiscard]] const Capability* best(Codec codec, Direction direction);
// Why `codec` cannot be done in `direction` here (empty when it can): the
// backend's own account when one tried, else that nothing in this build
// implements it.
[[nodiscard]] std::string unavailable_reason(Codec codec, Direction direction);

}  // namespace brovideo
