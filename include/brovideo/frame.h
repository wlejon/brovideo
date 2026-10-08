#pragma once
// The data that moves through brovideo: frames an encoder takes, the packets
// it makes of them, and the pictures a decoder makes of those.

#include <cstdint>
#include <memory>
#include <vector>

namespace brovideo {

enum class PixelFormat : uint8_t {
    RGBA8 = 0,  // 4 bytes per pixel in memory order R G B A
    NV12 = 1,   // Y: `height` rows of `stride` bytes; then at `uv_offset`,
                // (height + 1) / 2 rows of `stride` bytes of interleaved U V
    BGRA8 = 2,  // 4 bytes per pixel in memory order B G R A
};

// ---- encoder input ---------------------------------------------------------------

struct DmabufPlane {
    int fd = -1;
    uint32_t offset = 0;
    uint32_t pitch = 0;
};

// One picture to encode. Either a dmabuf (plane_count > 0: the GPU path,
// Linux) or a CPU picture (plane_count == 0): `cpu` points at `height` rows
// of `cpu_format` pixels (RGBA8 or BGRA8), `cpu_stride` bytes apart (0:
// width * 4). Nothing here is owned: the fds and the memory belong to the
// caller, which keeps them valid until the encoder calls the release
// callback it was given with the frame.
struct Frame {
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t drm_format = 0;    // DRM fourcc of a dmabuf frame, e.g. XRGB8888 (ignored for CPU frames)
    uint64_t modifier = 0;      // DRM format modifier
    uint32_t plane_count = 0;   // 0 => CPU frame
    DmabufPlane planes[4];
    int acquire_fence_fd = -1;  // sync_file; the content is ready when it signals (-1: ready). Not owned.
    const uint8_t* cpu = nullptr;
    uint32_t cpu_stride = 0;
    PixelFormat cpu_format = PixelFormat::RGBA8;  // RGBA8 or BGRA8
    int64_t pts_ns = 0;         // carried through to the packet unchanged

    [[nodiscard]] bool is_cpu() const noexcept { return plane_count == 0; }
    [[nodiscard]] uint32_t cpu_row_bytes() const noexcept { return cpu_stride ? cpu_stride : width * 4; }
};

// ---- encoder output ----------------------------------------------------------------

// One encoded picture: Annex B for H.264/HEVC, a temporal unit of OBUs for
// AV1, the Raw format (docs/raw.md) for Codec::Raw. A keyframe carries its
// parameter sets in band, so a decoder needs nothing but the packets.
struct Packet {
    std::vector<uint8_t> data;
    bool keyframe = false;
    int64_t pts_ns = 0;
};

// ---- decoder output ----------------------------------------------------------------

// Where a decoded picture lives.
enum class PictureMemory : uint8_t {
    Cpu = 0,    // `data`, laid out as `format` says
    D3D11 = 1,  // `gpu`: a slice of an NV12 ID3D11Texture2D (Media Foundation, DXVA)
};

// A picture on the GPU. The handles are borrowed: `keepalive` holds the
// decoder's output sample, so the texture stays valid for as long as the
// Picture (or a copy of `keepalive`) lives. Hold one only briefly: the
// decoder works from a small pool of these surfaces and stalls when they
// are all held.
struct GpuPicture {
    void* device = nullptr;    // D3D11: ID3D11Device*
    void* texture = nullptr;   // D3D11: ID3D11Texture2D* (NV12, possibly an array)
    uint32_t subresource = 0;  // D3D11: the slice / subresource index holding this picture
    uint32_t x = 0;            // the visible picture's origin in the texture
    uint32_t y = 0;
    std::shared_ptr<void> keepalive;
};

// A decoded picture. `width` x `height` is the visible picture: decoders
// crop what they can (H.264/HEVC conformance windows, a DXVA surface's
// alignment), but AV1 cannot signal a crop and some encoders pad it (VCN 4
// codes 1080 lines as 1082), so a consumer that knows the intended size
// crops to it: for a CPU picture that is just a smaller width / height over
// the same rows and uv_offset.
struct Picture {
    bool ready = false;  // false: the decoder consumed the input but has no picture yet
    uint32_t width = 0;
    uint32_t height = 0;
    PixelFormat format = PixelFormat::RGBA8;
    PictureMemory memory = PictureMemory::Cpu;
    // Cpu: the pixels.
    uint32_t stride = 0;
    uint32_t uv_offset = 0;  // NV12 only
    std::vector<uint8_t> data;
    // D3D11: the texture (data stays empty).
    GpuPicture gpu;
    int64_t pts_ns = 0;  // the pts given with the packet this picture came from
};

}  // namespace brovideo
