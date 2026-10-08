# brovideo

Hardware video encode and decode as a dependency-free C++20 library: frames
(a dmabuf with its acquire fence, or CPU RGBA/BGRA) in, H.264 / HEVC / AV1
packets out; packets in, pictures (CPU NV12 or a D3D11 texture) out. Codecs
are reached only through the platform's own APIs: VA-API on Linux, Media
Foundation on Windows. No ffmpeg, no GPL.

In the [bro ecosystem](https://github.com/wlejon/bro/blob/main/docs/ecosystem.md)
brovideo is the codec layer of broremote (remote desktop), and is meant for
bro's `<video>`, `VideoEncoder` and screen recording later. It knows nothing
about where the video goes: consumers keep their own policy (which codec,
when to ask for a keyframe, when to encode).

[docs/design.md](docs/design.md) is the design, the API contract, the
backends and their driver workarounds; [docs/raw.md](docs/raw.md) is the Raw
codec's packet format.

## Status

| Part | State |
|------|-------|
| API: codecs, capability probe, Encoder (keyframe request, reconfigure), Decoder (pts, CPU or GPU pictures) | done |
| Raw codec (CPU RGBA, run-length + XOR-delta), everywhere | done |
| VA-API encoder (Linux): H.264, HEVC, AV1; dmabuf or CPU input | done, tested on radeonsi (VCN 4) |
| Media Foundation decoder (Windows): H.264, HEVC, AV1; D3D11 or software | done, tested on an RTX 4090 |
| VA-API decode, Vulkan Video, VideoToolbox | not yet (the backend slot is there) |

```cpp
#include <brovideo/brovideo.h>

brovideo::EncoderConfig ec;
ec.codec = brovideo::Codec::HEVC;
ec.width = 1920;
ec.height = 1080;
std::string err;
auto enc = brovideo::create_encoder(ec, &err);          // null with err when nothing here can
brovideo::Packet pkt;
enc->encode(frame, /*force_keyframe=*/false, [&] { /* frame memory free */ }, pkt, &err);

auto dec = brovideo::create_decoder({brovideo::Codec::HEVC}, &err);
brovideo::Picture pic;
dec->decode(pkt.data, pkt.pts_ns, pic, &err);           // pic.ready: NV12 in pic.data
```

## Building

Windows (Visual Studio 2022; one build dir, the config picked at build time):
```bash
cmake -B build
cmake --build build --config Release
ctest --test-dir build -C Release
```

Linux (needs libva, libva-drm and libdrm for the VA-API encoder, and
[brodmabuf](../brodmabuf) checked out beside this repository):
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build
```

Options: `BROVIDEO_BUILD_TESTS` (on), `BROVIDEO_WITH_VAAPI` (on on Linux when
libva is found), `BROVIDEO_WITH_MF` (on on Windows). brodmabuf resolves as an
existing `brodmabuf` target, then `-DBRODMABUF_DIR`, then `../brodmabuf`.

Tests use ffmpeg as an oracle when it is on PATH (never linked).
Environment: `BROVIDEO_HARDWARE=0` decodes in software,
`BROVIDEO_VAAPI_DEVICE=/dev/dri/renderD129` picks the VA-API node,
`BROVIDEO_VAAPI_DEBUG=1` logs the VA-API backend.

MIT licensed.
