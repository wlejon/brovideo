# brovideo design

brovideo encodes and decodes video through the platform's own hardware
APIs: frames (a dmabuf with its acquire fence, or CPU RGBA/BGRA) in, packets
out; packets in, pictures (CPU memory or a GPU texture) out. It is a C++20
static library with no dependencies beyond the platform: libva and libdrm on
Linux (and brodmabuf, the ecosystem's dependency-free dmabuf sibling), Media
Foundation and D3D11 on Windows. No ffmpeg, no x264, no GPL: ffmpeg is used
only by the tests, as an oracle, when it is on PATH.

It knows nothing about where the video goes. broremote (remote desktop) uses
it today; bro's `<video>`, `VideoEncoder` and screen recording are meant to
be able to use it later. So nothing here is about sessions, links, viewers or
flow control: a consumer decides which codec to use, when it wants a
keyframe and when to encode at all.

## Parts

```
include/brovideo/
  codec.h       Codec, Direction, Backend, Capability; the capability probe
  frame.h       Frame (encoder input), Packet, Picture / GpuPicture (decoder output)
  encoder.h     EncoderConfig, EncoderInfo, Encoder, create_encoder
  decoder.h     Hardware, DecoderConfig, Decoder, create_decoder
  bitstream.h   is_keyframe(codec, packet)
  brovideo.h    all of the above
src/
  factory.cpp   names, the probe (once per process), the factory, the
                Encoder base (keyframe and release bookkeeping)
  backend.h     the one shape a backend has (BackendOps) and the list of them
  bitstream.cpp keyframe detection: H.264 IDR, HEVC IRAP, AV1 sequence header + key frame
  raw/          the Raw codec (docs/raw.md)
  vaapi/        VA-API encoder (Linux): va_device (nodes, probe), va_source
                (fence, dmabuf import, CPU upload), va_vpp (colour
                conversion), va_encoder (the common half), va_h264,
                va_hevc, va_av1 (parameters and packed headers)
  mf/           Media Foundation decoder (Windows): mf_util (startup, MFT
                lookup, D3D11 device), mf_decoder, mf_probe
tests/          test_raw (everywhere), test_vaapi (Linux), test_mf (Windows);
                tests/fixtures holds streams recorded from the VA-API encoder
```

CMake target `brovideo` (alias `brovideo::brovideo`). Options:
`BROVIDEO_BUILD_TESTS` (on), `BROVIDEO_WITH_VAAPI` (auto on Linux when libva,
libva-drm and libdrm are found; it needs brodmabuf: an existing `brodmabuf`
target, else `../brodmabuf`, else the commit pinned in CMakeLists.txt,
fetched by cmake/bro_deps.cmake), `BROVIDEO_WITH_MF` (on
for Windows). A top-level MSVC build links the CRT statically; inside another
project brovideo takes that project's CRT.

## API

```cpp
enum class Codec : uint8_t { Raw = 0, H264 = 1, HEVC = 2, AV1 = 3 };  // stable values
enum class Direction : uint8_t { Encode, Decode };
enum class Backend : uint8_t { Raw, VaApi, MediaFoundation, VulkanVideo, VideoToolbox };
struct Capability { Codec codec; Direction direction; Backend backend; bool hardware;
                    uint32_t max_width, max_height; std::string device; };
const std::vector<Capability>& capabilities();  // probed once, best (hardware) first
std::vector<Codec> codecs(Direction);           // Raw first
bool supported(Codec, Direction);
const Capability* best(Codec, Direction);
std::string unavailable_reason(Codec, Direction);

struct EncoderConfig { Codec codec; uint32_t width, height, fps = 60, bitrate_kbps = 20000;
                       bool allow_software = true; };
class Encoder {
    bool encode(const Frame&, bool force_keyframe, const std::function<void()>& release,
                Packet& out, std::string* err);
    bool encode_repeat(bool force_keyframe, Packet& out, std::string* err);  // the last picture again
    bool can_repeat() const noexcept;
    void request_keyframe() noexcept;           // any thread; the next encode is a keyframe
    bool reconfigure(const EncoderConfig&, std::string* err);  // bitrate / fps / size, not codec
    const EncoderConfig& config() const;
    virtual EncoderInfo info() const;           // backend, hardware, description
};
std::unique_ptr<Encoder> create_encoder(const EncoderConfig&, std::string* err);

enum class Hardware : uint8_t { Prefer, Require, Forbid };
struct DecoderConfig { Codec codec; Hardware hardware = Prefer; PictureMemory output = Cpu; };
class Decoder {
    virtual bool decode(std::span<const uint8_t> packet, int64_t pts_ns, Picture& out, std::string* err);
    virtual std::string describe() const;       // e.g. "Media Foundation hevc, hardware (D3D11)"
    virtual bool hardware() const;
    Codec codec() const;
};
std::unique_ptr<Decoder> create_decoder(const DecoderConfig&, std::string* err);
```

The headers are the reference; what they promise:

- **One stream shape.** Every encoder makes a low-latency stream: a keyframe,
  then predicted frames each referencing only the picture before it, no
  B-frames, no reordering, one packet out of every `encode()`, constant
  bitrate. Keyframes carry their parameter sets in band (Annex B for
  H.264/HEVC, a temporal unit of OBUs with a sequence header for AV1), so a
  decoder needs nothing but the packets and can start at any keyframe.
- **Release.** `encode()` calls `release` exactly once, as soon as the
  frame's memory is no longer read (for a dmabuf, once its acquire fence
  signalled and the GPU copied it; before the encode itself finishes), on
  failure too. The base class guarantees it even when a backend forgets.
  Nothing in a `Frame` is owned: the fds and the CPU memory are the
  caller's.
- **Keyframes.** The packet is a keyframe when it is the encoder's first,
  after `reconfigure()`, after a failure, when `force_keyframe` is set, or
  when `request_keyframe()` was called since the last encode.
- **Reconfigure.** Bitrate, frame rate and size change from the next frame
  on, which is a keyframe; the codec cannot change. False leaves the encoder
  as it was, and the caller makes a new one.
- **Repeats.** `encode_repeat()` codes the last frame's picture again from
  the encoder's own copy (VA-API: its NV12 input surface; Raw: its last
  picture), with that frame's pts and `encode()`'s keyframe rules; no frame
  is held for it. A predicted repeat codes what the picture before it got
  wrong, so a soft picture sharpens: for a source that sends frames only on
  change, whose first frame after a big change would otherwise stay as
  soft as the HRD buffer left it (test_vaapi measures 18 dB rising to 50
  over about 25 HEVC repeats at 20 Mbit/s). A keyframe repeat gives a
  joining decoder the picture without a new frame. `can_repeat()` is false
  before the first frame, after a failure or `reconfigure()`, and on a
  backend that keeps no picture (Media Foundation has no encoder); then
  `encode_repeat()` returns false with `*err` untouched.
- **Damage.** A frame may say what changed since the previous one
  (`damage_known`, `damage`: rects in frame pixels; known and empty is the
  same picture again). It is a hint: every pixel is coded either way. CBR
  with a short HRD buffer sizes each frame by the per-frame budget, so right
  after a big change rate control has the QP near its coarsest, and a
  source that sends frames only on change sends a small change that keeps
  coming (a build's progress line rewritten at 20 Hz) as frames that each
  land that soft, with no still moment for repeats to sharpen it: halo, 20
  Mbit/s 1080p, a line rewritten right after a window opened lands at 20.7
  dB (HEVC), 19.0 (H.264), 22.8 (AV1). The VA-API encoder codes the damaged
  blocks finer than rate control chose, through the driver's ROI QP deltas
  (`VAEncMiscParameterTypeROI`, used only when the driver reports
  `roi_rc_qp_delta_support`): -30 QP steps for a damage of at most 2% of
  the picture, tapering linearly to none at 40%, so a big change keeps its
  small fast packet. The rects are aligned outward to the codec's QP block
  (16 for H.264, 64 for HEVC and AV1; radeonsi rounds a region's origin
  *up*, so an unaligned rect misses the line it is for) and merged down to
  the driver's region count. A boost makes the frame bigger than rate
  control would and rate control does not take all of it back, so the
  encoder counts every packet against the bitrate and tapers the boost
  while the stream is over, by up to half a second's worth (none past it):
  small damage arriving every frame stays within the bitrate, a burst is
  boosted in full. Keyframes and repeats get no boost. Measured with the
  boost: that line lands at 48.1 dB (HEVC), 36.8 (H.264), 45.7 (AV1); 16
  lines rewritten at 20 Hz land at 42.4 / 34.6 / 54.8 against 37.2 / 33.8
  / 50.2; a window opening is the same packet as without. A per-frame
  `max_qp` would do it too, but radeonsi applies a frame's QP range only
  through its per-picture rate control, which only a `max_frame_size`
  change triggers, and a cap on the whole frame makes a big change three to
  four times bigger (680-940 kB against about 220). Media Foundation
  ignores damage: NVENC's rate control already lands these lines at 54-60
  dB, with a big change as one large packet (0.7-1.1 MB) instead.
  `damage_boost()` and `BoostBudget` (src/damage.h) are the shared policy
  for any backend that adds this.
- **Lost sync.** `decode()` returns false when a packet is bad or the decoder
  lost sync (a predicted frame with no keyframe before it, or after an
  earlier failure); it then takes nothing but a keyframe. `out.ready` is
  false when the decoder took the packet but has no picture yet. The pts
  given with a packet comes back on its picture.
- **Pictures.** CPU pictures are RGBA8 (Raw) or NV12 (everything else).
  With `output = PictureMemory::D3D11`, Media Foundation in hardware hands
  over the decoder's own texture slice (`GpuPicture`: device, texture,
  subresource, origin, and a keepalive holding the output sample); anything
  else still comes back in CPU memory, so check `Picture::memory`. The
  visible size is the picture's: decoders crop what they can, but AV1 cannot
  signal a crop and VCN 4 pads it (1080 lines coded as 1082), so a consumer
  that knows the intended size crops to it.
- **The probe is honest.** A codec is listed only when it works on this
  machine (VA-API: the driver reports the entrypoint and VideoProc; Media
  Foundation: a real keyframe decodes to the right picture), not when a
  driver merely claims it. `unavailable_reason()` says why one is missing.
  Probing opens devices and takes up to about 15 ms per VA node and about
  0.6 s for Media Foundation, once per process.

Environment: `$BROVIDEO_HARDWARE=0` makes `Hardware::Prefer` act as `Forbid`
(software decoding); `$BROVIDEO_VAAPI_DEVICE` names the render node the VA-API
encoder uses; `$BROVIDEO_VAAPI_DEBUG=1` logs what the VA-API backend sees.

## Adding a backend

A backend is a `detail::BackendOps` (src/backend.h): a `probe()` that returns
what it can do here (and, for what it implements but cannot do, why), and
`create_encoder` / `create_decoder` for one of the capabilities it reported.
Adding one (VA-API decode, Vulkan Video, VideoToolbox) is its sources, its
`BROVIDEO_HAVE_*` definition in CMakeLists.txt, a declaration in backend.h
and one line in factory.cpp's list. The factory sorts every backend's
capabilities hardware first and picks the best one a config allows, so
consumers change nothing. `Backend::VulkanVideo` and `Backend::VideoToolbox`
are reserved for those.

## VA-API encoder (Linux)

Waits on the acquire fence (brodmabuf's `SyncFile`, a CPU `poll` on a dup of
the sync_file, failing after one second), imports the dmabuf as a VA surface
(DRM PRIME 2, with the modifier; XRGB8888, ARGB8888, XBGR8888, ABGR8888;
planes on one buffer are one object, so DCC metadata planes work), runs a
VideoProc pass to an NV12 surface it owns (BT.709, limited range, scaled to
the visible size at the top left of the coded surface), waits for that copy,
releases the frame, then encodes. CPU frames (RGBA8 or BGRA8) are copied into
a VA surface of that fourcc (mapped directly where the driver allows) and
released as soon as the copy is done; they take the same VideoProc path, so
there is no colour conversion on the CPU. The imported surface is made and
destroyed per frame: holding imports across frames would pin buffers the
caller may free.

- *Colour.* The RGB input is tagged BT.709, not sRGB: tagged sRGB, radeonsi
  also converts the transfer function and lifts every mid-tone by about five
  code values. Chroma is centre-sited (the pass then averages each 2x2 block
  rather than point-sampling, 1.3 dB RGB PSNR on the test pattern) and the
  stream says so (`chroma_sample_loc_type` 1), with the BT.709 limited-range
  colour description.
- *Device.* One VADisplay per encoder, opened on the first frame: for a
  dmabuf frame, the render node whose kernel driver exported it
  (`exp_name` in `/proc/self/fdinfo`), else the first render node with the
  codec and VideoProc. `$BROVIDEO_VAAPI_DEVICE` names the node instead. The
  probe opens each node once per process (driver init, about 15 ms) and is
  empty, not an error, without a node or a driver; libva itself is linked,
  so it must be installed where the library runs.
- *Sequence.* I and P only, no B-frames: an IDR, then predicted frames each
  referencing only the one before it (two reconstructed surfaces
  alternate), until the next keyframe. CBR at the configured bitrate with a
  50 ms HRD buffer (three frames at 60 fps: for a remote desktop a big change
  is one packet that must cross the link before it shows, and with half a
  second of buffer desktop-like content made packets of up to 450 kB, 4-5 ms
  over ssh, against about 110 kB and 1.5 ms now, mean latency unchanged; the
  first frame after a big change is softer and sharpens over the next few,
  which a still screen must get from `encode_repeat()`: see Repeats)
  and no filler (`disable_bit_stuffing`): padding a quiet picture up to the
  bitrate is wasted bandwidth, and on VCN 4 (encoder firmware ENC 1.24) an
  AV1 frame needing more than about 35 KB of padding OBU hangs the encode
  ring. Mesa enables filler in CBR unless the application turns it off;
  ffmpeg's VA-API encoders and RADV's Vulkan Video leave it on. H.264: High
  (Main / Constrained Baseline when High is missing), `frame_num` and POC
  (type 0, 2 per frame) with 8-bit wrap. HEVC Main: IDR_W_RADL then TRAIL_R,
  an explicit one-entry short-term RPS in each slice header, coding tools and
  block sizes from the driver's HEVC attributes. Surfaces follow
  `VASurfaceAttribAlignmentSize` (64x16 for HEVC on radeonsi); the SPS crops
  to the visible size.
- *Reconfigure.* A bitrate change keeps the session (device, context,
  surfaces) and makes new sequence parameters, which the forced keyframe
  carries with the new rate-control buffers. A size or frame-rate change
  closes the session; the next frame opens a new one.
- *Headers.* When the driver takes packed sequence, picture and slice
  headers, the encoder writes them: SPS (and VPS) with the colour
  description, chroma siting, timing, and `max_num_reorder_frames` 0 /
  `max_dec_frame_buffering` 1 so a decoder outputs each picture at once;
  PPS; the slice header. radeonsi (Mesa 26) parses them and writes the NAL
  units itself from what it parsed (it clears `transform_8x8_mode_flag`,
  which its hardware lacks). All three are required there: without a packed
  slice header it writes slices with `nal_unit_type` 0 and no SPS or PPS at
  all. A driver without packed-header support writes every header from the
  parameter buffers. Either way each IDR carries its parameter sets.
- *AV1* (profile 0, the same shape: slot 0 always holds the previous frame;
  sequence and frame headers packed, with the VA bit offsets). On desktop
  content AV1 gains little over HEVC, and radeonsi uses none of AV1's
  screen-content tools. AV1 cannot crop, and VCN 4 encodes the frame at its
  surface alignment (1366x770 as 1408x784, 1080 lines as 1082; a hardware
  limit), keeping the visible size only in `render_size`, which ffmpeg and
  dav1d do not apply: a consumer crops to the size it asked for. radeonsi's
  VA frontend reads only the first OBU of a packed-header buffer, so a
  temporal delimiter there hides the sequence header and the driver divides
  by zero in `vaEndPicture`: the encoder prepends the delimiter to each
  packet itself.
- *Measured* (halo: Radeon 8060S, VCN 4.0.5, Mesa radeonsi), dmabuf to
  packet, mean / p99: 1920x1080 H.264 2.9 / 4.5 ms, HEVC 2.9 / 4.2 ms, AV1
  2.7 / 3.9 ms; 2560x1440 4.4-4.5 / 7 ms.

## Media Foundation decoder (Windows)

The system's synchronous decoder MFT for the codec (`MFTEnumEx`, NV12
output): msmpeg2vdec for H.264, the HEVC and AV1 Video Extensions where
installed. Vendor asynchronous hardware MFTs are not used; the system MFTs do
DXVA themselves once given a device.

- *Hardware.* When the MFT is `MF_SA_D3D11_AWARE` and hardware is allowed,
  a D3D11 device (video support, multithread protected) goes to it through
  an `IMFDXGIDeviceManager`; if it refuses, the decoder runs in software
  (with `Hardware::Require`, creating it fails instead). DXVA pictures are slices of
  the decoder's texture array: for CPU output each is copied to a staging
  texture and read back; for D3D11 output the slice itself is handed over
  with the sample kept alive. Software pictures come as `IMF2DBuffer`s.
  Either way the visible area (the minimum display aperture) is what comes
  out, so a 1080-line picture in a 1088-line surface is 1080 lines.
- *Latency.* `CODECAPI_AVLowLatencyMode` and `MF_LOW_LATENCY` are set. The
  input type carries a placeholder size (the HEVC and AV1 MFTs offer no
  output type without one); the real size arrives as
  `MF_E_TRANSFORM_STREAM_CHANGE` on the first keyframe, and any later size
  change the same way, renegotiating NV12. Some MFTs (the HEVC extension in
  software) still hold each picture until they see the next access unit
  start; when a packet gives no picture, the decoder feeds an access unit
  delimiter (a temporal delimiter for AV1), which hands it over. A drain
  would too, but those MFTs then accept no further input.
- *Sync and errors.* The MFTs conceal rather than fail a predicted frame
  with no reference, so the decoder checks it itself (`is_keyframe`: an
  H.264 IDR, an HEVC IRAP, an AV1 temporal unit with a sequence header and a
  key frame) and returns false (lost sync) for anything before the first
  keyframe and after any failure, which also flushes the MFT. Garbage the
  MFT accepts is not detectable; it shows as corruption until the next
  keyframe.
- *Probe.* A codec is reported, separately in hardware (device "<adapter>
  (D3D11)") and in software ("CPU"), only when a real keyframe of it (a flat
  grey 192x192 picture, embedded) decodes to the right picture that way, once
  per process (about 0.6 s for all three). Why a codec is missing is kept and
  appended to `create_decoder`'s error.
- *Measured* (RTX 4090, Windows 11): H.264, HEVC and AV1 all decode in
  hardware. 1920x1080 to CPU NV12 per frame: H.264 1.9-2.3 ms hardware /
  3.6 ms software, HEVC 1.4-1.6 / 4.1 ms.

## Testing

- `test_raw` (everywhere): the Raw codec round trip with odd sizes and
  padded strides, unchanged frames as single runs, BGRA input; hostile
  packets (no reference, truncation, oversize, overruns, random junk); the
  Encoder contract (release exactly once on every path, forced and requested
  keyframes, reconfigure); the factory and probe; keyframe detection.
- `test_vaapi` (Linux with a VA encoder; skipped, exit 77, otherwise), per
  reported codec, ffmpeg decoding when it is on PATH: a 300-frame CPU
  sequence past the frame_num/POC wraps with forced and requested keyframes,
  decoded whole and from each keyframe alone (frame count, a clean decode,
  every picture compared with its source); BGRA frames; sizes cropped both
  ways; `reconfigure`: a bitrate drop to a quarter on grain (predicted
  packets follow the new budget), a rise, then a size change, each decoded;
  GBM dmabufs from brodmabuf with real sync_file fences: linear, GBM's pick
  of the primary plane's modifiers (DCC on amdgpu), and every modifier the
  plane offers that GBM can allocate; release called once per frame before
  `encode()` returns, with the frame's memory scribbled over in the
  callback. The bar is luma PSNR >= 35 dB against the ideal BT.709 luma: the
  pattern's full-swing colour edges cap RGB PSNR near 33-35 dB before any
  coding (measured as ffmpeg's own uncoded 4:2:0 round trip), so RGB must
  instead stay within 5 dB of that ceiling with under one code value of mean
  error per channel, which catches a wrong matrix, range, transfer or
  channel order. Also measures dmabuf-to-packet latency at 1920x1080 and
  2560x1440. `$BROVIDEO_TEST_CODEC` narrows it to one codec.
- `test_text_quality` (Linux with VA-API, Windows with Media Foundation;
  exit 77 without a hardware encoder; needs ffmpeg), per reported codec at
  1920x1080, 20 Mbit/s: a text page with lines rewritten as broremote sends
  them (frames on change, repeats between them until two packets come in
  under a quarter of the budget), each scenario with and without damage: a
  line at 60, 20 and 10 Hz; a window opening then a line at 20 Hz; 4, 16
  and 48 lines at 20 Hz; 16 lines at 60 Hz; the window alone. It prints the
  luma PSNR of the changing lines (as they land and after) and of the
  static text, the bitrate and the largest packet, and checks that lines
  land sharp (45 dB) where the encoder can, that damage is never more than
  1 dB worse anywhere, that it leaves the window packet and the bitrate of
  fast-changing damage alone, and on VA-API that a line after a window
  lands at least 32 dB and 10 dB better with damage. The bench for the
  Damage numbers above.
- `test_mf` (Windows): with ffmpeg on PATH, the test pattern encoded by
  libx264 / libx265 / libaom (I and P only) at 640x360 then 718x404 in one
  stream, decoded per codec the machine reports: every picture's size, frame
  counter, pts, luma PSNR >= 35 dB, RGB PSNR and colour swatches, in
  hardware and in software; D3D11 texture output read back and compared with
  the CPU picture; starting at a later keyframe (the predicted frames before
  it are lost sync, then every frame decodes); garbage and truncated packets
  mid-stream (no crash; the next keyframe decodes cleanly); and the
  1920x1080 decode time. Without ffmpeg those are skipped. Always: the
  VA-API encoder's own H.264 and HEVC (tests/fixtures/halo_vaapi.*, recorded
  from the halo at 640x360, 3 Mbit/s) decode with every picture matching the
  pattern frame its counter names.
