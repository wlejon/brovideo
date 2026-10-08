// The Media Foundation decoder (Windows), through the factory.
//
// With ffmpeg on PATH, the test pattern is encoded by libx264 / libx265 /
// libaom (I and P only, as the VA-API encoder makes them) at two sizes
// joined into one stream, then decoded: every picture must be the right size,
// carry the right frame counter and pts, and match its source (luma PSNR
// against the ideal BT.709 luma, RGB PSNR, the colour swatches). Also:
// starting at a later keyframe (predicted frames before it are a lost sync),
// garbage in the middle (no crash; the next keyframe recovers), software
// decoding, pictures handed over as D3D11 textures (the same pixels as the
// CPU copies), and the decode time per frame at 1920x1080.
//
// Without ffmpeg those parts are skipped; the committed fixtures recorded
// from the VA-API encoder (tests/fixtures/halo_*) are decoded either way.
#include "brovideo/brovideo.h"
#include "check.h"
#include "elementary.h"
#include "ffmpeg_oracle.h"
#include "picture.h"
#include "test_pattern.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <random>

#include <d3d11.h>
#include <process.h>
#include <wrl/client.h>

using namespace brovideo;
namespace fs = std::filesystem;
namespace tools = testkit;

namespace {

fs::path g_scratch;
bool g_hardware = true;

bool decodes(Codec c) { return supported(c, Direction::Decode); }

void set_hardware(bool on) { g_hardware = on; }

std::unique_ptr<Decoder> make_decoder(Codec c, std::string* err, PictureMemory output = PictureMemory::Cpu) {
    DecoderConfig dc;
    dc.codec = c;
    dc.hardware = g_hardware ? Hardware::Prefer : Hardware::Forbid;
    dc.output = output;
    return create_decoder(dc, err);
}

// ---- fixtures made with ffmpeg ---------------------------------------------------

const char* encoder_args(Codec c) {
    switch (c) {
        case Codec::H264:
            return "-c:v libx264 -preset veryfast -qp 16 -bf 0 -sc_threshold 0 -x264-params repeat-headers=1 -f h264";
        case Codec::HEVC:
            return "-c:v libx265 -preset veryfast -x265-params "
                   "bframes=0:scenecut=0:repeat-headers=1:info=0:log-level=error:qp=16 -f hevc";
        case Codec::AV1:
            return "-c:v libaom-av1 -usage realtime -cpu-used 8 -lag-in-frames 0 -crf 16 -b:v 0 -f obu";
        default: return "";
    }
}

// Pattern frames first .. first+count-1 at w x h, a keyframe every `gop`.
bool encode_segment(Codec c, const fs::path& out, uint32_t w, uint32_t h, uint32_t first, uint32_t count,
                    uint32_t gop) {
    const std::string cmd = "ffmpeg -hide_banner -nostdin -v error -y -f rawvideo -pix_fmt rgba -s " +
                            std::to_string(w) + "x" + std::to_string(h) +
                            " -r 60 -i - -sws_flags bicubic+accurate_rnd+full_chroma_int"
                            " -vf scale=out_color_matrix=bt709:out_range=limited,format=yuv420p"
                            " -color_primaries bt709 -color_trc bt709 -colorspace bt709 -color_range tv -g " +
                            std::to_string(gop) + " -keyint_min " + std::to_string(gop) + " " + encoder_args(c) +
                            " " + oracle::quote(out.string());
    std::FILE* p = popen(cmd.c_str(), oracle::kPipeWrite);
    if (!p) return false;
    std::vector<uint8_t> rgba(size_t(w) * h * 4);
    for (uint32_t n = first; n < first + count; ++n) {
        tools::draw_test_pattern(rgba.data(), w, h, n);
        if (std::fwrite(rgba.data(), 1, rgba.size(), p) != rgba.size()) break;
    }
    return pclose(p) == 0 && fs::exists(out) && fs::file_size(out) > 0;
}

struct Expected {
    uint64_t n;
    uint32_t w, h;
    bool key;
};

struct Fixture {
    std::vector<elementary::Packet> packets;
    std::vector<Expected> expect;
};

struct Segment {
    uint32_t w, h, count;
};

// The segments encoded separately and joined into one stream: a size change
// mid-stream, at a keyframe, as the server makes one.
bool make_fixture(Codec c, const std::string& name, const std::vector<Segment>& segments, uint32_t gop,
                  Fixture& fx) {
    std::vector<uint8_t> all;
    uint32_t first = 0;
    for (size_t i = 0; i < segments.size(); ++i) {
        const Segment& s = segments[i];
        const fs::path part = g_scratch / (name + "." + std::to_string(i));
        if (!encode_segment(c, part, s.w, s.h, first, s.count, gop)) {
            std::printf("   ffmpeg could not encode %s (no %s encoder?)\n", name.c_str(), codec_name(c));
            return false;
        }
        const auto bytes = elementary::read_file(part.string());
        all.insert(all.end(), bytes.begin(), bytes.end());
        for (uint32_t k = 0; k < s.count; ++k) fx.expect.push_back({first + k, s.w, s.h, k % gop == 0});
        first += s.count;
    }
    oracle::write_file((g_scratch / name).string(), all);
    fx.packets = elementary::split(c, all);
    CHECK_EQ(fx.packets.size(), fx.expect.size());
    return fx.packets.size() == fx.expect.size();
}

// ---- checking a picture ----------------------------------------------------------

struct Quality {
    double luma = 99, rgb = 99;
    void take(double l, double r) {
        luma = std::min(luma, l);
        rgb = std::min(rgb, r);
    }
};

double luma_psnr(const Picture& f, const std::vector<uint8_t>& source) {
    const auto ideal = oracle::ideal_luma(source.data(), f.width, f.height);
    std::vector<uint8_t> y(size_t(f.width) * f.height);
    for (uint32_t r = 0; r < f.height; ++r) {
        std::copy_n(f.data.data() + size_t(r) * f.stride, f.width, y.data() + size_t(r) * f.width);
    }
    return oracle::psnr_luma(y.data(), ideal);
}

// The picture is frame `n` of the pattern at w x h; false (with failures
// recorded) when not.
bool check_picture(const Picture& f, uint64_t n, uint32_t w, uint32_t h, Quality& q, double min_luma,
                   double min_rgb) {
    CHECK(f.ready);
    CHECK(f.format == PixelFormat::NV12);
    CHECK_EQ(f.width, w);
    CHECK_EQ(f.height, h);
    if (!f.ready || f.width != w || f.height != h) return false;
    CHECK(f.stride >= w);
    CHECK_EQ(f.uv_offset, f.stride * h);
    CHECK(f.data.size() >= size_t(f.stride) * (h + (h + 1) / 2));
    const auto rgba = tools::to_rgba(f);
    CHECK_EQ(tools::read_test_pattern_counter(rgba.data(), w, h, w * 4), int64_t(n));
    std::vector<uint8_t> source(size_t(w) * h * 4);
    tools::draw_test_pattern(source.data(), w, h, n);
    const double l = luma_psnr(f, source);
    const double r = tools::psnr_rgba(rgba.data(), source.data(), w, h);
    q.take(l, r);
    CHECK(l >= min_luma);
    CHECK(r >= min_rgb);
    // The swatches: pure red, green and blue, so a channel swap shows.
    const uint8_t* red = &rgba[(size_t(8) * w + 8) * 4];
    const uint8_t* green = &rgba[(size_t(8) * w + 24) * 4];
    const uint8_t* blue = &rgba[(size_t(8) * w + 40) * 4];
    CHECK(red[0] > 200 && red[1] < 50 && red[2] < 50);
    CHECK(green[0] < 50 && green[1] > 200 && green[2] < 50);
    CHECK(blue[0] < 50 && blue[1] < 50 && blue[2] > 200);
    return l >= min_luma && r >= min_rgb;
}

// ---- the cases -----------------------------------------------------------------

constexpr double kMinLuma = 35.0;
// The pattern's full-swing colour edges through 4:2:0 cap RGB PSNR near
// 30 dB before any coding with this nearest-neighbour chroma upsampling.
constexpr double kMinRgb = 26.0;

void decode_all(Codec c, const Fixture& fx, bool hardware) {
    check::phase(std::string(codec_name(c)) + (hardware ? ": whole stream, hardware" : ": whole stream, software"));
    set_hardware(hardware);
    std::string err;
    auto d = make_decoder(c, &err);
    CHECK(d != nullptr);
    if (!d) {
        std::printf("   %s\n", err.c_str());
        return;
    }
    std::printf("   %s\n", d->describe().c_str());
    CHECK_EQ(d->hardware(), hardware && best(c, Direction::Decode)->hardware);
    Quality q;
    Picture f;
    for (size_t i = 0; i < fx.packets.size(); ++i) {
        const Expected& e = fx.expect[i];
        const int64_t pts = int64_t(i) * 16666667 + 5;
        const bool ok = d->decode(fx.packets[i], pts, f, &err);
        CHECK(ok);
        if (!ok) {
            std::printf("   frame %zu: %s\n", i, err.c_str());
            continue;
        }
        CHECK_EQ(f.pts_ns, pts);
        CHECK(f.memory == PictureMemory::Cpu);
        check_picture(f, e.n, e.w, e.h, q, kMinLuma, kMinRgb);
    }
    std::printf("   %zu frames, sizes %ux%u then %ux%u; min luma PSNR %.1f dB, min RGB PSNR %.1f dB\n",
                fx.packets.size(), fx.expect.front().w, fx.expect.front().h, fx.expect.back().w, fx.expect.back().h,
                q.luma, q.rgb);
}

// A D3D11 picture's visible NV12 area read back to CPU memory, laid out as
// a CPU Picture would be (Y rows `stride` apart, then UV).
std::vector<uint8_t> read_texture(const Picture& g, uint32_t stride) {
    using Microsoft::WRL::ComPtr;
    auto* dev = static_cast<ID3D11Device*>(g.gpu.device);
    auto* tex = static_cast<ID3D11Texture2D*>(g.gpu.texture);
    D3D11_TEXTURE2D_DESC d{};
    tex->GetDesc(&d);
    D3D11_TEXTURE2D_DESC s{};
    s.Width = d.Width;
    s.Height = d.Height;
    s.MipLevels = 1;
    s.ArraySize = 1;
    s.Format = DXGI_FORMAT_NV12;
    s.SampleDesc.Count = 1;
    s.Usage = D3D11_USAGE_STAGING;
    s.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
    ComPtr<ID3D11Texture2D> staging;
    if (FAILED(dev->CreateTexture2D(&s, nullptr, &staging))) return {};
    ComPtr<ID3D11DeviceContext> ctx;
    dev->GetImmediateContext(&ctx);
    ctx->CopySubresourceRegion(staging.Get(), 0, 0, 0, 0, tex, g.gpu.subresource, nullptr);
    D3D11_MAPPED_SUBRESOURCE m{};
    if (FAILED(ctx->Map(staging.Get(), 0, D3D11_MAP_READ, 0, &m))) return {};
    const auto* y = static_cast<const uint8_t*>(m.pData);
    const uint8_t* uv = y + size_t(m.RowPitch) * d.Height;
    std::vector<uint8_t> out(size_t(stride) * (g.height + (g.height + 1) / 2));
    const size_t x0 = g.gpu.x & ~1u;
    for (uint32_t r = 0; r < g.height; ++r) {
        std::memcpy(out.data() + size_t(r) * stride, y + (size_t(g.gpu.y) + r) * m.RowPitch + x0, g.width);
    }
    for (uint32_t r = 0; r < (g.height + 1) / 2; ++r) {
        std::memcpy(out.data() + size_t(stride) * g.height + size_t(r) * stride,
                    uv + (size_t(g.gpu.y / 2) + r) * m.RowPitch + x0, (g.width + 1) & ~1u);
    }
    ctx->Unmap(staging.Get(), 0);
    return out;
}

// PictureMemory::D3D11: the decoder hands over its texture slices; each
// holds the same pixels as the CPU copy a second decoder makes of the same
// packet, and stays valid while the Picture lives (two are kept at a time).
void gpu_output(Codec c, const Fixture& fx) {
    check::phase(std::string(codec_name(c)) + ": pictures as D3D11 textures");
    const Capability* cap = best(c, Direction::Decode);
    if (!cap || !cap->hardware) {
        std::printf("   no hardware %s decoder: skipped\n", codec_name(c));
        return;
    }
    set_hardware(true);
    std::string err;
    auto gd = make_decoder(c, &err, PictureMemory::D3D11);
    auto cd = make_decoder(c, &err, PictureMemory::Cpu);
    CHECK(gd && cd);
    if (!gd || !cd) return;
    Picture held[2];
    size_t textures = 0, equal = 0;
    for (size_t i = 0; i < fx.packets.size(); ++i) {
        Picture& g = held[i % 2];
        Picture cpu;
        const int64_t pts = int64_t(i) + 1000;
        CHECK(gd->decode(fx.packets[i], pts, g, &err));
        CHECK(cd->decode(fx.packets[i], cpu, &err));
        CHECK(g.ready && cpu.ready);
        if (!g.ready || !cpu.ready) continue;
        CHECK_EQ(g.pts_ns, pts);
        CHECK(g.memory == PictureMemory::D3D11);
        if (g.memory != PictureMemory::D3D11) continue;
        ++textures;
        CHECK(g.gpu.texture != nullptr && g.gpu.device != nullptr && g.gpu.keepalive != nullptr);
        CHECK(g.data.empty());
        CHECK_EQ(g.width, cpu.width);
        CHECK_EQ(g.height, cpu.height);
        CHECK_EQ(g.width, fx.expect[i].w);
        const std::vector<uint8_t> px = read_texture(g, cpu.stride);
        CHECK(!px.empty());
        if (px.size() <= cpu.data.size() && std::equal(px.begin(), px.end(), cpu.data.begin())) ++equal;
        // The picture kept from the frame before is still readable.
        if (i > 0 && held[(i + 1) % 2].ready) CHECK(!read_texture(held[(i + 1) % 2], cpu.stride).empty());
    }
    CHECK_EQ(equal, textures);
    std::printf("   %zu texture pictures, %zu identical to the CPU copies\n", textures, equal);
}

// Joining mid-stream: predicted frames before the first keyframe are a lost
// sync (false, no picture); from the keyframe on, every frame decodes.
void start_late(Codec c, const Fixture& fx) {
    check::phase(std::string(codec_name(c)) + ": start at a later keyframe");
    set_hardware(true);
    std::string err;
    auto d = make_decoder(c, &err);
    CHECK(d != nullptr);
    if (!d) return;
    size_t start = 3;
    size_t key = start;
    while (key < fx.expect.size() && !fx.expect[key].key) ++key;
    CHECK(key < fx.expect.size());
    Quality q;
    Picture f;
    for (size_t i = start; i < fx.packets.size(); ++i) {
        const bool ok = d->decode(fx.packets[i], f, &err);
        if (i < key) {
            CHECK(!ok);
            CHECK(!f.ready);
            if (i == start) std::printf("   before the keyframe: %s\n", err.c_str());
        } else {
            CHECK(ok);
            if (ok) check_picture(f, fx.expect[i].n, fx.expect[i].w, fx.expect[i].h, q, kMinLuma, kMinRgb);
        }
    }
    std::printf("   decoded from frame %zu; min luma PSNR %.1f dB\n", key, q.luma);
}

// Garbage mid-stream must never crash the decoder; whatever it makes of it,
// the next keyframe decodes cleanly again.
void garbage(Codec c, const Fixture& fx) {
    check::phase(std::string(codec_name(c)) + ": garbage, then recovery at a keyframe");
    set_hardware(true);
    std::string err;
    auto d = make_decoder(c, &err);
    CHECK(d != nullptr);
    if (!d) return;
    Picture f;
    for (size_t i = 0; i < 4; ++i) CHECK(d->decode(fx.packets[i], f, &err));
    std::mt19937 rng(7);
    int failures = 0;
    for (int k = 0; k < 20; ++k) {
        std::vector<uint8_t> junk(64 + rng() % 4000);
        for (auto& b : junk) b = uint8_t(rng());
        if (k % 2 == 0 && c != Codec::AV1) {
            junk[0] = 0, junk[1] = 0, junk[2] = 0, junk[3] = 1;  // a start code, then nonsense
        }
        if (!d->decode(junk, f, &err)) ++failures;
    }
    // Truncated real packets.
    for (size_t i = 4; i < 8; ++i) {
        std::vector<uint8_t> half(fx.packets[i].begin(), fx.packets[i].begin() + fx.packets[i].size() / 3);
        if (!d->decode(half, f, &err)) ++failures;
    }
    std::printf("   %d of 24 bad packets reported as failures\n", failures);
    CHECK(d->decode({}, f, &err) == false);
    size_t key = 8;
    while (key < fx.expect.size() && !fx.expect[key].key) ++key;
    Quality q;
    for (size_t i = key; i < fx.packets.size(); ++i) {
        CHECK(d->decode(fx.packets[i], f, &err));
        check_picture(f, fx.expect[i].n, fx.expect[i].w, fx.expect[i].h, q, kMinLuma, kMinRgb);
    }
    std::printf("   recovered at frame %zu; min luma PSNR %.1f dB\n", key, q.luma);
}

void timing(Codec c, bool hardware) {
    check::phase(std::string(codec_name(c)) + ": 1920x1080 decode time" + (hardware ? ", hardware" : ", software"));
    Fixture fx;
    const std::string name = std::string("t1080.") + codec_name(c);
    if (!fs::exists(g_scratch / name)) {
        if (!make_fixture(c, name, {{1920, 1080, 120}}, 120, fx)) return;
    } else {
        fx.packets = elementary::split(c, elementary::read_file((g_scratch / name).string()));
        for (uint32_t k = 0; k < fx.packets.size(); ++k) fx.expect.push_back({k, 1920, 1080, k == 0});
    }
    set_hardware(hardware);
    std::string err;
    auto d = make_decoder(c, &err);
    CHECK(d != nullptr);
    if (!d) return;
    std::vector<double> ms;
    Picture f;
    Quality q;
    for (size_t i = 0; i < fx.packets.size(); ++i) {
        const auto t0 = std::chrono::steady_clock::now();
        const bool ok = d->decode(fx.packets[i], f, &err);
        ms.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
        CHECK(ok && f.ready);
        if (i % 30 == 29) check_picture(f, fx.expect[i].n, 1920, 1080, q, kMinLuma, kMinRgb);
    }
    std::vector<double> rest(ms.begin() + 1, ms.end());
    std::sort(rest.begin(), rest.end());
    double sum = 0;
    for (double v : rest) sum += v;
    std::printf("   %s: first %.2f ms; then mean %.2f, p50 %.2f, p99 %.2f, max %.2f ms per frame (to CPU NV12)\n",
                d->describe().c_str(), ms[0], sum / double(rest.size()), rest[rest.size() / 2],
                rest[size_t(0.99 * double(rest.size() - 1))], rest.back());
}

// ---- the VA-API encoder's own output (recorded on the halo) ---------------------

void halo_fixture(Codec c, const fs::path& path) {
    check::phase(std::string(codec_name(c)) + ": VA-API stream " + path.filename().string());
    const auto bytes = elementary::read_file(path.string());
    CHECK(!bytes.empty());
    const auto packets = elementary::split(c, bytes);
    CHECK(packets.size() >= 10);
    std::string err;
    for (bool hw : {true, false}) {
        set_hardware(hw);
        auto d = make_decoder(c, &err);
        CHECK(d != nullptr);
        if (!d) return;
        Quality q;
        Picture f;
        int64_t last = -1;
        size_t good = 0;
        for (size_t i = 0; i < packets.size(); ++i) {
            const bool ok = d->decode(packets[i], f, &err);
            CHECK(ok && f.ready);
            if (!ok || !f.ready) {
                std::printf("   packet %zu: %s\n", i, err.c_str());
                continue;
            }
            const auto rgba = tools::to_rgba(f);
            const int64_t n = tools::read_test_pattern_counter(rgba.data(), f.width, f.height, f.width * 4);
            CHECK(n > last);  // the counter only moves forward (frames may be skipped, never repeated)
            last = n;
            if (check_picture(f, uint64_t(n), f.width, f.height, q, kMinLuma, kMinRgb)) ++good;
        }
        std::printf("   %s: %zu packets, %ux%u, %zu match the pattern; min luma PSNR %.1f dB, min RGB PSNR %.1f dB\n",
                    d->describe().c_str(), packets.size(), f.width, f.height, good, q.luma, q.rgb);
    }
}

}  // namespace

int main() {
    check::watchdog(600);
    g_scratch = fs::temp_directory_path() / ("brovideo_test_mf_" + std::to_string(_getpid()));
    fs::create_directories(g_scratch);

    check::phase("probe");
    const auto t0 = std::chrono::steady_clock::now();
    const auto& caps = capabilities();
    std::printf("   probed in %.0f ms\n",
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
    for (const Capability& cap : caps) {
        std::printf("   %s %s: %s, %s, %s\n", codec_name(cap.codec),
                    cap.direction == Direction::Encode ? "encode" : "decode", backend_name(cap.backend),
                    cap.hardware ? "hardware" : "software", cap.device.c_str());
    }
    for (Codec c : {Codec::H264, Codec::HEVC, Codec::AV1}) {
        if (!decodes(c)) std::printf("   no %s decoder: %s\n", codec_name(c), unavailable_reason(c, Direction::Decode).c_str());
    }
    CHECK(decodes(Codec::H264));  // every Windows since 7 has the H.264 MFT
    CHECK(decodes(Codec::Raw));
    {
        // Every MF codec decodes in software; Require on a codec with only
        // software fails with a reason.
        std::string err;
        for (Codec c : codecs(Direction::Decode)) {
            if (c == Codec::Raw) continue;
            DecoderConfig dc;
            dc.codec = c;
            dc.hardware = Hardware::Forbid;
            auto d = create_decoder(dc, &err);
            CHECK(d != nullptr);
            if (d) CHECK(!d->hardware());
        }
        DecoderConfig raw_hw;
        raw_hw.codec = Codec::Raw;
        raw_hw.hardware = Hardware::Require;
        CHECK(create_decoder(raw_hw, &err) == nullptr);
        CHECK(!err.empty());
    }

    const bool ffmpeg = oracle::have_ffmpeg();
    if (!ffmpeg) std::printf("   ffmpeg is not on PATH: skipping the generated fixtures\n");
    for (Codec c : {Codec::H264, Codec::HEVC, Codec::AV1}) {
        if (!ffmpeg) break;
        if (!decodes(c)) {
            std::printf("-- %s: no decoder here, skipped\n", codec_name(c));
            continue;
        }
        Fixture fx;
        check::phase(std::string(codec_name(c)) + ": encoding fixtures with ffmpeg");
        if (!make_fixture(c, std::string("mixed.") + codec_name(c), {{640, 360, 30}, {718, 404, 20}}, 10, fx)) {
            continue;
        }
        decode_all(c, fx, true);
        decode_all(c, fx, false);
        gpu_output(c, fx);
        start_late(c, fx);
        garbage(c, fx);
        if (c != Codec::AV1) {
            timing(c, true);
            timing(c, false);
        }
    }

    const fs::path fixtures = BROVIDEO_FIXTURE_DIR;
    for (Codec c : {Codec::H264, Codec::HEVC}) {
        const fs::path p = fixtures / (std::string("halo_vaapi.") + codec_name(c));
        if (!fs::exists(p)) {
            std::printf("-- no %s\n", p.string().c_str());
            continue;
        }
        if (!decodes(c)) {
            std::printf("-- %s: no decoder here, %s skipped\n", codec_name(c), p.filename().string().c_str());
            continue;
        }
        halo_fixture(c, p);
    }
    set_hardware(true);
    std::error_code ec;
    fs::remove_all(g_scratch, ec);
    return check::finish();
}
