// The VA-API hardware decoder (Linux), through the factory.
//
// Tests round-trip VA-API hardware encode -> VA-API hardware decode for H.264,
// HEVC, and AV1 across resolution changes (including non-macroblock-aligned
// 718x404 to exercise SPS cropping and AV1 render_size), CPU NV12 readout vs
// ffmpeg's reference decode, DRM PRIME DMA-BUF surface export
// (PictureMemory::Dmabuf), joining mid-stream before a keyframe, corrupt/garbage
// packet resilience and recovery at the next keyframe, committed fixtures in
// tests/fixtures/halo_vaapi.*, and 1920x1080 decode latency.
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

#include <drm_fourcc.h>
#include <sys/stat.h>
#include <unistd.h>

using namespace brovideo;
namespace fs = std::filesystem;
namespace tools = testkit;

namespace {

constexpr int kSkip = 77;
fs::path g_scratch;

bool decodes(Codec c) { return supported(c, Direction::Decode); }
bool encodes(Codec c) { return supported(c, Direction::Encode); }

std::unique_ptr<Decoder> make_decoder(Codec c, std::string* err, PictureMemory output = PictureMemory::Cpu) {
    DecoderConfig dc;
    dc.codec = c;
    dc.hardware = Hardware::Require;
    dc.output = output;
    return create_decoder(dc, err);
}

std::unique_ptr<Encoder> make_encoder(Codec c, uint32_t w, uint32_t h, uint32_t kbps, std::string* err) {
    EncoderConfig ec;
    ec.codec = c;
    ec.width = w;
    ec.height = h;
    ec.bitrate_kbps = kbps;
    ec.allow_software = false;
    return create_encoder(ec, err);
}

struct Expected {
    uint64_t n;
    uint32_t w, h;
    bool key;
};

struct Fixture {
    std::vector<std::vector<uint8_t>> packets;
    std::vector<Expected> expect;
};

struct Segment {
    uint32_t w, h, count;
};

// Encode segments using the VA-API hardware encoder and join them into one stream.
bool make_vaapi_fixture(Codec c, const std::vector<Segment>& segments, uint32_t gop, uint32_t kbps, Fixture& fx) {
    fx.packets.clear();
    fx.expect.clear();
    uint32_t first = 0;
    for (const Segment& s : segments) {
        std::string err;
        auto enc = make_encoder(c, s.w, s.h, kbps, &err);
        CHECK(enc != nullptr);
        if (!enc) {
            std::printf("   cannot create %s encoder: %s\n", codec_name(c), err.c_str());
            return false;
        }
        std::vector<uint8_t> rgba(size_t(s.w) * s.h * 4);
        for (uint32_t k = 0; k < s.count; ++k) {
            const uint64_t n = first + k;
            tools::draw_test_pattern(rgba.data(), s.w, s.h, n);
            Frame f;
            f.width = s.w;
            f.height = s.h;
            f.cpu = rgba.data();
            f.pts_ns = int64_t(n) * 16666667;
            Packet pkt;
            const bool ok = enc->encode(f, k % gop == 0, {}, pkt, &err);
            CHECK(ok);
            if (!ok) {
                std::printf("   encode %s frame %llu failed: %s\n", codec_name(c),
                            static_cast<unsigned long long>(n), err.c_str());
                return false;
            }
            CHECK_EQ(pkt.keyframe, (k % gop == 0));
            fx.packets.push_back(std::move(pkt.data));
            fx.expect.push_back({n, s.w, s.h, k % gop == 0});
        }
        first += s.count;
    }
    return true;
}

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

constexpr double kMinLuma = 34.0;
constexpr double kMinRgb = 25.5;

bool check_picture(const Picture& f, uint64_t n, uint32_t w, uint32_t h, Quality& q, double min_luma = kMinLuma,
                   double min_rgb = kMinRgb) {
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
    const uint8_t* red = &rgba[(size_t(8) * w + 8) * 4];
    const uint8_t* green = &rgba[(size_t(8) * w + 24) * 4];
    const uint8_t* blue = &rgba[(size_t(8) * w + 40) * 4];
    CHECK(red[0] > 200 && red[1] < 50 && red[2] < 50);
    CHECK(green[0] < 50 && green[1] > 200 && green[2] < 50);
    CHECK(blue[0] < 50 && blue[1] < 50 && blue[2] > 200);
    return l >= min_luma && r >= min_rgb;
}

void decode_all(Codec c, const Fixture& fx) {
    check::phase(std::string(codec_name(c)) + ": VA-API encode -> VA-API decode (CPU NV12 + size change)");
    std::string err;
    auto d = make_decoder(c, &err, PictureMemory::Cpu);
    CHECK(d != nullptr);
    if (!d) {
        std::printf("   %s\n", err.c_str());
        return;
    }
    std::printf("   %s\n", d->describe().c_str());
    CHECK(d->hardware());
    Quality q;
    Picture f;
    for (size_t i = 0; i < fx.packets.size(); ++i) {
        const Expected& e = fx.expect[i];
        const int64_t pts = int64_t(i) * 16666667 + 7;
        const bool ok = d->decode(fx.packets[i], pts, f, &err);
        CHECK(ok);
        if (!ok) {
            std::printf("   frame %zu: %s\n", i, err.c_str());
            continue;
        }
        CHECK_EQ(f.pts_ns, pts);
        CHECK(f.memory == PictureMemory::Cpu);
        check_picture(f, e.n, e.w, e.h, q);
    }
    std::printf("   %zu frames, sizes %ux%u then %ux%u; min luma PSNR %.1f dB, min RGB PSNR %.1f dB\n",
                fx.packets.size(), fx.expect.front().w, fx.expect.front().h, fx.expect.back().w, fx.expect.back().h,
                q.luma, q.rgb);
}

// Compare VA-API hardware decoder output against ffmpeg's software decoder on the same bitstream.
void compare_with_ffmpeg(Codec c, const Fixture& fx) {
    if (!oracle::have_ffmpeg()) return;
    check::phase(std::string(codec_name(c)) + ": VA-API decode vs ffmpeg reference luma");
    std::vector<uint8_t> seg0;
    uint32_t w0 = fx.expect[0].w, h0 = fx.expect[0].h;
    size_t count0 = 0;
    for (size_t i = 0; i < fx.packets.size() && fx.expect[i].w == w0 && fx.expect[i].h == h0; ++i) {
        seg0.insert(seg0.end(), fx.packets[i].begin(), fx.packets[i].end());
        ++count0;
    }
    const std::string path = (g_scratch / (std::string("ref.") + codec_name(c))).string();
    oracle::write_file(path, seg0);
    std::vector<std::vector<uint8_t>> ff_luma;
    std::string log;
    CHECK(oracle::decode(path, c, w0, h0, ff_luma, log, true));
    CHECK_EQ(ff_luma.size(), count0);
    if (ff_luma.size() != count0) return;

    std::string err;
    auto d = make_decoder(c, &err, PictureMemory::Cpu);
    CHECK(d != nullptr);
    if (!d) return;
    double min_psnr_vs_ff = 99.0;
    Picture f;
    for (size_t i = 0; i < count0; ++i) {
        CHECK(d->decode(fx.packets[i], f, &err));
        std::vector<double> ref_y(size_t(w0) * h0);
        for (size_t k = 0; k < ref_y.size(); ++k) ref_y[k] = double(ff_luma[i][k]);
        std::vector<uint8_t> va_y(size_t(w0) * h0);
        for (uint32_t r = 0; r < h0; ++r) {
            std::copy_n(f.data.data() + size_t(r) * f.stride, w0, va_y.data() + size_t(r) * w0);
        }
        min_psnr_vs_ff = std::min(min_psnr_vs_ff, oracle::psnr_luma(va_y.data(), ref_y));
    }
    std::printf("   %zu frames compared vs ffmpeg: min luma PSNR %.1f dB\n", count0, min_psnr_vs_ff);
    CHECK(min_psnr_vs_ff >= 60.0);  // bit-exact or within 1 LSB rounding
}

// PictureMemory::Dmabuf: verify exported DRM PRIME fds, plane layout, and keepalive lifetime.
void dmabuf_output(Codec c, const Fixture& fx) {
    check::phase(std::string(codec_name(c)) + ": pictures as exported DRM PRIME DMA-BUFs");
    std::string err;
    auto gd = make_decoder(c, &err, PictureMemory::Dmabuf);
    auto cd = make_decoder(c, &err, PictureMemory::Cpu);
    CHECK(gd && cd);
    if (!gd || !cd) return;

    Picture held[2];
    size_t exported = 0;
    for (size_t i = 0; i < fx.packets.size(); ++i) {
        Picture& g = held[i % 2];
        Picture cpu;
        const int64_t pts = int64_t(i) + 2000;
        CHECK(gd->decode(fx.packets[i], pts, g, &err));
        CHECK(cd->decode(fx.packets[i], cpu, &err));
        CHECK(g.ready && cpu.ready);
        if (!g.ready || !cpu.ready) continue;
        CHECK_EQ(g.pts_ns, pts);
        CHECK(g.memory == PictureMemory::Dmabuf);
        CHECK(g.data.empty());
        CHECK_EQ(g.width, cpu.width);
        CHECK_EQ(g.height, cpu.height);
        CHECK_EQ(g.width, fx.expect[i].w);
        CHECK_EQ(g.height, fx.expect[i].h);
        CHECK_EQ(g.gpu.drm_format, uint32_t(DRM_FORMAT_NV12));
        CHECK(g.gpu.modifier != DRM_FORMAT_MOD_INVALID);
        CHECK(g.gpu.coded_width >= g.width);
        CHECK(g.gpu.coded_height >= g.height);
        CHECK(g.gpu.plane_count >= 2 && g.gpu.plane_count <= 4);
        CHECK(g.gpu.keepalive != nullptr);
        for (uint32_t p = 0; p < 2; ++p) {
            CHECK(g.gpu.planes[p].fd >= 0);
            CHECK(g.gpu.planes[p].pitch >= (p == 0 ? g.width : (g.width & ~1u)));
            struct stat st{};
            CHECK_EQ(::fstat(g.gpu.planes[p].fd, &st), 0);
        }
        // Verify the previously held picture's DMA-BUF fds are still open and valid.
        if (i > 0 && held[(i + 1) % 2].ready) {
            struct stat st_prev{};
            CHECK_EQ(::fstat(held[(i + 1) % 2].gpu.planes[0].fd, &st_prev), 0);
        }
        ++exported;
    }
    CHECK_EQ(exported, fx.packets.size());
    std::printf("   %zu DMA-BUF pictures exported (modifier 0x%016llx, %u planes)\n", exported,
                static_cast<unsigned long long>(held[0].gpu.modifier), held[0].gpu.plane_count);
}

void start_late(Codec c, const Fixture& fx) {
    check::phase(std::string(codec_name(c)) + ": start at a later keyframe");
    std::string err;
    auto d = make_decoder(c, &err);
    CHECK(d != nullptr);
    if (!d) return;
    const size_t start = 3;
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
            if (ok) check_picture(f, fx.expect[i].n, fx.expect[i].w, fx.expect[i].h, q);
        }
    }
    std::printf("   decoded from frame %zu; min luma PSNR %.1f dB\n", key, q.luma);
}

void garbage(Codec c, const Fixture& fx) {
    check::phase(std::string(codec_name(c)) + ": garbage, then recovery at a keyframe");
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
            junk[0] = 0, junk[1] = 0, junk[2] = 0, junk[3] = 1;
        }
        if (!d->decode(junk, f, &err)) ++failures;
    }
    for (size_t i = 4; i < 8; ++i) {
        std::vector<uint8_t> half(fx.packets[i].begin(), fx.packets[i].begin() + fx.packets[i].size() / 3);
        if (!d->decode(half, f, &err)) ++failures;
    }
    std::printf("   %d of 24 bad packets reported as failures\n", failures);
    CHECK(!d->decode({}, f, &err));
    size_t key = 10;
    while (key < fx.expect.size() && !fx.expect[key].key) ++key;
    Quality q;
    for (size_t i = key; i < fx.packets.size(); ++i) {
        CHECK(d->decode(fx.packets[i], f, &err));
        check_picture(f, fx.expect[i].n, fx.expect[i].w, fx.expect[i].h, q);
    }
    std::printf("   recovered at frame %zu; min luma PSNR %.1f dB\n", key, q.luma);
}

void halo_fixture(Codec c, const fs::path& path) {
    check::phase(std::string(codec_name(c)) + ": recorded VA-API stream " + path.filename().string());
    const auto bytes = elementary::read_file(path.string());
    CHECK(!bytes.empty());
    const auto packets = elementary::split(c, bytes);
    CHECK(packets.size() >= 10);
    std::string err;
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
        CHECK(n > last);
        last = n;
        if (check_picture(f, uint64_t(n), f.width, f.height, q)) ++good;
    }
    CHECK_EQ(good, packets.size());
    std::printf("   %s: %zu packets, %ux%u, %zu match the pattern; min luma PSNR %.1f dB, min RGB PSNR %.1f dB\n",
                d->describe().c_str(), packets.size(), f.width, f.height, good, q.luma, q.rgb);
}

void timing(Codec c) {
    check::phase(std::string(codec_name(c)) + ": 1920x1080 VA-API decode latency");
    Fixture fx;
    if (!make_vaapi_fixture(c, {{1920, 1080, 120}}, 120, 20000, fx)) return;
    for (PictureMemory mem : {PictureMemory::Cpu, PictureMemory::Dmabuf}) {
        std::string err;
        auto d = make_decoder(c, &err, mem);
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
            if (mem == PictureMemory::Cpu && i % 30 == 29) check_picture(f, fx.expect[i].n, 1920, 1080, q);
        }
        std::vector<double> rest(ms.begin() + 1, ms.end());
        std::sort(rest.begin(), rest.end());
        double sum = 0;
        for (double v : rest) sum += v;
        const double mean = sum / double(rest.size());
        std::printf("   %s (%s): first %.2f ms; mean %.2f, p50 %.2f, p99 %.2f, max %.2f ms\n",
                    d->describe().c_str(), mem == PictureMemory::Cpu ? "CPU NV12" : "DMA-BUF", ms[0], mean,
                    rest[rest.size() / 2], rest[size_t(0.99 * double(rest.size() - 1))], rest.back());
        CHECK(mean < 16.6);
    }
}

}  // namespace

int main() {
    check::watchdog(600);
    g_scratch = fs::temp_directory_path() / ("brovideo_test_vaapi_dec_" + std::to_string(::getpid()));
    fs::create_directories(g_scratch);

    check::phase("probe decode capabilities");
    std::vector<Codec> codecs_here;
    for (const Capability& cap : capabilities()) {
        std::printf("   %s %s: %s, %s, %ux%u, %s\n", codec_name(cap.codec),
                    cap.direction == Direction::Encode ? "encode" : "decode", backend_name(cap.backend),
                    cap.hardware ? "hardware" : "software", cap.max_width, cap.max_height, cap.device.c_str());
        if (cap.backend == Backend::VaApi && cap.direction == Direction::Decode) {
            if (std::find(codecs_here.begin(), codecs_here.end(), cap.codec) == codecs_here.end()) {
                codecs_here.push_back(cap.codec);
            }
        }
    }
    if (codecs_here.empty()) {
        std::printf("SKIP: no VA-API decoder on this machine\n");
        return kSkip;
    }

    for (Codec c : {Codec::H264, Codec::HEVC, Codec::AV1}) {
        CHECK(decodes(c));
        const Capability* b = best(c, Direction::Decode);
        CHECK(b != nullptr && b->backend == Backend::VaApi && b->hardware);
        std::string err;
        DecoderConfig sw;
        sw.codec = c;
        sw.hardware = Hardware::Forbid;
        CHECK(create_decoder(sw, &err) == nullptr);
        CHECK(!err.empty());
        DecoderConfig d3d;
        d3d.codec = c;
        d3d.output = PictureMemory::D3D11;
        CHECK(create_decoder(d3d, &err) == nullptr);
    }

    for (Codec c : codecs_here) {
        if (!encodes(c)) continue;
        Fixture fx;
        check::phase(std::string(codec_name(c)) + ": encoding multi-size fixture with VA-API");
        if (!make_vaapi_fixture(c, {{640, 360, 30}, {718, 404, 20}}, 10, 8000, fx)) continue;
        decode_all(c, fx);
        compare_with_ffmpeg(c, fx);
        dmabuf_output(c, fx);
        start_late(c, fx);
        garbage(c, fx);
        timing(c);
    }

#ifdef BROVIDEO_FIXTURE_DIR
    const fs::path fixtures = BROVIDEO_FIXTURE_DIR;
    for (Codec c : {Codec::H264, Codec::HEVC}) {
        const fs::path p = fixtures / (std::string("halo_vaapi.") + codec_name(c));
        if (fs::exists(p) && decodes(c)) halo_fixture(c, p);
    }
#endif

    std::error_code ec;
    fs::remove_all(g_scratch, ec);
    return check::finish();
}
