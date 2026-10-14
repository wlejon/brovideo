// The Media Foundation hardware encoder (Windows), round-tripped through the
// Media Foundation decoder (and ffmpeg when on PATH).
//
// For every codec the machine encodes (H.264, HEVC, AV1):
//   - Probe verification: Direction::Encode reported with Backend::MediaFoundation
//     and hardware == true.
//   - RGBA8 CPU frames: a 120-frame pattern sequence with forced and requested
//     keyframes, release called once per frame before encode() returns (with
//     the caller's buffer scribbled over inside release), decoded whole and
//     from each keyframe segment alone.
//   - BGRA8 and NV12 CPU frames (including padded strides), plus scaled input.
//   - Non-macroblock-aligned dimensions (718x404, 1366x770).
//   - encode_repeat(): sharpening after a scene change, and a keyframe repeat
//     decoded alone by a fresh decoder.
//   - reconfigure(): live bitrate drop and rise via ICodecAPI without tearing
//     down the session, followed by a resolution change.
//   - Per-frame encode time at 1920x1080.
#include "brovideo/brovideo.h"
#include "check.h"
#include "ffmpeg_oracle.h"
#include "picture.h"
#include "test_pattern.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <set>
#include <string>
#include <utility>
#include <vector>

using namespace brovideo;
namespace tools = testkit;

namespace {

constexpr double kMinLuma = 35.0;
constexpr double kMinRgb = 26.0;
constexpr int kSkip = 77;

std::vector<uint8_t> pattern(uint32_t w, uint32_t h, uint64_t n) {
    std::vector<uint8_t> px(size_t(w) * h * 4);
    tools::draw_test_pattern(px.data(), w, h, n);
    return px;
}

std::unique_ptr<Encoder> make_encoder(Codec c, uint32_t w, uint32_t h, uint32_t kbps, std::string* err) {
    EncoderConfig ec;
    ec.codec = c;
    ec.width = w;
    ec.height = h;
    ec.fps = 60;
    ec.bitrate_kbps = kbps;
    ec.allow_software = false;
    return create_encoder(ec, err);
}

std::unique_ptr<Decoder> make_decoder(Codec c, std::string* err, bool hardware = true) {
    DecoderConfig dc;
    dc.codec = c;
    dc.hardware = hardware ? Hardware::Prefer : Hardware::Forbid;
    dc.output = PictureMemory::Cpu;
    return create_decoder(dc, err);
}

struct Quality {
    double luma = 99.0;
    double rgb = 99.0;
    void take(double l, double r) {
        luma = std::min(luma, l);
        rgb = std::min(rgb, r);
    }
};

// Crops `pic` in place to (w, h) if an encoder padded an unaligned size (e.g.
// AV1 or hardware surface alignment without conformance crop).
void crop_if_padded(Codec c, Picture& pic, uint32_t w, uint32_t h) {
    if (c == Codec::AV1 && pic.width >= w && pic.height >= h && (pic.width != w || pic.height != h)) {
        pic.width = w;
        pic.height = h;
    }
}

bool check_picture(Codec c, Picture& f, uint64_t n, uint32_t w, uint32_t h, Quality& q) {
    CHECK(f.ready);
    CHECK(f.format == PixelFormat::NV12);
    crop_if_padded(c, f, w, h);
    CHECK_EQ(f.width, w);
    CHECK_EQ(f.height, h);
    if (!f.ready || f.width != w || f.height != h) return false;

    const auto rgba = tools::to_rgba(f);
    CHECK_EQ(tools::read_test_pattern_counter(rgba.data(), w, h, w * 4), int64_t(n));
    const auto src = pattern(w, h, n);
    const double l = tools::psnr_luma(f, src.data());
    const double r = tools::psnr_rgba(rgba.data(), src.data(), w, h);
    q.take(l, r);
    CHECK(l >= kMinLuma);
    CHECK(r >= kMinRgb);
    if (l < kMinLuma || r < kMinRgb) {
        std::printf("   frame %llu (%ux%u): luma %.2f dB, RGB %.2f dB\n", static_cast<unsigned long long>(n), w, h, l,
                    r);
    }
    const uint8_t* red = &rgba[(size_t(8) * w + 8) * 4];
    const uint8_t* green = &rgba[(size_t(8) * w + 24) * 4];
    const uint8_t* blue = &rgba[(size_t(8) * w + 40) * 4];
    CHECK(red[0] > 200 && red[1] < 50 && red[2] < 50);
    CHECK(green[0] < 50 && green[1] > 200 && green[2] < 50);
    CHECK(blue[0] < 50 && blue[1] < 50 && blue[2] > 200);
    return l >= kMinLuma && r >= kMinRgb;
}

void verify_stream(Codec c, const std::vector<Packet>& pkts, const std::vector<uint64_t>& source, uint32_t w,
                   uint32_t h, const std::string& label) {
    std::string err;
    auto dec = make_decoder(c, &err, true);
    CHECK(dec != nullptr);
    if (!dec) {
        std::printf("   %s: create_decoder failed: %s\n", label.c_str(), err.c_str());
        return;
    }
    Quality q;
    Picture pic;
    for (size_t i = 0; i < pkts.size(); ++i) {
        const bool ok = dec->decode(pkts[i].data, pkts[i].pts_ns, pic, &err);
        CHECK(ok && pic.ready);
        if (!ok || !pic.ready) {
            std::printf("   %s: packet %zu decode failed: %s\n", label.c_str(), i, err.c_str());
            return;
        }
        CHECK_EQ(pic.pts_ns, pkts[i].pts_ns);
        check_picture(c, pic, source[i], w, h, q);
    }

    // Each keyframe segment must also decode cleanly on its own in a fresh decoder.
    std::vector<size_t> keys;
    for (size_t i = 0; i < pkts.size(); ++i) {
        if (pkts[i].keyframe) keys.push_back(i);
    }
    for (size_t k = 0; k < keys.size(); ++k) {
        const size_t end = k + 1 < keys.size() ? keys[k + 1] : pkts.size();
        auto seg_dec = make_decoder(c, &err, true);
        CHECK(seg_dec != nullptr);
        if (!seg_dec) break;
        for (size_t i = keys[k]; i < end; ++i) {
            CHECK(seg_dec->decode(pkts[i].data, pkts[i].pts_ns, pic, &err));
            check_picture(c, pic, source[i], w, h, q);
        }
    }
    std::printf("   %s: %zu packets (%zu keyframes), %ux%u; min luma PSNR %.1f dB, min RGB PSNR %.1f dB\n",
                label.c_str(), pkts.size(), keys.size(), w, h, q.luma, q.rgb);
}

void test_rgba_sequence(Codec c) {
    check::phase(std::string(codec_name(c)) + ": RGBA8 sequence, forced and requested keyframes");
    const uint32_t w = 1280, h = 720, n_frames = 120;
    const std::set<uint32_t> forced = {40, 80, 81};
    const uint32_t requested = 100;
    std::string err;
    auto enc = make_encoder(c, w, h, 12000, &err);
    CHECK(enc != nullptr);
    if (!enc) {
        std::printf("   create_encoder: %s\n", err.c_str());
        return;
    }
    const EncoderInfo info = enc->info();
    CHECK(info.backend == Backend::MediaFoundation);
    CHECK(info.hardware);
    std::printf("   %s\n", info.description.c_str());

    std::vector<Packet> pkts;
    std::vector<uint64_t> source;
    std::vector<uint8_t> px(size_t(w) * h * 4);
    for (uint32_t n = 0; n < n_frames; ++n) {
        tools::draw_test_pattern(px.data(), w, h, n);
        Frame f;
        f.width = w;
        f.height = h;
        f.cpu = px.data();
        f.cpu_format = PixelFormat::RGBA8;
        f.pts_ns = int64_t(n) * 16666667 + 123;
        int releases = 0;
        const auto release = [&] {
            ++releases;
            std::memset(px.data(), 0x5a, px.size());
        };
        if (n == requested) enc->request_keyframe();
        Packet pkt;
        const bool ok = enc->encode(f, forced.count(n) > 0, release, pkt, &err);
        CHECK(ok);
        CHECK_EQ(releases, 1);
        if (!ok) {
            std::printf("   frame %u: %s\n", n, err.c_str());
            return;
        }
        const bool expect_key = (n == 0 || forced.count(n) > 0 || n == requested);
        CHECK_EQ(pkt.keyframe, expect_key);
        CHECK_EQ(is_keyframe(c, pkt.data), pkt.keyframe);
        CHECK(!pkt.data.empty());
        CHECK_EQ(pkt.pts_ns, f.pts_ns);
        pkts.push_back(std::move(pkt));
        source.push_back(n);
    }
    verify_stream(c, pkts, source, w, h, std::string(codec_name(c)) + "-rgba");
}

void test_bgra_and_nv12(Codec c) {
    check::phase(std::string(codec_name(c)) + ": BGRA8 and NV12 inputs (with padded strides)");
    const uint32_t w = 640, h = 360;
    std::string err;

    // 1. BGRA8 with padded stride on odd frames.
    {
        auto enc = make_encoder(c, w, h, 8000, &err);
        CHECK(enc != nullptr);
        if (!enc) return;
        std::vector<Packet> pkts;
        std::vector<uint64_t> source;
        for (uint32_t n = 0; n < 30; ++n) {
            auto px = pattern(w, h, n + 500);
            for (size_t i = 0; i < px.size(); i += 4) std::swap(px[i], px[i + 2]);
            const uint32_t stride = (n & 1) ? w * 4 + 48 : 0;
            std::vector<uint8_t> padded;
            const uint8_t* cpu = px.data();
            if (stride) {
                padded.assign(size_t(stride) * h, 0xCC);
                for (uint32_t y = 0; y < h; ++y) {
                    std::memcpy(padded.data() + size_t(y) * stride, px.data() + size_t(y) * w * 4, w * 4);
                }
                cpu = padded.data();
            }
            Frame f;
            f.width = w;
            f.height = h;
            f.cpu = cpu;
            f.cpu_stride = stride;
            f.cpu_format = PixelFormat::BGRA8;
            f.pts_ns = int64_t(n) * 1000;
            Packet pkt;
            CHECK(enc->encode(f, n == 15, {}, pkt, &err));
            pkts.push_back(std::move(pkt));
            source.push_back(n + 500);
        }
        verify_stream(c, pkts, source, w, h, std::string(codec_name(c)) + "-bgra");
    }

    // 2. NV12 input (tightly packed on even frames, padded stride on odd frames).
    {
        auto enc = make_encoder(c, w, h, 8000, &err);
        CHECK(enc != nullptr);
        if (!enc) return;
        std::vector<Packet> pkts;
        std::vector<uint64_t> source;
        for (uint32_t n = 0; n < 30; ++n) {
            const auto rgba = pattern(w, h, n + 800);
            const uint32_t pitch = (n & 1) ? w + 64 : w;
            std::vector<uint8_t> nv12(size_t(pitch) * h * 3 / 2, 0xEE);
            uint8_t* dst_y = nv12.data();
            uint8_t* dst_uv = nv12.data() + size_t(pitch) * h;
            for (uint32_t y = 0; y < h; y += 2) {
                for (uint32_t x = 0; x < w; x += 2) {
                    const uint8_t* p00 = &rgba[(size_t(y) * w + x) * 4];
                    const uint8_t* p01 = p00 + 4;
                    const uint8_t* p10 = &rgba[(size_t(y + 1) * w + x) * 4];
                    const uint8_t* p11 = p10 + 4;
                    dst_y[size_t(y) * pitch + x] =
                        uint8_t(16 + ((11966 * p00[0] + 40254 * p00[1] + 4063 * p00[2] + 32768) >> 16));
                    dst_y[size_t(y) * pitch + x + 1] =
                        uint8_t(16 + ((11966 * p01[0] + 40254 * p01[1] + 4063 * p01[2] + 32768) >> 16));
                    dst_y[size_t(y + 1) * pitch + x] =
                        uint8_t(16 + ((11966 * p10[0] + 40254 * p10[1] + 4063 * p10[2] + 32768) >> 16));
                    dst_y[size_t(y + 1) * pitch + x + 1] =
                        uint8_t(16 + ((11966 * p11[0] + 40254 * p11[1] + 4063 * p11[2] + 32768) >> 16));
                    const int sr = p00[0] + p01[0] + p10[0] + p11[0];
                    const int sg = p00[1] + p01[1] + p10[1] + p11[1];
                    const int sb = p00[2] + p01[2] + p10[2] + p11[2];
                    dst_uv[size_t(y / 2) * pitch + x] =
                        uint8_t(std::clamp(128 + ((-1650 * sr - 5549 * sg + 7199 * sb + 32768) >> 16), 16, 240));
                    dst_uv[size_t(y / 2) * pitch + x + 1] =
                        uint8_t(std::clamp(128 + ((7199 * sr - 6539 * sg - 660 * sb + 32768) >> 16), 16, 240));
                }
            }
            Frame f;
            f.width = w;
            f.height = h;
            f.cpu = nv12.data();
            f.cpu_stride = (n & 1) ? pitch : 0;
            f.cpu_format = PixelFormat::NV12;
            f.pts_ns = int64_t(n) * 2000;
            Packet pkt;
            CHECK(enc->encode(f, n == 10, {}, pkt, &err));
            pkts.push_back(std::move(pkt));
            source.push_back(n + 800);
        }
        verify_stream(c, pkts, source, w, h, std::string(codec_name(c)) + "-nv12");
    }
}

void test_odd_dimensions(Codec c) {
    check::phase(std::string(codec_name(c)) + ": non-macroblock-aligned sizes (718x404, 1366x770)");
    for (auto [w, h] : {std::pair<uint32_t, uint32_t>{718, 404}, {1366, 770}}) {
        std::string err;
        auto enc = make_encoder(c, w, h, 12000, &err);
        CHECK(enc != nullptr);
        if (!enc) {
            std::printf("   %ux%u: %s\n", w, h, err.c_str());
            return;
        }
        std::vector<Packet> pkts;
        std::vector<uint64_t> source;
        for (uint32_t n = 0; n < 30; ++n) {
            auto px = pattern(w, h, n + 1000);
            Frame f;
            f.width = w;
            f.height = h;
            f.cpu = px.data();
            f.pts_ns = int64_t(n) * 16666667;
            int releases = 0;
            Packet pkt;
            CHECK(enc->encode(f, n == 15, [&] { ++releases; }, pkt, &err));
            CHECK_EQ(releases, 1);
            pkts.push_back(std::move(pkt));
            source.push_back(n + 1000);
        }
        verify_stream(c, pkts, source, w, h,
                      std::string(codec_name(c)) + "-" + std::to_string(w) + "x" + std::to_string(h));
    }
}

double mean_kb(const std::vector<Packet>& pkts, size_t from, size_t to) {
    double sum = 0;
    for (size_t i = from; i < to; ++i) sum += double(pkts[i].data.size()) / 1000.0;
    return to > from ? sum / double(to - from) : 0;
}

void test_reconfigure(Codec c) {
    check::phase(std::string(codec_name(c)) + ": reconfigure bitrate live, then resolution");
    const uint32_t w = 1280, h = 720;
    std::string err;
    auto enc = make_encoder(c, w, h, 8000, &err);
    CHECK(enc != nullptr);
    if (!enc) return;

    std::vector<Packet> pkts;
    uint32_t rng = 1;
    auto encode_run = [&](uint32_t fw, uint32_t fh, uint32_t first, uint32_t count, bool noise) {
        for (uint32_t n = first; n < first + count; ++n) {
            auto px = pattern(fw, fh, n);
            if (noise) {
                for (auto& b : px) b = uint8_t(112 + ((rng = rng * 1664525u + 1013904223u) >> 27));
            }
            Frame f;
            f.width = fw;
            f.height = fh;
            f.cpu = px.data();
            f.pts_ns = int64_t(pkts.size()) * 16666667;
            Packet pkt;
            CHECK(enc->encode(f, false, {}, pkt, &err));
            pkts.push_back(std::move(pkt));
        }
    };

    encode_run(w, h, 0, 60, true);
    EncoderConfig cfg = enc->config();
    cfg.bitrate_kbps = 2000;
    CHECK(enc->reconfigure(cfg, &err));
    CHECK(!enc->can_repeat());
    encode_run(w, h, 60, 60, true);
    CHECK(pkts[60].keyframe);
    for (size_t i = 61; i < pkts.size(); ++i) CHECK(!pkts[i].keyframe);
    const double high_kb = mean_kb(pkts, 30, 60), low_kb = mean_kb(pkts, 90, 120);
    std::printf("   noise, predicted packets: %.1f kB at 8 Mbit/s, %.1f kB at 2 Mbit/s\n", high_kb, low_kb);
    CHECK(low_kb < high_kb / 2.0);

    cfg.bitrate_kbps = 12000;
    CHECK(enc->reconfigure(cfg, &err));
    size_t at = pkts.size();
    encode_run(w, h, 3000, 30, false);
    CHECK(pkts[at].keyframe);
    {
        std::vector<Packet> tail(pkts.begin() + std::ptrdiff_t(at), pkts.end());
        std::vector<uint64_t> source(tail.size());
        for (size_t i = 0; i < source.size(); ++i) source[i] = 3000 + i;
        verify_stream(c, tail, source, w, h, std::string(codec_name(c)) + "-rebitrate");
    }

    const uint32_t w2 = 718, h2 = 404;
    EncoderConfig resized = enc->config();
    resized.width = w2;
    resized.height = h2;
    CHECK(enc->reconfigure(resized, &err));
    at = pkts.size();
    encode_run(w2, h2, 2000, 30, false);
    CHECK(pkts[at].keyframe);
    {
        std::vector<Packet> tail(pkts.begin() + std::ptrdiff_t(at), pkts.end());
        std::vector<uint64_t> source(tail.size());
        for (size_t i = 0; i < source.size(); ++i) source[i] = 2000 + i;
        verify_stream(c, tail, source, w2, h2, std::string(codec_name(c)) + "-resized");
    }
}

void test_repeat(Codec c) {
    check::phase(std::string(codec_name(c)) + ": encode_repeat");
    const uint32_t w = 1280, h = 720;
    std::string err;
    auto enc = make_encoder(c, w, h, 12000, &err);
    auto dec = make_decoder(c, &err, true);
    CHECK(enc && dec);
    if (!enc || !dec) return;

    Packet pkt;
    CHECK(!enc->can_repeat());
    CHECK(!enc->encode_repeat(false, pkt, &err));
    CHECK(err.empty());

    const auto src = pattern(w, h, 42);
    Frame f;
    f.width = w;
    f.height = h;
    f.cpu = src.data();
    f.pts_ns = 777777;
    CHECK(enc->encode(f, false, {}, pkt, &err));
    CHECK(enc->can_repeat());

    Picture pic;
    CHECK(dec->decode(pkt.data, pkt.pts_ns, pic, &err));
    Quality q0;
    check_picture(c, pic, 42, w, h, q0);

    // Repeat predicted frames from the cached NV12 surface.
    for (int i = 0; i < 5; ++i) {
        CHECK(enc->encode_repeat(false, pkt, &err));
        CHECK(!pkt.keyframe);
        CHECK_EQ(pkt.pts_ns, int64_t(777777));
        CHECK(dec->decode(pkt.data, pkt.pts_ns, pic, &err));
        Quality qr;
        check_picture(c, pic, 42, w, h, qr);
    }

    // A keyframe repeat decodes alone in a brand-new decoder.
    CHECK(enc->encode_repeat(true, pkt, &err));
    CHECK(pkt.keyframe);
    CHECK(is_keyframe(c, pkt.data));
    CHECK_EQ(pkt.pts_ns, int64_t(777777));

    auto fresh_dec = make_decoder(c, &err, true);
    CHECK(fresh_dec != nullptr);
    if (fresh_dec) {
        CHECK(fresh_dec->decode(pkt.data, pkt.pts_ns, pic, &err));
        Quality qk;
        check_picture(c, pic, 42, w, h, qk);
        std::printf("   keyframe repeat decoded alone: luma %.1f dB, RGB %.1f dB\n", qk.luma, qk.rgb);
    }
}

void test_timing(Codec c) {
    check::phase(std::string(codec_name(c)) + ": 1920x1080 encode latency");
    const uint32_t w = 1920, h = 1080;
    std::string err;
    auto enc = make_encoder(c, w, h, 20000, &err);
    CHECK(enc != nullptr);
    if (!enc) return;

    std::vector<uint8_t> px = pattern(w, h, 0);
    std::vector<double> ms;
    for (uint32_t n = 0; n < 120; ++n) {
        px[(n * 13) % px.size()] = uint8_t(n);
        Frame f;
        f.width = w;
        f.height = h;
        f.cpu = px.data();
        f.pts_ns = int64_t(n) * 16666667;
        Packet pkt;
        const auto t0 = std::chrono::steady_clock::now();
        const bool ok = enc->encode(f, false, {}, pkt, &err);
        ms.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
        CHECK(ok && !pkt.data.empty());
    }
    std::vector<double> rest(ms.begin() + 1, ms.end());
    std::sort(rest.begin(), rest.end());
    double sum = 0;
    for (double v : rest) sum += v;
    std::printf("   %s: first %.2f ms; then mean %.2f, p50 %.2f, p99 %.2f, max %.2f ms per frame\n",
                enc->info().description.c_str(), ms[0], sum / double(rest.size()), rest[rest.size() / 2],
                rest[size_t(0.99 * double(rest.size() - 1))], rest.back());
}

}  // namespace

int main() {
    check::watchdog(600);
    check::phase("probe Media Foundation encoders");
    std::vector<Codec> codecs_here;
    const char* only = std::getenv("BROVIDEO_TEST_CODEC");
    for (const Capability& cap : capabilities()) {
        if (cap.backend != Backend::MediaFoundation || cap.direction != Direction::Encode) continue;
        CHECK(cap.hardware);
        if (std::find(codecs_here.begin(), codecs_here.end(), cap.codec) != codecs_here.end()) continue;
        if (!only || !*only || parse_codec(only) == cap.codec) codecs_here.push_back(cap.codec);
        std::printf("   %s encode: %s (%s)\n", codec_name(cap.codec), backend_name(cap.backend), cap.device.c_str());
    }
    if (codecs_here.empty()) {
        std::printf("SKIP: no Media Foundation hardware encoder on this machine\n");
        for (Codec c : {Codec::H264, Codec::HEVC, Codec::AV1}) {
            std::printf("   %s: %s\n", codec_name(c), unavailable_reason(c, Direction::Encode).c_str());
        }
        return kSkip;
    }

    for (Codec c : codecs_here) {
        test_rgba_sequence(c);
        test_bgra_and_nv12(c);
        test_odd_dimensions(c);
        test_reconfigure(c);
        test_repeat(c);
        test_timing(c);
    }
    return check::finish();
}
