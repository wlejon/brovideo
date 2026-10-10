// The VA-API encoder against ffmpeg as the oracle (Linux with a VA encoder;
// skipped otherwise). For every codec the machine reports:
//   - CPU frames: a 300-frame pattern sequence with forced and requested
//     keyframes, which runs frame_num / POC past their wrap points; ffmpeg
//     decodes the whole stream and each keyframe-led segment alone, every
//     picture is compared with its source by PSNR, and the frame count must
//     match. BGRA CPU frames the same way, shorter.
//   - A size change (a new encoder), at a size that needs cropping in both
//     directions; and reconfigure(): a new size in the same encoder, and a
//     bitrate change the packet sizes follow.
//   - dmabuf frames from GBM (brodmabuf), each with a real sync_file acquire
//     fence: linear XRGB8888, ARGB8888 with GBM's pick of the primary plane's
//     modifiers (as a compositor allocates scanout buffers), and a short run
//     on every XRGB8888 modifier the plane offers.
//   - release is called exactly once per frame, before encode() returns, and
//     the frame's memory is scribbled over in the release callback, so any
//     read after release would show as a PSNR failure.
//   - encode_repeat(): after a big change on a still screen, repeats of the
//     kept picture sharpen it to a settled still picture's quality and end
//     in near-empty packets; a keyframe repeat decodes alone.
//   - Per-frame encode latency of dmabuf frames at 1920x1080 and 2560x1440.
#include "brovideo/brovideo.h"
#include "check.h"
#include "dmabuf_frames.h"
#include "ffmpeg_oracle.h"
#include "test_pattern.h"

#include <dirent.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <set>
#include <tuple>

using namespace brovideo;
using testkit::draw_test_pattern;

namespace {

constexpr double kMinPsnr = 35.0;          // dB, luma, per picture, at the bitrates below
constexpr double kRgbBelowCeiling = 5.0;   // dB RGB PSNR may sit under the uncoded 4:2:0 ceiling
constexpr double kMaxBias = 1.0;           // code values of mean error per channel
constexpr int kSkip = 77;          // ctest SKIP_RETURN_CODE

bool g_ffmpeg = false;
std::string g_dir;

std::string path_for(const std::string& name) { return g_dir + "/" + name; }

std::vector<uint8_t> pattern(uint32_t w, uint32_t h, uint64_t n) {
    std::vector<uint8_t> px(size_t(w) * h * 4);
    draw_test_pattern(px.data(), w, h, n);
    return px;
}

std::vector<uint8_t> concat(const std::vector<Packet>& pkts, size_t from, size_t to) {
    std::vector<uint8_t> out;
    for (size_t i = from; i < to; ++i) out.insert(out.end(), pkts[i].data.begin(), pkts[i].data.end());
    return out;
}

std::unique_ptr<Encoder> make_encoder(Codec codec, uint32_t w, uint32_t h, uint32_t kbps, std::string* err) {
    EncoderConfig ec;
    ec.codec = codec;
    ec.width = w;
    ec.height = h;
    ec.bitrate_kbps = kbps;
    ec.allow_software = false;
    return create_encoder(ec, err);
}

struct Quality {
    double luma_min = 99, luma_mean = 0;  // decoded Y against the ideal BT.709 luma
    double rgb_min = 99, rgb_mean = 0;    // decoded RGB against the RGBA source
    double ceiling = 0;                   // RGB PSNR of an uncoded 4:2:0 round trip of the first source
    double bias_max = 0;                  // largest per-channel mean error (a range or transfer mistake)
};

// Decodes packets [from, to) and compares each picture with the pattern
// frame it was made from (`source[i]`).
//
// The bar is luma PSNR: the test pattern wraps every channel each 256 pixels,
// full-swing colour edges that 4:2:0 cannot carry, so RGB PSNR is capped near
// 33 dB before any coding (the ceiling, measured with ffmpeg's own
// conversion). RGB must stay within kRgbBelowCeiling of that ceiling, which
// catches a wrong matrix, range or channel order; luma must reach kMinPsnr.
Quality verify_range(Codec codec, const std::vector<Packet>& pkts, const std::vector<uint64_t>& source, size_t from,
                     size_t to, uint32_t w, uint32_t h, const std::string& name) {
    Quality q;
    const std::string path = path_for(name);
    oracle::write_file(path, concat(pkts, from, to));
    std::vector<std::vector<uint8_t>> rgb, luma;
    std::string log;
    CHECK(oracle::decode(path, codec, w, h, rgb, log));
    CHECK(oracle::decode(path, codec, w, h, luma, log, true));
    CHECK_EQ(rgb.size(), to - from);
    CHECK_EQ(luma.size(), to - from);
    CHECK_EQ(log, std::string());
    if (!log.empty()) std::printf("   ffmpeg (%s): %s\n", name.c_str(), log.substr(0, 600).c_str());
    const auto first = pattern(w, h, source[from]);
    const auto round = oracle::roundtrip_420(path + ".src", first, w, h);
    q.ceiling = round.empty() ? 0 : oracle::psnr(round.data(), first.data(), w, h);
    int reported = 0;
    const size_t n = std::min(rgb.size(), luma.size());
    for (size_t i = 0; i < n; ++i) {
        const auto src = pattern(w, h, source[from + i]);
        const double pr = oracle::psnr(rgb[i].data(), src.data(), w, h);
        const double py = oracle::psnr_luma(luma[i].data(), oracle::ideal_luma(src.data(), w, h));
        q.rgb_min = std::min(q.rgb_min, pr);
        q.luma_min = std::min(q.luma_min, py);
        q.rgb_mean += pr / double(n);
        q.luma_mean += py / double(n);
        for (double e : oracle::mean_error(rgb[i].data(), src.data(), w, h)) q.bias_max = std::max(q.bias_max, std::abs(e));
        if ((py < kMinPsnr || pr < q.ceiling - kRgbBelowCeiling) && ++reported <= 3) {
            std::printf("   %s: picture %zu (source %llu) luma %.2f dB, RGB %.2f dB, %s\n", name.c_str(), i,
                        static_cast<unsigned long long>(source[from + i]), py, pr,
                        oracle::channel_bias(rgb[i].data(), src.data(), w, h).c_str());
        }
    }
    CHECK(q.luma_min >= kMinPsnr);
    CHECK(q.ceiling > 0);
    CHECK(q.rgb_min >= q.ceiling - kRgbBelowCeiling);
    CHECK(q.bias_max <= kMaxBias);
    return q;
}

// The whole stream, then each keyframe-led segment on its own.
void verify_stream(Codec codec, const std::vector<Packet>& pkts, const std::vector<uint64_t>& source, uint32_t w,
                   uint32_t h, const std::string& name) {
    if (!g_ffmpeg) return;
    uint32_t pw = 0, ph = 0;
    oracle::write_file(path_for(name + ".probe"), concat(pkts, 0, std::min<size_t>(pkts.size(), 1)));
    CHECK(oracle::probe_size(path_for(name + ".probe"), codec, pw, ph));
    if (codec == Codec::AV1) {
        // AV1 cannot crop: the frame may be
        // padded, with the visible size in render_size (applied by decode()).
        CHECK(pw >= w && ph >= h);
        if (pw != w || ph != h) std::printf("   %s: AV1 frame %ux%u for a %ux%u picture\n", name.c_str(), pw, ph, w, h);
    } else {
        CHECK_EQ(pw, w);
        CHECK_EQ(ph, h);
    }
    const Quality q = verify_range(codec, pkts, source, 0, pkts.size(), w, h, name);
    std::printf("   %s: %zu pictures decoded; PSNR luma min %.2f mean %.2f dB; RGB min %.2f mean %.2f dB "
                "(uncoded 4:2:0 ceiling %.2f); max channel bias %.2f\n",
                name.c_str(), pkts.size(), q.luma_min, q.luma_mean, q.rgb_min, q.rgb_mean, q.ceiling, q.bias_max);
    std::vector<size_t> keys;
    for (size_t i = 0; i < pkts.size(); ++i) {
        if (pkts[i].keyframe) keys.push_back(i);
    }
    for (size_t k = 0; k < keys.size(); ++k) {
        const size_t end = k + 1 < keys.size() ? keys[k + 1] : pkts.size();
        verify_range(codec, pkts, source, keys[k], end, w, h, name + ".from" + std::to_string(keys[k]));
    }
    std::printf("   %s: %zu keyframe segments each decode alone\n", name.c_str(), keys.size());
}

// ---------------------------------------------------------------------------------

void test_cpu_sequence(Codec codec) {
    check::phase(std::string(codec_name(codec)) + ": CPU frames, forced and requested keyframes, wraps");
    const uint32_t w = 1280, h = 720, n_frames = 300;
    const std::set<uint32_t> forced = {100, 217, 218};
    const uint32_t requested = 260;
    std::string err;
    auto enc = make_encoder(codec, w, h, 12000, &err);
    CHECK(enc != nullptr);
    if (!enc) {
        std::printf("   create_encoder: %s\n", err.c_str());
        return;
    }
    const EncoderInfo info = enc->info();
    CHECK(info.backend == Backend::VaApi);
    CHECK(info.hardware);
    std::vector<Packet> pkts;
    std::vector<uint64_t> source;
    std::vector<uint8_t> px(size_t(w) * h * 4);
    for (uint32_t n = 0; n < n_frames; ++n) {
        draw_test_pattern(px.data(), w, h, n);
        Frame f;
        f.width = w;
        f.height = h;
        f.cpu = px.data();
        f.pts_ns = int64_t(n) * 16666667;
        int releases = 0;
        // Scribble over the pixels on release: a read after it would show.
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
        CHECK_EQ(pkt.keyframe, n == 0 || forced.count(n) > 0 || n == requested);
        CHECK_EQ(is_keyframe(codec, pkt.data), pkt.keyframe);
        CHECK(!pkt.data.empty());
        CHECK_EQ(pkt.pts_ns, f.pts_ns);
        pkts.push_back(std::move(pkt));
        source.push_back(n);
    }
    std::printf("   %s\n", enc->info().description.c_str());
    verify_stream(codec, pkts, source, w, h, std::string(codec_name(codec)) + "-cpu");
}

// BGRA CPU frames: the upload surface takes the frame's own byte order.
void test_bgra(Codec codec) {
    check::phase(std::string(codec_name(codec)) + ": BGRA CPU frames");
    const uint32_t w = 640, h = 360;
    std::string err;
    auto enc = make_encoder(codec, w, h, 8000, &err);
    CHECK(enc != nullptr);
    if (!enc) return;
    std::vector<Packet> pkts;
    std::vector<uint64_t> source;
    for (uint32_t n = 0; n < 30; ++n) {
        auto px = pattern(w, h, n + 500);
        for (size_t i = 0; i < px.size(); i += 4) std::swap(px[i], px[i + 2]);
        Frame f;
        f.width = w;
        f.height = h;
        f.cpu = px.data();
        f.cpu_format = PixelFormat::BGRA8;
        Packet pkt;
        CHECK(enc->encode(f, false, {}, pkt, &err));
        pkts.push_back(std::move(pkt));
        source.push_back(n + 500);
    }
    verify_stream(codec, pkts, source, w, h, std::string(codec_name(codec)) + "-bgra");
}

void test_size_change(Codec codec) {
    check::phase(std::string(codec_name(codec)) + ": size change (cropped size)");
    for (auto [w, h] : {std::pair<uint32_t, uint32_t>{640, 360}, {1366, 770}}) {
        std::string err;
        auto enc = make_encoder(codec, w, h, 12000, &err);
        CHECK(enc != nullptr);
        if (!enc) return;
        std::vector<Packet> pkts;
        std::vector<uint64_t> source;
        for (uint32_t n = 0; n < 40; ++n) {
            auto px = pattern(w, h, n + 1000);
            Frame f;
            f.width = w;
            f.height = h;
            f.cpu = px.data();
            int releases = 0;
            Packet pkt;
            CHECK(enc->encode(f, n == 20, [&] { ++releases; }, pkt, &err));
            CHECK_EQ(releases, 1);
            pkts.push_back(std::move(pkt));
            source.push_back(n + 1000);
        }
        verify_stream(codec, pkts, source, w, h,
                      std::string(codec_name(codec)) + "-" + std::to_string(w) + "x" + std::to_string(h));
    }
}

double mean_kb(const std::vector<Packet>& pkts, size_t from, size_t to) {
    double sum = 0;
    for (size_t i = from; i < to; ++i) sum += double(pkts[i].data.size()) / 1000.0;
    return to > from ? sum / double(to - from) : 0;
}

// reconfigure(), all in one encoder: the bitrate drops to a quarter
// mid-stream, then rises again, then the size changes. Each change starts
// with a keyframe. On grain (which fills any budget: the encoder pads
// nothing) the predicted packets follow the bitrate down; on the pattern,
// the stream after the rise and the stream after the size change decode to
// their sources at the configured sizes.
void test_reconfigure(Codec codec) {
    check::phase(std::string(codec_name(codec)) + ": reconfigure bitrate, then size");
    const uint32_t w = 1280, h = 720;
    std::string err;
    auto enc = make_encoder(codec, w, h, 8000, &err);
    CHECK(enc != nullptr);
    if (!enc) return;
    std::vector<Packet> pkts;
    uint32_t rng = 1;
    auto encode_run = [&](uint32_t fw, uint32_t fh, uint32_t first, uint32_t count, bool noise) {
        for (uint32_t n = first; n < first + count; ++n) {
            auto px = pattern(fw, fh, n);
            if (noise) {
                // Low-amplitude grain: full-range noise drives H.264 to QP 51
                // at 8 Mbit/s already, where no budget can lower it further.
                for (auto& b : px) b = uint8_t(112 + ((rng = rng * 1664525u + 1013904223u) >> 27));
            }
            Frame f;
            f.width = fw;
            f.height = fh;
            f.cpu = px.data();
            Packet pkt;
            CHECK(enc->encode(f, false, {}, pkt, &err));
            pkts.push_back(std::move(pkt));
        }
    };
    encode_run(w, h, 0, 60, true);
    EncoderConfig cfg = enc->config();
    cfg.bitrate_kbps = 2000;
    CHECK(enc->reconfigure(cfg, &err));
    encode_run(w, h, 60, 60, true);
    CHECK(pkts[60].keyframe);
    for (size_t i = 61; i < pkts.size(); ++i) CHECK(!pkts[i].keyframe);
    // Predicted packets well after each start, where rate control has settled.
    const double high_kb = mean_kb(pkts, 30, 60), low_kb = mean_kb(pkts, 90, 120);
    std::printf("   noise, predicted packets: %.1f kB at 8 Mbit/s, %.1f kB at 2 Mbit/s (budgets %.1f / %.1f)\n",
                high_kb, low_kb, 8000.0 / 8 / 60, 2000.0 / 8 / 60);
    CHECK(low_kb < high_kb / 2.5);
    CHECK(low_kb < 2000.0 / 8 / 60 * 1.5);  // near the new budget, not the old one

    cfg.bitrate_kbps = 12000;
    CHECK(enc->reconfigure(cfg, &err));
    size_t at = pkts.size();
    encode_run(w, h, 3000, 40, false);
    CHECK(pkts[at].keyframe);
    {
        std::vector<Packet> tail(pkts.begin() + std::ptrdiff_t(at), pkts.end());
        std::vector<uint64_t> source(tail.size());
        for (size_t i = 0; i < source.size(); ++i) source[i] = 3000 + i;
        verify_stream(codec, tail, source, w, h, std::string(codec_name(codec)) + "-rebitrate");
    }

    const uint32_t w2 = 960, h2 = 544;
    EncoderConfig resized = enc->config();
    resized.width = w2;
    resized.height = h2;
    CHECK(enc->reconfigure(resized, &err));
    at = pkts.size();
    encode_run(w2, h2, 2000, 30, false);
    CHECK(pkts[at].keyframe);
    std::vector<Packet> tail(pkts.begin() + std::ptrdiff_t(at), pkts.end());
    std::vector<uint64_t> source(tail.size());
    for (size_t i = 0; i < source.size(); ++i) source[i] = 2000 + i;
    verify_stream(codec, tail, source, w2, h2, std::string(codec_name(codec)) + "-resized");
}

// A desktop-like picture: a light page of dark text-like strokes (1-2 px,
// the detail rate control starves first) on coloured panels. Each seed lays
// it out differently, so one seed after another is a big change, a window
// opening.
std::vector<uint8_t> text_page(uint32_t w, uint32_t h, uint32_t seed) {
    std::vector<uint8_t> px(size_t(w) * h * 4, 255);
    uint32_t rng = seed * 2654435761u + 1;
    const auto next = [&] { return (rng = rng * 1664525u + 1013904223u) >> 8; };
    const auto rect = [&](uint32_t x0, uint32_t y0, uint32_t rw, uint32_t rh, uint8_t r, uint8_t g, uint8_t b) {
        for (uint32_t y = y0; y < std::min(h, y0 + rh); ++y) {
            for (uint32_t x = x0; x < std::min(w, x0 + rw); ++x) {
                uint8_t* p = &px[(size_t(y) * w + x) * 4];
                p[0] = r, p[1] = g, p[2] = b;
            }
        }
    };
    for (int i = 0; i < 6; ++i) {
        rect(next() % w, next() % h, 200 + next() % 600, 100 + next() % 400, uint8_t(150 + next() % 100),
             uint8_t(150 + next() % 100), uint8_t(150 + next() % 100));
    }
    // Lines of glyphs: 7x10 cells of random strokes, 4 px between lines.
    for (uint32_t ly = 8 + seed % 7; ly + 10 < h; ly += 14) {
        uint32_t x = 8 + next() % 40;
        const uint32_t end = w - 8 - next() % 300;
        while (x + 7 < end) {
            const uint32_t glyph = next();
            for (int s = 0; s < 4; ++s) {
                const uint32_t bits = glyph >> (s * 5);
                if (bits & 1) rect(x + (bits >> 1) % 6, ly, 1 + (bits >> 4 & 1), 10, 30, 30, 40);  // a stem
                else rect(x, ly + (bits >> 1) % 9, 6, 1, 30, 30, 40);                            // a bar
            }
            x += 8;
            if (next() % 7 == 0) x += 8;  // a space
        }
    }
    return px;
}

// encode_repeat(): an encoder fed frames only when the picture changes (a
// compositor holding an unchanged screen) leaves the first frame after a big
// change as soft as rate control made it, for as long as the screen stays
// still. Repeats of the kept picture sharpen it, end in near-empty packets,
// and a repeat can be a keyframe that decodes alone.
void test_repeat(Codec codec) {
    check::phase(std::string(codec_name(codec)) + ": encode_repeat sharpens a still picture");
    const uint32_t w = 1920, h = 1080, kbps = 20000;  // broremote's default stream
    const int settle = 30, max_repeats = 60;
    // broremote's rule: repeat until two packets running are under a quarter
    // of the per-frame budget (bitrate / fps), for at most a second. Not a
    // smaller bar: HEVC (radeonsi) never codes a still picture as empty, it
    // levels off near a seventh of the budget; and not one packet: the
    // frames after a keyframe that overdrew the HRD buffer are starved small.
    const size_t converged = size_t(kbps) * 1000 / 8 / 60 / 4;
    std::string err;
    auto enc = make_encoder(codec, w, h, kbps, &err);
    CHECK(enc != nullptr);
    if (!enc) return;
    std::vector<Packet> pkts;
    std::vector<int> source;  // the page each packet shows
    const std::vector<uint8_t> pages[2] = {text_page(w, h, 1), text_page(w, h, 2)};
    const auto encode = [&](int page) {
        Frame f;
        f.width = w;
        f.height = h;
        f.cpu = pages[page].data();
        f.pts_ns = int64_t(pkts.size());
        Packet pkt;
        CHECK(enc->encode(f, false, {}, pkt, &err));
        pkts.push_back(std::move(pkt));
        source.push_back(page);
    };
    const auto repeat = [&](bool keyframe) {
        Packet pkt;
        const bool ok = enc->encode_repeat(keyframe, pkt, &err);
        CHECK(ok);
        if (!ok) std::printf("   encode_repeat: %s\n", err.c_str());
        CHECK_EQ(pkt.keyframe, keyframe);
        CHECK_EQ(is_keyframe(codec, pkt.data), keyframe);
        CHECK(!pkt.data.empty());
        pkts.push_back(std::move(pkt));
        source.push_back(source.back());
    };
    err.clear();
    Packet none;
    CHECK(!enc->encode_repeat(false, none, &err));  // no picture yet
    CHECK(err.empty());
    // Repeats until converged; the index of the last.
    const auto sharpen = [&] {
        int small = 0;
        for (int i = 0; i < max_repeats && small < 2; ++i) {
            repeat(false);
            small = pkts.back().data.size() < converged ? small + 1 : 0;
        }
        CHECK_EQ(small, 2);
        return pkts.size() - 1;
    };
    for (int i = 0; i < settle; ++i) encode(0);  // a still screen, rate control settled
    const size_t change = pkts.size();
    encode(1);                                   // the window opens, then the screen holds
    CHECK(enc->can_repeat());
    const size_t sharp = sharpen();
    const size_t key = pkts.size();
    repeat(true);                                // a viewer joins the still screen
    const size_t key_sharp = sharpen();
    CHECK_EQ(pkts[change].pts_ns, pkts[key].pts_ns);
    if (!g_ffmpeg) return;

    const std::string path = path_for(std::string(codec_name(codec)) + "-repeat");
    oracle::write_file(path, concat(pkts, 0, pkts.size()));
    std::vector<std::vector<uint8_t>> luma;
    std::string log;
    CHECK(oracle::decode(path, codec, w, h, luma, log, true));
    CHECK_EQ(luma.size(), pkts.size());
    CHECK_EQ(log, std::string());
    if (luma.size() != pkts.size()) return;
    const std::vector<double> ideal[2] = {oracle::ideal_luma(pages[0].data(), w, h),
                                          oracle::ideal_luma(pages[1].data(), w, h)};
    std::vector<double> q(pkts.size());
    for (size_t i = 0; i < pkts.size(); ++i) q[i] = oracle::psnr_luma(luma[i].data(), ideal[source[i]]);
    std::printf("   still page %.2f dB; after the change %.2f dB (%.1f kB); %zu repeats to %.2f dB:", q[change - 1],
                q[change], double(pkts[change].data.size()) / 1024.0, sharp - change, q[sharp]);
    for (size_t i = change + 1; i <= sharp; ++i) std::printf(" %.1f/%.0fk", q[i], double(pkts[i].data.size()) / 1024.0);
    std::printf("\n   keyframe repeat %.2f dB (%.1f kB), %zu repeats to %.2f dB:", q[key],
                double(pkts[key].data.size()) / 1024.0, key_sharp - key, q[key_sharp]);
    for (size_t i = key + 1; i <= key_sharp; ++i) std::printf(" %.1f/%.0fk", q[i], double(pkts[i].data.size()) / 1024.0);
    std::printf("\n");
    CHECK(q[sharp] >= kMinPsnr);
    CHECK(q[sharp] >= q[change] + 1.0);      // the repeats did the work
    CHECK(q[sharp] >= q[change - 1] - 1.0);  // as sharp as a settled still picture
    // After a keyframe the encoders' rate control may level off lower (AV1
    // near 40 dB here): only sharper, and past the bar.
    CHECK(q[key_sharp] >= kMinPsnr);
    CHECK(q[key_sharp] >= q[key] + 10.0);
    // The keyframe repeat and its repeats decode alone, as a joining viewer sees them.
    const std::string kpath = path + ".key";
    oracle::write_file(kpath, concat(pkts, key, pkts.size()));
    std::vector<std::vector<uint8_t>> alone;
    CHECK(oracle::decode(kpath, codec, w, h, alone, log, true));
    CHECK_EQ(alone.size(), pkts.size() - key);
    if (!alone.empty()) CHECK(std::abs(oracle::psnr_luma(alone.back().data(), ideal[1]) - q[key_sharp]) < 0.01);
}

std::string render_node() {
    if (const char* forced = std::getenv("BROVIDEO_VAAPI_DEVICE"); forced && *forced) return forced;
    std::vector<std::string> nodes;
    if (DIR* d = opendir("/dev/dri")) {
        while (dirent* e = readdir(d)) {
            if (std::strncmp(e->d_name, "renderD", 7) == 0) nodes.push_back(std::string("/dev/dri/") + e->d_name);
        }
        closedir(d);
    }
    std::sort(nodes.begin(), nodes.end());
    return nodes.empty() ? std::string() : nodes.front();
}

std::unique_ptr<brodmabuf::GbmDevice> open_gbm() {
    const std::string node = render_node();
    auto dev = brodmabuf::GbmDevice::open(node);
    if (!dev.ok()) {
        std::printf("   cannot open a GBM device on %s: %s\n", node.c_str(), std::string(dev.error_message()).c_str());
        return nullptr;
    }
    return std::move(dev.value());
}

struct Latency {
    double mean = 0, p99 = 0, max = 0;
};

Latency summarize(std::vector<double> ms) {
    Latency l;
    if (ms.empty()) return l;
    std::sort(ms.begin(), ms.end());
    for (double v : ms) l.mean += v;
    l.mean /= double(ms.size());
    l.p99 = ms[size_t(0.99 * double(ms.size() - 1))];
    l.max = ms.back();
    return l;
}

// Encodes `n_frames` dmabuf frames from a pool of three GBM buffers.
// Returns false when the buffers cannot be allocated (reported).
// `modifiers` empty: linear.
bool run_dmabuf(Codec codec, brodmabuf::GbmDevice& gbm, uint32_t w, uint32_t h, uint32_t format,
                const std::vector<uint64_t>& modifiers, uint32_t n_frames, uint32_t bitrate_kbps, bool verify,
                const std::string& name, Latency* lat) {
    std::string err;
    std::unique_ptr<testkit::DmabufFrame> bufs[3];
    for (auto& b : bufs) {
        b = std::make_unique<testkit::DmabufFrame>();
        if (!b->alloc(gbm, w, h, format, modifiers, &err)) {
            // GBM declining one modifier on its own means a compositor on
            // this GPU cannot render to it either: nothing to encode.
            const bool single = modifiers.size() == 1 && modifiers[0] != DRM_FORMAT_MOD_LINEAR;
            std::printf("   %s: %s%s\n", name.c_str(), err.c_str(), single ? " (GBM cannot allocate it: skipped)" : "");
            CHECK(single);
            return false;
        }
    }
    std::printf("   %s: modifier 0x%016llx, %u plane(s)\n", name.c_str(),
                static_cast<unsigned long long>(bufs[0]->frame.modifier), bufs[0]->frame.plane_count);
    CHECK(bufs[0]->frame.modifier != DRM_FORMAT_MOD_INVALID);
    auto enc = make_encoder(codec, w, h, bitrate_kbps, &err);
    CHECK(enc != nullptr);
    if (!enc) return false;
    std::vector<Packet> pkts;
    std::vector<uint64_t> source;
    std::vector<double> ms;
    const auto scribble = pattern(w, h, 999999);
    int fences = 0;
    for (uint32_t n = 0; n < n_frames; ++n) {
        testkit::DmabufFrame& b = *bufs[n % 3];
        CHECK(b.fill(pattern(w, h, n).data()));
        Frame f = b.frame;
        const brodmabuf::UniqueFd fence = b.export_fence();
        f.acquire_fence_fd = fence.get();
        fences += fence.valid();
        f.pts_ns = int64_t(n) * 16666667;
        int releases = 0;
        const auto release = [&] {
            ++releases;
            if (verify) b.fill(scribble.data());  // the producer reuses the buffer at once
        };
        Packet pkt;
        const auto t0 = std::chrono::steady_clock::now();
        const bool ok = enc->encode(f, verify && n == n_frames / 2, release, pkt, &err);
        const double t = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        if (n > 0) ms.push_back(t);  // the first frame opens the device
        CHECK(ok);
        CHECK_EQ(releases, 1);
        if (!ok) {
            std::printf("   %s frame %u: %s\n", name.c_str(), n, err.c_str());
            return false;
        }
        pkts.push_back(std::move(pkt));
        source.push_back(n);
    }
    CHECK(fences > 0);
    if (lat) *lat = summarize(ms);
    if (verify) verify_stream(codec, pkts, source, w, h, name);
    return true;
}

void test_dmabuf(Codec codec) {
    check::phase(std::string(codec_name(codec)) + ": dmabuf frames (GBM)");
    auto gbm = open_gbm();
    if (!gbm) {
        CHECK(false);
        return;
    }
    const std::string c = codec_name(codec);
    run_dmabuf(codec, *gbm, 1280, 720, DRM_FORMAT_XRGB8888, {}, 90, 12000, true, c + "-dmabuf-linear-xrgb", nullptr);
    const auto argb = testkit::scanout_modifiers(DRM_FORMAT_ARGB8888);
    const auto xrgb = testkit::scanout_modifiers(DRM_FORMAT_XRGB8888);
    if (argb.empty() || xrgb.empty()) {
        std::printf("   NOTE: no KMS plane modifiers readable (no /dev/dri/card*): tiled buffers not tested\n");
        return;
    }
    // GBM's pick, as a compositor allocates its scanout buffers.
    run_dmabuf(codec, *gbm, 1280, 720, DRM_FORMAT_ARGB8888, argb, 90, 12000, true, c + "-dmabuf-scanout-argb", nullptr);
    // And each modifier the plane offers on its own.
    std::printf("   %zu XRGB8888 scanout modifiers\n", xrgb.size());
    for (uint64_t mod : xrgb) {
        if (mod == DRM_FORMAT_MOD_INVALID) continue;
        char hex[24];
        std::snprintf(hex, sizeof hex, "%016llx", static_cast<unsigned long long>(mod));
        run_dmabuf(codec, *gbm, 640, 360, DRM_FORMAT_XRGB8888, {mod}, 12, 8000, true, c + "-mod-" + hex, nullptr);
    }
}

void test_latency(Codec codec) {
    check::phase(std::string(codec_name(codec)) + ": dmabuf encode latency");
    auto gbm = open_gbm();
    if (!gbm) {
        CHECK(false);
        return;
    }
    // 2560x1440 at 30 Mbps is also the AV1 ring-hang regression: with CBR
    // filler on, its quiet frames need more padding than VCN 4 survives.
    for (auto [w, h, kbps] : {std::tuple<uint32_t, uint32_t, uint32_t>{1920, 1080, 20000}, {2560, 1440, 30000}}) {
        Latency lat;
        const std::string name = std::string(codec_name(codec)) + "-" + std::to_string(w) + "x" + std::to_string(h);
        if (!run_dmabuf(codec, *gbm, w, h, DRM_FORMAT_XRGB8888, testkit::scanout_modifiers(DRM_FORMAT_XRGB8888), 240,
                        kbps, false, name, &lat)) {
            continue;
        }
        std::printf("   LATENCY %s scanout dmabuf -> packet: mean %.2f ms, p99 %.2f ms, max %.2f ms\n", name.c_str(),
                    lat.mean, lat.p99, lat.max);
        CHECK(lat.mean < 16.6);  // well inside a 60 Hz frame
    }
}

void remove_dir(const std::string& dir) {
    if (DIR* d = opendir(dir.c_str())) {
        while (dirent* e = readdir(d)) {
            if (e->d_name[0] != '.') std::remove((dir + "/" + e->d_name).c_str());
        }
        closedir(d);
    }
    ::rmdir(dir.c_str());
}

}  // namespace

int main() {
    check::watchdog(900);
    std::vector<Codec> codecs_here;
    // $BROVIDEO_TEST_CODEC narrows the run to one codec.
    const char* only = std::getenv("BROVIDEO_TEST_CODEC");
    for (const Capability& c : capabilities()) {
        if (c.backend != Backend::VaApi || c.direction != Direction::Encode) continue;
        if (std::find(codecs_here.begin(), codecs_here.end(), c.codec) != codecs_here.end()) continue;
        if (!only || !*only || parse_codec(only) == c.codec) codecs_here.push_back(c.codec);
        std::printf("%s encode: %s\n", codec_name(c.codec), c.device.c_str());
    }
    if (codecs_here.empty()) {
        std::printf("SKIP: no VA-API encoder on this machine\n");
        for (Codec c : {Codec::H264, Codec::HEVC, Codec::AV1}) {
            std::printf("  %s: %s\n", codec_name(c), unavailable_reason(c, Direction::Encode).c_str());
        }
        return kSkip;
    }
    g_ffmpeg = oracle::have_ffmpeg();
    if (!g_ffmpeg) std::printf("NOTE: ffmpeg is not on PATH: bitstreams are encoded but not decoded or compared\n");
    char tmpl[] = "/tmp/brovideo-vaapi-XXXXXX";
    if (!mkdtemp(tmpl)) {
        std::printf("FAIL: mkdtemp\n");
        return 1;
    }
    g_dir = tmpl;
    std::printf("codecs:");
    for (Codec c : codecs_here) std::printf(" %s", codec_name(c));
    std::printf("; work dir %s\n", g_dir.c_str());

    for (Codec c : codecs_here) {
        test_cpu_sequence(c);
        test_bgra(c);
        test_size_change(c);
        test_reconfigure(c);
        test_repeat(c);
        test_dmabuf(c);
        test_latency(c);
    }
    const int rc = check::finish();
    if (rc == 0) remove_dir(g_dir);
    else std::printf("bitstreams kept in %s\n", g_dir.c_str());
    return rc;
}
