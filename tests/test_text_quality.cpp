// Text quality on a live desktop, against ffmpeg as the oracle, for every
// hardware encoder the machine has (VA-API on Linux, Media Foundation on
// Windows; skipped without one, or without ffmpeg).
//
// It drives the encoder the way broremote's server does: a tick per frame
// interval, a frame when the screen changed, and between frames predicted
// repeats of the last picture until two packets running are under a quarter
// of the per-frame budget or a second has passed. Then it decodes every
// packet and measures luma PSNR against the source, per region:
//   - line@N: a still text page where one line is rewritten N times a
//     second (a ninja status line, a counter). The line's quality as each
//     new frame lands and on average, and the static text around it.
//   - window, then line@N: the same, starting the moment the page appeared
//     (a window opened and the build started in it), so the line changes
//     while rate control is still starved by the big change.
//   - K lines@N: a block of K lines rewritten (a busier region).
//   - window: the page changes completely (a window opening) and then holds
//     still: the size of that packet (what must cross the link before the
//     viewer can show it), its quality, and how many ticks it takes to
//     settle.
// Every case runs twice: frames without damage, and frames that say what
// changed (Frame::damage, as a compositor's frames do), and the checks
// compare the two: damage must never cost quality or the big change's
// packet size, must stay near the bitrate, and on VA-API (which boosts a
// small damage through ROI QP deltas) must make the line after a window
// land readable at once.
// $BROVIDEO_TEST_CODEC narrows the run to one codec.
#include "brovideo/brovideo.h"
#include "check.h"
#include "ffmpeg_oracle.h"
#include "text_page.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>

#if defined(_WIN32)
#include <windows.h>
#else
#include <unistd.h>
#endif

using namespace brovideo;
using testkit::kLinePitch;

namespace {

constexpr int kSkip = 77;
constexpr uint32_t kW = 1920, kH = 1080, kKbps = 20000, kFps = 60;  // broremote's default stream
constexpr uint32_t kPageSeed = 1, kWindowSeed = 2;
constexpr uint32_t kLineIndex = 70;  // the rewritten line, near the bottom of the page
constexpr uint32_t kLineEnd = 1120;  // 140 columns of 8 px
constexpr double kSharp = 45.0;      // dB luma PSNR: text as crisp as a settled still page (47-50 here)

std::string g_dir;

uint32_t line_row() { return testkit::first_line(kPageSeed) + kLineIndex * kLinePitch; }

// What a picture shows: a page, with `lines` lines ending at the rewritten
// line rewritten from `line` (-1: as the page has it).
struct Shown {
    uint32_t page = kPageSeed;
    int64_t line = -1;
    uint32_t lines = 1;
};

std::vector<uint8_t> render(const Shown& s) {
    std::vector<uint8_t> px = testkit::text_page(kW, kH, s.page);
    for (uint32_t i = 0; s.line >= 0 && i < s.lines; ++i) {
        testkit::rewrite_line(px.data(), kW, kH, line_row() - i * kLinePitch, 0, kLineEnd,
                              uint32_t(s.line) * 1000u + i);
    }
    return px;
}

// The rows `lines` rewritten lines cover.
Rect band_of(uint32_t lines) {
    return Rect{0, line_row() - 2 - (lines - 1) * kLinePitch, kLineEnd, lines * kLinePitch};
}

// Luma PSNR of a decoded Y plane against the ideal luma of `rgba`, over the
// rectangle `r`, or (outside) over everything but it.
double psnr_region(const uint8_t* y, const uint8_t* rgba, const Rect& r, bool outside) {
    double se = 0;
    size_t n = 0;
    for (uint32_t row = 0; row < kH; ++row) {
        const bool in_rows = row >= r.y && row < r.y + r.height;
        if (!outside && !in_rows) continue;
        for (uint32_t x = 0; x < kW; ++x) {
            const bool in = in_rows && x >= r.x && x < r.x + r.width;
            if (in == outside) continue;
            const uint8_t* p = rgba + (size_t(row) * kW + x) * 4;
            const double ideal = 16.0 + 219.0 * (0.2126 * p[0] + 0.7152 * p[1] + 0.0722 * p[2]) / 255.0;
            const double d = double(y[size_t(row) * kW + x]) - ideal;
            se += d * d;
            ++n;
        }
    }
    const double mse = n ? se / double(n) : 0;
    return mse <= 1e-10 ? 99.0 : 10.0 * std::log10(255.0 * 255.0 / mse);
}

// One tick of the stream: the packet sent in it (if any) and what it shows.
struct Tick {
    int packet = -1;     // index into the packets, -1: nothing sent (the viewer keeps the last picture)
    bool frame = false;  // a new frame (else a repeat)
    Shown shown;
};

// broremote's encode loop, on a virtual clock.
class Stream {
public:
    explicit Stream(Encoder& enc) : enc_(enc) {}

    // `damage`: what changed since the last frame, when the source says (as a compositor does).
    void frame(const Shown& s, const std::vector<uint8_t>& px, const std::vector<Rect>* damage = nullptr) {
        Frame f;
        f.width = kW;
        f.height = kH;
        f.cpu = px.data();
        f.pts_ns = int64_t(ticks_.size());
        if (damage) {
            f.damage_known = true;
            f.damage = *damage;
        }
        Packet pkt;
        std::string err;
        const auto t0 = std::chrono::steady_clock::now();
        const bool ok = enc_.encode(f, false, {}, pkt, &err);
        encode_ms_.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
        CHECK(ok);
        if (!ok) std::printf("   encode: %s\n", err.c_str());
        shown_ = s;
        push(std::move(pkt), true);
        repeats_left_ = int(kFps);
        small_run_ = 0;
    }

    // A tick with no new frame: a repeat while it is still sharpening.
    void idle() {
        if (repeats_left_ <= 0) {
            ticks_.push_back({-1, false, shown_});
            return;
        }
        Packet pkt;
        std::string err;
        const bool ok = enc_.encode_repeat(false, pkt, &err);
        CHECK(ok);
        if (!ok) std::printf("   encode_repeat: %s\n", err.c_str());
        const size_t bar = size_t(kKbps) * 1000 / 8 / kFps / 4;
        small_run_ = pkt.data.size() < bar ? small_run_ + 1 : 0;
        repeats_left_ = small_run_ >= 2 ? 0 : repeats_left_ - 1;
        push(std::move(pkt), false);
    }

    [[nodiscard]] bool settled() const { return repeats_left_ <= 0; }
    [[nodiscard]] size_t tick_count() const { return ticks_.size(); }
    [[nodiscard]] const std::vector<Tick>& ticks() const { return ticks_; }
    [[nodiscard]] const std::vector<Packet>& packets() const { return pkts_; }
    [[nodiscard]] const std::vector<double>& encode_ms() const { return encode_ms_; }

private:
    void push(Packet pkt, bool frame) {
        ticks_.push_back({int(pkts_.size()), frame, shown_});
        pkts_.push_back(std::move(pkt));
    }

    Encoder& enc_;
    std::vector<Packet> pkts_;
    std::vector<Tick> ticks_;
    std::vector<double> encode_ms_;
    Shown shown_;
    int repeats_left_ = 0;
    int small_run_ = 0;
};

std::unique_ptr<Encoder> make_encoder(Codec codec) {
    EncoderConfig ec;
    ec.codec = codec;
    ec.width = kW;
    ec.height = kH;
    ec.fps = kFps;
    ec.bitrate_kbps = kKbps;
    ec.allow_software = false;
    std::string err;
    auto enc = create_encoder(ec, &err);
    if (!enc) std::printf("   create_encoder: %s\n", err.c_str());
    return enc;
}

// A still page, settled: frames until rate control has seen it, then
// repeats until they stop.
void settle(Stream& s, const Shown& shown) {
    const auto px = render(shown);
    for (int i = 0; i < 30; ++i) s.frame(shown, px);
    for (int i = 0; i < 120 && !s.settled(); ++i) s.idle();
}

// Decodes the stream and gives each packet's decoded luma to `fn(index, y)`.
bool decode_stream(Codec codec, const Stream& s, const std::string& name,
                   const std::function<void(size_t, const uint8_t*)>& fn) {
    std::vector<uint8_t> all;
    for (const Packet& p : s.packets()) all.insert(all.end(), p.data.begin(), p.data.end());
    const std::string path = g_dir + "/" + name;
    oracle::write_file(path, all);
    std::vector<std::vector<uint8_t>> luma;
    std::string log;
    const bool ok = oracle::decode(path, codec, kW, kH, luma, log, true);
    CHECK(ok);
    CHECK_EQ(log, std::string());
    CHECK_EQ(luma.size(), s.packets().size());
    if (!ok || luma.size() != s.packets().size()) return false;
    for (size_t i = 0; i < luma.size(); ++i) fn(i, luma[i].data());
    std::remove(path.c_str());
    return true;
}

struct LineResult {
    double line_arrival_mean = 0, line_arrival_min = 99;  // the line as each new frame lands
    double line_mean = 0;                                  // the line over every tick
    double static_mean = 0, static_min = 99;               // the rest of the page over every tick
    double kbps = 0;                                       // what the run sent
    double max_kb = 0;
    double second_frame = 0;  // after a window: the line in the first frame that changes only the line
    bool ok = false;          // measured
    Backend backend = Backend::Raw;
};

// A page whose `lines` lines (ending near the bottom) are rewritten `hz`
// times a second for two seconds. With `after_window`, the page has just
// appeared (a window opened, the build started) and the line starts
// changing at once; the whole run is measured. Otherwise the page was still
// and settled, and the second half is measured (rate control has seen the
// line change for a second). With `damage`, each frame says what changed
// (the lines; the first frame after a window, everything), as a
// compositor's frames do.
LineResult run_line(Codec codec, uint32_t hz, bool damage, bool after_window = false, uint32_t lines = 1) {
    const std::string name = std::string(codec_name(codec)) + (after_window ? " window, then " : " ") +
                             (lines > 1 ? std::to_string(lines) + " lines@" : "line@") + std::to_string(hz) +
                             (damage ? ", with damage" : "");
    check::phase(name);
    LineResult r;
    auto enc = make_encoder(codec);
    CHECK(enc != nullptr);
    if (!enc) return r;
    r.backend = enc->info().backend;
    Stream s(*enc);
    settle(s, Shown{after_window ? kWindowSeed : kPageSeed, -1});
    const size_t from = s.tick_count();
    const uint32_t every = kFps / hz;
    const int ticks = int(kFps) * 2;
    const Rect band = band_of(lines);
    const std::vector<Rect> line_damage{band}, everything{Rect{0, 0, kW, kH}};
    for (int t = 0; t < ticks; ++t) {
        if (t % every == 0) {
            const Shown shown{kPageSeed, t / int(every), lines};
            const auto* d = !damage ? nullptr : (after_window && t == 0) ? &everything : &line_damage;
            s.frame(shown, render(shown), d);
        } else {
            s.idle();
        }
    }
    const size_t measure_from = after_window ? from : from + size_t(ticks) / 2;
    std::vector<double> line_q(s.packets().size(), 0), static_q(s.packets().size(), 0);
    const auto& ticks_v = s.ticks();
    const bool ok = decode_stream(codec, s, name, [&](size_t i, const uint8_t* y) {
        // The source of packet i: the tick that sent it.
        size_t t = 0;
        while (t < ticks_v.size() && ticks_v[t].packet != int(i)) ++t;
        if (t < measure_from) return;
        const auto px = render(ticks_v[t].shown);
        line_q[i] = psnr_region(y, px.data(), band, false);
        static_q[i] = psnr_region(y, px.data(), band, true);
    });
    if (!ok) return r;
    size_t n = 0, arrivals = 0, last = 0, bytes = 0;
    for (size_t t = measure_from; t < ticks_v.size(); ++t) {
        const Tick& k = ticks_v[t];
        if (k.packet >= 0) {
            last = size_t(k.packet);
            bytes += s.packets()[last].data.size();
            r.max_kb = std::max(r.max_kb, double(s.packets()[last].data.size()) / 1024.0);
        }
        r.line_mean += line_q[last];
        r.static_mean += static_q[last];
        r.static_min = std::min(r.static_min, static_q[last]);
        ++n;
        if (k.frame) {
            r.line_arrival_mean += line_q[last];
            r.line_arrival_min = std::min(r.line_arrival_min, line_q[last]);
            ++arrivals;
        }
    }
    if (after_window) {
        // Tick by tick: the line / the static text.
        std::printf("   ");
        size_t shown = 0, frames = 0;
        for (size_t t = measure_from; t < std::min(ticks_v.size(), measure_from + 40); ++t) {
            if (ticks_v[t].packet >= 0) shown = size_t(ticks_v[t].packet);
            if (ticks_v[t].frame && ++frames == 2) r.second_frame = line_q[shown];
            std::printf(" %.0f/%.0f%s", line_q[shown], static_q[shown], ticks_v[t].frame ? "*" : "");
        }
        std::printf("\n");
    }
    r.line_mean /= double(n);
    r.static_mean /= double(n);
    r.line_arrival_mean /= double(std::max<size_t>(1, arrivals));
    r.kbps = double(bytes) * 8.0 / 1000.0 / (double(n) / kFps);
    std::printf("   line as it lands %.1f dB mean, %.1f min; line %.1f dB mean; static text %.1f dB mean, %.1f min; "
                "%.0f kbit/s, largest packet %.1f kB\n",
                r.line_arrival_mean, r.line_arrival_min, r.line_mean, r.static_mean, r.static_min, r.kbps, r.max_kb);
    r.ok = true;
    return r;
}

struct WindowResult {
    double change_kb = 0, change_q = 0, change_encode_ms = 0;
    double settled_q = 0;
    int ticks_to_settle = 0;  // ticks until within 1 dB of the settled quality
};

// The page changes completely and then holds still. With `damage`, the
// frame says so (all of it changed).
WindowResult run_window(Codec codec, bool damage) {
    const std::string name = std::string(codec_name(codec)) + " window" + (damage ? ", with damage" : "");
    check::phase(name);
    WindowResult r;
    auto enc = make_encoder(codec);
    CHECK(enc != nullptr);
    if (!enc) return r;
    Stream s(*enc);
    settle(s, Shown{});
    const size_t change = s.tick_count();
    const size_t change_frame = s.encode_ms().size();
    const Shown window{kWindowSeed, -1};
    const std::vector<Rect> everything{Rect{0, 0, kW, kH}};
    s.frame(window, render(window), damage ? &everything : nullptr);
    for (int i = 0; i < int(kFps); ++i) s.idle();
    const auto px = render(window);
    std::vector<double> q(s.packets().size(), 0);
    const size_t first = size_t(s.ticks()[change].packet);
    const bool ok = decode_stream(codec, s, name, [&](size_t i, const uint8_t* y) {
        if (i >= first) q[i] = psnr_region(y, px.data(), Rect{0, 0, 0, 0}, true);
    });
    if (!ok) return r;
    r.change_kb = double(s.packets()[first].data.size()) / 1024.0;
    r.change_q = q[first];
    r.change_encode_ms = s.encode_ms()[change_frame];
    size_t last = first;
    std::vector<double> per_tick;
    for (size_t t = change; t < s.tick_count(); ++t) {
        if (s.ticks()[t].packet >= 0) last = size_t(s.ticks()[t].packet);
        per_tick.push_back(q[last]);
    }
    r.settled_q = per_tick.back();
    r.ticks_to_settle = int(per_tick.size());
    for (size_t t = 0; t < per_tick.size(); ++t) {
        if (per_tick[t] >= r.settled_q - 1.0) {
            r.ticks_to_settle = int(t);
            break;
        }
    }
    std::printf("   change %.1f kB, %.1f ms to encode, %.1f dB as it lands; settles to %.1f dB in %d ticks:", r.change_kb,
                r.change_encode_ms, r.change_q, r.settled_q, r.ticks_to_settle);
    for (size_t t = 0; t < std::min<size_t>(per_tick.size(), 30); ++t) std::printf(" %.0f", per_tick[t]);
    std::printf("\n");
    return r;
}

std::string make_work_dir() {
#if defined(_WIN32)
    char base[MAX_PATH];
    GetTempPathA(MAX_PATH, base);
    const std::string dir = std::string(base) + "brovideo-text-" + std::to_string(GetCurrentProcessId());
    CreateDirectoryA(dir.c_str(), nullptr);
    return dir;
#else
    char tmpl[] = "/tmp/brovideo-text-XXXXXX";
    return mkdtemp(tmpl) ? std::string(tmpl) : std::string();
#endif
}

}  // namespace

int main() {
    check::watchdog(900);
    std::vector<Codec> codecs;
    const char* only = std::getenv("BROVIDEO_TEST_CODEC");
    for (const Capability& c : capabilities()) {
        if (c.direction != Direction::Encode || !c.hardware || c.codec == Codec::Raw) continue;
        if (std::find(codecs.begin(), codecs.end(), c.codec) != codecs.end()) continue;
        if (only && *only && parse_codec(only) != c.codec) continue;
        codecs.push_back(c.codec);
        std::printf("%s encode: %s %s\n", codec_name(c.codec), backend_name(c.backend), c.device.c_str());
    }
    if (codecs.empty()) {
        std::printf("SKIP: no hardware encoder on this machine\n");
        return kSkip;
    }
    if (!oracle::have_ffmpeg()) {
        std::printf("SKIP: ffmpeg is not on PATH\n");
        return kSkip;
    }
    g_dir = make_work_dir();
    if (g_dir.empty()) {
        std::printf("FAIL: no work directory\n");
        return 1;
    }
    for (Codec c : codecs) {
        // Each case without damage (what the encoder did before it took any)
        // and with it.
        LineResult line[2][3], after_window[2], block[2][3], fast_block[2];
        WindowResult window[2];
        for (int d = 0; d < 2; ++d) {
            const uint32_t rates[3] = {60, 20, 10};
            for (int i = 0; i < 3; ++i) line[d][i] = run_line(c, rates[i], d);
            after_window[d] = run_line(c, 20, d, true);
            const uint32_t sizes[3] = {4, 16, 48};
            for (int i = 0; i < 3; ++i) block[d][i] = run_line(c, 20, d, false, sizes[i]);
            fast_block[d] = run_line(c, 60, d, false, 16);
            window[d] = run_window(c, d);
        }
        check::phase(std::string(codec_name(c)) + ": damage");
        // A text line rewritten on a still page lands sharp, damage or not.
        for (int d = 0; d < 2; ++d) {
            for (const LineResult& r : line[d]) {
                CHECK(r.line_arrival_min >= kSharp);
                CHECK(r.static_min >= kSharp);
            }
        }
        // Damage never makes a changing region softer, nor the text around it.
        const auto not_worse = [](const LineResult& with, const LineResult& without) {
            CHECK(with.line_arrival_mean >= without.line_arrival_mean - 1.0);
            CHECK(with.static_mean >= without.static_mean - 1.0);
        };
        for (int i = 0; i < 3; ++i) not_worse(line[1][i], line[0][i]);
        for (int i = 0; i < 3; ++i) not_worse(block[1][i], block[0][i]);
        not_worse(after_window[1], after_window[0]);
        not_worse(fast_block[1], fast_block[0]);
        // A window opening (all of it damage) is the same packet as before.
        CHECK(window[1].change_kb <= window[0].change_kb * 1.05 + 1.0);
        // Damage that keeps coming fast stays near the stream's bitrate.
        CHECK(fast_block[1].kbps <= fast_block[0].kbps * 1.15);
        // Where the encoder acts on damage (VA-API: ROI QP deltas), the line
        // rewritten right after a window opened lands readable at once
        // instead of as soft as the window, and a block of changing lines
        // lands sharper.
        if (after_window[1].backend == Backend::VaApi) {
            std::printf("   line after a window: %.1f dB without damage, %.1f with; 16 lines as they land: %.1f, %.1f\n",
                        after_window[0].second_frame, after_window[1].second_frame, block[0][1].line_arrival_mean,
                        block[1][1].line_arrival_mean);
            CHECK(after_window[1].second_frame >= 32.0);  // measured 37 (H.264) to 48 (HEVC); 19-23 without
            CHECK(after_window[1].second_frame >= after_window[0].second_frame + 10.0);
            CHECK(block[1][1].line_arrival_mean >= block[0][1].line_arrival_mean);
        }
    }
    return check::finish();
}
