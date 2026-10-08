#pragma once
// ffmpeg as a test oracle: decode an elementary stream to RGB with the
// BT.709 limited-range matrix the encoders signal, and compare pictures by
// PSNR. ffmpeg is never linked; it runs as a child process when it is on
// PATH. Commands go through popen (sh on POSIX, cmd.exe on Windows), so
// paths are double-quoted and must not contain quotes.

#include "brovideo/codec.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#if defined(_WIN32)
#define popen _popen
#define pclose _pclose
#endif

namespace oracle {

#if defined(_WIN32)
inline constexpr const char* kPipeRead = "rb";
inline constexpr const char* kPipeWrite = "wb";
inline constexpr const char* kNullDevice = "NUL";
#else
inline constexpr const char* kPipeRead = "r";
inline constexpr const char* kPipeWrite = "w";
inline constexpr const char* kNullDevice = "/dev/null";
#endif

inline std::string quote(const std::string& path) { return "\"" + path + "\""; }

inline bool have_ffmpeg() {
    return std::system((std::string("ffmpeg -hide_banner -version >") + kNullDevice + " 2>&1").c_str()) == 0;
}

inline void write_file(const std::string& path, const std::vector<uint8_t>& data) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out.write(reinterpret_cast<const char*>(data.data()), std::streamsize(data.size()));
}

inline std::string read_text(const std::string& path) {
    std::ifstream in(path);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

inline const char* demuxer(brovideo::Codec c) {
    switch (c) {
        case brovideo::Codec::H264: return "h264";
        case brovideo::Codec::HEVC: return "hevc";
        case brovideo::Codec::AV1: return "obu";
        default: return "rawvideo";
    }
}

// Runs an ffmpeg command whose stdout is raw pictures of `size` bytes each.
inline bool read_pictures(const std::string& cmd, size_t size, std::vector<std::vector<uint8_t>>& frames,
                          std::string& log, const std::string& err_path) {
    frames.clear();
    std::FILE* p = popen((cmd + " 2>" + quote(err_path)).c_str(), kPipeRead);
    if (!p) {
        log = "popen failed";
        return false;
    }
    for (;;) {
        std::vector<uint8_t> f(size);
        const size_t n = std::fread(f.data(), 1, size, p);
        if (n == 0) break;
        if (n != size) {
            log += "short picture (" + std::to_string(n) + " of " + std::to_string(size) + " bytes; size mismatch?)\n";
            break;
        }
        frames.push_back(std::move(f));
    }
    const int rc = pclose(p);
    log += read_text(err_path);
    std::remove(err_path.c_str());
    return rc == 0;
}

// Decodes `path` into tightly packed pictures of width x height: RGB24
// converted with the BT.709 limited-range matrix the encoders signal, or
// (luma_only) the decoder's own Y plane, untouched. `log` gets whatever
// ffmpeg printed at error level: a clean decode prints nothing.
inline bool decode(const std::string& path, brovideo::Codec codec, uint32_t width, uint32_t height,
                   std::vector<std::vector<uint8_t>>& frames, std::string& log, bool luma_only = false) {
    std::string cmd = std::string("ffmpeg -hide_banner -nostdin -v error -f ") + demuxer(codec) + " -i " + quote(path);
    // The top-left width x height: a no-op for H.264/HEVC (their decoded
    // size is the visible one); for AV1 it applies render_size, which ffmpeg
    // does not.
    const std::string crop = "crop=" + std::to_string(width) + ":" + std::to_string(height) + ":0:0";
    if (luma_only) {
        // yuv420p is the decoders' native output: no conversion, and Y comes first.
        cmd += " -vf " + crop + " -f rawvideo -pix_fmt yuv420p -";
    } else {
        cmd += " -sws_flags bicubic+accurate_rnd+full_chroma_int -vf " + crop +
               ",scale=in_range=limited:in_color_matrix=bt709:out_range=full -f rawvideo -pix_fmt rgb24 -";
    }
    const size_t size = luma_only ? size_t(width) * height + 2 * (size_t((width + 1) / 2) * ((height + 1) / 2))
                                  : size_t(width) * height * 3;
    const bool ok = read_pictures(cmd, size, frames, log, path + ".err");
    if (luma_only) {
        for (auto& f : frames) f.resize(size_t(width) * height);
    }
    return ok;
}

// The best an RGB picture can come back through 4:2:0 BT.709 limited range
// with no coding at all: ffmpeg's own RGBA -> YUV 4:2:0 -> RGB round trip.
inline std::vector<uint8_t> roundtrip_420(const std::string& scratch, const std::vector<uint8_t>& rgba, uint32_t width,
                                          uint32_t height) {
    write_file(scratch, rgba);
    const std::string cmd =
        "ffmpeg -hide_banner -nostdin -v error -f rawvideo -pix_fmt rgba -s " + std::to_string(width) + "x" +
        std::to_string(height) + " -i " + quote(scratch) +
        " -sws_flags bicubic+accurate_rnd+full_chroma_int"
        " -vf scale=out_color_matrix=bt709:out_range=limited,format=yuv420p,"
        "scale=in_color_matrix=bt709:in_range=limited:out_range=full -f rawvideo -pix_fmt rgb24 -";
    std::vector<std::vector<uint8_t>> frames;
    std::string log;
    read_pictures(cmd, size_t(width) * height * 3, frames, log, scratch + ".err");
    std::remove(scratch.c_str());
    return frames.empty() ? std::vector<uint8_t>{} : frames[0];
}

// The ideal BT.709 limited-range luma of an RGBA picture.
inline std::vector<double> ideal_luma(const uint8_t* rgba, uint32_t width, uint32_t height) {
    std::vector<double> y(size_t(width) * height);
    for (size_t i = 0; i < y.size(); ++i) {
        const double r = rgba[i * 4], g = rgba[i * 4 + 1], b = rgba[i * 4 + 2];
        y[i] = 16.0 + 219.0 * (0.2126 * r + 0.7152 * g + 0.0722 * b) / 255.0;
    }
    return y;
}

// PSNR in dB of a decoded Y plane against the ideal luma.
inline double psnr_luma(const uint8_t* y, const std::vector<double>& ideal) {
    double se = 0;
    for (size_t i = 0; i < ideal.size(); ++i) {
        const double d = double(y[i]) - ideal[i];
        se += d * d;
    }
    const double mse = se / double(ideal.size());
    return mse <= 1e-10 ? 99.0 : 10.0 * std::log10(255.0 * 255.0 / mse);
}

// The stream's dimensions as ffprobe reports them.
inline bool probe_size(const std::string& path, brovideo::Codec codec, uint32_t& w, uint32_t& h) {
    const std::string cmd = std::string("ffprobe -v error -f ") + demuxer(codec) + " -select_streams v:0 " +
                            "-show_entries stream=width,height -of csv=p=0 " + quote(path);
    std::FILE* p = popen(cmd.c_str(), kPipeRead);
    if (!p) return false;
    unsigned a = 0, b = 0;
    const int got = std::fscanf(p, "%u,%u", &a, &b);
    pclose(p);
    w = a;
    h = b;
    return got == 2;
}

// PSNR in dB of an RGB24 picture against an RGBA source of the same size.
inline double psnr(const uint8_t* rgb, const uint8_t* rgba, uint32_t width, uint32_t height) {
    double se = 0;
    const size_t n = size_t(width) * height;
    for (size_t i = 0; i < n; ++i) {
        for (int c = 0; c < 3; ++c) {
            const double d = double(rgb[i * 3 + c]) - double(rgba[i * 4 + c]);
            se += d * d;
        }
    }
    const double mse = se / double(n * 3);
    return mse <= 1e-10 ? 99.0 : 10.0 * std::log10(255.0 * 255.0 / mse);
}

// Mean signed error per channel (decoded - source), for diagnosing a colour
// conversion mismatch as opposed to coding loss.
inline std::array<double, 3> mean_error(const uint8_t* rgb, const uint8_t* rgba, uint32_t width, uint32_t height) {
    std::array<double, 3> sum{0, 0, 0};
    const size_t n = size_t(width) * height;
    for (size_t i = 0; i < n; ++i) {
        for (int c = 0; c < 3; ++c) sum[c] += double(rgb[i * 3 + c]) - double(rgba[i * 4 + c]);
    }
    for (double& s : sum) s /= double(n);
    return sum;
}

inline std::string channel_bias(const uint8_t* rgb, const uint8_t* rgba, uint32_t width, uint32_t height) {
    const auto e = mean_error(rgb, rgba, width, height);
    char buf[96];
    std::snprintf(buf, sizeof buf, "bias R %+.2f G %+.2f B %+.2f", e[0], e[1], e[2]);
    return buf;
}

// An RGB24 picture as RGBA (for the test pattern's counter reader).
inline std::vector<uint8_t> to_rgba(const std::vector<uint8_t>& rgb) {
    std::vector<uint8_t> out(rgb.size() / 3 * 4);
    for (size_t i = 0, j = 0; i < rgb.size(); i += 3, j += 4) {
        out[j] = rgb[i];
        out[j + 1] = rgb[i + 1];
        out[j + 2] = rgb[i + 2];
        out[j + 3] = 255;
    }
    return out;
}

}  // namespace oracle
