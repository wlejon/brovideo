#pragma once
// Decoded pictures as plain RGBA, for checking them against the test pattern
// and for writing them out: NV12 is converted with the BT.709 limited-range
// matrix the encoders signal (chroma taken from the 2x2 block a pixel is in).

#include "brovideo/frame.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace testkit {

// `f` as tightly packed RGBA (R G B A per pixel), whatever its format.
inline std::vector<uint8_t> to_rgba(const brovideo::Picture& f) {
    std::vector<uint8_t> out(size_t(f.width) * f.height * 4);
    if (f.format == brovideo::PixelFormat::RGBA8) {
        for (uint32_t y = 0; y < f.height; ++y) {
            std::copy_n(f.data.data() + size_t(y) * f.stride, size_t(f.width) * 4, out.data() + size_t(y) * f.width * 4);
        }
        return out;
    }
    const auto clamp = [](double v) { return uint8_t(std::clamp(v + 0.5, 0.0, 255.0)); };
    for (uint32_t y = 0; y < f.height; ++y) {
        const uint8_t* yrow = f.data.data() + size_t(y) * f.stride;
        const uint8_t* uvrow = f.data.data() + f.uv_offset + size_t(y / 2) * f.stride;
        uint8_t* o = out.data() + size_t(y) * f.width * 4;
        for (uint32_t x = 0; x < f.width; ++x) {
            const double l = 1.164383 * (double(yrow[x]) - 16.0);
            const double u = double(uvrow[(x & ~1u)]) - 128.0;
            const double v = double(uvrow[(x & ~1u) + 1]) - 128.0;
            o[x * 4 + 0] = clamp(l + 1.792741 * v);
            o[x * 4 + 1] = clamp(l - 0.213249 * u - 0.532909 * v);
            o[x * 4 + 2] = clamp(l + 2.112402 * u);
            o[x * 4 + 3] = 255;
        }
    }
    return out;
}

// A row of 32 one-bit blocks of `block` pixels starting at x = 3 * block,
// y = row * block (the test pattern's counter, row 0, and its input marker,
// row 1), read straight from the decoded picture: luma for NV12, so no
// conversion. -1 when the picture is too small.
inline int64_t read_block_row(const brovideo::Picture& f, uint32_t row, uint32_t block) {
    if (f.width < 35 * block || f.height < (row + 1) * block || f.data.empty()) return -1;
    const size_t y = size_t(row) * block + block / 2;
    uint64_t v = 0;
    for (uint32_t i = 0; i < 32; ++i) {
        const size_t x = size_t(3 + i) * block + block / 2;
        int lum = 0;
        if (f.format == brovideo::PixelFormat::NV12) {
            lum = f.data[y * f.stride + x];  // limited range: black 16, white 235
        } else {
            const uint8_t* p = f.data.data() + y * f.stride + x * 4;
            lum = (p[0] + p[1] + p[2]) / 3;
        }
        v = (v << 1) | (lum >= 128 ? 1u : 0u);
    }
    return int64_t(v);
}

// PSNR in dB between two RGBA pictures of the same size, over R, G and B.
inline double psnr_rgba(const uint8_t* a, const uint8_t* b, uint32_t width, uint32_t height) {
    double se = 0;
    const size_t n = size_t(width) * height;
    for (size_t i = 0; i < n; ++i) {
        for (int c = 0; c < 3; ++c) {
            const double d = double(a[i * 4 + c]) - double(b[i * 4 + c]);
            se += d * d;
        }
    }
    const double mse = se / double(n * 3);
    return mse <= 1e-10 ? 99.0 : 10.0 * std::log10(255.0 * 255.0 / mse);
}

// PSNR in dB of an NV12 picture's luma against the ideal BT.709
// limited-range luma of an RGBA source of the same size.
inline double psnr_luma(const brovideo::Picture& f, const uint8_t* rgba) {
    if (f.format != brovideo::PixelFormat::NV12) return 0;
    double se = 0;
    for (uint32_t y = 0; y < f.height; ++y) {
        const uint8_t* row = f.data.data() + size_t(y) * f.stride;
        const uint8_t* src = rgba + size_t(y) * f.width * 4;
        for (uint32_t x = 0; x < f.width; ++x) {
            const double ideal =
                16.0 + 219.0 * (0.2126 * src[x * 4] + 0.7152 * src[x * 4 + 1] + 0.0722 * src[x * 4 + 2]) / 255.0;
            const double d = double(row[x]) - ideal;
            se += d * d;
        }
    }
    const double mse = se / (double(f.width) * f.height);
    return mse <= 1e-10 ? 99.0 : 10.0 * std::log10(255.0 * 255.0 / mse);
}

}  // namespace testkit
