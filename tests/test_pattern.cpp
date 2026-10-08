#include "test_pattern.h"

namespace testkit {

namespace {

void fill_block(uint8_t* rgba, uint32_t width, uint32_t height, uint32_t x0, uint32_t y0, uint32_t size, uint8_t r,
                uint8_t g, uint8_t b) {
    for (uint32_t y = y0; y < y0 + size && y < height; ++y) {
        for (uint32_t x = x0; x < x0 + size && x < width; ++x) {
            uint8_t* p = rgba + (size_t(y) * width + x) * 4;
            p[0] = r;
            p[1] = g;
            p[2] = b;
            p[3] = 255;
        }
    }
}

}  // namespace

void draw_test_pattern(uint8_t* rgba, uint32_t width, uint32_t height, uint64_t n) {
    const uint32_t shift = uint32_t(n);
    const uint32_t bar = width ? uint32_t((n * 8) % width) : 0;
    for (uint32_t y = 0; y < height; ++y) {
        uint8_t* row = rgba + size_t(y) * width * 4;
        for (uint32_t x = 0; x < width; ++x) {
            uint8_t* p = row + size_t(x) * 4;
            const uint32_t sx = x + shift;
            const bool on_bar = x >= bar && x < bar + 8;
            p[0] = on_bar ? 255 : uint8_t(sx);
            p[1] = on_bar ? 255 : uint8_t(y + shift / 2);
            p[2] = on_bar ? 255 : uint8_t((sx + y) / 2);
            p[3] = 255;
        }
    }
    // Swatches, then the counter bits.
    const uint32_t b = kPatternBlock;
    fill_block(rgba, width, height, 0, 0, b, 255, 0, 0);
    fill_block(rgba, width, height, b, 0, b, 0, 255, 0);
    fill_block(rgba, width, height, 2 * b, 0, b, 0, 0, 255);
    for (uint32_t i = 0; i < 32; ++i) {
        const bool bit = ((n >> (31 - i)) & 1) != 0;
        const uint8_t v = bit ? 255 : 0;
        fill_block(rgba, width, height, (3 + i) * b, 0, b, v, v, v);
    }
}

int64_t read_test_pattern_counter(const uint8_t* rgba, uint32_t width, uint32_t height, uint32_t stride) {
    const uint32_t b = kPatternBlock;
    if (width < 35 * b || height < b) return -1;
    uint64_t v = 0;
    for (uint32_t i = 0; i < 32; ++i) {
        // Sample the block's centre: robust to a lossy codec's edges.
        const uint8_t* p = rgba + size_t(b / 2) * stride + size_t((3 + i) * b + b / 2) * 4;
        const int lum = (p[0] + p[1] + p[2]) / 3;
        v = (v << 1) | (lum >= 128 ? 1u : 0u);
    }
    return int64_t(v);
}

}  // namespace testkit
