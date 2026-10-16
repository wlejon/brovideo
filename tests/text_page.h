#pragma once
// Desktop-like test pictures: a light page of dark text-like strokes (1-2 px,
// the detail rate control starves first) on coloured panels, and one line of
// it redrawn, as a terminal rewrites a progress line.

#include <algorithm>
#include <cstdint>
#include <vector>

namespace testkit {

// Lines of the page sit every kLinePitch rows, glyphs kGlyphHeight tall.
inline constexpr uint32_t kLinePitch = 14;
inline constexpr uint32_t kGlyphHeight = 10;

struct PageRng {
    uint32_t state;
    explicit PageRng(uint32_t seed) : state(seed * 2654435761u + 1) {}
    uint32_t operator()() { return (state = state * 1664525u + 1013904223u) >> 8; }
};

inline void fill_rect(uint8_t* px, uint32_t w, uint32_t h, uint32_t x0, uint32_t y0, uint32_t rw, uint32_t rh,
                      uint8_t r, uint8_t g, uint8_t b) {
    for (uint32_t y = y0; y < std::min(h, y0 + rh); ++y) {
        for (uint32_t x = x0; x < std::min(w, x0 + rw); ++x) {
            uint8_t* p = &px[(size_t(y) * w + x) * 4];
            p[0] = r, p[1] = g, p[2] = b;
        }
    }
}

// Glyphs from x to `end` on the line whose glyphs start at row `ly`: 7x10
// cells of random strokes, 8 px apart, with a space now and then.
inline void draw_glyphs(uint8_t* px, uint32_t w, uint32_t h, uint32_t ly, uint32_t x, uint32_t end, PageRng& next) {
    while (x + 7 < end) {
        const uint32_t glyph = next();
        for (int s = 0; s < 4; ++s) {
            const uint32_t bits = glyph >> (s * 5);
            if (bits & 1) fill_rect(px, w, h, x + (bits >> 1) % 6, ly, 1 + (bits >> 4 & 1), kGlyphHeight, 30, 30, 40);
            else fill_rect(px, w, h, x, ly + (bits >> 1) % 9, 6, 1, 30, 30, 40);
        }
        x += 8;
        if (next() % 7 == 0) x += 8;
    }
}

// The row the first line of a page with this seed starts at.
inline uint32_t first_line(uint32_t seed) { return 8 + seed % 7; }

// Each seed lays the page out differently, so one seed after another is a
// big change, a window opening.
inline std::vector<uint8_t> text_page(uint32_t w, uint32_t h, uint32_t seed) {
    std::vector<uint8_t> px(size_t(w) * h * 4, 255);
    PageRng next(seed);
    for (int i = 0; i < 6; ++i) {
        // (The draws in one argument list, as the pages have always been made.)
        fill_rect(px.data(), w, h, next() % w, next() % h, 200 + next() % 600, 100 + next() % 400,
                  uint8_t(150 + next() % 100), uint8_t(150 + next() % 100), uint8_t(150 + next() % 100));
    }
    for (uint32_t ly = first_line(seed); ly + kGlyphHeight < h; ly += kLinePitch) {
        const uint32_t x = 8 + next() % 40;
        const uint32_t end = w - 8 - next() % 300;
        draw_glyphs(px.data(), w, h, ly, x, end, next);
    }
    return px;
}

// Rewrites one line of a page in place: the band of rows [ly - 2, ly + 12)
// from x0 to x1 cleared to white, then new glyphs from `seed` (a ninja status
// line or a counter, rewritten as the build goes).
inline void rewrite_line(uint8_t* px, uint32_t w, uint32_t h, uint32_t ly, uint32_t x0, uint32_t x1, uint32_t seed) {
    fill_rect(px, w, h, x0, ly - 2, x1 - x0, kLinePitch, 255, 255, 255);
    PageRng next(seed + 0x9e3779b9u);
    draw_glyphs(px, w, h, ly, x0 + 4, x1 - 4, next);
}

}  // namespace testkit
