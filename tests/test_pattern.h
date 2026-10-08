#pragma once
// The test pattern (the same one broremote's serve-test draws, so streams
// recorded there decode against it): CPU RGBA frames that make motion and colour errors
// visible. A diagonal gradient scrolls one pixel per frame (red follows x,
// green follows y, blue follows their sum), a white bar sweeps across, and
// the frame counter is drawn along the top as 32 blocks, one bit each (white
// = 1, most significant bit first), framed by pure red, green and blue
// swatches so a channel swap shows at once.

#include <cstdint>
#include <vector>

namespace testkit {

inline constexpr uint32_t kPatternBlock = 16;  // counter block size in pixels

// Fill `rgba` (width * height * 4 bytes, tightly packed) with frame `n`.
void draw_test_pattern(uint8_t* rgba, uint32_t width, uint32_t height, uint64_t n);

// Read the frame counter back from a decoded picture (the inverse of the
// counter blocks); -1 when the picture is too small to hold it.
int64_t read_test_pattern_counter(const uint8_t* rgba, uint32_t width, uint32_t height, uint32_t stride);

}  // namespace testkit
