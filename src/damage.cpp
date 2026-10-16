#include "damage.h"

#include <algorithm>
#include <cmath>

namespace brovideo::detail {

namespace {

struct Box {
    uint32_t x0, y0, x1, y1;  // half-open
    [[nodiscard]] uint64_t area() const { return uint64_t(x1 - x0) * (y1 - y0); }
};

Box merged(const Box& a, const Box& b) {
    return {std::min(a.x0, b.x0), std::min(a.y0, b.y0), std::max(a.x1, b.x1), std::max(a.y1, b.y1)};
}

bool overlap(const Box& a, const Box& b) { return a.x0 < b.x1 && b.x0 < a.x1 && a.y0 < b.y1 && b.y0 < a.y1; }

// The pixels the boxes cover, counting overlaps once: a sweep over the
// distinct column edges (there are few boxes).
uint64_t covered(const std::vector<Box>& boxes) {
    std::vector<uint32_t> xs;
    for (const Box& b : boxes) {
        xs.push_back(b.x0);
        xs.push_back(b.x1);
    }
    std::sort(xs.begin(), xs.end());
    xs.erase(std::unique(xs.begin(), xs.end()), xs.end());
    uint64_t total = 0;
    std::vector<std::pair<uint32_t, uint32_t>> spans;
    for (size_t i = 0; i + 1 < xs.size(); ++i) {
        spans.clear();
        for (const Box& b : boxes) {
            if (b.x0 <= xs[i] && b.x1 >= xs[i + 1]) spans.emplace_back(b.y0, b.y1);
        }
        std::sort(spans.begin(), spans.end());
        uint32_t end = 0;
        uint64_t rows = 0;
        for (const auto& [y0, y1] : spans) {
            const uint32_t from = std::max(y0, end);
            if (y1 > from) rows += y1 - from;
            end = std::max(end, y1);
        }
        total += rows * (xs[i + 1] - xs[i]);
    }
    return total;
}

}  // namespace

QualityBoost damage_boost(const Frame& f, bool keyframe, uint32_t width, uint32_t height, uint32_t block,
                          size_t max_regions, double scale) {
    QualityBoost out;
    if (keyframe || !f.damage_known || f.damage.empty() || f.width == 0 || f.height == 0 || width == 0 ||
        height == 0 || block == 0 || max_regions == 0) {
        return out;
    }
    // Into picture pixels (the encoder scales a frame of another size), out
    // to the block grid: a block the damage touches at all is boosted.
    std::vector<Box> boxes;
    for (const Rect& r : f.damage) {
        if (r.width == 0 || r.height == 0 || r.x >= f.width || r.y >= f.height) continue;
        const uint64_t rx1 = std::min<uint64_t>(uint64_t(r.x) + r.width, f.width);
        const uint64_t ry1 = std::min<uint64_t>(uint64_t(r.y) + r.height, f.height);
        const uint64_t x0 = uint64_t(r.x) * width / f.width, y0 = uint64_t(r.y) * height / f.height;
        const uint64_t x1 = (rx1 * width + f.width - 1) / f.width, y1 = (ry1 * height + f.height - 1) / f.height;
        Box b{uint32_t(x0 / block * block), uint32_t(y0 / block * block),
              uint32_t(std::min<uint64_t>((x1 + block - 1) / block * block, width)),
              uint32_t(std::min<uint64_t>((y1 + block - 1) / block * block, height))};
        if (b.x1 > b.x0 && b.y1 > b.y0) boxes.push_back(b);
    }
    if (boxes.empty()) return out;
    // Overlapping boxes become one; then, while there are too many, the pair
    // whose union adds the least area.
    for (bool again = true; again;) {
        again = false;
        for (size_t i = 0; i < boxes.size() && !again; ++i) {
            for (size_t j = i + 1; j < boxes.size(); ++j) {
                if (overlap(boxes[i], boxes[j])) {
                    boxes[i] = merged(boxes[i], boxes[j]);
                    boxes.erase(boxes.begin() + std::ptrdiff_t(j));
                    again = true;
                    break;
                }
            }
        }
    }
    while (boxes.size() > max_regions) {
        size_t bi = 0, bj = 1;
        uint64_t best = UINT64_MAX;
        for (size_t i = 0; i < boxes.size(); ++i) {
            for (size_t j = i + 1; j < boxes.size(); ++j) {
                const uint64_t grow = merged(boxes[i], boxes[j]).area() - boxes[i].area() - boxes[j].area();
                if (grow < best) best = grow, bi = i, bj = j;
            }
        }
        boxes[bi] = merged(boxes[bi], boxes[bj]);
        boxes.erase(boxes.begin() + std::ptrdiff_t(bj));
    }
    out.area = double(covered(boxes)) / (double(width) * height);
    if (out.area >= kNoBoostArea) return out;
    const double t = std::clamp((out.area - kFullBoostArea) / (kNoBoostArea - kFullBoostArea), 0.0, 1.0);
    out.qp_delta = -int(std::lround(kMaxBoost * (1.0 - t) * std::clamp(scale, 0.0, 1.0)));
    if (out.qp_delta == 0) return out;
    for (const Box& b : boxes) out.regions.push_back(Rect{b.x0, b.y0, b.x1 - b.x0, b.y1 - b.y0});
    return out;
}

}  // namespace brovideo::detail
