#pragma once
// Damage-aware quality, the policy every backend shares: which regions of a
// frame to code finer than rate control would, and by how much, from the
// frame's damage (Frame::damage_known / damage).
//
// Constant-bitrate rate control sizes a frame by the per-frame budget, and a
// low-latency stream keeps the buffer it may borrow from short (VA-API: 50
// ms), so a big change lands soft and sharpens over the predicted frames and
// repeats after it. That is right for a big change, but a source that sends
// frames only on change also sends a small change (a line of text being
// rewritten, a counter) as a frame of its own, and a region that changes
// again before it has sharpened is shown soft for as long as it keeps
// changing. When the damage is small, coding just that region finer costs
// little and it lands sharp at once; when it is big, the boost tapers off to
// nothing, so a window opening stays as fast (as small a packet) as before.
// See docs/design.md, "Damage".

#include "brovideo/frame.h"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace brovideo::detail {

// The boost: QP steps finer than rate control's choice for the frame. The
// finest, kMaxBoost, for a damage of at most kFullBoostArea of the picture
// (a few lines of text: right after a big change rate control has the
// frame near the coarsest QP, and it takes this much to make text
// readable); it falls linearly to none at kNoBoostArea, where a frame is a
// big change and rate control's own choice stands.
inline constexpr int kMaxBoost = 30;
inline constexpr double kFullBoostArea = 0.02;
inline constexpr double kNoBoostArea = 0.40;

struct QualityBoost {
    int qp_delta = 0;            // <= 0: QP steps for the regions (0: no boost)
    std::vector<Rect> regions;   // in picture pixels, on the block grid, within the picture
    double area = 0;             // the fraction of the picture the regions cover
};

// The boost for frame `f` encoded as a `width` x `height` picture (the
// damage is scaled from the frame's size), with regions on a `block` grid
// (the codec's QP granularity), at most `max_regions` of them (more are
// merged, each into the one it grows least), scaled by `scale` (0 to 1,
// BoostBudget::scale()). None when the damage is unknown, empty or too big,
// and for a keyframe.
QualityBoost damage_boost(const Frame& f, bool keyframe, uint32_t width, uint32_t height, uint32_t block,
                          size_t max_regions, double scale = 1.0);

// Keeps the boost within the bitrate. A boosted frame is bigger than rate
// control would have made it, and rate control does not take all of that
// back from the frames after it, so damage that stays small but keeps
// coming fast (many lines rewritten every frame) would push the stream over
// its bitrate for as long as it lasts. This counts what the stream spent
// against what the bitrate allows per packet (every packet: frames, repeats,
// keyframes); while it is over by up to kTolerance of a second's bitrate
// the boost tapers, and past that there is none until the stream is back
// under. A short burst (a build's status line for a few seconds) is boosted
// in full.
class BoostBudget {
public:
    static constexpr double kTolerance = 0.5;  // seconds of the bitrate

    void configure(uint32_t bitrate_kbps, uint32_t fps) {
        per_packet_ = double(bitrate_kbps) * 1000.0 / 8.0 / double(fps ? fps : 60);
        tolerance_ = double(bitrate_kbps) * 1000.0 / 8.0 * kTolerance;
        bank_ = 0;
    }
    void spent(size_t bytes) {
        bank_ += per_packet_ - double(bytes);
        if (bank_ > tolerance_) bank_ = tolerance_;
        if (bank_ < -tolerance_) bank_ = -tolerance_;
    }
    // 1 while the stream is within its bitrate, falling to 0 at the tolerance.
    [[nodiscard]] double scale() const {
        if (tolerance_ <= 0 || bank_ >= 0) return 1.0;
        return 1.0 + bank_ / tolerance_;
    }

private:
    double per_packet_ = 0, tolerance_ = 0, bank_ = 0;
};

}  // namespace brovideo::detail
