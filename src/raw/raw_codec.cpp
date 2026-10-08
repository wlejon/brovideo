// Codec::Raw: CPU RGBA frames, run-length coded, with XOR-delta predicted
// frames between keyframes. It exists so a consumer's protocol, server and
// client run and are tested everywhere (CI, Windows, a box with no hardware
// encoder); it is not for real use. The format (docs/raw.md):
//
//   u8 kind            0 = intra, 1 = delta (XOR with the previous picture)
//   varint width, varint height
//   tokens until width * height pixels are covered:
//     varint t, n = (t >> 1) + 1
//     t & 1: a run, one 4-byte pixel repeated n times
//     else:  a literal, n pixels of 4 bytes
//
// A pixel is 4 bytes R G B A (a BGRA8 frame is swizzled on the way in); in a
// delta frame it is XORed with the same pixel of the previous picture, so
// unchanged areas are runs of zero. Varints are LEB128.
#include "backend.h"

#include <cstring>

namespace brovideo::detail {

namespace {

constexpr uint8_t kIntra = 0;
constexpr uint8_t kDelta = 1;
constexpr uint64_t kMaxDimension = 16384;
// Largest picture the decoder allocates (a 1 GiB picture from a few bytes of
// hostile input is not something a test codec needs to allow).
constexpr uint64_t kMaxPixels = uint64_t(8192) * 8192;
constexpr size_t kMinRun = 3;

inline uint32_t px(const uint8_t* p, size_t i) {
    uint32_t v;
    std::memcpy(&v, p + i * 4, 4);
    return v;
}

void put_varint(std::vector<uint8_t>& out, uint64_t v) {
    while (v >= 0x80) {
        out.push_back(uint8_t(v) | 0x80);
        v >>= 7;
    }
    out.push_back(uint8_t(v));
}

void put_literal(std::vector<uint8_t>& out, const uint8_t* pixels, size_t from, size_t to) {
    if (from == to) return;
    put_varint(out, uint64_t(to - from - 1) << 1);
    out.insert(out.end(), pixels + from * 4, pixels + to * 4);
}

// Run-length code `n` pixels.
void rle(const uint8_t* pixels, size_t n, std::vector<uint8_t>& out) {
    size_t lit = 0;
    size_t i = 0;
    while (i < n) {
        const uint32_t v = px(pixels, i);
        size_t r = 1;
        while (i + r < n && px(pixels, i + r) == v) ++r;
        if (r >= kMinRun) {
            put_literal(out, pixels, lit, i);
            put_varint(out, (uint64_t(r - 1) << 1) | 1);
            out.insert(out.end(), pixels + i * 4, pixels + i * 4 + 4);
            i += r;
            lit = i;
        } else {
            i += r;
        }
    }
    put_literal(out, pixels, lit, n);
}

// A bounds-checked reader over one packet: any overrun marks it failed.
class Reader {
public:
    explicit Reader(std::span<const uint8_t> b) : b_(b) {}
    [[nodiscard]] bool ok() const { return ok_; }
    uint8_t u8() {
        if (!need(1)) return 0;
        return b_[pos_++];
    }
    uint64_t varint() {
        uint64_t v = 0;
        for (int shift = 0; shift < 64; shift += 7) {
            if (!need(1)) return 0;
            const uint8_t b = b_[pos_++];
            if (shift == 63 && b > 1) break;  // overflow
            v |= uint64_t(b & 0x7F) << shift;
            if (!(b & 0x80)) return v;
        }
        ok_ = false;
        return 0;
    }
    uint64_t varint_max(uint64_t limit) {
        const uint64_t v = varint();
        if (v > limit) {
            ok_ = false;
            return 0;
        }
        return v;
    }
    const uint8_t* bytes(size_t n) {
        if (!need(n)) return nullptr;
        const uint8_t* p = b_.data() + pos_;
        pos_ += n;
        return p;
    }

private:
    bool need(size_t n) {
        if (!ok_ || b_.size() - pos_ < n) {
            ok_ = false;
            return false;
        }
        return true;
    }
    std::span<const uint8_t> b_;
    size_t pos_ = 0;
    bool ok_ = true;
};

class RawEncoder final : public Encoder {
public:
    explicit RawEncoder(const EncoderConfig& c) : Encoder(c) {}

    EncoderInfo info() const override {
        EncoderInfo i;
        i.backend = Backend::Raw;
        i.description = "raw (CPU)";
        return i;
    }

protected:
    bool encode_frame(const Frame& frame, bool keyframe, const std::function<void()>& release, Packet& out,
                      std::string* err) override {
        if (!frame.is_cpu() || !frame.cpu) {
            release();
            if (err) *err = "the raw codec takes CPU frames only";
            return false;
        }
        if (frame.cpu_format != PixelFormat::RGBA8 && frame.cpu_format != PixelFormat::BGRA8) {
            release();
            if (err) *err = "the raw codec takes RGBA8 or BGRA8 frames";
            return false;
        }
        if (frame.width != config_.width || frame.height != config_.height) {
            release();
            if (err) *err = "frame size differs from the encoder's";
            return false;
        }
        const size_t w = frame.width, h = frame.height, row = w * 4;
        cur_.resize(row * h);
        for (size_t y = 0; y < h; ++y) std::memcpy(cur_.data() + y * row, frame.cpu + y * frame.cpu_row_bytes(), row);
        release();  // everything below reads our copy
        if (frame.cpu_format == PixelFormat::BGRA8) {
            for (size_t i = 0; i < cur_.size(); i += 4) std::swap(cur_[i], cur_[i + 2]);
        }

        const bool intra = keyframe || prev_.size() != cur_.size();
        const uint8_t* src = cur_.data();
        if (!intra) {
            delta_.resize(cur_.size());
            for (size_t i = 0; i < cur_.size(); ++i) delta_[i] = uint8_t(cur_[i] ^ prev_[i]);
            src = delta_.data();
        }
        out.data.reserve(64 + cur_.size() / 8);
        out.data.push_back(intra ? kIntra : kDelta);
        put_varint(out.data, w);
        put_varint(out.data, h);
        rle(src, w * h, out.data);
        out.keyframe = intra;
        prev_.swap(cur_);
        return true;
    }

    bool apply_config(const EncoderConfig&, std::string*) override {
        prev_.clear();  // the next frame is intra anyway
        return true;
    }

private:
    std::vector<uint8_t> cur_, prev_, delta_;
};

class RawDecoder final : public Decoder {
public:
    RawDecoder() : Decoder(Codec::Raw) {}

    bool decode(std::span<const uint8_t> packet, int64_t pts_ns, Picture& out, std::string* err) override {
        out.ready = false;
        Reader r(packet);
        const uint8_t kind = r.u8();
        const uint64_t w = r.varint_max(kMaxDimension);
        const uint64_t h = r.varint_max(kMaxDimension);
        if (!r.ok() || kind > kDelta || w == 0 || h == 0 || w * h > kMaxPixels) {
            return fail(err, "malformed raw packet header");
        }
        const size_t n = size_t(w * h);
        if (kind == kDelta && (pic_.size() != n * 4 || w != width_)) {
            return fail(err, "raw delta frame without a matching reference (lost sync)");
        }
        next_.resize(n * 4);
        size_t i = 0;
        while (i < n) {
            const uint64_t t = r.varint();
            if (!r.ok()) return fail(err, "truncated raw packet");
            const uint64_t count = (t >> 1) + 1;
            if (count > n - i) return fail(err, "raw token runs past the picture");
            if (t & 1) {
                const uint8_t* p = r.bytes(4);
                if (!p) return fail(err, "truncated raw packet");
                for (uint64_t k = 0; k < count; ++k) std::memcpy(next_.data() + (i + k) * 4, p, 4);
            } else {
                const uint8_t* p = r.bytes(size_t(count) * 4);
                if (!p) return fail(err, "truncated raw packet");
                std::memcpy(next_.data() + i * 4, p, size_t(count) * 4);
            }
            i += size_t(count);
        }
        if (kind == kDelta) {
            for (size_t k = 0; k < next_.size(); ++k) next_[k] ^= pic_[k];
        }
        pic_.swap(next_);
        width_ = uint32_t(w);
        height_ = uint32_t(h);
        out.ready = true;
        out.width = width_;
        out.height = height_;
        out.format = PixelFormat::RGBA8;
        out.memory = PictureMemory::Cpu;
        out.stride = width_ * 4;
        out.uv_offset = 0;
        out.data = pic_;
        out.gpu = GpuPicture{};
        out.pts_ns = pts_ns;
        return true;
    }

    std::string describe() const override { return "raw (CPU)"; }
    bool hardware() const override { return false; }

private:
    bool fail(std::string* err, const char* what) {
        if (err) *err = what;
        pic_.clear();  // a predicted frame after an error must not decode against a stale picture
        return false;
    }

    std::vector<uint8_t> pic_, next_;
    uint32_t width_ = 0;
    uint32_t height_ = 0;
};

ProbeResult probe_raw() {
    ProbeResult r;
    for (Direction d : {Direction::Encode, Direction::Decode}) {
        Capability c;
        c.codec = Codec::Raw;
        c.direction = d;
        c.backend = Backend::Raw;
        c.max_width = c.max_height = uint32_t(kMaxDimension);
        c.device = "CPU";
        r.caps.push_back(c);
    }
    return r;
}

std::unique_ptr<Encoder> make_encoder(const EncoderConfig& c, const Capability&, std::string*) {
    return std::make_unique<RawEncoder>(c);
}

std::unique_ptr<Decoder> make_decoder(const DecoderConfig&, const Capability&, std::string*) {
    return std::make_unique<RawDecoder>();
}

}  // namespace

const BackendOps& raw_backend() {
    static const BackendOps ops{Backend::Raw, probe_raw, make_encoder, make_decoder};
    return ops;
}

}  // namespace brovideo::detail
