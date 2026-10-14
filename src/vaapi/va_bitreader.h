#pragma once
// Bitstream reader for VA-API decoding: Annex B NAL splitting, RBSP
// unemulation (tracking emulation prevention byte counts for HEVC), Exp-Golomb
// codes, and AV1 bit primitives (su, ns, uvlc, leb128).

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace brovideo::vaapi {

// Splits an Annex B access unit into its NAL units (each span starts at the
// first byte of the NAL header and ends where the next 3- or 4-byte start code
// begins).
inline std::vector<std::span<const uint8_t>> split_annexb_nals(std::span<const uint8_t> b) {
    std::vector<std::span<const uint8_t>> out;
    const size_t n = b.size();
    std::vector<size_t> starts;
    std::vector<size_t> headers;
    for (size_t i = 0; i + 3 <= n; ++i) {
        if (b[i] == 0 && b[i + 1] == 0 && b[i + 2] == 1) {
            const size_t sc = (i > 0 && b[i - 1] == 0) ? i - 1 : i;
            starts.push_back(sc);
            headers.push_back(i + 3);
            i += 2;
        }
    }
    for (size_t k = 0; k < starts.size(); ++k) {
        const size_t hdr = headers[k];
        const size_t end = (k + 1 < starts.size()) ? starts[k + 1] : n;
        if (hdr < end) out.push_back(b.subspan(hdr, end - hdr));
    }
    return out;
}

class BitReader {
public:
    // Construct over raw bytes (AV1 OBUs) or with 00 00 03 unemulation (H.264/HEVC RBSP).
    BitReader(std::span<const uint8_t> raw, bool unemulate_rbsp, size_t max_unemulate_bytes = 0) {
        if (!unemulate_rbsp) {
            data_ = raw;
            compute_rbsp_end();
            return;
        }
        const size_t limit = max_unemulate_bytes ? std::min(raw.size(), max_unemulate_bytes) : raw.size();
        rbsp_.reserve(limit);
        emu_prefix_.reserve(limit + 1);
        emu_prefix_.push_back(0);
        int zeros = 0;
        uint32_t emu = 0;
        for (size_t i = 0; i < limit; ++i) {
            const uint8_t byte = raw[i];
            if (zeros >= 2 && byte == 0x03) {
                ++emu;
                zeros = 0;
                continue;
            }
            rbsp_.push_back(byte);
            emu_prefix_.push_back(emu);
            zeros = (byte == 0) ? zeros + 1 : 0;
        }
        data_ = rbsp_;
        compute_rbsp_end();
    }

    [[nodiscard]] bool ok() const noexcept { return ok_; }
    [[nodiscard]] size_t bit_pos() const noexcept { return bit_pos_; }
    [[nodiscard]] size_t byte_pos() const noexcept { return bit_pos_ / 8; }
    [[nodiscard]] size_t size_bits() const noexcept { return data_.size() * 8; }

    [[nodiscard]] uint32_t emulation_bytes_in_prefix(size_t rbsp_bytes) const noexcept {
        if (emu_prefix_.empty()) return 0;
        const size_t idx = std::min(rbsp_bytes, emu_prefix_.size() - 1);
        return emu_prefix_[idx];
    }

    uint32_t u(uint32_t bits) noexcept {
        if (bits == 0) return 0;
        if (bits > 32 || bit_pos_ + bits > data_.size() * 8) {
            ok_ = false;
            return 0;
        }
        uint32_t val = 0;
        for (uint32_t i = 0; i < bits; ++i) {
            const uint8_t byte = data_[bit_pos_ >> 3];
            const uint32_t bit = (byte >> (7 - (bit_pos_ & 7))) & 1u;
            val = (val << 1) | bit;
            ++bit_pos_;
        }
        return val;
    }

    bool flag() noexcept { return u(1) != 0; }

    uint32_t ue() noexcept {
        uint32_t zeros = 0;
        while (bit_pos_ < data_.size() * 8 && !flag()) {
            if (++zeros > 31) {
                ok_ = false;
                return 0;
            }
        }
        if (!ok_) return 0;
        if (zeros == 0) return 0;
        const uint32_t suffix = u(zeros);
        return ((1u << zeros) - 1u) + suffix;
    }

    int32_t se() noexcept {
        const uint32_t v = ue();
        if (v == 0) return 0;
        return (v & 1u) ? int32_t((v + 1u) >> 1) : -int32_t(v >> 1);
    }

    int32_t su(uint32_t bits) noexcept {
        const uint32_t v = u(bits);
        const uint32_t sign_mask = 1u << (bits - 1);
        if (v & sign_mask) return int32_t(v) - int32_t(1u << bits);
        return int32_t(v);
    }

    uint32_t ns(uint32_t n) noexcept {
        if (n <= 1) return 0;
        uint32_t w = 0;
        uint32_t temp = n;
        while (temp > 0) {
            temp >>= 1;
            ++w;
        }
        const uint32_t m = (1u << w) - n;
        const uint32_t v = u(w - 1);
        if (v < m) return v;
        const uint32_t extra = u(1);
        return (v << 1) - m + extra;
    }

    uint32_t uvlc() noexcept {
        uint32_t leading_zeros = 0;
        while (!flag()) {
            if (++leading_zeros >= 32 || !ok_) return 0xffffffffu;
        }
        if (leading_zeros >= 32) return 0xffffffffu;
        const uint32_t value = u(leading_zeros);
        return value + (1u << leading_zeros) - 1u;
    }

    void align_byte() noexcept {
        while (bit_pos_ & 7) {
            if (flag()) ok_ = false;
        }
    }

    void hevc_byte_alignment() noexcept {
        if (!flag()) ok_ = false;
        while (bit_pos_ & 7) {
            if (flag()) ok_ = false;
        }
    }

    [[nodiscard]] bool more_rbsp_data() const noexcept { return ok_ && bit_pos_ < rbsp_end_bit_; }

private:
    void compute_rbsp_end() noexcept {
        rbsp_end_bit_ = 0;
        for (size_t i = data_.size(); i-- > 0;) {
            if (data_[i] != 0) {
                for (int b = 0; b < 8; ++b) {
                    if (data_[i] & (1u << b)) {
                        rbsp_end_bit_ = i * 8 + size_t(7 - b);
                        return;
                    }
                }
            }
        }
    }

    std::vector<uint8_t> rbsp_;
    std::vector<uint32_t> emu_prefix_;
    std::span<const uint8_t> data_;
    size_t bit_pos_ = 0;
    size_t rbsp_end_bit_ = 0;
    bool ok_ = true;
};

inline bool read_obu_leb128(std::span<const uint8_t> b, size_t& pos, uint64_t& value) noexcept {
    value = 0;
    for (int i = 0; i < 8; ++i) {
        if (pos >= b.size()) return false;
        const uint8_t byte = b[pos++];
        value |= uint64_t(byte & 0x7f) << (7 * i);
        if (!(byte & 0x80)) return true;
    }
    return false;
}

}  // namespace brovideo::vaapi
