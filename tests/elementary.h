#pragma once
// Elementary streams cut back into the packets an encoder made: Annex B
// H.264 / HEVC into access units, an AV1 OBU stream (ffmpeg's -f obu) into
// temporal units. Enough parsing for streams with one slice layout per
// picture; not a general demuxer.

#include "brovideo/codec.h"

#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace elementary {

using Packet = std::vector<uint8_t>;

inline std::vector<uint8_t> read_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    return std::vector<uint8_t>(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

namespace detail {

// Offsets of every start code (00 00 01, with a preceding 00 folded in).
inline std::vector<size_t> start_codes(const std::vector<uint8_t>& b) {
    std::vector<size_t> out;
    for (size_t i = 0; i + 3 <= b.size(); ++i) {
        if (b[i] == 0 && b[i + 1] == 0 && b[i + 2] == 1) {
            out.push_back(i > 0 && b[i - 1] == 0 ? i - 1 : i);
            i += 2;
        }
    }
    return out;
}

inline size_t header_offset(const std::vector<uint8_t>& b, size_t sc) { return b[sc + 2] == 1 ? sc + 3 : sc + 4; }

}  // namespace detail

// Annex B into access units.
inline std::vector<Packet> split_annexb(brovideo::Codec codec, const std::vector<uint8_t>& b) {
    const bool hevc = codec == brovideo::Codec::HEVC;
    const std::vector<size_t> starts = detail::start_codes(b);
    std::vector<Packet> out;
    Packet cur;
    bool cur_has_vcl = false;
    for (size_t k = 0; k < starts.size(); ++k) {
        const size_t begin = starts[k];
        const size_t end = k + 1 < starts.size() ? starts[k + 1] : b.size();
        const size_t h = detail::header_offset(b, begin);
        if (h >= end) continue;
        bool vcl = false, first_slice = false, prefix = false;
        if (hevc) {
            const int type = (b[h] >> 1) & 0x3F;
            vcl = type <= 31;
            first_slice = vcl && h + 2 < end && (b[h + 2] & 0x80);
            prefix = type == 32 || type == 33 || type == 34 || type == 35 || type == 39;
        } else {
            const int type = b[h] & 0x1F;
            vcl = type == 1 || type == 5;
            first_slice = vcl && h + 1 < end && (b[h + 1] & 0x80);  // first_mb_in_slice == 0
            prefix = type == 6 || type == 7 || type == 8 || type == 9;
        }
        if (cur_has_vcl && (prefix || first_slice)) {
            out.push_back(std::move(cur));
            cur.clear();
            cur_has_vcl = false;
        }
        cur.insert(cur.end(), b.begin() + std::ptrdiff_t(begin), b.begin() + std::ptrdiff_t(end));
        cur_has_vcl = cur_has_vcl || vcl;
    }
    if (!cur.empty()) out.push_back(std::move(cur));
    return out;
}

// An OBU stream into temporal units, each starting at a temporal delimiter.
inline std::vector<Packet> split_obu(const std::vector<uint8_t>& b) {
    std::vector<Packet> out;
    size_t pos = 0;
    while (pos < b.size()) {
        const size_t begin = pos;
        const uint8_t hdr = b[pos++];
        const int type = (hdr >> 3) & 0x0F;
        if (hdr & 0x04) ++pos;
        uint64_t size = 0;
        if (!(hdr & 0x02)) return out;  // ffmpeg always writes sizes
        for (int i = 0; i < 8 && pos < b.size(); ++i) {
            const uint8_t byte = b[pos++];
            size |= uint64_t(byte & 0x7F) << (7 * i);
            if (!(byte & 0x80)) break;
        }
        pos += size_t(size);
        if (pos > b.size()) break;
        if (type == 2 || out.empty()) out.emplace_back();
        out.back().insert(out.back().end(), b.begin() + std::ptrdiff_t(begin), b.begin() + std::ptrdiff_t(pos));
    }
    return out;
}

inline std::vector<Packet> split(brovideo::Codec codec, const std::vector<uint8_t>& b) {
    return codec == brovideo::Codec::AV1 ? split_obu(b) : split_annexb(codec, b);
}

}  // namespace elementary
