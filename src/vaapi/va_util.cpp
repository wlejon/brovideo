#include "vaapi/va_util.h"

#include <cstring>

namespace brovideo::vaapi {

std::string va_error(const char* what, VAStatus s) {
    return std::string(what) + ": " + vaErrorStr(s) + " (" + std::to_string(int(s)) + ")";
}

bool BufferList::add(VABufferType type, const void* data, size_t size, std::string* err) {
    VABufferID id = VA_INVALID_ID;
    // vaCreateBuffer copies `data`; the const_cast is the API's, not a write.
    const VAStatus s = vaCreateBuffer(dpy_, ctx_, type, unsigned(size), 1, const_cast<void*>(data), &id);
    if (!va_check(s, "vaCreateBuffer", err)) return false;
    ids_.push_back(id);
    return true;
}

bool BufferList::add_misc_raw(VAEncMiscParameterType type, const void* payload, size_t size, std::string* err) {
    std::vector<uint8_t> buf(sizeof(VAEncMiscParameterBuffer) + size, 0);
    auto* misc = reinterpret_cast<VAEncMiscParameterBuffer*>(buf.data());
    misc->type = type;
    std::memcpy(buf.data() + offsetof(VAEncMiscParameterBuffer, data), payload, size);
    return add(VAEncMiscParameterBufferType, buf.data(), buf.size(), err);
}

bool BufferList::add_packed(uint32_t packed_type, const std::vector<uint8_t>& data, size_t bits, std::string* err,
                            bool emulation) {
    VAEncPackedHeaderParameterBuffer p{};
    p.type = packed_type;
    p.bit_length = uint32_t(bits);
    p.has_emulation_bytes = emulation ? 1 : 0;
    return add(VAEncPackedHeaderParameterBufferType, p, err) &&
           add(VAEncPackedHeaderDataBufferType, data.data(), data.size(), err);
}

bool BufferList::render(std::string* err) {
    if (ids_.empty()) return true;
    return va_check(vaRenderPicture(dpy_, ctx_, ids_.data(), int(ids_.size())), "vaRenderPicture", err);
}

void BufferList::clear() {
    for (VABufferID id : ids_) vaDestroyBuffer(dpy_, id);
    ids_.clear();
}

void BitWriter::u(uint32_t bits, uint64_t value) {
    for (uint32_t i = bits; i-- > 0;) {
        if (bit_count_ % 8 == 0) bytes_.push_back(0);
        if ((value >> i) & 1) bytes_.back() |= uint8_t(0x80 >> (bit_count_ % 8));
        ++bit_count_;
    }
}

void BitWriter::ue(uint32_t value) {
    const uint64_t v = uint64_t(value) + 1;
    uint32_t len = 0;
    while ((v >> len) > 1) ++len;
    u(len, 0);
    u(len + 1, v);
}

void BitWriter::se(int32_t value) {
    ue(value > 0 ? uint32_t(2 * int64_t(value) - 1) : uint32_t(-2 * int64_t(value)));
}

void BitWriter::leb128(uint64_t value) {
    do {
        uint8_t byte = value & 0x7f;
        value >>= 7;
        if (value) byte |= 0x80;
        u(8, byte);
    } while (value);
}

void BitWriter::trailing_bits() {
    u(1, 1);
    align_zero();
}

void BitWriter::align_zero() {
    while (bit_count_ % 8) u(1, 0);
}

void BitWriter::append_bytes(const std::vector<uint8_t>& b) {
    bytes_.insert(bytes_.end(), b.begin(), b.end());
    bit_count_ += b.size() * 8;
}

std::vector<uint8_t> annexb_nal(const std::vector<uint8_t>& header, const std::vector<uint8_t>& rbsp) {
    std::vector<uint8_t> out{0, 0, 0, 1};
    out.insert(out.end(), header.begin(), header.end());
    int zeros = 0;
    for (uint8_t b : rbsp) {
        if (zeros >= 2 && b <= 3) {
            out.push_back(3);
            zeros = 0;
        }
        out.push_back(b);
        zeros = b == 0 ? zeros + 1 : 0;
    }
    return out;
}

}  // namespace brovideo::vaapi
