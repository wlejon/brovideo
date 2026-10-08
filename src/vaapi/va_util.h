#pragma once
// Small VA-API helpers shared by the encoder's parts: status reporting,
// parameter buffers that destroy themselves, and a bit writer for the packed
// headers (H.264/HEVC NAL units, AV1 OBUs).

#include <va/va.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace brovideo::vaapi {

// "what: <vaErrorStr> (status)".
std::string va_error(const char* what, VAStatus s);

// Sets *err and returns false when `s` is not VA_STATUS_SUCCESS.
inline bool va_check(VAStatus s, const char* what, std::string* err) {
    if (s == VA_STATUS_SUCCESS) return true;
    if (err) *err = va_error(what, s);
    return false;
}

// The parameter buffers of one vaBeginPicture/vaEndPicture, destroyed
// together once the picture has been submitted.
class BufferList {
public:
    BufferList(VADisplay dpy, VAContextID ctx) : dpy_(dpy), ctx_(ctx) {}
    ~BufferList() { clear(); }
    BufferList(const BufferList&) = delete;
    BufferList& operator=(const BufferList&) = delete;

    // A buffer of `type` holding a copy of `data`.
    bool add(VABufferType type, const void* data, size_t size, std::string* err);
    template <class T>
    bool add(VABufferType type, const T& value, std::string* err) {
        return add(type, &value, sizeof(T), err);
    }
    // A misc parameter buffer: VAEncMiscParameterBuffer header + `payload`.
    template <class T>
    bool add_misc(VAEncMiscParameterType type, const T& payload, std::string* err) {
        return add_misc_raw(type, &payload, sizeof(T), err);
    }
    // A packed header: the parameter buffer and the data buffer. `bits` is
    // the payload length in bits. `emulation`: the data already carries
    // emulation prevention bytes (H.264/HEVC; AV1 has none).
    bool add_packed(uint32_t packed_type, const std::vector<uint8_t>& data, size_t bits, std::string* err,
                    bool emulation = true);

    // vaRenderPicture of every buffer added so far.
    bool render(std::string* err);
    void clear();

private:
    bool add_misc_raw(VAEncMiscParameterType type, const void* payload, size_t size, std::string* err);

    VADisplay dpy_;
    VAContextID ctx_;
    std::vector<VABufferID> ids_;
};

// MSB-first bit writer for packed headers.
class BitWriter {
public:
    void u(uint32_t bits, uint64_t value);  // `bits` <= 64
    void flag(bool b) { u(1, b ? 1 : 0); }
    void ue(uint32_t value);                 // Exp-Golomb
    void se(int32_t value);
    void leb128(uint64_t value);             // AV1, byte aligned
    void trailing_bits();                    // rbsp_trailing_bits: a 1, then zeros to a byte boundary
    void align_zero();                       // zeros to a byte boundary
    [[nodiscard]] bool byte_aligned() const { return bit_count_ % 8 == 0; }
    [[nodiscard]] size_t bit_count() const { return bit_count_; }
    [[nodiscard]] const std::vector<uint8_t>& bytes() const { return bytes_; }
    void append_bytes(const std::vector<uint8_t>& b);  // requires byte alignment

private:
    std::vector<uint8_t> bytes_;
    size_t bit_count_ = 0;
};

// An Annex B NAL unit: start code, the header bytes, then the RBSP with
// emulation prevention bytes inserted.
std::vector<uint8_t> annexb_nal(const std::vector<uint8_t>& header, const std::vector<uint8_t>& rbsp);

}  // namespace brovideo::vaapi
