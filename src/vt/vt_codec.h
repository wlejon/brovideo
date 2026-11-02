#pragma once
// macOS VideoToolbox hardware/software H.264 and HEVC encoder & decoder.

#include "backend.h"
#include "brovideo/decoder.h"
#include "brovideo/encoder.h"

#include <CoreFoundation/CoreFoundation.h>
#include <CoreMedia/CoreMedia.h>
#include <CoreVideo/CoreVideo.h>
#include <VideoToolbox/VideoToolbox.h>

#include <cstdint>
#include <functional>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace brovideo::vt {

class VtEncoder final : public Encoder {
public:
    static std::unique_ptr<VtEncoder> create(const EncoderConfig& config, std::string* err);
    ~VtEncoder() override;

    EncoderInfo info() const override;
    [[nodiscard]] bool hardware() const noexcept { return hardware_; }

protected:
    bool encode_frame(const Frame& frame, bool keyframe, const std::function<void()>& release, Packet& out,
                      std::string* err) override;
    bool apply_config(const EncoderConfig& next, std::string* err) override;
    bool repeats() const override { return true; }
    bool encode_repeat_frame(bool keyframe, Packet& out, std::string* err) override;

private:
    explicit VtEncoder(const EncoderConfig& config) : Encoder(config) {}

    bool open_session(const EncoderConfig& cfg, std::string* err);
    void close_session();
    bool convert_to_nv12(const Frame& frame, std::string* err);
    bool encode_nv12(bool keyframe, Packet& out, std::string* err);

    static void compression_callback(void* output_callback_ref_con, void* source_frame_ref_con, OSStatus status,
                                     VTEncodeInfoFlags info_flags, CMSampleBufferRef sample_buffer);

    VTCompressionSessionRef session_ = nullptr;
    bool hardware_ = false;
    uint32_t enc_w_ = 0;
    uint32_t enc_h_ = 0;
    uint32_t fps_ = 60;
    int64_t frame_index_ = 0;
    std::vector<uint8_t> nv12_;

    // Populated synchronously during VTCompressionSessionCompleteFrames.
    OSStatus cb_status_ = noErr;
    bool cb_got_packet_ = false;
    Packet cb_packet_;
};

class VtDecoder final : public Decoder {
public:
    static std::unique_ptr<VtDecoder> create(Codec codec, Hardware hw_mode, PictureMemory output, std::string* err);
    ~VtDecoder() override;

    bool decode(std::span<const uint8_t> packet, int64_t pts_ns, Picture& out, std::string* err) override;
    using Decoder::decode;
    [[nodiscard]] std::string describe() const override;
    [[nodiscard]] bool hardware() const override { return hardware_; }

private:
    VtDecoder(Codec codec, Hardware hw_mode, PictureMemory output)
        : Decoder(codec), hw_mode_(hw_mode), output_(output) {}

    bool ensure_format_and_session(const std::vector<std::span<const uint8_t>>& nals, std::string* err);
    bool open_session(CMVideoFormatDescriptionRef fmt, std::string* err);
    void close_session();
    void reset_after_error();

    static void decompression_callback(void* decompression_output_ref_con, void* source_frame_ref_con, OSStatus status,
                                       VTDecodeInfoFlags info_flags, CVImageBufferRef image_buffer,
                                       CMTime presentation_time_stamp, CMTime presentation_duration);

    Hardware hw_mode_ = Hardware::Prefer;
    PictureMemory output_ = PictureMemory::Cpu;
    VTDecompressionSessionRef session_ = nullptr;
    CMVideoFormatDescriptionRef format_desc_ = nullptr;
    bool hardware_ = false;
    bool synced_ = false;

    std::vector<uint8_t> vps_;
    std::vector<uint8_t> sps_;
    std::vector<uint8_t> pps_;

    OSStatus cb_status_ = noErr;
    Picture cb_picture_;
};

}  // namespace brovideo::vt
