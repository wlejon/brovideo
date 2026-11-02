// macOS VideoToolbox H.264 / HEVC encoder implementation.
#include "vt/vt_codec.h"

#include <algorithm>
#include <cstring>

namespace brovideo::vt {

namespace {

bool fail_msg(std::string* err, const std::string& msg) {
    if (err) *err = msg;
    return false;
}

bool fail_status(std::string* err, const char* where, OSStatus status) {
    if (err) *err = std::string(where) + " failed (OSStatus " + std::to_string(int(status)) + ")";
    return false;
}

void set_cf_int32(VTCompressionSessionRef s, CFStringRef key, int32_t value) {
    CFNumberRef num = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &value);
    if (num) {
        VTSessionSetProperty(s, key, num);
        CFRelease(num);
    }
}

void append_start_code(std::vector<uint8_t>& out) {
    static constexpr uint8_t kStartCode[4] = {0x00, 0x00, 0x00, 0x01};
    out.insert(out.end(), std::begin(kStartCode), std::end(kStartCode));
}

void extract_parameter_sets(Codec codec, CMFormatDescriptionRef fmt, std::vector<uint8_t>& out) {
    if (!fmt) return;
    size_t count = 0;
    if (codec == Codec::H264) {
        if (CMVideoFormatDescriptionGetH264ParameterSetAtIndex(fmt, 0, nullptr, nullptr, &count, nullptr) != noErr) {
            return;
        }
        for (size_t i = 0; i < count; ++i) {
            const uint8_t* ptr = nullptr;
            size_t len = 0;
            if (CMVideoFormatDescriptionGetH264ParameterSetAtIndex(fmt, i, &ptr, &len, nullptr, nullptr) == noErr &&
                ptr && len > 0) {
                append_start_code(out);
                out.insert(out.end(), ptr, ptr + len);
            }
        }
    } else if (codec == Codec::HEVC) {
        if (__builtin_available(macOS 10.13, *)) {
            if (CMVideoFormatDescriptionGetHEVCParameterSetAtIndex(fmt, 0, nullptr, nullptr, &count, nullptr) !=
                noErr) {
                return;
            }
            for (size_t i = 0; i < count; ++i) {
                const uint8_t* ptr = nullptr;
                size_t len = 0;
                if (CMVideoFormatDescriptionGetHEVCParameterSetAtIndex(fmt, i, &ptr, &len, nullptr, nullptr) == noErr &&
                    ptr && len > 0) {
                    append_start_code(out);
                    out.insert(out.end(), ptr, ptr + len);
                }
            }
        }
    }
}

bool sample_is_keyframe(CMSampleBufferRef sbuf) {
    CFArrayRef attachments = CMSampleBufferGetSampleAttachmentsArray(sbuf, false);
    if (!attachments || CFArrayGetCount(attachments) == 0) return true;
    auto dict = static_cast<CFDictionaryRef>(CFArrayGetValueAtIndex(attachments, 0));
    if (!dict) return true;
    CFBooleanRef not_sync = nullptr;
    if (CFDictionaryGetValueIfPresent(dict, kCMSampleAttachmentKey_NotSync, reinterpret_cast<const void**>(&not_sync))) {
        return !CFBooleanGetValue(not_sync);
    }
    return true;
}

}  // namespace

std::unique_ptr<VtEncoder> VtEncoder::create(const EncoderConfig& config, std::string* err) {
    if (config.codec != Codec::H264 && config.codec != Codec::HEVC) {
        fail_msg(err, std::string("VideoToolbox encoder does not support ") + codec_name(config.codec));
        return nullptr;
    }
    if (config.width == 0 || config.height == 0) {
        fail_msg(err, "encoder width and height must be non-zero");
        return nullptr;
    }
    std::unique_ptr<VtEncoder> enc(new VtEncoder(config));
    if (!enc->open_session(config, err)) return nullptr;
    return enc;
}

VtEncoder::~VtEncoder() {
    close_session();
}

EncoderInfo VtEncoder::info() const {
    EncoderInfo ei;
    ei.backend = Backend::VideoToolbox;
    ei.hardware = hardware_;
    ei.description = std::string("VideoToolbox ") + codec_name(config_.codec) +
                     (hardware_ ? ", hardware" : ", software");
    return ei;
}

void VtEncoder::compression_callback(void* output_callback_ref_con, void* /*source_frame_ref_con*/, OSStatus status,
                                     VTEncodeInfoFlags info_flags, CMSampleBufferRef sample_buffer) {
    auto* self = static_cast<VtEncoder*>(output_callback_ref_con);
    if (!self) return;
    self->cb_status_ = status;
    if (status != noErr || !sample_buffer || (info_flags & kVTEncodeInfo_FrameDropped)) {
        return;
    }

    Packet pkt;
    pkt.keyframe = sample_is_keyframe(sample_buffer);
    if (pkt.keyframe) {
        CMFormatDescriptionRef fmt = CMSampleBufferGetFormatDescription(sample_buffer);
        extract_parameter_sets(self->config_.codec, fmt, pkt.data);
    }

    CMBlockBufferRef block = CMSampleBufferGetDataBuffer(sample_buffer);
    if (!block) return;

    size_t total_len = 0;
    char* data_ptr = nullptr;
    if (CMBlockBufferGetDataPointer(block, 0, nullptr, &total_len, &data_ptr) != noErr || !data_ptr) {
        return;
    }

    const auto* bytes = reinterpret_cast<const uint8_t*>(data_ptr);
    size_t offset = 0;
    while (offset + 4 <= total_len) {
        uint32_t nal_len = (uint32_t(bytes[offset + 0]) << 24) |
                           (uint32_t(bytes[offset + 1]) << 16) |
                           (uint32_t(bytes[offset + 2]) << 8) |
                           uint32_t(bytes[offset + 3]);
        offset += 4;
        if (nal_len == 0 || offset + nal_len > total_len) break;
        append_start_code(pkt.data);
        pkt.data.insert(pkt.data.end(), bytes + offset, bytes + offset + nal_len);
        offset += nal_len;
    }

    if (!pkt.data.empty()) {
        self->cb_packet_ = std::move(pkt);
        self->cb_got_packet_ = true;
    }
}

bool VtEncoder::open_session(const EncoderConfig& cfg, std::string* err) {
    close_session();

    enc_w_ = (cfg.width + 1u) & ~1u;
    enc_h_ = (cfg.height + 1u) & ~1u;
    fps_ = cfg.fps > 0 ? cfg.fps : 60;

    const CMVideoCodecType codec_type =
        (cfg.codec == Codec::HEVC) ? kCMVideoCodecType_HEVC : kCMVideoCodecType_H264;

    CFMutableDictionaryRef spec = CFDictionaryCreateMutable(
        kCFAllocatorDefault, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
    CFDictionarySetValue(spec, kVTVideoEncoderSpecification_EnableHardwareAcceleratedVideoEncoder, kCFBooleanTrue);
    if (!cfg.allow_software) {
        CFDictionarySetValue(spec, kVTVideoEncoderSpecification_RequireHardwareAcceleratedVideoEncoder,
                             kCFBooleanTrue);
    }

    OSStatus status = VTCompressionSessionCreate(
        kCFAllocatorDefault, int32_t(enc_w_), int32_t(enc_h_), codec_type, spec, nullptr, nullptr,
        &VtEncoder::compression_callback, this, &session_);
    CFRelease(spec);

    if (status != noErr || !session_) {
        session_ = nullptr;
        return fail_status(err, "VTCompressionSessionCreate", status);
    }

    VTSessionSetProperty(session_, kVTCompressionPropertyKey_RealTime, kCFBooleanTrue);
    VTSessionSetProperty(session_, kVTCompressionPropertyKey_AllowFrameReordering, kCFBooleanFalse);
    set_cf_int32(session_, kVTCompressionPropertyKey_MaxFrameDelayCount, 1);

    const int32_t bps = int32_t(std::clamp<uint64_t>(uint64_t(cfg.bitrate_kbps) * 1000u, 100000u, 500000000u));
    set_cf_int32(session_, kVTCompressionPropertyKey_AverageBitRate, bps);
    set_cf_int32(session_, kVTCompressionPropertyKey_ExpectedFrameRate, int32_t(fps_));
    set_cf_int32(session_, kVTCompressionPropertyKey_MaxKeyFrameInterval, int32_t(fps_ * 10));

    if (cfg.codec == Codec::H264) {
        VTSessionSetProperty(session_, kVTCompressionPropertyKey_ProfileLevel, kVTProfileLevel_H264_Main_AutoLevel);
    } else if (cfg.codec == Codec::HEVC) {
        if (__builtin_available(macOS 10.13, *)) {
            VTSessionSetProperty(session_, kVTCompressionPropertyKey_ProfileLevel, kVTProfileLevel_HEVC_Main_AutoLevel);
        }
    }

    status = VTCompressionSessionPrepareToEncodeFrames(session_);
    if (status != noErr) {
        close_session();
        return fail_status(err, "VTCompressionSessionPrepareToEncodeFrames", status);
    }

    hardware_ = false;
    CFBooleanRef using_hw = nullptr;
    if (VTSessionCopyProperty(session_, kVTCompressionPropertyKey_UsingHardwareAcceleratedVideoEncoder,
                              kCFAllocatorDefault, &using_hw) == noErr &&
        using_hw) {
        hardware_ = CFBooleanGetValue(using_hw);
        CFRelease(using_hw);
    }
    if (!cfg.allow_software && !hardware_) {
        close_session();
        return fail_msg(err, "VideoToolbox hardware encoder is unavailable for codec");
    }

    frame_index_ = 0;
    return true;
}

void VtEncoder::close_session() {
    if (session_) {
        VTCompressionSessionInvalidate(session_);
        CFRelease(session_);
        session_ = nullptr;
    }
    hardware_ = false;
}

bool VtEncoder::apply_config(const EncoderConfig& next, std::string* err) {
    if (!session_ || next.width != config_.width || next.height != config_.height || next.fps != config_.fps) {
        nv12_.clear();
        return open_session(next, err);
    }
    if (next.bitrate_kbps != config_.bitrate_kbps) {
        const int32_t bps = int32_t(std::clamp<uint64_t>(uint64_t(next.bitrate_kbps) * 1000u, 100000u, 500000000u));
        set_cf_int32(session_, kVTCompressionPropertyKey_AverageBitRate, bps);
    }
    return true;
}

bool VtEncoder::encode_frame(const Frame& frame, bool keyframe, const std::function<void()>& release, Packet& out,
                             std::string* err) {
    if (frame.width == 0 || frame.height == 0) return fail_msg(err, "frame is empty");
    if (!frame.is_cpu()) return fail_msg(err, "VideoToolbox encoder requires a CPU frame");
    if (!frame.cpu) return fail_msg(err, "CPU frame has no pixels");
    if (!session_ && !open_session(config_, err)) return false;

    if (!convert_to_nv12(frame, err)) return false;
    release();
    return encode_nv12(keyframe, out, err);
}

bool VtEncoder::encode_repeat_frame(bool keyframe, Packet& out, std::string* err) {
    if (!session_ || nv12_.empty()) return false;
    return encode_nv12(keyframe, out, err);
}

bool VtEncoder::convert_to_nv12(const Frame& frame, std::string* err) {
    const uint32_t w = enc_w_, h = enc_h_;
    nv12_.resize(size_t(w) * h * 3 / 2);
    uint8_t* dst_y = nv12_.data();
    uint8_t* dst_uv = nv12_.data() + size_t(w) * h;

    if (frame.cpu_format == PixelFormat::NV12) {
        const size_t src_pitch = frame.cpu_stride ? frame.cpu_stride : ((frame.width + 1) & ~1u);
        const uint8_t* src_y = frame.cpu;
        const uint8_t* src_uv = frame.cpu + src_pitch * frame.height;
        if (frame.width == w && frame.height == h) {
            for (uint32_t y = 0; y < h; ++y) {
                std::memcpy(dst_y + size_t(y) * w, src_y + size_t(y) * src_pitch, w);
            }
            for (uint32_t y = 0; y < h / 2; ++y) {
                std::memcpy(dst_uv + size_t(y) * w, src_uv + size_t(y) * src_pitch, w);
            }
        } else {
            for (uint32_t y = 0; y < h; ++y) {
                const uint32_t sy = std::min<uint32_t>(uint32_t(uint64_t(y) * frame.height / h), frame.height - 1);
                const uint8_t* srow = src_y + size_t(sy) * src_pitch;
                uint8_t* drow = dst_y + size_t(y) * w;
                for (uint32_t x = 0; x < w; ++x) {
                    const uint32_t sx = std::min<uint32_t>(uint32_t(uint64_t(x) * frame.width / w), frame.width - 1);
                    drow[x] = srow[sx];
                }
            }
            const uint32_t src_ch_h = (frame.height + 1) / 2;
            const uint32_t src_ch_w = (frame.width + 1) / 2;
            for (uint32_t y = 0; y < h / 2; ++y) {
                const uint32_t sy = std::min<uint32_t>(uint32_t(uint64_t(y) * src_ch_h / (h / 2)), src_ch_h - 1);
                const uint8_t* srow = src_uv + size_t(sy) * src_pitch;
                uint8_t* drow = dst_uv + size_t(y) * w;
                for (uint32_t x = 0; x < w / 2; ++x) {
                    const uint32_t sx = std::min<uint32_t>(uint32_t(uint64_t(x) * src_ch_w / (w / 2)), src_ch_w - 1);
                    drow[x * 2 + 0] = srow[sx * 2 + 0];
                    drow[x * 2 + 1] = srow[sx * 2 + 1];
                }
            }
        }
        return true;
    }

    if (frame.cpu_format != PixelFormat::RGBA8 && frame.cpu_format != PixelFormat::BGRA8) {
        return fail_msg(err, "CPU frames must be RGBA8, BGRA8 or NV12");
    }

    const int ri = frame.cpu_format == PixelFormat::BGRA8 ? 2 : 0;
    const int bi = frame.cpu_format == PixelFormat::BGRA8 ? 0 : 2;
    const size_t src_stride = frame.cpu_row_bytes();

    // BT.709 limited range (Y in 16..235, UV in 16..240) with 2x2 averaged chroma.
    for (uint32_t y = 0; y < h; y += 2) {
        const uint32_t sy0 = std::min<uint32_t>(uint32_t(uint64_t(y) * frame.height / h), frame.height - 1);
        const uint32_t sy1 = std::min<uint32_t>(uint32_t(uint64_t(y + 1) * frame.height / h), frame.height - 1);
        const uint8_t* s0 = frame.cpu + size_t(sy0) * src_stride;
        const uint8_t* s1 = frame.cpu + size_t(sy1) * src_stride;
        uint8_t* y0 = dst_y + size_t(y) * w;
        uint8_t* y1 = dst_y + size_t(y + 1) * w;
        uint8_t* uv = dst_uv + size_t(y / 2) * w;
        for (uint32_t x = 0; x < w; x += 2) {
            const uint32_t sx0 = std::min<uint32_t>(uint32_t(uint64_t(x) * frame.width / w), frame.width - 1);
            const uint32_t sx1 = std::min<uint32_t>(uint32_t(uint64_t(x + 1) * frame.width / w), frame.width - 1);
            const uint8_t* p00 = s0 + size_t(sx0) * 4;
            const uint8_t* p01 = s0 + size_t(sx1) * 4;
            const uint8_t* p10 = s1 + size_t(sx0) * 4;
            const uint8_t* p11 = s1 + size_t(sx1) * 4;
            const int r00 = p00[ri], g00 = p00[1], b00 = p00[bi];
            const int r01 = p01[ri], g01 = p01[1], b01 = p01[bi];
            const int r10 = p10[ri], g10 = p10[1], b10 = p10[bi];
            const int r11 = p11[ri], g11 = p11[1], b11 = p11[bi];
            y0[x + 0] = uint8_t(16 + ((11966 * r00 + 40254 * g00 + 4063 * b00 + 32768) >> 16));
            y0[x + 1] = uint8_t(16 + ((11966 * r01 + 40254 * g01 + 4063 * b01 + 32768) >> 16));
            y1[x + 0] = uint8_t(16 + ((11966 * r10 + 40254 * g10 + 4063 * b10 + 32768) >> 16));
            y1[x + 1] = uint8_t(16 + ((11966 * r11 + 40254 * g11 + 4063 * b11 + 32768) >> 16));
            const int sr = r00 + r01 + r10 + r11;
            const int sg = g00 + g01 + g10 + g11;
            const int sb = b00 + b01 + b10 + b11;
            uv[x + 0] = uint8_t(std::clamp(128 + ((-1650 * sr - 5549 * sg + 7199 * sb + 32768) >> 16), 16, 240));
            uv[x + 1] = uint8_t(std::clamp(128 + ((7199 * sr - 6539 * sg - 660 * sb + 32768) >> 16), 16, 240));
        }
    }
    return true;
}

bool VtEncoder::encode_nv12(bool keyframe, Packet& out, std::string* err) {
    CVPixelBufferRef pixbuf = nullptr;
    CVReturn cv_ret = CVPixelBufferCreate(
        kCFAllocatorDefault, enc_w_, enc_h_, kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange, nullptr, &pixbuf);
    if (cv_ret != kCVReturnSuccess || !pixbuf) {
        return fail_status(err, "CVPixelBufferCreate", cv_ret);
    }

    CVPixelBufferLockBaseAddress(pixbuf, 0);
    auto* dst_y = static_cast<uint8_t*>(CVPixelBufferGetBaseAddressOfPlane(pixbuf, 0));
    auto* dst_uv = static_cast<uint8_t*>(CVPixelBufferGetBaseAddressOfPlane(pixbuf, 1));
    const size_t stride_y = CVPixelBufferGetBytesPerRowOfPlane(pixbuf, 0);
    const size_t stride_uv = CVPixelBufferGetBytesPerRowOfPlane(pixbuf, 1);

    const uint8_t* src_y = nv12_.data();
    const uint8_t* src_uv = nv12_.data() + size_t(enc_w_) * enc_h_;
    for (uint32_t y = 0; y < enc_h_; ++y) {
        std::memcpy(dst_y + size_t(y) * stride_y, src_y + size_t(y) * enc_w_, enc_w_);
    }
    for (uint32_t y = 0; y < enc_h_ / 2; ++y) {
        std::memcpy(dst_uv + size_t(y) * stride_uv, src_uv + size_t(y) * enc_w_, enc_w_);
    }
    CVPixelBufferUnlockBaseAddress(pixbuf, 0);

    CFMutableDictionaryRef frame_props = nullptr;
    if (keyframe) {
        frame_props = CFDictionaryCreateMutable(
            kCFAllocatorDefault, 1, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
        CFDictionarySetValue(frame_props, kVTEncodeFrameOptionKey_ForceKeyFrame, kCFBooleanTrue);
    }

    cb_status_ = noErr;
    cb_got_packet_ = false;
    cb_packet_ = Packet{};

    const CMTime pts = CMTimeMake(frame_index_++, int32_t(fps_));
    const CMTime duration = CMTimeMake(1, int32_t(fps_));
    VTEncodeInfoFlags flags = 0;
    OSStatus status = VTCompressionSessionEncodeFrame(session_, pixbuf, pts, duration, frame_props, nullptr, &flags);
    if (frame_props) CFRelease(frame_props);
    CVPixelBufferRelease(pixbuf);

    if (status != noErr) {
        return fail_status(err, "VTCompressionSessionEncodeFrame", status);
    }

    status = VTCompressionSessionCompleteFrames(session_, kCMTimeInvalid);
    if (status != noErr) {
        return fail_status(err, "VTCompressionSessionCompleteFrames", status);
    }
    if (cb_status_ != noErr) {
        return fail_status(err, "VideoToolbox compression callback", cb_status_);
    }
    if (!cb_got_packet_ || cb_packet_.data.empty()) {
        return fail_msg(err, "VideoToolbox encoder produced no output packet");
    }

    out = std::move(cb_packet_);
    return true;
}

}  // namespace brovideo::vt
