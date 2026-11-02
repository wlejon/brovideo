// macOS VideoToolbox H.264 / HEVC decoder + backend factory entry points.
#include "vt/vt_codec.h"

#include "brovideo/bitstream.h"

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

std::vector<std::span<const uint8_t>> split_annexb_nals(std::span<const uint8_t> b) {
    std::vector<std::span<const uint8_t>> nals;
    const size_t n = b.size();
    auto next_start = [&](size_t from) -> size_t {
        for (size_t j = from; j + 2 < n; ++j) {
            if (b[j] == 0 && b[j + 1] == 0 && b[j + 2] == 1) return j;
        }
        return n;
    };
    size_t i = next_start(0);
    while (i < n) {
        const size_t begin = i + 3;
        size_t end = next_start(begin);
        const size_t following = end;
        while (end > begin && b[end - 1] == 0) --end;
        if (end > begin) nals.push_back(b.subspan(begin, end - begin));
        i = following;
    }
    return nals;
}

}  // namespace

std::unique_ptr<VtDecoder> VtDecoder::create(Codec codec, Hardware hw_mode, PictureMemory output, std::string* err) {
    if (codec != Codec::H264 && codec != Codec::HEVC) {
        fail_msg(err, std::string("VideoToolbox decoder does not support ") + codec_name(codec));
        return nullptr;
    }
    return std::unique_ptr<VtDecoder>(new VtDecoder(codec, hw_mode, output));
}

VtDecoder::~VtDecoder() {
    close_session();
    if (format_desc_) {
        CFRelease(format_desc_);
        format_desc_ = nullptr;
    }
}

std::string VtDecoder::describe() const {
    return std::string("VideoToolbox ") + codec_name(codec()) + (hardware_ ? ", hardware" : ", software");
}

void VtDecoder::decompression_callback(void* decompression_output_ref_con, void* /*source_frame_ref_con*/,
                                       OSStatus status, VTDecodeInfoFlags info_flags, CVImageBufferRef image_buffer,
                                       CMTime /*presentation_time_stamp*/, CMTime /*presentation_duration*/) {
    auto* self = static_cast<VtDecoder*>(decompression_output_ref_con);
    if (!self) return;
    self->cb_status_ = status;
    if (status != noErr || !image_buffer || (info_flags & kVTDecodeInfo_FrameDropped)) {
        return;
    }

    CVPixelBufferLockBaseAddress(image_buffer, kCVPixelBufferLock_ReadOnly);
    const auto w = uint32_t(CVPixelBufferGetWidth(image_buffer));
    const auto h = uint32_t(CVPixelBufferGetHeight(image_buffer));
    const auto* src_y = static_cast<const uint8_t*>(CVPixelBufferGetBaseAddressOfPlane(image_buffer, 0));
    const auto* src_uv = static_cast<const uint8_t*>(CVPixelBufferGetBaseAddressOfPlane(image_buffer, 1));
    const size_t stride_y = CVPixelBufferGetBytesPerRowOfPlane(image_buffer, 0);
    const size_t stride_uv = CVPixelBufferGetBytesPerRowOfPlane(image_buffer, 1);

    if (src_y && src_uv && w > 0 && h > 0) {
        const uint32_t tight_stride = (w + 1u) & ~1u;
        const uint32_t chroma_h = (h + 1u) / 2u;
        Picture pic;
        pic.ready = true;
        pic.width = w;
        pic.height = h;
        pic.format = PixelFormat::NV12;
        pic.memory = PictureMemory::Cpu;
        pic.stride = tight_stride;
        pic.uv_offset = tight_stride * h;
        pic.data.resize(size_t(tight_stride) * (h + chroma_h));

        uint8_t* dst_y = pic.data.data();
        uint8_t* dst_uv = pic.data.data() + pic.uv_offset;
        for (uint32_t r = 0; r < h; ++r) {
            std::memcpy(dst_y + size_t(r) * tight_stride, src_y + size_t(r) * stride_y, w);
        }
        for (uint32_t r = 0; r < chroma_h; ++r) {
            std::memcpy(dst_uv + size_t(r) * tight_stride, src_uv + size_t(r) * stride_uv, tight_stride);
        }
        self->cb_picture_ = std::move(pic);
    }
    CVPixelBufferUnlockBaseAddress(image_buffer, kCVPixelBufferLock_ReadOnly);
}

bool VtDecoder::ensure_format_and_session(const std::vector<std::span<const uint8_t>>& nals, std::string* err) {
    bool params_changed = false;
    for (const auto& nal : nals) {
        if (nal.empty()) continue;
        if (codec() == Codec::H264) {
            const int type = nal[0] & 0x1F;
            if (type == 7) {
                if (! std::equal(sps_.begin(), sps_.end(), nal.begin(), nal.end())) {
                    sps_.assign(nal.begin(), nal.end());
                    params_changed = true;
                }
            } else if (type == 8) {
                if (!std::equal(pps_.begin(), pps_.end(), nal.begin(), nal.end())) {
                    pps_.assign(nal.begin(), nal.end());
                    params_changed = true;
                }
            }
        } else if (codec() == Codec::HEVC) {
            const int type = (nal[0] >> 1) & 0x3F;
            if (type == 32) {
                if (!std::equal(vps_.begin(), vps_.end(), nal.begin(), nal.end())) {
                    vps_.assign(nal.begin(), nal.end());
                    params_changed = true;
                }
            } else if (type == 33) {
                if (!std::equal(sps_.begin(), sps_.end(), nal.begin(), nal.end())) {
                    sps_.assign(nal.begin(), nal.end());
                    params_changed = true;
                }
            } else if (type == 34) {
                if (!std::equal(pps_.begin(), pps_.end(), nal.begin(), nal.end())) {
                    pps_.assign(nal.begin(), nal.end());
                    params_changed = true;
                }
            }
        }
    }

    if (!params_changed && session_ && format_desc_) return true;

    CMVideoFormatDescriptionRef new_fmt = nullptr;
    OSStatus status = noErr;
    if (codec() == Codec::H264) {
        if (sps_.empty() || pps_.empty()) {
            return fail_msg(err, "H.264 stream is missing SPS/PPS parameter sets");
        }
        const uint8_t* ptrs[2] = {sps_.data(), pps_.data()};
        const size_t sizes[2] = {sps_.size(), pps_.size()};
        status = CMVideoFormatDescriptionCreateFromH264ParameterSets(
            kCFAllocatorDefault, 2, ptrs, sizes, 4, &new_fmt);
    } else {
        if (vps_.empty() || sps_.empty() || pps_.empty()) {
            return fail_msg(err, "HEVC stream is missing VPS/SPS/PPS parameter sets");
        }
        const uint8_t* ptrs[3] = {vps_.data(), sps_.data(), pps_.data()};
        const size_t sizes[3] = {vps_.size(), sps_.size(), pps_.size()};
        if (__builtin_available(macOS 10.13, *)) {
            status = CMVideoFormatDescriptionCreateFromHEVCParameterSets(
                kCFAllocatorDefault, 3, ptrs, sizes, 4, nullptr, &new_fmt);
        } else {
            return fail_msg(err, "HEVC requires macOS 10.13+");
        }
    }

    if (status != noErr || !new_fmt) {
        return fail_status(err, "CMVideoFormatDescriptionCreateFromParameterSets", status);
    }

    if (session_ && format_desc_ &&
        (CMFormatDescriptionEqual(format_desc_, new_fmt) ||
         VTDecompressionSessionCanAcceptFormatDescription(session_, new_fmt))) {
        CFRelease(format_desc_);
        format_desc_ = new_fmt;
        return true;
    }

    if (!open_session(new_fmt, err)) {
        CFRelease(new_fmt);
        return false;
    }
    if (format_desc_) CFRelease(format_desc_);
    format_desc_ = new_fmt;
    return true;
}

bool VtDecoder::open_session(CMVideoFormatDescriptionRef fmt, std::string* err) {
    close_session();

    CFMutableDictionaryRef dec_spec = CFDictionaryCreateMutable(
        kCFAllocatorDefault, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
    if (hw_mode_ == Hardware::Forbid) {
        CFDictionarySetValue(dec_spec, kVTVideoDecoderSpecification_EnableHardwareAcceleratedVideoDecoder,
                             kCFBooleanFalse);
    } else {
        CFDictionarySetValue(dec_spec, kVTVideoDecoderSpecification_EnableHardwareAcceleratedVideoDecoder,
                             kCFBooleanTrue);
        if (hw_mode_ == Hardware::Require) {
            CFDictionarySetValue(dec_spec, kVTVideoDecoderSpecification_RequireHardwareAcceleratedVideoDecoder,
                                 kCFBooleanTrue);
        }
    }

    CFMutableDictionaryRef attr = CFDictionaryCreateMutable(
        kCFAllocatorDefault, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
    const int32_t pix_fmt = kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange;
    CFNumberRef pix_fmt_num = CFNumberCreate(kCFAllocatorDefault, kCFNumberSInt32Type, &pix_fmt);
    CFDictionarySetValue(attr, kCVPixelBufferPixelFormatTypeKey, pix_fmt_num);
    CFRelease(pix_fmt_num);

    VTDecompressionOutputCallbackRecord cb_rec{};
    cb_rec.decompressionOutputCallback = &VtDecoder::decompression_callback;
    cb_rec.decompressionOutputRefCon = this;

    OSStatus status = VTDecompressionSessionCreate(
        kCFAllocatorDefault, fmt, dec_spec, attr, &cb_rec, &session_);
    CFRelease(dec_spec);
    CFRelease(attr);

    if (status != noErr || !session_) {
        session_ = nullptr;
        return fail_status(err, "VTDecompressionSessionCreate", status);
    }

    hardware_ = false;
    CFBooleanRef using_hw = nullptr;
    if (VTSessionCopyProperty(session_, kVTDecompressionPropertyKey_UsingHardwareAcceleratedVideoDecoder,
                              kCFAllocatorDefault, &using_hw) == noErr &&
        using_hw) {
        hardware_ = CFBooleanGetValue(using_hw);
        CFRelease(using_hw);
    }
    if (hw_mode_ == Hardware::Require && !hardware_) {
        close_session();
        return fail_msg(err, "VideoToolbox hardware decoder is unavailable");
    }
    return true;
}

void VtDecoder::close_session() {
    if (session_) {
        VTDecompressionSessionInvalidate(session_);
        CFRelease(session_);
        session_ = nullptr;
    }
    hardware_ = false;
}

void VtDecoder::reset_after_error() {
    synced_ = false;
}

bool VtDecoder::decode(std::span<const uint8_t> packet, int64_t pts_ns, Picture& out, std::string* err) {
    out = Picture{};
    if (packet.empty()) return fail_msg(err, "empty packet");

    if (!synced_) {
        if (!is_keyframe(codec(), packet)) {
            return fail_msg(err, "waiting for a keyframe");
        }
    }

    auto nals = split_annexb_nals(packet);
    if (nals.empty()) {
        reset_after_error();
        return fail_msg(err, "packet contains no Annex-B NAL units");
    }

    if (!ensure_format_and_session(nals, err)) {
        reset_after_error();
        return false;
    }

    // Build 4-byte length-prefixed AVCC / HVCC sample buffer from VCL + SEI NALs.
    std::vector<uint8_t> avcc;
    for (const auto& nal : nals) {
        if (nal.empty()) continue;
        if (codec() == Codec::H264) {
            const int type = nal[0] & 0x1F;
            if (type == 7 || type == 8 || type == 9) continue;  // skip SPS, PPS, AUD
        } else if (codec() == Codec::HEVC) {
            const int type = (nal[0] >> 1) & 0x3F;
            if (type == 32 || type == 33 || type == 34 || type == 35) continue;  // skip VPS, SPS, PPS, AUD
        }
        const auto len = uint32_t(nal.size());
        const uint8_t hdr[4] = {
            uint8_t((len >> 24) & 0xFF),
            uint8_t((len >> 16) & 0xFF),
            uint8_t((len >> 8) & 0xFF),
            uint8_t(len & 0xFF),
        };
        avcc.insert(avcc.end(), std::begin(hdr), std::end(hdr));
        avcc.insert(avcc.end(), nal.begin(), nal.end());
    }

    if (avcc.empty()) {
        reset_after_error();
        return fail_msg(err, "packet contains no slice NAL units");
    }

    CMBlockBufferRef block = nullptr;
    OSStatus status = CMBlockBufferCreateWithMemoryBlock(
        kCFAllocatorDefault, nullptr, avcc.size(), kCFAllocatorDefault, nullptr, 0, avcc.size(), 0, &block);
    if (status != noErr || !block) {
        reset_after_error();
        return fail_status(err, "CMBlockBufferCreateWithMemoryBlock", status);
    }
    status = CMBlockBufferReplaceDataBytes(avcc.data(), block, 0, avcc.size());
    if (status != noErr) {
        CFRelease(block);
        reset_after_error();
        return fail_status(err, "CMBlockBufferReplaceDataBytes", status);
    }

    CMSampleBufferRef sbuf = nullptr;
    const size_t sample_size = avcc.size();
    status = CMSampleBufferCreateReady(
        kCFAllocatorDefault, block, format_desc_, 1, 0, nullptr, 1, &sample_size, &sbuf);
    CFRelease(block);
    if (status != noErr || !sbuf) {
        reset_after_error();
        return fail_status(err, "CMSampleBufferCreateReady", status);
    }

    cb_status_ = noErr;
    cb_picture_ = Picture{};

    VTDecodeInfoFlags info_flags = 0;
    status = VTDecompressionSessionDecodeFrame(session_, sbuf, 0, nullptr, &info_flags);
    CFRelease(sbuf);
    if (status != noErr) {
        reset_after_error();
        return fail_status(err, "VTDecompressionSessionDecodeFrame", status);
    }

    VTDecompressionSessionWaitForAsynchronousFrames(session_);
    if (cb_status_ != noErr) {
        reset_after_error();
        return fail_status(err, "VideoToolbox decompression callback", cb_status_);
    }
    if (!cb_picture_.ready) {
        reset_after_error();
        return fail_msg(err, "VideoToolbox decoder produced no picture");
    }

    synced_ = true;
    out = std::move(cb_picture_);
    out.pts_ns = pts_ns;
    return true;
}

}  // namespace brovideo::vt

namespace brovideo::detail {

namespace {

bool probe_vt_encode(Codec c, bool* got_hw, std::string& why) {
    EncoderConfig ec;
    ec.codec = c;
    ec.width = 256;
    ec.height = 256;
    ec.fps = 60;
    ec.bitrate_kbps = 4000;
    ec.allow_software = true;
    auto enc = vt::VtEncoder::create(ec, &why);
    if (!enc) return false;

    std::vector<uint8_t> nv12(256 * 256 * 3 / 2, 0x80);
    Frame f;
    f.width = 256;
    f.height = 256;
    f.cpu = nv12.data();
    f.cpu_format = PixelFormat::NV12;
    Packet pkt;
    if (!enc->encode(f, true, {}, pkt, &why)) return false;
    if (pkt.data.empty() || !is_keyframe(c, pkt.data)) {
        why = "VideoToolbox encoder did not produce a valid keyframe";
        return false;
    }
    if (got_hw) *got_hw = enc->hardware();
    return true;
}

bool probe_vt_decode(Codec c, Hardware hw_mode, bool* got_hw, std::string& why) {
    // Encode a 192x192 grey keyframe and decode it to verify end-to-end VideoToolbox decode.
    EncoderConfig ec;
    ec.codec = c;
    ec.width = 192;
    ec.height = 192;
    ec.fps = 60;
    ec.bitrate_kbps = 4000;
    ec.allow_software = true;
    auto enc = vt::VtEncoder::create(ec, &why);
    if (!enc) return false;

    std::vector<uint8_t> nv12(192 * 192 * 3 / 2, 0x80);
    Frame f;
    f.width = 192;
    f.height = 192;
    f.cpu = nv12.data();
    f.cpu_format = PixelFormat::NV12;
    Packet pkt;
    if (!enc->encode(f, true, {}, pkt, &why)) return false;

    auto dec = vt::VtDecoder::create(c, hw_mode, PictureMemory::Cpu, &why);
    if (!dec) return false;
    Picture pic;
    if (!dec->decode(pkt.data, pic, &why)) return false;
    if (!pic.ready || pic.width != 192 || pic.height != 192) {
        why = "VideoToolbox decoder returned an invalid picture";
        return false;
    }
    if (got_hw) *got_hw = dec->hardware();
    return true;
}

Capability make_vt_cap(Codec c, Direction dir, bool hw) {
    Capability cap;
    cap.codec = c;
    cap.direction = dir;
    cap.backend = Backend::VideoToolbox;
    cap.hardware = hw;
    cap.device = hw ? "VideoToolbox (Hardware)" : "VideoToolbox (Software)";
    return cap;
}

ProbeResult probe_vt() {
    ProbeResult r;
    for (Codec c : {Codec::H264, Codec::HEVC}) {
        bool enc_hw = false;
        std::string why_enc;
        if (probe_vt_encode(c, &enc_hw, why_enc)) {
            r.caps.push_back(make_vt_cap(c, Direction::Encode, enc_hw));
        } else {
            r.missing.push_back({c, Direction::Encode, "VideoToolbox: " + why_enc});
        }

        bool dec_hw = false;
        std::string why_dec_hw, why_dec_sw;
        if (probe_vt_decode(c, Hardware::Prefer, &dec_hw, why_dec_hw)) {
            r.caps.push_back(make_vt_cap(c, Direction::Decode, dec_hw));
            if (dec_hw && probe_vt_decode(c, Hardware::Forbid, nullptr, why_dec_sw)) {
                r.caps.push_back(make_vt_cap(c, Direction::Decode, false));
            }
        } else {
            r.missing.push_back({c, Direction::Decode, "VideoToolbox: " + why_dec_hw});
        }
    }
    return r;
}

std::unique_ptr<Encoder> create_vt_encoder(const EncoderConfig& config, const Capability&, std::string* err) {
    return vt::VtEncoder::create(config, err);
}

std::unique_ptr<Decoder> create_vt_decoder(const DecoderConfig& config, const Capability& cap, std::string* err) {
    const Hardware mode = (config.hardware == Hardware::Require) ? Hardware::Require
                        : (!cap.hardware)                        ? Hardware::Forbid
                                                                 : Hardware::Prefer;
    return vt::VtDecoder::create(config.codec, mode, config.output, err);
}

}  // namespace

const BackendOps& vt_backend() {
    static const BackendOps ops{Backend::VideoToolbox, probe_vt, create_vt_encoder, create_vt_decoder};
    return ops;
}

}  // namespace brovideo::detail
