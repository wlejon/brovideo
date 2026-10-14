#include "mf_encoder.h"

#include "brovideo/bitstream.h"

#include <codecapi.h>
#include <mferror.h>

#include <algorithm>
#include <cstring>

namespace brovideo::mf {

namespace {

bool fail(std::string* err, const char* what, HRESULT hr) {
    if (err) *err = hr_text(what, hr);
    return false;
}

bool fail_msg(std::string* err, const std::string& msg) {
    if (err) *err = msg;
    return false;
}

uint32_t bitrate_bps(const EncoderConfig& c) {
    const uint64_t kbps = c.bitrate_kbps ? c.bitrate_kbps : 20000;
    return uint32_t(std::min<uint64_t>(kbps * 1000, 0xffffffffu));
}

HRESULT set_bool(ICodecAPI* api, const GUID& guid, bool val) {
    if (!api) return E_NOINTERFACE;
    VARIANT v{};
    v.vt = VT_BOOL;
    v.boolVal = val ? VARIANT_TRUE : VARIANT_FALSE;
    return api->SetValue(&guid, &v);
}

HRESULT set_u32(ICodecAPI* api, const GUID& guid, uint32_t val) {
    if (!api) return E_NOINTERFACE;
    VARIANT v{};
    v.vt = VT_UI4;
    v.ulVal = val;
    return api->SetValue(&guid, &v);
}

void apply_codec_api(ICodecAPI* api, uint32_t bps) {
    if (!api) return;
    set_bool(api, CODECAPI_AVLowLatencyMode, true);
    set_bool(api, CODECAPI_AVEncCommonLowLatency, true);
    if (FAILED(set_u32(api, CODECAPI_AVEncCommonRateControlMode, eAVEncCommonRateControlMode_CBR))) {
        set_u32(api, CODECAPI_AVEncCommonRateControlMode, eAVEncCommonRateControlMode_LowDelayVBR);
    }
    set_u32(api, CODECAPI_AVEncCommonMeanBitRate, bps);
    set_u32(api, CODECAPI_AVEncCommonMaxBitRate, bps);
    set_u32(api, CODECAPI_AVEncMPVDefaultBPictureCount, 0);
    if (FAILED(set_u32(api, CODECAPI_AVEncMPVGOPSize, 0xffffffffu))) {
        if (FAILED(set_u32(api, CODECAPI_AVEncMPVGOPSize, 0x7fffffffu))) {
            set_u32(api, CODECAPI_AVEncMPVGOPSize, 60000);
        }
    }
}

// Extracts the non-VCL prefix (AUD/VPS/SPS/PPS for Annex B, or the Sequence
// Header OBU for AV1) from a keyframe packet so it can be prepended if a
// later forced keyframe omits parameter sets.
std::vector<uint8_t> extract_headers(Codec codec, const std::vector<uint8_t>& pkt) {
    if (codec == Codec::AV1) {
        size_t pos = 0;
        while (pos < pkt.size()) {
            const size_t start = pos;
            const uint8_t h = pkt[pos++];
            const int type = (h >> 3) & 0x0F;
            if (h & 0x04) ++pos;
            if (!(h & 0x02)) break;
            uint64_t sz = 0;
            for (int i = 0; i < 8 && pos < pkt.size(); ++i) {
                const uint8_t b = pkt[pos++];
                sz |= uint64_t(b & 0x7F) << (7 * i);
                if (!(b & 0x80)) break;
            }
            if (pos + sz > pkt.size()) break;
            pos += size_t(sz);
            if (type == 1) return std::vector<uint8_t>(pkt.begin() + std::ptrdiff_t(start), pkt.begin() + std::ptrdiff_t(pos));
        }
        return {};
    }
    // Annex B: keep all parameter set NALs (7/8 for H.264; 32/33/34 for HEVC).
    std::vector<uint8_t> out;
    const size_t n = pkt.size();
    auto next_sc = [&](size_t from) -> size_t {
        for (size_t j = from; j + 2 < n; ++j) {
            if (pkt[j] == 0 && pkt[j + 1] == 0 && pkt[j + 2] == 1) return j;
        }
        return n;
    };
    size_t i = next_sc(0);
    while (i < n) {
        const size_t sc_start = (i > 0 && pkt[i - 1] == 0) ? i - 1 : i;
        const size_t nal_begin = i + 3;
        const size_t next = next_sc(nal_begin);
        const size_t nal_end = (next < n && next > nal_begin && pkt[next - 1] == 0) ? next - 1 : next;
        if (nal_end > nal_begin) {
            bool is_param = false;
            if (codec == Codec::H264) {
                const int t = pkt[nal_begin] & 0x1F;
                is_param = (t == 7 || t == 8);
            } else if (codec == Codec::HEVC) {
                const int t = (pkt[nal_begin] >> 1) & 0x3F;
                is_param = (t == 32 || t == 33 || t == 34);
            }
            if (is_param) {
                out.insert(out.end(), pkt.begin() + std::ptrdiff_t(sc_start), pkt.begin() + std::ptrdiff_t(nal_end));
            }
        }
        i = next;
    }
    return out;
}

}  // namespace

std::unique_ptr<MfEncoder> MfEncoder::create(const EncoderConfig& config, std::string* err) {
    if (!ensure_started(err)) return nullptr;
    std::unique_ptr<MfEncoder> e(new MfEncoder(config));
    if (!e->open_session(config, err)) return nullptr;
    return e;
}

MfEncoder::~MfEncoder() { close_session(); }

EncoderInfo MfEncoder::info() const {
    EncoderInfo i;
    i.backend = Backend::MediaFoundation;
    i.hardware = true;
    i.description = std::string("Media Foundation ") + codec_name(config_.codec) + ", hardware";
    if (!mft_name_.empty() || !adapter_.empty()) {
        i.description += " (";
        if (!mft_name_.empty()) i.description += mft_name_;
        if (!mft_name_.empty() && !adapter_.empty()) i.description += "; ";
        if (!adapter_.empty()) i.description += adapter_;
        i.description += ")";
    }
    return i;
}

bool MfEncoder::open_session(const EncoderConfig& cfg, std::string* err) {
    close_session();
    if (!device_ && !create_d3d11(device_, manager_, err)) return false;
    adapter_ = adapter_name(device_.Get());
    const UINT32 vendor = adapter_vendor_id(device_.Get());

    auto acts = enum_encoder_mfts(cfg.codec, vendor, err);
    if (acts.empty()) return false;

    std::string last;
    for (const auto& act : acts) {
        if (init_mft(act.Get(), cfg, &last)) {
            mft_name_ = activate_name(act.Get());
            return true;
        }
        close_session();
    }
    if (err) *err = last.empty() ? "failed to initialize the hardware encoder MFT" : last;
    return false;
}

bool MfEncoder::init_mft(IMFActivate* act, const EncoderConfig& cfg, std::string* err) {
    HRESULT hr = act->ActivateObject(IID_PPV_ARGS(&mft_));
    if (FAILED(hr) || !mft_) return fail(err, "activating the encoder MFT", hr);

    ComPtr<IMFAttributes> attrs;
    if (SUCCEEDED(mft_->GetAttributes(&attrs)) && attrs) {
        is_async_ = MFGetAttributeUINT32(attrs.Get(), MF_TRANSFORM_ASYNC, FALSE) != 0;
        if (is_async_) attrs->SetUINT32(MF_TRANSFORM_ASYNC_UNLOCK, TRUE);
        attrs->SetUINT32(MF_LOW_LATENCY, TRUE);
    }

    if (manager_) {
        hr = mft_->ProcessMessage(MFT_MESSAGE_SET_D3D_MANAGER, reinterpret_cast<ULONG_PTR>(manager_.Get()));
        if (FAILED(hr)) return fail(err, "MFT_MESSAGE_SET_D3D_MANAGER (encoder)", hr);
    }

    if (FAILED(mft_->GetStreamIDs(1, &in_id_, 1, &out_id_))) {
        in_id_ = 0;
        out_id_ = 0;
    }

    mft_.As(&codec_api_);
    const uint32_t bps = bitrate_bps(cfg);
    apply_codec_api(codec_api_.Get(), bps);

    enc_w_ = (cfg.width + 1) & ~1u;
    enc_h_ = (cfg.height + 1) & ~1u;
    fps_ = cfg.fps ? cfg.fps : 60;

    GUID subtype{};
    subtype_for(cfg.codec, subtype);
    ComPtr<IMFMediaType> out_type;
    for (DWORD i = 0;; ++i) {
        ComPtr<IMFMediaType> t;
        if (FAILED(mft_->GetOutputAvailableType(out_id_, i, &t)) || !t) break;
        GUID sub{};
        if (SUCCEEDED(t->GetGUID(MF_MT_SUBTYPE, &sub)) && sub == subtype) {
            out_type = t;
            break;
        }
    }
    if (!out_type) {
        hr = MFCreateMediaType(&out_type);
        if (FAILED(hr)) return fail(err, "MFCreateMediaType (output)", hr);
        out_type->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
        out_type->SetGUID(MF_MT_SUBTYPE, subtype);
    }
    out_type->SetUINT32(MF_MT_AVG_BITRATE, bps);
    MFSetAttributeSize(out_type.Get(), MF_MT_FRAME_SIZE, enc_w_, enc_h_);
    MFSetAttributeRatio(out_type.Get(), MF_MT_FRAME_RATE, fps_, 1);
    out_type->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive);
    MFSetAttributeRatio(out_type.Get(), MF_MT_PIXEL_ASPECT_RATIO, 1, 1);
    out_type->SetUINT32(MF_MT_VIDEO_PRIMARIES, MFVideoPrimaries_BT709);
    out_type->SetUINT32(MF_MT_TRANSFER_FUNCTION, MFVideoTransFunc_709);
    out_type->SetUINT32(MF_MT_YUV_MATRIX, MFVideoTransferMatrix_BT709);
    out_type->SetUINT32(MF_MT_VIDEO_NOMINAL_RANGE, MFNominalRange_16_235);
    hr = mft_->SetOutputType(out_id_, out_type.Get(), 0);
    if (FAILED(hr)) {
        out_type->DeleteItem(MF_MT_VIDEO_PRIMARIES);
        out_type->DeleteItem(MF_MT_TRANSFER_FUNCTION);
        out_type->DeleteItem(MF_MT_YUV_MATRIX);
        out_type->DeleteItem(MF_MT_VIDEO_NOMINAL_RANGE);
        hr = mft_->SetOutputType(out_id_, out_type.Get(), 0);
        if (FAILED(hr)) return fail(err, "setting the encoder's output type", hr);
    }

    ComPtr<IMFMediaType> in_type;
    for (DWORD i = 0;; ++i) {
        ComPtr<IMFMediaType> t;
        if (FAILED(mft_->GetInputAvailableType(in_id_, i, &t)) || !t) break;
        GUID sub{};
        if (SUCCEEDED(t->GetGUID(MF_MT_SUBTYPE, &sub)) && sub == MFVideoFormat_NV12) {
            in_type = t;
            break;
        }
    }
    if (!in_type) {
        hr = MFCreateMediaType(&in_type);
        if (FAILED(hr)) return fail(err, "MFCreateMediaType (input)", hr);
        in_type->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
        in_type->SetGUID(MF_MT_SUBTYPE, MFVideoFormat_NV12);
    }
    MFSetAttributeSize(in_type.Get(), MF_MT_FRAME_SIZE, enc_w_, enc_h_);
    MFSetAttributeRatio(in_type.Get(), MF_MT_FRAME_RATE, fps_, 1);
    in_type->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive);
    MFSetAttributeRatio(in_type.Get(), MF_MT_PIXEL_ASPECT_RATIO, 1, 1);
    hr = mft_->SetInputType(in_id_, in_type.Get(), 0);
    if (FAILED(hr)) return fail(err, "setting the encoder's input type", hr);

    apply_codec_api(codec_api_.Get(), bps);

    MFT_OUTPUT_STREAM_INFO osi{};
    hr = mft_->GetOutputStreamInfo(out_id_, &osi);
    if (FAILED(hr)) return fail(err, "reading the encoder's output stream info", hr);
    provides_samples_ =
        (osi.dwFlags & (MFT_OUTPUT_STREAM_PROVIDES_SAMPLES | MFT_OUTPUT_STREAM_CAN_PROVIDE_SAMPLES)) != 0;
    out_sample_size_ = osi.cbSize;
    update_sequence_header();

    if (is_async_) {
        hr = mft_.As(&events_);
        if (FAILED(hr) || !events_) return fail(err, "querying IMFMediaEventGenerator on async encoder MFT", hr);
    }

    hr = mft_->ProcessMessage(MFT_MESSAGE_NOTIFY_BEGIN_STREAMING, 0);
    if (FAILED(hr)) return fail(err, "MFT_MESSAGE_NOTIFY_BEGIN_STREAMING (encoder)", hr);
    hr = mft_->ProcessMessage(MFT_MESSAGE_NOTIFY_START_OF_STREAM, 0);
    if (FAILED(hr)) return fail(err, "MFT_MESSAGE_NOTIFY_START_OF_STREAM (encoder)", hr);

    need_input_ = 0;
    next_time_ = 0;
    return true;
}

void MfEncoder::close_session() {
    if (mft_) {
        mft_->ProcessMessage(MFT_MESSAGE_NOTIFY_END_OF_STREAM, in_id_);
        mft_->ProcessMessage(MFT_MESSAGE_NOTIFY_END_STREAMING, 0);
        if (manager_) mft_->ProcessMessage(MFT_MESSAGE_SET_D3D_MANAGER, 0);
    }
    events_.Reset();
    codec_api_.Reset();
    mft_.Reset();
    is_async_ = false;
    need_input_ = 0;
    next_time_ = 0;
    seq_header_.clear();
}

void MfEncoder::update_sequence_header() {
    if (!mft_) return;
    ComPtr<IMFMediaType> cur;
    if (FAILED(mft_->GetOutputCurrentType(out_id_, &cur)) || !cur) return;
    UINT32 len = 0;
    if (SUCCEEDED(cur->GetBlobSize(MF_MT_MPEG_SEQUENCE_HEADER, &len)) && len > 0) {
        seq_header_.resize(len);
        if (FAILED(cur->GetBlob(MF_MT_MPEG_SEQUENCE_HEADER, seq_header_.data(), len, &len))) {
            seq_header_.clear();
        }
    }
}

bool MfEncoder::encode_frame(const Frame& frame, bool keyframe, const std::function<void()>& release, Packet& out,
                             std::string* err) {
    if (frame.width == 0 || frame.height == 0) return fail_msg(err, "frame is empty");
    if (!frame.is_cpu()) return fail_msg(err, "Media Foundation encoder requires a CPU frame");
    if (!frame.cpu) return fail_msg(err, "CPU frame has no pixels");
    if (!mft_ && !open_session(config_, err)) return false;

    if (!convert_to_nv12(frame, err)) return false;
    release();  // the caller's pixels are now in nv12_
    return encode_nv12(keyframe, out, err);
}

bool MfEncoder::encode_repeat_frame(bool keyframe, Packet& out, std::string* err) {
    if (!mft_ || nv12_.empty()) return false;
    return encode_nv12(keyframe, out, err);
}

bool MfEncoder::apply_config(const EncoderConfig& next, std::string* err) {
    if (!mft_ || next.width != config_.width || next.height != config_.height || next.fps != config_.fps) {
        nv12_.clear();
        return open_session(next, err);
    }
    if (next.bitrate_kbps != config_.bitrate_kbps) {
        const uint32_t bps = bitrate_bps(next);
        if (!codec_api_ || FAILED(set_u32(codec_api_.Get(), CODECAPI_AVEncCommonMeanBitRate, bps))) {
            nv12_.clear();
            return open_session(next, err);
        }
        set_u32(codec_api_.Get(), CODECAPI_AVEncCommonMaxBitRate, bps);
    }
    return true;
}

bool MfEncoder::convert_to_nv12(const Frame& frame, std::string* err) {
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

    // BT.709 limited range (Y in 16..235, UV in 16..240) with 2x2 averaged
    // centre-sited chroma, matching VA-API's VideoProc pass and the test
    // oracle.
    if (frame.width == w && frame.height == h) {
        for (uint32_t y = 0; y < h; y += 2) {
            const uint8_t* s0 = frame.cpu + size_t(y) * src_stride;
            const uint8_t* s1 = frame.cpu + size_t(y + 1) * src_stride;
            uint8_t* y0 = dst_y + size_t(y) * w;
            uint8_t* y1 = dst_y + size_t(y + 1) * w;
            uint8_t* uv = dst_uv + size_t(y / 2) * w;
            for (uint32_t x = 0; x < w; x += 2) {
                const uint8_t* p00 = s0 + size_t(x) * 4;
                const uint8_t* p01 = p00 + 4;
                const uint8_t* p10 = s1 + size_t(x) * 4;
                const uint8_t* p11 = p10 + 4;
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

bool MfEncoder::encode_nv12(bool keyframe, Packet& out, std::string* err) {
    if (is_async_) {
        for (int guard = 0; need_input_ == 0 && guard < 64; ++guard) {
            ComPtr<IMFMediaEvent> ev;
            const HRESULT hr = events_->GetEvent(0, &ev);
            if (FAILED(hr) || !ev) return fail(err, "waiting for encoder input event", hr);
            MediaEventType met = 0;
            ev->GetType(&met);
            if (met == METransformNeedInput) {
                ++need_input_;
            } else if (met == METransformHaveOutput) {
                Packet scratch;
                bool got = false;
                if (!pull_output(scratch, got, err)) return false;
            }
        }
        if (need_input_ == 0) return fail_msg(err, "encoder MFT did not request input");
    }

    if (keyframe && codec_api_) {
        set_u32(codec_api_.Get(), CODECAPI_AVEncVideoForceKeyFrame, 1);
    }

    ComPtr<IMFMediaBuffer> buf;
    HRESULT hr = MFCreateMemoryBuffer(DWORD(nv12_.size()), &buf);
    if (FAILED(hr)) return fail(err, "MFCreateMemoryBuffer (encoder input)", hr);
    BYTE* p = nullptr;
    hr = buf->Lock(&p, nullptr, nullptr);
    if (FAILED(hr)) return fail(err, "locking encoder input buffer", hr);
    std::memcpy(p, nv12_.data(), nv12_.size());
    buf->Unlock();
    buf->SetCurrentLength(DWORD(nv12_.size()));

    ComPtr<IMFSample> sample;
    hr = MFCreateSample(&sample);
    if (FAILED(hr)) return fail(err, "MFCreateSample (encoder input)", hr);
    sample->AddBuffer(buf.Get());
    const LONGLONG dur = 10000000LL / std::max<uint32_t>(fps_, 1);
    sample->SetSampleTime(next_time_ * dur);
    sample->SetSampleDuration(dur);
    if (keyframe) sample->SetUINT32(MFSampleExtension_CleanPoint, TRUE);
    ++next_time_;

    hr = mft_->ProcessInput(in_id_, sample.Get(), 0);
    if (FAILED(hr)) return fail(err, "encoder ProcessInput", hr);
    if (is_async_ && need_input_ > 0) --need_input_;

    for (int guard = 0; guard < 64; ++guard) {
        if (is_async_) {
            ComPtr<IMFMediaEvent> ev;
            hr = events_->GetEvent(0, &ev);
            if (FAILED(hr) || !ev) return fail(err, "waiting for encoder output event", hr);
            MediaEventType met = 0;
            ev->GetType(&met);
            if (met == METransformNeedInput) {
                ++need_input_;
                continue;
            }
            if (met != METransformHaveOutput) continue;
        }
        bool got = false;
        if (!pull_output(out, got, err)) return false;
        if (got) {
            if (out.data.empty()) return fail_msg(err, "the encoder produced an empty packet");
            finish_packet(keyframe, out);
            return true;
        }
    }
    return fail_msg(err, "the encoder produced no output packet");
}

bool MfEncoder::pull_output(Packet& out, bool& got_packet, std::string* err) {
    got_packet = false;
    MFT_OUTPUT_DATA_BUFFER ob{};
    ob.dwStreamID = out_id_;
    ComPtr<IMFSample> own;
    if (!provides_samples_) {
        ComPtr<IMFMediaBuffer> b;
        const DWORD sz = out_sample_size_ ? out_sample_size_ : std::max<DWORD>(enc_w_ * enc_h_ * 3 / 2, 1u << 20);
        HRESULT hr = MFCreateMemoryBuffer(sz, &b);
        if (SUCCEEDED(hr)) hr = MFCreateSample(&own);
        if (SUCCEEDED(hr)) hr = own->AddBuffer(b.Get());
        if (FAILED(hr)) return fail(err, "allocating encoder output sample", hr);
        ob.pSample = own.Get();
    }

    DWORD status = 0;
    const HRESULT hr = mft_->ProcessOutput(0, 1, &ob, &status);
    if (ob.pEvents) ob.pEvents->Release();
    ComPtr<IMFSample> given;
    if (provides_samples_ && ob.pSample) given.Attach(ob.pSample);

    if (hr == MF_E_TRANSFORM_STREAM_CHANGE) {
        ComPtr<IMFMediaType> next_out;
        HRESULT shr = mft_->GetOutputAvailableType(out_id_, 0, &next_out);
        if (SUCCEEDED(shr) && next_out) shr = mft_->SetOutputType(out_id_, next_out.Get(), 0);
        if (FAILED(shr)) return fail(err, "renegotiating encoder output type", shr);
        MFT_OUTPUT_STREAM_INFO osi{};
        if (SUCCEEDED(mft_->GetOutputStreamInfo(out_id_, &osi))) {
            provides_samples_ =
                (osi.dwFlags & (MFT_OUTPUT_STREAM_PROVIDES_SAMPLES | MFT_OUTPUT_STREAM_CAN_PROVIDE_SAMPLES)) != 0;
            out_sample_size_ = osi.cbSize;
        }
        update_sequence_header();
        return true;
    }
    if (FAILED(hr)) return fail(err, "encoder ProcessOutput", hr);

    IMFSample* s = provides_samples_ ? given.Get() : own.Get();
    if (!s) return true;

    DWORD buf_count = 0;
    s->GetBufferCount(&buf_count);
    out.data.clear();
    for (DWORD i = 0; i < buf_count; ++i) {
        ComPtr<IMFMediaBuffer> b;
        if (FAILED(s->GetBufferByIndex(i, &b)) || !b) continue;
        BYTE* p = nullptr;
        DWORD len = 0;
        if (SUCCEEDED(b->Lock(&p, nullptr, &len))) {
            if (p && len > 0) out.data.insert(out.data.end(), p, p + len);
            b->Unlock();
        }
    }
    out.keyframe = MFGetAttributeUINT32(s, MFSampleExtension_CleanPoint, FALSE) != 0;
    got_packet = true;
    return true;
}

void MfEncoder::finish_packet(bool keyframe, Packet& out) {
    if (config_.codec == Codec::AV1 && !out.data.empty() && ((out.data[0] >> 3) & 0x0F) != 2) {
        static const uint8_t kTd[2] = {0x12, 0x00};
        out.data.insert(out.data.begin(), kTd, kTd + 2);
    }

    const bool is_key = keyframe || out.keyframe || is_keyframe(config_.codec, out.data);
    if (is_key) {
        if (is_keyframe(config_.codec, out.data)) {
            if (seq_header_.empty()) seq_header_ = extract_headers(config_.codec, out.data);
        } else if (!seq_header_.empty()) {
            const size_t insert_at =
                (config_.codec == Codec::AV1 && out.data.size() >= 2 && ((out.data[0] >> 3) & 0x0F) == 2) ? 2 : 0;
            out.data.insert(out.data.begin() + std::ptrdiff_t(insert_at), seq_header_.begin(), seq_header_.end());
        }
    }
    out.keyframe = is_key;
}

}  // namespace brovideo::mf
