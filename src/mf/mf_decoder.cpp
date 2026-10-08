#include "mf_decoder.h"

#include "brovideo/bitstream.h"

#include <strmif.h>  // ICodecAPI
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

// What a D3D11 picture keeps alive: the MFT's output sample (so its surface
// is not reused), and the texture and device the Picture points at.
struct GpuHold {
    ComPtr<IMFSample> sample;
    ComPtr<ID3D11Texture2D> texture;
    ComPtr<ID3D11Device> device;
};

}  // namespace

std::unique_ptr<MfDecoder> MfDecoder::create(Codec codec, bool hardware, PictureMemory output, std::string* err) {
    if (!ensure_started(err)) return nullptr;
    std::unique_ptr<MfDecoder> d(new MfDecoder(codec, output));
    if (!d->init(hardware, err)) return nullptr;
    return d;
}

MfDecoder::~MfDecoder() {
    if (mft_) {
        mft_->ProcessMessage(MFT_MESSAGE_NOTIFY_END_STREAMING, 0);
        // Detach the device before the MFT goes, so it releases its surfaces.
        if (manager_) mft_->ProcessMessage(MFT_MESSAGE_SET_D3D_MANAGER, 0);
    }
}

bool MfDecoder::init(bool hardware, std::string* err) {
    mft_ = create_decoder_mft(codec(), err);
    if (!mft_) return false;

    bool d3d_aware = false;
    ComPtr<IMFAttributes> attrs;
    if (SUCCEEDED(mft_->GetAttributes(&attrs)) && attrs) {
        attrs->SetUINT32(MF_LOW_LATENCY, TRUE);
        d3d_aware = MFGetAttributeUINT32(attrs.Get(), MF_SA_D3D11_AWARE, FALSE) != 0;
    }
    // Low latency: each picture is output as soon as it is decoded instead
    // of after the MFT's reorder window fills.
    ComPtr<ICodecAPI> codec_api;
    if (SUCCEEDED(mft_.As(&codec_api))) {
        VARIANT v;
        VariantInit(&v);
        v.vt = VT_BOOL;
        v.boolVal = VARIANT_TRUE;
        codec_api->SetValue(&CODECAPI_AVLowLatencyMode, &v);
    }

    if (hardware && d3d_aware) {
        std::string why;
        if (create_d3d11(device_, manager_, &why)) {
            const HRESULT hr =
                mft_->ProcessMessage(MFT_MESSAGE_SET_D3D_MANAGER, reinterpret_cast<ULONG_PTR>(manager_.Get()));
            if (FAILED(hr)) {
                manager_.Reset();
                device_.Reset();
            } else {
                device_->GetImmediateContext(&context_);
            }
        }
    }

    GUID subtype{};
    subtype_for(codec(), subtype);
    ComPtr<IMFMediaType> in;
    HRESULT hr = MFCreateMediaType(&in);
    if (SUCCEEDED(hr)) hr = in->SetGUID(MF_MT_MAJOR_TYPE, MFMediaType_Video);
    if (SUCCEEDED(hr)) hr = in->SetGUID(MF_MT_SUBTYPE, subtype);
    if (SUCCEEDED(hr)) hr = in->SetUINT32(MF_MT_INTERLACE_MODE, MFVideoInterlace_Progressive);
    // The HEVC and AV1 MFTs offer no output type until the input has a size.
    // Any size will do: the real one comes from the stream, as a stream
    // change when the first keyframe is decoded.
    if (SUCCEEDED(hr)) hr = MFSetAttributeSize(in.Get(), MF_MT_FRAME_SIZE, 1280, 720);
    if (SUCCEEDED(hr)) hr = MFSetAttributeRatio(in.Get(), MF_MT_FRAME_RATE, 60, 1);
    if (SUCCEEDED(hr)) hr = in->SetUINT32(MF_LOW_LATENCY, TRUE);
    if (SUCCEEDED(hr)) hr = mft_->SetInputType(0, in.Get(), 0);
    if (FAILED(hr)) return fail(err, "setting the decoder's input type", hr);
    if (!negotiate_output(err)) return false;
    mft_->ProcessMessage(MFT_MESSAGE_NOTIFY_BEGIN_STREAMING, 0);
    mft_->ProcessMessage(MFT_MESSAGE_NOTIFY_START_OF_STREAM, 0);
    return true;
}

// Pick NV12 among the MFT's output types and note the geometry. Runs at
// start and on every MF_E_TRANSFORM_STREAM_CHANGE (a new size).
bool MfDecoder::negotiate_output(std::string* err) {
    ComPtr<IMFMediaType> chosen;
    for (DWORD i = 0;; ++i) {
        ComPtr<IMFMediaType> t;
        const HRESULT hr = mft_->GetOutputAvailableType(0, i, &t);
        if (hr == MF_E_NO_MORE_TYPES) break;
        if (FAILED(hr)) return fail(err, "listing the decoder's output types", hr);
        GUID sub{};
        if (SUCCEEDED(t->GetGUID(MF_MT_SUBTYPE, &sub)) && sub == MFVideoFormat_NV12) {
            chosen = t;
            break;
        }
    }
    if (!chosen) {
        if (err) *err = "the decoder offers no NV12 output";
        return false;
    }
    HRESULT hr = mft_->SetOutputType(0, chosen.Get(), 0);
    if (FAILED(hr)) return fail(err, "setting the decoder's output type", hr);

    coded_w_ = coded_h_ = 0;
    MFGetAttributeSize(chosen.Get(), MF_MT_FRAME_SIZE, &coded_w_, &coded_h_);
    vis_x_ = vis_y_ = 0;
    vis_w_ = coded_w_;
    vis_h_ = coded_h_;
    MFVideoArea area{};
    UINT32 got = 0;
    if (SUCCEEDED(chosen->GetBlob(MF_MT_MINIMUM_DISPLAY_APERTURE, reinterpret_cast<UINT8*>(&area), sizeof area,
                                  &got)) ||
        SUCCEEDED(chosen->GetBlob(MF_MT_GEOMETRIC_APERTURE, reinterpret_cast<UINT8*>(&area), sizeof area, &got))) {
        if (got == sizeof area && area.Area.cx > 0 && area.Area.cy > 0) {
            vis_x_ = UINT32(std::max<SHORT>(area.OffsetX.value, 0));
            vis_y_ = UINT32(std::max<SHORT>(area.OffsetY.value, 0));
            vis_w_ = std::min<UINT32>(UINT32(area.Area.cx), coded_w_ - std::min(vis_x_, coded_w_));
            vis_h_ = std::min<UINT32>(UINT32(area.Area.cy), coded_h_ - std::min(vis_y_, coded_h_));
        }
    }
    default_stride_ = MFGetAttributeUINT32(chosen.Get(), MF_MT_DEFAULT_STRIDE, coded_w_);
    if (LONG(default_stride_) <= 0) default_stride_ = coded_w_;

    MFT_OUTPUT_STREAM_INFO info{};
    hr = mft_->GetOutputStreamInfo(0, &info);
    if (FAILED(hr)) return fail(err, "reading the decoder's output stream info", hr);
    provides_samples_ =
        (info.dwFlags & (MFT_OUTPUT_STREAM_PROVIDES_SAMPLES | MFT_OUTPUT_STREAM_CAN_PROVIDE_SAMPLES)) != 0;
    sample_size_ = info.cbSize;
    return true;
}

bool MfDecoder::decode(std::span<const uint8_t> packet, int64_t pts_ns, Picture& out, std::string* err) {
    out.ready = false;
    if (!mft_) {
        if (err) *err = "the decoder is not usable after an earlier failure";
        return false;
    }
    if (packet.empty()) {
        if (err) *err = "empty packet";
        return false;
    }
    const bool key = is_keyframe(codec(), packet);
    if (!synced_ && !key) {
        if (err) *err = "lost sync: a predicted frame with no keyframe before it";
        return false;
    }
    bool ok = feed(packet, pts_ns, err) && drain(out, err);
    if (ok && !out.ready) {
        // Some MFTs (the HEVC extension decoding in software) hold each
        // picture until they see the next access unit begin, even in
        // low-latency mode with no reordering in the stream. Every packet
        // here is a whole picture, so say the next one has begun: an access
        // unit delimiter (a temporal delimiter for AV1) is legal before any
        // picture and decodes to nothing. (A drain would hand the picture
        // over too, but these MFTs then take no further input.)
        static const uint8_t kHevcAud[] = {0, 0, 0, 1, 0x46, 0x01, 0x50};
        static const uint8_t kH264Aud[] = {0, 0, 0, 1, 0x09, 0xF0};
        static const uint8_t kAv1Td[] = {0x12, 0x00};
        const std::span<const uint8_t> kick = codec() == Codec::HEVC   ? std::span<const uint8_t>(kHevcAud)
                                              : codec() == Codec::H264 ? std::span<const uint8_t>(kH264Aud)
                                                                       : std::span<const uint8_t>(kAv1Td);
        ok = feed(kick, pts_ns, err) && drain(out, err);
    }
    if (!ok) {
        reset_after_error();
        out.ready = false;
        out.gpu = GpuPicture{};
        return false;
    }
    if (key) synced_ = true;
    return true;
}

bool MfDecoder::feed(std::span<const uint8_t> bitstream, int64_t pts_ns, std::string* err) {
    ComPtr<IMFMediaBuffer> buf;
    HRESULT hr = MFCreateMemoryBuffer(DWORD(bitstream.size()), &buf);
    if (FAILED(hr)) return fail(err, "MFCreateMemoryBuffer", hr);
    BYTE* p = nullptr;
    hr = buf->Lock(&p, nullptr, nullptr);
    if (FAILED(hr)) return fail(err, "locking the input buffer", hr);
    std::memcpy(p, bitstream.data(), bitstream.size());
    buf->Unlock();
    buf->SetCurrentLength(DWORD(bitstream.size()));
    ComPtr<IMFSample> sample;
    hr = MFCreateSample(&sample);
    if (FAILED(hr)) return fail(err, "MFCreateSample", hr);
    sample->AddBuffer(buf.Get());
    // Sample times are a counter (the MFTs want them rising); the caller's
    // pts rides beside them and is found again by the output's time.
    sample->SetSampleTime(next_time_);
    sample->SetSampleDuration(1);
    pts_.emplace_back(next_time_, pts_ns);
    if (pts_.size() > 64) pts_.pop_front();
    ++next_time_;

    hr = mft_->ProcessInput(0, sample.Get(), 0);
    if (hr == MF_E_NOTACCEPTING) {
        // Output is pending that an earlier call did not collect: collect it
        // (and drop it: a newer picture is coming) and try again.
        Picture scratch;
        if (!drain(scratch, err)) return false;
        hr = mft_->ProcessInput(0, sample.Get(), 0);
    }
    if (FAILED(hr)) return fail(err, "the decoder rejected the packet", hr);
    return true;
}

int64_t MfDecoder::pts_for(IMFSample* sample) {
    LONGLONG t = 0;
    if (FAILED(sample->GetSampleTime(&t))) return pts_.empty() ? 0 : pts_.back().second;
    for (auto it = pts_.begin(); it != pts_.end(); ++it) {
        if (it->first == t) {
            const int64_t pts = it->second;
            pts_.erase(pts_.begin(), it);  // older ones will not come out any more
            return pts;
        }
    }
    return pts_.empty() ? 0 : pts_.back().second;
}

// Collect every picture the MFT has; `out` keeps the newest.
bool MfDecoder::drain(Picture& out, std::string* err) {
    for (int guard = 0; guard < 64; ++guard) {
        MFT_OUTPUT_DATA_BUFFER ob{};
        ob.dwStreamID = 0;
        ComPtr<IMFSample> own;
        if (!provides_samples_) {
            ComPtr<IMFMediaBuffer> b;
            HRESULT hr = MFCreateMemoryBuffer(sample_size_ ? sample_size_ : coded_w_ * coded_h_ * 3 / 2, &b);
            if (SUCCEEDED(hr)) hr = MFCreateSample(&own);
            if (SUCCEEDED(hr)) hr = own->AddBuffer(b.Get());
            if (FAILED(hr)) return fail(err, "allocating an output sample", hr);
            ob.pSample = own.Get();
        }
        DWORD status = 0;
        const HRESULT hr = mft_->ProcessOutput(0, 1, &ob, &status);
        if (ob.pEvents) ob.pEvents->Release();
        ComPtr<IMFSample> given;
        if (provides_samples_ && ob.pSample) given.Attach(ob.pSample);
        if (hr == MF_E_TRANSFORM_NEED_MORE_INPUT) return true;
        if (hr == MF_E_TRANSFORM_STREAM_CHANGE) {
            if (!negotiate_output(err)) return false;
            continue;
        }
        if (FAILED(hr)) return fail(err, "decoding", hr);
        IMFSample* s = provides_samples_ ? given.Get() : own.Get();
        if (!s) continue;
        if (!copy_out(s, out, err)) return false;
        out.pts_ns = pts_for(s);
        out.ready = true;
    }
    if (err) *err = "the decoder kept producing output";
    return false;
}

bool MfDecoder::copy_out(IMFSample* sample, Picture& out, std::string* err) {
    ComPtr<IMFMediaBuffer> buf;
    HRESULT hr = sample->GetBufferByIndex(0, &buf);
    if (FAILED(hr)) return fail(err, "reading the decoded sample", hr);
    ComPtr<IMFDXGIBuffer> dxgi;
    if (SUCCEEDED(buf.As(&dxgi))) {
        if (output_ == PictureMemory::D3D11) return hand_over(sample, dxgi.Get(), out, err);
        return copy_texture(dxgi.Get(), out, err);
    }
    return copy_memory(buf.Get(), out, err);
}

// A DXVA picture handed over as it is: the texture array slice, with the
// sample held so the MFT does not reuse the surface while the Picture lives.
bool MfDecoder::hand_over(IMFSample* sample, IMFDXGIBuffer* buffer, Picture& out, std::string* err) {
    auto hold = std::make_shared<GpuHold>();
    HRESULT hr = buffer->GetResource(IID_PPV_ARGS(&hold->texture));
    if (FAILED(hr)) return fail(err, "reading the decoded texture", hr);
    UINT sub = 0;
    buffer->GetSubresourceIndex(&sub);
    D3D11_TEXTURE2D_DESC d{};
    hold->texture->GetDesc(&d);
    if (d.Format != DXGI_FORMAT_NV12) {
        if (err) *err = "the decoder output a texture that is not NV12";
        return false;
    }
    if (vis_x_ + vis_w_ > d.Width || vis_y_ + vis_h_ > d.Height) {
        if (err) *err = "the decoded texture is smaller than the picture";
        return false;
    }
    hold->texture->GetDevice(&hold->device);
    hold->sample = sample;
    out.width = vis_w_;
    out.height = vis_h_;
    out.format = PixelFormat::NV12;
    out.memory = PictureMemory::D3D11;
    out.stride = 0;
    out.uv_offset = 0;
    out.data.clear();
    out.gpu.device = hold->device.Get();
    out.gpu.texture = hold->texture.Get();
    out.gpu.subresource = sub;
    out.gpu.x = vis_x_;
    out.gpu.y = vis_y_;
    out.gpu.keepalive = std::move(hold);
    return true;
}

// A DXVA picture: one slice of the decoder's texture array, copied to a
// staging texture and read back.
bool MfDecoder::copy_texture(IMFDXGIBuffer* buffer, Picture& out, std::string* err) {
    ComPtr<ID3D11Texture2D> tex;
    HRESULT hr = buffer->GetResource(IID_PPV_ARGS(&tex));
    if (FAILED(hr)) return fail(err, "reading the decoded texture", hr);
    UINT sub = 0;
    buffer->GetSubresourceIndex(&sub);
    D3D11_TEXTURE2D_DESC d{};
    tex->GetDesc(&d);
    if (d.Format != DXGI_FORMAT_NV12) {
        if (err) *err = "the decoder output a texture that is not NV12";
        return false;
    }
    if (!context_) {
        ComPtr<ID3D11Device> dev;
        tex->GetDevice(&dev);
        dev->GetImmediateContext(&context_);
    }
    if (!staging_ || staging_w_ != d.Width || staging_h_ != d.Height) {
        staging_.Reset();
        D3D11_TEXTURE2D_DESC s{};
        s.Width = d.Width;
        s.Height = d.Height;
        s.MipLevels = 1;
        s.ArraySize = 1;
        s.Format = DXGI_FORMAT_NV12;
        s.SampleDesc.Count = 1;
        s.Usage = D3D11_USAGE_STAGING;
        s.CPUAccessFlags = D3D11_CPU_ACCESS_READ;
        ComPtr<ID3D11Device> dev;
        tex->GetDevice(&dev);
        hr = dev->CreateTexture2D(&s, nullptr, &staging_);
        if (FAILED(hr)) return fail(err, "creating the staging texture", hr);
        staging_w_ = d.Width;
        staging_h_ = d.Height;
    }
    context_->CopySubresourceRegion(staging_.Get(), 0, 0, 0, 0, tex.Get(), sub, nullptr);
    D3D11_MAPPED_SUBRESOURCE m{};
    hr = context_->Map(staging_.Get(), 0, D3D11_MAP_READ, 0, &m);
    if (FAILED(hr)) return fail(err, "mapping the staging texture", hr);
    // The texture can be larger than the output type's frame size (DXVA
    // surfaces are allocated aligned); the UV plane follows its full height.
    const bool fits = vis_x_ + vis_w_ <= d.Width && vis_y_ + vis_h_ <= d.Height;
    const auto* y = static_cast<const uint8_t*>(m.pData);
    if (fits) pack(y, y + size_t(m.RowPitch) * d.Height, m.RowPitch, out);
    context_->Unmap(staging_.Get(), 0);
    if (!fits) {
        if (err) *err = "the decoded texture is smaller than the picture";
        return false;
    }
    return true;
}

// A software picture in a system-memory buffer: Y rows then UV rows, each
// plane coded_h_ rows tall.
bool MfDecoder::copy_memory(IMFMediaBuffer* buffer, Picture& out, std::string* err) {
    ComPtr<IMF2DBuffer> b2;
    if (SUCCEEDED(buffer->QueryInterface(IID_PPV_ARGS(&b2)))) {
        BYTE* scan0 = nullptr;
        LONG pitch = 0;
        const HRESULT hr = b2->Lock2D(&scan0, &pitch);
        if (FAILED(hr)) return fail(err, "locking the decoded picture", hr);
        if (pitch > 0) pack(scan0, scan0 + size_t(pitch) * coded_h_, size_t(pitch), out);
        b2->Unlock2D();
        if (pitch <= 0) {
            if (err) *err = "the decoded picture is bottom-up";
            return false;
        }
        return true;
    }
    BYTE* p = nullptr;
    DWORD max_len = 0, len = 0;
    const HRESULT hr = buffer->Lock(&p, &max_len, &len);
    if (FAILED(hr)) return fail(err, "locking the decoded picture", hr);
    const size_t pitch = default_stride_;
    const size_t need = pitch * coded_h_ + pitch * ((coded_h_ + 1) / 2);
    const bool ok = len >= need;
    if (ok) pack(p, p + pitch * coded_h_, pitch, out);
    buffer->Unlock();
    if (!ok) {
        if (err) *err = "the decoded picture is shorter than its size";
        return false;
    }
    return true;
}

// Crop the visible picture out of an NV12 buffer into `out`: Y rows `stride`
// bytes apart, then the interleaved UV rows at uv_offset.
void MfDecoder::pack(const uint8_t* y, const uint8_t* uv, size_t pitch, Picture& out) const {
    const uint32_t w = vis_w_, h = vis_h_;
    const uint32_t stride = (w + 1) & ~1u;
    const uint32_t chroma_rows = (h + 1) / 2;
    out.width = w;
    out.height = h;
    out.format = PixelFormat::NV12;
    out.memory = PictureMemory::Cpu;
    out.gpu = GpuPicture{};
    out.stride = stride;
    out.uv_offset = stride * h;
    out.data.resize(size_t(stride) * h + size_t(stride) * chroma_rows);
    const size_t x0 = vis_x_ & ~1u;
    const size_t row = std::min<size_t>(stride, pitch > x0 ? pitch - x0 : 0);
    for (uint32_t r = 0; r < h; ++r) {
        std::memcpy(out.data.data() + size_t(r) * stride, y + (size_t(vis_y_) + r) * pitch + x0, row);
    }
    uint8_t* dst = out.data.data() + out.uv_offset;
    for (uint32_t r = 0; r < chroma_rows; ++r) {
        std::memcpy(dst + size_t(r) * stride, uv + (size_t(vis_y_ / 2) + r) * pitch + x0, row);
    }
}

void MfDecoder::reset_after_error() {
    synced_ = false;
    pts_.clear();
    if (mft_) mft_->ProcessMessage(MFT_MESSAGE_COMMAND_FLUSH, 0);
}

std::string MfDecoder::adapter() const { return device_ ? adapter_name(device_.Get()) : std::string(); }

std::string MfDecoder::describe() const {
    std::string s = "Media Foundation ";
    s += codec_name(codec());
    s += hardware() ? ", hardware (D3D11)" : ", software";
    return s;
}

}  // namespace brovideo::mf
