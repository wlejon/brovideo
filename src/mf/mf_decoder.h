#pragma once
// The Media Foundation decoder: the system's decoder MFT for H.264, HEVC or
// AV1, with DXVA through a D3D11 device manager where the MFT and the machine
// allow it, otherwise in software. Low-latency mode, NV12 out: copied to CPU
// memory and cropped to the visible picture, or (PictureMemory::D3D11) the
// decoder's own texture slice handed over. Size changes inside the stream
// are renegotiated as the MFT reports them.

#include "brovideo/decoder.h"
#include "mf_util.h"

#include <deque>
#include <memory>
#include <span>
#include <string>
#include <utility>

namespace brovideo::mf {

class MfDecoder final : public Decoder {
public:
    // `hardware`: try a D3D11 device first (falling back to software when the
    // MFT or the machine cannot take one). `output`: where pictures go (a
    // software decode always comes to the CPU). Null with *err when the codec
    // has no usable MFT.
    static std::unique_ptr<MfDecoder> create(Codec codec, bool hardware, PictureMemory output, std::string* err);
    ~MfDecoder() override;

    bool decode(std::span<const uint8_t> packet, int64_t pts_ns, Picture& out, std::string* err) override;
    using Decoder::decode;
    [[nodiscard]] std::string describe() const override;
    // Whether the MFT took the D3D11 device (it decodes through DXVA).
    [[nodiscard]] bool hardware() const override { return manager_ != nullptr; }
    // The D3D11 adapter's name, when hardware().
    [[nodiscard]] std::string adapter() const;

private:
    MfDecoder(Codec codec, PictureMemory output) : Decoder(codec), output_(output) {}
    bool init(bool hardware, std::string* err);
    bool negotiate_output(std::string* err);
    bool feed(std::span<const uint8_t> bitstream, int64_t pts_ns, std::string* err);
    bool drain(Picture& out, std::string* err);
    bool copy_out(IMFSample* sample, Picture& out, std::string* err);
    bool hand_over(IMFSample* sample, IMFDXGIBuffer* buffer, Picture& out, std::string* err);
    bool copy_texture(IMFDXGIBuffer* buffer, Picture& out, std::string* err);
    bool copy_memory(IMFMediaBuffer* buffer, Picture& out, std::string* err);
    void pack(const uint8_t* y, const uint8_t* uv, size_t pitch, Picture& out) const;
    int64_t pts_for(IMFSample* sample);
    void reset_after_error();

    PictureMemory output_;
    ComPtr<IMFTransform> mft_;
    ComPtr<ID3D11Device> device_;
    ComPtr<ID3D11DeviceContext> context_;
    ComPtr<IMFDXGIDeviceManager> manager_;
    ComPtr<ID3D11Texture2D> staging_;
    UINT staging_w_ = 0, staging_h_ = 0;

    // The current output type.
    bool provides_samples_ = false;
    DWORD sample_size_ = 0;
    UINT32 coded_w_ = 0, coded_h_ = 0;   // the buffer's frame size (macroblock aligned)
    UINT32 vis_x_ = 0, vis_y_ = 0;       // the visible picture inside it
    UINT32 vis_w_ = 0, vis_h_ = 0;
    UINT32 default_stride_ = 0;

    bool synced_ = false;  // a keyframe has been taken since the start or the last error
    LONGLONG next_time_ = 0;
    // The pts of each packet in flight, by the sample time it was fed with.
    std::deque<std::pair<LONGLONG, int64_t>> pts_;
};

}  // namespace brovideo::mf
