#pragma once
// The Media Foundation hardware encoder: the GPU vendor's encoder MFT for
// H.264, HEVC or AV1 (NVENC, AMF, QSV), driven asynchronously through
// IMFMediaEventGenerator (or synchronously when the MFT is sync) with a
// D3D11 device manager. Low-latency mode, no B-frames, CBR, NV12 input
// converted from CPU RGBA8/BGRA8/NV12 in BT.709 limited range and kept so
// encode_repeat() re-encodes the cached picture without holding the caller's
// frame.

#include "brovideo/encoder.h"
#include "mf_util.h"

#include <strmif.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace brovideo::mf {

class MfEncoder final : public Encoder {
public:
    static std::unique_ptr<MfEncoder> create(const EncoderConfig& config, std::string* err);
    ~MfEncoder() override;

    EncoderInfo info() const override;
    [[nodiscard]] std::string adapter() const { return adapter_; }

protected:
    bool encode_frame(const Frame& frame, bool keyframe, const std::function<void()>& release, Packet& out,
                      std::string* err) override;
    bool apply_config(const EncoderConfig& next, std::string* err) override;
    bool repeats() const override { return true; }
    bool encode_repeat_frame(bool keyframe, Packet& out, std::string* err) override;

private:
    explicit MfEncoder(const EncoderConfig& config) : Encoder(config) {}

    bool open_session(const EncoderConfig& cfg, std::string* err);
    bool init_mft(IMFActivate* act, const EncoderConfig& cfg, std::string* err);
    void close_session();
    void update_sequence_header();
    bool convert_to_nv12(const Frame& frame, std::string* err);
    bool encode_nv12(bool keyframe, Packet& out, std::string* err);
    bool pull_output(Packet& out, bool& got_packet, std::string* err);
    void finish_packet(bool keyframe, Packet& out);

    ComPtr<ID3D11Device> device_;
    ComPtr<IMFDXGIDeviceManager> manager_;
    std::string adapter_;
    std::string mft_name_;

    ComPtr<IMFTransform> mft_;
    ComPtr<ICodecAPI> codec_api_;
    ComPtr<IMFMediaEventGenerator> events_;
    bool is_async_ = false;
    DWORD in_id_ = 0;
    DWORD out_id_ = 0;
    bool provides_samples_ = false;
    DWORD out_sample_size_ = 0;

    uint32_t enc_w_ = 0;
    uint32_t enc_h_ = 0;
    uint32_t fps_ = 60;
    uint32_t need_input_ = 0;
    LONGLONG next_time_ = 0;

    std::vector<uint8_t> nv12_;
    std::vector<uint8_t> seq_header_;
};

}  // namespace brovideo::mf
