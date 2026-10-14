// The Media Foundation backend's factory entry points. A codec is reported
// only when this machine decodes a real keyframe of it (a flat grey 192x192
// picture, encoded once with ffmpeg and embedded below): through D3D11 (a
// hardware capability) and in software (a software one), each only when it
// really works. An installed MFT that cannot actually decode (a missing
// extension's stub, a GPU without the codec) is not listed.
#include "backend.h"
#include "brovideo/bitstream.h"
#include "mf_decoder.h"
#include "mf_encoder.h"

namespace brovideo::detail {

namespace {

// libx264 baseline, SEI removed.
const uint8_t kProbeH264[] = {
    0x00, 0x00, 0x00, 0x01, 0x67, 0x42, 0xc0, 0x0c, 0xa6, 0x11, 0x0c, 0x19, 0xb0, 0x11, 0x00, 0x00, 0x03, 0x00, 0x01,
    0x00, 0x00, 0x03, 0x00, 0x02, 0x0f, 0x14, 0x2a, 0x11, 0x80, 0x00, 0x00, 0x00, 0x01, 0x68, 0xc8, 0x42, 0x0f, 0x2c,
    0x80, 0x00, 0x00, 0x01, 0x65, 0x88, 0x82, 0x02, 0xbe, 0x4c, 0x50, 0x00, 0x10, 0x2d, 0x93, 0x93, 0x93, 0x93, 0x93,
    0x93, 0x93, 0x93, 0x93, 0x93, 0x93, 0xae, 0xba, 0xeb, 0xae, 0xba, 0xeb, 0xae, 0xba, 0xeb, 0xae, 0xba, 0xeb, 0xae,
    0xba, 0xeb, 0xae, 0xba, 0xeb, 0xae, 0xba, 0xeb, 0xae, 0xba, 0xeb, 0xae, 0xba, 0xeb, 0xae, 0xba, 0xeb, 0xae, 0xba,
    0xeb, 0xae, 0xba, 0xeb, 0xae, 0xba, 0xeb, 0xae, 0xba, 0xeb, 0xae, 0xba, 0xeb, 0xae, 0xba, 0xeb, 0xae, 0xba, 0xeb,
    0xae, 0xba, 0xeb, 0xae, 0xba, 0xeb, 0xae, 0xba, 0xeb, 0xae, 0xba, 0xeb, 0xae, 0xba, 0xeb, 0xae, 0xba, 0xeb, 0xae,
    0xba, 0xeb, 0xae, 0xba, 0xeb, 0xae, 0xba, 0xeb, 0xae, 0xba, 0xeb, 0xae, 0xba, 0xeb, 0xae, 0xba, 0xeb, 0xae, 0xba,
    0xeb, 0xae, 0xba, 0xeb, 0xc0};

// libx265 Main, no B-frames (sps_max_num_reorder_pics 0, as the VA-API
// encoder writes), SEI removed.
const uint8_t kProbeHevc[] = {
    0x00, 0x00, 0x00, 0x01, 0x40, 0x01, 0x0c, 0x01, 0xff, 0xff, 0x01, 0x60, 0x00, 0x00, 0x03, 0x00, 0x90, 0x00, 0x00,
    0x03, 0x00, 0x00, 0x03, 0x00, 0x1e, 0x92, 0x80, 0x90, 0x00, 0x00, 0x00, 0x01, 0x42, 0x01, 0x01, 0x01, 0x60, 0x00,
    0x00, 0x03, 0x00, 0x90, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x00, 0x1e, 0xa0, 0x18, 0x20, 0x30, 0x59, 0x64, 0xa9,
    0x24, 0xca, 0xf0, 0x16, 0x80, 0x80, 0x00, 0x00, 0x03, 0x00, 0x80, 0x00, 0x00, 0x03, 0x00, 0x84, 0x00, 0x00, 0x00,
    0x01, 0x44, 0x01, 0xc1, 0x72, 0xb4, 0x62, 0x40, 0x00, 0x00, 0x01, 0x28, 0x01, 0xaf, 0x76, 0x4d, 0x68, 0xeb, 0xae,
    0xfb, 0xfe, 0x75, 0x57, 0xca, 0xc1, 0x71, 0x43, 0x16, 0xf5, 0xec, 0x40, 0xa6, 0x65, 0x35, 0x20, 0x28, 0x82, 0xee,
    0x93, 0x04, 0x10, 0x9b, 0x55, 0x40};

// libaom profile 0: a temporal unit (delimiter, sequence header, frame).
const uint8_t kProbeAv1[] = {0x12, 0x00, 0x0a, 0x0b, 0x00, 0x00, 0x00, 0x03, 0xbd, 0xfd, 0xf9, 0xb5,
                             0xf2, 0x00, 0x80, 0x32, 0x12, 0x10, 0x00, 0x90, 0x00, 0x00, 0x00, 0xa0,
                             0x00, 0x20, 0x01, 0xb5, 0x34, 0xcf, 0x1d, 0x7b, 0x2c, 0x26, 0x80};

std::span<const uint8_t> probe_stream(Codec c) {
    switch (c) {
        case Codec::H264: return kProbeH264;
        case Codec::HEVC: return kProbeHevc;
        case Codec::AV1: return kProbeAv1;
        default: return {};
    }
}

// Encodes a 256x256 grey frame; true when it produces a valid keyframe packet.
bool probe_encodes(Codec c, std::string* adapter, std::string& why) {
    EncoderConfig ec;
    ec.codec = c;
    ec.width = 256;
    ec.height = 256;
    ec.fps = 60;
    ec.bitrate_kbps = 4000;
    ec.allow_software = false;
    auto e = mf::MfEncoder::create(ec, &why);
    if (!e) return false;
    std::vector<uint8_t> nv12(256 * 256 * 3 / 2, 0x80);
    Frame f;
    f.width = 256;
    f.height = 256;
    f.cpu = nv12.data();
    f.cpu_format = PixelFormat::NV12;
    Packet pkt;
    if (!e->encode(f, true, {}, pkt, &why)) return false;
    if (pkt.data.empty() || !is_keyframe(c, pkt.data)) {
        why = "the encoder MFT did not produce a valid keyframe";
        return false;
    }
    if (adapter) *adapter = e->adapter();
    return true;
}

// Decodes the probe keyframe; true when it comes out 192x192 and grey
// (0x80 is luma 126 in limited range).
bool probe_decodes(Codec c, bool hardware, bool* got_hardware, std::string* adapter, std::string& why) {
    auto d = mf::MfDecoder::create(c, hardware, PictureMemory::Cpu, &why);
    if (!d) return false;
    Picture f;
    if (!d->decode(probe_stream(c), f, &why)) return false;
    if (!f.ready) {
        why = "the decoder gave no picture for a keyframe";
        return false;
    }
    if (f.width != 192 || f.height != 192 || f.format != PixelFormat::NV12) {
        why = "the decoder gave a picture of the wrong size";
        return false;
    }
    const int y = f.data[size_t(96) * f.stride + 96];
    const int u = f.data[f.uv_offset + size_t(48) * f.stride + 96];
    const int v = f.data[f.uv_offset + size_t(48) * f.stride + 97];
    if (y < 118 || y > 134 || u < 120 || u > 136 || v < 120 || v > 136) {
        why = "the decoder's picture is wrong (" + d->describe() + ")";
        return false;
    }
    if (got_hardware) *got_hardware = d->hardware();
    if (adapter) *adapter = d->adapter();
    return true;
}

Capability make_capability(Codec c, Direction dir, bool hardware, const std::string& device) {
    Capability cap;
    cap.codec = c;
    cap.direction = dir;
    cap.backend = Backend::MediaFoundation;
    cap.hardware = hardware;
    cap.device = device;
    return cap;
}

ProbeResult probe() {
    ProbeResult r;
    std::string err;
    if (!mf::ensure_started(&err)) {
        for (Codec c : {Codec::H264, Codec::HEVC, Codec::AV1}) {
            r.missing.push_back({c, Direction::Encode, err});
            r.missing.push_back({c, Direction::Decode, err});
        }
        return r;
    }
    for (Codec c : {Codec::H264, Codec::HEVC, Codec::AV1}) {
        std::string enc_adapter, why_enc;
        if (probe_encodes(c, &enc_adapter, why_enc)) {
            r.caps.push_back(make_capability(c, Direction::Encode, true,
                                             enc_adapter.empty() ? "D3D11" : enc_adapter + " (D3D11)"));
        } else {
            r.missing.push_back({c, Direction::Encode, "Media Foundation: " + why_enc});
        }

        bool hw = false;
        std::string adapter, why_hw, why_sw;
        const bool hw_ok = probe_decodes(c, true, &hw, &adapter, why_hw);
        if (hw_ok && hw) {
            r.caps.push_back(make_capability(c, Direction::Decode, true, adapter.empty() ? "D3D11" : adapter + " (D3D11)"));
        }
        if (hw_ok && !hw) {
            // The MFT would not take the device and decoded in software:
            // that was the software probe.
            r.caps.push_back(make_capability(c, Direction::Decode, false, "CPU"));
        } else if (probe_decodes(c, false, nullptr, nullptr, why_sw)) {
            r.caps.push_back(make_capability(c, Direction::Decode, false, "CPU"));
        } else if (!hw_ok) {
            const std::string why =
                why_sw.empty() || why_sw == why_hw ? why_hw : why_hw + "; in software: " + why_sw;
            r.missing.push_back({c, Direction::Decode, "Media Foundation: " + why});
        }
    }
    return r;
}

std::unique_ptr<Encoder> create_encoder(const EncoderConfig& config, const Capability&, std::string* err) {
    return mf::MfEncoder::create(config, err);
}

std::unique_ptr<Decoder> create_decoder(const DecoderConfig& config, const Capability& cap, std::string* err) {
    auto d = mf::MfDecoder::create(config.codec, cap.hardware, config.output, err);
    // An MFT that refuses the D3D11 device falls back to software, which
    // Require does not accept.
    if (d && config.hardware == Hardware::Require && !d->hardware()) {
        if (err) *err = std::string("the ") + codec_name(config.codec) + " decoder refused the D3D11 device";
        return nullptr;
    }
    return d;
}

}  // namespace

const BackendOps& mf_backend() {
    static const BackendOps ops{Backend::MediaFoundation, probe, create_encoder, create_decoder};
    return ops;
}

}  // namespace brovideo::detail
