// Everywhere: the Raw codec through the factory (exact pixels, keyframes,
// strides, BGRA, hostile packets), the Encoder base's contract (release
// exactly once, forced / requested keyframes, reconfigure), the capability
// probe, names, and keyframe detection for every codec.
#include "brovideo/brovideo.h"
#include "check.h"

#include <cstring>
#include <random>

using namespace brovideo;

namespace {

std::vector<uint8_t> pattern(uint32_t w, uint32_t h, uint32_t seed) {
    std::vector<uint8_t> px(size_t(w) * h * 4);
    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            uint8_t* p = &px[(size_t(y) * w + x) * 4];
            p[0] = uint8_t(x * 3 + seed);
            p[1] = uint8_t(y * 5);
            p[2] = uint8_t((x + y) / 4 == 3 ? 0 : seed * 7);
            p[3] = 255;
        }
    }
    return px;
}

std::unique_ptr<Encoder> raw_encoder(uint32_t w, uint32_t h, std::string* err) {
    EncoderConfig ec;
    ec.codec = Codec::Raw;
    ec.width = w;
    ec.height = h;
    ec.bitrate_kbps = 1000;
    return create_encoder(ec, err);
}

std::unique_ptr<Decoder> raw_decoder(std::string* err) {
    DecoderConfig dc;
    dc.codec = Codec::Raw;
    return create_decoder(dc, err);
}

Frame cpu_frame(const std::vector<uint8_t>& px, uint32_t w, uint32_t h) {
    Frame f;
    f.width = w;
    f.height = h;
    f.cpu = px.data();
    return f;
}

void test_raw_codec() {
    check::phase("raw codec");
    std::string err;
    const uint32_t w = 61, h = 37;  // odd sizes
    auto enc = raw_encoder(w, h, &err);
    auto dec = raw_decoder(&err);
    CHECK(enc && dec);
    if (!enc || !dec) return;
    CHECK(!dec->hardware());
    CHECK(enc->info().backend == Backend::Raw);
    for (uint32_t i = 0; i < 6; ++i) {
        // A padded stride on odd frames.
        const uint32_t stride = (i & 1) ? w * 4 + 12 : 0;
        std::vector<uint8_t> src = pattern(w, h, i < 3 ? 0 : i);  // frames 1, 2 repeat frame 0
        std::vector<uint8_t> padded;
        const uint8_t* cpu = src.data();
        if (stride) {
            padded.assign(size_t(stride) * h, 0xEE);
            for (uint32_t y = 0; y < h; ++y) std::memcpy(&padded[size_t(y) * stride], &src[size_t(y) * w * 4], w * 4);
            cpu = padded.data();
        }
        Frame f;
        f.width = w;
        f.height = h;
        f.cpu = cpu;
        f.cpu_stride = stride;
        f.pts_ns = i;
        int released = 0;
        Packet pkt;
        CHECK(enc->encode(f, i == 4, [&] { ++released; }, pkt, &err));
        CHECK_EQ(released, 1);
        CHECK_EQ(pkt.keyframe, i == 0 || i == 4);
        CHECK_EQ(is_keyframe(Codec::Raw, pkt.data), pkt.keyframe);
        CHECK_EQ(pkt.pts_ns, int64_t(i));
        if (i == 1 || i == 2) CHECK(pkt.data.size() < 16);  // an unchanged frame is a single run
        Picture out;
        CHECK(dec->decode(pkt.data, int64_t(i) * 10, out, &err));
        CHECK(out.ready);
        CHECK_EQ(out.width, w);
        CHECK_EQ(out.height, h);
        CHECK_EQ(out.stride, w * 4);
        CHECK(out.format == PixelFormat::RGBA8);
        CHECK(out.memory == PictureMemory::Cpu);
        CHECK_EQ(out.pts_ns, int64_t(i) * 10);
        CHECK(out.data == src);
    }
    // Wrong size and non-CPU frames fail, still releasing exactly once; the
    // packet after a failure is a keyframe.
    {
        std::vector<uint8_t> px(size_t(w + 1) * h * 4);
        Frame f = cpu_frame(px, w + 1, h);
        int released = 0;
        Packet pkt;
        CHECK(!enc->encode(f, false, [&] { ++released; }, pkt, &err));
        CHECK_EQ(released, 1);
        Frame d;
        d.width = w;
        d.height = h;
        d.plane_count = 1;
        CHECK(!enc->encode(d, false, [&] { ++released; }, pkt, &err));
        CHECK_EQ(released, 2);
        const auto src = pattern(w, h, 9);
        CHECK(enc->encode(cpu_frame(src, w, h), false, [&] { ++released; }, pkt, &err));
        CHECK(pkt.keyframe);
    }
    check::phase("raw codec hostile");
    {
        auto d2 = raw_decoder(&err);
        Picture out;
        // A delta frame with no reference: lost sync.
        std::vector<uint8_t> delta = {1, 2, 2, 0x07, 0, 0, 0, 0};
        CHECK(!d2->decode(delta, out, &err));
        CHECK(!is_keyframe(Codec::Raw, delta));
        // Truncated, oversized and overrunning packets.
        CHECK(!d2->decode(std::vector<uint8_t>{}, out, &err));
        CHECK(!d2->decode(std::vector<uint8_t>{0, 2}, out, &err));
        std::vector<uint8_t> huge = {0, 0x80, 0x80, 0x04, 0x80, 0x80, 0x04};  // 65536 x 65536
        CHECK(!d2->decode(huge, out, &err));
        std::vector<uint8_t> overrun = {0, 2, 2, 0x0F, 1, 2, 3, 4};  // a run of 8 into 4 pixels
        CHECK(!d2->decode(overrun, out, &err));
        std::vector<uint8_t> shortlit = {0, 2, 2, 0x06, 1, 2, 3};  // a literal of 4 pixels, 3 bytes
        CHECK(!d2->decode(shortlit, out, &err));
        std::vector<uint8_t> ok = {0, 2, 2, 0x07, 9, 8, 7, 6};
        CHECK(d2->decode(ok, out, &err));
        CHECK_EQ(out.data.size(), size_t(16));
        CHECK_EQ(out.data[15], uint8_t(6));
        // After an error, a delta frame must not decode against the old picture.
        CHECK(!d2->decode(overrun, out, &err));
        CHECK(!d2->decode(delta, out, &err));
        std::mt19937 rng(99);
        for (int i = 0; i < 5000; ++i) {
            std::vector<uint8_t> junk(size_t(rng() % 32));
            for (auto& b : junk) b = uint8_t(rng());
            if (!junk.empty()) junk[0] &= 1;
            if (junk.size() > 2) junk[1] &= 0x3F, junk[2] &= 0x3F;
            d2->decode(junk, out, &err);
        }
        CHECK(true);
    }
}

void test_encoder_contract() {
    check::phase("encoder: BGRA, requested keyframes, reconfigure");
    std::string err;
    const uint32_t w = 40, h = 24;
    auto enc = raw_encoder(w, h, &err);
    auto dec = raw_decoder(&err);
    CHECK(enc && dec);
    if (!enc || !dec) return;
    // BGRA in, RGBA out.
    auto rgba = pattern(w, h, 3);
    auto bgra = rgba;
    for (size_t i = 0; i < bgra.size(); i += 4) std::swap(bgra[i], bgra[i + 2]);
    Frame f = cpu_frame(bgra, w, h);
    f.cpu_format = PixelFormat::BGRA8;
    Packet pkt;
    Picture out;
    CHECK(enc->encode(f, false, {}, pkt, &err));  // an empty release is allowed
    CHECK(pkt.keyframe);
    CHECK(dec->decode(pkt.data, out, &err));
    CHECK(out.data == rgba);
    f.cpu_format = PixelFormat::NV12;
    CHECK(!enc->encode(f, false, {}, pkt, &err));

    // request_keyframe(): the next packet only.
    const Frame rf = cpu_frame(rgba, w, h);
    CHECK(enc->encode(rf, false, {}, pkt, &err));
    CHECK(pkt.keyframe);  // after the failure above
    CHECK(enc->encode(rf, false, {}, pkt, &err));
    CHECK(!pkt.keyframe);
    enc->request_keyframe();
    CHECK(enc->encode(rf, false, {}, pkt, &err));
    CHECK(pkt.keyframe);
    CHECK(enc->encode(rf, false, {}, pkt, &err));
    CHECK(!pkt.keyframe);

    // reconfigure(): a new size from the next frame, which is a keyframe; the
    // codec cannot change, and a refused config leaves the encoder as it was.
    EncoderConfig next = enc->config();
    next.codec = Codec::H264;
    CHECK(!enc->reconfigure(next, &err));
    next = enc->config();
    next.width = 0;
    CHECK(!enc->reconfigure(next, &err));
    CHECK_EQ(enc->config().width, w);
    next = enc->config();
    next.width = 32;
    next.height = 16;
    next.bitrate_kbps = 500;
    CHECK(enc->reconfigure(next, &err));
    CHECK_EQ(enc->config().width, uint32_t(32));
    CHECK_EQ(enc->config().bitrate_kbps, uint32_t(500));
    const auto small = pattern(32, 16, 5);
    CHECK(enc->encode(cpu_frame(small, 32, 16), false, {}, pkt, &err));
    CHECK(pkt.keyframe);
    CHECK(dec->decode(pkt.data, out, &err));
    CHECK_EQ(out.width, uint32_t(32));
    CHECK(out.data == small);
}

void test_factory() {
    check::phase("factory and probe");
    std::string err;
    const auto encs = codecs(Direction::Encode);
    const auto decs = codecs(Direction::Decode);
    CHECK(!encs.empty() && encs.front() == Codec::Raw);
    CHECK(!decs.empty() && decs.front() == Codec::Raw);
    for (const Capability& c : capabilities()) {
        std::printf("   %s %s: %s, %s, %s\n", codec_name(c.codec), c.direction == Direction::Encode ? "encode" : "decode",
                    backend_name(c.backend), c.hardware ? "hardware" : "software", c.device.c_str());
    }
    for (Codec c : all_codecs()) {
        for (Direction d : {Direction::Encode, Direction::Decode}) {
            const bool can = supported(c, d);
            CHECK_EQ(unavailable_reason(c, d).empty(), can);
            CHECK_EQ(best(c, d) != nullptr, can);
            if (!can) {
                std::printf("   no %s %s: %s\n", codec_name(c), d == Direction::Encode ? "encoder" : "decoder",
                            unavailable_reason(c, d).c_str());
            }
        }
    }
    EncoderConfig empty;
    empty.codec = Codec::Raw;
    CHECK(!create_encoder(empty, &err));
    EncoderConfig hw_only;
    hw_only.codec = Codec::Raw;
    hw_only.width = hw_only.height = 16;
    hw_only.allow_software = false;
    CHECK(!create_encoder(hw_only, &err));
    CHECK(!err.empty());
    CHECK(parse_codec("h265") == Codec::HEVC);
    CHECK(parse_codec("raw") == Codec::Raw);
    CHECK(parse_codec("av1") == Codec::AV1);
    CHECK(!parse_codec("vp9"));
    for (Codec c : all_codecs()) CHECK(parse_codec(codec_name(c)) == c);
    CHECK_EQ(std::string(backend_name(Backend::VaApi)), std::string("vaapi"));
}

void test_bitstream() {
    check::phase("keyframe detection");
    // H.264: an IDR slice (type 5) is a keyframe, a non-IDR slice (1) is not.
    const std::vector<uint8_t> h264_idr = {0, 0, 0, 1, 0x67, 0x42, 0, 0, 1, 0x68, 0xce, 0, 0, 1, 0x65, 0x88, 0x84};
    const std::vector<uint8_t> h264_p = {0, 0, 0, 1, 0x41, 0x9a, 0x02};
    CHECK(is_keyframe(Codec::H264, h264_idr));
    CHECK(!is_keyframe(Codec::H264, h264_p));
    // HEVC: IDR_W_RADL (19) and CRA (21) are IRAP, TRAIL_R (1) is not.
    CHECK(is_keyframe(Codec::HEVC, std::vector<uint8_t>{0, 0, 1, 19 << 1, 1, 0xaf}));
    CHECK(is_keyframe(Codec::HEVC, std::vector<uint8_t>{0, 0, 1, 21 << 1, 1, 0xaf}));
    CHECK(!is_keyframe(Codec::HEVC, std::vector<uint8_t>{0, 0, 1, 1 << 1, 1, 0xd0}));
    // AV1: delimiter, sequence header, then a frame whose frame_type is KEY
    // (0) or INTER (1).
    const std::vector<uint8_t> av1_key = {0x12, 0x00, 0x0a, 0x01, 0x00, 0x32, 0x01, 0x10};
    const std::vector<uint8_t> av1_inter = {0x12, 0x00, 0x0a, 0x01, 0x00, 0x32, 0x01, 0x30};
    const std::vector<uint8_t> av1_no_seq = {0x12, 0x00, 0x32, 0x01, 0x10};
    CHECK(is_keyframe(Codec::AV1, av1_key));
    CHECK(!is_keyframe(Codec::AV1, av1_inter));
    CHECK(!is_keyframe(Codec::AV1, av1_no_seq));
    // Truncated or empty input is never a keyframe, and never a crash.
    std::mt19937 rng(5);
    for (Codec c : all_codecs()) {
        CHECK(!is_keyframe(c, std::vector<uint8_t>{}));
        for (int i = 0; i < 2000; ++i) {
            std::vector<uint8_t> junk(size_t(rng() % 24));
            for (auto& b : junk) b = uint8_t(rng());
            (void)is_keyframe(c, junk);
        }
    }
}

}  // namespace

int main() {
    check::watchdog(120);
    test_raw_codec();
    test_encoder_contract();
    test_factory();
    test_bitstream();
    return check::finish();
}
