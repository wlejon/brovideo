// AV1 Main (profile 0) for the VA-API encoder, in the same shape as H.264:
// a KEY_FRAME, then INTER_FRAMEs that each reference only the frame before
// it, every one shown at once. Reference slot 0 always holds the previous
// frame: a key frame refreshes all eight slots, an inter frame refreshes slot
// 0, and all seven ref_frame_idx point at it. The packet is one temporal
// unit: a temporal delimiter, the sequence header on key frames, the frame
// header, the tile group.
//
// Headers are always packed (the VA AV1 interface leaves the uncompressed
// headers to the application; the driver patches qindex, loop filter and
// CDEF through the bit offsets given here, or, like radeonsi, parses and
// rewrites the frame header).
//
// AV1 has no cropping. The headers ask for the visible size, but radeonsi
// encodes the frame at its surface alignment (64x16, and 1080 lines as
// 1082) and rewrites the sequence header to match; render_size, which it
// keeps, is the only place the visible size survives: a consumer crops
// decoded pictures to the size it configured. docs/design.md has the rest.
#include "vaapi/va_codec.h"

#include <va/va_enc_av1.h>

#include <algorithm>

namespace brovideo::vaapi {
namespace {

constexpr uint8_t kObuSequenceHeader = 1;
constexpr uint8_t kObuTemporalDelimiter = 2;
constexpr uint8_t kObuFrameHeader = 3;
constexpr uint32_t kOrderHintBits = 8;
constexpr uint8_t kPrimaryRefNone = 7;
constexpr uint8_t kBaseQindex = 128;  // a default; CBR replaces it per frame
constexpr uint8_t kLoopFilterY = 16;
constexpr uint8_t kLoopFilterUV = 8;

struct LevelLimit {
    uint8_t idx;        // seq_level_idx
    uint64_t max_ps;    // MaxPicSize
    uint64_t max_rate;  // MaxDisplayRate (samples per second)
    uint32_t max_kbps;  // Main tier
};
constexpr LevelLimit kLevels[] = {
    {0, 147456, 4423680, 1500},          {1, 278784, 8363520, 3000},          {4, 665856, 19975680, 6000},
    {5, 1065024, 31950720, 10000},       {8, 2359296, 70778880, 12000},       {9, 2359296, 141557760, 20000},
    {12, 8912896, 267386880, 30000},     {13, 8912896, 534773760, 40000},     {14, 8912896, 1069547520, 60000},
    {16, 35651584, 1069547520, 60000},   {17, 35651584, 2139095040, 100000},  {18, 35651584, 4278190080, 160000},
};

uint8_t pick_level(uint64_t samples, uint32_t fps, uint32_t kbps) {
    for (const LevelLimit& l : kLevels) {
        if (samples <= l.max_ps && samples * fps <= l.max_rate && kbps <= l.max_kbps) return l.idx;
    }
    return 19;
}

// tile_log2(blkSize, target): the smallest k with blkSize << k >= target.
uint32_t tile_log2(uint32_t blk, uint32_t target) {
    uint32_t k = 0;
    while ((blk << k) < target) ++k;
    return k;
}

std::vector<uint8_t> obu(uint8_t type, const std::vector<uint8_t>& payload) {
    BitWriter b;
    b.u(8, uint32_t(type << 3) | 0x02);  // obu_has_size_field
    b.leb128(payload.size());
    b.append_bytes(payload);
    return b.bytes();
}

class Av1 final : public CodecImpl {
public:
    Av1(const StreamParams& sp) : sp_(sp) {
        level_ = pick_level(uint64_t(sp.width) * sp.height, sp.fps, sp.bitrate_bps / 1000);
        // Tiles: as few as the limits allow (one, below 4096 pixels wide),
        // uniformly spaced, 64x64 superblocks.
        const uint32_t mi_cols = 2 * ((sp.width + 7) >> 3);
        const uint32_t mi_rows = 2 * ((sp.height + 7) >> 3);
        sb_cols_ = (mi_cols + 15) >> 4;
        sb_rows_ = (mi_rows + 15) >> 4;
        const uint32_t max_tile_width_sb = 4096 >> 6;
        const uint32_t max_tile_area_sb = (4096 * 2304) >> 12;
        min_log2_cols_ = tile_log2(max_tile_width_sb, sb_cols_);
        max_log2_cols_ = tile_log2(1, std::min(sb_cols_, 64u));
        max_log2_rows_ = tile_log2(1, std::min(sb_rows_, 64u));
        const uint32_t min_log2_tiles = std::max(min_log2_cols_, tile_log2(max_tile_area_sb, sb_rows_ * sb_cols_));
        cols_log2_ = min_log2_cols_;
        rows_log2_ = min_log2_tiles > cols_log2_ ? min_log2_tiles - cols_log2_ : 0;
        tile_w_sb_ = (sb_cols_ + (1u << cols_log2_) - 1) >> cols_log2_;
        tile_h_sb_ = (sb_rows_ + (1u << rows_log2_) - 1) >> rows_log2_;
        tile_cols_ = (sb_cols_ + tile_w_sb_ - 1) / tile_w_sb_;
        tile_rows_ = (sb_rows_ + tile_h_sb_ - 1) / tile_h_sb_;
        const uint32_t ext2 = sp.caps.av1_ext2;
        tile_size_bytes_minus1_ = ext2 == VA_ATTRIB_NOT_SUPPORTED ? 3 : (ext2 & 3);
        debug_log("av1: level idx %u, %ux%u superblocks, %ux%u tiles, packed headers offered 0x%x", unsigned(level_),
                  sb_cols_, sb_rows_, tile_cols_, tile_rows_, sp.caps.packed_headers);
    }

    uint32_t packed_headers() const override { return VA_ENC_PACKED_HEADER_SEQUENCE | VA_ENC_PACKED_HEADER_PICTURE; }

    bool add_sequence(BufferList& bufs, const PictureParams&, std::string* err) override {
        VAEncSequenceParameterBufferAV1 s{};
        s.seq_profile = 0;
        s.seq_level_idx = level_;
        s.seq_tier = 0;
        s.intra_period = 0;
        s.ip_period = 1;
        s.bits_per_second = sp_.bitrate_bps;
        s.seq_fields.bits.enable_order_hint = 1;
        s.seq_fields.bits.enable_cdef = 1;
        s.seq_fields.bits.subsampling_x = 1;
        s.seq_fields.bits.subsampling_y = 1;
        s.order_hint_bits_minus_1 = kOrderHintBits - 1;
        return bufs.add(VAEncSequenceParameterBufferType, s, err);
    }

    bool add_picture(BufferList& bufs, const PictureParams& pic, std::string* err) override {
        // Packed headers first: the frame header's bit offsets go into the
        // picture parameters.
        std::vector<uint8_t> seq_data;
        if (pic.keyframe) {
            BitWriter sh;
            write_sequence_header(sh);
            seq_data = obu(kObuSequenceHeader, sh.bytes());
        }
        FrameHeader fh = write_frame_header(pic);
        std::vector<uint8_t> pic_data;
        const uint32_t obu_start = 0;
        pic_data.push_back(uint8_t(kObuFrameHeader << 3) | 0x02);
        // obu_size in a fixed four-byte LEB128, so a driver can rewrite it in place.
        const uint32_t size = uint32_t(fh.bits.bytes().size());
        pic_data.push_back(uint8_t((size & 0x7f) | 0x80));
        pic_data.push_back(uint8_t(((size >> 7) & 0x7f) | 0x80));
        pic_data.push_back(uint8_t(((size >> 14) & 0x7f) | 0x80));
        pic_data.push_back(uint8_t((size >> 21) & 0x7f));
        const uint32_t payload_bit = (obu_start + 5) * 8;
        pic_data.insert(pic_data.end(), fh.bits.bytes().begin(), fh.bits.bytes().end());

        VAEncPictureParameterBufferAV1 p{};
        p.frame_width_minus_1 = uint16_t(sp_.width - 1);
        p.frame_height_minus_1 = uint16_t(sp_.height - 1);
        p.reconstructed_frame = pic.recon;
        p.coded_buf = pic.coded;
        for (auto& r : p.reference_frames) r = VA_INVALID_SURFACE;
        if (!pic.keyframe) {
            p.reference_frames[0] = pic.ref;
            for (auto& i : p.ref_frame_idx) i = 0;
            p.ref_frame_ctrl_l0.fields.search_idx0 = 1;  // LAST_FRAME
        }
        p.primary_ref_frame = pic.keyframe ? kPrimaryRefNone : 0;
        p.order_hint = uint8_t(pic.index_in_gop % (1u << kOrderHintBits));
        p.refresh_frame_flags = pic.keyframe ? 0xff : 0x01;
        p.picture_flags.bits.frame_type = pic.keyframe ? 0 : 1;
        p.picture_flags.bits.error_resilient_mode = pic.keyframe;
        p.picture_flags.bits.enable_frame_obu = 0;
        p.filter_level[0] = kLoopFilterY;
        p.filter_level[1] = kLoopFilterY;
        p.filter_level_u = kLoopFilterUV;
        p.filter_level_v = kLoopFilterUV;
        p.interpolation_filter = 4;  // switchable
        p.base_qindex = kBaseQindex;
        p.min_base_qindex = 1;
        p.max_base_qindex = 255;
        p.mode_control_flags.bits.tx_mode = 2;  // TX_MODE_SELECT
        p.mode_control_flags.bits.reference_mode = 0;  // single reference
        p.tile_cols = uint8_t(tile_cols_);
        p.tile_rows = uint8_t(tile_rows_);
        for (uint32_t i = 0; i < tile_cols_; ++i) {
            p.width_in_sbs_minus_1[i] = uint16_t(std::min(tile_w_sb_, sb_cols_ - i * tile_w_sb_) - 1);
        }
        for (uint32_t i = 0; i < tile_rows_; ++i) {
            p.height_in_sbs_minus_1[i] = uint16_t(std::min(tile_h_sb_, sb_rows_ - i * tile_h_sb_) - 1);
        }
        p.context_update_tile_id = 0;
        p.cdef_damping_minus_3 = 2;
        p.cdef_bits = 0;
        p.cdef_y_strengths[0] = 4 << 2 | 1;   // primary 4, secondary 1
        p.cdef_uv_strengths[0] = 2 << 2 | 0;
        p.bit_offset_qindex = payload_bit + fh.qindex;
        p.bit_offset_segmentation = payload_bit + fh.segmentation;
        p.bit_offset_loopfilter_params = payload_bit + fh.loop_filter;
        p.bit_offset_cdef_params = payload_bit + fh.cdef;
        p.size_in_bits_cdef_params = fh.cdef_bits;
        p.byte_offset_frame_hdr_obu_size = obu_start + 1;
        p.size_in_bits_frame_hdr_obu = 8 + 32 + fh.header_bits;  // up to and including the trailing one bit
        p.tile_group_obu_hdr_info.bits.obu_has_size_field = 1;
        if (!bufs.add(VAEncPictureParameterBufferType, p, err)) return false;

        if (!seq_data.empty() && !bufs.add_packed(VAEncPackedHeaderSequence, seq_data, seq_data.size() * 8, err, false)) {
            return false;
        }
        if (!bufs.add_packed(VAEncPackedHeaderPicture, pic_data, pic_data.size() * 8, err, false)) return false;

        VAEncTileGroupBufferAV1 tg{};
        tg.tg_start = 0;
        tg.tg_end = uint8_t(tile_cols_ * tile_rows_ - 1);
        return bufs.add(VAEncSliceParameterBufferType, tg, err);
    }

    // A temporal unit starts with a temporal delimiter. It cannot go in a
    // packed header (radeonsi then fails to parse the sequence header and
    // divides by zero in vaEndPicture) and the driver does not write one, so
    // it is added here unless the driver already did.
    void finish_packet(std::vector<uint8_t>& data) override {
        if (data.size() >= 1 && ((data[0] >> 3) & 0xf) == kObuTemporalDelimiter) return;
        static const uint8_t td[2] = {uint8_t(kObuTemporalDelimiter << 3) | 0x02, 0x00};
        data.insert(data.begin(), td, td + 2);
    }

private:
    struct FrameHeader {
        BitWriter bits;
        // Bit offsets within the frame header OBU's payload.
        uint32_t qindex = 0, segmentation = 0, loop_filter = 0, cdef = 0, cdef_bits = 0;
        uint32_t header_bits = 0;  // through the trailing one bit
    };

    void write_sequence_header(BitWriter& b) const {
        b.u(3, 0);      // seq_profile: Main
        b.flag(false);  // still_picture
        b.flag(false);  // reduced_still_picture_header
        // timing_info: radeonsi takes the frame rate from it (without it the
        // driver divides by zero in vaEndPicture).
        b.flag(true);   // timing_info_present_flag
        b.u(32, 1);     // num_units_in_display_tick
        b.u(32, sp_.fps);  // time_scale
        b.flag(false);  // equal_picture_interval: the rate is a hint
        b.flag(false);  // decoder_model_info_present_flag
        b.flag(false);  // initial_display_delay_present_flag
        b.u(5, 0);      // operating_points_cnt_minus_1
        b.u(12, 0);     // operating_point_idc[0]
        b.u(5, level_); // seq_level_idx[0]
        if (level_ > 7) b.flag(false);  // seq_tier[0]: Main
        b.u(4, 15);     // frame_width_bits_minus_1
        b.u(4, 15);     // frame_height_bits_minus_1
        b.u(16, sp_.width - 1);   // max_frame_width_minus_1
        b.u(16, sp_.height - 1);  // max_frame_height_minus_1
        b.flag(false);  // frame_id_numbers_present_flag
        b.flag(false);  // use_128x128_superblock
        b.flag(false);  // enable_filter_intra
        b.flag(false);  // enable_intra_edge_filter
        b.flag(false);  // enable_interintra_compound
        b.flag(false);  // enable_masked_compound
        b.flag(false);  // enable_warped_motion
        b.flag(false);  // enable_dual_filter
        b.flag(true);   // enable_order_hint
        b.flag(false);  // enable_jnt_comp
        b.flag(false);  // enable_ref_frame_mvs
        b.flag(false);  // seq_choose_screen_content_tools
        b.flag(false);  // seq_force_screen_content_tools = 0 (so no integer-mv syntax)
        b.u(3, kOrderHintBits - 1);
        b.flag(false);  // enable_superres
        b.flag(true);   // enable_cdef
        b.flag(false);  // enable_restoration
        // color_config
        b.flag(false);  // high_bitdepth
        b.flag(false);  // mono_chrome
        b.flag(true);   // color_description_present_flag
        b.u(8, 1);      // color_primaries: BT.709
        b.u(8, 1);      // transfer_characteristics: BT.709
        b.u(8, 1);      // matrix_coefficients: BT.709
        b.flag(false);  // color_range: studio (limited)
        b.u(2, 0);      // chroma_sample_position: unknown (AV1 cannot say centre)
        b.flag(false);  // separate_uv_delta_q
        b.flag(false);  // film_grain_params_present
        b.trailing_bits();
    }

    FrameHeader write_frame_header(const PictureParams& pic) const {
        FrameHeader fh;
        BitWriter& b = fh.bits;
        const bool key = pic.keyframe;
        b.flag(false);       // show_existing_frame
        b.u(2, key ? 0 : 1); // frame_type: KEY_FRAME / INTER_FRAME
        b.flag(true);        // show_frame
        if (!key) b.flag(false);  // error_resilient_mode (a shown key frame implies 1)
        b.flag(false);       // disable_cdf_update
        b.flag(false);       // frame_size_override_flag
        b.u(kOrderHintBits, pic.index_in_gop % (1u << kOrderHintBits));  // order_hint
        if (!key) {
            b.u(3, 0);       // primary_ref_frame: slot 0's contexts
            b.u(8, 0x01);    // refresh_frame_flags: slot 0
            b.flag(false);   // frame_refs_short_signaling
            for (int i = 0; i < 7; ++i) b.u(3, 0);  // ref_frame_idx[i]: slot 0
        }
        b.flag(true);        // render_and_frame_size_different
        b.u(16, sp_.width - 1);
        b.u(16, sp_.height - 1);
        if (!key) {
            b.flag(false);   // allow_high_precision_mv
            b.flag(true);    // is_filter_switchable
            b.flag(false);   // is_motion_mode_switchable
        }
        b.flag(false);       // disable_frame_end_update_cdf
        // tile_info
        b.flag(true);        // uniform_tile_spacing_flag
        if (cols_log2_ < max_log2_cols_) b.flag(false);  // increment_tile_cols_log2
        if (rows_log2_ < max_log2_rows_) b.flag(false);  // increment_tile_rows_log2
        if (cols_log2_ || rows_log2_) {
            b.u(cols_log2_ + rows_log2_, 0);  // context_update_tile_id
            b.u(2, tile_size_bytes_minus1_);
        }
        // quantization_params
        fh.qindex = uint32_t(b.bit_count());
        b.u(8, kBaseQindex);
        b.flag(false);  // DeltaQYDc delta_coded
        b.flag(false);  // DeltaQUDc delta_coded
        b.flag(false);  // DeltaQUAc delta_coded
        b.flag(false);  // using_qmatrix
        fh.segmentation = uint32_t(b.bit_count());
        b.flag(false);  // segmentation_enabled
        b.flag(false);  // delta_q_present (base_q_idx > 0)
        // loop_filter_params
        fh.loop_filter = uint32_t(b.bit_count());
        b.u(6, kLoopFilterY);   // loop_filter_level[0]
        b.u(6, kLoopFilterY);   // loop_filter_level[1]
        b.u(6, kLoopFilterUV);  // loop_filter_level[2]
        b.u(6, kLoopFilterUV);  // loop_filter_level[3]
        b.u(3, 0);              // loop_filter_sharpness
        b.flag(false);          // loop_filter_delta_enabled
        // cdef_params
        fh.cdef = uint32_t(b.bit_count());
        b.u(2, 2);  // cdef_damping_minus_3
        b.u(2, 0);  // cdef_bits
        b.u(4, 4);  // cdef_y_pri_strength[0]
        b.u(2, 1);  // cdef_y_sec_strength[0]
        b.u(4, 2);  // cdef_uv_pri_strength[0]
        b.u(2, 0);  // cdef_uv_sec_strength[0]
        fh.cdef_bits = uint32_t(b.bit_count()) - fh.cdef;
        b.flag(true);  // tx_mode_select
        if (!key) b.flag(false);  // reference_select
        b.flag(false);  // reduced_tx_set
        if (!key) {
            for (int i = 0; i < 7; ++i) b.flag(false);  // is_global
        }
        fh.header_bits = uint32_t(b.bit_count()) + 1;
        b.trailing_bits();
        return fh;
    }

    StreamParams sp_;
    uint8_t level_ = 9;
    uint32_t sb_cols_ = 0, sb_rows_ = 0;
    uint32_t min_log2_cols_ = 0, max_log2_cols_ = 0, max_log2_rows_ = 0;
    uint32_t cols_log2_ = 0, rows_log2_ = 0;
    uint32_t tile_w_sb_ = 0, tile_h_sb_ = 0, tile_cols_ = 1, tile_rows_ = 1;
    uint32_t tile_size_bytes_minus1_ = 3;
};

}  // namespace

std::unique_ptr<CodecImpl> make_av1(const StreamParams& sp) { return std::make_unique<Av1>(sp); }

}  // namespace brovideo::vaapi
