// HEVC Main for the VA-API encoder, in the same shape as H.264 (va_h264.cpp):
// IDR then TRAIL_R pictures, each referencing only the one before it through
// an explicit one-entry short-term RPS in its slice header; VPS/SPS/PPS in
// band on every IDR; Annex B.
//
// Headers are packed (VPS + SPS as the sequence header, PPS, the slice
// header) when the driver takes all three, for the same reasons as H.264:
// the colour description, chroma siting and a zero reorder window, and
// because radeonsi takes the conformance window (the crop) only from a
// packed SPS. Coding tools follow the driver's VAConfigAttribEncHEVCFeatures
// and block sizes.
#include "vaapi/va_codec.h"

#include <va/va_enc_hevc.h>

namespace brovideo::vaapi {
namespace {

constexpr uint32_t kLog2MaxPocLsb = 8;  // POC = frame index; its lsb wraps every 256 frames
constexpr int kInitQp = 26;
constexpr uint8_t kNalIdrWRadl = 19;
constexpr uint8_t kNalTrailR = 1;
constexpr uint8_t kNalVps = 32;
constexpr uint8_t kNalSps = 33;
constexpr uint8_t kNalPps = 34;

struct LevelLimit {
    uint8_t idc;        // 30 x level
    uint64_t max_ps;    // luma samples per picture
    uint64_t max_sr;    // luma samples per second
    uint32_t max_kbps;  // Main tier
};
constexpr LevelLimit kLevels[] = {
    {90, 552960, 16588800, 6000},        {93, 983040, 33177600, 10000},       {120, 2228224, 66846720, 12000},
    {123, 2228224, 133693440, 20000},    {150, 8912896, 267386880, 25000},    {153, 8912896, 534773760, 40000},
    {156, 8912896, 1069547520, 60000},   {180, 35651584, 1069547520, 60000},  {183, 35651584, 2139095040, 120000},
    {186, 35651584, 4278190080, 240000},
};

uint8_t pick_level(uint64_t samples, uint32_t fps, uint32_t kbps) {
    for (const LevelLimit& l : kLevels) {
        if (samples <= l.max_ps && samples * fps <= l.max_sr && kbps <= l.max_kbps) return l.idc;
    }
    return 186;
}

VAPictureHEVC invalid_picture() {
    VAPictureHEVC p{};
    p.picture_id = VA_INVALID_SURFACE;
    p.flags = VA_PICTURE_HEVC_INVALID;
    return p;
}

// A 2-bit VAConfigAttribEncHEVCFeatures field: supported or required.
bool feature(uint32_t features, int shift) { return features != VA_ATTRIB_NOT_SUPPORTED && ((features >> shift) & 3) != 0; }

class Hevc final : public CodecImpl {
public:
    Hevc(const StreamParams& sp, uint32_t features, uint32_t block_sizes) : sp_(sp) {
        // Features (va.h, VAConfigAttribValEncHEVCFeatures): 2 bits each.
        amp_ = feature(features, 4);
        sao_ = feature(features, 6);
        strong_intra_smoothing_ = feature(features, 12);
        if (block_sizes != VA_ATTRIB_NOT_SUPPORTED && block_sizes != 0) {
            log2_max_ctb_ = 3 + (block_sizes & 3);
            log2_min_cb_ = 3 + ((block_sizes >> 4) & 3);
            log2_max_tb_ = 2 + ((block_sizes >> 6) & 3);
            log2_min_tb_ = 2 + ((block_sizes >> 8) & 3);
            depth_inter_ = (block_sizes >> 10) & 3;
            depth_intra_ = (block_sizes >> 14) & 3;
        }
        const uint32_t ctb = 1u << log2_max_ctb_;
        ctus_ = ((sp.coded_width + ctb - 1) / ctb) * ((sp.coded_height + ctb - 1) / ctb);
        level_ = pick_level(uint64_t(sp.coded_width) * sp.coded_height, sp.fps, sp.bitrate_bps / 1000);
        const uint32_t want = VA_ENC_PACKED_HEADER_SEQUENCE | VA_ENC_PACKED_HEADER_PICTURE | VA_ENC_PACKED_HEADER_SLICE;
        packed_ = (sp.caps.packed_headers & want) == want ? want : VA_ENC_PACKED_HEADER_NONE;
        debug_log("hevc: level %u, CTB %u, CB >= %u, TB %u..%u, amp %d sao %d sis %d, packed headers 0x%x",
                  unsigned(level_), ctb, 1u << log2_min_cb_, 1u << log2_min_tb_, 1u << log2_max_tb_, amp_, sao_,
                  strong_intra_smoothing_, packed_);
    }

    uint32_t packed_headers() const override { return packed_; }

    bool add_sequence(BufferList& bufs, const PictureParams&, std::string* err) override {
        VAEncSequenceParameterBufferHEVC s{};
        s.general_profile_idc = 1;  // Main
        s.general_level_idc = level_;
        s.general_tier_flag = 0;
        s.intra_period = 0;
        s.intra_idr_period = 0;
        s.ip_period = 1;
        s.bits_per_second = sp_.bitrate_bps;
        s.pic_width_in_luma_samples = uint16_t(sp_.coded_width);
        s.pic_height_in_luma_samples = uint16_t(sp_.coded_height);
        s.seq_fields.bits.chroma_format_idc = 1;
        s.seq_fields.bits.amp_enabled_flag = amp_;
        s.seq_fields.bits.sample_adaptive_offset_enabled_flag = sao_;
        s.seq_fields.bits.strong_intra_smoothing_enabled_flag = strong_intra_smoothing_;
        s.seq_fields.bits.sps_temporal_mvp_enabled_flag = 0;
        s.seq_fields.bits.low_delay_seq = 1;
        s.log2_min_luma_coding_block_size_minus3 = uint8_t(log2_min_cb_ - 3);
        s.log2_diff_max_min_luma_coding_block_size = uint8_t(log2_max_ctb_ - log2_min_cb_);
        s.log2_min_transform_block_size_minus2 = uint8_t(log2_min_tb_ - 2);
        s.log2_diff_max_min_transform_block_size = uint8_t(log2_max_tb_ - log2_min_tb_);
        s.max_transform_hierarchy_depth_inter = uint8_t(depth_inter_);
        s.max_transform_hierarchy_depth_intra = uint8_t(depth_intra_);
        s.vui_parameters_present_flag = 1;
        s.vui_fields.bits.vui_timing_info_present_flag = 1;
        s.vui_fields.bits.bitstream_restriction_flag = 1;
        s.vui_fields.bits.motion_vectors_over_pic_boundaries_flag = 1;
        s.vui_fields.bits.restricted_ref_pic_lists_flag = 1;
        s.vui_fields.bits.log2_max_mv_length_horizontal = 15;
        s.vui_fields.bits.log2_max_mv_length_vertical = 15;
        s.vui_num_units_in_tick = 1;
        s.vui_time_scale = sp_.fps;
        return bufs.add(VAEncSequenceParameterBufferType, s, err);
    }

    bool add_picture(BufferList& bufs, const PictureParams& pic, std::string* err) override {
        const int32_t poc = int32_t(pic.index_in_gop % (1u << 30));
        VAEncPictureParameterBufferHEVC p{};
        p.decoded_curr_pic.picture_id = pic.recon;
        p.decoded_curr_pic.pic_order_cnt = poc;
        p.decoded_curr_pic.flags = 0;
        for (auto& r : p.reference_frames) r = invalid_picture();
        VAPictureHEVC ref = invalid_picture();
        if (!pic.keyframe) {
            ref.picture_id = pic.ref;
            ref.pic_order_cnt = poc - 1;
            ref.flags = VA_PICTURE_HEVC_RPS_ST_CURR_BEFORE;
            p.reference_frames[0] = ref;
        }
        p.coded_buf = pic.coded;
        p.collocated_ref_pic_index = 0xff;  // no temporal MV prediction
        p.last_picture = 0;
        p.pic_init_qp = kInitQp;
        p.diff_cu_qp_delta_depth = 0;
        p.num_ref_idx_l0_default_active_minus1 = 0;
        p.num_ref_idx_l1_default_active_minus1 = 0;
        p.slice_pic_parameter_set_id = 0;
        p.nal_unit_type = pic.keyframe ? kNalIdrWRadl : kNalTrailR;
        p.pic_fields.bits.idr_pic_flag = pic.keyframe;
        p.pic_fields.bits.coding_type = pic.keyframe ? 1 : 2;  // I : P
        p.pic_fields.bits.reference_pic_flag = 1;
        p.pic_fields.bits.cu_qp_delta_enabled_flag = 1;
        if (!bufs.add(VAEncPictureParameterBufferType, p, err)) return false;

        if (pic.keyframe && (packed_ & VA_ENC_PACKED_HEADER_SEQUENCE)) {
            BitWriter vps, sps;
            write_vps(vps);
            write_sps(sps);
            auto nal = annexb_nal(nal_header(kNalVps), vps.bytes());
            const auto sps_nal = annexb_nal(nal_header(kNalSps), sps.bytes());
            nal.insert(nal.end(), sps_nal.begin(), sps_nal.end());
            if (!bufs.add_packed(VAEncPackedHeaderSequence, nal, nal.size() * 8, err)) return false;
        }
        if (pic.keyframe && (packed_ & VA_ENC_PACKED_HEADER_PICTURE)) {
            BitWriter pps;
            write_pps(pps);
            const auto nal = annexb_nal(nal_header(kNalPps), pps.bytes());
            if (!bufs.add_packed(VAEncPackedHeaderPicture, nal, nal.size() * 8, err)) return false;
        }
        if (packed_ & VA_ENC_PACKED_HEADER_SLICE) {
            BitWriter sh;
            write_slice_header(sh, pic);
            const auto nal = annexb_nal(nal_header(pic.keyframe ? kNalIdrWRadl : kNalTrailR), sh.bytes());
            if (!bufs.add_packed(VAEncPackedHeaderSlice, nal, nal.size() * 8, err)) return false;
        }

        VAEncSliceParameterBufferHEVC sl{};
        sl.slice_segment_address = 0;
        sl.num_ctu_in_slice = ctus_;
        sl.slice_type = pic.keyframe ? 2 : 1;  // I : P
        sl.slice_pic_parameter_set_id = 0;
        sl.num_ref_idx_l0_active_minus1 = 0;
        sl.num_ref_idx_l1_active_minus1 = 0;
        for (auto& r : sl.ref_pic_list0) r = invalid_picture();
        for (auto& r : sl.ref_pic_list1) r = invalid_picture();
        if (!pic.keyframe) sl.ref_pic_list0[0] = ref;
        sl.max_num_merge_cand = 5;
        sl.slice_qp_delta = 0;
        sl.slice_fields.bits.last_slice_of_pic_flag = 1;
        sl.slice_fields.bits.slice_sao_luma_flag = sao_;
        sl.slice_fields.bits.slice_sao_chroma_flag = sao_;
        sl.slice_fields.bits.collocated_from_l0_flag = 1;
        return bufs.add(VAEncSliceParameterBufferType, sl, err);
    }

private:
    static std::vector<uint8_t> nal_header(uint8_t type) {
        return {uint8_t(type << 1), 1};  // nuh_layer_id 0, nuh_temporal_id_plus1 1
    }

    void write_profile_tier_level(BitWriter& b) const {
        b.u(2, 0);      // general_profile_space
        b.flag(false);  // general_tier_flag: Main
        b.u(5, 1);      // general_profile_idc: Main
        for (int j = 0; j < 32; ++j) b.flag(j == 1 || j == 2);  // compatible with Main and Main 10
        b.flag(true);   // general_progressive_source_flag
        b.flag(false);  // general_interlaced_source_flag
        b.flag(false);  // general_non_packed_constraint_flag
        b.flag(true);   // general_frame_only_constraint_flag
        b.u(32, 0);     // general_reserved_zero_43bits (32 + 11)
        b.u(11, 0);
        b.flag(false);  // general_inbld_flag / reserved
        b.u(8, level_);
    }

    // Sub-layer ordering info: the current picture and one reference, no reordering.
    static void write_ordering(BitWriter& b) {
        b.flag(true);  // *_sub_layer_ordering_info_present_flag
        b.ue(1);       // max_dec_pic_buffering_minus1
        b.ue(0);       // max_num_reorder_pics
        b.ue(0);       // max_latency_increase_plus1
    }

    void write_vps(BitWriter& b) const {
        b.u(4, 0);       // vps_video_parameter_set_id
        b.flag(true);    // vps_base_layer_internal_flag
        b.flag(true);    // vps_base_layer_available_flag
        b.u(6, 0);       // vps_max_layers_minus1
        b.u(3, 0);       // vps_max_sub_layers_minus1
        b.flag(true);    // vps_temporal_id_nesting_flag
        b.u(16, 0xffff); // vps_reserved_0xffff_16bits
        write_profile_tier_level(b);
        write_ordering(b);
        b.u(6, 0);       // vps_max_layer_id
        b.ue(0);         // vps_num_layer_sets_minus1
        b.flag(true);    // vps_timing_info_present_flag
        b.u(32, 1);      // vps_num_units_in_tick
        b.u(32, sp_.fps);  // vps_time_scale
        b.flag(false);   // vps_poc_proportional_to_timing_flag
        b.ue(0);         // vps_num_hrd_parameters
        b.flag(false);   // vps_extension_flag
        b.trailing_bits();
    }

    void write_sps(BitWriter& b) const {
        b.u(4, 0);     // sps_video_parameter_set_id
        b.u(3, 0);     // sps_max_sub_layers_minus1
        b.flag(true);  // sps_temporal_id_nesting_flag
        write_profile_tier_level(b);
        b.ue(0);       // sps_seq_parameter_set_id
        b.ue(1);       // chroma_format_idc 4:2:0
        b.ue(sp_.coded_width);
        b.ue(sp_.coded_height);
        const uint32_t crop_right = (sp_.coded_width - sp_.width) / 2;  // in 2-pixel units (4:2:0)
        const uint32_t crop_bottom = (sp_.coded_height - sp_.height) / 2;
        b.flag(crop_right || crop_bottom);  // conformance_window_flag
        if (crop_right || crop_bottom) {
            b.ue(0);
            b.ue(crop_right);
            b.ue(0);
            b.ue(crop_bottom);
        }
        b.ue(0);  // bit_depth_luma_minus8
        b.ue(0);  // bit_depth_chroma_minus8
        b.ue(kLog2MaxPocLsb - 4);
        write_ordering(b);
        b.ue(log2_min_cb_ - 3);
        b.ue(log2_max_ctb_ - log2_min_cb_);
        b.ue(log2_min_tb_ - 2);
        b.ue(log2_max_tb_ - log2_min_tb_);
        b.ue(depth_inter_);
        b.ue(depth_intra_);
        b.flag(false);  // scaling_list_enabled_flag
        b.flag(amp_);
        b.flag(sao_);
        b.flag(false);  // pcm_enabled_flag
        b.ue(0);        // num_short_term_ref_pic_sets: each slice carries its own
        b.flag(false);  // long_term_ref_pics_present_flag
        b.flag(false);  // sps_temporal_mvp_enabled_flag
        b.flag(strong_intra_smoothing_);
        b.flag(true);   // vui_parameters_present_flag
        b.flag(false);  // aspect_ratio_info_present_flag
        b.flag(false);  // overscan_info_present_flag
        b.flag(true);   // video_signal_type_present_flag
        b.u(3, 5);      // video_format: unspecified
        b.flag(false);  // video_full_range_flag: limited
        b.flag(true);   // colour_description_present_flag
        b.u(8, 1);      // colour_primaries: BT.709
        b.u(8, 1);      // transfer_characteristics: BT.709
        b.u(8, 1);      // matrix_coeffs: BT.709
        b.flag(true);   // chroma_loc_info_present_flag
        b.ue(1);        // chroma_sample_loc_type_top_field: centre (the VPP pass averages)
        b.ue(1);        // chroma_sample_loc_type_bottom_field
        b.flag(false);  // neutral_chroma_indication_flag
        b.flag(false);  // field_seq_flag
        b.flag(false);  // frame_field_info_present_flag
        b.flag(false);  // default_display_window_flag
        b.flag(true);   // vui_timing_info_present_flag
        b.u(32, 1);     // vui_num_units_in_tick
        b.u(32, sp_.fps);  // vui_time_scale
        b.flag(false);  // vui_poc_proportional_to_timing_flag
        b.flag(false);  // vui_hrd_parameters_present_flag
        b.flag(true);   // bitstream_restriction_flag
        b.flag(false);  // tiles_fixed_structure_flag
        b.flag(true);   // motion_vectors_over_pic_boundaries_flag
        b.flag(true);   // restricted_ref_pic_lists_flag
        b.ue(0);        // min_spatial_segmentation_idc
        b.ue(0);        // max_bytes_per_pic_denom
        b.ue(0);        // max_bits_per_min_cu_denom
        b.ue(15);       // log2_max_mv_length_horizontal
        b.ue(15);       // log2_max_mv_length_vertical
        b.flag(false);  // sps_extension_present_flag
        b.trailing_bits();
    }

    void write_pps(BitWriter& b) const {
        b.ue(0);        // pps_pic_parameter_set_id
        b.ue(0);        // pps_seq_parameter_set_id
        b.flag(false);  // dependent_slice_segments_enabled_flag
        b.flag(false);  // output_flag_present_flag
        b.u(3, 0);      // num_extra_slice_header_bits
        b.flag(false);  // sign_data_hiding_enabled_flag
        b.flag(false);  // cabac_init_present_flag
        b.ue(0);        // num_ref_idx_l0_default_active_minus1
        b.ue(0);        // num_ref_idx_l1_default_active_minus1
        b.se(kInitQp - 26);
        b.flag(false);  // constrained_intra_pred_flag
        b.flag(false);  // transform_skip_enabled_flag
        b.flag(true);   // cu_qp_delta_enabled_flag (rate control)
        b.ue(0);        // diff_cu_qp_delta_depth
        b.se(0);        // pps_cb_qp_offset
        b.se(0);        // pps_cr_qp_offset
        b.flag(false);  // pps_slice_chroma_qp_offsets_present_flag
        b.flag(false);  // weighted_pred_flag
        b.flag(false);  // weighted_bipred_flag
        b.flag(false);  // transquant_bypass_enabled_flag
        b.flag(false);  // tiles_enabled_flag
        b.flag(false);  // entropy_coding_sync_enabled_flag
        b.flag(false);  // pps_loop_filter_across_slices_enabled_flag
        b.flag(false);  // deblocking_filter_control_present_flag
        b.flag(false);  // pps_scaling_list_data_present_flag
        b.flag(false);  // lists_modification_present_flag
        b.ue(0);        // log2_parallel_merge_level_minus2
        b.flag(false);  // slice_segment_header_extension_present_flag
        b.flag(false);  // pps_extension_present_flag
        b.trailing_bits();
    }

    void write_slice_header(BitWriter& b, const PictureParams& pic) const {
        b.flag(true);  // first_slice_segment_in_pic_flag
        if (pic.keyframe) b.flag(false);  // no_output_of_prior_pics_flag (IRAP)
        b.ue(0);       // slice_pic_parameter_set_id
        b.ue(pic.keyframe ? 2 : 1);  // slice_type: I / P
        if (!pic.keyframe) {
            b.u(kLog2MaxPocLsb, pic.index_in_gop % (1u << kLog2MaxPocLsb));  // slice_pic_order_cnt_lsb
            b.flag(false);  // short_term_ref_pic_set_sps_flag: the RPS follows
            // st_ref_pic_set(0): one picture before this one, used.
            b.ue(1);        // num_negative_pics
            b.ue(0);        // num_positive_pics
            b.ue(0);        // delta_poc_s0_minus1: POC - 1
            b.flag(true);   // used_by_curr_pic_s0_flag
        }
        if (sao_) {
            b.flag(true);  // slice_sao_luma_flag
            b.flag(true);  // slice_sao_chroma_flag
        }
        if (!pic.keyframe) {
            b.flag(false);  // num_ref_idx_active_override_flag
            b.ue(0);        // five_minus_max_num_merge_cand
        }
        b.se(0);  // slice_qp_delta
        // byte_alignment()
        b.flag(true);
        b.align_zero();
    }

    StreamParams sp_;
    bool amp_ = false;
    bool sao_ = false;
    bool strong_intra_smoothing_ = false;
    uint32_t log2_max_ctb_ = 5;  // defaults when the driver reports no block sizes: CTB 32
    uint32_t log2_min_cb_ = 3;
    uint32_t log2_max_tb_ = 5;
    uint32_t log2_min_tb_ = 2;
    uint32_t depth_inter_ = 2;
    uint32_t depth_intra_ = 2;
    uint32_t ctus_ = 0;
    uint8_t level_ = 123;
    uint32_t packed_ = 0;
};

}  // namespace

std::unique_ptr<CodecImpl> make_hevc(const StreamParams& sp) {
    return std::make_unique<Hevc>(sp, sp.caps.hevc_features, sp.caps.hevc_block_sizes);
}

}  // namespace brovideo::vaapi
