// HEVC (H.265) Annex B bitstream parser and VA-API VLD buffer builder.
#include "vaapi/va_bitreader.h"
#include "vaapi/va_decoder.h"

#include <va/va_dec_hevc.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <vector>

namespace brovideo::vaapi {
namespace {

uint32_t ceil_log2(uint32_t x) {
    if (x <= 1) return 0;
    uint32_t b = 0;
    --x;
    while (x > 0) {
        x >>= 1;
        ++b;
    }
    return b;
}

// HEVC Table 7-5 & 7-6 default 8x8/16x16/32x32 scaling lists (in raster order via upright diagonal scan, or filled).
// Default 4x4 scaling list is all 16.
constexpr uint8_t kDefaultHevc8x8Intra[64] = {
    16, 16, 16, 16, 17, 18, 21, 24, 16, 16, 16, 16, 17, 19, 22, 25, 16, 16, 17, 18, 20, 22,
    25, 29, 16, 16, 18, 21, 24, 27, 31, 36, 17, 17, 20, 24, 30, 35, 41, 47, 18, 19, 22, 27,
    35, 44, 54, 65, 21, 22, 25, 31, 41, 54, 70, 88, 24, 25, 29, 36, 47, 65, 88, 115};

constexpr uint8_t kDefaultHevc8x8Inter[64] = {
    16, 16, 16, 16, 17, 18, 20, 24, 16, 16, 16, 17, 18, 20, 24, 25, 16, 16, 17, 18, 20, 24,
    25, 28, 16, 17, 18, 20, 24, 25, 28, 33, 17, 18, 20, 24, 25, 28, 33, 41, 18, 20, 24, 25,
    28, 33, 41, 54, 20, 24, 25, 28, 33, 41, 54, 71, 24, 25, 28, 33, 41, 54, 71, 91};

void init_default_hevc_iq(VAIQMatrixBufferHEVC& iq) {
    std::memset(iq.ScalingList4x4, 16, sizeof(iq.ScalingList4x4));
    for (int i = 0; i < 6; ++i) {
        const uint8_t* src = (i < 3) ? kDefaultHevc8x8Intra : kDefaultHevc8x8Inter;
        std::memcpy(iq.ScalingList8x8[i], src, 64);
        std::memcpy(iq.ScalingList16x16[i], src, 64);
        iq.ScalingListDC16x16[i] = 16;
    }
    std::memcpy(iq.ScalingList32x32[0], kDefaultHevc8x8Intra, 64);
    std::memcpy(iq.ScalingList32x32[1], kDefaultHevc8x8Inter, 64);
    iq.ScalingListDC32x32[0] = 16;
    iq.ScalingListDC32x32[1] = 16;
}

void skip_scaling_list_data(BitReader& r) {
    for (int size_id = 0; size_id < 4; ++size_id) {
        const int num_matrices = (size_id == 3) ? 2 : 6;
        for (int matrix_id = 0; matrix_id < num_matrices; ++matrix_id) {
            if (!r.flag()) {
                r.ue();  // scaling_list_pred_matrix_id_delta
            } else {
                const int coef_num = std::min(64, 1 << (4 + (size_id << 1)));
                if (size_id > 1) r.se();  // scaling_list_dc_coef_minus8
                for (int i = 0; i < coef_num; ++i) r.se();
            }
        }
    }
}

struct StRps {
    uint32_t num_negative_pics = 0;
    uint32_t num_positive_pics = 0;
    int32_t delta_poc_s0[16]{};
    bool used_s0[16]{};
    int32_t delta_poc_s1[16]{};
    bool used_s1[16]{};
};

bool parse_st_rps(BitReader& r, uint32_t idx, uint32_t num_sps_rps, const StRps* sps_rps, StRps& out) {
    out = StRps{};
    bool inter_pred = false;
    if (idx != 0) inter_pred = r.flag();
    if (inter_pred) {
        uint32_t delta_idx_minus1 = 0;
        if (idx == num_sps_rps) delta_idx_minus1 = r.ue();
        if (delta_idx_minus1 + 1 > idx) return false;
        const StRps& ref = sps_rps[idx - (delta_idx_minus1 + 1)];
        const uint32_t sign = r.flag() ? 1 : 0;
        const uint32_t abs_minus1 = r.ue();
        const int32_t delta_rps = (1 - 2 * int32_t(sign)) * int32_t(abs_minus1 + 1);
        const uint32_t num_delta = ref.num_negative_pics + ref.num_positive_pics;
        if (num_delta > 16) return false;
        bool used[17]{};
        bool use_delta[17]{};
        for (uint32_t j = 0; j <= num_delta; ++j) {
            used[j] = r.flag();
            use_delta[j] = used[j] ? true : r.flag();
        }
        uint32_t i = 0;
        for (int j = int(ref.num_positive_pics) - 1; j >= 0; --j) {
            const int32_t d_poc = ref.delta_poc_s1[j] + delta_rps;
            if (d_poc < 0 && use_delta[ref.num_negative_pics + uint32_t(j)]) {
                out.delta_poc_s0[i] = d_poc;
                out.used_s0[i] = used[ref.num_negative_pics + uint32_t(j)];
                ++i;
            }
        }
        if (delta_rps < 0 && use_delta[num_delta]) {
            out.delta_poc_s0[i] = delta_rps;
            out.used_s0[i] = used[num_delta];
            ++i;
        }
        for (uint32_t j = 0; j < ref.num_negative_pics; ++j) {
            const int32_t d_poc = ref.delta_poc_s0[j] + delta_rps;
            if (d_poc < 0 && use_delta[j]) {
                out.delta_poc_s0[i] = d_poc;
                out.used_s0[i] = used[j];
                ++i;
            }
        }
        out.num_negative_pics = i;

        i = 0;
        for (int j = int(ref.num_negative_pics) - 1; j >= 0; --j) {
            const int32_t d_poc = ref.delta_poc_s0[j] + delta_rps;
            if (d_poc > 0 && use_delta[j]) {
                out.delta_poc_s1[i] = d_poc;
                out.used_s1[i] = used[j];
                ++i;
            }
        }
        if (delta_rps > 0 && use_delta[num_delta]) {
            out.delta_poc_s1[i] = delta_rps;
            out.used_s1[i] = used[num_delta];
            ++i;
        }
        for (uint32_t j = 0; j < ref.num_positive_pics; ++j) {
            const int32_t d_poc = ref.delta_poc_s1[j] + delta_rps;
            if (d_poc > 0 && use_delta[ref.num_negative_pics + j]) {
                out.delta_poc_s1[i] = d_poc;
                out.used_s1[i] = used[ref.num_negative_pics + j];
                ++i;
            }
        }
        out.num_positive_pics = i;
        return r.ok() && (out.num_negative_pics + out.num_positive_pics <= 16);
    }

    out.num_negative_pics = r.ue();
    out.num_positive_pics = r.ue();
    if (out.num_negative_pics > 16 || out.num_positive_pics > 16 ||
        out.num_negative_pics + out.num_positive_pics > 16) {
        return false;
    }
    int32_t prev = 0;
    for (uint32_t i = 0; i < out.num_negative_pics; ++i) {
        const uint32_t delta_minus1 = r.ue();
        prev -= int32_t(delta_minus1 + 1);
        out.delta_poc_s0[i] = prev;
        out.used_s0[i] = r.flag();
    }
    prev = 0;
    for (uint32_t i = 0; i < out.num_positive_pics; ++i) {
        const uint32_t delta_minus1 = r.ue();
        prev += int32_t(delta_minus1 + 1);
        out.delta_poc_s1[i] = prev;
        out.used_s1[i] = r.flag();
    }
    return r.ok();
}

void skip_profile_tier_level(BitReader& r, bool profile_present, uint32_t max_sub_layers_minus1) {
    if (profile_present) {
        r.u(32);  // space(2)+tier(1)+profile(5)+compat(24)
        r.u(32);  // compat(8)+constraint(24)
        r.u(24);  // constraint(24)
    }
    r.u(8);  // general_level_idc
    bool sub_profile[8]{}, sub_level[8]{};
    for (uint32_t i = 0; i < max_sub_layers_minus1; ++i) {
        sub_profile[i] = r.flag();
        sub_level[i] = r.flag();
    }
    if (max_sub_layers_minus1 > 0) {
        for (uint32_t i = max_sub_layers_minus1; i < 8; ++i) r.u(2);
    }
    for (uint32_t i = 0; i < max_sub_layers_minus1; ++i) {
        if (sub_profile[i]) { r.u(32); r.u(32); r.u(24); }
        if (sub_level[i]) r.u(8);
    }
}

struct HevcSps {
    bool valid = false, separate_colour_plane_flag = false, scaling_list_enabled_flag = false, amp_enabled_flag = false;
    bool sample_adaptive_offset_enabled_flag = false, pcm_enabled_flag = false, pcm_loop_filter_disabled_flag = false;
    bool long_term_ref_pics_present_flag = false, sps_temporal_mvp_enabled_flag = false, strong_intra_smoothing_enabled_flag = false;
    uint32_t vps_id = 0, max_sub_layers_minus1 = 0, sps_id = 0, chroma_format_idc = 1;
    uint32_t pic_width_in_luma_samples = 0, pic_height_in_luma_samples = 0;
    uint32_t vis_x = 0, vis_y = 0, vis_w = 0, vis_h = 0;
    uint32_t bit_depth_luma_minus8 = 0, bit_depth_chroma_minus8 = 0, log2_max_pic_order_cnt_lsb_minus4 = 0;
    uint32_t sps_max_dec_pic_buffering_minus1 = 1, sps_max_num_reorder_pics = 0;
    uint32_t log2_min_luma_coding_block_size_minus3 = 0, log2_diff_max_min_luma_coding_block_size = 0;
    uint32_t log2_min_luma_transform_block_size_minus2 = 0, log2_diff_max_min_luma_transform_block_size = 0;
    uint32_t max_transform_hierarchy_depth_inter = 0, max_transform_hierarchy_depth_intra = 0;
    uint32_t pcm_sample_bit_depth_luma_minus1 = 0, pcm_sample_bit_depth_chroma_minus1 = 0;
    uint32_t log2_min_pcm_luma_coding_block_size_minus3 = 0, log2_diff_max_min_pcm_luma_coding_block_size = 0;
    uint32_t num_short_term_ref_pic_sets = 0, num_long_term_ref_pics_sps = 0;
    std::array<StRps, 65> st_rps{};
    uint32_t lt_ref_pic_poc_lsb_sps[32]{};
    bool used_by_curr_pic_lt_sps_flag[32]{};
};

struct HevcPps {
    bool valid = false, dependent_slice_segments_enabled_flag = false, output_flag_present_flag = false;
    bool sign_data_hiding_enabled_flag = false, cabac_init_present_flag = false, constrained_intra_pred_flag = false;
    bool transform_skip_enabled_flag = false, cu_qp_delta_enabled_flag = false, pps_slice_chroma_qp_offsets_present_flag = false;
    bool weighted_pred_flag = false, weighted_bipred_flag = false, transquant_bypass_enabled_flag = false;
    bool tiles_enabled_flag = false, entropy_coding_sync_enabled_flag = false, uniform_spacing_flag = true;
    bool loop_filter_across_tiles_enabled_flag = true, pps_loop_filter_across_slices_enabled_flag = false;
    bool deblocking_filter_control_present_flag = false, deblocking_filter_override_enabled_flag = false;
    bool pps_deblocking_filter_disabled_flag = false, lists_modification_present_flag = false;
    bool slice_segment_header_extension_present_flag = false;
    uint32_t pps_id = 0, sps_id = 0, num_extra_slice_header_bits = 0, diff_cu_qp_delta_depth = 0;
    uint32_t num_ref_idx_l0_default_active_minus1 = 0, num_ref_idx_l1_default_active_minus1 = 0;
    uint32_t num_tile_columns_minus1 = 0, num_tile_rows_minus1 = 0, log2_parallel_merge_level_minus2 = 0;
    int32_t init_qp_minus26 = 0, pps_cb_qp_offset = 0, pps_cr_qp_offset = 0, pps_beta_offset_div2 = 0, pps_tc_offset_div2 = 0;
    uint16_t column_width_minus1[19]{};
    uint16_t row_height_minus1[21]{};
};

struct HevcDpbEntry {
    int surface_idx = -1;
    VASurfaceID surface = VA_INVALID_SURFACE;
    int32_t poc = 0;
    bool long_term = false, referenced = true;
};

VAPictureHEVC invalid_hevc_pic() {
    VAPictureHEVC p{};
    p.picture_id = VA_INVALID_SURFACE;
    p.flags = VA_PICTURE_HEVC_INVALID;
    return p;
}

class HevcDecoder final : public DecodeCodec {
public:
    ~HevcDecoder() override { clear_dpb(); }
    void reset() override { clear_dpb(); prev_tid0_poc_ = 0; }

    bool decode_packet(std::span<const uint8_t> packet, DecodeSession& session, DecodedUnit& out,
                       std::string* err) override {
        session_ = &session;
        out = DecodedUnit{};
        const auto nals = split_annexb_nals(packet);
        if (nals.empty()) {
            if (err) *err = "HEVC packet has no Annex B start codes";
            return false;
        }
        std::vector<std::span<const uint8_t>> slice_nals;
        for (const auto& nal : nals) {
            if (nal.size() < 2) continue;
            if (nal[0] & 0x80) {
                if (err) *err = "HEVC NAL forbidden_zero_bit is set";
                return false;
            }
            const uint8_t type = (nal[0] >> 1) & 0x3fu;
            if (type == 33) {
                if (!parse_sps(nal.subspan(2), err)) return false;
            } else if (type == 34) {
                if (!parse_pps(nal.subspan(2), err)) return false;
            } else if (type <= 21 && (type <= 9 || type >= 16)) {
                slice_nals.push_back(nal);
            }
        }
        return slice_nals.empty() ? true : decode_slices(slice_nals, out, err);
    }

private:
    void clear_dpb() {
        if (session_) {
            for (const auto& e : dpb_) session_->unref_surface(e.surface_idx);
        }
        dpb_.clear();
    }

    bool parse_sps(std::span<const uint8_t> body, std::string* err) {
        BitReader r(body, true);
        HevcSps s{};
        s.vps_id = r.u(4);
        s.max_sub_layers_minus1 = r.u(3);
        if (s.max_sub_layers_minus1 > 6) return false;
        r.flag();  // sps_temporal_id_nesting_flag
        skip_profile_tier_level(r, true, s.max_sub_layers_minus1);
        s.sps_id = r.ue();
        if (s.sps_id >= 16) {
            if (err) *err = "HEVC SPS id out of range";
            return false;
        }
        s.chroma_format_idc = r.ue();
        if (s.chroma_format_idc == 3) s.separate_colour_plane_flag = r.flag();
        s.pic_width_in_luma_samples = r.ue();
        s.pic_height_in_luma_samples = r.ue();
        if (s.pic_width_in_luma_samples == 0 || s.pic_height_in_luma_samples == 0 ||
            s.pic_width_in_luma_samples > 16384 || s.pic_height_in_luma_samples > 16384) {
            if (err) *err = "invalid HEVC picture dimensions";
            return false;
        }
        uint32_t conf_l = 0, conf_r = 0, conf_t = 0, conf_b = 0;
        if (r.flag()) { conf_l = r.ue(); conf_r = r.ue(); conf_t = r.ue(); conf_b = r.ue(); }
        const uint32_t sub_w = (s.chroma_format_idc == 1 || s.chroma_format_idc == 2) ? 2u : 1u;
        const uint32_t sub_h = (s.chroma_format_idc == 1) ? 2u : 1u;
        const uint32_t cx = (conf_l + conf_r) * sub_w, cy = (conf_t + conf_b) * sub_h;
        if (cx >= s.pic_width_in_luma_samples || cy >= s.pic_height_in_luma_samples) {
            if (err) *err = "invalid HEVC conformance window";
            return false;
        }
        s.vis_x = conf_l * sub_w;
        s.vis_y = conf_t * sub_h;
        s.vis_w = s.pic_width_in_luma_samples - cx;
        s.vis_h = s.pic_height_in_luma_samples - cy;

        s.bit_depth_luma_minus8 = r.ue();
        s.bit_depth_chroma_minus8 = r.ue();
        s.log2_max_pic_order_cnt_lsb_minus4 = r.ue();
        if (s.log2_max_pic_order_cnt_lsb_minus4 > 12) return false;
        const bool sub_layer_order = r.flag();
        for (uint32_t i = sub_layer_order ? 0 : s.max_sub_layers_minus1; i <= s.max_sub_layers_minus1; ++i) {
            s.sps_max_dec_pic_buffering_minus1 = r.ue();
            s.sps_max_num_reorder_pics = r.ue();
            r.ue();  // sps_max_latency_increase_plus1
        }
        s.log2_min_luma_coding_block_size_minus3 = r.ue();
        s.log2_diff_max_min_luma_coding_block_size = r.ue();
        s.log2_min_luma_transform_block_size_minus2 = r.ue();
        s.log2_diff_max_min_luma_transform_block_size = r.ue();
        s.max_transform_hierarchy_depth_inter = r.ue();
        s.max_transform_hierarchy_depth_intra = r.ue();
        s.scaling_list_enabled_flag = r.flag();
        if (s.scaling_list_enabled_flag && r.flag()) skip_scaling_list_data(r);
        s.amp_enabled_flag = r.flag();
        s.sample_adaptive_offset_enabled_flag = r.flag();
        s.pcm_enabled_flag = r.flag();
        if (s.pcm_enabled_flag) {
            s.pcm_sample_bit_depth_luma_minus1 = r.u(4);
            s.pcm_sample_bit_depth_chroma_minus1 = r.u(4);
            s.log2_min_pcm_luma_coding_block_size_minus3 = r.ue();
            s.log2_diff_max_min_pcm_luma_coding_block_size = r.ue();
            s.pcm_loop_filter_disabled_flag = r.flag();
        }
        s.num_short_term_ref_pic_sets = r.ue();
        if (s.num_short_term_ref_pic_sets > 64) return false;
        for (uint32_t i = 0; i < s.num_short_term_ref_pic_sets; ++i) {
            if (!parse_st_rps(r, i, s.num_short_term_ref_pic_sets, s.st_rps.data(), s.st_rps[i])) {
                if (err) *err = "invalid HEVC SPS short-term RPS";
                return false;
            }
        }
        s.long_term_ref_pics_present_flag = r.flag();
        if (s.long_term_ref_pics_present_flag) {
            s.num_long_term_ref_pics_sps = r.ue();
            if (s.num_long_term_ref_pics_sps > 32) return false;
            for (uint32_t i = 0; i < s.num_long_term_ref_pics_sps; ++i) {
                s.lt_ref_pic_poc_lsb_sps[i] = r.u(s.log2_max_pic_order_cnt_lsb_minus4 + 4);
                s.used_by_curr_pic_lt_sps_flag[i] = r.flag();
            }
        }
        s.sps_temporal_mvp_enabled_flag = r.flag();
        s.strong_intra_smoothing_enabled_flag = r.flag();
        if (!r.ok()) {
            if (err) *err = "truncated HEVC SPS";
            return false;
        }
        s.valid = true;
        sps_[s.sps_id] = s;
        return true;
    }

    bool parse_pps(std::span<const uint8_t> body, std::string* err) {
        BitReader r(body, true);
        HevcPps p{};
        p.pps_id = r.ue();
        p.sps_id = r.ue();
        if (p.pps_id >= 64 || p.sps_id >= 16 || !sps_[p.sps_id].valid) {
            if (err) *err = "HEVC PPS references invalid SPS";
            return false;
        }
        const HevcSps& sps = sps_[p.sps_id];
        p.dependent_slice_segments_enabled_flag = r.flag();
        p.output_flag_present_flag = r.flag();
        p.num_extra_slice_header_bits = r.u(3);
        p.sign_data_hiding_enabled_flag = r.flag();
        p.cabac_init_present_flag = r.flag();
        p.num_ref_idx_l0_default_active_minus1 = r.ue();
        p.num_ref_idx_l1_default_active_minus1 = r.ue();
        if (p.num_ref_idx_l0_default_active_minus1 > 14 || p.num_ref_idx_l1_default_active_minus1 > 14) return false;
        p.init_qp_minus26 = r.se();
        p.constrained_intra_pred_flag = r.flag();
        p.transform_skip_enabled_flag = r.flag();
        p.cu_qp_delta_enabled_flag = r.flag();
        if (p.cu_qp_delta_enabled_flag) p.diff_cu_qp_delta_depth = r.ue();
        p.pps_cb_qp_offset = r.se();
        p.pps_cr_qp_offset = r.se();
        p.pps_slice_chroma_qp_offsets_present_flag = r.flag();
        p.weighted_pred_flag = r.flag();
        p.weighted_bipred_flag = r.flag();
        p.transquant_bypass_enabled_flag = r.flag();
        p.tiles_enabled_flag = r.flag();
        p.entropy_coding_sync_enabled_flag = r.flag();
        if (p.tiles_enabled_flag) {
            p.num_tile_columns_minus1 = r.ue();
            p.num_tile_rows_minus1 = r.ue();
            if (p.num_tile_columns_minus1 >= 19 || p.num_tile_rows_minus1 >= 21) return false;
            p.uniform_spacing_flag = r.flag();
            const uint32_t ctb_log2 =
                sps.log2_min_luma_coding_block_size_minus3 + 3 + sps.log2_diff_max_min_luma_coding_block_size;
            const uint32_t ctb_size = 1u << ctb_log2;
            const uint32_t pic_w_ctbs = (sps.pic_width_in_luma_samples + ctb_size - 1) / ctb_size;
            const uint32_t pic_h_ctbs = (sps.pic_height_in_luma_samples + ctb_size - 1) / ctb_size;
            if (!p.uniform_spacing_flag) {
                uint32_t sum_w = 0, sum_h = 0;
                for (uint32_t i = 0; i < p.num_tile_columns_minus1; ++i) {
                    p.column_width_minus1[i] = uint16_t(r.ue());
                    sum_w += p.column_width_minus1[i] + 1;
                }
                if (sum_w < pic_w_ctbs) p.column_width_minus1[p.num_tile_columns_minus1] = uint16_t(pic_w_ctbs - sum_w - 1);
                for (uint32_t i = 0; i < p.num_tile_rows_minus1; ++i) {
                    p.row_height_minus1[i] = uint16_t(r.ue());
                    sum_h += p.row_height_minus1[i] + 1;
                }
                if (sum_h < pic_h_ctbs) p.row_height_minus1[p.num_tile_rows_minus1] = uint16_t(pic_h_ctbs - sum_h - 1);
            } else {
                const uint32_t cols = p.num_tile_columns_minus1 + 1;
                const uint32_t rows = p.num_tile_rows_minus1 + 1;
                for (uint32_t i = 0; i < cols; ++i) {
                    const uint32_t w = ((i + 1) * pic_w_ctbs) / cols - (i * pic_w_ctbs) / cols;
                    p.column_width_minus1[i] = uint16_t(w > 0 ? w - 1 : 0);
                }
                for (uint32_t i = 0; i < rows; ++i) {
                    const uint32_t h = ((i + 1) * pic_h_ctbs) / rows - (i * pic_h_ctbs) / rows;
                    p.row_height_minus1[i] = uint16_t(h > 0 ? h - 1 : 0);
                }
            }
            p.loop_filter_across_tiles_enabled_flag = r.flag();
        }
        p.pps_loop_filter_across_slices_enabled_flag = r.flag();
        p.deblocking_filter_control_present_flag = r.flag();
        if (p.deblocking_filter_control_present_flag) {
            p.deblocking_filter_override_enabled_flag = r.flag();
            p.pps_deblocking_filter_disabled_flag = r.flag();
            if (!p.pps_deblocking_filter_disabled_flag) {
                p.pps_beta_offset_div2 = r.se();
                p.pps_tc_offset_div2 = r.se();
            }
        }
        if (r.flag()) skip_scaling_list_data(r);  // pps_scaling_list_data_present_flag
        p.lists_modification_present_flag = r.flag();
        p.log2_parallel_merge_level_minus2 = r.ue();
        p.slice_segment_header_extension_present_flag = r.flag();
        if (!r.ok()) {
            if (err) *err = "truncated HEVC PPS";
            return false;
        }
        p.valid = true;
        pps_[p.pps_id] = p;
        return true;
    }

    int32_t compute_poc(const HevcSps& sps, uint8_t nal_type, uint8_t temporal_id, uint32_t poc_lsb) {
        if (nal_type == 19 || nal_type == 20) {
            prev_tid0_poc_ = 0;
            return 0;
        }
        const int32_t max_lsb = 1 << (sps.log2_max_pic_order_cnt_lsb_minus4 + 4);
        const int32_t prev_lsb = prev_tid0_poc_ & (max_lsb - 1);
        const int32_t prev_msb = prev_tid0_poc_ - prev_lsb;
        int32_t msb = 0;
        if (int32_t(poc_lsb) < prev_lsb && (prev_lsb - int32_t(poc_lsb)) >= (max_lsb / 2)) {
            msb = prev_msb + max_lsb;
        } else if (int32_t(poc_lsb) > prev_lsb && (int32_t(poc_lsb) - prev_lsb) > (max_lsb / 2)) {
            msb = prev_msb - max_lsb;
        } else {
            msb = prev_msb;
        }
        if (nal_type == 16 || nal_type == 17 || nal_type == 18) msb = 0;
        const int32_t poc = msb + int32_t(poc_lsb);
        // SLNR pictures (RADL_N=6, RASL_N=8, TRAIL_N=0, TSA_N=2, STSA_N=4) do not update prevTid0Pic
        const bool sublayer_non_ref = (nal_type <= 14 && (nal_type & 1) == 0);
        if (temporal_id == 0 && !sublayer_non_ref && (nal_type < 6 || nal_type > 9)) {
            prev_tid0_poc_ = poc;
        }
        return poc;
    }

    static void skip_pred_weight_table(BitReader& r, const HevcSps& sps, uint32_t slice_type, uint32_t num_l0,
                                       uint32_t num_l1, VASliceParameterBufferHEVC& sp) {
        sp.luma_log2_weight_denom = uint8_t(r.ue());
        const bool chroma = (sps.chroma_format_idc != 0);
        if (chroma) sp.delta_chroma_log2_weight_denom = int8_t(r.se());
        const int chroma_log2 = int(sp.luma_log2_weight_denom) + int(sp.delta_chroma_log2_weight_denom);
        bool luma_w_l0[15]{}, chroma_w_l0[15]{};
        for (uint32_t i = 0; i <= num_l0; ++i) luma_w_l0[i] = r.flag();
        if (chroma) {
            for (uint32_t i = 0; i <= num_l0; ++i) chroma_w_l0[i] = r.flag();
        }
        for (uint32_t i = 0; i <= num_l0; ++i) {
            if (luma_w_l0[i]) {
                sp.delta_luma_weight_l0[i] = int8_t(r.se());
                sp.luma_offset_l0[i] = int8_t(r.se());
            }
            if (chroma_w_l0[i]) {
                for (int j = 0; j < 2; ++j) {
                    sp.delta_chroma_weight_l0[i][j] = int8_t(r.se());
                    const int32_t delta_off = r.se();
                    const int32_t w = (1 << chroma_log2) + sp.delta_chroma_weight_l0[i][j];
                    const int32_t off = std::clamp(128 + delta_off - ((128 * w) >> chroma_log2), -128, 127);
                    sp.ChromaOffsetL0[i][j] = int8_t(off);
                }
            }
        }
        if (slice_type == 0) {
            bool luma_w_l1[15]{}, chroma_w_l1[15]{};
            for (uint32_t i = 0; i <= num_l1; ++i) luma_w_l1[i] = r.flag();
            if (chroma) {
                for (uint32_t i = 0; i <= num_l1; ++i) chroma_w_l1[i] = r.flag();
            }
            for (uint32_t i = 0; i <= num_l1; ++i) {
                if (luma_w_l1[i]) {
                    sp.delta_luma_weight_l1[i] = int8_t(r.se());
                    sp.luma_offset_l1[i] = int8_t(r.se());
                }
                if (chroma_w_l1[i]) {
                    for (int j = 0; j < 2; ++j) {
                        sp.delta_chroma_weight_l1[i][j] = int8_t(r.se());
                        const int32_t delta_off = r.se();
                        const int32_t w = (1 << chroma_log2) + sp.delta_chroma_weight_l1[i][j];
                        const int32_t off = std::clamp(128 + delta_off - ((128 * w) >> chroma_log2), -128, 127);
                        sp.ChromaOffsetL1[i][j] = int8_t(off);
                    }
                }
            }
        }
    }

    bool decode_slices(const std::vector<std::span<const uint8_t>>& slice_nals, DecodedUnit& out, std::string* err) {
        struct ParsedSlice {
            VASliceParameterBufferHEVC sp{};
            std::span<const uint8_t> nal;
        };
        std::vector<ParsedSlice> slices;
        const HevcSps* sps = nullptr;
        const HevcPps* pps = nullptr;
        int32_t pic_poc = 0;
        uint8_t pic_nal_type = 0;
        uint32_t pic_st_rps_bits = 0;
        bool pic_all_intra = true;
        VAPictureHEVC ref_frames[15];
        for (auto& rf : ref_frames) rf = invalid_hevc_pic();
        std::vector<uint8_t> st_curr_before, st_curr_after, lt_curr;

        for (size_t s_idx = 0; s_idx < slice_nals.size(); ++s_idx) {
            const auto nal = slice_nals[s_idx];
            const uint8_t nal_type = (nal[0] >> 1) & 0x3fu;
            const uint8_t temporal_id = (nal[1] & 7u) ? uint8_t((nal[1] & 7u) - 1) : 0;
            const bool irap = (nal_type >= 16 && nal_type <= 23);
            const bool idr = (nal_type == 19 || nal_type == 20);
            BitReader r(nal.subspan(2), true, 1024);

            const bool first_slice = r.flag();
            if (irap) r.flag();  // no_output_of_prior_pics_flag
            const uint32_t pps_id = r.ue();
            if (pps_id >= 64 || !pps_[pps_id].valid) {
                if (err) *err = "HEVC slice references invalid PPS";
                return false;
            }
            const HevcPps& cur_pps = pps_[pps_id];
            const HevcSps& cur_sps = sps_[cur_pps.sps_id];
            if (!cur_sps.valid) {
                if (err) *err = "HEVC slice references invalid SPS";
                return false;
            }

            bool dependent = false;
            uint32_t slice_segment_address = 0;
            if (!first_slice) {
                if (cur_pps.dependent_slice_segments_enabled_flag) dependent = r.flag();
                const uint32_t ctb_log2 = cur_sps.log2_min_luma_coding_block_size_minus3 + 3 +
                                          cur_sps.log2_diff_max_min_luma_coding_block_size;
                const uint32_t ctb_size = 1u << ctb_log2;
                const uint32_t pic_ctbs = ((cur_sps.pic_width_in_luma_samples + ctb_size - 1) / ctb_size) *
                                          ((cur_sps.pic_height_in_luma_samples + ctb_size - 1) / ctb_size);
                slice_segment_address = r.u(ceil_log2(pic_ctbs));
            }

            VASliceParameterBufferHEVC sp{};
            std::memset(sp.RefPicList, 0xff, sizeof(sp.RefPicList));
            sp.slice_data_size = uint32_t(nal.size());
            sp.slice_data_offset = 0;
            sp.slice_data_flag = VA_SLICE_DATA_FLAG_ALL;
            sp.slice_segment_address = slice_segment_address;
            sp.LongSliceFlags.fields.LastSliceOfPic = (s_idx + 1 == slice_nals.size()) ? 1 : 0;
            sp.LongSliceFlags.fields.dependent_slice_segment_flag = dependent ? 1 : 0;
            sp.collocated_ref_idx = 0xff;

            if (dependent && !slices.empty()) {
                const VASliceParameterBufferHEVC& prev = slices.back().sp;
                sp.LongSliceFlags.fields.slice_type = prev.LongSliceFlags.fields.slice_type;
                sp.LongSliceFlags.fields.color_plane_id = prev.LongSliceFlags.fields.color_plane_id;
                sp.LongSliceFlags.fields.slice_sao_luma_flag = prev.LongSliceFlags.fields.slice_sao_luma_flag;
                sp.LongSliceFlags.fields.slice_sao_chroma_flag = prev.LongSliceFlags.fields.slice_sao_chroma_flag;
                sp.LongSliceFlags.fields.mvd_l1_zero_flag = prev.LongSliceFlags.fields.mvd_l1_zero_flag;
                sp.LongSliceFlags.fields.cabac_init_flag = prev.LongSliceFlags.fields.cabac_init_flag;
                sp.LongSliceFlags.fields.slice_temporal_mvp_enabled_flag =
                    prev.LongSliceFlags.fields.slice_temporal_mvp_enabled_flag;
                sp.LongSliceFlags.fields.slice_deblocking_filter_disabled_flag =
                    prev.LongSliceFlags.fields.slice_deblocking_filter_disabled_flag;
                sp.LongSliceFlags.fields.collocated_from_l0_flag = prev.LongSliceFlags.fields.collocated_from_l0_flag;
                sp.LongSliceFlags.fields.slice_loop_filter_across_slices_enabled_flag =
                    prev.LongSliceFlags.fields.slice_loop_filter_across_slices_enabled_flag;
                sp.collocated_ref_idx = prev.collocated_ref_idx;
                sp.num_ref_idx_l0_active_minus1 = prev.num_ref_idx_l0_active_minus1;
                sp.num_ref_idx_l1_active_minus1 = prev.num_ref_idx_l1_active_minus1;
                std::memcpy(sp.RefPicList, prev.RefPicList, sizeof(sp.RefPicList));
                sp.five_minus_max_num_merge_cand = prev.five_minus_max_num_merge_cand;
            } else {
                r.u(cur_pps.num_extra_slice_header_bits);
                const uint32_t slice_type = r.ue();
                if (slice_type > 2) return false;
                if (slice_type != 2) pic_all_intra = false;
                sp.LongSliceFlags.fields.slice_type = slice_type;
                if (cur_pps.output_flag_present_flag) r.flag();
                if (cur_sps.separate_colour_plane_flag) sp.LongSliceFlags.fields.color_plane_id = r.u(2);

                uint32_t poc_lsb = 0;
                StRps cur_rps{};
                uint32_t st_rps_bits = 0;
                if (!idr) {
                    poc_lsb = r.u(cur_sps.log2_max_pic_order_cnt_lsb_minus4 + 4);
                    const bool sps_rps_flag = r.flag();
                    if (!sps_rps_flag) {
                        const size_t start_bit = r.bit_pos();
                        if (!parse_st_rps(r, cur_sps.num_short_term_ref_pic_sets, cur_sps.num_short_term_ref_pic_sets,
                                          cur_sps.st_rps.data(), cur_rps)) {
                            if (err) *err = "invalid HEVC slice short-term RPS";
                            return false;
                        }
                        st_rps_bits = uint32_t(r.bit_pos() - start_bit);
                    } else {
                        uint32_t idx = 0;
                        if (cur_sps.num_short_term_ref_pic_sets > 1) {
                            idx = r.u(ceil_log2(cur_sps.num_short_term_ref_pic_sets));
                        }
                        if (idx >= cur_sps.num_short_term_ref_pic_sets) return false;
                        cur_rps = cur_sps.st_rps[idx];
                    }
                    if (cur_sps.long_term_ref_pics_present_flag) {
                        uint32_t num_lt_sps = 0;
                        if (cur_sps.num_long_term_ref_pics_sps > 0) num_lt_sps = r.ue();
                        const uint32_t num_lt_pics = r.ue();
                        const uint32_t total_lt = num_lt_sps + num_lt_pics;
                        if (total_lt > 16) return false;
                        for (uint32_t i = 0; i < total_lt; ++i) {
                            if (i < num_lt_sps) {
                                if (cur_sps.num_long_term_ref_pics_sps > 1) {
                                    r.u(ceil_log2(cur_sps.num_long_term_ref_pics_sps));
                                }
                            } else {
                                r.u(cur_sps.log2_max_pic_order_cnt_lsb_minus4 + 4);
                                r.flag();
                            }
                            if (r.flag()) r.ue();  // delta_poc_msb_cycle_lt
                        }
                    }
                    if (cur_sps.sps_temporal_mvp_enabled_flag) {
                        sp.LongSliceFlags.fields.slice_temporal_mvp_enabled_flag = r.flag() ? 1 : 0;
                    }
                }

                if (s_idx == 0) {
                    sps = &cur_sps;
                    pps = &cur_pps;
                    pic_nal_type = nal_type;
                    pic_st_rps_bits = st_rps_bits;
                    pic_poc = compute_poc(*sps, nal_type, temporal_id, poc_lsb);
                    if (idr || nal_type == 16 || nal_type == 17 || nal_type == 18) {
                        clear_dpb();
                    } else {
                        // Build ReferenceFrames[15] and RPS subsets from cur_rps.
                        for (auto& e : dpb_) e.referenced = false;
                        uint8_t rf_count = 0;
                        auto find_and_add = [&](int32_t target_poc, uint32_t flag, std::vector<uint8_t>* subset) {
                            for (auto& e : dpb_) {
                                if (e.poc == target_poc && rf_count < 15) {
                                    e.referenced = true;
                                    uint8_t idx = rf_count++;
                                    ref_frames[idx].picture_id = e.surface;
                                    ref_frames[idx].pic_order_cnt = e.poc;
                                    ref_frames[idx].flags = flag;
                                    if (subset) subset->push_back(idx);
                                    return;
                                }
                            }
                        };
                        for (uint32_t i = 0; i < cur_rps.num_negative_pics; ++i) {
                            const int32_t ref_poc = pic_poc + cur_rps.delta_poc_s0[i];
                            find_and_add(ref_poc, cur_rps.used_s0[i] ? VA_PICTURE_HEVC_RPS_ST_CURR_BEFORE : 0,
                                         cur_rps.used_s0[i] ? &st_curr_before : nullptr);
                        }
                        for (uint32_t i = 0; i < cur_rps.num_positive_pics; ++i) {
                            const int32_t ref_poc = pic_poc + cur_rps.delta_poc_s1[i];
                            find_and_add(ref_poc, cur_rps.used_s1[i] ? VA_PICTURE_HEVC_RPS_ST_CURR_AFTER : 0,
                                         cur_rps.used_s1[i] ? &st_curr_after : nullptr);
                        }
                        // Drop unreferenced pictures from DPB.
                        for (auto it = dpb_.begin(); it != dpb_.end();) {
                            if (!it->referenced) {
                                session_->unref_surface(it->surface_idx);
                                it = dpb_.erase(it);
                            } else {
                                ++it;
                            }
                        }
                    }
                }

                if (cur_sps.sample_adaptive_offset_enabled_flag) {
                    sp.LongSliceFlags.fields.slice_sao_luma_flag = r.flag() ? 1 : 0;
                    if (cur_sps.chroma_format_idc != 0) {
                        sp.LongSliceFlags.fields.slice_sao_chroma_flag = r.flag() ? 1 : 0;
                    }
                }

                uint32_t num_l0 = cur_pps.num_ref_idx_l0_default_active_minus1;
                uint32_t num_l1 = cur_pps.num_ref_idx_l1_default_active_minus1;
                bool mod_l0 = false, mod_l1 = false;
                uint32_t list_entry_l0[15]{}, list_entry_l1[15]{};
                if (slice_type == 1 || slice_type == 0) {
                    if (r.flag()) {
                        num_l0 = r.ue();
                        if (slice_type == 0) num_l1 = r.ue();
                    }
                    if (num_l0 > 14 || num_l1 > 14) return false;
                    const uint32_t num_pic_total_curr =
                        uint32_t(st_curr_before.size() + st_curr_after.size() + lt_curr.size());
                    if (cur_pps.lists_modification_present_flag && num_pic_total_curr > 1) {
                        const uint32_t bits = ceil_log2(num_pic_total_curr);
                        mod_l0 = r.flag();
                        if (mod_l0) {
                            for (uint32_t i = 0; i <= num_l0; ++i) list_entry_l0[i] = r.u(bits);
                        }
                        if (slice_type == 0) {
                            mod_l1 = r.flag();
                            if (mod_l1) {
                                for (uint32_t i = 0; i <= num_l1; ++i) list_entry_l1[i] = r.u(bits);
                            }
                        }
                    }
                    if (slice_type == 0) sp.LongSliceFlags.fields.mvd_l1_zero_flag = r.flag() ? 1 : 0;
                    if (cur_pps.cabac_init_present_flag) sp.LongSliceFlags.fields.cabac_init_flag = r.flag() ? 1 : 0;
                    if (sp.LongSliceFlags.fields.slice_temporal_mvp_enabled_flag) {
                        bool collocated_from_l0 = true;
                        if (slice_type == 0) collocated_from_l0 = r.flag();
                        sp.LongSliceFlags.fields.collocated_from_l0_flag = collocated_from_l0 ? 1 : 0;
                        if ((collocated_from_l0 && num_l0 > 0) || (!collocated_from_l0 && num_l1 > 0)) {
                            sp.collocated_ref_idx = uint8_t(r.ue());
                        } else {
                            sp.collocated_ref_idx = 0;
                        }
                    } else {
                        sp.LongSliceFlags.fields.collocated_from_l0_flag = 1;
                        sp.collocated_ref_idx = 0xff;
                    }
                    if ((cur_pps.weighted_pred_flag && slice_type == 1) ||
                        (cur_pps.weighted_bipred_flag && slice_type == 0)) {
                        skip_pred_weight_table(r, cur_sps, slice_type, num_l0, num_l1, sp);
                    }
                    sp.five_minus_max_num_merge_cand = uint8_t(r.ue());

                    // Construct RefPicList[0] and RefPicList[1] (indices into ReferenceFrames[15]).
                    std::vector<uint8_t> temp0, temp1;
                    temp0.insert(temp0.end(), st_curr_before.begin(), st_curr_before.end());
                    temp0.insert(temp0.end(), st_curr_after.begin(), st_curr_after.end());
                    temp0.insert(temp0.end(), lt_curr.begin(), lt_curr.end());
                    if (!temp0.empty()) {
                        for (uint32_t i = 0; i <= num_l0; ++i) {
                            const uint32_t idx = mod_l0 ? list_entry_l0[i] : i;
                            sp.RefPicList[0][i] = temp0[idx % temp0.size()];
                        }
                    }
                    if (slice_type == 0) {
                        temp1.insert(temp1.end(), st_curr_after.begin(), st_curr_after.end());
                        temp1.insert(temp1.end(), st_curr_before.begin(), st_curr_before.end());
                        temp1.insert(temp1.end(), lt_curr.begin(), lt_curr.end());
                        if (!temp1.empty()) {
                            for (uint32_t i = 0; i <= num_l1; ++i) {
                                const uint32_t idx = mod_l1 ? list_entry_l1[i] : i;
                                sp.RefPicList[1][i] = temp1[idx % temp1.size()];
                            }
                        }
                    }
                }
                sp.num_ref_idx_l0_active_minus1 = (slice_type == 2) ? 0 : uint8_t(num_l0);
                sp.num_ref_idx_l1_active_minus1 = (slice_type == 0) ? uint8_t(num_l1) : 0;

                sp.slice_qp_delta = int8_t(r.se());
                if (cur_pps.pps_slice_chroma_qp_offsets_present_flag) {
                    sp.slice_cb_qp_offset = int8_t(r.se());
                    sp.slice_cr_qp_offset = int8_t(r.se());
                }
                bool deblock_override = false;
                if (cur_pps.deblocking_filter_override_enabled_flag) deblock_override = r.flag();
                bool deblock_disabled = cur_pps.pps_deblocking_filter_disabled_flag;
                int32_t beta_div2 = cur_pps.pps_beta_offset_div2;
                int32_t tc_div2 = cur_pps.pps_tc_offset_div2;
                if (deblock_override) {
                    deblock_disabled = r.flag();
                    if (!deblock_disabled) {
                        beta_div2 = r.se();
                        tc_div2 = r.se();
                    }
                }
                sp.LongSliceFlags.fields.slice_deblocking_filter_disabled_flag = deblock_disabled ? 1 : 0;
                sp.slice_beta_offset_div2 = int8_t(beta_div2);
                sp.slice_tc_offset_div2 = int8_t(tc_div2);

                bool lf_across_slices = cur_pps.pps_loop_filter_across_slices_enabled_flag;
                if (cur_pps.pps_loop_filter_across_slices_enabled_flag &&
                    (sp.LongSliceFlags.fields.slice_sao_luma_flag || sp.LongSliceFlags.fields.slice_sao_chroma_flag ||
                     !deblock_disabled)) {
                    lf_across_slices = r.flag();
                }
                sp.LongSliceFlags.fields.slice_loop_filter_across_slices_enabled_flag = lf_across_slices ? 1 : 0;
            }

            if (cur_pps.tiles_enabled_flag || cur_pps.entropy_coding_sync_enabled_flag) {
                const uint32_t num_ep = r.ue();
                sp.num_entry_point_offsets = uint16_t(num_ep);
                if (num_ep > 0) {
                    const uint32_t off_len_minus1 = r.ue();
                    if (off_len_minus1 > 31) return false;
                    for (uint32_t i = 0; i < num_ep; ++i) r.u(off_len_minus1 + 1);
                }
            }
            if (cur_pps.slice_segment_header_extension_present_flag) {
                const uint32_t ext_len = r.ue();
                for (uint32_t i = 0; i < ext_len; ++i) r.u(8);
            }
            r.hevc_byte_alignment();
            if (!r.ok()) {
                if (err) *err = "invalid HEVC slice header";
                return false;
            }
            const size_t rbsp_header_bytes = r.byte_pos();
            sp.slice_data_byte_offset = uint32_t(2 + rbsp_header_bytes);
            sp.slice_data_num_emu_prevn_bytes = uint16_t(r.emulation_bytes_in_prefix(rbsp_header_bytes));
            slices.push_back(ParsedSlice{sp, nal});
        }

        VASurfaceID target = VA_INVALID_SURFACE;
        const int surf_idx =
            session_->acquire_surface(sps->pic_width_in_luma_samples, sps->pic_height_in_luma_samples, target, err);
        if (surf_idx < 0) return false;

        VAPictureParameterBufferHEVC pp{};
        pp.CurrPic.picture_id = target;
        pp.CurrPic.pic_order_cnt = pic_poc;
        pp.CurrPic.flags = 0;
        for (int i = 0; i < 15; ++i) pp.ReferenceFrames[i] = ref_frames[i];
        pp.pic_width_in_luma_samples = uint16_t(sps->pic_width_in_luma_samples);
        pp.pic_height_in_luma_samples = uint16_t(sps->pic_height_in_luma_samples);
        pp.pic_fields.bits.chroma_format_idc = sps->chroma_format_idc;
        pp.pic_fields.bits.separate_colour_plane_flag = sps->separate_colour_plane_flag;
        pp.pic_fields.bits.pcm_enabled_flag = sps->pcm_enabled_flag;
        pp.pic_fields.bits.scaling_list_enabled_flag = sps->scaling_list_enabled_flag;
        pp.pic_fields.bits.transform_skip_enabled_flag = pps->transform_skip_enabled_flag;
        pp.pic_fields.bits.amp_enabled_flag = sps->amp_enabled_flag;
        pp.pic_fields.bits.strong_intra_smoothing_enabled_flag = sps->strong_intra_smoothing_enabled_flag;
        pp.pic_fields.bits.sign_data_hiding_enabled_flag = pps->sign_data_hiding_enabled_flag;
        pp.pic_fields.bits.constrained_intra_pred_flag = pps->constrained_intra_pred_flag;
        pp.pic_fields.bits.cu_qp_delta_enabled_flag = pps->cu_qp_delta_enabled_flag;
        pp.pic_fields.bits.weighted_pred_flag = pps->weighted_pred_flag;
        pp.pic_fields.bits.weighted_bipred_flag = pps->weighted_bipred_flag;
        pp.pic_fields.bits.transquant_bypass_enabled_flag = pps->transquant_bypass_enabled_flag;
        pp.pic_fields.bits.tiles_enabled_flag = pps->tiles_enabled_flag;
        pp.pic_fields.bits.entropy_coding_sync_enabled_flag = pps->entropy_coding_sync_enabled_flag;
        pp.pic_fields.bits.pps_loop_filter_across_slices_enabled_flag = pps->pps_loop_filter_across_slices_enabled_flag;
        pp.pic_fields.bits.loop_filter_across_tiles_enabled_flag = pps->loop_filter_across_tiles_enabled_flag;
        pp.pic_fields.bits.pcm_loop_filter_disabled_flag = sps->pcm_loop_filter_disabled_flag;
        pp.pic_fields.bits.NoPicReorderingFlag = (sps->sps_max_num_reorder_pics == 0) ? 1 : 0;
        pp.pic_fields.bits.NoBiPredFlag = 1;
        pp.sps_max_dec_pic_buffering_minus1 = uint8_t(sps->sps_max_dec_pic_buffering_minus1);
        pp.bit_depth_luma_minus8 = uint8_t(sps->bit_depth_luma_minus8);
        pp.bit_depth_chroma_minus8 = uint8_t(sps->bit_depth_chroma_minus8);
        pp.pcm_sample_bit_depth_luma_minus1 = uint8_t(sps->pcm_sample_bit_depth_luma_minus1);
        pp.pcm_sample_bit_depth_chroma_minus1 = uint8_t(sps->pcm_sample_bit_depth_chroma_minus1);
        pp.log2_min_luma_coding_block_size_minus3 = uint8_t(sps->log2_min_luma_coding_block_size_minus3);
        pp.log2_diff_max_min_luma_coding_block_size = uint8_t(sps->log2_diff_max_min_luma_coding_block_size);
        pp.log2_min_transform_block_size_minus2 = uint8_t(sps->log2_min_luma_transform_block_size_minus2);
        pp.log2_diff_max_min_transform_block_size = uint8_t(sps->log2_diff_max_min_luma_transform_block_size);
        pp.log2_min_pcm_luma_coding_block_size_minus3 = uint8_t(sps->log2_min_pcm_luma_coding_block_size_minus3);
        pp.log2_diff_max_min_pcm_luma_coding_block_size = uint8_t(sps->log2_diff_max_min_pcm_luma_coding_block_size);
        pp.max_transform_hierarchy_depth_intra = uint8_t(sps->max_transform_hierarchy_depth_intra);
        pp.max_transform_hierarchy_depth_inter = uint8_t(sps->max_transform_hierarchy_depth_inter);
        pp.init_qp_minus26 = int8_t(pps->init_qp_minus26);
        pp.diff_cu_qp_delta_depth = uint8_t(pps->diff_cu_qp_delta_depth);
        pp.pps_cb_qp_offset = int8_t(pps->pps_cb_qp_offset);
        pp.pps_cr_qp_offset = int8_t(pps->pps_cr_qp_offset);
        pp.log2_parallel_merge_level_minus2 = uint8_t(pps->log2_parallel_merge_level_minus2);
        if (pps->tiles_enabled_flag) {
            pp.num_tile_columns_minus1 = uint8_t(pps->num_tile_columns_minus1);
            pp.num_tile_rows_minus1 = uint8_t(pps->num_tile_rows_minus1);
            std::memcpy(pp.column_width_minus1, pps->column_width_minus1, sizeof(pp.column_width_minus1));
            std::memcpy(pp.row_height_minus1, pps->row_height_minus1, sizeof(pp.row_height_minus1));
        }
        pp.slice_parsing_fields.bits.lists_modification_present_flag = pps->lists_modification_present_flag;
        pp.slice_parsing_fields.bits.long_term_ref_pics_present_flag = sps->long_term_ref_pics_present_flag;
        pp.slice_parsing_fields.bits.sps_temporal_mvp_enabled_flag = sps->sps_temporal_mvp_enabled_flag;
        pp.slice_parsing_fields.bits.cabac_init_present_flag = pps->cabac_init_present_flag;
        pp.slice_parsing_fields.bits.output_flag_present_flag = pps->output_flag_present_flag;
        pp.slice_parsing_fields.bits.dependent_slice_segments_enabled_flag = pps->dependent_slice_segments_enabled_flag;
        pp.slice_parsing_fields.bits.pps_slice_chroma_qp_offsets_present_flag =
            pps->pps_slice_chroma_qp_offsets_present_flag;
        pp.slice_parsing_fields.bits.sample_adaptive_offset_enabled_flag = sps->sample_adaptive_offset_enabled_flag;
        pp.slice_parsing_fields.bits.deblocking_filter_override_enabled_flag =
            pps->deblocking_filter_override_enabled_flag;
        pp.slice_parsing_fields.bits.pps_disable_deblocking_filter_flag = pps->pps_deblocking_filter_disabled_flag;
        pp.slice_parsing_fields.bits.slice_segment_header_extension_present_flag =
            pps->slice_segment_header_extension_present_flag;
        pp.slice_parsing_fields.bits.RapPicFlag = (pic_nal_type >= 16 && pic_nal_type <= 21) ? 1 : 0;
        pp.slice_parsing_fields.bits.IdrPicFlag = (pic_nal_type == 19 || pic_nal_type == 20) ? 1 : 0;
        pp.slice_parsing_fields.bits.IntraPicFlag = pic_all_intra ? 1 : 0;
        pp.log2_max_pic_order_cnt_lsb_minus4 = uint8_t(sps->log2_max_pic_order_cnt_lsb_minus4);
        pp.num_short_term_ref_pic_sets = uint8_t(sps->num_short_term_ref_pic_sets);
        pp.num_long_term_ref_pic_sps = uint8_t(sps->num_long_term_ref_pics_sps);
        pp.num_ref_idx_l0_default_active_minus1 = uint8_t(pps->num_ref_idx_l0_default_active_minus1);
        pp.num_ref_idx_l1_default_active_minus1 = uint8_t(pps->num_ref_idx_l1_default_active_minus1);
        pp.pps_beta_offset_div2 = int8_t(pps->pps_beta_offset_div2);
        pp.pps_tc_offset_div2 = int8_t(pps->pps_tc_offset_div2);
        pp.num_extra_slice_header_bits = uint8_t(pps->num_extra_slice_header_bits);
        pp.st_rps_bits = pic_st_rps_bits;

        VAIQMatrixBufferHEVC iq{};
        if (sps->scaling_list_enabled_flag) init_default_hevc_iq(iq);

        const bool ok = session_->submit_picture(
            target,
            [&](BufferList& bufs, std::string* berr) {
                if (!bufs.add(VAPictureParameterBufferType, pp, berr)) return false;
                if (sps->scaling_list_enabled_flag && !bufs.add(VAIQMatrixBufferType, iq, berr)) return false;
                for (const auto& sl : slices) {
                    if (!bufs.add(VASliceParameterBufferType, sl.sp, berr)) return false;
                    if (!bufs.add(VASliceDataBufferType, sl.nal.data(), sl.nal.size(), berr)) return false;
                }
                return true;
            },
            err);
        if (!ok) return false;

        session_->ref_surface(surf_idx);
        HevcDpbEntry e{};
        e.surface_idx = surf_idx;
        e.surface = target;
        e.poc = pic_poc;
        dpb_.push_back(e);

        out.has_output = true;
        out.surface = target;
        out.surface_idx = surf_idx;
        out.coded_width = sps->pic_width_in_luma_samples;
        out.coded_height = sps->pic_height_in_luma_samples;
        out.vis_x = sps->vis_x;
        out.vis_y = sps->vis_y;
        out.vis_w = sps->vis_w;
        out.vis_h = sps->vis_h;
        return true;
    }

    DecodeSession* session_ = nullptr;
    std::array<HevcSps, 16> sps_{};
    std::array<HevcPps, 64> pps_{};
    std::vector<HevcDpbEntry> dpb_;
    int32_t prev_tid0_poc_ = 0;
};

}  // namespace

std::unique_ptr<DecodeCodec> make_hevc_decoder() { return std::make_unique<HevcDecoder>(); }

}  // namespace brovideo::vaapi
