// H.264 Annex B bitstream parser and VA-API VLD buffer builder.
#include "vaapi/va_bitreader.h"
#include "vaapi/va_decoder.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <vector>

namespace brovideo::vaapi {
namespace {

// H.264 Table 7-3 / 7-4: zig-zag scan to raster index.
constexpr uint8_t kZigzag4x4[16] = {0, 1, 4, 8, 5, 2, 3, 6, 9, 12, 13, 10, 7, 11, 14, 15};

constexpr uint8_t kZigzag8x8[64] = {0,  1,  8,  16, 9,  2,  3,  10, 17, 24, 32, 25, 18, 11, 4,  5,
                                    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6,  7,  14, 21, 28,
                                    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
                                    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63};

// H.264 Table 7-2 default scaling lists (in zig-zag order).
constexpr uint8_t kDefault4x4Intra[16] = {6, 13, 13, 20, 20, 20, 28, 28, 28, 28, 32, 32, 32, 37, 37, 42};
constexpr uint8_t kDefault4x4Inter[16] = {10, 14, 14, 20, 20, 20, 24, 24, 24, 24, 27, 27, 27, 30, 30, 34};
constexpr uint8_t kDefault8x8Intra[64] = {
    6,  10, 10, 13, 11, 13, 16, 16, 16, 16, 18, 18, 18, 18, 18, 23, 23, 23, 23, 23, 23, 25,
    25, 25, 25, 25, 25, 25, 27, 27, 27, 27, 27, 27, 27, 27, 29, 29, 29, 29, 29, 29, 29, 31,
    31, 31, 31, 31, 31, 33, 33, 33, 33, 33, 36, 36, 36, 36, 38, 38, 38, 40, 40, 42};
constexpr uint8_t kDefault8x8Inter[64] = {
    9,  13, 13, 15, 13, 15, 17, 17, 17, 17, 19, 19, 19, 19, 19, 21, 21, 21, 21, 21, 21, 22,
    22, 22, 22, 22, 22, 22, 24, 24, 24, 24, 24, 24, 24, 24, 25, 25, 25, 25, 25, 25, 25, 27,
    27, 27, 27, 27, 27, 28, 28, 28, 28, 28, 30, 30, 30, 30, 32, 32, 32, 33, 33, 35};

void parse_scaling_list(BitReader& r, uint8_t* raster_dst, size_t size, const uint8_t* default_zz,
                        const uint8_t* fallback_raster) {
    const uint8_t* scan = (size == 16) ? kZigzag4x4 : kZigzag8x8;
    int last_scale = 8;
    int next_scale = 8;
    for (size_t j = 0; j < size; ++j) {
        if (next_scale != 0) {
            const int32_t delta_scale = r.se();
            next_scale = (last_scale + delta_scale + 256) % 256;
            if (j == 0 && next_scale == 0) {
                for (size_t k = 0; k < size; ++k) raster_dst[scan[k]] = default_zz[k];
                return;
            }
        }
        const uint8_t val = uint8_t((next_scale == 0) ? last_scale : next_scale);
        raster_dst[scan[j]] = val;
        last_scale = val;
    }
    (void)fallback_raster;
}

struct Sps {
    bool valid = false, separate_colour_plane_flag = false, seq_scaling_matrix_present_flag = false;
    bool delta_pic_order_always_zero_flag = false, gaps_in_frame_num_value_allowed_flag = false;
    bool frame_mbs_only_flag = true, mb_adaptive_frame_field_flag = false, direct_8x8_inference_flag = false;
    uint8_t profile_idc = 0, level_idc = 0;
    uint32_t seq_parameter_set_id = 0, chroma_format_idc = 1, bit_depth_luma_minus8 = 0, bit_depth_chroma_minus8 = 0;
    uint8_t scaling_list_4x4[6][16]{};
    uint8_t scaling_list_8x8[2][64]{};
    uint32_t log2_max_frame_num_minus4 = 0, pic_order_cnt_type = 0, log2_max_pic_order_cnt_lsb_minus4 = 0;
    int32_t offset_for_non_ref_pic = 0, offset_for_top_to_bottom_field = 0;
    uint32_t num_ref_frames_in_pic_order_cnt_cycle = 0, max_num_ref_frames = 1;
    int32_t offset_for_ref_frame[256]{};
    uint32_t pic_width_in_mbs_minus1 = 0, pic_height_in_map_units_minus1 = 0;
    uint32_t coded_width = 0, coded_height = 0, vis_x = 0, vis_y = 0, vis_w = 0, vis_h = 0;
};

struct Pps {
    bool valid = false, entropy_coding_mode_flag = false, bottom_field_pic_order_in_frame_present_flag = false;
    bool weighted_pred_flag = false, deblocking_filter_control_present_flag = false;
    bool constrained_intra_pred_flag = false, redundant_pic_cnt_present_flag = false;
    bool transform_8x8_mode_flag = false, pic_scaling_matrix_present_flag = false;
    uint32_t pic_parameter_set_id = 0, seq_parameter_set_id = 0, num_slice_groups_minus1 = 0;
    uint32_t num_ref_idx_l0_default_active_minus1 = 0, num_ref_idx_l1_default_active_minus1 = 0, weighted_bipred_idc = 0;
    int32_t pic_init_qp_minus26 = 0, pic_init_qs_minus26 = 0, chroma_qp_index_offset = 0;
    int32_t second_chroma_qp_index_offset = 0;
    uint8_t scaling_list_4x4[6][16]{};
    uint8_t scaling_list_8x8[2][64]{};
};

struct DpbEntry {
    int surface_idx = -1;
    VASurfaceID surface = VA_INVALID_SURFACE;
    uint32_t frame_num = 0, long_term_frame_idx = 0;
    int32_t pic_num = 0, poc = 0;
    bool long_term = false;
};

struct ModOp { uint32_t idc = 3, val = 0; };
struct MmcoOp { uint32_t op = 0, val1 = 0, val2 = 0; };

bool is_high_profile(uint8_t p) {
    return p == 100 || p == 110 || p == 122 || p == 244 || p == 44 || p == 83 || p == 86 || p == 118 || p == 128 ||
           p == 138 || p == 139 || p == 134 || p == 135;
}

VAPictureH264 invalid_pic() {
    VAPictureH264 p{};
    p.picture_id = VA_INVALID_SURFACE;
    p.flags = VA_PICTURE_H264_INVALID;
    return p;
}

class H264Decoder final : public DecodeCodec {
public:
    ~H264Decoder() override { clear_dpb(); }

    void reset() override {
        clear_dpb();
        prev_poc_msb_ = 0;
        prev_poc_lsb_ = 0;
        prev_frame_num_ = 0;
        prev_frame_num_offset_ = 0;
        max_long_term_frame_idx_ = -1;
    }

    bool decode_packet(std::span<const uint8_t> packet, DecodeSession& session, DecodedUnit& out,
                       std::string* err) override {
        session_ = &session;
        out = DecodedUnit{};
        const auto nals = split_annexb_nals(packet);
        if (nals.empty()) {
            if (err) *err = "H.264 packet has no Annex B start codes";
            return false;
        }

        std::vector<std::span<const uint8_t>> slice_nals;
        for (const auto& nal : nals) {
            if (nal.empty()) continue;
            if (nal[0] & 0x80) {
                if (err) *err = "H.264 NAL forbidden_zero_bit is set";
                return false;
            }
            const uint8_t type = nal[0] & 0x1f;
            if (type == 7) {
                if (!parse_sps(nal.subspan(1), err)) return false;
            } else if (type == 8) {
                if (!parse_pps(nal.subspan(1), err)) return false;
            } else if (type == 1 || type == 5) {
                slice_nals.push_back(nal);
            }
        }
        if (slice_nals.empty()) return true;  // AUD / SEI / parameter sets only

        return decode_slices(slice_nals, out, err);
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
        Sps s{};
        std::memset(s.scaling_list_4x4, 16, sizeof(s.scaling_list_4x4));
        std::memset(s.scaling_list_8x8, 16, sizeof(s.scaling_list_8x8));
        s.profile_idc = uint8_t(r.u(8));
        r.u(8);  // constraint flags + reserved
        s.level_idc = uint8_t(r.u(8));
        s.seq_parameter_set_id = r.ue();
        if (s.seq_parameter_set_id >= 32) {
            if (err) *err = "H.264 SPS id out of range";
            return false;
        }
        if (is_high_profile(s.profile_idc)) {
            s.chroma_format_idc = r.ue();
            if (s.chroma_format_idc == 3) s.separate_colour_plane_flag = r.flag();
            s.bit_depth_luma_minus8 = r.ue();
            s.bit_depth_chroma_minus8 = r.ue();
            r.flag();  // qpprime_y_zero_transform_bypass_flag
            s.seq_scaling_matrix_present_flag = r.flag();
            if (s.seq_scaling_matrix_present_flag) {
                const int count = (s.chroma_format_idc != 3) ? 8 : 12;
                for (int i = 0; i < count; ++i) {
                    if (r.flag()) {
                        if (i < 6) {
                            parse_scaling_list(r, s.scaling_list_4x4[i], 16,
                                               i < 3 ? kDefault4x4Intra : kDefault4x4Inter, nullptr);
                        } else if (i < 8) {
                            parse_scaling_list(r, s.scaling_list_8x8[i - 6], 64,
                                               (i == 6) ? kDefault8x8Intra : kDefault8x8Inter, nullptr);
                        } else {
                            uint8_t dummy[64];
                            parse_scaling_list(r, dummy, 64, kDefault8x8Intra, nullptr);
                        }
                    } else {
                        if (i < 6) {
                            if (i == 0 || i == 3) {
                                const uint8_t* def = (i == 0) ? kDefault4x4Intra : kDefault4x4Inter;
                                for (int k = 0; k < 16; ++k) s.scaling_list_4x4[i][kZigzag4x4[k]] = def[k];
                            } else {
                                std::memcpy(s.scaling_list_4x4[i], s.scaling_list_4x4[i - 1], 16);
                            }
                        } else if (i < 8) {
                            const uint8_t* def = (i == 6) ? kDefault8x8Intra : kDefault8x8Inter;
                            for (int k = 0; k < 64; ++k) s.scaling_list_8x8[i - 6][kZigzag8x8[k]] = def[k];
                        }
                    }
                }
            }
        }
        s.log2_max_frame_num_minus4 = r.ue();
        if (s.log2_max_frame_num_minus4 > 12) {
            if (err) *err = "H.264 log2_max_frame_num_minus4 out of range";
            return false;
        }
        s.pic_order_cnt_type = r.ue();
        if (s.pic_order_cnt_type == 0) {
            s.log2_max_pic_order_cnt_lsb_minus4 = r.ue();
            if (s.log2_max_pic_order_cnt_lsb_minus4 > 12) {
                if (err) *err = "H.264 log2_max_pic_order_cnt_lsb_minus4 out of range";
                return false;
            }
        } else if (s.pic_order_cnt_type == 1) {
            s.delta_pic_order_always_zero_flag = r.flag();
            s.offset_for_non_ref_pic = r.se();
            s.offset_for_top_to_bottom_field = r.se();
            s.num_ref_frames_in_pic_order_cnt_cycle = r.ue();
            if (s.num_ref_frames_in_pic_order_cnt_cycle > 255) return false;
            for (uint32_t i = 0; i < s.num_ref_frames_in_pic_order_cnt_cycle; ++i) {
                s.offset_for_ref_frame[i] = r.se();
            }
        } else if (s.pic_order_cnt_type > 2) {
            if (err) *err = "H.264 pic_order_cnt_type out of range";
            return false;
        }
        s.max_num_ref_frames = r.ue();
        s.gaps_in_frame_num_value_allowed_flag = r.flag();
        s.pic_width_in_mbs_minus1 = r.ue();
        s.pic_height_in_map_units_minus1 = r.ue();
        s.frame_mbs_only_flag = r.flag();
        if (!s.frame_mbs_only_flag) s.mb_adaptive_frame_field_flag = r.flag();
        s.direct_8x8_inference_flag = r.flag();
        uint32_t crop_l = 0, crop_r = 0, crop_t = 0, crop_b = 0;
        if (r.flag()) {
            crop_l = r.ue();
            crop_r = r.ue();
            crop_t = r.ue();
            crop_b = r.ue();
        }
        if (!r.ok()) {
            if (err) *err = "truncated H.264 SPS";
            return false;
        }
        const uint32_t mbs_h = (s.pic_height_in_map_units_minus1 + 1) * (s.frame_mbs_only_flag ? 1u : 2u);
        s.coded_width = (s.pic_width_in_mbs_minus1 + 1) * 16;
        s.coded_height = mbs_h * 16;
        if (s.coded_width == 0 || s.coded_height == 0 || s.coded_width > 16384 || s.coded_height > 16384) {
            if (err) *err = "invalid H.264 picture dimensions";
            return false;
        }
        const uint32_t sub_w = (s.chroma_format_idc == 1 || s.chroma_format_idc == 2) ? 2u : 1u;
        const uint32_t sub_h = (s.chroma_format_idc == 1) ? 2u : 1u;
        const uint32_t crop_unit_x = (s.chroma_format_idc == 0 || s.separate_colour_plane_flag) ? 1u : sub_w;
        const uint32_t crop_unit_y =
            ((s.chroma_format_idc == 0 || s.separate_colour_plane_flag) ? 1u : sub_h) * (s.frame_mbs_only_flag ? 1u : 2u);
        const uint32_t cx = (crop_l + crop_r) * crop_unit_x;
        const uint32_t cy = (crop_t + crop_b) * crop_unit_y;
        if (cx >= s.coded_width || cy >= s.coded_height) {
            if (err) *err = "invalid H.264 crop offsets";
            return false;
        }
        s.vis_x = crop_l * crop_unit_x;
        s.vis_y = crop_t * crop_unit_y;
        s.vis_w = s.coded_width - cx;
        s.vis_h = s.coded_height - cy;
        s.valid = true;
        sps_[s.seq_parameter_set_id] = s;
        return true;
    }

    bool parse_pps(std::span<const uint8_t> body, std::string* err) {
        BitReader r(body, true);
        Pps p{};
        p.pic_parameter_set_id = r.ue();
        p.seq_parameter_set_id = r.ue();
        if (p.pic_parameter_set_id >= 256 || p.seq_parameter_set_id >= 32 || !sps_[p.seq_parameter_set_id].valid) {
            if (err) *err = "H.264 PPS references invalid SPS";
            return false;
        }
        const Sps& sps = sps_[p.seq_parameter_set_id];
        std::memcpy(p.scaling_list_4x4, sps.scaling_list_4x4, sizeof(p.scaling_list_4x4));
        std::memcpy(p.scaling_list_8x8, sps.scaling_list_8x8, sizeof(p.scaling_list_8x8));
        p.entropy_coding_mode_flag = r.flag();
        p.bottom_field_pic_order_in_frame_present_flag = r.flag();
        p.num_slice_groups_minus1 = r.ue();
        if (p.num_slice_groups_minus1 > 0) {
            if (err) *err = "H.264 FMO slice groups are not supported";
            return false;
        }
        p.num_ref_idx_l0_default_active_minus1 = r.ue();
        p.num_ref_idx_l1_default_active_minus1 = r.ue();
        if (p.num_ref_idx_l0_default_active_minus1 > 31 || p.num_ref_idx_l1_default_active_minus1 > 31) return false;
        p.weighted_pred_flag = r.flag();
        p.weighted_bipred_idc = r.u(2);
        p.pic_init_qp_minus26 = r.se();
        p.pic_init_qs_minus26 = r.se();
        p.chroma_qp_index_offset = r.se();
        p.second_chroma_qp_index_offset = p.chroma_qp_index_offset;
        p.deblocking_filter_control_present_flag = r.flag();
        p.constrained_intra_pred_flag = r.flag();
        p.redundant_pic_cnt_present_flag = r.flag();
        if (r.more_rbsp_data()) {
            p.transform_8x8_mode_flag = r.flag();
            p.pic_scaling_matrix_present_flag = r.flag();
            if (p.pic_scaling_matrix_present_flag) {
                const int count = 6 + ((sps.chroma_format_idc != 3) ? 2 : 6) * (p.transform_8x8_mode_flag ? 1 : 0);
                for (int i = 0; i < count; ++i) {
                    if (r.flag()) {
                        if (i < 6) {
                            parse_scaling_list(r, p.scaling_list_4x4[i], 16,
                                               i < 3 ? kDefault4x4Intra : kDefault4x4Inter, nullptr);
                        } else if (i < 8) {
                            parse_scaling_list(r, p.scaling_list_8x8[i - 6], 64,
                                               (i == 6) ? kDefault8x8Intra : kDefault8x8Inter, nullptr);
                        } else {
                            uint8_t dummy[64];
                            parse_scaling_list(r, dummy, 64, kDefault8x8Intra, nullptr);
                        }
                    } else {
                        if (i < 6) {
                            if (i == 0 || i == 3) {
                                if (!sps.seq_scaling_matrix_present_flag) {
                                    const uint8_t* def = (i == 0) ? kDefault4x4Intra : kDefault4x4Inter;
                                    for (int k = 0; k < 16; ++k) p.scaling_list_4x4[i][kZigzag4x4[k]] = def[k];
                                }
                            } else {
                                std::memcpy(p.scaling_list_4x4[i], p.scaling_list_4x4[i - 1], 16);
                            }
                        } else if (i < 8 && !sps.seq_scaling_matrix_present_flag) {
                            const uint8_t* def = (i == 6) ? kDefault8x8Intra : kDefault8x8Inter;
                            for (int k = 0; k < 64; ++k) p.scaling_list_8x8[i - 6][kZigzag8x8[k]] = def[k];
                        }
                    }
                }
            }
            p.second_chroma_qp_index_offset = r.se();
        }
        if (!r.ok()) {
            if (err) *err = "truncated H.264 PPS";
            return false;
        }
        p.valid = true;
        pps_[p.pic_parameter_set_id] = p;
        return true;
    }

    int32_t compute_poc(const Sps& sps, bool idr, uint32_t nal_ref_idc, uint32_t frame_num, uint32_t poc_lsb,
                        int32_t delta_poc_bottom, int32_t delta_poc0) {
        if (idr) {
            prev_poc_msb_ = 0;
            prev_poc_lsb_ = 0;
            prev_frame_num_offset_ = 0;
            prev_frame_num_ = 0;
        }
        if (sps.pic_order_cnt_type == 0) {
            const uint32_t max_poc_lsb = 1u << (sps.log2_max_pic_order_cnt_lsb_minus4 + 4);
            int32_t poc_msb = 0;
            if (poc_lsb < prev_poc_lsb_ && (prev_poc_lsb_ - poc_lsb) >= (max_poc_lsb / 2)) {
                poc_msb = prev_poc_msb_ + int32_t(max_poc_lsb);
            } else if (poc_lsb > prev_poc_lsb_ && (poc_lsb - prev_poc_lsb_) > (max_poc_lsb / 2)) {
                poc_msb = prev_poc_msb_ - int32_t(max_poc_lsb);
            } else {
                poc_msb = prev_poc_msb_;
            }
            const int32_t top_poc = poc_msb + int32_t(poc_lsb);
            const int32_t bot_poc = top_poc + delta_poc_bottom;
            if (nal_ref_idc != 0) {
                prev_poc_msb_ = poc_msb;
                prev_poc_lsb_ = poc_lsb;
            }
            return std::min(top_poc, bot_poc);
        }
        const uint32_t max_frame_num = 1u << (sps.log2_max_frame_num_minus4 + 4);
        uint32_t frame_num_offset = 0;
        if (!idr) {
            frame_num_offset = prev_frame_num_offset_ + (frame_num < prev_frame_num_ ? max_frame_num : 0);
        }
        prev_frame_num_ = frame_num;
        prev_frame_num_offset_ = frame_num_offset;
        if (sps.pic_order_cnt_type == 2) {
            if (idr) return 0;
            const uint32_t abs_fn = frame_num_offset + frame_num;
            return int32_t(nal_ref_idc ? 2 * abs_fn : 2 * abs_fn - 1);
        }
        // pic_order_cnt_type == 1
        uint32_t abs_fn = (sps.num_ref_frames_in_pic_order_cnt_cycle != 0) ? (frame_num_offset + frame_num) : 0;
        if (nal_ref_idc == 0 && abs_fn > 0) --abs_fn;
        int32_t expected_poc = 0;
        if (abs_fn > 0) {
            const uint32_t cycle = (abs_fn - 1) / sps.num_ref_frames_in_pic_order_cnt_cycle;
            const uint32_t idx = (abs_fn - 1) % sps.num_ref_frames_in_pic_order_cnt_cycle;
            int32_t delta_per_cycle = 0;
            for (uint32_t i = 0; i < sps.num_ref_frames_in_pic_order_cnt_cycle; ++i) {
                delta_per_cycle += sps.offset_for_ref_frame[i];
            }
            expected_poc = int32_t(cycle) * delta_per_cycle;
            for (uint32_t i = 0; i <= idx; ++i) expected_poc += sps.offset_for_ref_frame[i];
        }
        if (nal_ref_idc == 0) expected_poc += sps.offset_for_non_ref_pic;
        return expected_poc + delta_poc0;
    }

    void update_pic_nums(uint32_t curr_frame_num, uint32_t max_frame_num) {
        for (auto& e : dpb_) {
            if (!e.long_term) {
                const int32_t wrap = (e.frame_num > curr_frame_num) ? int32_t(e.frame_num) - int32_t(max_frame_num)
                                                                    : int32_t(e.frame_num);
                e.pic_num = wrap;
            }
        }
    }

    static VAPictureH264 to_va_pic(const DpbEntry& e) {
        VAPictureH264 p{};
        p.picture_id = e.surface;
        p.frame_idx = e.long_term ? e.long_term_frame_idx : e.frame_num;
        p.flags = e.long_term ? VA_PICTURE_H264_LONG_TERM_REFERENCE : VA_PICTURE_H264_SHORT_TERM_REFERENCE;
        p.TopFieldOrderCnt = e.poc;
        p.BottomFieldOrderCnt = e.poc;
        return p;
    }

    void build_ref_list_p(uint32_t curr_frame_num, uint32_t max_frame_num, uint32_t num_active,
                          const std::vector<ModOp>& mods, VAPictureH264 (&out_list)[32]) {
        for (auto& r : out_list) r = invalid_pic();
        std::vector<const DpbEntry*> st, lt;
        for (const auto& e : dpb_) {
            if (e.long_term) lt.push_back(&e);
            else st.push_back(&e);
        }
        std::sort(st.begin(), st.end(), [](const DpbEntry* a, const DpbEntry* b) { return a->pic_num > b->pic_num; });
        std::sort(lt.begin(), lt.end(),
                  [](const DpbEntry* a, const DpbEntry* b) { return a->long_term_frame_idx < b->long_term_frame_idx; });
        std::vector<const DpbEntry*> list;
        list.insert(list.end(), st.begin(), st.end());
        list.insert(list.end(), lt.begin(), lt.end());

        if (!mods.empty()) {
            int32_t pic_num_pred = int32_t(curr_frame_num);
            size_t ref_idx = 0;
            for (const ModOp& m : mods) {
                if (m.idc == 3 || ref_idx >= 32) break;
                const DpbEntry* target = nullptr;
                if (m.idc == 0 || m.idc == 1) {
                    const int32_t diff = int32_t(m.val + 1);
                    int32_t pn_no_wrap = (m.idc == 0) ? (pic_num_pred - diff) : (pic_num_pred + diff);
                    if (pn_no_wrap < 0) pn_no_wrap += int32_t(max_frame_num);
                    if (pn_no_wrap >= int32_t(max_frame_num)) pn_no_wrap -= int32_t(max_frame_num);
                    pic_num_pred = pn_no_wrap;
                    const int32_t pn = (pn_no_wrap > int32_t(curr_frame_num))
                                           ? (pn_no_wrap - int32_t(max_frame_num))
                                           : pn_no_wrap;
                    for (const auto& e : dpb_) {
                        if (!e.long_term && e.pic_num == pn) {
                            target = &e;
                            break;
                        }
                    }
                } else if (m.idc == 2) {
                    for (const auto& e : dpb_) {
                        if (e.long_term && e.long_term_frame_idx == m.val) {
                            target = &e;
                            break;
                        }
                    }
                }
                if (target) {
                    list.insert(list.begin() + std::ptrdiff_t(ref_idx), target);
                    ++ref_idx;
                    for (size_t k = ref_idx; k < list.size();) {
                        if (list[k] == target) list.erase(list.begin() + std::ptrdiff_t(k));
                        else ++k;
                    }
                }
            }
        }
        for (uint32_t i = 0; i < num_active && i < list.size() && i < 32; ++i) {
            out_list[i] = to_va_pic(*list[i]);
        }
    }

    void build_ref_lists_b(int32_t curr_poc, uint32_t num_l0, uint32_t num_l1, VAPictureH264 (&l0)[32],
                           VAPictureH264 (&l1)[32]) {
        for (auto& r : l0) r = invalid_pic();
        for (auto& r : l1) r = invalid_pic();
        std::vector<const DpbEntry*> before, after, lt;
        for (const auto& e : dpb_) {
            if (e.long_term) lt.push_back(&e);
            else if (e.poc < curr_poc) before.push_back(&e);
            else after.push_back(&e);
        }
        std::sort(before.begin(), before.end(), [](const DpbEntry* a, const DpbEntry* b) { return a->poc > b->poc; });
        std::sort(after.begin(), after.end(), [](const DpbEntry* a, const DpbEntry* b) { return a->poc < b->poc; });
        std::sort(lt.begin(), lt.end(),
                  [](const DpbEntry* a, const DpbEntry* b) { return a->long_term_frame_idx < b->long_term_frame_idx; });
        std::vector<const DpbEntry*> list0, list1;
        list0.insert(list0.end(), before.begin(), before.end());
        list0.insert(list0.end(), after.begin(), after.end());
        list0.insert(list0.end(), lt.begin(), lt.end());
        list1.insert(list1.end(), after.begin(), after.end());
        list1.insert(list1.end(), before.begin(), before.end());
        list1.insert(list1.end(), lt.begin(), lt.end());
        if (list1.size() > 1 && list1 == list0) std::swap(list1[0], list1[1]);
        for (uint32_t i = 0; i < num_l0 && i < list0.size() && i < 32; ++i) l0[i] = to_va_pic(*list0[i]);
        for (uint32_t i = 0; i < num_l1 && i < list1.size() && i < 32; ++i) l1[i] = to_va_pic(*list1[i]);
    }

    void mark_dpb(const Sps& sps, bool idr, bool long_term_ref_flag, bool adaptive_marking,
                  const std::vector<MmcoOp>& mmcos, uint32_t curr_frame_num, int32_t& curr_poc, int surf_idx,
                  VASurfaceID surf) {
        if (idr) {
            clear_dpb();
            session_->ref_surface(surf_idx);
            DpbEntry e{};
            e.surface_idx = surf_idx;
            e.surface = surf;
            e.frame_num = curr_frame_num;
            e.pic_num = int32_t(curr_frame_num);
            e.poc = curr_poc;
            e.long_term = long_term_ref_flag;
            e.long_term_frame_idx = 0;
            max_long_term_frame_idx_ = long_term_ref_flag ? 0 : -1;
            dpb_.push_back(e);
            return;
        }
        bool curr_marked_lt = false;
        uint32_t curr_lt_idx = 0;
        if (adaptive_marking) {
            for (const MmcoOp& m : mmcos) {
                if (m.op == 1) {
                    const int32_t target_pn = int32_t(curr_frame_num) - int32_t(m.val1 + 1);
                    for (auto it = dpb_.begin(); it != dpb_.end(); ++it) {
                        if (!it->long_term && it->pic_num == target_pn) {
                            session_->unref_surface(it->surface_idx);
                            dpb_.erase(it);
                            break;
                        }
                    }
                } else if (m.op == 2) {
                    for (auto it = dpb_.begin(); it != dpb_.end(); ++it) {
                        if (it->long_term && it->long_term_frame_idx == m.val1) {
                            session_->unref_surface(it->surface_idx);
                            dpb_.erase(it);
                            break;
                        }
                    }
                } else if (m.op == 3) {
                    const int32_t target_pn = int32_t(curr_frame_num) - int32_t(m.val1 + 1);
                    for (auto it = dpb_.begin(); it != dpb_.end(); ++it) {
                        if (it->long_term && it->long_term_frame_idx == m.val2) {
                            session_->unref_surface(it->surface_idx);
                            dpb_.erase(it);
                            break;
                        }
                    }
                    for (auto& e : dpb_) {
                        if (!e.long_term && e.pic_num == target_pn) {
                            e.long_term = true;
                            e.long_term_frame_idx = m.val2;
                            break;
                        }
                    }
                } else if (m.op == 4) {
                    max_long_term_frame_idx_ = int32_t(m.val1) - 1;
                    for (auto it = dpb_.begin(); it != dpb_.end();) {
                        if (it->long_term && int32_t(it->long_term_frame_idx) > max_long_term_frame_idx_) {
                            session_->unref_surface(it->surface_idx);
                            it = dpb_.erase(it);
                        } else {
                            ++it;
                        }
                    }
                } else if (m.op == 5) {
                    clear_dpb();
                    max_long_term_frame_idx_ = -1;
                    prev_poc_msb_ = 0;
                    prev_poc_lsb_ = 0;
                    prev_frame_num_ = 0;
                    prev_frame_num_offset_ = 0;
                    curr_poc = 0;
                } else if (m.op == 6) {
                    for (auto it = dpb_.begin(); it != dpb_.end(); ++it) {
                        if (it->long_term && it->long_term_frame_idx == m.val1) {
                            session_->unref_surface(it->surface_idx);
                            dpb_.erase(it);
                            break;
                        }
                    }
                    curr_marked_lt = true;
                    curr_lt_idx = m.val1;
                }
            }
        } else {
            const size_t max_refs = std::max<size_t>(1, sps.max_num_ref_frames);
            while (dpb_.size() >= max_refs) {
                auto oldest = dpb_.end();
                for (auto it = dpb_.begin(); it != dpb_.end(); ++it) {
                    if (!it->long_term && (oldest == dpb_.end() || it->pic_num < oldest->pic_num)) {
                        oldest = it;
                    }
                }
                if (oldest == dpb_.end()) oldest = dpb_.begin();
                session_->unref_surface(oldest->surface_idx);
                dpb_.erase(oldest);
            }
        }
        session_->ref_surface(surf_idx);
        DpbEntry e{};
        e.surface_idx = surf_idx;
        e.surface = surf;
        e.frame_num = curr_frame_num;
        e.pic_num = int32_t(curr_frame_num);
        e.poc = curr_poc;
        e.long_term = curr_marked_lt;
        e.long_term_frame_idx = curr_lt_idx;
        dpb_.push_back(e);
    }

    bool decode_slices(const std::vector<std::span<const uint8_t>>& slice_nals, DecodedUnit& out, std::string* err) {
        struct ParsedSlice {
            VASliceParameterBufferH264 sp{};
            std::span<const uint8_t> nal;
        };
        std::vector<ParsedSlice> slices;
        const Sps* sps = nullptr;
        const Pps* pps = nullptr;
        uint32_t pic_frame_num = 0;
        int32_t pic_poc = 0;
        uint32_t pic_nal_ref_idc = 0;
        bool pic_idr = false;
        bool pic_lt_ref_flag = false;
        bool pic_adaptive_marking = false;
        std::vector<MmcoOp> pic_mmcos;

        for (size_t s_idx = 0; s_idx < slice_nals.size(); ++s_idx) {
            const auto nal = slice_nals[s_idx];
            const uint32_t nal_ref_idc = (nal[0] >> 5) & 3u;
            const uint32_t nal_unit_type = nal[0] & 0x1fu;
            const bool idr = (nal_unit_type == 5);
            BitReader r(nal.subspan(1), true, 1024);

            const uint32_t first_mb_in_slice = r.ue();
            const uint32_t slice_type = r.ue() % 5;
            const uint32_t pps_id = r.ue();
            if (pps_id >= 256 || !pps_[pps_id].valid) {
                if (err) *err = "H.264 slice references invalid PPS";
                return false;
            }
            const Pps& cur_pps = pps_[pps_id];
            const Sps& cur_sps = sps_[cur_pps.seq_parameter_set_id];
            if (!cur_sps.valid) {
                if (err) *err = "H.264 slice references invalid SPS";
                return false;
            }
            if (cur_sps.separate_colour_plane_flag) r.u(2);
            const uint32_t frame_num = r.u(cur_sps.log2_max_frame_num_minus4 + 4);
            bool field_pic_flag = false;
            if (!cur_sps.frame_mbs_only_flag) {
                field_pic_flag = r.flag();
                if (field_pic_flag) r.flag();  // bottom_field_flag
            }
            if (idr) r.ue();  // idr_pic_id
            uint32_t poc_lsb = 0;
            int32_t delta_poc_bottom = 0, delta_poc0 = 0;
            if (cur_sps.pic_order_cnt_type == 0) {
                poc_lsb = r.u(cur_sps.log2_max_pic_order_cnt_lsb_minus4 + 4);
                if (cur_pps.bottom_field_pic_order_in_frame_present_flag && !field_pic_flag) {
                    delta_poc_bottom = r.se();
                }
            } else if (cur_sps.pic_order_cnt_type == 1 && !cur_sps.delta_pic_order_always_zero_flag) {
                delta_poc0 = r.se();
                if (cur_pps.bottom_field_pic_order_in_frame_present_flag && !field_pic_flag) r.se();
            }
            if (cur_pps.redundant_pic_cnt_present_flag) r.ue();
            uint8_t direct_spatial_mv_pred_flag = 0;
            if (slice_type == 1) direct_spatial_mv_pred_flag = uint8_t(r.flag() ? 1 : 0);

            uint32_t num_l0 = cur_pps.num_ref_idx_l0_default_active_minus1;
            uint32_t num_l1 = cur_pps.num_ref_idx_l1_default_active_minus1;
            if (slice_type == 0 || slice_type == 3 || slice_type == 1) {
                if (r.flag()) {
                    num_l0 = r.ue();
                    if (slice_type == 1) num_l1 = r.ue();
                }
            }
            if (num_l0 > 31 || num_l1 > 31) {
                if (err) *err = "H.264 num_ref_idx_active_minus1 out of range";
                return false;
            }

            std::vector<ModOp> mods_l0;
            if (slice_type != 2 && slice_type != 4) {
                if (r.flag()) {
                    for (int k = 0; k < 33; ++k) {
                        ModOp m{};
                        m.idc = r.ue();
                        if (m.idc == 3) break;
                        if (m.idc > 3) return false;
                        m.val = r.ue();
                        mods_l0.push_back(m);
                    }
                }
            }
            if (slice_type == 1) {
                if (r.flag()) {
                    for (int k = 0; k < 33; ++k) {
                        const uint32_t idc = r.ue();
                        if (idc == 3) break;
                        if (idc > 3) return false;
                        r.ue();
                    }
                }
            }

            VASliceParameterBufferH264 sp{};
            sp.slice_data_size = uint32_t(nal.size());
            sp.slice_data_offset = 0;
            sp.slice_data_flag = VA_SLICE_DATA_FLAG_ALL;
            sp.first_mb_in_slice = uint16_t(first_mb_in_slice);
            sp.slice_type = uint8_t(slice_type);
            sp.direct_spatial_mv_pred_flag = direct_spatial_mv_pred_flag;
            sp.num_ref_idx_l0_active_minus1 = (slice_type == 2 || slice_type == 4) ? 0 : uint8_t(num_l0);
            sp.num_ref_idx_l1_active_minus1 = (slice_type == 1) ? uint8_t(num_l1) : 0;

            if ((cur_pps.weighted_pred_flag && (slice_type == 0 || slice_type == 3)) ||
                (cur_pps.weighted_bipred_idc == 1 && slice_type == 1)) {
                sp.luma_log2_weight_denom = uint8_t(r.ue());
                if (cur_sps.chroma_format_idc != 0 && !cur_sps.separate_colour_plane_flag) {
                    sp.chroma_log2_weight_denom = uint8_t(r.ue());
                }
                const int16_t def_luma = int16_t(1 << sp.luma_log2_weight_denom);
                const int16_t def_chroma = int16_t(1 << sp.chroma_log2_weight_denom);
                sp.luma_weight_l0_flag = 1;
                sp.chroma_weight_l0_flag = 1;
                for (uint32_t i = 0; i <= num_l0; ++i) {
                    if (r.flag()) {
                        sp.luma_weight_l0[i] = int16_t(r.se());
                        sp.luma_offset_l0[i] = int16_t(r.se());
                    } else {
                        sp.luma_weight_l0[i] = def_luma;
                    }
                    if (cur_sps.chroma_format_idc != 0 && !cur_sps.separate_colour_plane_flag) {
                        if (r.flag()) {
                            for (int j = 0; j < 2; ++j) {
                                sp.chroma_weight_l0[i][j] = int16_t(r.se());
                                sp.chroma_offset_l0[i][j] = int16_t(r.se());
                            }
                        } else {
                            sp.chroma_weight_l0[i][0] = sp.chroma_weight_l0[i][1] = def_chroma;
                        }
                    }
                }
                if (slice_type == 1) {
                    sp.luma_weight_l1_flag = 1;
                    sp.chroma_weight_l1_flag = 1;
                    for (uint32_t i = 0; i <= num_l1; ++i) {
                        if (r.flag()) {
                            sp.luma_weight_l1[i] = int16_t(r.se());
                            sp.luma_offset_l1[i] = int16_t(r.se());
                        } else {
                            sp.luma_weight_l1[i] = def_luma;
                        }
                        if (cur_sps.chroma_format_idc != 0 && !cur_sps.separate_colour_plane_flag) {
                            if (r.flag()) {
                                for (int j = 0; j < 2; ++j) {
                                    sp.chroma_weight_l1[i][j] = int16_t(r.se());
                                    sp.chroma_offset_l1[i][j] = int16_t(r.se());
                                }
                            } else {
                                sp.chroma_weight_l1[i][0] = sp.chroma_weight_l1[i][1] = def_chroma;
                            }
                        }
                    }
                }
            }

            bool lt_ref_flag = false;
            bool adaptive_marking = false;
            std::vector<MmcoOp> mmcos;
            if (nal_ref_idc != 0) {
                if (idr) {
                    r.flag();  // no_output_of_prior_pics_flag
                    lt_ref_flag = r.flag();
                } else {
                    adaptive_marking = r.flag();
                    if (adaptive_marking) {
                        for (int k = 0; k < 32; ++k) {
                            MmcoOp m{};
                            m.op = r.ue();
                            if (m.op == 0) break;
                            if (m.op > 6) return false;
                            if (m.op == 1 || m.op == 3) m.val1 = r.ue();
                            if (m.op == 2) m.val1 = r.ue();
                            if (m.op == 3) m.val2 = r.ue();
                            if (m.op == 6 || m.op == 4) m.val1 = r.ue();
                            mmcos.push_back(m);
                        }
                    }
                }
            }

            if (cur_pps.entropy_coding_mode_flag && slice_type != 2 && slice_type != 4) {
                sp.cabac_init_idc = uint8_t(r.ue());
            }
            sp.slice_qp_delta = int8_t(r.se());
            if (slice_type == 3 || slice_type == 4) {
                if (slice_type == 3) r.flag();
                r.se();
            }
            if (cur_pps.deblocking_filter_control_present_flag) {
                sp.disable_deblocking_filter_idc = uint8_t(r.ue());
                if (sp.disable_deblocking_filter_idc != 1) {
                    sp.slice_alpha_c0_offset_div2 = int8_t(r.se());
                    sp.slice_beta_offset_div2 = int8_t(r.se());
                }
            }
            if (!r.ok()) {
                if (err) *err = "invalid H.264 slice header";
                return false;
            }
            sp.slice_data_bit_offset = uint16_t(8 + r.bit_pos());

            if (s_idx == 0) {
                sps = &cur_sps;
                pps = &cur_pps;
                pic_frame_num = frame_num;
                pic_nal_ref_idc = nal_ref_idc;
                pic_idr = idr;
                pic_lt_ref_flag = lt_ref_flag;
                pic_adaptive_marking = adaptive_marking;
                pic_mmcos = std::move(mmcos);
                pic_poc = compute_poc(*sps, idr, nal_ref_idc, frame_num, poc_lsb, delta_poc_bottom, delta_poc0);
                if (idr) clear_dpb();
                update_pic_nums(frame_num, 1u << (sps->log2_max_frame_num_minus4 + 4));
            }

            if (slice_type == 2 || slice_type == 4) {
                for (auto& ref : sp.RefPicList0) ref = invalid_pic();
                for (auto& ref : sp.RefPicList1) ref = invalid_pic();
            } else if (slice_type == 0 || slice_type == 3) {
                build_ref_list_p(pic_frame_num, 1u << (sps->log2_max_frame_num_minus4 + 4), num_l0 + 1, mods_l0,
                                 sp.RefPicList0);
                for (auto& ref : sp.RefPicList1) ref = invalid_pic();
            } else {
                build_ref_lists_b(pic_poc, num_l0 + 1, num_l1 + 1, sp.RefPicList0, sp.RefPicList1);
            }
            slices.push_back(ParsedSlice{sp, nal});
        }

        VASurfaceID target = VA_INVALID_SURFACE;
        const int surf_idx = session_->acquire_surface(sps->coded_width, sps->coded_height, target, err);
        if (surf_idx < 0) return false;

        VAPictureParameterBufferH264 pp{};
        pp.CurrPic.picture_id = target;
        pp.CurrPic.frame_idx = pic_frame_num;
        pp.CurrPic.flags = 0;
        pp.CurrPic.TopFieldOrderCnt = pic_poc;
        pp.CurrPic.BottomFieldOrderCnt = pic_poc;
        for (size_t i = 0; i < 16; ++i) {
            pp.ReferenceFrames[i] = (i < dpb_.size()) ? to_va_pic(dpb_[i]) : invalid_pic();
        }
        pp.picture_width_in_mbs_minus1 = uint16_t(sps->pic_width_in_mbs_minus1);
        pp.picture_height_in_mbs_minus1 =
            uint16_t((sps->pic_height_in_map_units_minus1 + 1) * (sps->frame_mbs_only_flag ? 1u : 2u) - 1);
        pp.bit_depth_luma_minus8 = uint8_t(sps->bit_depth_luma_minus8);
        pp.bit_depth_chroma_minus8 = uint8_t(sps->bit_depth_chroma_minus8);
        pp.num_ref_frames = uint8_t(sps->max_num_ref_frames);
        pp.seq_fields.bits.chroma_format_idc = sps->chroma_format_idc;
        pp.seq_fields.bits.residual_colour_transform_flag = sps->separate_colour_plane_flag;
        pp.seq_fields.bits.gaps_in_frame_num_value_allowed_flag = sps->gaps_in_frame_num_value_allowed_flag;
        pp.seq_fields.bits.frame_mbs_only_flag = sps->frame_mbs_only_flag;
        pp.seq_fields.bits.mb_adaptive_frame_field_flag = sps->mb_adaptive_frame_field_flag;
        pp.seq_fields.bits.direct_8x8_inference_flag = sps->direct_8x8_inference_flag;
        pp.seq_fields.bits.MinLumaBiPredSize8x8 = (sps->level_idc >= 31) ? 1 : 0;
        pp.seq_fields.bits.log2_max_frame_num_minus4 = sps->log2_max_frame_num_minus4;
        pp.seq_fields.bits.pic_order_cnt_type = sps->pic_order_cnt_type;
        pp.seq_fields.bits.log2_max_pic_order_cnt_lsb_minus4 = sps->log2_max_pic_order_cnt_lsb_minus4;
        pp.seq_fields.bits.delta_pic_order_always_zero_flag = sps->delta_pic_order_always_zero_flag;
        pp.pic_init_qp_minus26 = int8_t(pps->pic_init_qp_minus26);
        pp.pic_init_qs_minus26 = int8_t(pps->pic_init_qs_minus26);
        pp.chroma_qp_index_offset = int8_t(pps->chroma_qp_index_offset);
        pp.second_chroma_qp_index_offset = int8_t(pps->second_chroma_qp_index_offset);
        pp.pic_fields.bits.entropy_coding_mode_flag = pps->entropy_coding_mode_flag;
        pp.pic_fields.bits.weighted_pred_flag = pps->weighted_pred_flag;
        pp.pic_fields.bits.weighted_bipred_idc = pps->weighted_bipred_idc;
        pp.pic_fields.bits.transform_8x8_mode_flag = pps->transform_8x8_mode_flag;
        pp.pic_fields.bits.field_pic_flag = 0;
        pp.pic_fields.bits.constrained_intra_pred_flag = pps->constrained_intra_pred_flag;
        pp.pic_fields.bits.pic_order_present_flag = pps->bottom_field_pic_order_in_frame_present_flag;
        pp.pic_fields.bits.deblocking_filter_control_present_flag = pps->deblocking_filter_control_present_flag;
        pp.pic_fields.bits.redundant_pic_cnt_present_flag = pps->redundant_pic_cnt_present_flag;
        pp.pic_fields.bits.reference_pic_flag = (pic_nal_ref_idc != 0) ? 1 : 0;
        pp.frame_num = uint16_t(pic_frame_num);

        VAIQMatrixBufferH264 iq{};
        std::memcpy(iq.ScalingList4x4, pps->scaling_list_4x4, sizeof(iq.ScalingList4x4));
        std::memcpy(iq.ScalingList8x8, pps->scaling_list_8x8, sizeof(iq.ScalingList8x8));

        const bool ok = session_->submit_picture(
            target,
            [&](BufferList& bufs, std::string* berr) {
                if (!bufs.add(VAPictureParameterBufferType, pp, berr)) return false;
                if (!bufs.add(VAIQMatrixBufferType, iq, berr)) return false;
                for (const auto& sl : slices) {
                    if (!bufs.add(VASliceParameterBufferType, sl.sp, berr)) return false;
                    if (!bufs.add(VASliceDataBufferType, sl.nal.data(), sl.nal.size(), berr)) return false;
                }
                return true;
            },
            err);
        if (!ok) return false;

        if (pic_nal_ref_idc != 0) {
            mark_dpb(*sps, pic_idr, pic_lt_ref_flag, pic_adaptive_marking, pic_mmcos, pic_frame_num, pic_poc, surf_idx,
                     target);
        }

        out.has_output = true;
        out.surface = target;
        out.surface_idx = surf_idx;
        out.coded_width = sps->coded_width;
        out.coded_height = sps->coded_height;
        out.vis_x = sps->vis_x;
        out.vis_y = sps->vis_y;
        out.vis_w = sps->vis_w;
        out.vis_h = sps->vis_h;
        return true;
    }

    DecodeSession* session_ = nullptr;
    std::array<Sps, 32> sps_{};
    std::array<Pps, 256> pps_{};
    std::vector<DpbEntry> dpb_;
    int32_t prev_poc_msb_ = 0;
    uint32_t prev_poc_lsb_ = 0;
    uint32_t prev_frame_num_ = 0;
    uint32_t prev_frame_num_offset_ = 0;
    int32_t max_long_term_frame_idx_ = -1;
};

}  // namespace

std::unique_ptr<DecodeCodec> make_h264_decoder() { return std::make_unique<H264Decoder>(); }

}  // namespace brovideo::vaapi
