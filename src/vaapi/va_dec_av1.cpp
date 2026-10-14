// AV1 OBU bitstream parser and VA-API VLD buffer builder.
#include "vaapi/va_bitreader.h"
#include "vaapi/va_decoder.h"

#include <va/va_dec_av1.h>

#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif

#include <algorithm>
#include <array>
#include <cstring>
#include <vector>

namespace brovideo::vaapi {
namespace {

uint32_t tile_log2(uint32_t blk_size, uint32_t target) {
    uint32_t k = 0;
    while ((blk_size << k) < target) ++k;
    return k;
}

int32_t get_relative_dist(int32_t a, int32_t b, bool enable_order_hint, uint32_t order_hint_bits) {
    if (!enable_order_hint || order_hint_bits == 0) return 0;
    int32_t diff = a - b;
    const int32_t m = 1 << (order_hint_bits - 1);
    return (diff & (m - 1)) - (diff & m);
}

struct Av1Seq {
    bool valid = false, still_picture = false, reduced_still_picture_header = false;
    bool timing_info_present_flag = false, equal_picture_interval = false, decoder_model_info_present_flag = false;
    bool initial_display_delay_present_flag = false, frame_id_numbers_present_flag = false;
    bool use_128x128_superblock = false, enable_filter_intra = false, enable_intra_edge_filter = false;
    bool enable_interintra_compound = false, enable_masked_compound = false, enable_warped_motion = false;
    bool enable_dual_filter = false, enable_order_hint = false, enable_jnt_comp = false, enable_ref_frame_mvs = false;
    bool enable_superres = false, enable_cdef = false, enable_restoration = false, mono_chrome = false;
    bool color_range = false, subsampling_x = true, subsampling_y = true, separate_uv_delta_q = false;
    bool film_grain_params_present = false;
    uint8_t seq_profile = 0, matrix_coefficients = 2, chroma_sample_position = 0;
    uint32_t buffer_delay_length_minus_1 = 0, buffer_removal_time_length_minus_1 = 0;
    uint32_t frame_presentation_time_length_minus_1 = 0, operating_points_cnt_minus_1 = 0;
    uint32_t operating_point_idc[32]{};
    bool decoder_model_present_for_this_op[32]{};
    uint32_t frame_width_bits_minus_1 = 0, frame_height_bits_minus_1 = 0;
    uint32_t max_frame_width_minus_1 = 0, max_frame_height_minus_1 = 0;
    uint32_t delta_frame_id_length_minus_2 = 0, additional_frame_id_length_minus_1 = 0;
    uint32_t seq_force_screen_content_tools = 0, seq_force_integer_mv = 0, order_hint_bits_minus_1 = 0, bit_depth = 8;
};

struct RefSlot {
    bool valid = false;
    int surface_idx = -1;
    VASurfaceID surface = VA_INVALID_SURFACE;
    uint32_t upscaled_width = 0, frame_width = 0, frame_height = 0, render_width = 0, render_height = 0;
    uint8_t order_hint = 0;
    int8_t ref_deltas[8] = {1, 0, 0, 0, -1, 0, -1, -1};
    int8_t mode_deltas[2] = {0, 0};
};

class Av1Decoder final : public DecodeCodec {
public:
    ~Av1Decoder() override { clear_refs(); }
    void reset() override { clear_refs(); have_frame_header_ = false; }

    bool decode_packet(std::span<const uint8_t> packet, DecodeSession& session, DecodedUnit& out,
                       std::string* err) override {
        session_ = &session;
        out = DecodedUnit{};
        if (packet.empty()) {
            if (err) *err = "empty AV1 packet";
            return false;
        }

        have_frame_header_ = false;
        tiles_.clear();
        size_t pos = 0;
        while (pos < packet.size()) {
            const uint8_t hdr = packet[pos++];
            if (hdr & 0x80) {
                if (err) *err = "AV1 OBU forbidden bit is set";
                return false;
            }
            const uint8_t obu_type = (hdr >> 3) & 0x0fu;
            const bool has_ext = (hdr & 0x04) != 0;
            const bool has_size = (hdr & 0x02) != 0;
            if (has_ext) {
                if (pos >= packet.size()) return false;
                ++pos;
            }
            uint64_t obu_size = 0;
            if (has_size) {
                if (!read_obu_leb128(packet, pos, obu_size)) {
                    if (err) *err = "invalid AV1 OBU LEB128 size";
                    return false;
                }
            } else {
                obu_size = packet.size() - pos;
            }
            if (obu_size > packet.size() - pos) {
                if (err) *err = "AV1 OBU size exceeds packet";
                return false;
            }
            const size_t payload_offset = pos;
            const std::span<const uint8_t> payload = packet.subspan(pos, size_t(obu_size));
            pos += size_t(obu_size);

            if (obu_type == 1) {  // OBU_SEQUENCE_HEADER
                if (!parse_sequence_header(payload, err)) return false;
            } else if (obu_type == 3 || obu_type == 7) {  // OBU_FRAME_HEADER / REDUNDANT_FRAME_HEADER
                if (!have_frame_header_) {
                    BitReader r(payload, false);
                    bool shown_existing = false;
                    if (!parse_uncompressed_header(r, shown_existing, out, err)) return false;
                    if (shown_existing) return true;
                    have_frame_header_ = true;
                }
            } else if (obu_type == 4) {  // OBU_TILE_GROUP
                if (!have_frame_header_) {
                    if (err) *err = "AV1 tile group without frame header";
                    return false;
                }
                if (!parse_tile_group(payload, payload_offset, err)) return false;
            } else if (obu_type == 6) {  // OBU_FRAME
                BitReader r(payload, false);
                bool shown_existing = false;
                if (!parse_uncompressed_header(r, shown_existing, out, err)) return false;
                if (shown_existing) return true;
                r.align_byte();
                if (!r.ok() || r.byte_pos() > payload.size()) {
                    if (err) *err = "invalid AV1 OBU_FRAME header alignment";
                    return false;
                }
                have_frame_header_ = true;
                const size_t tg_off = r.byte_pos();
                if (!parse_tile_group(payload.subspan(tg_off), payload_offset + tg_off, err)) return false;
            }

            if (have_frame_header_ && !tiles_.empty() &&
                tiles_.back().tg_end == uint16_t(uint32_t(pp_.tile_cols) * pp_.tile_rows - 1)) {
                if (!submit_current_frame(packet, out, err)) return false;
                have_frame_header_ = false;
                tiles_.clear();
            }
        }
        if (have_frame_header_ && !tiles_.empty()) {
            if (!submit_current_frame(packet, out, err)) return false;
            have_frame_header_ = false;
            tiles_.clear();
        }
        return true;
    }

private:
    void clear_refs() {
        if (session_) {
            for (auto& s : ref_slots_) {
                if (s.valid && s.surface_idx >= 0) session_->unref_surface(s.surface_idx);
                s = RefSlot{};
            }
        }
    }

    bool parse_sequence_header(std::span<const uint8_t> payload, std::string* err) {
        BitReader r(payload, false);
        Av1Seq s{};
        s.seq_profile = uint8_t(r.u(3));
        s.still_picture = r.flag();
        s.reduced_still_picture_header = r.flag();
        if (s.reduced_still_picture_header) {
            r.u(5);  // seq_level_idx[0]
        } else {
            s.timing_info_present_flag = r.flag();
            if (s.timing_info_present_flag) {
                r.u(32);  // num_units_in_display_tick
                r.u(32);  // time_scale
                s.equal_picture_interval = r.flag();
                if (s.equal_picture_interval) r.uvlc();
                s.decoder_model_info_present_flag = r.flag();
                if (s.decoder_model_info_present_flag) {
                    s.buffer_delay_length_minus_1 = r.u(5);
                    r.u(32);  // num_units_in_decoding_tick
                    s.buffer_removal_time_length_minus_1 = r.u(5);
                    s.frame_presentation_time_length_minus_1 = r.u(5);
                }
            }
            s.initial_display_delay_present_flag = r.flag();
            s.operating_points_cnt_minus_1 = r.u(5);
            for (uint32_t i = 0; i <= s.operating_points_cnt_minus_1; ++i) {
                s.operating_point_idc[i] = r.u(12);
                const uint32_t level = r.u(5);
                if (level > 7) r.flag();
                if (s.decoder_model_info_present_flag) {
                    s.decoder_model_present_for_this_op[i] = r.flag();
                    if (s.decoder_model_present_for_this_op[i]) {
                        r.u(s.buffer_delay_length_minus_1 + 1);
                        r.u(s.buffer_delay_length_minus_1 + 1);
                        r.flag();
                    }
                }
                if (s.initial_display_delay_present_flag && r.flag()) r.u(4);
            }
        }
        s.frame_width_bits_minus_1 = r.u(4);
        s.frame_height_bits_minus_1 = r.u(4);
        s.max_frame_width_minus_1 = r.u(s.frame_width_bits_minus_1 + 1);
        s.max_frame_height_minus_1 = r.u(s.frame_height_bits_minus_1 + 1);
        s.frame_id_numbers_present_flag = s.reduced_still_picture_header ? false : r.flag();
        if (s.frame_id_numbers_present_flag) {
            s.delta_frame_id_length_minus_2 = r.u(4);
            s.additional_frame_id_length_minus_1 = r.u(3);
        }
        s.use_128x128_superblock = r.flag();
        s.enable_filter_intra = r.flag();
        s.enable_intra_edge_filter = r.flag();
        if (s.reduced_still_picture_header) {
            s.seq_force_screen_content_tools = 2;
            s.seq_force_integer_mv = 2;
        } else {
            s.enable_interintra_compound = r.flag();
            s.enable_masked_compound = r.flag();
            s.enable_warped_motion = r.flag();
            s.enable_dual_filter = r.flag();
            s.enable_order_hint = r.flag();
            if (s.enable_order_hint) {
                s.enable_jnt_comp = r.flag();
                s.enable_ref_frame_mvs = r.flag();
            }
            s.seq_force_screen_content_tools = r.flag() ? 2u : r.u(1);
            if (s.seq_force_screen_content_tools > 0) {
                s.seq_force_integer_mv = r.flag() ? 2u : r.u(1);
            } else {
                s.seq_force_integer_mv = 2;
            }
            if (s.enable_order_hint) s.order_hint_bits_minus_1 = r.u(3);
        }
        s.enable_superres = r.flag();
        s.enable_cdef = r.flag();
        s.enable_restoration = r.flag();

        // color_config()
        const bool high_bitdepth = r.flag();
        if (s.seq_profile == 2 && high_bitdepth) {
            s.bit_depth = r.flag() ? 12 : 10;
        } else {
            s.bit_depth = high_bitdepth ? 10 : 8;
        }
        s.mono_chrome = (s.seq_profile == 1) ? false : r.flag();
        uint8_t cp = 2, tc = 2, mc = 2;
        if (r.flag()) {
            cp = uint8_t(r.u(8));
            tc = uint8_t(r.u(8));
            mc = uint8_t(r.u(8));
        }
        s.matrix_coefficients = mc;
        if (s.mono_chrome) {
            s.color_range = r.flag();
            s.subsampling_x = true;
            s.subsampling_y = true;
            s.chroma_sample_position = 0;
            s.separate_uv_delta_q = false;
        } else if (cp == 1 && tc == 13 && mc == 0) {
            s.color_range = true;
            s.subsampling_x = false;
            s.subsampling_y = false;
            s.separate_uv_delta_q = r.flag();
        } else {
            s.color_range = r.flag();
            if (s.seq_profile == 0) {
                s.subsampling_x = true;
                s.subsampling_y = true;
            } else if (s.seq_profile == 1) {
                s.subsampling_x = false;
                s.subsampling_y = false;
            } else {
                if (s.bit_depth == 12) {
                    s.subsampling_x = r.flag();
                    s.subsampling_y = s.subsampling_x ? r.flag() : false;
                } else {
                    s.subsampling_x = true;
                    s.subsampling_y = false;
                }
            }
            if (s.subsampling_x && s.subsampling_y) s.chroma_sample_position = uint8_t(r.u(2));
            s.separate_uv_delta_q = r.flag();
        }
        s.film_grain_params_present = r.flag();
        if (!r.ok()) {
            if (err) *err = "truncated AV1 sequence header";
            return false;
        }
        s.valid = true;
        seq_ = s;
        return true;
    }

    int32_t read_delta_q(BitReader& r) { return r.flag() ? r.su(7) : 0; }

    bool parse_uncompressed_header(BitReader& r, bool& shown_existing, DecodedUnit& out, std::string* err) {
        shown_existing = false;
        if (!seq_.valid) {
            if (err) *err = "AV1 frame header before sequence header";
            return false;
        }
        pp_ = VADecPictureParameterBufferAV1{};
        pp_.profile = seq_.seq_profile;
        pp_.order_hint_bits_minus_1 = seq_.enable_order_hint ? uint8_t(seq_.order_hint_bits_minus_1) : 0;
        pp_.bit_depth_idx = (seq_.bit_depth == 12) ? 2 : (seq_.bit_depth == 10 ? 1 : 0);
        pp_.matrix_coefficients = seq_.matrix_coefficients;
        pp_.seq_info_fields.fields.still_picture = seq_.still_picture;
        pp_.seq_info_fields.fields.use_128x128_superblock = seq_.use_128x128_superblock;
        pp_.seq_info_fields.fields.enable_filter_intra = seq_.enable_filter_intra;
        pp_.seq_info_fields.fields.enable_intra_edge_filter = seq_.enable_intra_edge_filter;
        pp_.seq_info_fields.fields.enable_interintra_compound = seq_.enable_interintra_compound;
        pp_.seq_info_fields.fields.enable_masked_compound = seq_.enable_masked_compound;
        pp_.seq_info_fields.fields.enable_dual_filter = seq_.enable_dual_filter;
        pp_.seq_info_fields.fields.enable_order_hint = seq_.enable_order_hint;
        pp_.seq_info_fields.fields.enable_jnt_comp = seq_.enable_jnt_comp;
        pp_.seq_info_fields.fields.enable_cdef = seq_.enable_cdef;
        pp_.seq_info_fields.fields.mono_chrome = seq_.mono_chrome;
        pp_.seq_info_fields.fields.color_range = seq_.color_range;
        pp_.seq_info_fields.fields.subsampling_x = seq_.subsampling_x;
        pp_.seq_info_fields.fields.subsampling_y = seq_.subsampling_y;
        pp_.seq_info_fields.fields.chroma_sample_position = seq_.chroma_sample_position;
        pp_.seq_info_fields.fields.film_grain_params_present = seq_.film_grain_params_present;

        const uint32_t id_len =
            seq_.frame_id_numbers_present_flag
                ? (seq_.additional_frame_id_length_minus_1 + seq_.delta_frame_id_length_minus_2 + 3)
                : 0;
        uint32_t frame_type = 0;
        bool is_intra = true;
        bool show_frame = true;
        bool showable_frame = false;
        bool error_resilient = true;

        if (!seq_.reduced_still_picture_header) {
            if (r.flag()) {  // show_existing_frame
                const uint32_t idx = r.u(3);
                if (seq_.decoder_model_info_present_flag && !seq_.equal_picture_interval) {
                    r.u(seq_.frame_presentation_time_length_minus_1 + 1);
                }
                if (seq_.frame_id_numbers_present_flag) r.u(id_len);
                if (!r.ok() || !ref_slots_[idx].valid) {
                    if (err) *err = "AV1 show_existing_frame references invalid slot";
                    return false;
                }
                const RefSlot& slot = ref_slots_[idx];
                out.has_output = true;
                out.surface = slot.surface;
                out.surface_idx = slot.surface_idx;
                out.coded_width = slot.upscaled_width;
                out.coded_height = slot.frame_height;
                out.vis_w = (slot.render_width > 0 && slot.render_width <= slot.upscaled_width)
                                ? slot.render_width
                                : slot.upscaled_width;
                out.vis_h = (slot.render_height > 0 && slot.render_height <= slot.frame_height)
                                ? slot.render_height
                                : slot.frame_height;
                shown_existing = true;
                return true;
            }
            frame_type = r.u(2);
            is_intra = (frame_type == 0 || frame_type == 2);
            show_frame = r.flag();
            if (show_frame && seq_.decoder_model_info_present_flag && !seq_.equal_picture_interval) {
                r.u(seq_.frame_presentation_time_length_minus_1 + 1);
            }
            showable_frame = show_frame ? (frame_type != 0) : r.flag();
            error_resilient = (frame_type == 3 || (frame_type == 0 && show_frame)) ? true : r.flag();
        }

        const bool disable_cdf_update = r.flag();
        const bool allow_screen_content =
            (seq_.seq_force_screen_content_tools == 2) ? r.flag() : (seq_.seq_force_screen_content_tools != 0);
        bool force_integer_mv = is_intra ? allow_screen_content : false;
        if (!is_intra && allow_screen_content) {
            force_integer_mv = (seq_.seq_force_integer_mv == 2) ? r.flag() : (seq_.seq_force_integer_mv != 0);
        }
        if (seq_.frame_id_numbers_present_flag) r.u(id_len);
        const bool size_override =
            (frame_type == 3) ? true : (seq_.reduced_still_picture_header ? false : r.flag());
        const uint8_t order_hint =
            seq_.enable_order_hint ? uint8_t(r.u(seq_.order_hint_bits_minus_1 + 1)) : 0;
        const uint8_t primary_ref = (is_intra || error_resilient) ? 7 : uint8_t(r.u(3));

        if (seq_.decoder_model_info_present_flag && r.flag()) {
            for (uint32_t i = 0; i <= seq_.operating_points_cnt_minus_1; ++i) {
                if (seq_.decoder_model_present_for_this_op[i]) {
                    r.u(seq_.buffer_removal_time_length_minus_1 + 1);
                }
            }
        }

        refresh_frame_flags_ = (frame_type == 3 || (frame_type == 0 && show_frame)) ? 0xffu : uint8_t(r.u(8));
        if (!is_intra || refresh_frame_flags_ != 0xffu) {
            if (error_resilient && seq_.enable_order_hint) {
                for (int i = 0; i < 8; ++i) r.u(seq_.order_hint_bits_minus_1 + 1);
            }
        }

        uint32_t frame_w = seq_.max_frame_width_minus_1 + 1;
        uint32_t frame_h = seq_.max_frame_height_minus_1 + 1;
        uint32_t upscaled_w = frame_w;
        uint32_t render_w = frame_w;
        uint32_t render_h = frame_h;
        bool use_superres = false;
        uint8_t superres_denom = 8;

        auto read_superres = [&] {
            if (seq_.enable_superres && r.flag()) {
                use_superres = true;
                superres_denom = uint8_t(r.u(3) + 9);
                upscaled_w = frame_w;
                frame_w = (upscaled_w * 8 + (superres_denom / 2)) / superres_denom;
            } else {
                use_superres = false;
                superres_denom = 8;
                upscaled_w = frame_w;
            }
        };
        auto read_frame_size = [&] {
            if (size_override) {
                frame_w = r.u(seq_.frame_width_bits_minus_1 + 1) + 1;
                frame_h = r.u(seq_.frame_height_bits_minus_1 + 1) + 1;
            }
            read_superres();
        };
        auto read_render_size = [&] {
            if (r.flag()) {
                render_w = r.u(16) + 1;
                render_h = r.u(16) + 1;
            } else {
                render_w = upscaled_w;
                render_h = frame_h;
            }
        };

        bool allow_intrabc = false;
        bool allow_high_precision_mv = false;
        bool is_motion_mode_switchable = false;
        bool use_ref_frame_mvs = false;
        uint8_t interp_filter = 0;

        if (is_intra) {
            read_frame_size();
            read_render_size();
            if (allow_screen_content && upscaled_w == frame_w) allow_intrabc = r.flag();
        } else {
            bool short_signaling = false;
            if (seq_.enable_order_hint) {
                short_signaling = r.flag();
                if (short_signaling) {
                    const uint8_t last_idx = uint8_t(r.u(3));
                    const uint8_t gold_idx = uint8_t(r.u(3));
                    for (int i = 0; i < 7; ++i) pp_.ref_frame_idx[i] = last_idx;
                    pp_.ref_frame_idx[3] = gold_idx;
                }
            }
            for (int i = 0; i < 7; ++i) {
                if (!short_signaling) pp_.ref_frame_idx[i] = uint8_t(r.u(3));
                if (seq_.frame_id_numbers_present_flag) r.u(seq_.delta_frame_id_length_minus_2 + 2);
            }
            if (size_override && !error_resilient) {
                bool found = false;
                for (int i = 0; i < 7; ++i) {
                    if (r.flag()) {
                        const RefSlot& rs = ref_slots_[pp_.ref_frame_idx[i]];
                        upscaled_w = rs.upscaled_width;
                        frame_w = upscaled_w;
                        frame_h = rs.frame_height;
                        render_w = rs.render_width;
                        render_h = rs.render_height;
                        read_superres();
                        found = true;
                        break;
                    }
                }
                if (!found) {
                    read_frame_size();
                    read_render_size();
                }
            } else {
                read_frame_size();
                read_render_size();
            }
            allow_high_precision_mv = force_integer_mv ? false : r.flag();
            interp_filter = r.flag() ? 4 : uint8_t(r.u(2));
            is_motion_mode_switchable = r.flag();
            use_ref_frame_mvs = (error_resilient || !seq_.enable_ref_frame_mvs) ? false : r.flag();
        }

        const bool disable_frame_end_update_cdf =
            (seq_.reduced_still_picture_header || disable_cdf_update) ? true : r.flag();

        // tile_info()
        const uint32_t mi_cols = 2 * ((frame_w + 7) >> 3);
        const uint32_t mi_rows = 2 * ((frame_h + 7) >> 3);
        const uint32_t sb_cols = seq_.use_128x128_superblock ? ((mi_cols + 31) >> 5) : ((mi_cols + 15) >> 4);
        const uint32_t sb_rows = seq_.use_128x128_superblock ? ((mi_rows + 31) >> 5) : ((mi_rows + 15) >> 4);
        const uint32_t sb_shift = seq_.use_128x128_superblock ? 5 : 4;
        const uint32_t sb_size = sb_shift + 2;
        const uint32_t max_tile_w_sb = 4096 >> sb_size;
        const uint32_t max_tile_area_sb = (4096 * 2304) >> (2 * sb_size);
        const uint32_t min_log2_cols = tile_log2(max_tile_w_sb, sb_cols);
        const uint32_t max_log2_cols = tile_log2(1, std::min(sb_cols, 64u));
        const uint32_t max_log2_rows = tile_log2(1, std::min(sb_rows, 64u));
        const uint32_t min_log2_tiles = std::max(min_log2_cols, tile_log2(max_tile_area_sb, sb_rows * sb_cols));

        const bool uniform_tiles = r.flag();
        uint32_t tile_cols_log2 = min_log2_cols;
        uint32_t tile_rows_log2 = 0;
        uint32_t tile_cols = 1, tile_rows = 1;
        if (uniform_tiles) {
            while (tile_cols_log2 < max_log2_cols && r.flag()) ++tile_cols_log2;
            const uint32_t tile_w_sb = (sb_cols + (1u << tile_cols_log2) - 1) >> tile_cols_log2;
            uint32_t i = 0;
            for (uint32_t start_sb = 0; start_sb < sb_cols && i < 64; start_sb += tile_w_sb, ++i) {
                pp_.width_in_sbs_minus_1[std::min(i, 62u)] = uint16_t(std::min(tile_w_sb, sb_cols - start_sb) - 1);
            }
            tile_cols = i;
            const uint32_t min_log2_r = (min_log2_tiles > tile_cols_log2) ? (min_log2_tiles - tile_cols_log2) : 0;
            tile_rows_log2 = min_log2_r;
            while (tile_rows_log2 < max_log2_rows && r.flag()) ++tile_rows_log2;
            const uint32_t tile_h_sb = (sb_rows + (1u << tile_rows_log2) - 1) >> tile_rows_log2;
            i = 0;
            for (uint32_t start_sb = 0; start_sb < sb_rows && i < 64; start_sb += tile_h_sb, ++i) {
                pp_.height_in_sbs_minus_1[std::min(i, 62u)] = uint16_t(std::min(tile_h_sb, sb_rows - start_sb) - 1);
            }
            tile_rows = i;
        } else {
            uint32_t widest_sb = 1;
            uint32_t start_sb = 0, i = 0;
            for (; start_sb < sb_cols && i < 64; ++i) {
                const uint32_t max_w = std::min(sb_cols - start_sb, max_tile_w_sb);
                const uint32_t w_sb = r.ns(max_w) + 1;
                pp_.width_in_sbs_minus_1[std::min(i, 62u)] = uint16_t(w_sb - 1);
                widest_sb = std::max(widest_sb, w_sb);
                start_sb += w_sb;
            }
            tile_cols = i;
            tile_cols_log2 = tile_log2(1, tile_cols);
            const uint32_t max_area = (min_log2_tiles > 0) ? std::max(max_tile_area_sb >> (min_log2_tiles + 1), 1u)
                                                           : sb_cols * sb_rows;
            const uint32_t max_h_sb = std::max(max_area / widest_sb, 1u);
            start_sb = 0;
            i = 0;
            for (; start_sb < sb_rows && i < 64; ++i) {
                const uint32_t max_h = std::min(sb_rows - start_sb, max_h_sb);
                const uint32_t h_sb = r.ns(max_h) + 1;
                pp_.height_in_sbs_minus_1[std::min(i, 62u)] = uint16_t(h_sb - 1);
                start_sb += h_sb;
            }
            tile_rows = i;
            tile_rows_log2 = tile_log2(1, tile_rows);
        }
        pp_.tile_cols = uint8_t(tile_cols);
        pp_.tile_rows = uint8_t(tile_rows);
        tile_cols_log2_ = tile_cols_log2;
        tile_rows_log2_ = tile_rows_log2;
        if (tile_cols_log2 > 0 || tile_rows_log2 > 0) {
            pp_.context_update_tile_id = uint16_t(r.u(tile_cols_log2 + tile_rows_log2));
            tile_size_bytes_ = r.u(2) + 1;
        } else {
            pp_.context_update_tile_id = 0;
            tile_size_bytes_ = 4;
        }

        // quantization_params()
        pp_.base_qindex = uint8_t(r.u(8));
        pp_.y_dc_delta_q = int8_t(read_delta_q(r));
        if (!seq_.mono_chrome) {
            bool diff_uv_delta = false;
            if (seq_.separate_uv_delta_q) diff_uv_delta = r.flag();
            pp_.u_dc_delta_q = int8_t(read_delta_q(r));
            pp_.u_ac_delta_q = int8_t(read_delta_q(r));
            if (diff_uv_delta) {
                pp_.v_dc_delta_q = int8_t(read_delta_q(r));
                pp_.v_ac_delta_q = int8_t(read_delta_q(r));
            } else {
                pp_.v_dc_delta_q = pp_.u_dc_delta_q;
                pp_.v_ac_delta_q = pp_.u_ac_delta_q;
            }
        }
        if (r.flag()) {  // using_qmatrix
            pp_.qmatrix_fields.bits.using_qmatrix = 1;
            pp_.qmatrix_fields.bits.qm_y = r.u(4);
            pp_.qmatrix_fields.bits.qm_u = r.u(4);
            pp_.qmatrix_fields.bits.qm_v = seq_.separate_uv_delta_q ? r.u(4) : pp_.qmatrix_fields.bits.qm_u;
        }

        // segmentation_params()
        bool seg_enabled = r.flag();
        pp_.seg_info.segment_info_fields.bits.enabled = seg_enabled ? 1 : 0;
        if (seg_enabled) {
            if (primary_ref == 7) {
                pp_.seg_info.segment_info_fields.bits.update_map = 1;
                pp_.seg_info.segment_info_fields.bits.temporal_update = 0;
                pp_.seg_info.segment_info_fields.bits.update_data = 1;
            } else {
                const bool upd_map = r.flag();
                pp_.seg_info.segment_info_fields.bits.update_map = upd_map ? 1 : 0;
                if (upd_map) pp_.seg_info.segment_info_fields.bits.temporal_update = r.flag() ? 1 : 0;
                pp_.seg_info.segment_info_fields.bits.update_data = r.flag() ? 1 : 0;
            }
            if (pp_.seg_info.segment_info_fields.bits.update_data) {
                constexpr uint8_t kBits[8] = {8, 6, 6, 6, 6, 3, 0, 0};
                constexpr bool kSigned[8] = {true, true, true, true, true, false, false, false};
                for (int i = 0; i < 8; ++i) {
                    for (int j = 0; j < 8; ++j) {
                        if (r.flag()) {
                            pp_.seg_info.feature_mask[i] |= uint8_t(1u << j);
                            if (kBits[j] > 0) {
                                pp_.seg_info.feature_data[i][j] =
                                    int16_t(kSigned[j] ? r.su(kBits[j] + 1) : int32_t(r.u(kBits[j])));
                            }
                        }
                    }
                }
            }
        }

        // delta_q_params() and delta_lf_params()
        bool delta_q_present = false;
        if (pp_.base_qindex > 0) delta_q_present = r.flag();
        pp_.mode_control_fields.bits.delta_q_present_flag = delta_q_present ? 1 : 0;
        if (delta_q_present) {
            pp_.mode_control_fields.bits.log2_delta_q_res = r.u(2);
            if (!allow_intrabc) {
                const bool delta_lf = r.flag();
                pp_.mode_control_fields.bits.delta_lf_present_flag = delta_lf ? 1 : 0;
                if (delta_lf) {
                    pp_.mode_control_fields.bits.log2_delta_lf_res = r.u(2);
                    pp_.mode_control_fields.bits.delta_lf_multi = r.flag() ? 1 : 0;
                }
            }
        }

        const bool CodedLossless = (pp_.base_qindex == 0 && pp_.y_dc_delta_q == 0 && pp_.u_dc_delta_q == 0 &&
                                    pp_.u_ac_delta_q == 0 && pp_.v_dc_delta_q == 0 && pp_.v_ac_delta_q == 0);
        const bool AllLossless = CodedLossless && (frame_w == upscaled_w);

        // Initialize loop filter deltas from primary_ref or default.
        static constexpr int8_t kDefRefDeltas[8] = {1, 0, 0, 0, -1, 0, -1, -1};
        if (primary_ref == 7 || !ref_slots_[pp_.ref_frame_idx[primary_ref]].valid) {
            std::memcpy(pp_.ref_deltas, kDefRefDeltas, 8);
            pp_.mode_deltas[0] = pp_.mode_deltas[1] = 0;
        } else {
            const RefSlot& prs = ref_slots_[pp_.ref_frame_idx[primary_ref]];
            std::memcpy(pp_.ref_deltas, prs.ref_deltas, 8);
            std::memcpy(pp_.mode_deltas, prs.mode_deltas, 2);
        }

        // loop_filter_params()
        if (!AllLossless && !allow_intrabc) {
            pp_.filter_level[0] = uint8_t(r.u(6));
            pp_.filter_level[1] = uint8_t(r.u(6));
            if (!seq_.mono_chrome && (pp_.filter_level[0] || pp_.filter_level[1])) {
                pp_.filter_level_u = uint8_t(r.u(6));
                pp_.filter_level_v = uint8_t(r.u(6));
            }
            pp_.loop_filter_info_fields.bits.sharpness_level = uint8_t(r.u(3));
            const bool lf_delta_enabled = r.flag();
            pp_.loop_filter_info_fields.bits.mode_ref_delta_enabled = lf_delta_enabled ? 1 : 0;
            if (lf_delta_enabled) {
                const bool lf_delta_update = r.flag();
                pp_.loop_filter_info_fields.bits.mode_ref_delta_update = lf_delta_update ? 1 : 0;
                if (lf_delta_update) {
                    for (int i = 0; i < 8; ++i) {
                        if (r.flag()) pp_.ref_deltas[i] = int8_t(r.su(7));
                    }
                    for (int i = 0; i < 2; ++i) {
                        if (r.flag()) pp_.mode_deltas[i] = int8_t(r.su(7));
                    }
                }
            }
        }

        // cdef_params()
        if (!CodedLossless && !allow_intrabc && seq_.enable_cdef) {
            pp_.cdef_damping_minus_3 = uint8_t(r.u(2));
            pp_.cdef_bits = uint8_t(r.u(2));
            const int n_cdef = 1 << pp_.cdef_bits;
            for (int i = 0; i < n_cdef; ++i) {
                const uint32_t y_pri = r.u(4);
                const uint32_t y_sec = r.u(2);
                pp_.cdef_y_strengths[i] = uint8_t((y_pri << 2) | (y_sec & 3u));
                if (!seq_.mono_chrome) {
                    const uint32_t uv_pri = r.u(4);
                    const uint32_t uv_sec = r.u(2);
                    pp_.cdef_uv_strengths[i] = uint8_t((uv_pri << 2) | (uv_sec & 3u));
                }
            }
        }

        // lr_params()
        if (!AllLossless && !allow_intrabc && seq_.enable_restoration) {
            constexpr uint16_t kRemapLr[4] = {0, 3, 1, 2};
            const uint16_t lr_y = kRemapLr[r.u(2)];
            uint16_t lr_u = 0, lr_v = 0;
            if (!seq_.mono_chrome) {
                lr_u = kRemapLr[r.u(2)];
                lr_v = kRemapLr[r.u(2)];
            }
            pp_.loop_restoration_fields.bits.yframe_restoration_type = lr_y;
            pp_.loop_restoration_fields.bits.cbframe_restoration_type = lr_u;
            pp_.loop_restoration_fields.bits.crframe_restoration_type = lr_v;
            if (lr_y || lr_u || lr_v) {
                uint32_t lr_shift = r.u(1);
                if (!seq_.use_128x128_superblock && lr_shift) lr_shift += r.u(1);
                pp_.loop_restoration_fields.bits.lr_unit_shift = uint16_t(lr_shift);
                if (!seq_.mono_chrome && (lr_u || lr_v) && seq_.subsampling_x && seq_.subsampling_y) {
                    pp_.loop_restoration_fields.bits.lr_uv_shift = uint16_t(r.u(1));
                }
            }
        }

        // read_tx_mode()
        if (CodedLossless) {
            pp_.mode_control_fields.bits.tx_mode = 0;  // ONLY_4X4
        } else {
            pp_.mode_control_fields.bits.tx_mode = r.flag() ? 2 : 1;  // TX_MODE_SELECT : TX_MODE_LARGEST
        }

        // frame_reference_mode()
        bool reference_select = false;
        if (!is_intra) reference_select = r.flag();
        pp_.mode_control_fields.bits.reference_select = reference_select ? 1 : 0;

        // skip_mode_params()
        bool skip_mode_allowed = false;
        if (!is_intra && reference_select && seq_.enable_order_hint) {
            const uint32_t oh_bits = seq_.order_hint_bits_minus_1 + 1;
            int forward_idx = -1, backward_idx = -1;
            int32_t forward_hint = 0, backward_hint = 0;
            for (int i = 0; i < 7; ++i) {
                const uint8_t ref_hint = ref_slots_[pp_.ref_frame_idx[i]].order_hint;
                const int32_t dist = get_relative_dist(ref_hint, order_hint, true, oh_bits);
                if (dist < 0) {
                    if (forward_idx < 0 || get_relative_dist(ref_hint, forward_hint, true, oh_bits) > 0) {
                        forward_idx = i;
                        forward_hint = ref_hint;
                    }
                } else if (dist > 0) {
                    if (backward_idx < 0 || get_relative_dist(ref_hint, backward_hint, true, oh_bits) < 0) {
                        backward_idx = i;
                        backward_hint = ref_hint;
                    }
                }
            }
            if (forward_idx >= 0 && backward_idx >= 0) {
                skip_mode_allowed = true;
            } else if (forward_idx >= 0) {
                int second_forward_idx = -1;
                for (int i = 0; i < 7; ++i) {
                    const uint8_t ref_hint = ref_slots_[pp_.ref_frame_idx[i]].order_hint;
                    if (get_relative_dist(ref_hint, forward_hint, true, oh_bits) < 0) {
                        if (second_forward_idx < 0 ||
                            get_relative_dist(ref_hint,
                                              ref_slots_[pp_.ref_frame_idx[second_forward_idx]].order_hint, true,
                                              oh_bits) > 0) {
                            second_forward_idx = i;
                        }
                    }
                }
                if (second_forward_idx >= 0) skip_mode_allowed = true;
            }
        }
        pp_.mode_control_fields.bits.skip_mode_present = (skip_mode_allowed && r.flag()) ? 1 : 0;

        const bool allow_warped_motion =
            (!is_intra && !error_resilient && seq_.enable_warped_motion) ? r.flag() : false;
        pp_.mode_control_fields.bits.reduced_tx_set_used = r.flag() ? 1 : 0;

        // global_motion_params()
        for (int i = 0; i < 7; ++i) {
            pp_.wm[i].wmtype = VAAV1TransformationIdentity;
            pp_.wm[i].wmmat[2] = 1 << 16;
            pp_.wm[i].wmmat[5] = 1 << 16;
        }
        if (!is_intra) {
            for (int ref = 1; ref <= 7; ++ref) {
                if (r.flag()) {  // is_global
                    // If global motion is used, skip its syntax or fail cleanly if complex
                    if (err) *err = "AV1 global motion parameters not supported";
                    return false;
                }
            }
        }

        // film_grain_params()
        if (seq_.film_grain_params_present && (show_frame || showable_frame)) {
            if (r.flag()) {  // apply_grain
                if (err) *err = "AV1 film grain synthesis not supported";
                return false;
            }
        }

        if (!r.ok()) {
            if (err) *err = "truncated AV1 frame header";
            return false;
        }

        pp_.frame_width_minus1 = uint16_t(upscaled_w - 1);
        pp_.frame_height_minus1 = uint16_t(frame_h - 1);
        pp_.primary_ref_frame = primary_ref;
        pp_.order_hint = order_hint;
        pp_.superres_scale_denominator = superres_denom;
        pp_.interp_filter = interp_filter;
        pp_.pic_info_fields.bits.frame_type = frame_type;
        pp_.pic_info_fields.bits.show_frame = show_frame ? 1 : 0;
        pp_.pic_info_fields.bits.showable_frame = showable_frame ? 1 : 0;
        pp_.pic_info_fields.bits.error_resilient_mode = error_resilient ? 1 : 0;
        pp_.pic_info_fields.bits.disable_cdf_update = disable_cdf_update ? 1 : 0;
        pp_.pic_info_fields.bits.allow_screen_content_tools = allow_screen_content ? 1 : 0;
        pp_.pic_info_fields.bits.force_integer_mv = force_integer_mv ? 1 : 0;
        pp_.pic_info_fields.bits.allow_intrabc = allow_intrabc ? 1 : 0;
        pp_.pic_info_fields.bits.use_superres = use_superres ? 1 : 0;
        pp_.pic_info_fields.bits.allow_high_precision_mv = allow_high_precision_mv ? 1 : 0;
        pp_.pic_info_fields.bits.is_motion_mode_switchable = is_motion_mode_switchable ? 1 : 0;
        pp_.pic_info_fields.bits.use_ref_frame_mvs = use_ref_frame_mvs ? 1 : 0;
        pp_.pic_info_fields.bits.disable_frame_end_update_cdf = disable_frame_end_update_cdf ? 1 : 0;
        pp_.pic_info_fields.bits.uniform_tile_spacing_flag = uniform_tiles ? 1 : 0;
        pp_.pic_info_fields.bits.allow_warped_motion = allow_warped_motion ? 1 : 0;

        cur_upscaled_w_ = upscaled_w;
        cur_frame_w_ = frame_w;
        cur_frame_h_ = frame_h;
        cur_render_w_ = render_w;
        cur_render_h_ = render_h;
        return true;
    }

    bool parse_tile_group(std::span<const uint8_t> tg_payload, size_t payload_offset_in_pkt, std::string* err) {
        const uint32_t num_tiles = uint32_t(pp_.tile_cols) * pp_.tile_rows;
        if (num_tiles == 0) return false;
        BitReader r(tg_payload, false);
        uint32_t tg_start = 0;
        uint32_t tg_end = num_tiles - 1;
        if (num_tiles > 1 && r.flag()) {
            const uint32_t tile_bits = tile_cols_log2_ + tile_rows_log2_;
            tg_start = r.u(tile_bits);
            tg_end = r.u(tile_bits);
        }
        r.align_byte();
        if (!r.ok() || tg_start > tg_end || tg_end >= num_tiles) {
            if (err) *err = "invalid AV1 tile group header";
            return false;
        }
        size_t pos = r.byte_pos();
        for (uint32_t t = tg_start; t <= tg_end; ++t) {
            size_t tile_size = 0;
            if (t < tg_end) {
                if (pos + tile_size_bytes_ > tg_payload.size()) {
                    if (err) *err = "truncated AV1 tile size";
                    return false;
                }
                uint32_t sz_minus_1 = 0;
                for (uint32_t b = 0; b < tile_size_bytes_; ++b) {
                    sz_minus_1 |= uint32_t(tg_payload[pos + b]) << (8 * b);
                }
                pos += tile_size_bytes_;
                tile_size = size_t(sz_minus_1) + 1;
            } else {
                if (pos > tg_payload.size()) return false;
                tile_size = tg_payload.size() - pos;
            }
            if (tile_size == 0 || pos + tile_size > tg_payload.size()) {
                if (err) *err = "invalid AV1 tile data size";
                return false;
            }
            VASliceParameterBufferAV1 sp{};
            sp.slice_data_size = uint32_t(tile_size);
            sp.slice_data_offset = uint32_t(payload_offset_in_pkt + pos);
            sp.slice_data_flag = VA_SLICE_DATA_FLAG_ALL;
            sp.tile_row = uint16_t(t / pp_.tile_cols);
            sp.tile_column = uint16_t(t % pp_.tile_cols);
            sp.tg_start = uint16_t(tg_start);
            sp.tg_end = uint16_t(tg_end);
            tiles_.push_back(sp);
            pos += tile_size;
        }
        return true;
    }

    bool submit_current_frame(std::span<const uint8_t> packet, DecodedUnit& out, std::string* err) {
        VASurfaceID target = VA_INVALID_SURFACE;
        const int surf_idx = session_->acquire_surface(cur_upscaled_w_, cur_frame_h_, target, err);
        if (surf_idx < 0) return false;

        pp_.current_frame = target;
        pp_.current_display_picture = target;
        for (int i = 0; i < 8; ++i) {
            pp_.ref_frame_map[i] = ref_slots_[i].valid ? ref_slots_[i].surface : VA_INVALID_SURFACE;
        }

        const bool ok = session_->submit_picture(
            target,
            [&](BufferList& bufs, std::string* berr) {
                if (!bufs.add(VAPictureParameterBufferType, pp_, berr)) return false;
                for (const auto& t : tiles_) {
                    if (!bufs.add(VASliceParameterBufferType, t, berr)) return false;
                }
                if (!bufs.add(VASliceDataBufferType, packet.data(), packet.size(), berr)) return false;
                return true;
            },
            err);
        if (!ok) return false;

        for (int i = 0; i < 8; ++i) {
            if (refresh_frame_flags_ & (1u << i)) {
                if (ref_slots_[i].valid && ref_slots_[i].surface_idx >= 0) {
                    session_->unref_surface(ref_slots_[i].surface_idx);
                }
                session_->ref_surface(surf_idx);
                ref_slots_[i].valid = true;
                ref_slots_[i].surface_idx = surf_idx;
                ref_slots_[i].surface = target;
                ref_slots_[i].upscaled_width = cur_upscaled_w_;
                ref_slots_[i].frame_width = cur_frame_w_;
                ref_slots_[i].frame_height = cur_frame_h_;
                ref_slots_[i].render_width = cur_render_w_;
                ref_slots_[i].render_height = cur_render_h_;
                ref_slots_[i].order_hint = pp_.order_hint;
                std::memcpy(ref_slots_[i].ref_deltas, pp_.ref_deltas, 8);
                std::memcpy(ref_slots_[i].mode_deltas, pp_.mode_deltas, 2);
            }
        }

        if (pp_.pic_info_fields.bits.show_frame) {
            out.has_output = true;
            out.surface = target;
            out.surface_idx = surf_idx;
            out.coded_width = cur_upscaled_w_;
            out.coded_height = cur_frame_h_;
            out.vis_x = 0;
            out.vis_y = 0;
            out.vis_w = (cur_render_w_ > 0 && cur_render_w_ <= cur_upscaled_w_) ? cur_render_w_ : cur_upscaled_w_;
            out.vis_h = (cur_render_h_ > 0 && cur_render_h_ <= cur_frame_h_) ? cur_render_h_ : cur_frame_h_;
        }
        return true;
    }

    DecodeSession* session_ = nullptr;
    Av1Seq seq_{};
    VADecPictureParameterBufferAV1 pp_{};
    std::array<RefSlot, 8> ref_slots_{};
    std::vector<VASliceParameterBufferAV1> tiles_;
    bool have_frame_header_ = false;
    uint8_t refresh_frame_flags_ = 0;
    uint32_t tile_cols_log2_ = 0;
    uint32_t tile_rows_log2_ = 0;
    uint32_t tile_size_bytes_ = 4;
    uint32_t cur_upscaled_w_ = 0;
    uint32_t cur_frame_w_ = 0;
    uint32_t cur_frame_h_ = 0;
    uint32_t cur_render_w_ = 0;
    uint32_t cur_render_h_ = 0;
};

}  // namespace

std::unique_ptr<DecodeCodec> make_av1_decoder() { return std::make_unique<Av1Decoder>(); }

}  // namespace brovideo::vaapi
