// H.264 for the VA-API encoder: low-latency I/P, one reference, Annex B with
// SPS/PPS in band on every IDR.
//
// Headers: when the driver accepts packed sequence, picture and slice headers
// we write SPS, PPS and the slice header ourselves. The SPS then carries what
// a low-latency decoder needs: the BT.709 limited-range colour description,
// centre chroma siting, and bitstream_restriction with max_num_reorder_frames
// = 0 and max_dec_frame_buffering = 1, so a decoder outputs each picture as
// soon as it is decoded instead of holding a reorder window.
//
// radeonsi (Mesa 26) parses the packed headers and writes the NAL units
// itself from what it parsed, correcting what its hardware cannot do (it
// clears transform_8x8_mode_flag). All three are needed there: given no
// packed slice header it writes slices with nal_unit_type 0, and no SPS or
// PPS at all. Without packed-header support (other drivers) the driver
// writes every header from the parameter buffers.
#include "vaapi/va_codec.h"

#include <va/va_enc_h264.h>

namespace brovideo::vaapi {
namespace {

constexpr uint32_t kLog2MaxFrameNum = 8;   // frame_num wraps every 256 reference frames
constexpr uint32_t kLog2MaxPocLsb = 8;     // POC = 2 * frame index; its lsb wraps every 128 frames
constexpr int kInitQp = 26;

struct LevelLimit {
    uint8_t idc;
    uint32_t max_mbps;    // macroblocks per second
    uint32_t max_fs;      // macroblocks per frame
    uint32_t max_kbps;    // Main profile MaxBR (High is 1.25x)
};
constexpr LevelLimit kLevels[] = {
    {30, 40500, 1620, 10000},     {31, 108000, 3600, 14000},    {32, 216000, 5120, 20000},
    {40, 245760, 8192, 20000},    {41, 245760, 8192, 50000},    {42, 522240, 8704, 50000},
    {50, 589824, 22080, 135000},  {51, 983040, 36864, 240000},  {52, 2073600, 36864, 240000},
    {60, 4177920, 139264, 240000}, {61, 8355840, 139264, 480000}, {62, 16711680, 139264, 800000},
};

uint8_t pick_level(uint32_t mbs, uint32_t fps, uint32_t kbps, bool high) {
    for (const LevelLimit& l : kLevels) {
        const uint64_t max_kbps = high ? uint64_t(l.max_kbps) * 5 / 4 : l.max_kbps;
        if (mbs <= l.max_fs && uint64_t(mbs) * fps <= l.max_mbps && kbps <= max_kbps) return l.idc;
    }
    return 62;
}

VAPictureH264 invalid_picture() {
    VAPictureH264 p{};
    p.picture_id = VA_INVALID_SURFACE;
    p.flags = VA_PICTURE_H264_INVALID;
    return p;
}

class H264 final : public CodecImpl {
public:
    explicit H264(const StreamParams& sp) : sp_(sp) {
        profile_ = sp.caps.profile;
        high_ = profile_ == VAProfileH264High;
        cabac_ = profile_ != VAProfileH264ConstrainedBaseline;
        width_mbs_ = sp.coded_width / 16;
        height_mbs_ = sp.coded_height / 16;
        crop_right_ = (sp.coded_width - sp.width) / 2;  // in 2-pixel units (4:2:0)
        crop_bottom_ = (sp.coded_height - sp.height) / 2;
        level_ = pick_level(width_mbs_ * height_mbs_, sp.fps, sp.bitrate_bps / 1000, high_);
        const uint32_t want = VA_ENC_PACKED_HEADER_SEQUENCE | VA_ENC_PACKED_HEADER_PICTURE | VA_ENC_PACKED_HEADER_SLICE;
        packed_ = (sp.caps.packed_headers & want) == want ? want : VA_ENC_PACKED_HEADER_NONE;
        debug_log("h264: profile %d level %u, %ux%u MBs, packed headers 0x%x (driver offers 0x%x)", int(profile_),
                  unsigned(level_), width_mbs_, height_mbs_, packed_, sp.caps.packed_headers);
    }

    uint32_t packed_headers() const override { return packed_; }

    bool add_sequence(BufferList& bufs, const PictureParams&, std::string* err) override {
        VAEncSequenceParameterBufferH264 s{};
        s.seq_parameter_set_id = 0;
        s.level_idc = level_;
        // No periodic keyframes from the driver's point of view: every IDR is
        // requested explicitly through the picture parameters.
        s.intra_period = 0;
        s.intra_idr_period = 0;
        s.ip_period = 1;
        s.bits_per_second = sp_.bitrate_bps;
        s.max_num_ref_frames = 1;
        s.picture_width_in_mbs = uint16_t(width_mbs_);
        s.picture_height_in_mbs = uint16_t(height_mbs_);
        s.seq_fields.bits.chroma_format_idc = 1;
        s.seq_fields.bits.frame_mbs_only_flag = 1;
        s.seq_fields.bits.direct_8x8_inference_flag = 1;
        s.seq_fields.bits.log2_max_frame_num_minus4 = kLog2MaxFrameNum - 4;
        s.seq_fields.bits.pic_order_cnt_type = 0;
        s.seq_fields.bits.log2_max_pic_order_cnt_lsb_minus4 = kLog2MaxPocLsb - 4;
        if (crop_right_ || crop_bottom_) {
            s.frame_cropping_flag = 1;
            s.frame_crop_right_offset = crop_right_;
            s.frame_crop_bottom_offset = crop_bottom_;
        }
        s.vui_parameters_present_flag = 1;
        s.vui_fields.bits.timing_info_present_flag = 1;
        s.vui_fields.bits.bitstream_restriction_flag = 1;
        s.vui_fields.bits.log2_max_mv_length_horizontal = 15;
        s.vui_fields.bits.log2_max_mv_length_vertical = 15;
        s.vui_fields.bits.motion_vectors_over_pic_boundaries_flag = 1;
        s.num_units_in_tick = 1;
        s.time_scale = 2 * sp_.fps;
        return bufs.add(VAEncSequenceParameterBufferType, s, err);
    }

    bool add_picture(BufferList& bufs, const PictureParams& pic, std::string* err) override {
        const uint32_t frame_num = uint32_t(pic.index_in_gop % (1u << kLog2MaxFrameNum));
        const int32_t poc = int32_t((2 * pic.index_in_gop) % (1u << 30));
        const uint32_t poc_lsb = uint32_t(2 * pic.index_in_gop) % (1u << kLog2MaxPocLsb);

        VAEncPictureParameterBufferH264 p{};
        p.CurrPic.picture_id = pic.recon;
        p.CurrPic.frame_idx = frame_num;
        p.CurrPic.flags = 0;
        p.CurrPic.TopFieldOrderCnt = poc;
        p.CurrPic.BottomFieldOrderCnt = poc;
        for (auto& r : p.ReferenceFrames) r = invalid_picture();
        VAPictureH264 ref = invalid_picture();
        if (!pic.keyframe) {
            const uint64_t prev = pic.index_in_gop - 1;
            ref.picture_id = pic.ref;
            ref.frame_idx = uint32_t(prev % (1u << kLog2MaxFrameNum));
            ref.flags = VA_PICTURE_H264_SHORT_TERM_REFERENCE;
            ref.TopFieldOrderCnt = int32_t((2 * prev) % (1u << 30));
            ref.BottomFieldOrderCnt = ref.TopFieldOrderCnt;
            p.ReferenceFrames[0] = ref;
        }
        p.coded_buf = pic.coded;
        p.pic_parameter_set_id = 0;
        p.seq_parameter_set_id = 0;
        p.frame_num = uint16_t(frame_num);
        p.pic_init_qp = kInitQp;
        p.num_ref_idx_l0_active_minus1 = 0;
        p.pic_fields.bits.idr_pic_flag = pic.keyframe;
        p.pic_fields.bits.reference_pic_flag = 1;
        p.pic_fields.bits.entropy_coding_mode_flag = cabac_;
        p.pic_fields.bits.transform_8x8_mode_flag = high_;
        p.pic_fields.bits.deblocking_filter_control_present_flag = 1;
        if (!bufs.add(VAEncPictureParameterBufferType, p, err)) return false;

        if (pic.keyframe && (packed_ & VA_ENC_PACKED_HEADER_SEQUENCE)) {
            BitWriter sps;
            write_sps(sps);
            const auto nal = annexb_nal({0x67}, sps.bytes());
            if (!bufs.add_packed(VAEncPackedHeaderSequence, nal, nal.size() * 8, err)) return false;
        }
        if (pic.keyframe && (packed_ & VA_ENC_PACKED_HEADER_PICTURE)) {
            BitWriter pps;
            write_pps(pps);
            const auto nal = annexb_nal({0x68}, pps.bytes());
            if (!bufs.add_packed(VAEncPackedHeaderPicture, nal, nal.size() * 8, err)) return false;
        }

        if (packed_ & VA_ENC_PACKED_HEADER_SLICE) {
            BitWriter sh;
            write_slice_header(sh, pic, frame_num, poc_lsb);
            const uint8_t nal_header = pic.keyframe ? 0x65 : 0x41;  // IDR (ref_idc 3) : non-IDR (ref_idc 2)
            const auto nal = annexb_nal({nal_header}, sh.bytes());
            // The slice header is not byte aligned: the length counts the
            // start code, the NAL header and the header's own bits.
            const size_t emulation = nal.size() - 5 - sh.bytes().size();
            const size_t bits = (5 + emulation) * 8 + sh.bit_count();
            if (!bufs.add_packed(VAEncPackedHeaderSlice, nal, bits, err)) return false;
        }

        VAEncSliceParameterBufferH264 sl{};
        sl.macroblock_address = 0;
        sl.num_macroblocks = width_mbs_ * height_mbs_;
        sl.macroblock_info = VA_INVALID_ID;
        sl.slice_type = pic.keyframe ? 2 : 0;  // I : P
        sl.pic_parameter_set_id = 0;
        sl.idr_pic_id = uint16_t(pic.keyframe_count & 0xffff);
        sl.pic_order_cnt_lsb = uint16_t(poc_lsb);
        sl.num_ref_idx_active_override_flag = 0;
        sl.num_ref_idx_l0_active_minus1 = 0;
        for (auto& r : sl.RefPicList0) r = invalid_picture();
        for (auto& r : sl.RefPicList1) r = invalid_picture();
        if (!pic.keyframe) sl.RefPicList0[0] = ref;
        sl.cabac_init_idc = 0;
        sl.slice_qp_delta = 0;
        sl.disable_deblocking_filter_idc = 0;
        return bufs.add(VAEncSliceParameterBufferType, sl, err);
    }

private:
    void write_sps(BitWriter& b) const {
        const uint32_t profile_idc = high_ ? 100 : profile_ == VAProfileH264Main ? 77 : 66;
        b.u(8, profile_idc);
        const bool cb = profile_ == VAProfileH264ConstrainedBaseline;
        b.flag(cb);  // constraint_set0: Baseline conformance
        b.flag(cb || profile_ == VAProfileH264Main);  // constraint_set1: Main conformance
        b.flag(false);
        b.flag(false);
        b.flag(false);
        b.flag(false);
        b.u(2, 0);
        b.u(8, level_);
        b.ue(0);  // seq_parameter_set_id
        if (high_) {
            b.ue(1);       // chroma_format_idc 4:2:0
            b.ue(0);       // bit_depth_luma_minus8
            b.ue(0);       // bit_depth_chroma_minus8
            b.flag(false); // qpprime_y_zero_transform_bypass_flag
            b.flag(false); // seq_scaling_matrix_present_flag
        }
        b.ue(kLog2MaxFrameNum - 4);
        b.ue(0);  // pic_order_cnt_type
        b.ue(kLog2MaxPocLsb - 4);
        b.ue(1);       // max_num_ref_frames
        b.flag(false); // gaps_in_frame_num_value_allowed_flag
        b.ue(width_mbs_ - 1);
        b.ue(height_mbs_ - 1);
        b.flag(true);  // frame_mbs_only_flag
        b.flag(true);  // direct_8x8_inference_flag
        const bool crop = crop_right_ || crop_bottom_;
        b.flag(crop);
        if (crop) {
            b.ue(0);
            b.ue(crop_right_);
            b.ue(0);
            b.ue(crop_bottom_);
        }
        b.flag(true);  // vui_parameters_present_flag
        b.flag(false); // aspect_ratio_info_present_flag
        b.flag(false); // overscan_info_present_flag
        b.flag(true);  // video_signal_type_present_flag
        b.u(3, 5);     // video_format: unspecified
        b.flag(false); // video_full_range_flag: limited
        b.flag(true);  // colour_description_present_flag
        b.u(8, 1);     // colour_primaries: BT.709
        b.u(8, 1);     // transfer_characteristics: BT.709
        b.u(8, 1);     // matrix_coefficients: BT.709
        b.flag(true);  // chroma_loc_info_present_flag
        b.ue(1);       // chroma_sample_loc_type_top_field: centre (the VPP pass averages)
        b.ue(1);       // chroma_sample_loc_type_bottom_field
        b.flag(true);  // timing_info_present_flag
        b.u(32, 1);    // num_units_in_tick
        b.u(32, 2 * sp_.fps);  // time_scale
        b.flag(false); // fixed_frame_rate_flag
        b.flag(false); // nal_hrd_parameters_present_flag
        b.flag(false); // vcl_hrd_parameters_present_flag
        b.flag(false); // pic_struct_present_flag
        b.flag(true);  // bitstream_restriction_flag
        b.flag(true);  // motion_vectors_over_pic_boundaries_flag
        b.ue(0);       // max_bytes_per_pic_denom
        b.ue(0);       // max_bits_per_mb_denom
        b.ue(15);      // log2_max_mv_length_horizontal
        b.ue(15);      // log2_max_mv_length_vertical
        b.ue(0);       // max_num_reorder_frames
        b.ue(1);       // max_dec_frame_buffering
        b.trailing_bits();
    }

    void write_pps(BitWriter& b) const {
        b.ue(0);  // pic_parameter_set_id
        b.ue(0);  // seq_parameter_set_id
        b.flag(cabac_);
        b.flag(false);  // bottom_field_pic_order_in_frame_present_flag
        b.ue(0);        // num_slice_groups_minus1
        b.ue(0);        // num_ref_idx_l0_default_active_minus1
        b.ue(0);        // num_ref_idx_l1_default_active_minus1
        b.flag(false);  // weighted_pred_flag
        b.u(2, 0);      // weighted_bipred_idc
        b.se(kInitQp - 26);
        b.se(0);        // pic_init_qs_minus26
        b.se(0);        // chroma_qp_index_offset
        b.flag(true);   // deblocking_filter_control_present_flag
        b.flag(false);  // constrained_intra_pred_flag
        b.flag(false);  // redundant_pic_cnt_present_flag
        if (high_) {
            b.flag(true);   // transform_8x8_mode_flag
            b.flag(false);  // pic_scaling_matrix_present_flag
            b.se(0);        // second_chroma_qp_index_offset
        }
        b.trailing_bits();
    }

    void write_slice_header(BitWriter& b, const PictureParams& pic, uint32_t frame_num, uint32_t poc_lsb) const {
        b.ue(0);                         // first_mb_in_slice
        b.ue(pic.keyframe ? 7 : 5);      // slice_type: I / P, the same for every slice of the picture
        b.ue(0);                         // pic_parameter_set_id
        b.u(kLog2MaxFrameNum, frame_num);
        if (pic.keyframe) b.ue(uint32_t(pic.keyframe_count & 0xffff));  // idr_pic_id
        b.u(kLog2MaxPocLsb, poc_lsb);
        if (!pic.keyframe) {
            b.flag(false);  // num_ref_idx_active_override_flag
            b.flag(false);  // ref_pic_list_modification_flag_l0
        }
        // dec_ref_pic_marking
        if (pic.keyframe) {
            b.flag(false);  // no_output_of_prior_pics_flag
            b.flag(false);  // long_term_reference_flag
        } else {
            b.flag(false);  // adaptive_ref_pic_marking_mode_flag: sliding window
        }
        if (cabac_ && !pic.keyframe) b.ue(0);  // cabac_init_idc
        b.se(0);  // slice_qp_delta
        b.ue(0);  // disable_deblocking_filter_idc
        b.se(0);  // slice_alpha_c0_offset_div2
        b.se(0);  // slice_beta_offset_div2
    }

    StreamParams sp_;
    VAProfile profile_ = VAProfileH264High;
    bool high_ = true;
    bool cabac_ = true;
    uint32_t width_mbs_ = 0;
    uint32_t height_mbs_ = 0;
    uint8_t level_ = 41;
    uint32_t packed_ = 0;
    uint32_t crop_right_ = 0;
    uint32_t crop_bottom_ = 0;
};

}  // namespace

std::unique_ptr<CodecImpl> make_h264(const StreamParams& sp) { return std::make_unique<H264>(sp); }

}  // namespace brovideo::vaapi
