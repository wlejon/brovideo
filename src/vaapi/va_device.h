#pragma once
// VA devices: the render nodes, an open VADisplay, and the cached probe of
// what each node can encode.

#include "brovideo/codec.h"

#include <va/va.h>

#include <memory>
#include <string>
#include <vector>

namespace brovideo::vaapi {

// One codec a node can encode, with what the encoder needs to know about it.
struct CodecCaps {
    Codec codec = Codec::H264;
    VAProfile profile = VAProfileNone;     // the best one offered (H.264: High, else Main, else Constrained Baseline)
    VAEntrypoint entrypoint = VAEntrypointEncSlice;
    uint32_t packed_headers = 0;           // VAConfigAttribEncPackedHeaders
    uint32_t max_width = 0;                // 0: not reported
    uint32_t max_height = 0;
    uint32_t quality_levels = 0;           // VAConfigAttribEncQualityRange
    uint32_t align_width = 16;             // surface alignment (VASurfaceAttribAlignmentSize), at least 16
    uint32_t align_height = 16;
    // Codec-specific attributes, VA_ATTRIB_NOT_SUPPORTED where they do not apply.
    uint32_t hevc_features = VA_ATTRIB_NOT_SUPPORTED;     // VAConfigAttribEncHEVCFeatures
    uint32_t hevc_block_sizes = VA_ATTRIB_NOT_SUPPORTED;  // VAConfigAttribEncHEVCBlockSizes
    uint32_t av1_features = VA_ATTRIB_NOT_SUPPORTED;      // VAConfigAttribEncAV1
    uint32_t av1_ext1 = VA_ATTRIB_NOT_SUPPORTED;          // VAConfigAttribEncAV1Ext1
    uint32_t av1_ext2 = VA_ATTRIB_NOT_SUPPORTED;          // VAConfigAttribEncAV1Ext2
};

struct NodeCaps {
    std::string path;    // /dev/dri/renderD128
    std::string driver;  // the kernel driver behind it ("amdgpu"), empty if unknown
    std::vector<CodecCaps> codecs;
    [[nodiscard]] const CodecCaps* find(Codec c) const;
};

// Every render node that has a VA driver with a VideoProc entrypoint and at
// least one usable encoder, in order of preference: $BROVIDEO_VAAPI_DEVICE
// alone when it is set, else renderD128, renderD129, ... Probed once per
// process (the first call opens each node; later calls return the cache).
const std::vector<NodeCaps>& probe_nodes();

// The kernel driver that exported a dmabuf ("amdgpu"), from
// /proc/self/fdinfo; empty when it cannot be told.
std::string dmabuf_exporter(int fd);

// An initialised VADisplay on one render node; closes both on destruction.
class Display {
public:
    static std::unique_ptr<Display> open(const std::string& path, std::string* err);
    ~Display();
    Display(const Display&) = delete;
    Display& operator=(const Display&) = delete;

    [[nodiscard]] VADisplay get() const { return dpy_; }
    [[nodiscard]] const std::string& path() const { return path_; }
    [[nodiscard]] const std::string& vendor() const { return vendor_; }

private:
    Display() = default;
    int fd_ = -1;
    VADisplay dpy_ = nullptr;
    std::string path_;
    std::string vendor_;
};

// Debug logging to stderr when $BROVIDEO_VAAPI_DEBUG is set.
bool debug_enabled();
void debug_log(const char* fmt, ...)
#if defined(__GNUC__)
    __attribute__((format(printf, 1, 2)))
#endif
    ;

}  // namespace brovideo::vaapi
