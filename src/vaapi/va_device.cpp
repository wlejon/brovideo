// Render node discovery, VADisplay lifetime, and the capability probe behind
// vaapi_encoders().
#include "vaapi/va_device.h"

#include "vaapi/va_codec.h"
#include "vaapi/va_util.h"

#include <va/va_drm.h>

#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <mutex>

namespace brovideo::vaapi {

bool debug_enabled() {
    static const bool on = [] {
        const char* v = std::getenv("BROVIDEO_VAAPI_DEBUG");
        return v && *v && std::strcmp(v, "0") != 0;
    }();
    return on;
}

void debug_log(const char* fmt, ...) {
    if (!debug_enabled()) return;
    std::va_list ap;
    va_start(ap, fmt);
    std::fputs("brovideo vaapi: ", stderr);
    std::vfprintf(stderr, fmt, ap);
    std::fputc('\n', stderr);
    va_end(ap);
}

namespace {

void va_debug_message(void*, const char* message) {
    if (debug_enabled()) std::fprintf(stderr, "libva: %s", message);
}

std::vector<std::string> render_node_paths() {
    if (const char* forced = std::getenv("BROVIDEO_VAAPI_DEVICE"); forced && *forced) return {forced};
    std::vector<std::pair<int, std::string>> nodes;
    if (DIR* d = opendir("/dev/dri")) {
        while (dirent* e = readdir(d)) {
            if (std::strncmp(e->d_name, "renderD", 7) != 0) continue;
            nodes.emplace_back(std::atoi(e->d_name + 7), std::string("/dev/dri/") + e->d_name);
        }
        closedir(d);
    }
    std::sort(nodes.begin(), nodes.end());
    std::vector<std::string> out;
    for (auto& n : nodes) out.push_back(std::move(n.second));
    return out;
}

std::string node_driver(const std::string& path) {
    const std::string name = path.substr(path.rfind('/') + 1);
    char buf[512];
    const std::string link = "/sys/class/drm/" + name + "/device/driver";
    const ssize_t n = readlink(link.c_str(), buf, sizeof(buf) - 1);
    if (n <= 0) return {};
    buf[n] = 0;
    const char* slash = std::strrchr(buf, '/');
    return slash ? slash + 1 : buf;
}

bool has_entrypoint(VADisplay dpy, VAProfile profile, VAEntrypoint ep) {
    const int max = vaMaxNumEntrypoints(dpy);
    if (max <= 0) return false;
    std::vector<VAEntrypoint> eps(static_cast<size_t>(max));
    int n = 0;
    if (vaQueryConfigEntrypoints(dpy, profile, eps.data(), &n) != VA_STATUS_SUCCESS) return false;
    return std::find(eps.begin(), eps.begin() + n, ep) != eps.begin() + n;
}

// The surface alignment the encoder needs (VASurfaceAttribAlignmentSize,
// libva 2.17+), queried on a throwaway config; 16x16 when not reported.
void surface_alignment(VADisplay dpy, VAProfile p, VAEntrypoint ep, CodecCaps& out) {
    out.align_width = out.align_height = 16;
    VAConfigAttrib a[2] = {{VAConfigAttribRTFormat, VA_RT_FORMAT_YUV420}, {VAConfigAttribRateControl, VA_RC_CBR}};
    VAConfigID cfg = VA_INVALID_ID;
    if (vaCreateConfig(dpy, p, ep, a, 2, &cfg) != VA_STATUS_SUCCESS) return;
    unsigned n = 0;
    vaQuerySurfaceAttributes(dpy, cfg, nullptr, &n);
    std::vector<VASurfaceAttrib> attrs(n);
    if (n && vaQuerySurfaceAttributes(dpy, cfg, attrs.data(), &n) == VA_STATUS_SUCCESS) {
        for (unsigned i = 0; i < n; ++i) {
            if (attrs[i].type != VASurfaceAttribAlignmentSize) continue;
            const uint32_t v = uint32_t(attrs[i].value.value.i);
            out.align_width = std::max(16u, 1u << (v & 0xf));
            out.align_height = std::max(16u, 1u << ((v >> 4) & 0xf));
        }
    }
    vaDestroyConfig(dpy, cfg);
}

// The best usable profile/entrypoint of `codec` on `dpy`, if any: 4:2:0
// 8-bit input and CBR rate control are required.
bool probe_codec(VADisplay dpy, const std::vector<VAProfile>& profiles, Codec codec, CodecCaps& out) {
    const int max = vaMaxNumProfiles(dpy);
    if (max <= 0) return false;
    std::vector<VAProfile> have(static_cast<size_t>(max));
    int n = 0;
    if (vaQueryConfigProfiles(dpy, have.data(), &n) != VA_STATUS_SUCCESS) return false;
    have.resize(size_t(n));
    for (VAProfile p : profiles) {
        if (std::find(have.begin(), have.end(), p) == have.end()) continue;
        for (VAEntrypoint ep : {VAEntrypointEncSlice, VAEntrypointEncSliceLP}) {
            if (!has_entrypoint(dpy, p, ep)) continue;
            VAConfigAttrib a[5] = {{VAConfigAttribRTFormat, 0},
                                   {VAConfigAttribRateControl, 0},
                                   {VAConfigAttribEncPackedHeaders, 0},
                                   {VAConfigAttribMaxPictureWidth, 0},
                                   {VAConfigAttribMaxPictureHeight, 0}};
            if (vaGetConfigAttributes(dpy, p, ep, a, 5) != VA_STATUS_SUCCESS) continue;
            // Asked one at a time: a driver may fail the whole call for an
            // attribute that does not apply to the profile.
            const auto attr = [&](VAConfigAttribType t) {
                VAConfigAttrib x{t, 0};
                return vaGetConfigAttributes(dpy, p, ep, &x, 1) == VA_STATUS_SUCCESS ? x.value
                                                                                     : uint32_t(VA_ATTRIB_NOT_SUPPORTED);
            };
            const uint32_t quality = attr(VAConfigAttribEncQualityRange);
            if (a[0].value == VA_ATTRIB_NOT_SUPPORTED || !(a[0].value & VA_RT_FORMAT_YUV420)) continue;
            if (a[1].value == VA_ATTRIB_NOT_SUPPORTED || !(a[1].value & VA_RC_CBR)) continue;
            out.codec = codec;
            out.profile = p;
            out.entrypoint = ep;
            out.packed_headers = a[2].value == VA_ATTRIB_NOT_SUPPORTED ? 0 : a[2].value;
            out.max_width = a[3].value == VA_ATTRIB_NOT_SUPPORTED ? 0 : a[3].value;
            out.max_height = a[4].value == VA_ATTRIB_NOT_SUPPORTED ? 0 : a[4].value;
            out.quality_levels = quality == VA_ATTRIB_NOT_SUPPORTED ? 0 : quality;
            out.hevc_features = attr(VAConfigAttribEncHEVCFeatures);
            out.hevc_block_sizes = attr(VAConfigAttribEncHEVCBlockSizes);
            out.av1_features = attr(VAConfigAttribEncAV1);
            out.av1_ext1 = attr(VAConfigAttribEncAV1Ext1);
            out.av1_ext2 = attr(VAConfigAttribEncAV1Ext2);
            surface_alignment(dpy, p, ep, out);
            return true;
        }
    }
    return false;
}

std::vector<NodeCaps> run_probe() {
    std::vector<NodeCaps> nodes;
    for (const std::string& path : render_node_paths()) {
        std::string err;
        auto display = Display::open(path, &err);
        if (!display) {
            debug_log("%s: %s", path.c_str(), err.c_str());
            continue;
        }
        VADisplay dpy = display->get();
        if (!has_entrypoint(dpy, VAProfileNone, VAEntrypointVideoProc)) {
            debug_log("%s: no VideoProc entrypoint", path.c_str());
            continue;
        }
        NodeCaps node;
        node.path = path;
        node.driver = node_driver(path);
        CodecCaps caps;
        for (Codec c : supported_codecs()) {
            if (probe_codec(dpy, codec_profiles(c), c, caps)) node.codecs.push_back(caps);
        }
        debug_log("%s (%s, %s): %zu encoders", path.c_str(), node.driver.c_str(), display->vendor().c_str(),
                  node.codecs.size());
        if (!node.codecs.empty()) nodes.push_back(std::move(node));
    }
    return nodes;
}

}  // namespace

const CodecCaps* NodeCaps::find(Codec c) const {
    for (const CodecCaps& cc : codecs) {
        if (cc.codec == c) return &cc;
    }
    return nullptr;
}

const std::vector<NodeCaps>& probe_nodes() {
    static std::once_flag once;
    static std::vector<NodeCaps> nodes;
    std::call_once(once, [] { nodes = run_probe(); });
    return nodes;
}

std::string dmabuf_exporter(int fd) {
    std::ifstream in("/proc/self/fdinfo/" + std::to_string(fd));
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("exp_name:", 0) == 0) {
            size_t i = 9;
            while (i < line.size() && (line[i] == ' ' || line[i] == '\t')) ++i;
            return line.substr(i);
        }
    }
    return {};
}

std::unique_ptr<Display> Display::open(const std::string& path, std::string* err) {
    std::unique_ptr<Display> d(new Display);
    d->path_ = path;
    d->fd_ = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
    if (d->fd_ < 0) {
        if (err) *err = "cannot open " + path + ": " + std::strerror(errno);
        return nullptr;
    }
    d->dpy_ = vaGetDisplayDRM(d->fd_);
    if (!d->dpy_) {
        if (err) *err = "vaGetDisplayDRM failed on " + path;
        return nullptr;
    }
    // libva prints its info and errors to stderr by default; ours go through
    // `err`, and libva's own only with BROVIDEO_VAAPI_DEBUG.
    vaSetInfoCallback(d->dpy_, va_debug_message, nullptr);
    vaSetErrorCallback(d->dpy_, va_debug_message, nullptr);
    int major = 0, minor = 0;
    const VAStatus s = vaInitialize(d->dpy_, &major, &minor);
    if (s != VA_STATUS_SUCCESS) {
        vaTerminate(d->dpy_);
        d->dpy_ = nullptr;
        if (err) *err = va_error(("vaInitialize on " + path).c_str(), s);
        return nullptr;
    }
    if (const char* v = vaQueryVendorString(d->dpy_)) d->vendor_ = v;
    return d;
}

Display::~Display() {
    if (dpy_) vaTerminate(dpy_);
    if (fd_ >= 0) ::close(fd_);
}

}  // namespace brovideo::vaapi
