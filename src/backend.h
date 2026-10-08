#pragma once
// The backends the factory (factory.cpp) knows, and the one shape each one
// has. Raw is built in everywhere; an optional backend is compiled in behind
// its BROVIDEO_HAVE_* definition and declares itself here. Adding a backend
// (VA-API decode, Vulkan Video, VideoToolbox) is: its sources, its
// BROVIDEO_HAVE_* in CMakeLists.txt, one entry below, and one line in
// factory.cpp's list. Nothing else changes.

#include "brovideo/decoder.h"
#include "brovideo/encoder.h"

#include <memory>
#include <string>
#include <vector>

namespace brovideo::detail {

// A codec / direction a backend implements but cannot do here, and why.
struct Missing {
    Codec codec = Codec::Raw;
    Direction direction = Direction::Encode;
    std::string why;
};

struct ProbeResult {
    std::vector<Capability> caps;  // best first
    std::vector<Missing> missing;
};

struct BackendOps {
    Backend backend = Backend::Raw;
    // What the backend can do on this machine. Called once per process by
    // the factory, which keeps the result.
    ProbeResult (*probe)() = nullptr;
    // Makes an encoder / decoder for one of the capabilities the probe
    // reported (null for a backend that does not encode / decode).
    std::unique_ptr<Encoder> (*create_encoder)(const EncoderConfig&, const Capability&, std::string* err) = nullptr;
    std::unique_ptr<Decoder> (*create_decoder)(const DecoderConfig&, const Capability&, std::string* err) = nullptr;
};

const BackendOps& raw_backend();
#if defined(BROVIDEO_HAVE_VAAPI)
const BackendOps& vaapi_backend();  // src/vaapi: encode
#endif
#if defined(BROVIDEO_HAVE_MF)
const BackendOps& mf_backend();  // src/mf: decode
#endif

}  // namespace brovideo::detail
