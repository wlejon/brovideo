#pragma once
// brovideo: hardware video encode and decode through the platform's own
// APIs (VA-API on Linux, Media Foundation on Windows), plus the built-in Raw
// codec everywhere. docs/design.md describes the parts.

#include "brovideo/bitstream.h"
#include "brovideo/codec.h"
#include "brovideo/decoder.h"
#include "brovideo/encoder.h"
#include "brovideo/frame.h"
