# The Raw codec

`Codec::Raw` (value 0) carries RGBA pixels, run-length coded, with predicted
frames between keyframes so that keyframe handling is exercised exactly as
with a real codec. It is built in on every platform and runs on the CPU. It
exists so that whatever carries brovideo packets (broremote's protocol and
server, for one) runs and is tested everywhere: CI, Windows, a box with no
hardware encoder. It is not meant for real use.

One packet is one picture:

```
packet := u8 kind                 -- 0 intra (a keyframe), 1 delta (predicted)
          varint width, varint height
          token*                  -- until width * height pixels are covered
token  := varint t                -- n = (t >> 1) + 1 pixels
          t & 1 = 1: u8[4] pixel  -- that pixel, n times
          t & 1 = 0: u8[4 * n]    -- n literal pixels
```

`varint` is unsigned LEB128. A pixel is the bytes R, G, B, A (the encoder
swizzles a BGRA8 frame into that order). In a delta packet each pixel is
XORed with the same pixel of the previous picture, so unchanged areas are
runs of zero; a delta packet without a previous picture of the same size is a
lost sync. A token that runs past the picture, a truncated packet, a width or
height above 16384, or a picture of more than 8192 × 8192 pixels is malformed.
After any error the decoder drops its reference, so only a keyframe decodes
next.

The encoder takes CPU frames only, of exactly the configured size (it does
not scale), and makes a keyframe on its first frame, after a reconfigure or a
failure, and when one is forced or requested. The decoder's pictures are
`PixelFormat::RGBA8` in CPU memory, `width * 4` bytes per row.
