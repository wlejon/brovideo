#pragma once
// Media Foundation plumbing shared by the decoder and its probe: process-wide
// MF startup, the per-thread COM apartment, HRESULT text, and finding the
// decoder MFT for a codec.

#include "brovideo/codec.h"

#include <d3d11.h>
#include <mfapi.h>
#include <mfidl.h>
#include <mftransform.h>
#include <wrl/client.h>

#include <string>
#include <vector>

namespace brovideo::mf {

using Microsoft::WRL::ComPtr;

// "<what>: 0x8000FFFF (Catastrophic failure)".
std::string hr_text(const char* what, HRESULT hr);

// MFStartup once per process (never shut down: MF objects may outlive any
// one decoder) and COM on the calling thread (multithreaded; a thread that is
// already single-threaded keeps its apartment, which MF's free-threaded
// objects tolerate). False with *err when Media Foundation is unavailable.
bool ensure_started(std::string* err);

// The MF subtype GUID of a codec; false for Raw.
bool subtype_for(Codec c, GUID& out);

// The decoder MFT for `codec` with NV12 output, best first (hardware-capable
// synchronous MFTs: the system's DXVA decoders). Null with *err when there is
// none registered.
ComPtr<IMFTransform> create_decoder_mft(Codec codec, std::string* err);

// Hardware encoder MFTs for `codec` with NV12 input, filtered to match the
// D3D11 adapter's PCI vendor ID (`vendor_id`, e.g. 0x10DE) when present so a
// stale MFT from another GPU vendor is never given this adapter's device.
std::vector<ComPtr<IMFActivate>> enum_encoder_mfts(Codec codec, UINT32 vendor_id, std::string* err);

// The MFT's friendly name (MFT_FRIENDLY_NAME_Attribute), or empty.
std::string activate_name(IMFActivate* act);

// A D3D11 device for hardware encoding/decoding (video support, multithread
// protected) and the DXGI device manager that hands it to an MFT. False with
// *err when there is no usable hardware device.
bool create_d3d11(ComPtr<ID3D11Device>& device, ComPtr<IMFDXGIDeviceManager>& manager, std::string* err);

// The DXGI adapter's description behind `device` ("NVIDIA GeForce RTX
// 4090"); empty when it cannot be read.
std::string adapter_name(ID3D11Device* device);

// The DXGI adapter's PCI VendorId (e.g. 0x10DE, 0x1002, 0x8086), or 0.
UINT32 adapter_vendor_id(ID3D11Device* device);

}  // namespace brovideo::mf
