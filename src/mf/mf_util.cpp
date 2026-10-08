#include "mf_util.h"

#include <d3d11_4.h>
#include <objbase.h>

#include <cstdio>
#include <mutex>

namespace brovideo::mf {

std::string hr_text(const char* what, HRESULT hr) {
    char head[96];
    std::snprintf(head, sizeof head, "%s: 0x%08lX", what, static_cast<unsigned long>(hr));
    std::string out = head;
    wchar_t* msg = nullptr;
    const DWORD n = FormatMessageW(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM |
                                       FORMAT_MESSAGE_IGNORE_INSERTS,
                                   nullptr, DWORD(hr), 0, reinterpret_cast<wchar_t*>(&msg), 0, nullptr);
    if (n && msg) {
        const int len = WideCharToMultiByte(CP_UTF8, 0, msg, int(n), nullptr, 0, nullptr, nullptr);
        std::string text(size_t(len), '\0');
        WideCharToMultiByte(CP_UTF8, 0, msg, int(n), text.data(), len, nullptr, nullptr);
        while (!text.empty() && (text.back() == '\n' || text.back() == '\r' || text.back() == ' ' ||
                                 text.back() == '.')) {
            text.pop_back();
        }
        out += " (" + text + ")";
    }
    if (msg) LocalFree(msg);
    return out;
}

bool ensure_started(std::string* err) {
    // COM per thread: S_FALSE (already initialised) and RPC_E_CHANGED_MODE
    // (an STA thread, e.g. one that called OleInitialize) are both fine for
    // the free-threaded MF objects used here. The matching CoUninitialize is
    // deliberately skipped: the thread may be using COM for its own reasons.
    thread_local bool com = false;
    if (!com) {
        const HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
        if (FAILED(hr) && hr != RPC_E_CHANGED_MODE) {
            if (err) *err = hr_text("CoInitializeEx", hr);
            return false;
        }
        com = true;
    }
    static std::once_flag once;
    static HRESULT started = E_FAIL;
    std::call_once(once, [] { started = MFStartup(MF_VERSION, MFSTARTUP_LITE); });
    if (FAILED(started)) {
        if (err) *err = hr_text("MFStartup", started);
        return false;
    }
    return true;
}

bool subtype_for(Codec c, GUID& out) {
    switch (c) {
        case Codec::H264: out = MFVideoFormat_H264; return true;
        case Codec::HEVC: out = MFVideoFormat_HEVC; return true;
        case Codec::AV1: out = MFVideoFormat_AV1; return true;
        default: return false;
    }
}

ComPtr<IMFTransform> create_decoder_mft(Codec codec, std::string* err) {
    GUID subtype{};
    if (!subtype_for(codec, subtype)) {
        if (err) *err = std::string("no Media Foundation decoder for ") + codec_name(codec);
        return nullptr;
    }
    MFT_REGISTER_TYPE_INFO in{MFMediaType_Video, subtype};
    MFT_REGISTER_TYPE_INFO out{MFMediaType_Video, MFVideoFormat_NV12};
    IMFActivate** acts = nullptr;
    UINT32 count = 0;
    // Synchronous MFTs only: the system decoders (msmpeg2vdec, the HEVC and
    // AV1 extensions) are synchronous and do DXVA themselves once given a
    // device manager; vendor async hardware MFTs need the event model.
    HRESULT hr = MFTEnumEx(MFT_CATEGORY_VIDEO_DECODER,
                           MFT_ENUM_FLAG_SYNCMFT | MFT_ENUM_FLAG_LOCALMFT | MFT_ENUM_FLAG_SORTANDFILTER, &in, &out,
                           &acts, &count);
    if (FAILED(hr) || count == 0) {
        if (err) {
            *err = FAILED(hr) ? hr_text("MFTEnumEx", hr)
                              : std::string("no ") + codec_name(codec) + " decoder is installed (Media Foundation)";
        }
        if (acts) CoTaskMemFree(acts);
        return nullptr;
    }
    ComPtr<IMFTransform> mft;
    std::string last;
    for (UINT32 i = 0; i < count; ++i) {
        if (!mft) {
            hr = acts[i]->ActivateObject(IID_PPV_ARGS(&mft));
            if (FAILED(hr)) {
                last = hr_text("activating the decoder MFT", hr);
                mft.Reset();
            }
        }
        acts[i]->Release();
    }
    CoTaskMemFree(acts);
    if (!mft && err) *err = last;
    return mft;
}

bool create_d3d11(ComPtr<ID3D11Device>& device, ComPtr<IMFDXGIDeviceManager>& manager, std::string* err) {
    static const D3D_FEATURE_LEVEL levels[] = {D3D_FEATURE_LEVEL_11_1, D3D_FEATURE_LEVEL_11_0, D3D_FEATURE_LEVEL_10_1,
                                               D3D_FEATURE_LEVEL_10_0};
    ComPtr<ID3D11DeviceContext> ctx;
    HRESULT hr = D3D11CreateDevice(nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr,
                                   D3D11_CREATE_DEVICE_VIDEO_SUPPORT | D3D11_CREATE_DEVICE_BGRA_SUPPORT, levels,
                                   UINT(std::size(levels)), D3D11_SDK_VERSION, &device, nullptr, &ctx);
    if (FAILED(hr)) {
        if (err) *err = hr_text("D3D11CreateDevice", hr);
        return false;
    }
    // The MFT uses the device from inside ProcessOutput while this decoder
    // copies frames out of it; both go through the immediate context.
    ComPtr<ID3D11Multithread> mt;
    if (SUCCEEDED(device.As(&mt))) mt->SetMultithreadProtected(TRUE);
    UINT token = 0;
    hr = MFCreateDXGIDeviceManager(&token, &manager);
    if (SUCCEEDED(hr)) hr = manager->ResetDevice(device.Get(), token);
    if (FAILED(hr)) {
        if (err) *err = hr_text("MFCreateDXGIDeviceManager", hr);
        device.Reset();
        manager.Reset();
        return false;
    }
    return true;
}

std::string adapter_name(ID3D11Device* device) {
    if (!device) return {};
    ComPtr<IDXGIDevice> dxgi;
    ComPtr<IDXGIAdapter> adapter;
    DXGI_ADAPTER_DESC desc{};
    if (FAILED(device->QueryInterface(IID_PPV_ARGS(&dxgi))) || FAILED(dxgi->GetAdapter(&adapter)) ||
        FAILED(adapter->GetDesc(&desc))) {
        return {};
    }
    const int len = WideCharToMultiByte(CP_UTF8, 0, desc.Description, -1, nullptr, 0, nullptr, nullptr);
    if (len <= 1) return {};
    std::string out(size_t(len - 1), '\0');
    WideCharToMultiByte(CP_UTF8, 0, desc.Description, -1, out.data(), len, nullptr, nullptr);
    return out;
}

}  // namespace brovideo::mf
