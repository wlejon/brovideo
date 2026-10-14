cmake_minimum_required(VERSION 3.24)

project(brovideo VERSION 0.1.0 LANGUAGES CXX)

# Dependencies (cmake/bro_deps.cmake): an outer project's target wins (under bro
# these are all bro's), else the ../<name> working tree beside the top-level
# project, else the head of its main, fetched at configure.
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/bro_deps.cmake)

# Targets
#   brovideo::brovideo   codec enum, frames / packets / pictures, Encoder / Decoder,
#                        the capability probe, and the backends this platform has
#
# Platform APIs only: VA-API (libva, libva-drm, libdrm) and brodmabuf on
# Linux, Media Foundation and D3D11 (the Windows SDK) on Windows. The Raw
# codec is built everywhere, so a build with no backend still encodes and
# decodes.

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(_brovideo_top_level OFF)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(_brovideo_top_level ON)
endif()

option(BROVIDEO_BUILD_TESTS "Build the brovideo tests" ${_brovideo_top_level})

if(MSVC AND _brovideo_top_level)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

function(brovideo_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /permissive- /utf-8 /MP)
        target_compile_definitions(${target} PRIVATE _CRT_SECURE_NO_WARNINGS)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra)
    endif()
endfunction()

add_library(brovideo STATIC
    src/factory.cpp
    src/bitstream.cpp
    src/raw/raw_codec.cpp
)
add_library(brovideo::brovideo ALIAS brovideo)
target_include_directories(brovideo
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)
brovideo_warnings(brovideo)

# ---- VA-API encoder (Linux) --------------------------------------------------------
# Auto: on when libva, libva-drm and libdrm (for the fourcc header) are found.
# It also needs brodmabuf (fence waits), a bro_dependency (cmake/bro_deps.cmake):
# an existing brodmabuf target (bro's), else ../brodmabuf, else the head of its main.
if(UNIX AND NOT APPLE)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(BROVIDEO_VA IMPORTED_TARGET GLOBAL libva libva-drm libdrm)
    endif()
    if(BROVIDEO_VA_FOUND)
        set(_brovideo_vaapi_default ON)
    else()
        set(_brovideo_vaapi_default OFF)
    endif()
    option(BROVIDEO_WITH_VAAPI "Build the VA-API encoder" ${_brovideo_vaapi_default})
    if(BROVIDEO_WITH_VAAPI AND NOT BROVIDEO_VA_FOUND)
        message(FATAL_ERROR "BROVIDEO_WITH_VAAPI is ON but libva, libva-drm and libdrm were not all found")
    endif()
    if(BROVIDEO_WITH_VAAPI)
        bro_dependency(brodmabuf GITHUB wlejon/brodmabuf
                       OPTIONS BRODMABUF_BUILD_TESTS=OFF)
    endif()
    if(BROVIDEO_WITH_VAAPI)
        message(STATUS "brovideo: VA-API encoder enabled (libva ${BROVIDEO_VA_libva_VERSION})")
        target_sources(brovideo PRIVATE
            src/vaapi/va_util.cpp
            src/vaapi/va_device.cpp
            src/vaapi/va_source.cpp
            src/vaapi/va_vpp.cpp
            src/vaapi/va_encoder.cpp
            src/vaapi/va_h264.cpp
            src/vaapi/va_hevc.cpp
            src/vaapi/va_av1.cpp
            src/vaapi/va_decoder.cpp
            src/vaapi/va_dec_h264.cpp
            src/vaapi/va_dec_hevc.cpp
            src/vaapi/va_dec_av1.cpp
        )
        target_compile_definitions(brovideo PRIVATE BROVIDEO_HAVE_VAAPI)
        target_link_libraries(brovideo PRIVATE PkgConfig::BROVIDEO_VA brodmabuf::brodmabuf)
    else()
        message(STATUS "brovideo: VA-API encoder disabled (libva, libva-drm and libdrm not all found)")
    endif()
else()
    set(BROVIDEO_WITH_VAAPI OFF)
endif()

# ---- Media Foundation (Windows) ----------------------------------------------------
# Everything it needs is in the Windows SDK; which codecs actually encode and
# decode is probed at run time (src/mf/mf_probe.cpp).
if(WIN32)
    option(BROVIDEO_WITH_MF "Build the Media Foundation backend" ON)
else()
    set(BROVIDEO_WITH_MF OFF)
endif()
if(BROVIDEO_WITH_MF)
    message(STATUS "brovideo: Media Foundation backend enabled")
    target_sources(brovideo PRIVATE
        src/mf/mf_util.cpp
        src/mf/mf_encoder.cpp
        src/mf/mf_decoder.cpp
        src/mf/mf_probe.cpp
    )
    target_compile_definitions(brovideo PRIVATE BROVIDEO_HAVE_MF WIN32_LEAN_AND_MEAN NOMINMAX UNICODE _UNICODE)
    target_link_libraries(brovideo PRIVATE mfplat mfuuid d3d11 dxgi ole32 oleaut32)
endif()

if(BROVIDEO_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
