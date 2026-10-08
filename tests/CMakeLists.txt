# Each test is a plain executable whose exit status is non-zero on any failed
# check (check.h); a watchdog in each turns a hang into a failure.
#   brovideo_test(<name> [sources...] [TIMEOUT s])
function(brovideo_test name)
    cmake_parse_arguments(PARSE_ARGV 1 BT "" "TIMEOUT" "")
    add_executable(${name} ${name}.cpp ${BT_UNPARSED_ARGUMENTS})
    target_link_libraries(${name} PRIVATE brovideo)
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    if(WIN32)
        target_compile_definitions(${name} PRIVATE WIN32_LEAN_AND_MEAN NOMINMAX)
    endif()
    add_test(NAME ${name} COMMAND ${name})
    if(NOT BT_TIMEOUT)
        set(BT_TIMEOUT 300)
    endif()
    set_tests_properties(${name} PROPERTIES TIMEOUT ${BT_TIMEOUT})
endfunction()

brovideo_test(test_raw TIMEOUT 120)

# The VA-API encoder against ffmpeg (skipped, exit 77, without a VA encoder).
# brodmabuf's GBM supplies the dmabuf frames.
if(BROVIDEO_WITH_VAAPI)
    brovideo_test(test_vaapi test_pattern.cpp TIMEOUT 900)
    target_link_libraries(test_vaapi PRIVATE brodmabuf::brodmabuf)
    set_tests_properties(test_vaapi PROPERTIES SKIP_RETURN_CODE 77)
endif()

# The Media Foundation decoder: fixtures made with ffmpeg when it is on PATH,
# and the VA-API encoder's recorded output in tests/fixtures.
if(BROVIDEO_WITH_MF)
    brovideo_test(test_mf test_pattern.cpp TIMEOUT 600)
    target_compile_definitions(test_mf PRIVATE BROVIDEO_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures")
    target_link_libraries(test_mf PRIVATE d3d11)
endif()
