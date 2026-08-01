# SPDX-FileCopyrightText: 2026 Robomous
# SPDX-License-Identifier: Apache-2.0
add_library(scena-capi SHARED
    src/capi.cpp
)
add_library(scena::capi ALIAS scena-capi)

target_include_directories(scena-capi PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
# The C ABI exposes scenario loading (scn_engine_load_xml_*), so it links the
# XML frontend as well. Both are PRIVATE: no frontend or kernel type appears in
# capi.h, which stays C-clean.
target_link_libraries(scena-capi PRIVATE scena::core scena::frontend-xml)
target_compile_definitions(scena-capi PRIVATE SCN_CAPI_EXPORTS)
target_compile_features(scena-capi PRIVATE cxx_std_20)
set_target_properties(scena-capi PROPERTIES
    CXX_EXTENSIONS OFF
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
)

scn_set_warnings(scena-capi)
scn_set_fp_strictness(scena-capi)
scn_enable_sanitizers(scena-capi)

# A pure-C consumer of the ABI, built as C11 and run as a test. capi.h has to
# stay C-clean, and the only way to keep it that way is to have a C compiler
# read it on every platform in CI — the exit criterion of p6-s1.
if(SCN_BUILD_TESTS)
    enable_language(C)
    add_executable(scena_c_consumer tests/c_consumer.c)
    target_link_libraries(scena_c_consumer PRIVATE scena::capi)
    set_target_properties(scena_c_consumer PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED ON
                                                      C_EXTENSIONS OFF)
    scn_enable_sanitizers(scena_c_consumer)
    add_test(NAME CApi.PureCConsumer COMMAND scena_c_consumer)
endif()
