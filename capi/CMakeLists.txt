# SPDX-License-Identifier: MIT
add_library(scena-capi SHARED
    src/capi.cpp
)
add_library(scena::capi ALIAS scena-capi)

target_include_directories(scena-capi PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(scena-capi PRIVATE scena::core)
target_compile_definitions(scena-capi PRIVATE SCN_CAPI_EXPORTS)
target_compile_features(scena-capi PRIVATE cxx_std_20)
set_target_properties(scena-capi PROPERTIES
    CXX_EXTENSIONS OFF
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
)

scn_set_warnings(scena-capi)
scn_enable_sanitizers(scena-capi)
