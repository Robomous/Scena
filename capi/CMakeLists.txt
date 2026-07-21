# SPDX-License-Identifier: MIT
add_library(kinema-capi SHARED
    src/capi.cpp
)
add_library(kinema::capi ALIAS kinema-capi)

target_include_directories(kinema-capi PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(kinema-capi PRIVATE kinema::core)
target_compile_definitions(kinema-capi PRIVATE KNM_CAPI_EXPORTS)
target_compile_features(kinema-capi PRIVATE cxx_std_20)
set_target_properties(kinema-capi PROPERTIES
    CXX_EXTENSIONS OFF
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
)

knm_set_warnings(kinema-capi)
knm_enable_sanitizers(kinema-capi)
