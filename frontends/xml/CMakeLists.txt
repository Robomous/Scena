# SPDX-FileCopyrightText: 2026 Robomous
# SPDX-License-Identifier: Apache-2.0
add_library(scena-frontend-xml STATIC
    src/loader.cpp
)
add_library(scena::frontend-xml ALIAS scena-frontend-xml)

target_include_directories(scena-frontend-xml PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
# pugixml is wired for the upcoming XML frontend phase; the loader stub does
# not use it yet.
target_link_libraries(scena-frontend-xml
    PUBLIC scena::core
    PRIVATE pugixml::pugixml
)
target_compile_features(scena-frontend-xml PUBLIC cxx_std_20)
set_target_properties(scena-frontend-xml PROPERTIES
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
)

scn_set_warnings(scena-frontend-xml)
scn_set_fp_strictness(scena-frontend-xml)
scn_enable_sanitizers(scena-frontend-xml)
