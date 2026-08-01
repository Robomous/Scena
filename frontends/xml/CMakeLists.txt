# SPDX-FileCopyrightText: 2026 Robomous
# SPDX-License-Identifier: Apache-2.0
add_library(scena-frontend-xml STATIC
    src/attributes.cpp
    src/loader.cpp
    src/read_actions.cpp
    src/read_common.cpp
    src/read_conditions.cpp
    src/read_entities.cpp
    src/read_geometry.cpp
    src/read_storyboard.cpp
    src/reader_context.cpp
)
add_library(scena::frontend-xml ALIAS scena-frontend-xml)

target_include_directories(scena-frontend-xml PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
# pugixml stays PRIVATE: it is an implementation detail of the reader and must
# never appear in a public scena/xml header.
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
