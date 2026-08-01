# SPDX-FileCopyrightText: 2026 Robomous
# SPDX-License-Identifier: Apache-2.0
# OpenDRIVE road backend (p3-s2): a separate library the core never links.
# Hosts construct the backend and hand it to the engine through the gateway
# (IRoadQuery); the dependency arrow points backend -> core only.
add_library(scena-roads-opendrive STATIC
    src/reader.cpp
    src/reference_line.cpp
)
add_library(scena::roads-opendrive ALIAS scena-roads-opendrive)

target_include_directories(scena-roads-opendrive PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(scena-roads-opendrive
    PUBLIC scena::core
    PRIVATE pugixml::pugixml
)
target_compile_features(scena-roads-opendrive PUBLIC cxx_std_20)
set_target_properties(scena-roads-opendrive PROPERTIES
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
)

scn_set_warnings(scena-roads-opendrive)
scn_set_fp_strictness(scena-roads-opendrive)
scn_enable_sanitizers(scena-roads-opendrive)
