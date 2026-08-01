# SPDX-FileCopyrightText: 2026 Robomous
# SPDX-License-Identifier: Apache-2.0
# Added after core/tests at the top level: links scena::testsupport (hex_bits)
# which is declared there.
include(GoogleTest)

add_executable(opendrive_geometry_test opendrive_geometry_test.cpp)
target_link_libraries(opendrive_geometry_test
    PRIVATE scena::roads-opendrive scena::testsupport GTest::gtest_main
)
target_compile_features(opendrive_geometry_test PRIVATE cxx_std_20)
set_target_properties(opendrive_geometry_test PROPERTIES CXX_EXTENSIONS OFF)
scn_set_warnings(opendrive_geometry_test)
scn_set_fp_strictness(opendrive_geometry_test)
scn_enable_sanitizers(opendrive_geometry_test)
gtest_discover_tests(opendrive_geometry_test DISCOVERY_TIMEOUT 60)
