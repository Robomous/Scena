# SPDX-FileCopyrightText: 2026 Robomous
# SPDX-License-Identifier: Apache-2.0
# Added after core/tests at the top level, matching roads/opendrive/tests.
include(GoogleTest)

foreach(name xml_infra_test)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} PRIVATE scena::frontend-xml GTest::gtest_main)
    target_compile_features(${name} PRIVATE cxx_std_20)
    set_target_properties(${name} PROPERTIES CXX_EXTENSIONS OFF)
    scn_set_warnings(${name})
    scn_set_fp_strictness(${name})
    scn_enable_sanitizers(${name})
    gtest_discover_tests(${name} DISCOVERY_TIMEOUT 60)
endforeach()
