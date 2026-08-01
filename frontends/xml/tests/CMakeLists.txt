# SPDX-FileCopyrightText: 2026 Robomous
# SPDX-License-Identifier: Apache-2.0
# Added after core/tests at the top level, matching roads/opendrive/tests.
include(GoogleTest)

foreach(name xml_byvalue_test xml_catalog_test xml_expression_test xml_infra_test
              xml_selection_test xml_storyboard_test)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} PRIVATE scena::frontend-xml GTest::gtest_main)
    target_compile_features(${name} PRIVATE cxx_std_20)
    set_target_properties(${name} PROPERTIES CXX_EXTENSIONS OFF)
    scn_set_warnings(${name})
    scn_set_fp_strictness(${name})
    scn_enable_sanitizers(${name})
    gtest_discover_tests(${name} DISCOVERY_TIMEOUT 60)
endforeach()

# The GS-1 fixture is a committed .xosc read from the source tree, so the test
# does not depend on the build layout.
target_compile_definitions(xml_storyboard_test
    PRIVATE SCENA_TEST_SCENARIO_DIR="${CMAKE_CURRENT_SOURCE_DIR}/scenarios")

# The catalog fixture tree is committed and read from the source tree, so the
# relative directories a scenario declares resolve the way a real one's do.
target_compile_definitions(xml_catalog_test
    PRIVATE SCENA_TEST_CATALOG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/catalogs")
