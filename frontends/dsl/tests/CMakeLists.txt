# SPDX-FileCopyrightText: 2026 Robomous
# SPDX-License-Identifier: Apache-2.0
include(GoogleTest)

function(scn_add_dsl_test name)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} PRIVATE scena::frontend-dsl GTest::gtest_main)
    target_compile_features(${name} PRIVATE cxx_std_20)
    set_target_properties(${name} PROPERTIES CXX_EXTENSIONS OFF)
    scn_set_warnings(${name})
    scn_set_fp_strictness(${name})
    scn_enable_sanitizers(${name})
    gtest_discover_tests(${name} DISCOVERY_TIMEOUT 60)
endfunction()

scn_add_dsl_test(dsl_lexer_test)
scn_add_dsl_test(dsl_parser_test)
scn_add_dsl_test(dsl_types_test)
scn_add_dsl_test(dsl_expression_test)
scn_add_dsl_test(dsl_constraint_test)
scn_add_dsl_test(dsl_import_test)
scn_add_dsl_test(dsl_stdlib_test)
