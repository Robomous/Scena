# SPDX-FileCopyrightText: 2026 Robomous
# SPDX-License-Identifier: Apache-2.0
include(GoogleTest)

# The CLI's behaviour is what a script depends on — the exit codes and the
# shape of a diagnostic line — so the test drives the built binary as a
# subprocess rather than linking its internals.
add_executable(scena_check_test scena_check_test.cpp)
target_link_libraries(scena_check_test PRIVATE GTest::gtest_main)
target_compile_features(scena_check_test PRIVATE cxx_std_20)
set_target_properties(scena_check_test PROPERTIES CXX_EXTENSIONS OFF)
target_compile_definitions(scena_check_test
    PRIVATE SCENA_CHECK_BINARY="$<TARGET_FILE:scena-check>")
scn_set_warnings(scena_check_test)
scn_set_fp_strictness(scena_check_test)
scn_enable_sanitizers(scena_check_test)
add_dependencies(scena_check_test scena-check)
gtest_discover_tests(scena_check_test DISCOVERY_TIMEOUT 60)
