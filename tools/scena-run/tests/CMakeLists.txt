# SPDX-FileCopyrightText: 2026 Robomous
# SPDX-License-Identifier: Apache-2.0
include(GoogleTest)

# The CLI's behaviour is what a script depends on — exit codes, the trace it
# writes, the round-trip of a double through the text file — so the test drives
# the built binary as a subprocess rather than linking its internals.
add_executable(scena_run_test scena_run_test.cpp)
target_link_libraries(scena_run_test PRIVATE GTest::gtest_main)
target_compile_features(scena_run_test PRIVATE cxx_std_20)
set_target_properties(scena_run_test PROPERTIES CXX_EXTENSIONS OFF)
target_compile_definitions(scena_run_test
    PRIVATE SCENA_RUN_BINARY="$<TARGET_FILE:scena-run>"
            SCENA_GOLDEN_DIR="${CMAKE_SOURCE_DIR}/tests/golden")
scn_set_warnings(scena_run_test)
scn_set_fp_strictness(scena_run_test)
scn_enable_sanitizers(scena_run_test)
add_dependencies(scena_run_test scena-run)
gtest_discover_tests(scena_run_test DISCOVERY_TIMEOUT 60)
