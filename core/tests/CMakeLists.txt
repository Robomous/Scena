# SPDX-FileCopyrightText: 2026 Robomous
# SPDX-License-Identifier: Apache-2.0
include(GoogleTest)

# Shared test-support library: the determinism fixture, the detmath probe list,
# and the trace recorder, used by both the tests and the trace runner so all
# three drive byte-for-byte identical workloads. Same warnings / FP-strictness /
# sanitizer / C++20-no-extensions treatment as the tests.
add_library(scena-testsupport STATIC
    support/fixtures.cpp
    support/detmath_probe.cpp
    support/trace_recorder.cpp
)
add_library(scena::testsupport ALIAS scena-testsupport)
target_link_libraries(scena-testsupport PUBLIC scena::core)
target_include_directories(scena-testsupport PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(scena-testsupport PUBLIC cxx_std_20)
set_target_properties(scena-testsupport PROPERTIES CXX_EXTENSIONS OFF)
scn_set_warnings(scena-testsupport)
scn_set_fp_strictness(scena-testsupport)
scn_enable_sanitizers(scena-testsupport)

# Plain executable (not a gtest, not via scn_add_test) so gtest_discover_tests
# never sees it; the determinism-trace CI job runs it directly.
add_executable(scena-trace-runner support/trace_runner.cpp)
target_link_libraries(scena-trace-runner PRIVATE scena::testsupport)
target_compile_features(scena-trace-runner PRIVATE cxx_std_20)
set_target_properties(scena-trace-runner PROPERTIES CXX_EXTENSIONS OFF)
scn_set_warnings(scena-trace-runner)
scn_set_fp_strictness(scena-trace-runner)
scn_enable_sanitizers(scena-trace-runner)

function(scn_add_test name)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} PRIVATE scena::core GTest::gtest_main)
    target_compile_features(${name} PRIVATE cxx_std_20)
    set_target_properties(${name} PROPERTIES CXX_EXTENSIONS OFF)
    scn_set_warnings(${name})
    scn_set_fp_strictness(${name})
    scn_enable_sanitizers(${name})
    gtest_discover_tests(${name} DISCOVERY_TIMEOUT 60)
endfunction()

scn_add_test(version_test)
scn_add_test(clock_test)
scn_add_test(scheduler_test)
scn_add_test(storyboard_test)
scn_add_test(engine_test)
scn_add_test(determinism_test)
scn_add_test(detmath_test)
scn_add_test(trigger_test)
scn_add_test(event_priority_test)
scn_add_test(diagnostics_test)
scn_add_test(entity_types_test)
scn_add_test(entity_model_test)
scn_add_test(condition_byvalue_test)
scn_add_test(condition_byentity_kinematics_test)
scn_add_test(bounding_box_test)
scn_add_test(obb2_test)
scn_add_test(condition_byentity_interaction_test)
scn_add_test(longitudinal_test)
scn_add_test(action_longitudinal_test)
scn_add_test(action_teleport_test)
scn_add_test(action_ir_test)
scn_add_test(action_distance_test)
scn_add_test(action_lateral_test)
scn_add_test(action_routing_test)
scn_add_test(action_controller_visibility_test)
scn_add_test(action_global_test)
scn_add_test(traffic_signal_test)

# The determinism and detmath tests share the fixture and probe list with the
# trace runner, so they link the support library.
target_link_libraries(determinism_test PRIVATE scena::testsupport)
target_link_libraries(detmath_test PRIVATE scena::testsupport)
# obb2_test hex-pins raw freespace distances, so it needs hex_bits().
target_link_libraries(obb2_test PRIVATE scena::testsupport)

if(SCN_BUILD_CAPI)
    scn_add_test(capi_test)
    target_link_libraries(capi_test PRIVATE scena::capi)
endif()
