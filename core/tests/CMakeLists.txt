# SPDX-License-Identifier: MIT
include(GoogleTest)

function(knm_add_test name)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} PRIVATE kinema::core GTest::gtest_main)
    target_compile_features(${name} PRIVATE cxx_std_20)
    set_target_properties(${name} PROPERTIES CXX_EXTENSIONS OFF)
    knm_set_warnings(${name})
    knm_enable_sanitizers(${name})
    gtest_discover_tests(${name} DISCOVERY_TIMEOUT 60)
endfunction()

knm_add_test(version_test)
knm_add_test(clock_test)
knm_add_test(scheduler_test)
knm_add_test(engine_test)
knm_add_test(determinism_test)

if(KNM_BUILD_CAPI)
    knm_add_test(capi_test)
    target_link_libraries(capi_test PRIVATE kinema::capi)
endif()
