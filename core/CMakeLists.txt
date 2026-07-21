# SPDX-License-Identifier: MIT
add_library(kinema-core STATIC
    src/version.cpp
    src/engine.cpp
    src/ir/action.cpp
    src/ir/condition.cpp
    src/runtime/clock.cpp
    src/runtime/scheduler.cpp
)
add_library(kinema::core ALIAS kinema-core)

target_include_directories(kinema-core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(kinema-core PUBLIC cxx_std_20)
set_target_properties(kinema-core PROPERTIES
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
)

knm_set_warnings(kinema-core)
knm_enable_sanitizers(kinema-core)
