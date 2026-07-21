# SPDX-License-Identifier: MIT
add_library(scena-core STATIC
    src/version.cpp
    src/engine.cpp
    src/ir/action.cpp
    src/ir/condition.cpp
    src/ir/trigger.cpp
    src/runtime/clock.cpp
    src/runtime/scheduler.cpp
)
add_library(scena::core ALIAS scena-core)

target_include_directories(scena-core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_features(scena-core PUBLIC cxx_std_20)
set_target_properties(scena-core PROPERTIES
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
)

scn_set_warnings(scena-core)
scn_set_fp_strictness(scena-core)
scn_enable_sanitizers(scena-core)
