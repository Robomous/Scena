# SPDX-FileCopyrightText: 2026 Robomous
# SPDX-License-Identifier: Apache-2.0
add_library(scena-core STATIC
    src/version.cpp
    src/engine.cpp
    src/ir/action.cpp
    src/ir/condition.cpp
    src/ir/date_time.cpp
    src/ir/entity.cpp
    src/ir/entity_condition.cpp
    src/ir/interaction_condition.cpp
    src/ir/rule.cpp
    src/ir/trigger.cpp
    src/runtime/clock.cpp
    src/runtime/detmath.cpp
    src/runtime/distance_measure.cpp
    src/runtime/element_ref.cpp
    src/runtime/lateral.cpp
    src/runtime/longitudinal.cpp
    src/runtime/obb2.cpp
    src/runtime/position_resolver.cpp
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
