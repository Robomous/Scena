# SPDX-FileCopyrightText: 2026 Robomous
# SPDX-License-Identifier: Apache-2.0
# OpenSCENARIO DSL frontend. Like the XML frontend it compiles into the shared
# Scenario IR; runtime semantics live in the runtime, never here.
#
# No parser generator: the lexer and parser are hand-written from the ASAM
# OpenSCENARIO DSL specification text (ADR-0027, clean room per ADR-0002).

add_library(scena-frontend-dsl STATIC
    src/lexer.cpp
    src/parser.cpp
)
add_library(scena::frontend-dsl ALIAS scena-frontend-dsl)

target_include_directories(scena-frontend-dsl PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(scena-frontend-dsl PUBLIC scena::core)
target_compile_features(scena-frontend-dsl PUBLIC cxx_std_20)
set_target_properties(scena-frontend-dsl PROPERTIES CXX_EXTENSIONS OFF)

scn_set_warnings(scena-frontend-dsl)
scn_set_fp_strictness(scena-frontend-dsl)
scn_enable_sanitizers(scena-frontend-dsl)
