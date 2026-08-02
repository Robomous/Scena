# SPDX-FileCopyrightText: 2026 Robomous
# SPDX-License-Identifier: Apache-2.0
# scena-check: the OpenSCENARIO DSL checking CLI.
#
# A thin consumer of the public DSL loader — check_file and the diagnostic
# sink — with no language semantics of its own. It links the DSL frontend only:
# checking a .osc file needs no engine, no road backend and no XML frontend.

add_executable(scena-check src/main.cpp)
target_link_libraries(scena-check PRIVATE scena::core scena::frontend-dsl)
target_compile_features(scena-check PRIVATE cxx_std_20)
set_target_properties(scena-check PROPERTIES CXX_EXTENSIONS OFF)
scn_set_warnings(scena-check)
scn_set_fp_strictness(scena-check)
scn_enable_sanitizers(scena-check)

if(SCN_BUILD_TESTS)
    add_subdirectory(tests)
endif()
