# SPDX-FileCopyrightText: 2026 Robomous
# SPDX-License-Identifier: Apache-2.0
# scena-run: the headless execution CLI and the golden suite's driver.
#
# A thin consumer of the public API — the loader, Engine and the gateway
# interface — with no scenario semantics of its own. Deliberately no
# visualization (ADR-0001, library-first).

add_executable(scena-run src/main.cpp)
target_link_libraries(scena-run PRIVATE scena::core scena::frontend-xml)
target_compile_features(scena-run PRIVATE cxx_std_20)
set_target_properties(scena-run PROPERTIES CXX_EXTENSIONS OFF)
scn_set_warnings(scena-run)
scn_set_fp_strictness(scena-run)
scn_enable_sanitizers(scena-run)

if(SCN_BUILD_TESTS)
    add_subdirectory(tests)
endif()
