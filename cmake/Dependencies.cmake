# SPDX-FileCopyrightText: 2026 Robomous
# SPDX-License-Identifier: Apache-2.0
# All third-party dependencies, fetched with FetchContent and pinned to exact tags.
# Every entry here must be recorded in THIRD_PARTY_LICENSES.md with its license.

include(FetchContent)

# Sanitizer instrumentation must be uniform across EVERY translation unit in the
# process, third-party ones included. libc++ only emits its std::vector
# container annotations in a translation unit compiled with -fsanitize=address:
# a vector's spare capacity is poisoned on allocation and unpoisoned as elements
# are constructed. Both halves of that protocol live in weak, inline template
# functions, so an uninstrumented dependency contributes copies that poison
# nothing — and the linker is free to pick a poisoning `__assign_with_size` and
# a non-unpoisoning `__construct_at_end` for the same call chain. The result is
# an AddressSanitizer container-overflow report inside an ordinary vector
# assignment, in code that is entirely correct.
#
# That is exactly what an ASan build on macOS reported for
# CApiTest.FollowNurbsTrajectoryFollowsTheCircle, in the knot-vector assignment
# of TrajectoryEvaluator::build_nurbs: googletest was the uninstrumented half.
# Linux happened to resolve the same symbols consistently, so CI never saw it.
# Instrumenting the dependencies keeps the annotations coherent and keeps a red
# local sanitizer run meaningful. Applied to the C++ dependencies that end up
# in a test process; nanobind is excluded because the ASan configuration builds
# with SCN_BUILD_PYTHON=OFF.
function(scn_sanitize_dependency target)
    if(SCN_SANITIZE STREQUAL "" OR NOT TARGET ${target})
        return()
    endif()
    # A dependency may expose its public name as an alias (pugixml) and compile
    # its sources under another (pugixml-static); follow the alias, and skip
    # INTERFACE libraries, which compile nothing.
    get_target_property(_aliased ${target} ALIASED_TARGET)
    if(_aliased)
        set(target ${_aliased})
    endif()
    get_target_property(_type ${target} TYPE)
    if(_type STREQUAL "INTERFACE_LIBRARY")
        return()
    endif()
    scn_enable_sanitizers(${target})
endfunction()

# googletest — tests only, never linked into shipped libraries. BSD-3-Clause.
if(SCN_BUILD_TESTS)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.15.2
    )
    # Share the CRT on Windows so gtest matches the project's runtime library.
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
    foreach(_gtest_target gtest gtest_main gmock gmock_main)
        scn_sanitize_dependency(${_gtest_target})
    endforeach()
endif()

# pugixml — XML frontend only. MIT.
FetchContent_Declare(
    pugixml
    GIT_REPOSITORY https://github.com/zeux/pugixml.git
    GIT_TAG v1.14
)
FetchContent_MakeAvailable(pugixml)
foreach(_pugixml_target pugixml pugixml-static pugixml-shared)
    scn_sanitize_dependency(${_pugixml_target})
endforeach()

# nanobind — Python bindings only. BSD-3-Clause.
if(SCN_BUILD_PYTHON)
    find_package(Python 3.9 REQUIRED COMPONENTS Interpreter Development.Module)
    FetchContent_Declare(
        nanobind
        GIT_REPOSITORY https://github.com/wjakob/nanobind.git
        GIT_TAG v2.13.0
    )
    FetchContent_MakeAvailable(nanobind)
endif()
