# SPDX-License-Identifier: MIT
# All third-party dependencies, fetched with FetchContent and pinned to exact tags.
# Every entry here must be recorded in THIRD_PARTY_LICENSES.md with its license.

include(FetchContent)

# googletest — tests only, never linked into shipped libraries. BSD-3-Clause.
if(KNM_BUILD_TESTS)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.15.2
    )
    # Share the CRT on Windows so gtest matches the project's runtime library.
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()

# pugixml — XML frontend only. MIT.
FetchContent_Declare(
    pugixml
    GIT_REPOSITORY https://github.com/zeux/pugixml.git
    GIT_TAG v1.14
)
FetchContent_MakeAvailable(pugixml)

# nanobind — Python bindings only. BSD-3-Clause.
if(KNM_BUILD_PYTHON)
    find_package(Python 3.9 REQUIRED COMPONENTS Interpreter Development.Module)
    FetchContent_Declare(
        nanobind
        GIT_REPOSITORY https://github.com/wjakob/nanobind.git
        GIT_TAG v2.13.0
    )
    FetchContent_MakeAvailable(nanobind)
endif()
