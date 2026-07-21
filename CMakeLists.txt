# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.24)

# Keep the version in sync with core/include/scena/version.h and python/pyproject.toml.
project(scena VERSION 0.1.0 LANGUAGES CXX)

option(SCN_BUILD_TESTS "Build unit tests" ON)
option(SCN_BUILD_PYTHON "Build Python bindings" ON)
option(SCN_BUILD_CAPI "Build the C API shared library" ON)
option(SCN_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)
set(SCN_SANITIZE "" CACHE STRING "Sanitizers: empty, address, undefined, thread (comma-separated)")

set(CMAKE_CXX_EXTENSIONS OFF)

# Colocate executables and shared libraries so tests find the C API library on all platforms.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

include(cmake/CompilerWarnings.cmake)
include(cmake/Sanitizers.cmake)
include(cmake/Dependencies.cmake)

if(SCN_BUILD_TESTS)
    enable_testing()
endif()

add_subdirectory(core)
add_subdirectory(frontends/xml)

if(SCN_BUILD_CAPI)
    add_subdirectory(capi)
endif()

if(SCN_BUILD_PYTHON)
    add_subdirectory(python)
endif()

# Tests are added last so they can link targets from capi/ (declared after core/).
if(SCN_BUILD_TESTS)
    add_subdirectory(core/tests)
endif()
