# SPDX-License-Identifier: MIT
# Python bindings: nanobind extension module `scena._scena`.

nanobind_add_module(_scena src/bindings.cpp)
target_link_libraries(_scena PRIVATE scena::core)

# Stage an importable package under <build>/python so examples and tests can
# run against the build tree: PYTHONPATH=<build>/python.
set(SCN_PY_STAGE_DIR ${CMAKE_BINARY_DIR}/python)
set_target_properties(_scena PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${SCN_PY_STAGE_DIR}/scena
)
foreach(_config ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${_config} _config_upper)
    set_target_properties(_scena PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY_${_config_upper} ${SCN_PY_STAGE_DIR}/scena
    )
endforeach()
add_custom_command(TARGET _scena POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/src/scena/__init__.py
            ${SCN_PY_STAGE_DIR}/scena/__init__.py
)

# Wheel builds driven by scikit-build-core install the module into the
# `scena` package.
if(SKBUILD)
    install(TARGETS _scena LIBRARY DESTINATION scena)
endif()
