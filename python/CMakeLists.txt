# SPDX-License-Identifier: MIT
# Python bindings: nanobind extension module `kinema._kinema`.

nanobind_add_module(_kinema src/bindings.cpp)
target_link_libraries(_kinema PRIVATE kinema::core)

# Stage an importable package under <build>/python so examples and tests can
# run against the build tree: PYTHONPATH=<build>/python.
set(KNM_PY_STAGE_DIR ${CMAKE_BINARY_DIR}/python)
set_target_properties(_kinema PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${KNM_PY_STAGE_DIR}/kinema
)
foreach(_config ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER ${_config} _config_upper)
    set_target_properties(_kinema PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY_${_config_upper} ${KNM_PY_STAGE_DIR}/kinema
    )
endforeach()
add_custom_command(TARGET _kinema POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/src/kinema/__init__.py
            ${KNM_PY_STAGE_DIR}/kinema/__init__.py
)

# Wheel builds driven by scikit-build-core install the module into the
# `kinema` package.
if(SKBUILD)
    install(TARGETS _kinema LIBRARY DESTINATION kinema)
endif()
