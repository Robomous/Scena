# SPDX-License-Identifier: MIT
add_library(kinema-frontend-xml STATIC
    src/loader.cpp
)
add_library(kinema::frontend-xml ALIAS kinema-frontend-xml)

target_include_directories(kinema-frontend-xml PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
# pugixml is wired for the upcoming XML frontend phase; the loader stub does
# not use it yet.
target_link_libraries(kinema-frontend-xml
    PUBLIC kinema::core
    PRIVATE pugixml::pugixml
)
target_compile_features(kinema-frontend-xml PUBLIC cxx_std_20)
set_target_properties(kinema-frontend-xml PROPERTIES
    CXX_EXTENSIONS OFF
    POSITION_INDEPENDENT_CODE ON
)

knm_set_warnings(kinema-frontend-xml)
knm_enable_sanitizers(kinema-frontend-xml)
