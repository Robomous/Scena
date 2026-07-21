// SPDX-License-Identifier: MIT
#include "kinema/xml/loader.h"

namespace kinema::xml {

LoadResult load_xosc(const std::string& path) {
    LoadResult result;
    result.diagnostics.push_back(
        {Severity::Error,
         "OpenSCENARIO XML loading is not implemented in this phase (stub frontend): " + path});
    return result;
}

} // namespace kinema::xml
