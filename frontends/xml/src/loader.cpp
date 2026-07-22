// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#include "scena/xml/loader.h"

namespace scena::xml {

LoadResult load_xosc(const std::string& path) {
    LoadResult result;
    result.diagnostics.push_back(
        {Severity::Error,
         "OpenSCENARIO XML loading is not implemented in this phase (stub frontend): " + path});
    return result;
}

} // namespace scena::xml
