// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>

namespace scena {

// Keep in sync with the top-level CMakeLists.txt project version and python/pyproject.toml.
inline constexpr int kVersionMajor = 0;
inline constexpr int kVersionMinor = 1;
inline constexpr int kVersionPatch = 0;

/// Returns the library version as "major.minor.patch".
std::string version_string();

} // namespace scena
