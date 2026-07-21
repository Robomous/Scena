// SPDX-License-Identifier: MIT
#pragma once

#include <string>

namespace kinema {

// Keep in sync with the top-level CMakeLists.txt project version and python/pyproject.toml.
inline constexpr int kVersionMajor = 0;
inline constexpr int kVersionMinor = 1;
inline constexpr int kVersionPatch = 0;

/// Returns the library version as "major.minor.patch".
std::string version_string();

} // namespace kinema
