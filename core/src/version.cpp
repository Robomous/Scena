// SPDX-License-Identifier: MIT
#include "kinema/version.h"

#include <string>

namespace kinema {

std::string version_string() {
    return std::to_string(kVersionMajor) + "." + std::to_string(kVersionMinor) + "." +
           std::to_string(kVersionPatch);
}

} // namespace kinema
