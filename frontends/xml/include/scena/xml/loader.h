/*
 * Copyright 2026 Robomous
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <optional>
#include <string>
#include <vector>

#include "scena/ir/scenario.h"

namespace scena::xml {

enum class Severity {
    Info,
    Warning,
    Error,
};

/// One loader message tied to the input document.
struct Diagnostic {
    Severity severity = Severity::Error;
    std::string message;
};

/// Result of loading an OpenSCENARIO XML document: the compiled Scenario IR on
/// success, plus any diagnostics produced along the way.
struct LoadResult {
    std::optional<ir::Scenario> scenario;
    std::vector<Diagnostic> diagnostics;

    [[nodiscard]] bool ok() const { return scenario.has_value(); }
};

/// Loads an ASAM OpenSCENARIO XML (.xosc) file, versions 1.0-1.3, and compiles
/// it into the Scenario IR.
///
/// Stub in this phase: always fails with a "not implemented" diagnostic. Real
/// parsing arrives with the XML frontend phase.
LoadResult load_xosc(const std::string& path);

} // namespace scena::xml
