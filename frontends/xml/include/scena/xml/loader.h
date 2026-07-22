// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
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
