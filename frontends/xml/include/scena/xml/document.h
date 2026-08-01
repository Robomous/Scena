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

#include <string>

#include "scena/ir/scenario.h"

namespace scena::xml {

/// Revision of the ASAM OpenSCENARIO XML standard a document declares, read
/// from `FileHeader/@revMajor` and `@revMinor` (both required, unsignedShort).
///
/// Scena targets 1.0-1.3. 1.4 exists (it is the version of the local
/// reference copy) but is deliberately outside the target range and is
/// rejected by the loader, per the coverage matrix.
struct DocumentVersion {
    int rev_major = 0; ///< 0 when the document declared no readable version.
    int rev_minor = 0;

    /// True for the versions Scena executes: 1.0, 1.1, 1.2 and 1.3.
    ///
    /// Version 1.3 corrected the calculation specifications of the `Position`
    /// sub-classes; per ASAM OpenSCENARIO XML 1.4.0 §5 the corrected
    /// semantics are the ones an implementation should apply to *every*
    /// version, so Scena runs one set of position semantics for the whole
    /// range and does not branch on the minor revision.
    [[nodiscard]] bool is_supported() const noexcept {
        return rev_major == 1 && rev_minor >= 0 && rev_minor <= 3;
    }

    /// "<major>.<minor>", for diagnostics. Integer formatting only — the
    /// kernel's diagnostic contract forbids floating-point text in messages.
    [[nodiscard]] std::string to_string() const {
        return std::to_string(rev_major) + "." + std::to_string(rev_minor);
    }

    [[nodiscard]] friend bool operator==(const DocumentVersion&, const DocumentVersion&) = default;
};

/// Which of the three OpenSCENARIO XML document forms the root element holds
/// (§9): a scenario, a catalog, or a parameter-value-distribution file. The
/// form is decided by the single child of `OpenSCENARIO` that follows
/// `FileHeader`.
enum class DocumentKind {
    Unknown = 0,               ///< No recognizable definition child.
    Scenario,                  ///< ScenarioDefinition: Storyboard and friends.
    Catalog,                   ///< CatalogDefinition (§9.4); loaded in p4-s4.
    ParameterValueDistribution ///< §9.7; outside v0.0.1 scope (roadmap P4).
};

/// One loaded OpenSCENARIO XML document: what the file declared about itself
/// plus the Scenario IR compiled from it.
///
/// The document layer (p4-s1) fills `version` and `kind`; the storyboard and
/// entity lowering that fills `scenario` arrives with p4-s2, so a successful
/// load currently yields an empty scenario and a warning per unconsumed
/// element.
struct Document {
    DocumentVersion version;
    DocumentKind kind = DocumentKind::Unknown;
    ir::Scenario scenario;
};

} // namespace scena::xml
