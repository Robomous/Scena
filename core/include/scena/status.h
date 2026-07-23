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

namespace scena {

/// Result codes for all engine operations. The public API reports failures
/// through these codes; no exceptions cross the API boundary.
///
/// The C ABI mirrors these values, so enumerators are appended at the end and
/// never renumbered or removed.
enum class Status {
    Ok = 0,
    AlreadyInitialized,
    NotInitialized,
    UnknownEntity,
    InvalidControlMode,
    /// API misuse by the host: a null argument, a NaN or negative dt, an
    /// out-of-range enumerator. Never used for defects in scenario content.
    InvalidArgument,
    /// A frontend could not parse the source document. Reserved: no frontend
    /// exists yet, so the kernel never returns it (F1).
    ParseError,
    /// Scenario content violates a structural constraint of the standard —
    /// an empty name, a duplicate sibling name, a missing required child.
    ValidationError,
    /// Scenario content references something that does not exist, such as an
    /// action targeting an entity the scenario never declares.
    SemanticError,
    /// A construct the engine does not implement yet. Reported as a warning
    /// diagnostic at runtime rather than as a failure.
    UnsupportedFeature,
    /// A host referenced a named value (variable, parameter, user-defined
    /// value) the scenario does not declare — e.g. set_variable on a name
    /// with no VariableDeclaration. Host API misuse, like UnknownEntity, not
    /// a defect in scenario content.
    UnknownName,
    /// Scenario content uses a construct the standard deprecated (e.g. a
    /// ReachPositionCondition). Reported as a warning diagnostic; the
    /// construct is still executed.
    DeprecatedFeature,
};

} // namespace scena
