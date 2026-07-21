// SPDX-License-Identifier: MIT
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
};

} // namespace scena
