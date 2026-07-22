// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "scena/status.h"

namespace scena {

/// How serious a diagnostic is.
///
/// The relation to Status is an invariant of the whole kernel: an operation
/// that emitted an Error diagnostic returned a non-Ok Status, and an
/// operation that emitted only Warning or Info diagnostics returned
/// Status::Ok. A degraded scenario is not a broken engine — the runtime
/// reports what it skipped and keeps stepping.
enum class Severity {
    Info = 0,    ///< Informational; nothing was skipped or rejected.
    Warning = 1, ///< Something was skipped or degraded; the operation succeeded.
    Error = 2,   ///< The operation failed.
};

/// Position of a diagnostic in a source document.
///
/// The kernel builds scenarios in memory and therefore leaves every field at
/// its unknown value; the XML and DSL frontends fill it in when they lower a
/// document into the Scenario IR (F1 onward).
struct SourceLocation {
    std::string file; ///< Source path; empty when unknown.
    int line = 0;     ///< 1-based line; 0 when unknown.
    int column = 0;   ///< 1-based column; 0 when unknown.
};

/// One structured finding about a scenario, emitted by validation, by the
/// runtime, or (later) by a frontend.
///
/// Messages are deterministic by construction: they name entity ids, element
/// names, and indices only. They never contain a floating-point value —
/// std::to_string is locale-sensitive and its shortest-representation
/// behavior differs between standard libraries, which would break the
/// bit-identical-output contract for anything derived from a message.
struct Diagnostic {
    Severity severity = Severity::Error;

    /// Machine-readable category. Reuses Status rather than introducing a
    /// parallel enumeration, so a host can compare a diagnostic against the
    /// Status the failing call returned.
    Status code = Status::Ok;

    /// Human-readable description, lower case, no trailing period.
    std::string message;

    /// Element path the finding is anchored to; empty addresses the whole
    /// scenario. Extends the '/'-joined addressing of
    /// Engine::storyboard_element_state:
    ///   - storyboard elements: "story/act/group/maneuver/event"
    ///   - entities:            "entities/<id>", or "entities[<index>]" when
    ///                          the id is empty (0-based document order)
    ///   - init actions:        "init/action[<index>]"
    ///   - triggers:            "<owner>/startTrigger|stopTrigger/group[<i>]/
    ///                          condition[<j>]", using the condition's name in
    ///                          place of "condition[<j>]" when it has one; the
    ///                          storyboard's own stop trigger is "stopTrigger"
    ///   - event actions:       "<event>/action[<index>]"
    std::string path;

    SourceLocation location;

    /// ASAM checker rule UID this finding enforces, e.g.
    /// "asam.net:xosc:1.0.0:data_type.condition_delay_not_negative"; empty
    /// when the standard defines no rule for the constraint.
    std::string rule_id;
};

/// Ordered, append-only collector of diagnostics.
///
/// Insertion order is the report order and is never rearranged: findings
/// accumulate in document order so that two runs over the same scenario
/// produce element-wise identical sequences. The sink never deduplicates and
/// never records a timestamp — both would make output depend on something
/// other than the scenario.
class DiagnosticSink {
public:
    /// Appends a diagnostic.
    void report(Diagnostic diagnostic) { diagnostics_.push_back(std::move(diagnostic)); }

    /// All diagnostics, in report order.
    [[nodiscard]] const std::vector<Diagnostic>& diagnostics() const noexcept {
        return diagnostics_;
    }

    /// True when at least one diagnostic has Severity::Error.
    [[nodiscard]] bool has_errors() const noexcept {
        return std::any_of(diagnostics_.begin(), diagnostics_.end(),
                           [](const Diagnostic& d) { return d.severity == Severity::Error; });
    }

    /// Drops every collected diagnostic.
    void clear() noexcept { diagnostics_.clear(); }

private:
    std::vector<Diagnostic> diagnostics_;
};

} // namespace scena
