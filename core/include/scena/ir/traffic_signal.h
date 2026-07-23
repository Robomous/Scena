// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <optional>
#include <string>
#include <vector>

namespace scena::ir {

/// The observable state of one traffic signal during a phase, per ASAM
/// OpenSCENARIO XML 1.4.0 §TrafficSignalState.
///
/// Both fields are free-form strings by design: `traffic_signal_id` names a
/// signal in the road network file, and "interpretation and notation of state
/// are specific to the simulation engine used" (§6.11.4) — a signal's state may
/// be "go", "on;off" or anything else the host and the scenario agree on. Scena
/// stores and compares them as opaque strings and never interprets them.
struct TrafficSignalState {
    /// ID of the referenced signal in the road network file.
    std::string traffic_signal_id;
    /// Observable state, e.g. "off;off;on".
    std::string state;
};

/// One semantic phase of a signal cycle, per §Phase (§6.11.4).
///
/// The 1.4 `semantics` attribute (§TrafficSignalSemantics) is outside the
/// targeted 1.0–1.3 versions and is not modeled; neither is the 1.4
/// §TrafficSignalGroupState shorthand, whose per-signal expansion needs the
/// road network's signal group.
struct Phase {
    /// Unique within its controller (§6.11.4).
    std::string name;
    /// Unit: [s]. Range: [0..inf[ per rule
    /// asam.net:xosc:1.0.0:data_type.phase_duration_positive.
    double duration = 0.0;
    /// The signals this phase drives, in document order.
    std::vector<TrafficSignalState> signal_states;
};

/// A traffic signal controller: "provides a signal cycle to a single signal or
/// a signal group" (§6.11.1, §6.11.2).
///
/// The phases form a cycle whose duration is the sum of theirs, and "the first
/// Phase repeats after the last Phase has ended" (§6.11.4). A controller may
/// start late relative to another one: `delay` seconds after the referenced
/// controller's first phase starts (§6.11.3), which is how a progressive signal
/// system is expressed. `delay` requires `reference`.
struct TrafficSignalController {
    /// ID for reference within the scenario, and the reference to a signal
    /// group in the road network. Unique among controllers.
    std::string name;
    /// Offset to `reference`'s first phase. Unit: [s]. Range: [0..inf[.
    /// Present only together with `reference`.
    std::optional<double> delay;
    /// Name of another TrafficSignalController this one is chained to (rule
    /// reference_control.traffic_signal_controller_references, C.7.13).
    std::optional<std::string> reference;
    /// The ordered cycle. May be empty (the XSD allows 0..*), which simply
    /// means the controller drives nothing.
    std::vector<Phase> phases;
};

} // namespace scena::ir
