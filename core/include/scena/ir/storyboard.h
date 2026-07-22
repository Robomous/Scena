// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "scena/ir/action.h"
#include "scena/ir/trigger.h"

namespace scena::ir {

// The full ASAM OpenSCENARIO storyboard hierarchy
// (Storyboard -> Story -> Act -> ManeuverGroup -> Maneuver -> Event ->
// Action, nesting and cardinalities per ASAM OpenSCENARIO XML 1.4.0
// §8.3.2). Element names are part of the runtime contract: the engine's
// state query addresses elements by their name path, so names must be
// non-empty and unique among siblings (validated at Engine::init).
//
// Trigger hosting follows §7.6.1.1/§7.6.1.2 exactly: start triggers are
// hosted only by Act and Event, stop triggers only by Storyboard and Act.
// Every trigger field is optional, and absent is not the same as empty:
//   - std::nullopt — no trigger. A start trigger-less element starts with
//     its parent (§7.6.1.1); a stop trigger-less element is only stopped
//     through an ancestor's stop trigger.
//   - an engaged but empty Trigger — always false (§7.6.1): the element
//     never starts, respectively is never stopped by its own trigger.
//
// DSL lowerability note: the OpenSCENARIO DSL composition operators
// (serial / parallel / one_of, ASAM OpenSCENARIO DSL 2.2.0 §7.3.13) lower
// onto this model — parallel maps to the sibling-parallelism of §8.3.3.1,
// serial to trigger chaining on element completion, one_of to a selected
// single child. Keep additions to this hierarchy expressible in both
// standards.

/// How a starting event interacts with the other events of its Maneuver,
/// per §7.3.2 and §8.4.2.2. The scope is the Maneuver, not the maneuver
/// group: "A maneuver groups events creating a scope where the events can
/// interact with each other using the event priority rules" (§7.3.3).
///
/// The pre-1.3 literal `overwrite` is deprecated and its normative
/// description is word-for-word identical to `override`, so it is a purely
/// lexical synonym and carries no separate enumerator here; the XML
/// frontend maps it onto Override at load time (p4-s2). See ADR-0005.
enum class EventPriority {
    /// Terminates every *running* event of the same Maneuver when this one
    /// moves to runningState (§8.4.2.2). Standby siblings are untouched.
    Override,
    /// Starts regardless of the states of the other events of the Maneuver
    /// (§8.4.2.2). Scena's default, because it is the only literal that
    /// leaves sibling events alone.
    Parallel,
    /// Does not start while another event of the same Maneuver is in
    /// runningState; with no running sibling it starts normally (Priority
    /// class reference). A skipped start is a skipTransition and counts as
    /// an execution (§8.4.2.1).
    Skip,
};

/// Smallest triggerable unit: fires its actions when started.
/// Cardinality: actions 1..* (§8.3.2). An event without its own start
/// trigger enters runningState with its parent (§8.3); the §8.4.2 rule
/// that such an event inherits the enclosing Act's start trigger is
/// equivalent here because the Act's trigger has already fired by then.
struct Event {
    std::string name;
    std::optional<Trigger> start_trigger; ///< Absent: starts with parent.
    EventPriority priority = EventPriority::Parallel;
    /// How many times the event may execute (§8.3.3.2); executions are the
    /// sum of its startTransitions and skipTransitions (§8.4.2.1) and are
    /// performed sequentially. Zero means the event never executes: it
    /// completes with a skipTransition the moment it would enter standby
    /// (§8.4.2.1). Negative values are rejected at Engine::init. The XSD
    /// type is unsignedInt; a signed field is what turns a negative count
    /// arriving through the C ABI or Python into a reported error rather
    /// than a wraparound.
    int maximum_execution_count = 1;
    std::vector<std::shared_ptr<Action>> actions;
};

/// Cardinality: events 1..* (§8.3.2).
struct Maneuver {
    std::string name;
    std::vector<Event> events;
};

/// Cardinality: maneuvers 0..* (§8.3.2); an empty group completes
/// instantly (§8.4.4). `actors` lists entity ids; bulk application of
/// private actions to all actors (§7.5.4, §8.3.3.3) arrives with p5-s4.
/// The group's own maximumExecutionCount (§8.4.4) arrives with p4-s2 —
/// see ADR-0005 for why neither is part of p1-s3.
struct ManeuverGroup {
    std::string name;
    std::vector<std::string> actors;
    std::vector<Maneuver> maneuvers;
};

/// Cardinality: maneuver groups 1..* (§8.3.2). Acts and Events are the
/// only elements that own start triggers (§7.6.1.1); the Act and the
/// Storyboard are the only ones that own stop triggers (§7.6.1.2).
struct Act {
    std::string name;
    std::optional<Trigger> start_trigger; ///< Absent: starts with parent.
    /// Absent: only an ancestor's stop trigger can stop this subtree. When
    /// it fires, the act and all its descendants complete through a
    /// stopTransition (§7.6.1.2).
    std::optional<Trigger> stop_trigger;
    std::vector<ManeuverGroup> groups;
};

/// Cardinality: acts 1..* (§8.3.2).
struct Story {
    std::string name;
    std::vector<Act> acts;
};

/// Root of the executable hierarchy. Cardinality: stories 0..* (§8.3.2).
/// A storyboard never completes on its own: only its stop trigger moves it
/// to completeState; without one it runs forever (§8.4.7).
struct Storyboard {
    std::vector<Story> stories;
    std::optional<Trigger> stop_trigger; ///< Absent: never completes.
};

} // namespace scena::ir
