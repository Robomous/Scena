// SPDX-License-Identifier: MIT
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

/// Smallest triggerable unit: fires its actions when started.
/// Cardinality: actions 1..* (§8.3.2). An event without its own start
/// trigger enters runningState with its parent (§8.3); the §8.4.2 rule
/// that such an event inherits the enclosing Act's start trigger is
/// equivalent here because the Act's trigger has already fired by then.
struct Event {
    std::string name;
    std::optional<Trigger> start_trigger; ///< Absent: starts with parent.
    std::vector<std::shared_ptr<Action>> actions;
};

/// Cardinality: events 1..* (§8.3.2).
struct Maneuver {
    std::string name;
    std::vector<Event> events;
};

/// Cardinality: maneuvers 0..* (§8.3.2); an empty group completes
/// instantly (§8.4.4). `actors` lists entity ids; bulk application of
/// private actions to all actors arrives with the conflict rules sprint.
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
