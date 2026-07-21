// SPDX-License-Identifier: MIT
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "scena/ir/action.h"
#include "scena/ir/condition.h"

namespace scena::ir {

// The full ASAM OpenSCENARIO storyboard hierarchy
// (Storyboard -> Story -> Act -> ManeuverGroup -> Maneuver -> Event ->
// Action, nesting and cardinalities per ASAM OpenSCENARIO XML 1.4.0
// §8.3.2). Element names are part of the runtime contract: the engine's
// state query addresses elements by their name path, so names must be
// non-empty and unique among siblings (validated at Engine::init).
//
// Start and stop triggers are a single nullable Condition in this phase;
// the full Trigger model (condition groups, edges, delays — §7.6) replaces
// it in a later sprint without changing the hierarchy.
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
/// equivalent here because the Act's trigger has already fired by then —
/// revisited when the full Trigger model lands.
struct Event {
    std::string name;
    std::shared_ptr<Condition> start_trigger; ///< Null: starts with parent.
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
/// only elements that own start triggers (§7.6.1.1).
struct Act {
    std::string name;
    std::shared_ptr<Condition> start_trigger; ///< Null: starts with parent.
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
    std::shared_ptr<Condition> stop_trigger; ///< Null: never completes.
};

} // namespace scena::ir
