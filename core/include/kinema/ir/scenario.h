// SPDX-License-Identifier: MIT
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "kinema/ir/action.h"
#include "kinema/ir/condition.h"

namespace kinema::ir {

/// Who drives an entity each step.
enum class ControlMode {
    EngineControlled, ///< The engine integrates the entity's motion (default behavior).
    HostControlled,   ///< The host simulator reports the entity's state each step.
};

/// A scenario participant (vehicle, pedestrian, ...). Only identity and
/// control ownership exist in this phase; geometry, categories, and
/// properties follow with the full OpenSCENARIO entity model.
struct Entity {
    std::string id;
    std::string name;
    ControlMode control_mode = ControlMode::EngineControlled;
};

/// One trigger/effect pair of the simplified storyboard.
/// Shared ownership keeps Scenario copyable; conditions and actions are
/// immutable once attached, so sharing them between copies is safe.
struct StoryboardEntry {
    std::shared_ptr<Condition> condition;
    std::shared_ptr<Action> action;
};

/// Deliberately simplified precursor of the full ASAM OpenSCENARIO storyboard:
/// a flat, ordered list of (Condition, Action) pairs instead of the
/// Story / Act / ManeuverGroup / Maneuver / Event hierarchy. The hierarchy,
/// trigger groups, and priority rules arrive with the XML frontend phase; the
/// flat list is enough to exercise the scheduler state machine end to end.
struct Storyboard {
    std::vector<StoryboardEntry> entries;
};

/// Root of the Scenario IR: the common representation both frontends
/// (OpenSCENARIO XML and OpenSCENARIO DSL) compile into, and the only input
/// the runtime executes.
struct Scenario {
    std::string name;
    std::vector<Entity> entities;
    Storyboard storyboard;
};

} // namespace kinema::ir
