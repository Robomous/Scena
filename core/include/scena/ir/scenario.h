// SPDX-License-Identifier: MIT
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "scena/ir/action.h"
#include "scena/ir/storyboard.h"

namespace scena::ir {

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

/// Root of the Scenario IR: the common representation both frontends
/// (OpenSCENARIO XML and OpenSCENARIO DSL) compile into, and the only input
/// the runtime executes.
///
/// `init_actions` model the storyboard-external init phase: they are
/// applied concurrently during Engine::init, before simulation time
/// starts, per ASAM OpenSCENARIO XML 1.4.0 §8.5. All actions are
/// instantaneous in this phase, so every init action completes during
/// init; non-instantaneous init actions (which stay running into the
/// storyboard, §8.5.1.2) arrive with transition dynamics.
struct Scenario {
    std::string name;
    std::vector<Entity> entities;
    std::vector<std::shared_ptr<Action>> init_actions;
    Storyboard storyboard;
};

} // namespace scena::ir
