// SPDX-License-Identifier: MIT
#pragma once

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "scena/ir/action.h"
#include "scena/ir/entity.h"
#include "scena/ir/storyboard.h"

namespace scena::ir {

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

    /// Global parameters (§9.1): named string values assigned at load time and
    /// immutable at runtime, so a ParameterCondition's result is constant over
    /// a run. Typed declarations arrive with p4-s3; the by-value conditions
    /// compare stringly, matching the ParameterCondition XSD. std::less<>
    /// gives heterogeneous lookup so validation and evaluation can query with
    /// a std::string_view without allocating.
    std::map<std::string, std::string, std::less<>> parameters;

    /// Global variables (§6.12): named string values with an initialization
    /// value that the engine seeds into its runtime store at init and that a
    /// VariableAction or the host may change during the run.
    std::map<std::string, std::string, std::less<>> variables;

    Storyboard storyboard;
};

} // namespace scena::ir
