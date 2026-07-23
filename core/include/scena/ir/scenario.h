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

#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "scena/ir/action.h"
#include "scena/ir/entity.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/traffic_signal.h"

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

    /// Traffic signal controllers (§6.11.2), declared in the RoadNetwork
    /// section of a scenario file and kept here in document order — that order
    /// is the one the engine ticks them in and is part of the deterministic
    /// run. Controller names are unique, which init() validates.
    std::vector<TrafficSignalController> traffic_signal_controllers;

    Storyboard storyboard;
};

} // namespace scena::ir
