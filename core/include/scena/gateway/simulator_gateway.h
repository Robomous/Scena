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

#include <string>

#include "scena/entity_state.h"
#include "scena/entity_visibility.h"
#include "scena/ir/controller.h"
#include "scena/runtime/scheduler.h"

namespace scena::gateway {

class IRoadQuery;

/// Integration boundary between the engine and a host simulator.
///
/// The gateway is optional: without one the engine runs self-contained and the
/// host exchanges entity states through Engine::state() / Engine::report_state()
/// directly. With a gateway attached, the engine drives the exchange itself
/// once per step, honoring per-entity control ownership:
///  - engine-controlled entities are pushed to the host via publish_state();
///  - host-controlled entities are pulled from the host via poll_state().
///
/// Implementations must not block and must not call back into the engine from
/// within these methods; the engine invokes them synchronously inside step().
class ISimulatorGateway {
public:
    virtual ~ISimulatorGateway() = default;

    /// Called once per step for every engine-controlled entity, after the
    /// engine has integrated its motion for the step.
    virtual void publish_state(const std::string& entity_id, const EntityState& state) = 0;

    /// Polled once per step for every host-controlled entity, before
    /// storyboard evaluation. Return true and fill `out` to update the
    /// entity's state; return false to leave it unchanged this step.
    virtual bool poll_state(const std::string& entity_id, EntityState& out) = 0;

    /// Road-network access provided by the host, or nullptr when no road data
    /// is available. Ownership stays with the gateway implementation.
    virtual IRoadQuery* road_query() = 0;

    /// Called when an AssignControllerAction assigns a controller model to an
    /// entity (ASAM OpenSCENARIO XML 1.4.0 §AssignControllerAction). Scena does
    /// not implement controller models — the name, type and properties are a
    /// contract between the scenario author and the host, so the engine hands
    /// them over verbatim and in document order.
    ///
    /// Called synchronously while the action is applied, inside the storyboard
    /// evaluation phase of step() — a fixed point in the step, so the order of
    /// these calls is part of the deterministic run. The reference is borrowed
    /// for the duration of the call.
    ///
    /// Defaulted to a no-op: gateways written before p5-s5 keep compiling and
    /// simply ignore the hand-off (an amendment to ADR-0003, see ADR-0014).
    virtual void on_controller_assigned(const std::string& /*entity_id*/,
                                        const ir::Controller& /*controller*/) {}

    /// Called when a VisibilityAction changes an entity's detectability
    /// (§VisibilityAction). Same timing, borrowing and default-no-op rules as
    /// on_controller_assigned. The engine has no image generator, sensors, or
    /// traffic participants of its own, so acting on this is the host's job;
    /// Engine::visibility_of reports the current flags either way.
    virtual void on_visibility_changed(const std::string& /*entity_id*/,
                                       const EntityVisibility& /*visibility*/) {}

    /// Called when a CustomCommandAction fires (§7.4.3; Annex A Table 12: it
    /// completes immediately). `type` and `content` are "defined as a contract
    /// between the simulation environment provider and the author of a
    /// scenario", so the engine hands them over verbatim and interprets
    /// neither.
    ///
    /// Same timing, borrowing and default-no-op rules as
    /// on_controller_assigned; also called during init for an init-phase
    /// action. A host that does not implement it is not an error — §7.4.3 makes
    /// executability depend on the environment "recognizing these actions", so
    /// a no-op host *is* the documented contract and the engine emits no
    /// diagnostic for it.
    ///
    /// Determinism: the host must not call back into the engine from this
    /// callback. Reactions feed back only through the sanctioned setters
    /// between steps.
    virtual void on_custom_command(const std::string& /*type*/, const std::string& /*content*/) {}

    /// Called once at the start of every step(), before anything else the
    /// gateway sees, with the dt the host passed.
    ///
    /// The batching hook: a host that writes into a scene graph, a shared
    /// buffer or a network frame opens it here, receives the step's
    /// poll_state()/publish_state() calls, and commits in on_step_end(). The
    /// per-entity contract and the step order of ADR-0003 are unchanged — this
    /// only brackets them, so a gateway that ignores both hooks behaves exactly
    /// as before.
    ///
    /// Called even for a zero-dt step: a dt = 0 step is a real evaluation (it
    /// re-checks triggers and republishes state) and a host batching writes
    /// needs its brackets.
    virtual void on_step_begin(double /*dt*/) {}

    /// Called once at the end of every step(), after the last publish_state().
    /// The commit half of on_step_begin(). Not called if the step is rejected
    /// (an invalid dt, an uninitialized engine): nothing was opened.
    virtual void on_step_end(double /*dt*/) {}

    /// Called for every storyboard element that took a transition in the
    /// evaluation this step performed — the observer half of the storyboard,
    /// so a host does not have to poll every element it cares about.
    ///
    /// `path` is the element's name path from the story down, joined with '/'
    /// (the empty string is the storyboard itself), the same addressing
    /// Engine::storyboard_element_state uses. `state` is the state the element
    /// is in *after* the transition.
    ///
    /// Order is deterministic and part of the contract: document order, depth
    /// first, parents before their children, reported after the evaluation
    /// completes and before entity motion is integrated. A transition is a
    /// one-evaluation pulse, so each is reported exactly once.
    ///
    /// Same borrowing and no-reentrancy rules as the other callbacks.
    virtual void on_element_transition(const std::string& /*path*/, runtime::ElementState /*state*/,
                                       runtime::TransitionKind /*transition*/) {}
};

} // namespace scena::gateway
