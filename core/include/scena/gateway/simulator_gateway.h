// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>

#include "scena/entity_state.h"
#include "scena/entity_visibility.h"
#include "scena/ir/controller.h"

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
};

} // namespace scena::gateway
