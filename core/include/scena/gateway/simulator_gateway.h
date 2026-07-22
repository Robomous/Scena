// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>

#include "scena/entity_state.h"

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
};

} // namespace scena::gateway
