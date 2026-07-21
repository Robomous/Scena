// SPDX-License-Identifier: MIT
#pragma once

#include <map>
#include <optional>
#include <string>

#include "scena/ir/scenario.h"
#include "scena/runtime/clock.h"
#include "scena/runtime/scheduler.h"

namespace scena {

namespace gateway {
class ISimulatorGateway;
} // namespace gateway

/// Result codes for all engine operations. The public API reports failures
/// through these codes; no exceptions cross the API boundary.
enum class Status {
    Ok = 0,
    AlreadyInitialized,
    NotInitialized,
    UnknownEntity,
    InvalidControlMode,
    InvalidArgument,
};

/// Kinematic state of one entity in the world frame.
struct EntityState {
    double x = 0.0;       ///< World position, meters.
    double y = 0.0;       ///< World position, meters.
    double z = 0.0;       ///< World position, meters.
    double heading = 0.0; ///< Yaw around +Z, radians; 0 points along +X.
    double speed = 0.0;   ///< Longitudinal speed along the heading, m/s.
};

/// Step-based scenario execution engine.
///
/// The host simulator owns the clock: call init() once with a scenario, then
/// step(dt) at whatever rate the host dictates, query or report entity states
/// between steps, and close() when done. The engine spawns no threads and
/// imposes no main loop.
///
/// Control ownership is per entity: engine-controlled entities integrate
/// straight-line kinematics from speed and heading each step (placeholder
/// physics for this phase); host-controlled entities move only when the host
/// reports their state via report_state() or the gateway.
///
/// Determinism: identical scenario plus identical step sequence produces
/// bit-identical entity states. The engine reads no wall clock and uses no
/// randomness; entity updates iterate in a deterministic (sorted) order.
class Engine {
public:
    Engine() = default;

    /// Constructs an engine attached to a simulator gateway. The gateway is
    /// optional; without one the engine runs self-contained. The gateway must
    /// outlive the engine.
    explicit Engine(gateway::ISimulatorGateway* gateway);

    // The scheduler holds a pointer into the owned scenario, so the engine is
    // pinned to its address.
    Engine(const Engine&) = delete;
    Engine& operator=(const Engine&) = delete;
    Engine(Engine&&) = delete;
    Engine& operator=(Engine&&) = delete;

    ~Engine() = default;

    /// Loads a scenario and resets simulation time to zero. All entities start
    /// with a zero-initialized state. Fails with InvalidArgument on duplicate
    /// or empty entity ids and null storyboard pointers, and with
    /// UnknownEntity when an action targets a non-existent entity.
    Status init(ir::Scenario scenario);

    /// Advances simulation time by dt seconds (dt >= 0). Order within a step:
    ///  1. the clock advances to t' = t + dt;
    ///  2. host-controlled entity states are polled from the gateway, if any;
    ///  3. storyboard conditions are evaluated at t' and due actions fire once;
    ///  4. engine-controlled entities integrate kinematics over dt;
    ///  5. engine-controlled entity states are published to the gateway, if any.
    Status step(double dt);

    /// Current state of an entity, or std::nullopt when the id is unknown or
    /// the engine is not initialized.
    [[nodiscard]] std::optional<EntityState> state(const std::string& entity_id) const;

    /// Reports the authoritative state of a host-controlled entity. Fails with
    /// InvalidControlMode for engine-controlled entities.
    Status report_state(const std::string& entity_id, const EntityState& state);

    /// Simulation time in seconds since init().
    [[nodiscard]] double time() const noexcept;

    /// True between a successful init() and close().
    [[nodiscard]] bool initialized() const noexcept;

    /// Releases the scenario and returns the engine to the uninitialized
    /// state. The engine can be re-initialized afterwards.
    Status close();

private:
    struct EntityRecord {
        ir::ControlMode mode = ir::ControlMode::EngineControlled;
        EntityState state;
    };

    void apply(const ir::Action& action);

    gateway::ISimulatorGateway* gateway_ = nullptr;
    ir::Scenario scenario_;
    runtime::Clock clock_;
    runtime::Scheduler scheduler_;
    // std::map (not unordered_map) so per-step iteration order is deterministic.
    std::map<std::string, EntityRecord> entities_;
    bool initialized_ = false;
};

} // namespace scena
