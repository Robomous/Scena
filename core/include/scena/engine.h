// SPDX-License-Identifier: MIT
#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "scena/diagnostic.h"
#include "scena/ir/scenario.h"
#include "scena/runtime/clock.h"
#include "scena/runtime/scheduler.h"
#include "scena/status.h"

namespace scena {

namespace gateway {
class ISimulatorGateway;
} // namespace gateway

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
/// randomness; entity updates and the storyboard walk iterate in
/// deterministic (sorted / document) order.
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

    /// Loads a scenario and resets simulation time to zero. All entities
    /// start with a zero-initialized state.
    ///
    /// The init phase runs here, per ASAM OpenSCENARIO XML 1.4.0 §8.5: init
    /// actions are applied before simulation time starts, then the
    /// storyboard enters runningState and is evaluated once at t = 0
    /// (§8.4.7), firing events whose start condition already holds.
    ///
    /// Validation walks the whole scenario in document order and reports
    /// every defect it finds through diagnostics(); it does not stop at the
    /// first one. Fails with ValidationError on structural defects (duplicate
    /// or empty entity ids, null actions, an event without actions, a
    /// negative maximumExecutionCount, empty or sibling-duplicate storyboard
    /// element names — element names address the state query, an empty
    /// condition group, a negative or NaN condition delay), and with
    /// SemanticError when an actor or an action targets an entity the
    /// scenario does not declare. The returned code is that of the first
    /// error diagnostic in document order; a failed init leaves the engine
    /// untouched and uninitialized.
    Status init(ir::Scenario scenario);

    /// Advances simulation time by dt seconds (dt >= 0). Order within a step:
    ///  1. the clock advances to t' = t + dt;
    ///  2. host-controlled entity states are polled from the gateway, if any;
    ///  3. the storyboard is evaluated at t': the stop trigger is checked,
    ///     standby elements whose start condition holds enter runningState
    ///     (their events fire actions), and completion propagates;
    ///  4. engine-controlled entities integrate kinematics over dt;
    ///  5. engine-controlled entity states are published to the gateway, if any.
    Status step(double dt);

    /// Current state of an entity, or std::nullopt when the id is unknown or
    /// the engine is not initialized.
    [[nodiscard]] std::optional<EntityState> state(const std::string& entity_id) const;

    /// Reports the authoritative state of a host-controlled entity. Fails with
    /// InvalidControlMode for engine-controlled entities.
    Status report_state(const std::string& entity_id, const EntityState& state);

    /// Lifecycle state of a storyboard element, addressed by its name path
    /// from the story down, joined with '/'
    /// (e.g. "story/act/group/maneuver/event"); the empty path addresses the
    /// storyboard itself. std::nullopt when the engine is not initialized or
    /// the path names no element.
    [[nodiscard]] std::optional<runtime::ElementState>
    storyboard_element_state(const std::string& path) const;

    /// Last monitorable transition of a storyboard element (same addressing
    /// as storyboard_element_state).
    [[nodiscard]] std::optional<runtime::TransitionKind>
    storyboard_element_transition(const std::string& path) const;

    /// Structured findings from the current scenario, in report order:
    /// validation findings from init() first, then runtime findings in the
    /// order the steps that produced them ran.
    ///
    /// The sink is cleared by init() (after the AlreadyInitialized guard, so
    /// a rejected re-init preserves the previous record) and by
    /// clear_diagnostics(). close() deliberately does not clear it, so a host
    /// can inspect a finished or failed run post mortem.
    ///
    /// Not every failure emits: AlreadyInitialized, NotInitialized, an
    /// invalid dt, and report_state() failures return a Status only. They
    /// describe host API misuse rather than scenario content, and emitting
    /// from them would let a polling loop grow the sink without bound.
    [[nodiscard]] const std::vector<Diagnostic>& diagnostics() const noexcept;

    /// Drops every collected diagnostic.
    void clear_diagnostics() noexcept;

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

    runtime::ActionOutcome apply(const ir::Action& action);

    gateway::ISimulatorGateway* gateway_ = nullptr;
    ir::Scenario scenario_;
    runtime::Clock clock_;
    runtime::Scheduler scheduler_;
    // std::map (not unordered_map) so per-step iteration order is deterministic.
    std::map<std::string, EntityRecord> entities_;
    DiagnosticSink diagnostics_;
    bool initialized_ = false;
};

} // namespace scena
