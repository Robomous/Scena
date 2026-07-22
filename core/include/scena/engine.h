// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <functional>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "scena/diagnostic.h"
#include "scena/entity_state.h"
#include "scena/ir/action.h"
#include "scena/ir/date_time.h"
#include "scena/ir/scenario.h"
#include "scena/runtime/clock.h"
#include "scena/runtime/longitudinal.h"
#include "scena/runtime/scheduler.h"
#include "scena/status.h"

namespace scena {

namespace gateway {
class ISimulatorGateway;
} // namespace gateway

/// Step-based scenario execution engine.
///
/// The host simulator owns the clock: call init() once with a scenario, then
/// step(dt) at whatever rate the host dictates, query or report entity states
/// between steps, and close() when done. The engine spawns no threads and
/// imposes no main loop.
///
/// Control ownership is per entity: engine-controlled entities integrate
/// straight-line kinematics from speed and heading each step, where speed is
/// driven by a default longitudinal controller (SpeedAction / SpeedProfileAction
/// transition dynamics, clamped by the entity's Performance envelope — a
/// point-mass model, see docs/user-guide/motion.md); host-controlled entities
/// move only when the host reports their state via report_state() or the gateway.
///
/// Determinism: identical scenario plus identical step sequence produces
/// bit-identical entity states, on every platform and ISA. The engine reads no
/// wall clock and uses no randomness; entity updates and the storyboard walk
/// iterate in deterministic (sorted / document) order; floating-point
/// contraction is pinned off; and the kinematics integrator routes trig
/// through scena::runtime::det_sincos rather than platform libm. See
/// docs/user-guide/determinism.md for the full contract.
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
    ///  2b. derived observations are refreshed for every entity (both control
    ///     modes) from the snapshots conditions actually observe: acceleration
    ///     = (speed - prev_speed)/dt, the odometer accrues the Euclidean
    ///     displacement, and the standstill timer accumulates dt while speed is
    ///     exactly 0.0 and resets otherwise. Skipped entirely when dt == 0 (no
    ///     0/0, prev_sample untouched). This feeds the by-entity conditions
    ///     (§7.6.5.1); it is an addition to this list, not a reordering.
    ///  3. the storyboard is evaluated at t': the stop trigger is checked,
    ///     standby elements whose start condition holds enter runningState
    ///     (their events fire actions), in-progress transition-dynamics actions
    ///     are re-polled and advance the longitudinal controller (updating
    ///     speed to t'), and completion propagates;
    ///  4. engine-controlled entities integrate position from the updated speed
    ///     over dt;
    ///  5. engine-controlled entity states are published to the gateway, if any.
    Status step(double dt);

    /// Current state of an entity, or std::nullopt when the id is unknown or
    /// the engine is not initialized.
    [[nodiscard]] std::optional<EntityState> state(const std::string& entity_id) const;

    /// Reports the authoritative state of a host-controlled entity. Fails with
    /// InvalidControlMode for engine-controlled entities.
    Status report_state(const std::string& entity_id, const EntityState& state);

    /// Sets the current value of a global variable (§6.12), the host-side half
    /// of the VariableCondition interface. The variable must have been
    /// declared in the scenario (seeded into the runtime store at init);
    /// setting an undeclared name returns UnknownName and changes nothing.
    /// Fails with NotInitialized before init(). A rising-edge VariableCondition
    /// observes the change on the next step.
    Status set_variable(const std::string& name, std::string value);

    /// Current value of a global variable, or std::nullopt when the name is
    /// undeclared or the engine is not initialized.
    [[nodiscard]] std::optional<std::string> variable(const std::string& name) const;

    /// Creates or updates an externally supplied named value, the host-side
    /// half of the UserDefinedValueCondition interface. Unlike variables these
    /// names are not declared in the scenario, so any name is accepted and the
    /// call always succeeds. Values may be staged before init() — a value set
    /// beforehand is visible at the t = 0 evaluation — and persist across
    /// init(); close() clears them.
    Status set_user_defined_value(const std::string& name, std::string value);

    /// Current value of a user-defined value, or std::nullopt when it has not
    /// been set.
    [[nodiscard]] std::optional<std::string> user_defined_value(const std::string& name) const;

    /// Anchors the simulated time of day: `date_time` is taken to hold at the
    /// current simulation instant and advances one-for-one with simulation
    /// time thereafter (TimeOfDayCondition). Rejected with InvalidArgument
    /// when the DateTime is out of range. May be set before or after init and
    /// persists across init(); close() clears it. p5-s6's EnvironmentAction
    /// will feed the same anchor.
    Status set_date_time(const ir::DateTime& date_time);

    /// The current simulated instant as seconds since the Unix epoch, or
    /// std::nullopt when no time-of-day anchor has been set.
    [[nodiscard]] std::optional<double> date_time() const;

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
        // Optional geometry (p5-s3), copied once from the scenario at init and
        // immutable at runtime. The interaction conditions read it through the
        // entity-kinematics facet for freespace math; absent ⇒ freespace
        // metrics are a deterministic false.
        std::optional<ir::BoundingBox> bounding_box;
        // Performance envelope (p2-s2), present only on a Vehicle. Copied once
        // from the scenario at init; the default longitudinal controller clamps
        // transitions to it (max speed, and per-shape peak acceleration).
        std::optional<ir::Performance> performance;
        // Active longitudinal transition (p2-s2): the default controller
        // driving this entity's speed, and the action that owns it. An entity
        // has at most one; a new longitudinal action supersedes any previous.
        // active_longitudinal_action is compared by identity only (never
        // dereferenced for ordering), so it does not reach results.
        std::optional<runtime::LongitudinalController> longitudinal;
        const ir::Action* active_longitudinal_action = nullptr;
        // Active continuous relative speed target (p5-s4, §7.5.3): a
        // SpeedAction with a continuous RelativeTargetSpeed installs this and
        // no finite controller; each re-poll re-matches the reference entity's
        // speed. Mutually exclusive with `longitudinal`; both share
        // active_longitudinal_action so a new longitudinal action supersedes
        // either. Present ⇒ the action never completes on its own.
        std::optional<ir::RelativeTargetSpeed> continuous_speed;
        // Longitudinal actions superseded by a newer one on this entity but
        // still in their event's running set (p5-s4). On its next re-poll a
        // retired action reports Complete instead of reinstalling itself, so a
        // superseded ramp/continuous target does not fight the current owner
        // (§7.5.1 minimal single-domain conflict resolution; the full priority
        // catalog is #51). Membership is tested by identity only — never
        // iterated for a result — so it does not reach the determinism contract.
        std::vector<const ir::Action*> retired_longitudinal;
        // Derived observation state for the by-entity conditions (p5-s2),
        // written only by init seeding and the phase-2b refresh. Every member
        // is default-initialized: an uninitialized read would be a determinism
        // bug.
        bool has_prev_sample = false;
        EntityState prev_sample;            ///< Snapshot the previous evaluation observed.
        std::optional<double> acceleration; ///< m/s^2; absent until the first dt>0 refresh.
        double traveled_distance = 0.0;     ///< m, cumulative path length since init.
        double standstill_seconds = 0.0;    ///< s, contiguous time at speed == 0.0.
    };

    /// Applies an action on its first fire and re-polls it on later steps (the
    /// scheduler's fire callback). Longitudinal actions install and advance a
    /// LongitudinalController; other kinds keep their one-shot behaviour.
    runtime::ActionOutcome apply(const ir::Action& action);

    /// The record `action` targets, or nullptr when the entity is unknown —
    /// which init() already rejects, so the lookup failing is defensive: it
    /// reports a Warning and the caller skips the action.
    EntityRecord* record_for(const ir::Action& action);

    /// Drives one step of a LongitudinalDistanceAction on `record`: measures the
    /// current gap to the reference entity, commands the speed that closes it
    /// over the current step, and clamps that command by the action's
    /// DynamicConstraints and the entity's Performance envelope (ADR-0014).
    /// Returns Complete once a non-continuous action has reached its target;
    /// a continuous one returns Running forever (§7.5.3).
    runtime::ActionOutcome drive_distance_keeping(const ir::LongitudinalDistanceAction& action,
                                                  EntityRecord& record);

    /// Installs or advances the default longitudinal controller for a
    /// longitudinal action on `record`, returning its outcome (§7.4.1.2).
    runtime::ActionOutcome drive_longitudinal(const ir::Action& action, EntityRecord& record,
                                              runtime::LongitudinalController controller);

    /// Retires the longitudinal action currently owning `record` when a
    /// different `incoming` action supersedes it, so the outgoing action reports
    /// Complete on its next re-poll instead of fighting for the entity (p5-s4,
    /// §7.5.1 minimal conflict resolution). No-op when nothing is owned or the
    /// owner is `incoming` itself.
    void supersede_longitudinal(EntityRecord& record, const ir::Action* incoming);

    /// Builds the controller for a SpeedAction from `record`'s current speed,
    /// clamped by its Performance envelope (max speed and per-shape peak
    /// acceleration).
    [[nodiscard]] runtime::LongitudinalController
    build_speed_controller(const ir::SpeedAction& action, const EntityRecord& record) const;

    /// Resolves a RelativeTargetSpeed against its reference entity's current
    /// speed: delta ⇒ ref + value, factor ⇒ ref * value (§RelativeTargetSpeed).
    /// The reference is read from the entity table at call time — deterministic
    /// given the fixed storyboard/scheduler order (validated to exist at init).
    /// No Performance clamp is applied here; callers clamp as needed.
    [[nodiscard]] double resolve_relative_speed(const ir::RelativeTargetSpeed& target,
                                                const EntityRecord& record) const;

    /// Builds the controller for a SpeedProfileAction: one linear
    /// (position-mode) segment per entry, chained from `record`'s current
    /// speed. An entry with no time is reached as fast as the Performance
    /// envelope allows.
    [[nodiscard]] runtime::LongitudinalController
    build_profile_controller(const ir::SpeedProfileAction& action,
                             const EntityRecord& record) const;

    /// Snaps a just-installed init-action transition to its terminal value
    /// (§8.5: init actions are instantaneous).
    void finalize_longitudinal(const ir::Action& action, EntityRecord& record);

    /// The simulated instant (epoch seconds) at `simulation_time`, or nullopt
    /// when no time-of-day anchor has been set.
    [[nodiscard]] std::optional<double> current_date_time_seconds(double simulation_time) const;

    /// Phase-2b refresh of the derived observations (acceleration, odometer,
    /// standstill timer) for every entity from the current vs. previous
    /// snapshot, over `dt` (> 0). See step()'s documentation for the invariant.
    void refresh_observations(double dt);

    gateway::ISimulatorGateway* gateway_ = nullptr;
    ir::Scenario scenario_;
    runtime::Clock clock_;
    runtime::Scheduler scheduler_;
    // Duration of the step currently being processed, set at the top of step().
    // The scheduler's fire callback re-polls a longitudinal action without a dt
    // argument, so the controller reads the current step from here. 0 during
    // init and outside a step.
    double last_dt_ = 0.0;
    // std::map (not unordered_map) so per-step iteration order is deterministic.
    // std::less<> gives heterogeneous string_view lookup for the entity
    // facet without changing the (sorted) iteration order.
    std::map<std::string, EntityRecord, std::less<>> entities_;
    // Runtime variable store, seeded from scenario_.variables at init and
    // mutable through set_variable during the run (§6.12). std::less<> gives
    // heterogeneous lookup on a string_view. Cleared by close().
    std::map<std::string, std::string, std::less<>> variables_;
    // Host-supplied external values (UserDefinedValueCondition). Persist across
    // init() so a host can stage them before the run; cleared only by close().
    std::map<std::string, std::string, std::less<>> user_defined_values_;
    // Names already warned about during evaluation (unknown user value, unset
    // time of day), so each warns at most once. Keyed by diagnostic path;
    // cleared at init().
    std::set<std::string> warned_values_;
    // Time-of-day anchor: the simulated instant `anchor_epoch` (epoch seconds)
    // holds at simulation time `anchor_sim`, advancing one-for-one after that.
    // Persists across init(); cleared by close().
    bool date_time_set_ = false;
    double date_time_anchor_epoch_ = 0.0;
    double date_time_anchor_sim_ = 0.0;
    DiagnosticSink diagnostics_;
    bool initialized_ = false;
};

} // namespace scena
