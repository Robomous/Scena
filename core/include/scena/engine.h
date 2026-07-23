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
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "scena/diagnostic.h"
#include "scena/entity_state.h"
#include "scena/entity_visibility.h"
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

/// Lane width the engine assumes when no road network can supply a real one,
/// in metres. A modeling default, not a value the standard prescribes: ASAM
/// OpenSCENARIO defines lanes in the road network, which is external to it.
/// Change it with Engine::set_default_lane_width (ADR-0016).
inline constexpr double kDefaultLaneWidth = 3.5;

/// Which movement domains of an entity the engine currently controls, per ASAM
/// OpenSCENARIO XML 1.4.0 §ActivateControllerAction. Both start active: an
/// entity with no controller action is driven by the engine's default
/// controller in both domains. Deactivating a domain releases the engine's
/// control of it and suppresses actions targeting it (ADR-0014).
struct ControllerActivation {
    bool lateral = true;
    bool longitudinal = true;
};

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
    ///     over dt. An entity with a live lateral axis (p2-s3) composes that
    ///     advance with the lateral offset increment phase 3 commanded and
    ///     blends its heading from the two; every other entity integrates the
    ///     plain straight-line model, bit-for-bit as before (ADR-0016);
    ///  5. engine-controlled entity states are published to the gateway, if any.
    Status step(double dt);

    /// Current state of an entity, or std::nullopt when the id is unknown, the
    /// entity is not currently in the scenario (a DeleteEntityAction removed
    /// it, §EntityAction), or the engine is not initialized.
    [[nodiscard]] std::optional<EntityState> state(const std::string& entity_id) const;

    /// Whether a declared entity is currently in the scenario (§EntityAction),
    /// or std::nullopt when the id is not declared at all. Every entity starts
    /// active; a DeleteEntityAction makes it inactive and an AddEntityAction
    /// brings it back. An inactive entity does not move, is not published to
    /// the gateway, and is invisible to the by-entity conditions.
    [[nodiscard]] std::optional<bool> entity_active(const std::string& entity_id) const;

    /// The route currently assigned to an entity (§6.8.2), or nullptr when it
    /// has none, the id is unknown or inactive, or the engine is not
    /// initialized. The same "unknown or inactive" rule applies to
    /// assigned_controller_of, controller_activation_of and visibility_of: an
    /// entity that is not in the scenario reports nothing. A route
    /// is installed by an AssignRouteAction or an AcquirePositionAction and
    /// stays until another routing action overwrites it. The pointer is
    /// borrowed and is invalidated by the next routing action on that entity,
    /// by close(), and by init().
    [[nodiscard]] const ir::Route* route_of(const std::string& entity_id) const;

    /// The controller assigned to an entity by the last AssignControllerAction
    /// (§AssignControllerAction), or nullptr when it has none, the id is
    /// unknown, or the engine is not initialized. Borrowed with the same
    /// lifetime rules as route_of.
    [[nodiscard]] const ir::Controller* assigned_controller_of(const std::string& entity_id) const;

    /// Which movement domains the engine currently controls for an entity, or
    /// std::nullopt when the id is unknown. Both are active until an
    /// ActivateControllerAction (or an AssignControllerAction's activation
    /// flags) says otherwise.
    [[nodiscard]] std::optional<ControllerActivation>
    controller_activation_of(const std::string& entity_id) const;

    /// Current detectability of an entity (§VisibilityAction), or std::nullopt
    /// when the id is unknown. Visible everywhere until a VisibilityAction
    /// changes it.
    [[nodiscard]] std::optional<EntityVisibility> visibility_of(const std::string& entity_id) const;

    /// Reports the authoritative state of a host-controlled entity. Fails with
    /// InvalidControlMode for engine-controlled entities, and with
    /// UnknownEntity when the entity is not currently in the scenario — a
    /// deleted entity has no state to report (§EntityAction).
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
    /// persists across init(); close() clears it.
    ///
    /// The host setter always anchors an *advancing* clock. An
    /// EnvironmentAction feeds the same anchor and may freeze it instead
    /// (§TimeOfDay animation), which is the one difference between the two
    /// routes to the simulated instant.
    Status set_date_time(const ir::DateTime& date_time);

    /// The current simulated instant as seconds since the Unix epoch, or
    /// std::nullopt when no time-of-day anchor has been set. Frozen at the
    /// anchor while a non-animated §TimeOfDay is in force.
    [[nodiscard]] std::optional<double> date_time() const;

    /// Sets the lane width a LaneChangeAction assumes when the target lane
    /// cannot be resolved against a road network (ADR-0016). Rejected with
    /// InvalidArgument when `width` is not finite and positive. May be set
    /// before or after init and persists across init(); close() restores
    /// kDefaultLaneWidth.
    ///
    /// A host with an IRoadQuery that answers the lane queries never sees this
    /// value: real lane widths win.
    Status set_default_lane_width(double width);

    /// The lane width the flat-world lane model currently uses, in metres.
    [[nodiscard]] double default_lane_width() const noexcept;

    /// The current observable state of a traffic signal (§6.11.4), or
    /// std::nullopt when nothing has written that signal id yet — no
    /// controller phase names it and no TrafficSignalStateAction has forced
    /// it. Signal ids are free-form road-network references, so an unknown id
    /// is indistinguishable from an unwritten one.
    [[nodiscard]] std::optional<std::string> traffic_signal_state(const std::string& name) const;

    /// The name of the phase a traffic signal controller is currently in
    /// (§6.11.4), or std::nullopt when the controller is unknown, has no
    /// phases, or has not started yet — a controller with a `delay` has no
    /// phase until its start offset elapses (§6.11.3).
    [[nodiscard]] std::optional<std::string>
    traffic_signal_controller_phase(const std::string& name) const;

    /// The environment state accumulated by the EnvironmentActions applied so
    /// far (§Environment). Every member starts absent and an action merges in
    /// only what it carries, so an absent member means "never set", which is
    /// the same thing the standard's "doesn't change" amounts to at t = 0.
    /// The reference is borrowed and is invalidated by the next
    /// EnvironmentAction, by init(), and by close().
    [[nodiscard]] const ir::Environment& environment() const noexcept;

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
    /// The precomputed state of an entity following a polyline trajectory
    /// (§6.9), built once when the FollowTrajectoryAction starts.
    ///
    /// Everything the follower needs is resolved at install time — the segment
    /// lengths, the cumulative arc length, the per-segment heading (the one
    /// det_atan2 site), the effective vertex times — so a step only
    /// interpolates. `action` is compared by identity and never dereferenced
    /// for ordering, exactly like active_longitudinal_action.
    struct TrajectoryFollower {
        const ir::Action* action = nullptr;
        std::vector<ir::WorldPosition> points; ///< Vertices, after the initial-offset truncation.
        std::vector<double> arc;               ///< Cumulative arc length at each vertex [m].
        std::vector<double> heading;           ///< Heading of each segment [rad], size points-1.
        std::vector<double> times; ///< Effective vertex times [s]; empty without Timing.
        bool timing = false;       ///< True ⇒ the vertex times drive the motion.
        bool started = false;      ///< The entity has been placed on the trajectory.
        double traveled = 0.0;     ///< Arc length covered [m] (time-free mode).
    };

    /// The straight reference line an in-progress lateral action displaces an
    /// entity from (ADR-0016): the flat-world stand-in for a lane centre line
    /// until a road backend can supply the real one (#23).
    ///
    /// Captured once, when a lateral action first needs it, from the entity's
    /// position and heading at that instant. While it is live the entity's
    /// motion is composed of a longitudinal advance along the axis and a
    /// lateral displacement across it, and its heading is blended from the two
    /// — which is what turns an offset ramp into a plausible lane change.
    /// `cos`/`sin` come from one det_sincos of `heading` and are cached so the
    /// per-step composition adds no trigonometry.
    struct LateralAxis {
        double origin_x = 0.0; ///< World anchor: where the axis passes through.
        double origin_y = 0.0;
        double heading = 0.0; ///< Axis direction [rad].
        double cos = 1.0;     ///< det_sincos(heading).cos, cached once.
        double sin = 0.0;     ///< det_sincos(heading).sin, cached once.
    };

    struct EntityRecord {
        ir::ControlMode mode = ir::ControlMode::EngineControlled;
        EntityState state;
        // Whether the entity is currently in the scenario (§EntityAction).
        // A DeleteEntityAction clears this and an AddEntityAction sets it
        // again; the map's structure never changes at runtime, so entity
        // iteration order and per-record bookkeeping are unaffected — which is
        // exactly why the lifecycle is a flag and not an erase/insert.
        bool active = true;
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
        // Actions superseded by a newer one on this entity but still in their
        // event's running set (p5-s4). On its next re-poll a retired action
        // reports Complete instead of reinstalling itself, so a superseded
        // ramp, continuous target or trajectory does not fight the current
        // owner (§7.5.1 minimal single-domain conflict resolution; the full
        // priority catalog is #51). Membership is tested by identity only —
        // never iterated for a result — so it does not reach the determinism
        // contract.
        std::vector<const ir::Action*> retired_actions;
        // Lateral motion state (p2-s3, ADR-0016). The axis is live exactly
        // while an offset-model lateral action is displacing this entity;
        // without it the entity integrates the plain straight-line model, which
        // is what keeps every pre-lateral trace bit-identical.
        //
        // `lateral_transition` is the shaped offset ramp (the same
        // value-transition sequencer the longitudinal domain uses, over metres
        // of offset). `active_lateral_action` owns the single lateral slot and
        // is compared by identity only, never dereferenced for ordering, so it
        // does not reach results — exactly like active_longitudinal_action.
        std::optional<LateralAxis> lateral_axis;
        std::optional<runtime::LongitudinalController> lateral_transition;
        const ir::Action* active_lateral_action = nullptr;
        // Offset from the axis actually applied so far, and the offset the
        // storyboard phase commanded for this step; the integrate phase moves
        // the entity by their difference and then makes them equal. Positive is
        // left of the axis (§7.4.1.4, ISO 8855).
        double lateral_offset = 0.0;
        double lateral_offset_command = 0.0;
        // Lateral rate [m/s] carried across steps by a constrained
        // LateralDistanceAction, whose controller limits how fast the offset
        // may change. Unused by the ramp-driven actions.
        double lateral_rate = 0.0;
        // Set when a lateral action reached its goal during the storyboard
        // phase: the integrate phase still applies that step's last offset
        // increment, then snaps the heading back to the axis and dissolves the
        // axis. Dissolving in the storyboard phase would drop the increment.
        bool lateral_dissolve_pending = false;
        // The route assigned by the last AssignRouteAction or
        // AcquirePositionAction, which persists "until another action
        // overwrites them" (§6.8.2). Stored, not yet followed: route-following
        // motion needs a road network (p3-s4).
        std::optional<ir::Route> route;
        // Active trajectory follower (§6.9). Present ⇒ this entity's position
        // comes from the trajectory and the straight-line integrator is skipped
        // for the step the follower wrote.
        std::optional<TrajectoryFollower> trajectory;
        // Set when the trajectory follower wrote this entity's position during
        // the current step's storyboard evaluation, and consumed by the
        // integrate phase of that same step. It is the one-step hand-off that
        // keeps the phase order of step() intact (ADR-0014).
        bool trajectory_moved = false;
        // The controller model assigned by the last AssignControllerAction.
        // Scena implements no controller models: this is stored, handed to the
        // gateway, and readable by the host (§AssignControllerAction).
        std::optional<ir::Controller> assigned_controller;
        // Which movement domains the engine's default controller currently
        // drives (§ActivateControllerAction). Deactivating a domain releases
        // the engine's control of it; actions targeting it are then suppressed.
        ControllerActivation activation;
        // Detectability (§VisibilityAction): visible everywhere by default.
        EntityVisibility visibility;
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

    /// Applies an actor-less §7.4.2 / §7.4.3 action, or std::nullopt when the
    /// kind is not implemented (the caller then reports it like any other
    /// unsupported kind). Every global action completes in the evaluation it
    /// fires (Annex A Tables 11 and 12), so the outcome is always Complete;
    /// the optional distinguishes "handled" from "unknown", not the lifetime.
    std::optional<runtime::ActionOutcome> apply_global_action(const ir::GlobalAction& action);

    /// Reports `message` once per `path` during a run, through the same
    /// warned_values_ dedup the evaluation-time facet warnings use. Repeated
    /// applications of the same action (a re-executed event) must not grow the
    /// diagnostic sink without bound.
    void warn_once(std::string path, Status code, std::string message);

    /// The record `action` targets, or nullptr when the entity is unknown —
    /// which init() already rejects, so the lookup failing is defensive: it
    /// reports a Warning and the caller skips the action.
    ///
    /// Also nullptr when the entity is not currently in the scenario: a
    /// private action needs "a valid entity" as its actor (§7.5.2.2), so a
    /// DeleteEntityAction stops whatever was driving it. Every private-action
    /// branch maps a null record to Complete, which is how the owning event
    /// ends — one evaluation later than the spec's action-level stopTransition,
    /// because Scena has no per-action observable transitions (ADR-0015).
    EntityRecord* record_for(const ir::Action& action);

    /// Takes an entity out of the scenario (§DeleteEntityAction): clears every
    /// piece of runtime motion, assignment and derived-observation state while
    /// keeping the declared immutables (control mode, geometry, performance),
    /// so a later AddEntityAction restarts from a clean slate.
    void deactivate_entity(EntityRecord& record);

    /// Puts an entity into the scenario at `position` (§AddEntityAction): a
    /// fresh state, and the derived-observation baseline seeded from it exactly
    /// as init() seeds it, so the first step after the add reports no phantom
    /// acceleration.
    void activate_entity(EntityRecord& record, const ir::WorldPosition& position);

    /// Drives one step of a LongitudinalDistanceAction on `record`: measures the
    /// current gap to the reference entity, commands the speed that closes it
    /// over the current step, and clamps that command by the action's
    /// DynamicConstraints and the entity's Performance envelope (ADR-0014).
    /// Returns Complete once a non-continuous action has reached its target;
    /// a continuous one returns Running forever (§7.5.3).
    runtime::ActionOutcome drive_distance_keeping(const ir::LongitudinalDistanceAction& action,
                                                  EntityRecord& record);

    /// Drives one step of a FollowTrajectoryAction on `record`: installs the
    /// follower on the first fire (resolving segment lengths, headings and
    /// effective vertex times) and writes the entity's position and heading
    /// from it on every fire. Returns Complete at the end of the trajectory
    /// (Annex A Table 10).
    runtime::ActionOutcome drive_trajectory(const ir::FollowTrajectoryAction& action,
                                            EntityRecord& record);

    /// Installs or advances the default longitudinal controller for a
    /// longitudinal action on `record`, returning its outcome (§7.4.1.2).
    runtime::ActionOutcome drive_longitudinal(const ir::Action& action, EntityRecord& record,
                                              runtime::LongitudinalController controller);

    /// Releases the engine's control of a domain on `record`, retiring whatever
    /// action owned it so that action completes on its next re-poll (the
    /// §7.5.2.1 override path). The entity keeps the state it has: a released
    /// longitudinal domain holds the current speed, a released lateral one
    /// stops following its trajectory where it stands.
    void release_longitudinal_domain(EntityRecord& record);
    void release_lateral_domain(EntityRecord& record);

    /// Applies the tri-state activation flags of a controller action to
    /// `record`: an unset flag means "no change for controlling that domain",
    /// activating restores normal dispatch, deactivating releases the domain.
    void apply_activation(EntityRecord& record, const std::optional<bool>& lateral,
                          const std::optional<bool>& longitudinal);

    /// Reports that `action` was fired while one of the domains it needs is
    /// deactivated, and skips it (the missing-prerequisite analog of §7.5.2.2).
    void report_inactive_domain(const ir::Action& action, const char* domain);

    /// Retires the longitudinal action currently owning `record` when a
    /// different `incoming` action supersedes it, so the outgoing action reports
    /// Complete on its next re-poll instead of fighting for the entity (p5-s4,
    /// §7.5.1 minimal conflict resolution). No-op when nothing is owned or the
    /// owner is `incoming` itself.
    void supersede_longitudinal(EntityRecord& record, const ir::Action* incoming);

    /// The lateral counterpart: retires whichever action owns `record`'s
    /// lateral domain — the offset-model slot or a trajectory follower, both of
    /// which "assign a lateral control strategy" (Annex A Table 10) — when a
    /// different `incoming` action supersedes it.
    ///
    /// The axis is deliberately left standing: a lateral action superseding
    /// another continues on the same reference line, so an absolute lane offset
    /// keeps meaning the same thing across the handover. Callers that are not
    /// offset-model actions (a trajectory) dissolve it themselves.
    void supersede_lateral(EntityRecord& record, const ir::Action* incoming);

    /// Captures the lateral axis from `record`'s current pose, if none is live:
    /// the axis runs through the entity, along its heading, and the entity
    /// starts at offset zero on it. Zero encodes the flat-world assumption that
    /// an actor sits on its lane centre when a lateral action starts; a real
    /// in-lane offset needs a road backend (#23).
    void ensure_lateral_axis(EntityRecord& record);

    /// Drops the lateral axis and every offset the entity accumulated on it,
    /// leaving position and heading exactly where they are.
    void dissolve_lateral_axis(EntityRecord& record) noexcept;

    /// Drives one step of a LaneOffsetAction on `record` (Class
    /// `LaneOffsetAction`): installs a shaped offset ramp whose duration comes
    /// from maxLateralAcc, and commands the offset it reaches this step.
    /// Returns Complete once a non-continuous action has reached its target; a
    /// continuous one returns Running forever and re-corrects whenever the
    /// target moves (§7.5.3).
    runtime::ActionOutcome drive_lane_offset(const ir::LaneOffsetAction& action,
                                             EntityRecord& record, bool installing);

    /// Drives one step of a LaneChangeAction on `record` (Class
    /// `LaneChangeAction`): resolves the target lane once at install, then runs
    /// the offset ramp its TransitionDynamics describe — over seconds for the
    /// Time and Rate dimensions, over metres travelled for Distance. Returns
    /// Complete on "reaching the lateral centerline offset on the target lane"
    /// (Annex A Table 10).
    runtime::ActionOutcome drive_lane_change(const ir::LaneChangeAction& action,
                                             EntityRecord& record, bool installing);

    /// The offset across the live axis that `action`'s target lane sits at, or
    /// std::nullopt when it cannot be resolved and the action must stop.
    ///
    /// A relative target is resolved against the road network when the gateway
    /// offers one — actor and reference lane positions, the lane `count` lanes
    /// over, and both lane centre offsets — and otherwise against the
    /// flat-world model: `count` default lane widths from the reference
    /// entity's lateral position. An absolute lane id has no flat-world
    /// reading, so without a backend that answers it there is nothing to
    /// resolve (#23).
    [[nodiscard]] std::optional<double>
    resolve_lane_change_target(const ir::LaneChangeAction& action,
                               const EntityRecord& record) const;

    /// The lateral offset of `record`'s reference entity measured across the
    /// live axis, i.e. `(p_ref - axis.origin) . axis_normal`. Flat-world: every
    /// entity is assumed to sit on its own lane centre, so this doubles as the
    /// reference's lane position. Requires a live axis.
    [[nodiscard]] double reference_lateral_offset(const EntityRecord& record,
                                                  const std::string& entity_ref) const;

    /// Snaps a just-installed init-phase lateral transition to its terminal
    /// offset with one perpendicular translate, then dissolves the axis (§8.5:
    /// init actions are instantaneous, so no heading blending happens).
    void finalize_lateral(const ir::Action& action, EntityRecord& record);

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

    /// The live cycle state of one traffic signal controller (§6.11).
    ///
    /// The phase is derived arithmetically from simulation time rather than
    /// accumulated across steps: `fmod` is exact in IEEE, so a controller's
    /// phase at time t depends only on t and never drifts with the host's step
    /// pattern. `controller` is borrowed from the owned scenario_ and is never
    /// dereferenced for ordering.
    struct SignalControllerRuntime {
        const ir::TrafficSignalController* controller = nullptr;
        /// Cumulative phase-end offsets [0, d0, d0+d1, ..., total], summed
        /// once in declared order so the partial sums are fixed.
        std::vector<double> cumulative;
        /// Cycle duration, the sum of the phase durations (§6.11.4).
        double total = 0.0;
        /// Simulation time at which phase[0] first starts: the transitively
        /// resolved delay chain of §6.11.3. Zero for an unchained controller.
        double start_offset = 0.0;
        /// Simulation time the current cycle's phase[0] started. Equal to
        /// start_offset until a TrafficSignalControllerAction re-anchors it.
        double anchor = 0.0;
        /// The phase index whose states were last written, or absent when the
        /// controller has not started (or was just re-anchored).
        std::optional<std::size_t> applied_phase;
    };

    /// Ticks every traffic signal controller to simulation time `t`, writing
    /// the signal states of any phase it has just entered (§6.11.4).
    ///
    /// Runs at the top of the storyboard-evaluation phase, so a
    /// TrafficSignalStateAction fired by the storyboard writes *after* the
    /// controllers and its forced state survives until the next phase
    /// transition — the actions-win precedence the §11.12 example needs.
    void advance_signal_controllers(double t);

    /// Writes the signal states and the observable phase name of `runtime`'s
    /// phase `index`, in the phase's document order.
    void apply_signal_phase(const std::string& controller_name, SignalControllerRuntime& runtime,
                            std::size_t index);

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
    // Runtime overlay over scenario_.parameters, written only by the deprecated
    // ParameterSetAction / ParameterModifyAction (§ParameterSetAction,
    // deprecated with 1.2). A 1.0/1.1 file's parameter action must be visible to
    // the ParameterCondition that reads it, and the overlay gives it that
    // without breaking §9.1 immutability for 1.2+ content — which cannot
    // contain these actions at all. Consulted before scenario_.parameters;
    // cleared by init() and close().
    std::map<std::string, std::string, std::less<>> parameter_overrides_;
    // Host-supplied external values (UserDefinedValueCondition). Persist across
    // init() so a host can stage them before the run; cleared only by close().
    std::map<std::string, std::string, std::less<>> user_defined_values_;
    // Names already warned about during evaluation (unknown user value, unset
    // time of day), so each warns at most once. Keyed by diagnostic path;
    // cleared at init().
    std::set<std::string> warned_values_;
    // Time-of-day anchor: the simulated instant `anchor_epoch` (epoch seconds)
    // holds at simulation time `anchor_sim`, advancing one-for-one after that
    // while `animated` is true and standing still while it is false
    // (§TimeOfDay animation). Persists across init(); cleared by close().
    bool date_time_set_ = false;
    double date_time_anchor_epoch_ = 0.0;
    double date_time_anchor_sim_ = 0.0;
    bool date_time_animated_ = true;
    // Flat-world lane width for target-lane resolution without a road network
    // (ADR-0016). Host-settable state like the time-of-day anchor: persists
    // across init(), restored to the default by close().
    double default_lane_width_ = kDefaultLaneWidth;
    // Environment state (§Environment), merged member by member by every
    // EnvironmentAction. Per-run state: cleared by init() and close().
    ir::Environment environment_;
    // Observable traffic signal states, keyed by road-network signal id
    // (§6.11.4). Written by a controller's phase transitions and by
    // TrafficSignalStateAction, and read by TrafficSignalCondition. std::map
    // with std::less<> for deterministic iteration and heterogeneous lookup.
    std::map<std::string, std::string, std::less<>> signal_states_;
    // Current phase name per controller, the observable half of
    // signal_controllers_ that TrafficSignalControllerCondition reads. A
    // controller with no phase yet (before its delayed start) has no entry.
    std::map<std::string, std::string, std::less<>> controller_phases_;
    // Live cycle state per controller name, ticked once per step.
    std::map<std::string, SignalControllerRuntime, std::less<>> signal_controllers_;
    DiagnosticSink diagnostics_;
    bool initialized_ = false;
};

} // namespace scena
