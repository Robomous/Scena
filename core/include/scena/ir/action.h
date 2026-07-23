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

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "scena/ir/controller.h"
#include "scena/ir/coordinate_system.h"
#include "scena/ir/dynamics.h"
#include "scena/ir/environment.h"
#include "scena/ir/position.h"
#include "scena/ir/route.h"
#include "scena/ir/trajectory.h"

namespace scena::ir {

/// Base class for all scenario actions in the Scenario IR.
///
/// An action's lifetime is reported by the runtime as an
/// `runtime::ActionOutcome`: a step-shaped action completes in the evaluation
/// it fires, while an action governed by transition dynamics (a speed ramp)
/// stays in progress across steps until it reaches its target.
class Action {
public:
    virtual ~Action() = default;

    /// Identifier of the entity this action applies to.
    [[nodiscard]] virtual const std::string& entity_id() const = 0;

    /// Stable, human-readable name of the action kind, matching the ASAM
    /// element name (e.g. "SpeedAction"). Used in runtime diagnostics when an
    /// action kind is not yet implemented. Deliberately not typeid().name(),
    /// whose output is mangled and platform-dependent — the determinism
    /// contract requires a stable string.
    [[nodiscard]] virtual std::string_view kind() const noexcept = 0;
};

/// Base of the actor-less actions: the §7.4.2 global actions ("used to set or
/// modify non-entity-related quantities") and the §7.4.3 user-defined
/// CustomCommandAction.
///
/// A global action has no actor entity, so entity_id() is finalized here to a
/// static empty string — every existing call site keeps a valid reference.
/// Dispatch and validation branch on this base (a dynamic_cast), never on the
/// empty string: an entity id is scenario content and must not carry meaning by
/// being empty. Global actions are legal both in the init phase (§8.5) and in
/// storyboard events, and both routes reach the same Engine::apply.
class GlobalAction : public Action {
public:
    /// Always the empty string: a global action targets no entity.
    [[nodiscard]] const std::string& entity_id() const final;
};

/// How a RelativeTargetSpeed value combines with the reference entity's speed.
/// Per ASAM OpenSCENARIO XML 1.4.0 §SpeedTargetValueType.
enum class SpeedTargetValueType {
    /// The value is added to the reference speed: target = ref + value [m/s].
    Delta,
    /// The value scales the reference speed: target = ref * value (unitless).
    Factor,
};

/// A SpeedAction target defined relative to another entity's speed. Per ASAM
/// OpenSCENARIO XML 1.4.0 §RelativeTargetSpeed.
///
/// With `continuous == false` the relative target is resolved once, against the
/// reference entity's speed when the action starts, and reached through the
/// action's TransitionDynamics like an absolute target. With
/// `continuous == true` a controller keeps matching the reference continuously
/// and the action never ends by itself (§7.5.3); per the standard this must not
/// be combined with a time- or distance-dimensioned transition (⇒ Step).
struct RelativeTargetSpeed {
    /// Name of the reference entity whose speed the target is relative to.
    std::string entity_ref;
    /// Delta [m/s] or factor [1] depending on `value_type`.
    double value = 0.0;
    /// Whether `value` is a delta or a factor.
    SpeedTargetValueType value_type = SpeedTargetValueType::Delta;
    /// If true, the target is continuously maintained and the action never ends
    /// on its own (§7.5.3); only a stopTransition (or a superseding longitudinal
    /// action) ends it.
    bool continuous = false;
};

/// Transitions the longitudinal speed of an entity to a target speed, governed
/// by transition dynamics. Per ASAM OpenSCENARIO XML 1.4.0 §SpeedAction. The
/// target is either an absolute speed (§AbsoluteTargetSpeed) or a speed relative
/// to another entity (§RelativeTargetSpeed); the two are mutually exclusive
/// (§SpeedActionTarget is a union).
class SpeedAction final : public Action {
public:
    /// Step (instantaneous) absolute speed change: reaches `target_speed` in the
    /// evaluation it fires. The back-compatible short form, equivalent to the
    /// dynamics form with a Step shape.
    SpeedAction(std::string entity_id, double target_speed);

    /// Absolute speed transition to `target_speed` (m/s) shaped by `dynamics`.
    SpeedAction(std::string entity_id, double target_speed, TransitionDynamics dynamics);

    /// Relative speed transition: the target is `target` resolved against its
    /// reference entity's speed, shaped by `dynamics` (§RelativeTargetSpeed).
    SpeedAction(std::string entity_id, RelativeTargetSpeed target, TransitionDynamics dynamics);

    [[nodiscard]] const std::string& entity_id() const override;

    [[nodiscard]] std::string_view kind() const noexcept override;

    /// True when the target is relative to another entity (§RelativeTargetSpeed);
    /// false for an absolute target. When true, `target_speed()` is unused and
    /// `relative_target()` carries the target.
    [[nodiscard]] bool is_relative() const noexcept;

    /// Absolute target speed in meters per second. Meaningful only when
    /// `!is_relative()`.
    [[nodiscard]] double target_speed() const;

    /// The relative target, present iff `is_relative()` (§RelativeTargetSpeed).
    [[nodiscard]] const std::optional<RelativeTargetSpeed>& relative_target() const;

    /// How the target speed is reached (§SpeedActionDynamics).
    [[nodiscard]] const TransitionDynamics& dynamics() const;

private:
    std::string entity_id_;
    double target_speed_;
    std::optional<RelativeTargetSpeed> relative_target_;
    TransitionDynamics dynamics_;
};

/// One speed target of a SpeedProfileAction. Per ASAM OpenSCENARIO XML 1.4.0
/// §SpeedProfileEntry.
struct SpeedProfileEntry {
    /// The speed to reach. Unit: [m/s].
    double speed = 0.0;
    /// The time to reach it. The first entry's time is a delta from the start
    /// of the action, each later entry a delta from the previous entry. Absent
    /// ⇒ reached as soon as the Performance settings allow. Unit: [s], range
    /// [0..inf[.
    std::optional<double> time;
};

/// Changes an entity's speed through a series of speed targets over time. Per
/// ASAM OpenSCENARIO XML 1.4.0 §SpeedProfileAction.
///
/// Scena models `FollowingMode::Position`: strictly linear interpolation
/// between successive targets, starting from the entity's current speed. An
/// entry with no time is reached as fast as the Performance envelope allows.
/// The entityRef-relative profile and the jerk/DynamicConstraints of
/// followingMode=follow are deferred (ADR-0011).
class SpeedProfileAction final : public Action {
public:
    SpeedProfileAction(std::string entity_id, std::vector<SpeedProfileEntry> entries,
                       FollowingMode following_mode = FollowingMode::Position);

    [[nodiscard]] const std::string& entity_id() const override;

    [[nodiscard]] std::string_view kind() const noexcept override;

    /// The ordered series of speed targets (§SpeedProfileEntry).
    [[nodiscard]] const std::vector<SpeedProfileEntry>& entries() const;

    /// Interpolation behavior between targets (§FollowingMode).
    [[nodiscard]] FollowingMode following_mode() const;

private:
    std::string entity_id_;
    std::vector<SpeedProfileEntry> entries_;
    FollowingMode following_mode_;
};

/// Teleports an entity to a target position. Per ASAM OpenSCENARIO XML 1.4.0
/// §TeleportAction: a step (instantaneous) action that follows the §7.4 layer
/// prioritization hierarchy.
///
/// Scena models the world-frame target (§WorldPosition) only. The ten §6.3.8
/// Position variants and the PositionResolver (road/lane/relative/…) arrive with
/// p2-s4/p3-s4; this action generalizes to `ir::Position` there. The teleport
/// writes the entity's world position; orientation (part of the full Position)
/// is not modeled yet and is left unchanged.
class TeleportAction final : public Action {
public:
    TeleportAction(std::string entity_id, WorldPosition position);

    [[nodiscard]] const std::string& entity_id() const override;

    [[nodiscard]] std::string_view kind() const noexcept override;

    /// The world-frame target position (§WorldPosition).
    [[nodiscard]] const WorldPosition& position() const;

private:
    std::string entity_id_;
    WorldPosition position_;
};

/// Where the distance or time gap of a LongitudinalDistanceAction applies
/// relative to the reference entity, per ASAM OpenSCENARIO XML 1.4.0
/// §LongitudinalDisplacement [1.1]. Omitted ⇒ `TrailingReferencedEntity`.
enum class LongitudinalDisplacement {
    /// Either ahead of or behind the reference entity; the actor keeps the side
    /// it is currently on.
    Any,
    /// The actor stays behind the reference entity (the default).
    TrailingReferencedEntity,
    /// The actor stays ahead of the reference entity.
    LeadingReferencedEntity,
};

/// Limits on the acceleration, deceleration and speed a distance controller may
/// use, per §DynamicConstraints. Every field is optional and "missing value is
/// interpreted as 'inf'" — an absent limit does not constrain. The rate limits
/// (jerk) were added in 1.2; Scena stores them but does not yet clamp jerk
/// (deferred with the followingMode=follow jerk model, #62).
struct DynamicConstraints {
    std::optional<double> max_acceleration;      ///< m/s^2, Range [0..inf[.
    std::optional<double> max_acceleration_rate; ///< m/s^3, Range [0..inf[ [1.2].
    std::optional<double> max_deceleration;      ///< m/s^2, Range [0..inf[.
    std::optional<double> max_deceleration_rate; ///< m/s^3, Range [0..inf[ [1.2].
    std::optional<double> max_speed;             ///< m/s, Range [0..inf[.
};

/// Keeps a longitudinal distance or time gap to a reference entity, per
/// §LongitudinalDistanceAction: "activates a controller for the longitudinal
/// behavior of an entity in a way that a given distance or time gap to the
/// reference entity is maintained".
///
/// `distance` and `time_gap` are mutually exclusive (the XSD allows either
/// attribute; init rejects both or neither). With `continuous == false` the
/// action ends "by reaching the targeted distance" (Annex A Table 10); with
/// `continuous == true` it has no regular ending and only a stopTransition or a
/// superseding longitudinal action stops it (§7.5.3).
///
/// The action assigns a longitudinal control strategy (Table 10), so it
/// supersedes any SpeedAction / SpeedProfileAction driving the same entity.
class LongitudinalDistanceAction final : public Action {
public:
    LongitudinalDistanceAction(
        std::string entity_id, std::string entity_ref, std::optional<double> distance,
        std::optional<double> time_gap, bool freespace, bool continuous,
        CoordinateSystem coordinate_system = CoordinateSystem::Entity,
        LongitudinalDisplacement displacement = LongitudinalDisplacement::TrailingReferencedEntity,
        std::optional<DynamicConstraints> constraints = std::nullopt);

    [[nodiscard]] const std::string& entity_id() const override;

    [[nodiscard]] std::string_view kind() const noexcept override;

    /// Reference entity the distance is kept to (§entityRef).
    [[nodiscard]] const std::string& entity_ref() const;

    /// Target distance [m], present iff the action is distance-dimensioned.
    [[nodiscard]] const std::optional<double>& distance() const;

    /// Target time gap [s], present iff the action is time-gap-dimensioned.
    [[nodiscard]] const std::optional<double>& time_gap() const;

    /// True: the gap is measured between the closest bounding-box points;
    /// false: between reference points (§6.4.7).
    [[nodiscard]] bool freespace() const noexcept;

    /// True ⇒ the action never ends by itself (§7.5.3).
    [[nodiscard]] bool continuous() const noexcept;

    /// The coordinate system the gap is measured in; `Entity` (the default) is
    /// the actor's local system (§CoordinateSystem) [1.1].
    [[nodiscard]] CoordinateSystem coordinate_system() const noexcept;

    /// Which side of the reference entity the gap applies to [1.1].
    [[nodiscard]] LongitudinalDisplacement displacement() const noexcept;

    /// Optional limits on the controller's dynamics; absent ⇒ unlimited.
    [[nodiscard]] const std::optional<DynamicConstraints>& constraints() const;

private:
    std::string entity_id_;
    std::string entity_ref_;
    std::optional<double> distance_;
    std::optional<double> time_gap_;
    bool freespace_;
    bool continuous_;
    CoordinateSystem coordinate_system_;
    LongitudinalDisplacement displacement_;
    std::optional<DynamicConstraints> constraints_;
};

// --- Lateral actions (§7.4.1.4) ---------------------------------------------
//
// The lateral sign convention is one rule for all three actions: positive is
// the reference entity's +y axis, which points left (§6.3.4, ISO 8855; the
// road and lane t-axes of §6.3.2/§6.3.3 point left too, so the two agree).
//
// The 1.4 `layer` attribute of Absolute/RelativeTargetLane is deliberately not
// modeled: Scena targets XML 1.0-1.3, where lane layers (§7.4.1.3) do not
// exist at all. A 1.4 file's layer is a frontend concern when 1.4 is targeted.

/// Target lane of a LaneChangeAction given as a difference from the reference
/// entity's current lane, per ASAM OpenSCENARIO XML 1.4.0, Class
/// `RelativeTargetLane`: "Target lane defined as difference compared to the
/// reference entity's current lane evaluated in the reference entity's
/// coordinate system. [...] For ASAM OpenDRIVE maps, the road center lane is
/// not counted as a lane and thus omitted."
struct RelativeTargetLane {
    /// Name of the entity whose current lane the count is measured from; may be
    /// the actor itself.
    std::string entity_ref;
    /// Signed lane count; positive moves towards the reference entity's +y
    /// (left), negative towards -y (§7.4.1.4).
    int value = 0;
};

/// Target lane of a LaneChangeAction given by lane id, per Class
/// `AbsoluteTargetLane`: "Number (ID) of the target lane the entity will change
/// to." The id is a string in the XSD because it names an element of the road
/// network, so resolving it needs a road backend (#23).
struct AbsoluteTargetLane {
    std::string value;
};

/// Lane offset of a LaneOffsetAction measured from another entity's lane
/// position, per Class `RelativeTargetLaneOffset`. Positive is the reference
/// entity's +y (left), per §7.4.1.4: "Positive offset values are aligned with
/// the reference entities' positive y-axis."
struct RelativeTargetLaneOffset {
    std::string entity_ref;
    double value = 0.0; ///< Unit: [m], + = left of the reference entity.
};

/// Lane offset of a LaneOffsetAction measured from the actor's own lane centre
/// line, per Class `AbsoluteTargetLaneOffset`: "Lane offset with respect to the
/// entity's current lane's center line."
struct AbsoluteTargetLaneOffset {
    double value = 0.0; ///< Unit: [m], + = left.
};

/// Moves an entity from its current lane to a target lane, per Class
/// `LaneChangeAction`: "This action describes the transition between an
/// entity's current lane and its target lane. [...] The transition starts at
/// the current lane position, including the current lane offset and ends at the
/// target lane position, optionally including a targetLaneOffset. If no
/// targetLaneOffset is specified, it shall be zero after completing the action,
/// so that any previous offset is not taken into account."
///
/// The action assigns a lateral control strategy and ends "by reaching the
/// lateral centerline offset on the target lane" (Annex A Table 10). A Step
/// shape performs it instantaneously (§7.4.1.4) and assigns no control strategy
/// (§7.4.1.2).
///
/// Without a road backend Scena resolves the target in a flat world from a
/// configurable default lane width (Engine::set_default_lane_width); with one,
/// through the IRoadQuery lane queries. An absolute lane id has no flat-world
/// reading and needs the backend (ADR-0016, #23).
class LaneChangeAction final : public Action {
public:
    /// Lane change to a lane `target.value` lanes from `target.entity_ref`'s
    /// current lane (Class `RelativeTargetLane`).
    LaneChangeAction(std::string entity_id, RelativeTargetLane target, TransitionDynamics dynamics,
                     double target_lane_offset = 0.0);

    /// Lane change to the lane named by `target.value` (Class
    /// `AbsoluteTargetLane`).
    LaneChangeAction(std::string entity_id, AbsoluteTargetLane target, TransitionDynamics dynamics,
                     double target_lane_offset = 0.0);

    [[nodiscard]] const std::string& entity_id() const override;

    [[nodiscard]] std::string_view kind() const noexcept override;

    /// True when the target is a relative lane count; false for a lane id.
    [[nodiscard]] bool is_relative() const noexcept;

    /// The relative target, present iff `is_relative()`.
    [[nodiscard]] const std::optional<RelativeTargetLane>& relative_target() const;

    /// The absolute target, present iff `!is_relative()`.
    [[nodiscard]] const std::optional<AbsoluteTargetLane>& absolute_target() const;

    /// Shape and time/distance of the transition (§LaneChangeActionDynamics).
    [[nodiscard]] const TransitionDynamics& dynamics() const;

    /// "Lane offset to be reached at the target lane; the action will end
    /// there. Missing value is interpreted as 0." Unit: [m], + = left.
    [[nodiscard]] double target_lane_offset() const noexcept;

private:
    std::string entity_id_;
    std::optional<RelativeTargetLane> relative_target_;
    std::optional<AbsoluteTargetLane> absolute_target_;
    TransitionDynamics dynamics_;
    double target_lane_offset_;
};

/// Moves an entity to a lane offset and optionally keeps it there, per Class
/// `LaneOffsetAction`: "This action describes the transition to a defined lane
/// offset of an entity. The lane offset will be kept if the action is set as
/// continuous. [...] The dynamics are specified by providing the maxLateralAcc
/// used to keep the lane offset."
///
/// There is no authored duration: the transition time follows from the shape,
/// the offset delta and `max_lateral_acc` (ADR-0016). "Missing value is
/// interpreted as 'inf'" (Class `LaneOffsetActionDynamics`), which makes the
/// transition instantaneous, as does a Step shape (§7.4.1.4).
///
/// With `continuous == false` the action ends "by reaching the targeted lane
/// offset"; with `continuous == true` it has "no regular ending" (Annex A
/// Table 10, §7.5.3) and the offset is re-enforced every step.
class LaneOffsetAction final : public Action {
public:
    /// Offset measured from the actor's own lane centre line.
    LaneOffsetAction(std::string entity_id, AbsoluteTargetLaneOffset target, bool continuous,
                     DynamicsShape shape, std::optional<double> max_lateral_acc = std::nullopt);

    /// Offset measured from another entity's lane position.
    LaneOffsetAction(std::string entity_id, RelativeTargetLaneOffset target, bool continuous,
                     DynamicsShape shape, std::optional<double> max_lateral_acc = std::nullopt);

    [[nodiscard]] const std::string& entity_id() const override;

    [[nodiscard]] std::string_view kind() const noexcept override;

    /// True when the offset is relative to another entity.
    [[nodiscard]] bool is_relative() const noexcept;

    [[nodiscard]] const std::optional<RelativeTargetLaneOffset>& relative_target() const;

    [[nodiscard]] const std::optional<AbsoluteTargetLaneOffset>& absolute_target() const;

    /// True ⇒ the action never ends by itself (§7.5.3).
    [[nodiscard]] bool continuous() const noexcept;

    /// Geometrical shape of the transition (§LaneOffsetActionDynamics).
    [[nodiscard]] DynamicsShape shape() const noexcept;

    /// "Maximum lateral acceleration used to initially reach and afterwards
    /// keep the lane offset. Missing value is interpreted as 'inf'." Unit:
    /// [m/s^2]. Range: [0..inf[.
    [[nodiscard]] const std::optional<double>& max_lateral_acc() const;

private:
    std::string entity_id_;
    std::optional<RelativeTargetLaneOffset> relative_target_;
    std::optional<AbsoluteTargetLaneOffset> absolute_target_;
    bool continuous_;
    DynamicsShape shape_;
    std::optional<double> max_lateral_acc_;
};

/// Which side of the reference entity the lateral distance of a
/// LateralDistanceAction applies to, per Enumeration `LateralDisplacement`: "A
/// displacement relative to an entity along a lateral axis (e.g. the y axis of
/// the entity coordinate system)". Omitted ⇒ `Any`.
enum class LateralDisplacement {
    /// "Either left or right to the entity along the lateral dimension"; the
    /// actor keeps the side it is currently on.
    Any,
    /// "Left to the entity along the lateral dimension."
    LeftToReferencedEntity,
    /// "Right to the entity along the lateral dimension."
    RightToReferencedEntity,
};

/// Keeps a lateral distance to a reference entity, per Class
/// `LateralDistanceAction`: "This action describes a continuously kept lateral
/// distance of an entity with respect to a reference entity. The distance can
/// be maintained by using a controller, requiring limiting values for lateral
/// acceleration, lateral deceleration and lateral speed. Without this limiting
/// parameters lateral distance is kept rigid."
///
/// The action assigns a lateral control strategy; with `continuous == false` it
/// ends "by reaching the targeted lateral distance" and with
/// `continuous == true` it has "no regular ending" (Annex A Table 10).
///
/// The Lane coordinate system is accepted but cannot be honoured: §6.4.8.2.2
/// states the lane-CS lateral distance is undefined. Scena reports it and
/// completes the action, the road/trajectory-CS treatment of ADR-0009.
class LateralDistanceAction final : public Action {
public:
    LateralDistanceAction(std::string entity_id, std::string entity_ref, double distance,
                          bool freespace, bool continuous,
                          CoordinateSystem coordinate_system = CoordinateSystem::Entity,
                          LateralDisplacement displacement = LateralDisplacement::Any,
                          std::optional<DynamicConstraints> constraints = std::nullopt);

    [[nodiscard]] const std::string& entity_id() const override;

    [[nodiscard]] std::string_view kind() const noexcept override;

    /// "Name of the reference entity the lateral distance shall be kept to."
    [[nodiscard]] const std::string& entity_ref() const;

    /// "Lateral distance value. Missing value is interpreted as 0." Unit: [m].
    /// Range: [0..inf[.
    [[nodiscard]] double distance() const noexcept;

    /// True: measured between the closest bounding-box points; false: between
    /// reference points.
    [[nodiscard]] bool freespace() const noexcept;

    /// True ⇒ the action never ends by itself (§7.5.3).
    [[nodiscard]] bool continuous() const noexcept;

    /// The coordinate system the distance is measured in; `Entity` (the
    /// default) is the actor's local system.
    [[nodiscard]] CoordinateSystem coordinate_system() const noexcept;

    /// Which side of the reference entity the distance applies to.
    [[nodiscard]] LateralDisplacement displacement() const noexcept;

    /// Optional limits on the lateral controller; absent ⇒ "lateral distance is
    /// kept rigid".
    [[nodiscard]] const std::optional<DynamicConstraints>& constraints() const;

private:
    std::string entity_id_;
    std::string entity_ref_;
    double distance_;
    bool freespace_;
    bool continuous_;
    CoordinateSystem coordinate_system_;
    LateralDisplacement displacement_;
    std::optional<DynamicConstraints> constraints_;
};

/// Assigns a route to an entity, per §AssignRouteAction (§6.8.2). The action
/// "does not override any action that controls either lateral or longitudinal
/// domain" and completes immediately (Annex A Table 10): it installs routing
/// intent, it does not move the entity. The assigned route stays in place until
/// another routing action overwrites it (§6.8.2).
class AssignRouteAction final : public Action {
public:
    AssignRouteAction(std::string entity_id, Route route);

    [[nodiscard]] const std::string& entity_id() const override;

    [[nodiscard]] std::string_view kind() const noexcept override;

    /// The route to assign; requires at least two waypoints (§Route).
    [[nodiscard]] const Route& route() const;

private:
    std::string entity_id_;
    Route route_;
};

/// Routes an entity towards a target position, per §AcquirePositionAction.
/// Per §7.4.1.4 "a route with two waypoints is created: current position as
/// first and specified position as last waypoint"; the action then completes
/// immediately (Annex A Table 10). The entity's position at apply time becomes
/// the first waypoint, so the installed route depends on when the action fires.
class AcquirePositionAction final : public Action {
public:
    AcquirePositionAction(std::string entity_id, WorldPosition position);

    [[nodiscard]] const std::string& entity_id() const override;

    [[nodiscard]] std::string_view kind() const noexcept override;

    /// The world-frame target position to acquire (§WorldPosition).
    [[nodiscard]] const WorldPosition& position() const;

private:
    std::string entity_id_;
    WorldPosition position_;
};

/// Moves an entity along a trajectory, per §FollowTrajectoryAction (§6.9).
///
/// The time reference decides which domains the action assigns (Annex A
/// Table 10): with no timing it owns the lateral domain only and the entity's
/// current longitudinal control (a SpeedAction, say) sets the pace; with a
/// Timing it owns the longitudinal domain too and the vertex times set the
/// pace. Either way it ends "by reaching the end of the trajectory".
///
/// Scena models `FollowingMode::Position` (strict adherence to the shape);
/// `Follow` is accepted and treated as Position with a warning, the ADR-0011
/// precedent — a true steering controller arrives with p2-s5.
class FollowTrajectoryAction final : public Action {
public:
    FollowTrajectoryAction(std::string entity_id, Trajectory trajectory,
                           FollowingMode following_mode = FollowingMode::Position,
                           std::optional<Timing> time_reference = std::nullopt,
                           double initial_distance_offset = 0.0);

    [[nodiscard]] const std::string& entity_id() const override;

    [[nodiscard]] std::string_view kind() const noexcept override;

    /// The path to follow (§Trajectory).
    [[nodiscard]] const Trajectory& trajectory() const;

    /// How closely the actor adheres to the shape (§TrajectoryFollowingMode).
    [[nodiscard]] FollowingMode following_mode() const noexcept;

    /// The timing adjustment, or std::nullopt for §TimeReference "None" — the
    /// trajectory's time information is then ignored.
    [[nodiscard]] const std::optional<Timing>& time_reference() const;

    /// An offset into the trajectory, truncating it so following starts at that
    /// arc length. Unit: [m]. Range: [0..arclength of the trajectory].
    [[nodiscard]] double initial_distance_offset() const noexcept;

private:
    std::string entity_id_;
    Trajectory trajectory_;
    FollowingMode following_mode_;
    std::optional<Timing> time_reference_;
    double initial_distance_offset_;
};

/// Assigns a controller model to an entity, per §AssignControllerAction, and
/// optionally activates or deactivates it per domain. An unset activation flag
/// means "no change for controlling that dimension". Completes immediately
/// (Annex A Table 10).
///
/// The lighting and animation activation flags are not modeled (those domains
/// have no runtime in Scena); a scenario using them is not rejected, they are
/// simply not represented here.
class AssignControllerAction final : public Action {
public:
    AssignControllerAction(std::string entity_id, Controller controller,
                           std::optional<bool> activate_lateral = std::nullopt,
                           std::optional<bool> activate_longitudinal = std::nullopt);

    [[nodiscard]] const std::string& entity_id() const override;

    [[nodiscard]] std::string_view kind() const noexcept override;

    /// The controller to assign (§Controller).
    [[nodiscard]] const Controller& controller() const;

    /// Activation request for the lateral domain; absent ⇒ no change.
    [[nodiscard]] const std::optional<bool>& activate_lateral() const;

    /// Activation request for the longitudinal domain; absent ⇒ no change.
    [[nodiscard]] const std::optional<bool>& activate_longitudinal() const;

private:
    std::string entity_id_;
    Controller controller_;
    std::optional<bool> activate_lateral_;
    std::optional<bool> activate_longitudinal_;
};

/// Activates or deactivates controlled behavior per domain, per
/// §ActivateControllerAction. An unset flag means "no change for controlling
/// that domain". Completes immediately (Annex A Table 10).
///
/// Scena's engine is the default controller (docs/user-guide/motion.md), so
/// deactivating a domain releases the engine's control of it: the entity holds
/// what it has and actions targeting that domain are suppressed until it is
/// activated again (ADR-0014).
///
/// Standalone use of this action was deprecated in 1.1 in favor of
/// ControllerAction/ActivateControllerAction; both spellings lower to this IR
/// node, and it is the frontend (P4) that reports the deprecation.
class ActivateControllerAction final : public Action {
public:
    ActivateControllerAction(std::string entity_id, std::optional<bool> lateral,
                             std::optional<bool> longitudinal);

    [[nodiscard]] const std::string& entity_id() const override;

    [[nodiscard]] std::string_view kind() const noexcept override;

    /// Activation request for the lateral domain; absent ⇒ no change.
    [[nodiscard]] const std::optional<bool>& lateral() const;

    /// Activation request for the longitudinal domain; absent ⇒ no change.
    [[nodiscard]] const std::optional<bool>& longitudinal() const;

private:
    std::string entity_id_;
    std::optional<bool> lateral_;
    std::optional<bool> longitudinal_;
};

/// Toggles an entity's detectability, per §VisibilityAction: visibility in the
/// image generator, to sensors, and to other traffic participants. All three
/// attributes are required by the XSD, so the action always states a complete
/// visibility. Completes immediately (Annex A Table 10).
///
/// The 1.2 `sensorReferenceSet` (naming individual sensors) is not modeled.
class VisibilityAction final : public Action {
public:
    VisibilityAction(std::string entity_id, bool graphics, bool sensors, bool traffic);

    [[nodiscard]] const std::string& entity_id() const override;

    [[nodiscard]] std::string_view kind() const noexcept override;

    /// Visible in the host's image generator(s).
    [[nodiscard]] bool graphics() const noexcept;

    /// Visible to the host's sensor model(s).
    [[nodiscard]] bool sensors() const noexcept;

    /// Visible to other traffic participants.
    [[nodiscard]] bool traffic() const noexcept;

private:
    std::string entity_id_;
    bool graphics_;
    bool sensors_;
    bool traffic_;
};

// --- Global actions (§7.4.2) ------------------------------------------------

/// The arithmetic a modify action applies to the value it references, per
/// §VariableModifyRule (and the deprecated §ModifyRule, which has the same two
/// members). "Either adding a value to a variable or multiply the variable by a
/// value."
enum class ModifyOperator {
    /// §VariableAddValueRule / §ParameterAddValueRule: current + value.
    Add,
    /// §VariableMultiplyByValueRule / §ParameterMultiplyByValueRule:
    /// current * value.
    Multiply,
};

/// Sets a global variable to a given value, per §VariableSetAction (≥1.2).
/// Variables are the one named-value namespace that changes during a run
/// (§6.12); the new value is a string, exactly as the XSD spells it, and the
/// VariableCondition comparison rules decide how it reads. Completes
/// immediately (Annex A Table 11).
class VariableSetAction final : public GlobalAction {
public:
    VariableSetAction(std::string variable_ref, std::string value);

    [[nodiscard]] std::string_view kind() const noexcept override;

    /// Name of the variable to write; must be declared (§6.12, rule
    /// reference_control.resolvable_variable_reference).
    [[nodiscard]] const std::string& variable_ref() const;

    /// The new value.
    [[nodiscard]] const std::string& value() const;

private:
    std::string variable_ref_;
    std::string value_;
};

/// Modifies a global variable arithmetically, per §VariableModifyAction (≥1.2).
///
/// Rule C.2.6 (`data_type.variable_modification_or_comparison_possible`)
/// restricts this to numeric variables: Scena has no typed declarations yet
/// (p4-s3), so it applies the operator when the current value parses as a
/// scalar and reports the rule as a Warning otherwise, leaving the variable
/// untouched. Completes immediately (Table 11).
class VariableModifyAction final : public GlobalAction {
public:
    VariableModifyAction(std::string variable_ref, ModifyOperator op, double value);

    [[nodiscard]] std::string_view kind() const noexcept override;

    [[nodiscard]] const std::string& variable_ref() const;

    /// Whether the value is added or multiplied in.
    [[nodiscard]] ModifyOperator op() const noexcept;

    /// The operand of the operator.
    [[nodiscard]] double value() const noexcept;

private:
    std::string variable_ref_;
    ModifyOperator op_;
    double value_;
};

/// Sets a global parameter to a given value, per §ParameterSetAction —
/// **deprecated with 1.2** in favor of VariableSetAction.
///
/// Parameters are immutable at runtime from 1.2 on (§9.1), so this action can
/// only appear in a 1.0/1.1 file. Scena executes it against a runtime overlay
/// that shadows the immutable declaration for the ParameterCondition that reads
/// it, and warns once with Status::DeprecatedFeature. Completes immediately.
class ParameterSetAction final : public GlobalAction {
public:
    ParameterSetAction(std::string parameter_ref, std::string value);

    [[nodiscard]] std::string_view kind() const noexcept override;

    /// Name of the parameter to write; must be declared in the scenario.
    [[nodiscard]] const std::string& parameter_ref() const;

    [[nodiscard]] const std::string& value() const;

private:
    std::string parameter_ref_;
    std::string value_;
};

/// Modifies a global parameter arithmetically, per §ParameterModifyAction —
/// **deprecated with 1.2** in favor of VariableModifyAction. Same overlay,
/// deprecation warning and numeric restriction as ParameterSetAction /
/// VariableModifyAction.
class ParameterModifyAction final : public GlobalAction {
public:
    ParameterModifyAction(std::string parameter_ref, ModifyOperator op, double value);

    [[nodiscard]] std::string_view kind() const noexcept override;

    [[nodiscard]] const std::string& parameter_ref() const;
    [[nodiscard]] ModifyOperator op() const noexcept;
    [[nodiscard]] double value() const noexcept;

private:
    std::string parameter_ref_;
    ModifyOperator op_;
    double value_;
};

/// Sets weather state, road conditions and the time of day, per
/// §EnvironmentAction. Completes immediately (Annex A Table 11).
///
/// §Environment gives the merge its semantics: an absent member "doesn't
/// change", so the action updates only what it carries and the engine keeps
/// the rest. The catalog-reference form of the action (§EnvironmentAction is a
/// union of an inline Environment and a CatalogReference) arrives with
/// catalogs (P4).
class EnvironmentAction final : public GlobalAction {
public:
    explicit EnvironmentAction(Environment environment);

    [[nodiscard]] std::string_view kind() const noexcept override;

    /// The environment update to merge in.
    [[nodiscard]] const Environment& environment() const;

private:
    Environment environment_;
};

/// Issues a user-defined command to the host, per §UserDefinedAction /
/// §CustomCommandAction (§7.4.3): "Users may create their own actions which may
/// incorporate a command or a script file." Completes immediately (Annex A
/// Table 12).
///
/// Both strings are opaque to the engine: `type` and `content` are "defined as
/// a contract between the simulation environment provider and the author of a
/// scenario", so Scena hands them to the gateway verbatim and reads no meaning
/// into either. Without a gateway the action is a silent no-op — §7.4.3 makes
/// executability depend on "the ability of the specific simulation environment
/// recognizing these actions", so a host that does not implement it is not an
/// error.
class CustomCommandAction final : public GlobalAction {
public:
    CustomCommandAction(std::string type, std::string content);

    [[nodiscard]] std::string_view kind() const noexcept override;

    /// The command type agreed between host and scenario author.
    [[nodiscard]] const std::string& type() const;

    /// The command itself, in whatever form that contract specifies.
    [[nodiscard]] const std::string& content() const;

private:
    std::string type_;
    std::string content_;
};

/// Forces the observable state of a named traffic signal, per
/// §InfrastructureAction / §TrafficSignalAction / §TrafficSignalStateAction:
/// "control the state of a traffic signal". Completes immediately (Annex A
/// Table 11).
///
/// The write outlives the action: the forced state stands until the controlling
/// TrafficSignalController's next phase transition overwrites it, which is what
/// makes the §11.12 "traffic light failure" example work — a signal is forced
/// into a broken state and stays there. `name` is a road-network signal id, so
/// it is free-form (rule traffic_signal_state_action_references, C.7.14, is not
/// checkable without a road network).
class TrafficSignalStateAction final : public GlobalAction {
public:
    TrafficSignalStateAction(std::string name, std::string state);

    [[nodiscard]] std::string_view kind() const noexcept override;

    /// ID of the referenced signal in the road network file.
    [[nodiscard]] const std::string& name() const;

    /// The observable state to force, e.g. "off;off;on".
    [[nodiscard]] const std::string& state() const;

private:
    std::string name_;
    std::string state_;
};

/// Jumps a traffic signal controller to a named phase, per
/// §TrafficSignalControllerAction: "set a specific phase of a traffic signal
/// controller, typically affecting a collection of signals". Completes
/// immediately (Table 11).
///
/// The cycle restarts at that phase and continues from there in declared order.
/// Both references are checkable within the scenario (rule
/// traffic_signal_controller_action_references, C.7.11).
class TrafficSignalControllerAction final : public GlobalAction {
public:
    TrafficSignalControllerAction(std::string traffic_signal_controller_ref, std::string phase);

    [[nodiscard]] std::string_view kind() const noexcept override;

    /// Name of the referenced TrafficSignalController.
    [[nodiscard]] const std::string& traffic_signal_controller_ref() const;

    /// Name of the targeted phase of that controller.
    [[nodiscard]] const std::string& phase() const;

private:
    std::string traffic_signal_controller_ref_;
    std::string phase_;
};

/// Adds a declared entity to the running scenario at a position, per
/// §EntityAction / §AddEntityAction. "Entities to be added or deleted must be
/// defined in the Entities section. An entity can only exist in one copy.
/// Adding an already active entity will have no effect" — so the entity_ref
/// resolves against the declaration, not against a live instance, and adding an
/// active entity is a no-op. Completes immediately (Annex A Table 11).
///
/// Scena models the world-frame target only, the p5-s4 TeleportAction
/// precedent; the other §6.3.8 position variants arrive with the
/// PositionResolver (p2-s4/p3-s4).
class AddEntityAction final : public GlobalAction {
public:
    AddEntityAction(std::string entity_ref, WorldPosition position);

    [[nodiscard]] std::string_view kind() const noexcept override;

    /// Name of the declared entity to add (§EntityAction entityRef).
    [[nodiscard]] const std::string& entity_ref() const;

    /// Where the entity appears (§WorldPosition).
    [[nodiscard]] const WorldPosition& position() const;

private:
    std::string entity_ref_;
    WorldPosition position_;
};

/// Removes an entity from the running scenario, per §EntityAction /
/// §DeleteEntityAction. "Deleting an already inactive entity" has no effect.
/// A deleted entity stops moving, stops being published to the host, and
/// disappears from the by-entity conditions; its declaration survives, so an
/// AddEntityAction can bring it back. Completes immediately (Table 11).
class DeleteEntityAction final : public GlobalAction {
public:
    explicit DeleteEntityAction(std::string entity_ref);

    [[nodiscard]] std::string_view kind() const noexcept override;

    [[nodiscard]] const std::string& entity_ref() const;

private:
    std::string entity_ref_;
};

} // namespace scena::ir
