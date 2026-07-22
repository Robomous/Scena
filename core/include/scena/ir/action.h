// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "scena/ir/dynamics.h"
#include "scena/ir/position.h"

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

} // namespace scena::ir
