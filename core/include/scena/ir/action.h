// SPDX-License-Identifier: MIT
#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "scena/ir/dynamics.h"

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

/// Transitions the longitudinal speed of an entity to a target speed, governed
/// by transition dynamics. Per ASAM OpenSCENARIO XML 1.4.0 §SpeedAction (the
/// target is the absolute-speed case; a speed relative to another entity is a
/// later addition — see ADR-0011).
class SpeedAction final : public Action {
public:
    /// Step (instantaneous) speed change: reaches `target_speed` in the
    /// evaluation it fires. The back-compatible short form, equivalent to the
    /// dynamics form with a Step shape.
    SpeedAction(std::string entity_id, double target_speed);

    /// Speed transition to `target_speed` (m/s) shaped by `dynamics`.
    SpeedAction(std::string entity_id, double target_speed, TransitionDynamics dynamics);

    [[nodiscard]] const std::string& entity_id() const override;

    [[nodiscard]] std::string_view kind() const noexcept override;

    /// Target speed in meters per second.
    [[nodiscard]] double target_speed() const;

    /// How the target speed is reached (§SpeedActionDynamics).
    [[nodiscard]] const TransitionDynamics& dynamics() const;

private:
    std::string entity_id_;
    double target_speed_;
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

} // namespace scena::ir
