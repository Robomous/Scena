// SPDX-License-Identifier: MIT
#pragma once

#include <string>
#include <string_view>

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

} // namespace scena::ir
