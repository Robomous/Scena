// SPDX-License-Identifier: MIT
#pragma once

#include <string>
#include <string_view>

namespace scena::ir {

/// Base class for all scenario actions in the Scenario IR.
///
/// Actions in this phase are instantaneous: they take effect fully in the step
/// in which their condition fires. Transition dynamics (ramps, profiles) come
/// with the full ASAM OpenSCENARIO action set in a later phase.
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

/// Instantaneously sets the longitudinal speed of an entity. Placeholder for
/// the ASAM OpenSCENARIO SpeedAction: no transition dynamics yet.
class SpeedAction final : public Action {
public:
    SpeedAction(std::string entity_id, double target_speed);

    [[nodiscard]] const std::string& entity_id() const override;

    [[nodiscard]] std::string_view kind() const noexcept override;

    /// Target speed in meters per second.
    [[nodiscard]] double target_speed() const;

private:
    std::string entity_id_;
    double target_speed_;
};

} // namespace scena::ir
