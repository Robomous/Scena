// SPDX-License-Identifier: MIT
#pragma once

#include <string>

namespace kinema::ir {

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
};

/// Instantaneously sets the longitudinal speed of an entity. Placeholder for
/// the ASAM OpenSCENARIO SpeedAction: no transition dynamics yet.
class SpeedAction final : public Action {
public:
    SpeedAction(std::string entity_id, double target_speed);

    [[nodiscard]] const std::string& entity_id() const override;

    /// Target speed in meters per second.
    [[nodiscard]] double target_speed() const;

private:
    std::string entity_id_;
    double target_speed_;
};

} // namespace kinema::ir
