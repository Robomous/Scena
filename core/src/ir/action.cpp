// SPDX-License-Identifier: MIT
#include "scena/ir/action.h"

#include <utility>

namespace scena::ir {

SpeedAction::SpeedAction(std::string entity_id, double target_speed)
    : entity_id_(std::move(entity_id)), target_speed_(target_speed) {}
// dynamics_ defaults to a Step transition, so this reaches the target
// instantaneously — the historical SpeedAction behaviour.

SpeedAction::SpeedAction(std::string entity_id, double target_speed, TransitionDynamics dynamics)
    : entity_id_(std::move(entity_id)), target_speed_(target_speed), dynamics_(dynamics) {}

const std::string& SpeedAction::entity_id() const {
    return entity_id_;
}

std::string_view SpeedAction::kind() const noexcept {
    return "SpeedAction";
}

double SpeedAction::target_speed() const {
    return target_speed_;
}

const TransitionDynamics& SpeedAction::dynamics() const {
    return dynamics_;
}

} // namespace scena::ir
