// SPDX-License-Identifier: MIT
#include "scena/ir/action.h"

#include <utility>

namespace scena::ir {

SpeedAction::SpeedAction(std::string entity_id, double target_speed)
    : entity_id_(std::move(entity_id)), target_speed_(target_speed) {}

const std::string& SpeedAction::entity_id() const {
    return entity_id_;
}

double SpeedAction::target_speed() const {
    return target_speed_;
}

} // namespace scena::ir
