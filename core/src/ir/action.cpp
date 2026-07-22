// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
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

SpeedProfileAction::SpeedProfileAction(std::string entity_id,
                                       std::vector<SpeedProfileEntry> entries,
                                       FollowingMode following_mode)
    : entity_id_(std::move(entity_id)), entries_(std::move(entries)),
      following_mode_(following_mode) {}

const std::string& SpeedProfileAction::entity_id() const {
    return entity_id_;
}

std::string_view SpeedProfileAction::kind() const noexcept {
    return "SpeedProfileAction";
}

const std::vector<SpeedProfileEntry>& SpeedProfileAction::entries() const {
    return entries_;
}

FollowingMode SpeedProfileAction::following_mode() const {
    return following_mode_;
}

} // namespace scena::ir
