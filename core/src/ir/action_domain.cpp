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

#include "scena/ir/action_domain.h"

#include "scena/ir/action.h"
#include "scena/ir/dynamics.h"

namespace scena::ir {

namespace {

/// §7.4.1.2: a Step-shaped transition "does not assign a control strategy as the
/// changes are enacted instantaneously". Named so the three call sites read as
/// the clause they implement.
bool sets_a_state(const TransitionDynamics& dynamics) noexcept {
    return dynamics.shape == DynamicsShape::Step;
}

} // namespace

ActionDomain control_domains(const Action& action) noexcept {
    // Annex A Table 10, in the table's own order. A dynamic_cast ladder rather
    // than a virtual on Action: the classification is a property of the
    // standard's table, not of the action, and keeping it here means the table
    // can be read against the specification in one place.

    // --- longitudinal only ---------------------------------------------------
    if (const auto* speed = dynamic_cast<const SpeedAction*>(&action)) {
        return sets_a_state(speed->dynamics()) ? ActionDomain::None : ActionDomain::Longitudinal;
    }
    if (dynamic_cast<const SpeedProfileAction*>(&action) != nullptr) {
        // A profile has no Step option: every entry is reached over its time.
        return ActionDomain::Longitudinal;
    }
    if (dynamic_cast<const LongitudinalDistanceAction*>(&action) != nullptr) {
        return ActionDomain::Longitudinal;
    }

    // --- lateral only --------------------------------------------------------
    if (const auto* lane_change = dynamic_cast<const LaneChangeAction*>(&action)) {
        return sets_a_state(lane_change->dynamics()) ? ActionDomain::None : ActionDomain::Lateral;
    }
    if (const auto* lane_offset = dynamic_cast<const LaneOffsetAction*>(&action)) {
        // LaneOffsetActionDynamics carries a shape but no dimension; a Step
        // displacement is "performed instantaneously - not over time" (§7.4.1.4)
        // and, continuous or not, assigns nothing. A continuous non-Step offset
        // keeps the domain forever (§7.5.3).
        return lane_offset->shape() == DynamicsShape::Step ? ActionDomain::None
                                                           : ActionDomain::Lateral;
    }
    if (dynamic_cast<const LateralDistanceAction*>(&action) != nullptr) {
        return ActionDomain::Lateral;
    }

    // --- both motion domains -------------------------------------------------
    if (const auto* trajectory = dynamic_cast<const FollowTrajectoryAction*>(&action)) {
        // Table 10 marks FollowTrajectoryAction lateral in every configuration,
        // and longitudinal only with `timeReference = timing`: without a timing
        // the actor moves along the path "with the current longitudinal control
        // (e.g. speed keeping)", which is to say it leaves that domain alone.
        return trajectory->time_reference().has_value()
                   ? (ActionDomain::Lateral | ActionDomain::Longitudinal)
                   : ActionDomain::Lateral;
    }

    // Everything else — TeleportAction, the routing actions, the controller
    // actions, VisibilityAction, and every global action — is listed in
    // §7.4.1.2 as a non-motion-control action, or is outside Scena's subset.
    // Either way it holds no domain and cannot conflict.
    return ActionDomain::None;
}

} // namespace scena::ir
