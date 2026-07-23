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

#include "scena/ir/interaction_condition.h"

#include <cmath>
#include <optional>
#include <utility>
#include <variant>
#include <vector>

#include "scena/ir/evaluation_context.h"
#include "scena/ir/rule.h"
#include "scena/runtime/distance_measure.h"
#include "scena/runtime/obb2.h"

namespace scena::ir {

// --- DistanceCondition ------------------------------------------------------

DistanceCondition::DistanceCondition(TriggeringEntities triggering, WorldPosition position,
                                     double value, bool freespace, Rule rule,
                                     std::optional<CoordinateSystem> coordinate_system,
                                     std::optional<RelativeDistanceType> relative_distance_type,
                                     std::optional<RoutingAlgorithm> routing_algorithm,
                                     std::optional<bool> along_route)
    : ByEntityCondition(std::move(triggering)), position_(position), value_(value),
      freespace_(freespace), rule_(rule), coordinate_system_(coordinate_system),
      relative_distance_type_(relative_distance_type), routing_algorithm_(routing_algorithm),
      along_route_(along_route) {}

bool DistanceCondition::evaluate_for_entity(const EvaluationContext& context,
                                            std::string_view entity_id) const {
    const std::optional<EntityKinematics> trigger = context.entity_kinematics(entity_id);
    if (!trigger.has_value()) {
        return false;
    }
    const runtime::DistanceSpec spec{effective_coordinate_system(),
                                     effective_relative_distance_type(), freespace_};
    const std::optional<double> measured =
        runtime::measure_distance(*trigger, nullptr, position_, spec);
    if (!measured.has_value()) {
        return false;
    }
    return compare(*measured, rule_, value_);
}

const WorldPosition& DistanceCondition::position() const {
    return position_;
}

double DistanceCondition::value() const {
    return value_;
}

bool DistanceCondition::freespace() const {
    return freespace_;
}

Rule DistanceCondition::rule() const {
    return rule_;
}

const std::optional<CoordinateSystem>& DistanceCondition::coordinate_system() const {
    return coordinate_system_;
}

const std::optional<RelativeDistanceType>& DistanceCondition::relative_distance_type() const {
    return relative_distance_type_;
}

const std::optional<RoutingAlgorithm>& DistanceCondition::routing_algorithm() const {
    return routing_algorithm_;
}

const std::optional<bool>& DistanceCondition::along_route() const {
    return along_route_;
}

CoordinateSystem DistanceCondition::effective_coordinate_system() const {
    if (coordinate_system_.has_value()) {
        return *coordinate_system_;
    }
    // alongRoute is honored only when neither coordinateSystem nor
    // relativeDistanceType is authored (§ DistanceCondition); alongRoute=true
    // then selects road-based routing, which is deferred.
    if (!relative_distance_type_.has_value() && along_route_.value_or(false)) {
        return CoordinateSystem::Road;
    }
    return CoordinateSystem::Entity;
}

RelativeDistanceType DistanceCondition::effective_relative_distance_type() const {
    return relative_distance_type_.value_or(RelativeDistanceType::EuclidianDistance);
}

// --- RelativeDistanceCondition ----------------------------------------------

RelativeDistanceCondition::RelativeDistanceCondition(
    TriggeringEntities triggering, std::string entity_ref, double value, bool freespace,
    RelativeDistanceType relative_distance_type, Rule rule,
    std::optional<CoordinateSystem> coordinate_system,
    std::optional<RoutingAlgorithm> routing_algorithm)
    : ByEntityCondition(std::move(triggering)), entity_ref_(std::move(entity_ref)), value_(value),
      freespace_(freespace), relative_distance_type_(relative_distance_type), rule_(rule),
      coordinate_system_(coordinate_system), routing_algorithm_(routing_algorithm) {}

bool RelativeDistanceCondition::evaluate_for_entity(const EvaluationContext& context,
                                                    std::string_view entity_id) const {
    const std::optional<EntityKinematics> trigger = context.entity_kinematics(entity_id);
    const std::optional<EntityKinematics> reference = context.entity_kinematics(entity_ref_);
    if (!trigger.has_value() || !reference.has_value()) {
        return false; // a missing reference makes every triggering entity false
    }
    const runtime::DistanceSpec spec{effective_coordinate_system(), relative_distance_type_,
                                     freespace_};
    const std::optional<double> measured =
        runtime::measure_distance(*trigger, &reference.value(), WorldPosition{}, spec);
    if (!measured.has_value()) {
        return false;
    }
    return compare(*measured, rule_, value_);
}

const std::string& RelativeDistanceCondition::entity_ref() const {
    return entity_ref_;
}

double RelativeDistanceCondition::value() const {
    return value_;
}

bool RelativeDistanceCondition::freespace() const {
    return freespace_;
}

RelativeDistanceType RelativeDistanceCondition::relative_distance_type() const {
    return relative_distance_type_;
}

Rule RelativeDistanceCondition::rule() const {
    return rule_;
}

const std::optional<CoordinateSystem>& RelativeDistanceCondition::coordinate_system() const {
    return coordinate_system_;
}

const std::optional<RoutingAlgorithm>& RelativeDistanceCondition::routing_algorithm() const {
    return routing_algorithm_;
}

CoordinateSystem RelativeDistanceCondition::effective_coordinate_system() const {
    return coordinate_system_.value_or(CoordinateSystem::Entity);
}

// --- TimeHeadwayCondition ---------------------------------------------------

TimeHeadwayCondition::TimeHeadwayCondition(
    TriggeringEntities triggering, std::string entity_ref, double value, bool freespace, Rule rule,
    std::optional<CoordinateSystem> coordinate_system,
    std::optional<RelativeDistanceType> relative_distance_type,
    std::optional<RoutingAlgorithm> routing_algorithm, std::optional<bool> along_route)
    : ByEntityCondition(std::move(triggering)), entity_ref_(std::move(entity_ref)), value_(value),
      freespace_(freespace), rule_(rule), coordinate_system_(coordinate_system),
      relative_distance_type_(relative_distance_type), routing_algorithm_(routing_algorithm),
      along_route_(along_route) {}

bool TimeHeadwayCondition::evaluate_for_entity(const EvaluationContext& context,
                                               std::string_view entity_id) const {
    const std::optional<EntityKinematics> trigger = context.entity_kinematics(entity_id);
    const std::optional<EntityKinematics> reference = context.entity_kinematics(entity_ref_);
    if (!trigger.has_value() || !reference.has_value()) {
        return false;
    }
    const runtime::DistanceSpec spec{effective_coordinate_system(),
                                     effective_relative_distance_type(), freespace_};
    const std::optional<double> distance =
        runtime::measure_distance(*trigger, &reference.value(), WorldPosition{}, spec);
    if (!distance.has_value()) {
        return false;
    }
    // Only the triggering entity's speed counts (§ TimeHeadwayCondition). A
    // stopped or reversing follower never reaches the reference position; the
    // negated `> 0` also rejects NaN.
    const double s_trig = trigger->state.speed;
    if (!(s_trig > 0.0)) {
        return false;
    }
    return compare(*distance / s_trig, rule_, value_);
}

const std::string& TimeHeadwayCondition::entity_ref() const {
    return entity_ref_;
}

double TimeHeadwayCondition::value() const {
    return value_;
}

bool TimeHeadwayCondition::freespace() const {
    return freespace_;
}

Rule TimeHeadwayCondition::rule() const {
    return rule_;
}

const std::optional<CoordinateSystem>& TimeHeadwayCondition::coordinate_system() const {
    return coordinate_system_;
}

const std::optional<RelativeDistanceType>& TimeHeadwayCondition::relative_distance_type() const {
    return relative_distance_type_;
}

const std::optional<RoutingAlgorithm>& TimeHeadwayCondition::routing_algorithm() const {
    return routing_algorithm_;
}

const std::optional<bool>& TimeHeadwayCondition::along_route() const {
    return along_route_;
}

CoordinateSystem TimeHeadwayCondition::effective_coordinate_system() const {
    if (coordinate_system_.has_value()) {
        return *coordinate_system_;
    }
    if (!relative_distance_type_.has_value() && along_route_.value_or(false)) {
        return CoordinateSystem::Road;
    }
    return CoordinateSystem::Entity;
}

RelativeDistanceType TimeHeadwayCondition::effective_relative_distance_type() const {
    return relative_distance_type_.value_or(RelativeDistanceType::EuclidianDistance);
}

// --- TimeToCollisionCondition -----------------------------------------------

TimeToCollisionCondition::TimeToCollisionCondition(
    TriggeringEntities triggering, TimeToCollisionTarget target, double value, bool freespace,
    Rule rule, std::optional<CoordinateSystem> coordinate_system,
    std::optional<RelativeDistanceType> relative_distance_type,
    std::optional<RoutingAlgorithm> routing_algorithm, std::optional<bool> along_route)
    : ByEntityCondition(std::move(triggering)), target_(std::move(target)), value_(value),
      freespace_(freespace), rule_(rule), coordinate_system_(coordinate_system),
      relative_distance_type_(relative_distance_type), routing_algorithm_(routing_algorithm),
      along_route_(along_route) {}

bool TimeToCollisionCondition::evaluate_for_entity(const EvaluationContext& context,
                                                   std::string_view entity_id) const {
    const std::optional<EntityKinematics> trigger = context.entity_kinematics(entity_id);
    if (!trigger.has_value()) {
        return false;
    }
    // Resolve the target (holds_alternative/get rather than std::visit to keep
    // MSVC /W4 quiet). An entity target that is absent makes this false.
    std::optional<EntityKinematics> reference;
    const EntityKinematics* target_entity = nullptr;
    WorldPosition target_point{};
    if (std::holds_alternative<std::string>(target_)) {
        reference = context.entity_kinematics(std::get<std::string>(target_));
        if (!reference.has_value()) {
            return false;
        }
        target_entity = &reference.value();
    } else {
        target_point = std::get<WorldPosition>(target_);
    }

    const runtime::DistanceSpec spec{effective_coordinate_system(),
                                     effective_relative_distance_type(), freespace_};
    const std::optional<double> distance =
        runtime::measure_distance(*trigger, target_entity, target_point, spec);
    if (!distance.has_value()) {
        return false;
    }
    const std::optional<double> cs =
        runtime::closing_speed(*trigger, target_entity, target_point, spec);
    // No collision predicted when moving apart, stationary relative motion, or
    // coincident reference points (nullopt); the negated `> 0` rejects NaN.
    if (!cs.has_value() || !(*cs > 0.0)) {
        return false;
    }
    return compare(*distance / *cs, rule_, value_);
}

const TimeToCollisionTarget& TimeToCollisionCondition::target() const {
    return target_;
}

double TimeToCollisionCondition::value() const {
    return value_;
}

bool TimeToCollisionCondition::freespace() const {
    return freespace_;
}

Rule TimeToCollisionCondition::rule() const {
    return rule_;
}

const std::optional<CoordinateSystem>& TimeToCollisionCondition::coordinate_system() const {
    return coordinate_system_;
}

const std::optional<RelativeDistanceType>&
TimeToCollisionCondition::relative_distance_type() const {
    return relative_distance_type_;
}

const std::optional<RoutingAlgorithm>& TimeToCollisionCondition::routing_algorithm() const {
    return routing_algorithm_;
}

const std::optional<bool>& TimeToCollisionCondition::along_route() const {
    return along_route_;
}

CoordinateSystem TimeToCollisionCondition::effective_coordinate_system() const {
    if (coordinate_system_.has_value()) {
        return *coordinate_system_;
    }
    if (!relative_distance_type_.has_value() && along_route_.value_or(false)) {
        return CoordinateSystem::Road;
    }
    return CoordinateSystem::Entity;
}

RelativeDistanceType TimeToCollisionCondition::effective_relative_distance_type() const {
    return relative_distance_type_.value_or(RelativeDistanceType::EuclidianDistance);
}

// --- CollisionCondition -----------------------------------------------------

CollisionCondition::CollisionCondition(TriggeringEntities triggering, std::string entity_ref)
    : ByEntityCondition(std::move(triggering)), entity_ref_(std::move(entity_ref)) {}

bool CollisionCondition::evaluate_for_entity(const EvaluationContext& context,
                                             std::string_view entity_id) const {
    const std::optional<EntityKinematics> trigger = context.entity_kinematics(entity_id);
    const std::optional<EntityKinematics> reference = context.entity_kinematics(entity_ref_);
    if (!trigger.has_value() || !reference.has_value()) {
        return false;
    }
    // Collision is bounding-box intersection (§6.4.7.2). Without geometry on
    // both entities there is nothing to intersect: per-entity false.
    if (!trigger->bounding_box.has_value() || !reference->bounding_box.has_value()) {
        return false;
    }
    const runtime::Obb2 a = runtime::make_obb(trigger->state, *trigger->bounding_box);
    const runtime::Obb2 b = runtime::make_obb(reference->state, *reference->bounding_box);
    return runtime::obb_intersects(a, b);
}

const std::string& CollisionCondition::entity_ref() const {
    return entity_ref_;
}

// --- EndOfRoadCondition -----------------------------------------------------

EndOfRoadCondition::EndOfRoadCondition(TriggeringEntities triggering, double duration)
    : ByEntityCondition(std::move(triggering)), duration_(duration) {}

bool EndOfRoadCondition::evaluate_for_entity(const EvaluationContext& /*context*/,
                                             std::string_view /*entity_id*/) const {
    // "End of road" is a road-network predicate (§7.6.5.1); without IRoadQuery
    // (p3-s4) it is a deterministic false, warned once at init.
    return false;
}

double EndOfRoadCondition::duration() const {
    return duration_;
}

// --- OffroadCondition -------------------------------------------------------

OffroadCondition::OffroadCondition(TriggeringEntities triggering, double duration)
    : ByEntityCondition(std::move(triggering)), duration_(duration) {}

bool OffroadCondition::evaluate_for_entity(const EvaluationContext& /*context*/,
                                           std::string_view /*entity_id*/) const {
    // Road-network predicate (§7.6.5.1); deferred to p3-s4, deterministic false.
    return false;
}

double OffroadCondition::duration() const {
    return duration_;
}

// --- RelativeClearanceCondition ---------------------------------------------

RelativeClearanceCondition::RelativeClearanceCondition(TriggeringEntities triggering,
                                                       bool free_space, bool opposite_lanes,
                                                       double distance_backward,
                                                       double distance_forward,
                                                       std::vector<std::string> entity_refs,
                                                       std::vector<RelativeLaneRange> lane_ranges)
    : ByEntityCondition(std::move(triggering)), free_space_(free_space),
      opposite_lanes_(opposite_lanes), distance_backward_(distance_backward),
      distance_forward_(distance_forward), entity_refs_(std::move(entity_refs)),
      lane_ranges_(std::move(lane_ranges)) {}

bool RelativeClearanceCondition::evaluate_for_entity(const EvaluationContext& /*context*/,
                                                     std::string_view /*entity_id*/) const {
    // The checked area is defined in lane coordinates (§6.4.5); without a road
    // network (p3-s4) it cannot be evaluated: deterministic false, warned once.
    return false;
}

bool RelativeClearanceCondition::free_space() const {
    return free_space_;
}

bool RelativeClearanceCondition::opposite_lanes() const {
    return opposite_lanes_;
}

double RelativeClearanceCondition::distance_backward() const {
    return distance_backward_;
}

double RelativeClearanceCondition::distance_forward() const {
    return distance_forward_;
}

const std::vector<std::string>& RelativeClearanceCondition::entity_refs() const {
    return entity_refs_;
}

const std::vector<RelativeLaneRange>& RelativeClearanceCondition::lane_ranges() const {
    return lane_ranges_;
}

} // namespace scena::ir
