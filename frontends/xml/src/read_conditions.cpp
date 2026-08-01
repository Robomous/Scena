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

#include "read_conditions.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "read_common.h"
#include "read_geometry.h"
#include "scena/ir/condition.h"
#include "scena/ir/coordinate_system.h"
#include "scena/ir/entity_condition.h"
#include "scena/ir/evaluation_context.h"
#include "scena/ir/interaction_condition.h"
#include "scena/ir/rule.h"

namespace scena::xml::detail {

namespace {

constexpr std::initializer_list<EnumEntry<ir::Rule>> kRules = {
    {"equalTo", ir::Rule::EqualTo},         {"greaterThan", ir::Rule::GreaterThan},
    {"lessThan", ir::Rule::LessThan},       {"greaterOrEqual", ir::Rule::GreaterOrEqual},
    {"lessOrEqual", ir::Rule::LessOrEqual}, {"notEqualTo", ir::Rule::NotEqualTo},
};

constexpr std::initializer_list<EnumEntry<ir::ConditionEdge>> kEdges = {
    {"none", ir::ConditionEdge::None},
    {"rising", ir::ConditionEdge::Rising},
    {"falling", ir::ConditionEdge::Falling},
    {"risingOrFalling", ir::ConditionEdge::RisingOrFalling},
};

constexpr std::initializer_list<EnumEntry<ir::TriggeringEntitiesRule>> kTriggeringRules = {
    {"any", ir::TriggeringEntitiesRule::Any},
    {"all", ir::TriggeringEntitiesRule::All},
};

constexpr std::initializer_list<EnumEntry<ir::DirectionalDimension>> kDirections = {
    {"longitudinal", ir::DirectionalDimension::Longitudinal},
    {"lateral", ir::DirectionalDimension::Lateral},
    {"vertical", ir::DirectionalDimension::Vertical},
};

constexpr std::initializer_list<EnumEntry<ir::CoordinateSystem>> kCoordinateSystems = {
    {"entity", ir::CoordinateSystem::Entity}, {"lane", ir::CoordinateSystem::Lane},
    {"road", ir::CoordinateSystem::Road},     {"trajectory", ir::CoordinateSystem::Trajectory},
    {"world", ir::CoordinateSystem::World},
};

constexpr std::initializer_list<EnumEntry<ir::RelativeDistanceType>> kDistanceTypes = {
    {"longitudinal", ir::RelativeDistanceType::Longitudinal},
    {"lateral", ir::RelativeDistanceType::Lateral},
    {"cartesianDistance", ir::RelativeDistanceType::CartesianDistance},
    {"euclidianDistance", ir::RelativeDistanceType::EuclidianDistance},
};

constexpr std::initializer_list<EnumEntry<ir::RoutingAlgorithm>> kRoutingAlgorithms = {
    {"assignedRoute", ir::RoutingAlgorithm::AssignedRoute},
    {"fastest", ir::RoutingAlgorithm::Fastest},
    {"leastIntersections", ir::RoutingAlgorithm::LeastIntersections},
    {"shortest", ir::RoutingAlgorithm::Shortest},
    {"undefined", ir::RoutingAlgorithm::Undefined},
};

constexpr std::initializer_list<EnumEntry<ir::StoryboardElementType>> kElementTypes = {
    {"story", ir::StoryboardElementType::Story},
    {"act", ir::StoryboardElementType::Act},
    {"maneuverGroup", ir::StoryboardElementType::ManeuverGroup},
    {"maneuver", ir::StoryboardElementType::Maneuver},
    {"event", ir::StoryboardElementType::Event},
    {"action", ir::StoryboardElementType::Action},
};

constexpr std::initializer_list<EnumEntry<ir::StoryboardElementState>> kElementStates = {
    {"standbyState", ir::StoryboardElementState::StandbyState},
    {"runningState", ir::StoryboardElementState::RunningState},
    {"completeState", ir::StoryboardElementState::CompleteState},
    {"startTransition", ir::StoryboardElementState::StartTransition},
    {"endTransition", ir::StoryboardElementState::EndTransition},
    {"stopTransition", ir::StoryboardElementState::StopTransition},
    {"skipTransition", ir::StoryboardElementState::SkipTransition},
};

/// Reads an optional enumeration attribute into an optional: absent stays
/// nullopt, which is what the conditions read as "the standard's default,
/// decided at evaluation time".
template <typename T>
bool optional_enum(ReadContext& ctx, const pugi::xml_node& node, const char* name,
                   std::initializer_list<EnumEntry<T>> entries, std::optional<T>& out) {
    if (!node.attribute(name)) {
        return true;
    }
    T value{};
    if (!read_enum(ctx, node, name, entries, value)) {
        return false;
    }
    out = value;
    return true;
}

/// Reads the optional `alongRoute` attribute, deprecated in 1.1 in favor of
/// coordinateSystem/relativeDistanceType.
bool optional_along_route(ReadContext& ctx, const pugi::xml_node& node, std::optional<bool>& out) {
    if (!node.attribute("alongRoute")) {
        return true;
    }
    warn_deprecated(ctx, node, "use coordinateSystem and relativeDistanceType (1.1)");
    bool value = false;
    if (!optional_bool(ctx, node, "alongRoute", value)) {
        return false;
    }
    out = value;
    return true;
}

std::shared_ptr<ir::Condition> read_by_value(ReadContext& ctx, const pugi::xml_node& node) {
    static const char* const kVariants[] = {"ParameterCondition",
                                            "VariableCondition",
                                            "TimeOfDayCondition",
                                            "SimulationTimeCondition",
                                            "StoryboardElementStateCondition",
                                            "UserDefinedValueCondition",
                                            "TrafficSignalCondition",
                                            "TrafficSignalControllerCondition",
                                            nullptr};
    const pugi::xml_node variant = read_choice(ctx, node, kVariants);
    if (!variant) {
        return nullptr;
    }
    const std::string_view name = variant.name();

    if (name == "SimulationTimeCondition") {
        double value = 0.0;
        ir::Rule rule = ir::Rule::GreaterOrEqual;
        bool ok = require_double(ctx, variant, "value", value);
        ok = read_enum(ctx, variant, "rule", kRules, rule) && ok;
        return ok ? std::make_shared<ir::SimulationTimeCondition>(value, rule) : nullptr;
    }
    if (name == "ParameterCondition" || name == "VariableCondition") {
        const bool is_parameter = name == "ParameterCondition";
        std::string reference;
        std::string value;
        ir::Rule rule = ir::Rule::EqualTo;
        bool ok =
            require_string(ctx, variant, is_parameter ? "parameterRef" : "variableRef", reference);
        ok = require_string(ctx, variant, "value", value) && ok;
        ok = read_enum(ctx, variant, "rule", kRules, rule) && ok;
        if (!ok) {
            return nullptr;
        }
        if (is_parameter) {
            return std::make_shared<ir::ParameterCondition>(std::move(reference), rule,
                                                            std::move(value));
        }
        return std::make_shared<ir::VariableCondition>(std::move(reference), rule,
                                                       std::move(value));
    }
    if (name == "UserDefinedValueCondition") {
        std::string value_name;
        std::string value;
        ir::Rule rule = ir::Rule::EqualTo;
        bool ok = require_string(ctx, variant, "name", value_name);
        ok = require_string(ctx, variant, "value", value) && ok;
        ok = read_enum(ctx, variant, "rule", kRules, rule) && ok;
        return ok ? std::make_shared<ir::UserDefinedValueCondition>(std::move(value_name), rule,
                                                                    std::move(value))
                  : nullptr;
    }
    if (name == "TimeOfDayCondition") {
        ir::DateTime date_time;
        ir::Rule rule = ir::Rule::EqualTo;
        bool ok = read_date_time(ctx, variant, "dateTime", date_time);
        ok = read_enum(ctx, variant, "rule", kRules, rule) && ok;
        return ok ? std::make_shared<ir::TimeOfDayCondition>(date_time, rule) : nullptr;
    }
    if (name == "TrafficSignalCondition") {
        std::string signal_name;
        std::string state;
        bool ok = require_string(ctx, variant, "name", signal_name);
        ok = require_string(ctx, variant, "state", state) && ok;
        return ok ? std::make_shared<ir::TrafficSignalCondition>(std::move(signal_name),
                                                                 std::move(state))
                  : nullptr;
    }
    if (name == "TrafficSignalControllerCondition") {
        std::string controller_ref;
        std::string phase;
        bool ok = require_string(ctx, variant, "trafficSignalControllerRef", controller_ref);
        ok = require_string(ctx, variant, "phase", phase) && ok;
        return ok ? std::make_shared<ir::TrafficSignalControllerCondition>(
                        std::move(controller_ref), std::move(phase))
                  : nullptr;
    }

    // StoryboardElementStateCondition.
    ir::StoryboardElementType element_type = ir::StoryboardElementType::Event;
    ir::StoryboardElementState state = ir::StoryboardElementState::CompleteState;
    std::string element_ref;
    bool ok = read_enum(ctx, variant, "storyboardElementType", kElementTypes, element_type);
    ok = require_string(ctx, variant, "storyboardElementRef", element_ref) && ok;
    ok = read_enum(ctx, variant, "state", kElementStates, state) && ok;
    return ok ? std::make_shared<ir::StoryboardElementStateCondition>(element_type,
                                                                      std::move(element_ref), state)
              : nullptr;
}

/// Reads `TriggeringEntities` (§7.6.5.1): the any/all rule plus the entity
/// references it reduces over.
bool read_triggering_entities(ReadContext& ctx, const pugi::xml_node& node,
                              ir::TriggeringEntities& out) {
    const pugi::xml_node triggering = require_child(ctx, node, "TriggeringEntities");
    if (!triggering) {
        return false;
    }
    bool ok = read_enum(ctx, triggering, "triggeringEntitiesRule", kTriggeringRules, out.rule);
    for (pugi::xml_node reference : triggering.children("EntityRef")) {
        std::string entity_ref;
        if (require_string(ctx, reference, "entityRef", entity_ref)) {
            out.entity_refs.push_back(std::move(entity_ref));
        } else {
            ok = false;
        }
    }
    if (out.entity_refs.empty()) {
        ctx.report_at(triggering, Severity::Error, Status::ValidationError,
                      element_path(triggering), "TriggeringEntities names no entity");
        ok = false;
    }
    return ok;
}

/// A world position read from a condition's `Position` element. The
/// position-valued conditions (ReachPosition, Distance) take a WorldPosition
/// in the IR, so only that variant lowers; the rest are reported.
bool read_condition_position(ReadContext& ctx, const pugi::xml_node& node, ir::WorldPosition& out) {
    const pugi::xml_node position = require_child(ctx, node, "Position");
    if (!position) {
        return false;
    }
    const std::optional<ir::Position> read = read_position(ctx, position);
    if (!read.has_value()) {
        return false;
    }
    if (const auto* world = std::get_if<ir::WorldPosition>(&*read)) {
        out = *world;
        return true;
    }
    warn_out_of_scope(ctx, position,
                      "this condition compares against a world position; other position "
                      "variants need resolution at evaluation time (deferred)");
    return false;
}

std::shared_ptr<ir::Condition> read_entity_condition(ReadContext& ctx, const pugi::xml_node& node,
                                                     ir::TriggeringEntities triggering) {
    static const char* const kVariants[] = {"EndOfRoadCondition",
                                            "CollisionCondition",
                                            "OffroadCondition",
                                            "TimeHeadwayCondition",
                                            "TimeToCollisionCondition",
                                            "AccelerationCondition",
                                            "StandStillCondition",
                                            "SpeedCondition",
                                            "RelativeSpeedCondition",
                                            "TraveledDistanceCondition",
                                            "ReachPositionCondition",
                                            "DistanceCondition",
                                            "RelativeDistanceCondition",
                                            "RelativeClearanceCondition",
                                            "AngleCondition",
                                            "RelativeAngleCondition",
                                            nullptr};
    const pugi::xml_node variant = read_choice(ctx, node, kVariants);
    if (!variant) {
        return nullptr;
    }
    const std::string_view name = variant.name();

    if (name == "SpeedCondition" || name == "AccelerationCondition") {
        double value = 0.0;
        ir::Rule rule = ir::Rule::GreaterThan;
        std::optional<ir::DirectionalDimension> direction;
        bool ok = require_double(ctx, variant, "value", value);
        ok = read_enum(ctx, variant, "rule", kRules, rule) && ok;
        ok = optional_enum(ctx, variant, "direction", kDirections, direction) && ok;
        if (!ok) {
            return nullptr;
        }
        if (name == "SpeedCondition") {
            return std::make_shared<ir::SpeedCondition>(std::move(triggering), value, rule,
                                                        direction);
        }
        return std::make_shared<ir::AccelerationCondition>(std::move(triggering), value, rule,
                                                           direction);
    }
    if (name == "RelativeSpeedCondition") {
        std::string entity_ref;
        double value = 0.0;
        ir::Rule rule = ir::Rule::GreaterThan;
        std::optional<ir::DirectionalDimension> direction;
        bool ok = require_string(ctx, variant, "entityRef", entity_ref);
        ok = require_double(ctx, variant, "value", value) && ok;
        ok = read_enum(ctx, variant, "rule", kRules, rule) && ok;
        ok = optional_enum(ctx, variant, "direction", kDirections, direction) && ok;
        return ok ? std::make_shared<ir::RelativeSpeedCondition>(
                        std::move(triggering), std::move(entity_ref), value, rule, direction)
                  : nullptr;
    }
    if (name == "StandStillCondition" || name == "EndOfRoadCondition" ||
        name == "OffroadCondition") {
        double duration = 0.0;
        if (!require_double(ctx, variant, "duration", duration)) {
            return nullptr;
        }
        if (name == "StandStillCondition") {
            return std::make_shared<ir::StandStillCondition>(std::move(triggering), duration);
        }
        if (name == "EndOfRoadCondition") {
            return std::make_shared<ir::EndOfRoadCondition>(std::move(triggering), duration);
        }
        return std::make_shared<ir::OffroadCondition>(std::move(triggering), duration);
    }
    if (name == "TraveledDistanceCondition") {
        double value = 0.0;
        return require_double(ctx, variant, "value", value)
                   ? std::make_shared<ir::TraveledDistanceCondition>(std::move(triggering), value)
                   : nullptr;
    }
    if (name == "CollisionCondition") {
        // The By-type alternative (collision against an object category) is
        // not modeled: the IR condition takes an entity reference.
        const pugi::xml_node entity = variant.child("EntityRef");
        if (!entity) {
            warn_out_of_scope(ctx, variant,
                              "collision against an object type is outside the loaded subset");
            return nullptr;
        }
        std::string entity_ref;
        return require_string(ctx, entity, "entityRef", entity_ref)
                   ? std::make_shared<ir::CollisionCondition>(std::move(triggering),
                                                              std::move(entity_ref))
                   : nullptr;
    }
    if (name == "ReachPositionCondition") {
        // Deprecated in 1.2 in favor of DistanceCondition; still executed.
        warn_deprecated(ctx, variant, "use DistanceCondition (1.2)");
        double tolerance = 0.0;
        ir::WorldPosition position;
        bool ok = require_double(ctx, variant, "tolerance", tolerance);
        ok = read_condition_position(ctx, variant, position) && ok;
        return ok ? std::make_shared<ir::ReachPositionCondition>(std::move(triggering), position,
                                                                 tolerance)
                  : nullptr;
    }
    if (name == "DistanceCondition") {
        double value = 0.0;
        bool freespace = false;
        ir::Rule rule = ir::Rule::LessThan;
        ir::WorldPosition position;
        std::optional<ir::CoordinateSystem> coordinate_system;
        std::optional<ir::RelativeDistanceType> distance_type;
        std::optional<ir::RoutingAlgorithm> routing;
        std::optional<bool> along_route;
        bool ok = require_double(ctx, variant, "value", value);
        ok = optional_bool(ctx, variant, "freespace", freespace) && ok;
        ok = read_enum(ctx, variant, "rule", kRules, rule) && ok;
        ok = optional_enum(ctx, variant, "coordinateSystem", kCoordinateSystems,
                           coordinate_system) &&
             ok;
        ok = optional_enum(ctx, variant, "relativeDistanceType", kDistanceTypes, distance_type) &&
             ok;
        ok = optional_enum(ctx, variant, "routingAlgorithm", kRoutingAlgorithms, routing) && ok;
        ok = optional_along_route(ctx, variant, along_route) && ok;
        ok = read_condition_position(ctx, variant, position) && ok;
        return ok ? std::make_shared<ir::DistanceCondition>(std::move(triggering), position, value,
                                                            freespace, rule, coordinate_system,
                                                            distance_type, routing, along_route)
                  : nullptr;
    }
    if (name == "RelativeDistanceCondition") {
        std::string entity_ref;
        double value = 0.0;
        bool freespace = false;
        ir::RelativeDistanceType distance_type = ir::RelativeDistanceType::EuclidianDistance;
        ir::Rule rule = ir::Rule::LessThan;
        std::optional<ir::CoordinateSystem> coordinate_system;
        std::optional<ir::RoutingAlgorithm> routing;
        bool ok = require_string(ctx, variant, "entityRef", entity_ref);
        ok = require_double(ctx, variant, "value", value) && ok;
        ok = optional_bool(ctx, variant, "freespace", freespace) && ok;
        ok = read_enum(ctx, variant, "relativeDistanceType", kDistanceTypes, distance_type) && ok;
        ok = read_enum(ctx, variant, "rule", kRules, rule) && ok;
        ok = optional_enum(ctx, variant, "coordinateSystem", kCoordinateSystems,
                           coordinate_system) &&
             ok;
        ok = optional_enum(ctx, variant, "routingAlgorithm", kRoutingAlgorithms, routing) && ok;
        return ok ? std::make_shared<ir::RelativeDistanceCondition>(
                        std::move(triggering), std::move(entity_ref), value, freespace,
                        distance_type, rule, coordinate_system, routing)
                  : nullptr;
    }
    if (name == "TimeHeadwayCondition") {
        std::string entity_ref;
        double value = 0.0;
        bool freespace = false;
        ir::Rule rule = ir::Rule::LessThan;
        std::optional<ir::CoordinateSystem> coordinate_system;
        std::optional<ir::RelativeDistanceType> distance_type;
        std::optional<ir::RoutingAlgorithm> routing;
        std::optional<bool> along_route;
        bool ok = require_string(ctx, variant, "entityRef", entity_ref);
        ok = require_double(ctx, variant, "value", value) && ok;
        ok = optional_bool(ctx, variant, "freespace", freespace) && ok;
        ok = read_enum(ctx, variant, "rule", kRules, rule) && ok;
        ok = optional_enum(ctx, variant, "coordinateSystem", kCoordinateSystems,
                           coordinate_system) &&
             ok;
        ok = optional_enum(ctx, variant, "relativeDistanceType", kDistanceTypes, distance_type) &&
             ok;
        ok = optional_enum(ctx, variant, "routingAlgorithm", kRoutingAlgorithms, routing) && ok;
        ok = optional_along_route(ctx, variant, along_route) && ok;
        return ok ? std::make_shared<ir::TimeHeadwayCondition>(
                        std::move(triggering), std::move(entity_ref), value, freespace, rule,
                        coordinate_system, distance_type, routing, along_route)
                  : nullptr;
    }
    if (name == "TimeToCollisionCondition") {
        double value = 0.0;
        bool freespace = false;
        ir::Rule rule = ir::Rule::LessThan;
        std::optional<ir::CoordinateSystem> coordinate_system;
        std::optional<ir::RelativeDistanceType> distance_type;
        std::optional<ir::RoutingAlgorithm> routing;
        std::optional<bool> along_route;
        bool ok = require_double(ctx, variant, "value", value);
        ok = optional_bool(ctx, variant, "freespace", freespace) && ok;
        ok = read_enum(ctx, variant, "rule", kRules, rule) && ok;
        ok = optional_enum(ctx, variant, "coordinateSystem", kCoordinateSystems,
                           coordinate_system) &&
             ok;
        ok = optional_enum(ctx, variant, "relativeDistanceType", kDistanceTypes, distance_type) &&
             ok;
        ok = optional_enum(ctx, variant, "routingAlgorithm", kRoutingAlgorithms, routing) && ok;
        ok = optional_along_route(ctx, variant, along_route) && ok;

        // The target is a choice of an entity reference or a position
        // (§TimeToCollisionConditionTarget).
        const pugi::xml_node target_node =
            require_child(ctx, variant, "TimeToCollisionConditionTarget");
        if (!target_node || !ok) {
            return nullptr;
        }
        ir::TimeToCollisionTarget target;
        if (const pugi::xml_node entity = target_node.child("EntityRef")) {
            std::string entity_ref;
            if (!require_string(ctx, entity, "entityRef", entity_ref)) {
                return nullptr;
            }
            target = std::move(entity_ref);
        } else {
            ir::WorldPosition position;
            if (!read_condition_position(ctx, target_node, position)) {
                return nullptr;
            }
            target = position;
        }
        return std::make_shared<ir::TimeToCollisionCondition>(
            std::move(triggering), std::move(target), value, freespace, rule, coordinate_system,
            distance_type, routing, along_route);
    }
    if (name == "RelativeClearanceCondition") {
        bool free_space = false;
        bool opposite_lanes = false;
        double distance_backward = 0.0;
        double distance_forward = 0.0;
        std::vector<std::string> entity_refs;
        std::vector<ir::RelativeLaneRange> lane_ranges;
        bool ok = optional_bool(ctx, variant, "freeSpace", free_space);
        ok = optional_bool(ctx, variant, "oppositeLanes", opposite_lanes) && ok;
        ok = optional_double(ctx, variant, "distanceBackward", distance_backward) && ok;
        ok = optional_double(ctx, variant, "distanceForward", distance_forward) && ok;
        for (pugi::xml_node reference : variant.children("EntityRef")) {
            std::string entity_ref;
            if (require_string(ctx, reference, "entityRef", entity_ref)) {
                entity_refs.push_back(std::move(entity_ref));
            } else {
                ok = false;
            }
        }
        for (pugi::xml_node range : variant.children("RelativeLaneRange")) {
            ir::RelativeLaneRange entry;
            // Absent bounds mean "unbounded on that side", so each is read
            // into the optional only when the attribute is there.
            if (range.attribute("from")) {
                int from = 0;
                ok = optional_int(ctx, range, "from", from) && ok;
                entry.from = from;
            }
            if (range.attribute("to")) {
                int to = 0;
                ok = optional_int(ctx, range, "to", to) && ok;
                entry.to = to;
            }
            lane_ranges.push_back(entry);
        }
        return ok ? std::make_shared<ir::RelativeClearanceCondition>(
                        std::move(triggering), free_space, opposite_lanes, distance_backward,
                        distance_forward, std::move(entity_refs), std::move(lane_ranges))
                  : nullptr;
    }

    // AngleCondition / RelativeAngleCondition are 1.4 additions and have no
    // IR counterpart in the targeted 1.0-1.3 range.
    warn_out_of_scope(ctx, variant, "this condition is outside the 1.0-1.3 loaded subset");
    return nullptr;
}

std::shared_ptr<ir::Condition> read_condition_expression(ReadContext& ctx,
                                                         const pugi::xml_node& node) {
    static const char* const kVariants[] = {"ByEntityCondition", "ByValueCondition", nullptr};
    const pugi::xml_node variant = read_choice(ctx, node, kVariants);
    if (!variant) {
        return nullptr;
    }
    if (std::string_view(variant.name()) == "ByValueCondition") {
        return read_by_value(ctx, variant);
    }
    ir::TriggeringEntities triggering;
    if (!read_triggering_entities(ctx, variant, triggering)) {
        return nullptr;
    }
    const pugi::xml_node entity_condition = require_child(ctx, variant, "EntityCondition");
    if (!entity_condition) {
        return nullptr;
    }
    return read_entity_condition(ctx, entity_condition, std::move(triggering));
}

} // namespace

std::optional<ir::Trigger> read_trigger(ReadContext& ctx, const pugi::xml_node& node) {
    static const char* const kConsumed[] = {"ConditionGroup", nullptr};
    warn_unconsumed_children(ctx, node, kConsumed);

    ir::Trigger trigger;
    bool ok = true;
    for (pugi::xml_node group_node : node.children("ConditionGroup")) {
        static const char* const kGroupConsumed[] = {"Condition", nullptr};
        warn_unconsumed_children(ctx, group_node, kGroupConsumed);

        ir::ConditionGroup group;
        for (pugi::xml_node condition_node : group_node.children("Condition")) {
            ir::TriggerCondition condition;
            bool condition_ok = require_string(ctx, condition_node, "name", condition.name);
            condition_ok =
                optional_double(ctx, condition_node, "delay", condition.delay) && condition_ok;
            condition_ok =
                read_enum(ctx, condition_node, "conditionEdge", kEdges, condition.edge) &&
                condition_ok;
            if (condition.delay < 0.0) {
                ctx.report_at(condition_node, Severity::Error, Status::ValidationError,
                              attribute_path(condition_node, "delay"),
                              "condition delay must not be negative",
                              "asam.net:xosc:1.0.0:data_type.condition_delay_not_negative");
                condition_ok = false;
            }
            condition.expression = read_condition_expression(ctx, condition_node);
            if (!condition.expression || !condition_ok) {
                // A condition that could not be lowered would silently weaken
                // the AND it belongs to, so the whole trigger is rejected.
                ok = false;
                continue;
            }
            group.conditions.push_back(std::move(condition));
        }
        if (group.conditions.empty()) {
            ctx.report_at(group_node, Severity::Error, Status::ValidationError,
                          element_path(group_node), "ConditionGroup declares no condition");
            ok = false;
            continue;
        }
        trigger.groups.push_back(std::move(group));
    }
    return ok ? std::optional<ir::Trigger>(std::move(trigger)) : std::nullopt;
}

} // namespace scena::xml::detail
