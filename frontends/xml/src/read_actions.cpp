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

#include "read_actions.h"

#include <optional>
#include <utility>

#include "catalog.h"
#include "parameters.h"
#include "read_common.h"
#include "read_entities.h"
#include "read_geometry.h"
#include "scena/ir/controller.h"
#include "scena/ir/coordinate_system.h"
#include "scena/ir/environment.h"

namespace scena::xml::detail {

namespace {

constexpr std::initializer_list<EnumEntry<ir::SpeedTargetValueType>> kSpeedTargetValueTypes = {
    {"delta", ir::SpeedTargetValueType::Delta},
    {"factor", ir::SpeedTargetValueType::Factor},
};

constexpr std::initializer_list<EnumEntry<ir::CoordinateSystem>> kCoordinateSystems = {
    {"entity", ir::CoordinateSystem::Entity}, {"lane", ir::CoordinateSystem::Lane},
    {"road", ir::CoordinateSystem::Road},     {"trajectory", ir::CoordinateSystem::Trajectory},
    {"world", ir::CoordinateSystem::World},
};

constexpr std::initializer_list<EnumEntry<ir::LongitudinalDisplacement>>
    kLongitudinalDisplacements = {
        {"any", ir::LongitudinalDisplacement::Any},
        {"trailingReferencedEntity", ir::LongitudinalDisplacement::TrailingReferencedEntity},
        {"leadingReferencedEntity", ir::LongitudinalDisplacement::LeadingReferencedEntity},
};

constexpr std::initializer_list<EnumEntry<ir::LateralDisplacement>> kLateralDisplacements = {
    {"any", ir::LateralDisplacement::Any},
    {"leftToReferencedEntity", ir::LateralDisplacement::LeftToReferencedEntity},
    {"rightToReferencedEntity", ir::LateralDisplacement::RightToReferencedEntity},
};

constexpr std::initializer_list<EnumEntry<ir::DynamicsShape>> kDynamicsShapes = {
    {"linear", ir::DynamicsShape::Linear},
    {"cubic", ir::DynamicsShape::Cubic},
    {"sinusoidal", ir::DynamicsShape::Sinusoidal},
    {"step", ir::DynamicsShape::Step},
};

constexpr std::initializer_list<EnumEntry<ir::FollowingMode>> kFollowingModes = {
    {"position", ir::FollowingMode::Position},
    {"follow", ir::FollowingMode::Follow},
};

constexpr std::initializer_list<EnumEntry<ir::ControllerType>> kControllerTypes = {
    {"lateral", ir::ControllerType::Lateral},   {"longitudinal", ir::ControllerType::Longitudinal},
    {"lighting", ir::ControllerType::Lighting}, {"animation", ir::ControllerType::Animation},
    {"movement", ir::ControllerType::Movement}, {"appearance", ir::ControllerType::Appearance},
    {"all", ir::ControllerType::All},
};

constexpr std::initializer_list<EnumEntry<ir::PrecipitationType>> kPrecipitationTypes = {
    {"dry", ir::PrecipitationType::Dry},
    {"rain", ir::PrecipitationType::Rain},
    {"snow", ir::PrecipitationType::Snow},
};

constexpr std::initializer_list<EnumEntry<ir::ReferenceContext>> kReferenceContexts = {
    {"absolute", ir::ReferenceContext::Absolute},
    {"relative", ir::ReferenceContext::Relative},
};

/// FractionalCloudCover (§1.2) spells okta counts as words; the IR carries the
/// count.
constexpr std::initializer_list<EnumEntry<int>> kCloudCovers = {
    {"zeroOktas", 0}, {"oneOktas", 1}, {"twoOktas", 2},   {"threeOktas", 3}, {"fourOktas", 4},
    {"fiveOktas", 5}, {"sixOktas", 6}, {"sevenOktas", 7}, {"eightOktas", 8}, {"nineOktas", 9},
};

/// Reads a DynamicConstraints element (§DynamicConstraints), present on the
/// distance-keeping actions.
std::optional<ir::DynamicConstraints> read_dynamic_constraints(ReadContext& ctx,
                                                               const pugi::xml_node& node) {
    if (!node) {
        return std::nullopt;
    }
    ir::DynamicConstraints constraints;
    (void)optional_double(ctx, node, "maxAcceleration", constraints.max_acceleration);
    (void)optional_double(ctx, node, "maxAccelerationRate", constraints.max_acceleration_rate);
    (void)optional_double(ctx, node, "maxDeceleration", constraints.max_deceleration);
    (void)optional_double(ctx, node, "maxDecelerationRate", constraints.max_deceleration_rate);
    (void)optional_double(ctx, node, "maxSpeed", constraints.max_speed);
    return constraints;
}

std::shared_ptr<ir::Action> read_speed_action(ReadContext& ctx, const pugi::xml_node& node,
                                              const std::string& entity_id) {
    ir::TransitionDynamics dynamics;
    const pugi::xml_node dynamics_node = require_child(ctx, node, "SpeedActionDynamics");
    if (!dynamics_node || !read_transition_dynamics(ctx, dynamics_node, dynamics)) {
        return nullptr;
    }
    const pugi::xml_node target = require_child(ctx, node, "SpeedActionTarget");
    if (!target) {
        return nullptr;
    }
    static const char* const kTargets[] = {"AbsoluteTargetSpeed", "RelativeTargetSpeed", nullptr};
    const pugi::xml_node chosen = read_choice(ctx, target, kTargets);
    if (!chosen) {
        return nullptr;
    }
    if (std::string_view(chosen.name()) == "AbsoluteTargetSpeed") {
        double value = 0.0;
        if (!require_double(ctx, chosen, "value", value)) {
            return nullptr;
        }
        return std::make_shared<ir::SpeedAction>(entity_id, value, dynamics);
    }
    ir::RelativeTargetSpeed relative;
    bool ok = require_string(ctx, chosen, "entityRef", relative.entity_ref);
    ok = require_double(ctx, chosen, "value", relative.value) && ok;
    ok = read_enum(ctx, chosen, "speedTargetValueType", kSpeedTargetValueTypes,
                   relative.value_type) &&
         ok;
    ok = optional_bool(ctx, chosen, "continuous", relative.continuous) && ok;
    return ok ? std::make_shared<ir::SpeedAction>(entity_id, std::move(relative), dynamics)
              : nullptr;
}

std::shared_ptr<ir::Action> read_speed_profile_action(ReadContext& ctx, const pugi::xml_node& node,
                                                      const std::string& entity_id) {
    static const char* const kConsumed[] = {"SpeedProfileEntry", "DynamicConstraints", nullptr};
    warn_unconsumed_children(ctx, node, kConsumed);
    if (node.attribute("entityRef")) {
        // The entity-relative profile is deferred with the rest of the
        // follow-mode work (ADR-0011, #62).
        warn_out_of_scope(ctx, node, "entity-relative speed profiles are deferred (#62)");
    }
    if (const pugi::xml_node constraints = node.child("DynamicConstraints")) {
        warn_out_of_scope(ctx, constraints, "speed-profile dynamic constraints are deferred (#62)");
    }

    ir::FollowingMode following_mode = ir::FollowingMode::Position;
    bool ok = read_enum(ctx, node, "followingMode", kFollowingModes, following_mode);
    std::vector<ir::SpeedProfileEntry> entries;
    for (pugi::xml_node entry_node : node.children("SpeedProfileEntry")) {
        ir::SpeedProfileEntry entry;
        ok = require_double(ctx, entry_node, "speed", entry.speed) && ok;
        ok = optional_double(ctx, entry_node, "time", entry.time) && ok;
        entries.push_back(entry);
    }
    if (entries.empty()) {
        ctx.report_at(node, Severity::Error, Status::ValidationError, element_path(node),
                      "SpeedProfileAction declares no entry");
        return nullptr;
    }
    return ok ? std::make_shared<ir::SpeedProfileAction>(entity_id, std::move(entries),
                                                         following_mode)
              : nullptr;
}

std::shared_ptr<ir::Action> read_longitudinal_distance_action(ReadContext& ctx,
                                                              const pugi::xml_node& node,
                                                              const std::string& entity_id) {
    std::string entity_ref;
    std::optional<double> distance;
    std::optional<double> time_gap;
    bool freespace = false;
    bool continuous = true;
    ir::CoordinateSystem coordinate_system = ir::CoordinateSystem::Entity;
    ir::LongitudinalDisplacement displacement =
        ir::LongitudinalDisplacement::TrailingReferencedEntity;

    bool ok = require_string(ctx, node, "entityRef", entity_ref);
    ok = optional_double(ctx, node, "distance", distance) && ok;
    ok = optional_double(ctx, node, "timeGap", time_gap) && ok;
    ok = optional_bool(ctx, node, "freespace", freespace) && ok;
    ok = optional_bool(ctx, node, "continuous", continuous) && ok;
    ok = read_enum(ctx, node, "coordinateSystem", kCoordinateSystems, coordinate_system) && ok;
    ok = read_enum(ctx, node, "displacement", kLongitudinalDisplacements, displacement) && ok;
    if (distance.has_value() == time_gap.has_value()) {
        ctx.report_at(node, Severity::Error, Status::ValidationError, element_path(node),
                      "LongitudinalDistanceAction needs exactly one of 'distance' and 'timeGap'");
        ok = false;
    }
    std::optional<ir::DynamicConstraints> constraints =
        read_dynamic_constraints(ctx, node.child("DynamicConstraints"));
    return ok ? std::make_shared<ir::LongitudinalDistanceAction>(
                    entity_id, std::move(entity_ref), distance, time_gap, freespace, continuous,
                    coordinate_system, displacement, constraints)
              : nullptr;
}

std::shared_ptr<ir::Action> read_lane_change_action(ReadContext& ctx, const pugi::xml_node& node,
                                                    const std::string& entity_id) {
    ir::TransitionDynamics dynamics;
    const pugi::xml_node dynamics_node = require_child(ctx, node, "LaneChangeActionDynamics");
    if (!dynamics_node || !read_transition_dynamics(ctx, dynamics_node, dynamics)) {
        return nullptr;
    }
    double target_offset = 0.0;
    bool ok = optional_double(ctx, node, "targetLaneOffset", target_offset);

    const pugi::xml_node target = require_child(ctx, node, "LaneChangeTarget");
    if (!target) {
        return nullptr;
    }
    static const char* const kTargets[] = {"AbsoluteTargetLane", "RelativeTargetLane", nullptr};
    const pugi::xml_node chosen = read_choice(ctx, target, kTargets);
    if (!chosen) {
        return nullptr;
    }
    if (std::string_view(chosen.name()) == "AbsoluteTargetLane") {
        ir::AbsoluteTargetLane absolute;
        ok = require_string(ctx, chosen, "value", absolute.value) && ok;
        return ok ? std::make_shared<ir::LaneChangeAction>(entity_id, std::move(absolute), dynamics,
                                                           target_offset)
                  : nullptr;
    }
    ir::RelativeTargetLane relative;
    ok = require_string(ctx, chosen, "entityRef", relative.entity_ref) && ok;
    ok = require_int(ctx, chosen, "value", relative.value) && ok;
    return ok ? std::make_shared<ir::LaneChangeAction>(entity_id, std::move(relative), dynamics,
                                                       target_offset)
              : nullptr;
}

std::shared_ptr<ir::Action> read_lane_offset_action(ReadContext& ctx, const pugi::xml_node& node,
                                                    const std::string& entity_id) {
    bool continuous = true;
    bool ok = optional_bool(ctx, node, "continuous", continuous);

    ir::DynamicsShape shape = ir::DynamicsShape::Linear;
    std::optional<double> max_lateral_acc;
    const pugi::xml_node dynamics = require_child(ctx, node, "LaneOffsetActionDynamics");
    if (!dynamics) {
        return nullptr;
    }
    ok = read_enum(ctx, dynamics, "dynamicsShape", kDynamicsShapes, shape) && ok;
    ok = optional_double(ctx, dynamics, "maxLateralAcc", max_lateral_acc) && ok;

    const pugi::xml_node target = require_child(ctx, node, "LaneOffsetTarget");
    if (!target) {
        return nullptr;
    }
    static const char* const kTargets[] = {"AbsoluteTargetLaneOffset", "RelativeTargetLaneOffset",
                                           nullptr};
    const pugi::xml_node chosen = read_choice(ctx, target, kTargets);
    if (!chosen) {
        return nullptr;
    }
    if (std::string_view(chosen.name()) == "AbsoluteTargetLaneOffset") {
        ir::AbsoluteTargetLaneOffset absolute;
        ok = require_double(ctx, chosen, "value", absolute.value) && ok;
        return ok ? std::make_shared<ir::LaneOffsetAction>(entity_id, absolute, continuous, shape,
                                                           max_lateral_acc)
                  : nullptr;
    }
    ir::RelativeTargetLaneOffset relative;
    ok = require_string(ctx, chosen, "entityRef", relative.entity_ref) && ok;
    ok = require_double(ctx, chosen, "value", relative.value) && ok;
    return ok ? std::make_shared<ir::LaneOffsetAction>(entity_id, std::move(relative), continuous,
                                                       shape, max_lateral_acc)
              : nullptr;
}

std::shared_ptr<ir::Action> read_lateral_distance_action(ReadContext& ctx,
                                                         const pugi::xml_node& node,
                                                         const std::string& entity_id) {
    std::string entity_ref;
    double distance = 0.0;
    bool freespace = false;
    bool continuous = true;
    ir::CoordinateSystem coordinate_system = ir::CoordinateSystem::Entity;
    ir::LateralDisplacement displacement = ir::LateralDisplacement::Any;

    bool ok = require_string(ctx, node, "entityRef", entity_ref);
    // "Missing value is interpreted as 0" (§LateralDistanceAction).
    ok = optional_double(ctx, node, "distance", distance) && ok;
    ok = optional_bool(ctx, node, "freespace", freespace) && ok;
    ok = optional_bool(ctx, node, "continuous", continuous) && ok;
    ok = read_enum(ctx, node, "coordinateSystem", kCoordinateSystems, coordinate_system) && ok;
    ok = read_enum(ctx, node, "displacement", kLateralDisplacements, displacement) && ok;
    std::optional<ir::DynamicConstraints> constraints =
        read_dynamic_constraints(ctx, node.child("DynamicConstraints"));
    return ok ? std::make_shared<ir::LateralDistanceAction>(
                    entity_id, std::move(entity_ref), distance, freespace, continuous,
                    coordinate_system, displacement, constraints)
              : nullptr;
}

std::shared_ptr<ir::Action> read_routing_action(ReadContext& ctx, const pugi::xml_node& node,
                                                const std::string& entity_id) {
    static const char* const kVariants[] = {"AssignRouteAction", "FollowTrajectoryAction",
                                            "AcquirePositionAction", "RandomRouteAction", nullptr};
    const pugi::xml_node variant = read_choice(ctx, node, kVariants);
    if (!variant) {
        return nullptr;
    }
    const std::string_view name = variant.name();

    if (name == "AssignRouteAction") {
        pugi::xml_node route_node = variant.child("Route");
        std::optional<CatalogEntryScope> scope;
        if (const pugi::xml_node catalog = variant.child("CatalogReference")) {
            route_node = ctx.catalogs().resolve(ctx, catalog, CatalogKind::Route, "Route");
            if (!route_node) {
                return nullptr;
            }
            scope.emplace(ctx, catalog, route_node);
        }
        if (!route_node) {
            (void)require_child(ctx, variant, "Route");
            return nullptr;
        }
        const std::shared_ptr<ir::Route> route = read_route(ctx, route_node);
        if (!route) {
            return nullptr;
        }
        return std::make_shared<ir::AssignRouteAction>(entity_id, *route);
    }
    if (name == "AcquirePositionAction") {
        const pugi::xml_node position_node = require_child(ctx, variant, "Position");
        if (!position_node) {
            return nullptr;
        }
        std::optional<ir::Position> position = read_position(ctx, position_node);
        if (!position.has_value()) {
            return nullptr;
        }
        return std::make_shared<ir::AcquirePositionAction>(entity_id, std::move(*position));
    }
    if (name == "RandomRouteAction") {
        // A random route would need a random generator in the runtime, which
        // the determinism contract forbids (ADR-0001).
        warn_out_of_scope(ctx, variant, "random routing is excluded by the determinism contract");
        return nullptr;
    }

    // FollowTrajectoryAction.
    static const char* const kConsumed[] = {
        "Trajectory",    "CatalogReference",        "TrajectoryRef",
        "TimeReference", "TrajectoryFollowingMode", nullptr};
    warn_unconsumed_children(ctx, variant, kConsumed);

    // 1.1 moved the trajectory behind a TrajectoryRef wrapper; both spellings
    // occur in the targeted range.
    pugi::xml_node trajectory_node = variant.child("Trajectory");
    std::optional<CatalogEntryScope> trajectory_scope;
    pugi::xml_node trajectory_catalog = variant.child("CatalogReference");
    if (const pugi::xml_node reference = variant.child("TrajectoryRef")) {
        trajectory_node = reference.child("Trajectory");
        trajectory_catalog = reference.child("CatalogReference");
    }
    if (!trajectory_node && trajectory_catalog) {
        trajectory_node =
            ctx.catalogs().resolve(ctx, trajectory_catalog, CatalogKind::Trajectory, "Trajectory");
        if (!trajectory_node) {
            return nullptr;
        }
        trajectory_scope.emplace(ctx, trajectory_catalog, trajectory_node);
    }
    if (!trajectory_node) {
        ctx.report_at(variant, Severity::Error, Status::ValidationError, element_path(variant),
                      "FollowTrajectoryAction references no trajectory");
        return nullptr;
    }
    const std::shared_ptr<ir::Trajectory> trajectory = read_trajectory(ctx, trajectory_node);
    if (!trajectory) {
        return nullptr;
    }

    ir::FollowingMode following_mode = ir::FollowingMode::Position;
    bool ok = true;
    if (const pugi::xml_node mode = variant.child("TrajectoryFollowingMode")) {
        ok = read_enum(ctx, mode, "followingMode", kFollowingModes, following_mode) && ok;
    }
    double initial_offset = 0.0;
    ok = optional_double(ctx, variant, "initialDistanceOffset", initial_offset) && ok;

    std::optional<ir::Timing> timing;
    if (const pugi::xml_node reference = variant.child("TimeReference")) {
        if (const pugi::xml_node timing_node = reference.child("Timing")) {
            ir::Timing value;
            ok = read_enum(ctx, timing_node, "domainAbsoluteRelative", kReferenceContexts,
                           value.domain) &&
                 ok;
            ok = require_double(ctx, timing_node, "scale", value.scale) && ok;
            ok = require_double(ctx, timing_node, "offset", value.offset) && ok;
            timing = value;
        }
        // <None/> means "ignore the vertex times", which is the absent Timing.
    }
    return ok ? std::make_shared<ir::FollowTrajectoryAction>(entity_id, *trajectory, following_mode,
                                                             timing, initial_offset)
              : nullptr;
}

std::shared_ptr<ir::Action> read_controller_action(ReadContext& ctx, const pugi::xml_node& node,
                                                   const std::string& entity_id) {
    static const char* const kVariants[] = {
        "AssignControllerAction",   "OverrideControllerValueAction",
        "ActivateControllerAction", "AnimationAction",
        "LightStateAction",         nullptr};
    const pugi::xml_node variant = read_choice(ctx, node, kVariants);
    if (!variant) {
        return nullptr;
    }
    const std::string_view name = variant.name();

    if (name == "AssignControllerAction") {
        pugi::xml_node controller_node = variant.child("Controller");
        std::optional<CatalogEntryScope> scope;
        if (const pugi::xml_node catalog = variant.child("CatalogReference")) {
            controller_node =
                ctx.catalogs().resolve(ctx, catalog, CatalogKind::Controller, "Controller");
            if (!controller_node) {
                return nullptr;
            }
            scope.emplace(ctx, catalog, controller_node);
        }
        if (!controller_node) {
            (void)require_child(ctx, variant, "Controller");
            return nullptr;
        }
        ir::Controller controller;
        bool ok = require_string(ctx, controller_node, "name", controller.name);
        ok = read_enum(ctx, controller_node, "controllerType", kControllerTypes, controller.type) &&
             ok;
        read_properties(ctx, controller_node.child("Properties"), controller.properties);

        std::optional<bool> lateral;
        std::optional<bool> longitudinal;
        if (variant.attribute("activateLateral")) {
            bool value = false;
            ok = optional_bool(ctx, variant, "activateLateral", value) && ok;
            lateral = value;
        }
        if (variant.attribute("activateLongitudinal")) {
            bool value = false;
            ok = optional_bool(ctx, variant, "activateLongitudinal", value) && ok;
            longitudinal = value;
        }
        return ok ? std::make_shared<ir::AssignControllerAction>(entity_id, std::move(controller),
                                                                 lateral, longitudinal)
                  : nullptr;
    }
    if (name == "ActivateControllerAction") {
        std::optional<bool> lateral;
        std::optional<bool> longitudinal;
        bool ok = true;
        if (variant.attribute("lateral")) {
            bool value = false;
            ok = optional_bool(ctx, variant, "lateral", value) && ok;
            lateral = value;
        }
        if (variant.attribute("longitudinal")) {
            bool value = false;
            ok = optional_bool(ctx, variant, "longitudinal", value) && ok;
            longitudinal = value;
        }
        return ok ? std::make_shared<ir::ActivateControllerAction>(entity_id, lateral, longitudinal)
                  : nullptr;
    }
    warn_out_of_scope(ctx, variant, "this controller action has no runtime domain in Scena");
    return nullptr;
}

std::shared_ptr<ir::Action> read_appearance_action(ReadContext& ctx, const pugi::xml_node& node,
                                                   const std::string& entity_id) {
    static const char* const kVariants[] = {"VisibilityAction", "AnimationAction",
                                            "LightStateAction", nullptr};
    const pugi::xml_node variant = read_choice(ctx, node, kVariants);
    if (!variant) {
        return nullptr;
    }
    if (std::string_view(variant.name()) != "VisibilityAction") {
        warn_out_of_scope(ctx, variant, "lighting and animation have no runtime domain in Scena");
        return nullptr;
    }
    if (const pugi::xml_node sensors = variant.child("SensorReferenceSet")) {
        warn_out_of_scope(ctx, sensors, "individual sensor references are not modeled");
    }
    bool graphics = true;
    bool sensors = true;
    bool traffic = true;
    bool ok = optional_bool(ctx, variant, "graphics", graphics);
    ok = optional_bool(ctx, variant, "sensors", sensors) && ok;
    ok = optional_bool(ctx, variant, "traffic", traffic) && ok;
    return ok ? std::make_shared<ir::VisibilityAction>(entity_id, graphics, sensors, traffic)
              : nullptr;
}

bool read_environment(ReadContext& ctx, const pugi::xml_node& node, ir::Environment& out) {
    static const char* const kConsumed[] = {"TimeOfDay", "Weather", "RoadCondition",
                                            "ParameterDeclarations", nullptr};
    warn_unconsumed_children(ctx, node, kConsumed);
    const ParameterFrame frame(ctx.parameters());
    if (const pugi::xml_node declarations = node.child("ParameterDeclarations")) {
        read_parameter_declarations(ctx, declarations);
    }

    bool ok = require_string(ctx, node, "name", out.name);
    if (const pugi::xml_node time_of_day = node.child("TimeOfDay")) {
        ir::TimeOfDay value;
        ok = optional_bool(ctx, time_of_day, "animation", value.animation) && ok;
        ok = read_date_time(ctx, time_of_day, "dateTime", value.date_time) && ok;
        out.time_of_day = value;
    }
    if (const pugi::xml_node weather = node.child("Weather")) {
        ir::Weather value;
        if (const pugi::xml_node sun = weather.child("Sun")) {
            ir::Sun entry;
            ok = optional_double(ctx, sun, "azimuth", entry.azimuth) && ok;
            ok = optional_double(ctx, sun, "elevation", entry.elevation) && ok;
            ok = optional_double(ctx, sun, "illuminance", entry.illuminance) && ok;
            // 1.0 spelled the intensity "intensity"; 1.2 renamed it.
            ok = optional_double(ctx, sun, "intensity", entry.illuminance) && ok;
            value.sun = entry;
        }
        if (const pugi::xml_node fog = weather.child("Fog")) {
            ir::Fog entry;
            ok = require_double(ctx, fog, "visualRange", entry.visual_range) && ok;
            value.fog = entry;
        }
        if (const pugi::xml_node precipitation = weather.child("Precipitation")) {
            ir::Precipitation entry;
            ok = read_enum(ctx, precipitation, "precipitationType", kPrecipitationTypes,
                           entry.type) &&
                 ok;
            ok = optional_double(ctx, precipitation, "precipitationIntensity", entry.intensity) &&
                 ok;
            ok = optional_double(ctx, precipitation, "intensity", entry.intensity) && ok;
            value.precipitation = entry;
        }
        if (const pugi::xml_node wind = weather.child("Wind")) {
            ir::Wind entry;
            ok = require_double(ctx, wind, "direction", entry.direction) && ok;
            ok = require_double(ctx, wind, "speed", entry.speed) && ok;
            value.wind = entry;
        }
        ok = optional_double(ctx, weather, "temperature", value.temperature) && ok;
        ok = optional_double(ctx, weather, "atmosphericPressure", value.atmospheric_pressure) && ok;
        if (weather.attribute("fractionalCloudCover")) {
            int oktas = 0;
            ok = read_enum(ctx, weather, "fractionalCloudCover", kCloudCovers, oktas) && ok;
            value.fractional_cloud_cover_oktas = oktas;
        }
        if (const pugi::xml_node dome = weather.child("DomeImage")) {
            warn_out_of_scope(ctx, dome, "sky dome images are a rendering concern");
        }
        out.weather = value;
    }
    if (const pugi::xml_node road = node.child("RoadCondition")) {
        ir::RoadCondition value;
        ok = require_double(ctx, road, "frictionScaleFactor", value.friction_scale_factor) && ok;
        out.road_condition = value;
    }
    return ok;
}

} // namespace

bool read_traffic_signal_controller(ReadContext& ctx, const pugi::xml_node& node,
                                    ir::TrafficSignalController& out) {
    static const char* const kConsumed[] = {"Phase", nullptr};
    warn_unconsumed_children(ctx, node, kConsumed);

    bool ok = require_string(ctx, node, "name", out.name);
    if (node.attribute("delay")) {
        double delay = 0.0;
        ok = optional_double(ctx, node, "delay", delay) && ok;
        out.delay = delay;
    }
    if (const pugi::xml_attribute reference = node.attribute("reference")) {
        out.reference = reference.value();
    }
    for (pugi::xml_node phase_node : node.children("Phase")) {
        ir::Phase phase;
        ok = require_string(ctx, phase_node, "name", phase.name) && ok;
        ok = require_double(ctx, phase_node, "duration", phase.duration) && ok;
        for (pugi::xml_node state_node : phase_node.children("TrafficSignalState")) {
            ir::TrafficSignalState state;
            ok = require_string(ctx, state_node, "trafficSignalId", state.traffic_signal_id) && ok;
            ok = require_string(ctx, state_node, "state", state.state) && ok;
            phase.signal_states.push_back(std::move(state));
        }
        if (const pugi::xml_node group = phase_node.child("TrafficSignalGroupState")) {
            warn_out_of_scope(ctx, group, "traffic signal group states are 1.4-only");
        }
        out.phases.push_back(std::move(phase));
    }
    return ok;
}

std::shared_ptr<ir::Action> read_private_action(ReadContext& ctx, const pugi::xml_node& node,
                                                const std::string& entity_id) {
    static const char* const kVariants[] = {"LongitudinalAction",
                                            "LateralAction",
                                            "VisibilityAction",
                                            "SynchronizeAction",
                                            "ActivateControllerAction",
                                            "ControllerAction",
                                            "TeleportAction",
                                            "RoutingAction",
                                            "AppearanceAction",
                                            "TrailerAction",
                                            nullptr};
    const pugi::xml_node variant = read_choice(ctx, node, kVariants);
    if (!variant) {
        return nullptr;
    }
    const std::string_view name = variant.name();

    if (name == "TeleportAction") {
        const pugi::xml_node position_node = require_child(ctx, variant, "Position");
        if (!position_node) {
            return nullptr;
        }
        std::optional<ir::Position> position = read_position(ctx, position_node);
        if (!position.has_value()) {
            return nullptr;
        }
        return std::make_shared<ir::TeleportAction>(entity_id, std::move(*position));
    }
    if (name == "LongitudinalAction") {
        static const char* const kLongitudinal[] = {"SpeedAction", "LongitudinalDistanceAction",
                                                    "SpeedProfileAction", nullptr};
        const pugi::xml_node chosen = read_choice(ctx, variant, kLongitudinal);
        if (!chosen) {
            return nullptr;
        }
        const std::string_view longitudinal = chosen.name();
        if (longitudinal == "SpeedAction") {
            return read_speed_action(ctx, chosen, entity_id);
        }
        if (longitudinal == "SpeedProfileAction") {
            return read_speed_profile_action(ctx, chosen, entity_id);
        }
        return read_longitudinal_distance_action(ctx, chosen, entity_id);
    }
    if (name == "LateralAction") {
        static const char* const kLateral[] = {"LaneChangeAction", "LaneOffsetAction",
                                               "LateralDistanceAction", nullptr};
        const pugi::xml_node chosen = read_choice(ctx, variant, kLateral);
        if (!chosen) {
            return nullptr;
        }
        const std::string_view lateral = chosen.name();
        if (lateral == "LaneChangeAction") {
            return read_lane_change_action(ctx, chosen, entity_id);
        }
        if (lateral == "LaneOffsetAction") {
            return read_lane_offset_action(ctx, chosen, entity_id);
        }
        return read_lateral_distance_action(ctx, chosen, entity_id);
    }
    if (name == "RoutingAction") {
        return read_routing_action(ctx, variant, entity_id);
    }
    if (name == "ControllerAction") {
        return read_controller_action(ctx, variant, entity_id);
    }
    if (name == "ActivateControllerAction") {
        // Deprecated placement: 1.1 moved it under ControllerAction. Still
        // executed, exactly as the coverage matrix promises.
        warn_deprecated(ctx, variant, "use ControllerAction/ActivateControllerAction (1.1)");
        std::optional<bool> lateral;
        std::optional<bool> longitudinal;
        bool ok = true;
        if (variant.attribute("lateral")) {
            bool value = false;
            ok = optional_bool(ctx, variant, "lateral", value) && ok;
            lateral = value;
        }
        if (variant.attribute("longitudinal")) {
            bool value = false;
            ok = optional_bool(ctx, variant, "longitudinal", value) && ok;
            longitudinal = value;
        }
        return ok ? std::make_shared<ir::ActivateControllerAction>(entity_id, lateral, longitudinal)
                  : nullptr;
    }
    if (name == "VisibilityAction") {
        // 1.0-1.1 placement, directly under PrivateAction.
        bool graphics = true;
        bool sensors = true;
        bool traffic = true;
        bool ok = optional_bool(ctx, variant, "graphics", graphics);
        ok = optional_bool(ctx, variant, "sensors", sensors) && ok;
        ok = optional_bool(ctx, variant, "traffic", traffic) && ok;
        return ok ? std::make_shared<ir::VisibilityAction>(entity_id, graphics, sensors, traffic)
                  : nullptr;
    }
    if (name == "AppearanceAction") {
        return read_appearance_action(ctx, variant, entity_id);
    }
    if (name == "SynchronizeAction") {
        warn_out_of_scope(ctx, variant, "SynchronizeAction is Post-v0.0.1");
        return nullptr;
    }
    warn_out_of_scope(ctx, variant, "trailer actions are outside the v0.0.1 scope");
    return nullptr;
}

std::shared_ptr<ir::Action> read_global_action(ReadContext& ctx, const pugi::xml_node& node) {
    if (std::string_view(node.name()) == "UserDefinedAction") {
        const pugi::xml_node command = require_child(ctx, node, "CustomCommandAction");
        if (!command) {
            return nullptr;
        }
        std::string type;
        if (!require_string(ctx, command, "type", type)) {
            return nullptr;
        }
        // The command payload is the element's text content (§7.4.3).
        return std::make_shared<ir::CustomCommandAction>(std::move(type),
                                                         command.text().as_string());
    }

    static const char* const kVariants[] = {
        "EnvironmentAction", "EntityAction",   "InfrastructureAction", "SetMonitorAction",
        "ParameterAction",   "VariableAction", "TrafficAction",        nullptr};
    const pugi::xml_node variant = read_choice(ctx, node, kVariants);
    if (!variant) {
        return nullptr;
    }
    const std::string_view name = variant.name();

    if (name == "EnvironmentAction") {
        pugi::xml_node environment_node = variant.child("Environment");
        std::optional<CatalogEntryScope> scope;
        if (const pugi::xml_node catalog = variant.child("CatalogReference")) {
            environment_node =
                ctx.catalogs().resolve(ctx, catalog, CatalogKind::Environment, "Environment");
            if (!environment_node) {
                return nullptr;
            }
            scope.emplace(ctx, catalog, environment_node);
        }
        if (!environment_node) {
            (void)require_child(ctx, variant, "Environment");
            return nullptr;
        }
        ir::Environment environment;
        return read_environment(ctx, environment_node, environment)
                   ? std::make_shared<ir::EnvironmentAction>(std::move(environment))
                   : nullptr;
    }
    if (name == "EntityAction") {
        std::string entity_ref;
        if (!require_string(ctx, variant, "entityRef", entity_ref)) {
            return nullptr;
        }
        static const char* const kEntityActions[] = {"AddEntityAction", "DeleteEntityAction",
                                                     nullptr};
        const pugi::xml_node chosen = read_choice(ctx, variant, kEntityActions);
        if (!chosen) {
            return nullptr;
        }
        if (std::string_view(chosen.name()) == "DeleteEntityAction") {
            return std::make_shared<ir::DeleteEntityAction>(std::move(entity_ref));
        }
        const pugi::xml_node position_node = require_child(ctx, chosen, "Position");
        if (!position_node) {
            return nullptr;
        }
        std::optional<ir::Position> position = read_position(ctx, position_node);
        if (!position.has_value()) {
            return nullptr;
        }
        return std::make_shared<ir::AddEntityAction>(std::move(entity_ref), std::move(*position));
    }
    if (name == "InfrastructureAction") {
        const pugi::xml_node signal = require_child(ctx, variant, "TrafficSignalAction");
        if (!signal) {
            return nullptr;
        }
        static const char* const kSignalActions[] = {"TrafficSignalControllerAction",
                                                     "TrafficSignalStateAction", nullptr};
        const pugi::xml_node chosen = read_choice(ctx, signal, kSignalActions);
        if (!chosen) {
            return nullptr;
        }
        if (std::string_view(chosen.name()) == "TrafficSignalStateAction") {
            std::string signal_name;
            std::string state;
            bool ok = require_string(ctx, chosen, "name", signal_name);
            ok = require_string(ctx, chosen, "state", state) && ok;
            return ok ? std::make_shared<ir::TrafficSignalStateAction>(std::move(signal_name),
                                                                       std::move(state))
                      : nullptr;
        }
        std::string controller_ref;
        std::string phase;
        bool ok = require_string(ctx, chosen, "trafficSignalControllerRef", controller_ref);
        ok = require_string(ctx, chosen, "phase", phase) && ok;
        return ok ? std::make_shared<ir::TrafficSignalControllerAction>(std::move(controller_ref),
                                                                        std::move(phase))
                  : nullptr;
    }
    if (name == "ParameterAction" || name == "VariableAction") {
        const bool is_parameter = name == "ParameterAction";
        if (is_parameter) {
            // Deprecated in 1.2 in favor of VariableAction; still executed.
            warn_deprecated(ctx, variant, "use VariableAction (1.2)");
        }
        std::string reference;
        if (!require_string(ctx, variant, is_parameter ? "parameterRef" : "variableRef",
                            reference)) {
            return nullptr;
        }
        static const char* const kParameterActions[] = {"SetAction", "ModifyAction", nullptr};
        const pugi::xml_node chosen = read_choice(ctx, variant, kParameterActions);
        if (!chosen) {
            return nullptr;
        }
        if (std::string_view(chosen.name()) == "SetAction") {
            std::string value;
            if (!require_string(ctx, chosen, "value", value)) {
                return nullptr;
            }
            if (is_parameter) {
                return std::make_shared<ir::ParameterSetAction>(std::move(reference),
                                                                std::move(value));
            }
            return std::make_shared<ir::VariableSetAction>(std::move(reference), std::move(value));
        }
        const pugi::xml_node rule = require_child(ctx, chosen, "Rule");
        if (!rule) {
            return nullptr;
        }
        static const char* const kOperations[] = {"AddValue", "MultiplyByValue", nullptr};
        const pugi::xml_node operation = read_choice(ctx, rule, kOperations);
        if (!operation) {
            return nullptr;
        }
        const ir::ModifyOperator op = std::string_view(operation.name()) == "AddValue"
                                          ? ir::ModifyOperator::Add
                                          : ir::ModifyOperator::Multiply;
        double value = 0.0;
        if (!require_double(ctx, operation, "value", value)) {
            return nullptr;
        }
        if (is_parameter) {
            return std::make_shared<ir::ParameterModifyAction>(std::move(reference), op, value);
        }
        return std::make_shared<ir::VariableModifyAction>(std::move(reference), op, value);
    }
    if (name == "SetMonitorAction") {
        warn_out_of_scope(ctx, variant, "monitors are Post-v0.0.1");
        return nullptr;
    }
    warn_out_of_scope(ctx, variant, "the traffic family is Post-v0.0.1");
    return nullptr;
}

} // namespace scena::xml::detail
