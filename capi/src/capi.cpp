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

#include "scena/capi.h"

#include <memory>
#include <new>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "scena/diagnostic.h"
#include "scena/engine.h"
#include "scena/entity_visibility.h"
#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/controller.h"
#include "scena/ir/coordinate_system.h"
#include "scena/ir/date_time.h"
#include "scena/ir/environment.h"
#include "scena/ir/route.h"
#include "scena/ir/scenario.h"
#include "scena/ir/traffic_signal.h"
#include "scena/ir/trajectory.h"
#include "scena/status.h"
#include "scena/version.h"

struct scn_engine {
    scena::ir::Scenario scenario;
    scena::Engine engine;
    // Backing store for the last string returned by a get_* borrowed-string
    // accessor; overwritten by the next such call (see capi.h lifetime docs).
    std::string value_buffer;
};

namespace {

scn_status to_c_status(scena::Status status) {
    switch (status) {
    case scena::Status::Ok:
        return SCN_OK;
    case scena::Status::AlreadyInitialized:
        return SCN_ERROR_ALREADY_INITIALIZED;
    case scena::Status::NotInitialized:
        return SCN_ERROR_NOT_INITIALIZED;
    case scena::Status::UnknownEntity:
        return SCN_ERROR_UNKNOWN_ENTITY;
    case scena::Status::InvalidControlMode:
        return SCN_ERROR_INVALID_CONTROL_MODE;
    case scena::Status::InvalidArgument:
        return SCN_ERROR_INVALID_ARGUMENT;
    case scena::Status::ParseError:
        return SCN_ERROR_PARSE;
    case scena::Status::ValidationError:
        return SCN_ERROR_VALIDATION;
    case scena::Status::SemanticError:
        return SCN_ERROR_SEMANTIC;
    case scena::Status::UnsupportedFeature:
        return SCN_ERROR_UNSUPPORTED_FEATURE;
    case scena::Status::UnknownName:
        return SCN_ERROR_UNKNOWN_NAME;
    case scena::Status::DeprecatedFeature:
        return SCN_ERROR_DEPRECATED_FEATURE;
    }
    return SCN_ERROR_INTERNAL;
}

scn_severity to_c_severity(scena::Severity severity) {
    switch (severity) {
    case scena::Severity::Info:
        return SCN_SEVERITY_INFO;
    case scena::Severity::Warning:
        return SCN_SEVERITY_WARNING;
    case scena::Severity::Error:
        return SCN_SEVERITY_ERROR;
    }
    return SCN_SEVERITY_ERROR;
}

scena::EntityState from_c_state(const scn_entity_state& state) {
    return scena::EntityState{state.x,     state.y,     state.z,   state.heading,
                              state.speed, state.pitch, state.roll};
}

scn_entity_state to_c_state(const scena::EntityState& state) {
    return scn_entity_state{state.x,     state.y,     state.z,   state.heading,
                            state.speed, state.pitch, state.roll};
}

scena::ir::ControlMode from_c_control_mode(scn_control_mode mode) {
    return mode == SCN_CONTROL_HOST ? scena::ir::ControlMode::HostControlled
                                    : scena::ir::ControlMode::EngineControlled;
}

scena::ir::BoundingBox from_c_bounding_box(const scn_bounding_box& box) {
    return scena::ir::BoundingBox{box.center_x, box.center_y, box.center_z,
                                  box.length,   box.width,    box.height};
}

scn_bounding_box to_c_bounding_box(const scena::ir::BoundingBox& box) {
    return scn_bounding_box{box.center_x, box.center_y, box.center_z,
                            box.length,   box.width,    box.height};
}

scena::ir::Performance from_c_performance(const scn_performance& perf) {
    scena::ir::Performance out;
    out.max_speed = perf.max_speed;
    out.max_acceleration = perf.max_acceleration;
    out.max_deceleration = perf.max_deceleration;
    // A negative rate on the ABI means "unspecified" (the §Performance default
    // of infinite when the attribute is omitted).
    if (perf.max_acceleration_rate >= 0.0) {
        out.max_acceleration_rate = perf.max_acceleration_rate;
    }
    if (perf.max_deceleration_rate >= 0.0) {
        out.max_deceleration_rate = perf.max_deceleration_rate;
    }
    return out;
}

scn_performance to_c_performance(const scena::ir::Performance& perf) {
    scn_performance out{};
    out.max_speed = perf.max_speed;
    out.max_acceleration = perf.max_acceleration;
    out.max_deceleration = perf.max_deceleration;
    out.max_acceleration_rate = perf.max_acceleration_rate.value_or(-1.0);
    out.max_deceleration_rate = perf.max_deceleration_rate.value_or(-1.0);
    return out;
}

/* Finds an authored entity by id, or nullptr. Reads the scenario under
 * construction, which persists in the handle across init. */
const scena::ir::Entity* find_entity(const scn_engine* engine, const char* id) {
    for (const scena::ir::Entity& entity : engine->scenario.entities) {
        if (entity.id == id) {
            return &entity;
        }
    }
    return nullptr;
}

// The scn_* category enums are defined to mirror the ir enumerations 1:1 and in
// order, so a bounds-checked static_cast is a faithful mapping. Spot-check the
// correspondence at compile time (first, a deprecated middle, and last of each).
static_assert(static_cast<int>(scena::ir::VehicleCategory::Aircraft) == SCN_VEHICLE_AIRCRAFT);
static_assert(static_cast<int>(scena::ir::VehicleCategory::Motorbike) == SCN_VEHICLE_MOTORBIKE);
static_assert(static_cast<int>(scena::ir::VehicleCategory::WorkMachine) ==
              SCN_VEHICLE_WORK_MACHINE);
static_assert(static_cast<int>(scena::ir::PedestrianCategory::Wheelchair) ==
              SCN_PEDESTRIAN_WHEELCHAIR);
static_assert(static_cast<int>(scena::ir::MiscObjectCategory::Barrier) == SCN_MISC_BARRIER);
static_assert(static_cast<int>(scena::ir::MiscObjectCategory::Wind) == SCN_MISC_WIND);
static_assert(static_cast<int>(scena::ir::Role::None) == SCN_ROLE_NONE);
static_assert(static_cast<int>(scena::ir::Role::TrafficControl) == SCN_ROLE_TRAFFIC_CONTROL);

// The transition-dynamics enums mirror the IR enumerations 1:1 and in order.
static_assert(static_cast<int>(scena::ir::DynamicsShape::Linear) == SCN_DYNAMICS_SHAPE_LINEAR);
static_assert(static_cast<int>(scena::ir::DynamicsShape::Step) == SCN_DYNAMICS_SHAPE_STEP);
static_assert(static_cast<int>(scena::ir::DynamicsDimension::Time) == SCN_DYNAMICS_DIMENSION_TIME);
static_assert(static_cast<int>(scena::ir::DynamicsDimension::Rate) == SCN_DYNAMICS_DIMENSION_RATE);
static_assert(static_cast<int>(scena::ir::FollowingMode::Position) == SCN_FOLLOWING_MODE_POSITION);
static_assert(static_cast<int>(scena::ir::FollowingMode::Follow) == SCN_FOLLOWING_MODE_FOLLOW);

// The p5-s5 enums mirror the IR enumerations 1:1 and in order.
static_assert(static_cast<int>(scena::ir::CoordinateSystem::Entity) ==
              SCN_COORDINATE_SYSTEM_ENTITY);
static_assert(static_cast<int>(scena::ir::CoordinateSystem::World) == SCN_COORDINATE_SYSTEM_WORLD);
static_assert(static_cast<int>(scena::ir::LongitudinalDisplacement::Any) ==
              SCN_LONGITUDINAL_DISPLACEMENT_ANY);
static_assert(static_cast<int>(scena::ir::LongitudinalDisplacement::LeadingReferencedEntity) ==
              SCN_LONGITUDINAL_DISPLACEMENT_LEADING);
static_assert(static_cast<int>(scena::ir::RouteStrategy::Fastest) == SCN_ROUTE_STRATEGY_FASTEST);
static_assert(static_cast<int>(scena::ir::RouteStrategy::Shortest) == SCN_ROUTE_STRATEGY_SHORTEST);
static_assert(static_cast<int>(scena::ir::ReferenceContext::Absolute) ==
              SCN_REFERENCE_CONTEXT_ABSOLUTE);
static_assert(static_cast<int>(scena::ir::ReferenceContext::Relative) ==
              SCN_REFERENCE_CONTEXT_RELATIVE);
static_assert(static_cast<int>(scena::ir::ControllerType::Lateral) == SCN_CONTROLLER_TYPE_LATERAL);
static_assert(static_cast<int>(scena::ir::ControllerType::Movement) ==
              SCN_CONTROLLER_TYPE_MOVEMENT);
static_assert(static_cast<int>(scena::ir::ControllerType::All) == SCN_CONTROLLER_TYPE_ALL);

/* A negative value on the ABI means "unspecified" for the [0..inf[ fields of
 * DynamicConstraints, matching the scn_performance rate-limit convention. */
std::optional<double> from_c_optional_limit(double value) {
    if (value >= 0.0) {
        return value;
    }
    return std::nullopt;
}

scena::ir::DynamicConstraints from_c_constraints(const scn_dynamic_constraints& constraints) {
    scena::ir::DynamicConstraints out;
    out.max_acceleration = from_c_optional_limit(constraints.max_acceleration);
    out.max_acceleration_rate = from_c_optional_limit(constraints.max_acceleration_rate);
    out.max_deceleration = from_c_optional_limit(constraints.max_deceleration);
    out.max_deceleration_rate = from_c_optional_limit(constraints.max_deceleration_rate);
    out.max_speed = from_c_optional_limit(constraints.max_speed);
    return out;
}

/* Tri-state activation flag: negative is "no change", zero deactivates,
 * positive activates. */
std::optional<bool> from_c_tristate(int value) {
    if (value < 0) {
        return std::nullopt;
    }
    return value > 0;
}

/* The record of a live entity, or nullptr when the engine is not initialized
 * or the id is unknown. */
bool engine_has_entity(const scn_engine* engine, const char* id) {
    return engine->engine.state(id).has_value();
}

/* Maps and range-checks the priority enum arriving across the ABI. */
bool to_ir_priority(scn_event_priority priority, scena::ir::EventPriority& out) {
    switch (priority) {
    case SCN_PRIORITY_OVERRIDE:
        out = scena::ir::EventPriority::Override;
        return true;
    case SCN_PRIORITY_PARALLEL:
        out = scena::ir::EventPriority::Parallel;
        return true;
    case SCN_PRIORITY_SKIP:
        out = scena::ir::EventPriority::Skip;
        return true;
    }
    return false;
}

/* Appends one event (a SimulationTimeCondition start trigger at `at_time` plus
 * `action`) to a lazily created default Story -> Act -> ManeuverGroup ->
 * Maneuver chain — the flat C builder surface. May throw (allocations); callers
 * wrap it in try/catch. */
void append_storyboard_event(scn_engine* engine, double at_time, scena::ir::EventPriority priority,
                             int maximum_execution_count,
                             std::shared_ptr<scena::ir::Action> action) {
    scena::ir::Storyboard& storyboard = engine->scenario.storyboard;
    if (storyboard.stories.empty()) {
        scena::ir::Story story;
        story.name = "story";
        scena::ir::Act act;
        act.name = "act";
        scena::ir::ManeuverGroup group;
        group.name = "group";
        scena::ir::Maneuver maneuver;
        maneuver.name = "maneuver";
        group.maneuvers.push_back(std::move(maneuver));
        act.groups.push_back(std::move(group));
        story.acts.push_back(std::move(act));
        storyboard.stories.push_back(std::move(story));
    }
    scena::ir::Maneuver& maneuver =
        storyboard.stories.front().acts.front().groups.front().maneuvers.front();
    scena::ir::Event event;
    event.name = "event-" + std::to_string(maneuver.events.size() + 1);
    event.start_trigger =
        scena::ir::make_trigger(std::make_shared<scena::ir::SimulationTimeCondition>(at_time));
    event.priority = priority;
    event.maximum_execution_count = maximum_execution_count;
    event.actions.push_back(std::move(action));
    maneuver.events.push_back(std::move(event));
}

} // namespace

const char* scn_version(void) {
    static const std::string version = scena::version_string();
    return version.c_str();
}

scn_engine* scn_engine_create(void) {
    return new (std::nothrow) scn_engine();
}

void scn_engine_destroy(scn_engine* engine) {
    delete engine;
}

scn_status scn_engine_add_entity(scn_engine* engine, const char* id, const char* name,
                                 scn_control_mode control_mode) {
    if (engine == nullptr || id == nullptr || name == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        scena::ir::Entity entity;
        entity.id = id;
        entity.name = name;
        entity.control_mode = control_mode == SCN_CONTROL_HOST
                                  ? scena::ir::ControlMode::HostControlled
                                  : scena::ir::ControlMode::EngineControlled;
        engine->scenario.entities.push_back(std::move(entity));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_vehicle(scn_engine* engine, const char* id, const char* name,
                                  scn_control_mode control_mode, scn_vehicle_category category,
                                  const scn_bounding_box* bounding_box,
                                  const scn_performance* performance) {
    // The enum arrives across the ABI, so it is range checked (as unsigned, so a
    // negative value wraps above the max) rather than static_cast blindly.
    if (engine == nullptr || id == nullptr || name == nullptr || bounding_box == nullptr ||
        performance == nullptr ||
        static_cast<unsigned>(category) > static_cast<unsigned>(SCN_VEHICLE_WORK_MACHINE)) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        scena::ir::Vehicle vehicle;
        vehicle.category = static_cast<scena::ir::VehicleCategory>(category);
        vehicle.bounding_box = from_c_bounding_box(*bounding_box);
        vehicle.performance = from_c_performance(*performance);
        scena::ir::Entity entity;
        entity.id = id;
        entity.name = name;
        entity.control_mode = from_c_control_mode(control_mode);
        entity.object = std::move(vehicle);
        engine->scenario.entities.push_back(std::move(entity));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_pedestrian(scn_engine* engine, const char* id, const char* name,
                                     scn_control_mode control_mode,
                                     scn_pedestrian_category category,
                                     const scn_bounding_box* bounding_box) {
    if (engine == nullptr || id == nullptr || name == nullptr || bounding_box == nullptr ||
        static_cast<unsigned>(category) > static_cast<unsigned>(SCN_PEDESTRIAN_WHEELCHAIR)) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        scena::ir::Pedestrian pedestrian;
        pedestrian.category = static_cast<scena::ir::PedestrianCategory>(category);
        pedestrian.bounding_box = from_c_bounding_box(*bounding_box);
        scena::ir::Entity entity;
        entity.id = id;
        entity.name = name;
        entity.control_mode = from_c_control_mode(control_mode);
        entity.object = std::move(pedestrian);
        engine->scenario.entities.push_back(std::move(entity));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_misc_object(scn_engine* engine, const char* id, const char* name,
                                      scn_control_mode control_mode,
                                      scn_misc_object_category category,
                                      const scn_bounding_box* bounding_box) {
    if (engine == nullptr || id == nullptr || name == nullptr || bounding_box == nullptr ||
        static_cast<unsigned>(category) > static_cast<unsigned>(SCN_MISC_WIND)) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        scena::ir::MiscObject misc;
        misc.category = static_cast<scena::ir::MiscObjectCategory>(category);
        misc.bounding_box = from_c_bounding_box(*bounding_box);
        scena::ir::Entity entity;
        entity.id = id;
        entity.name = name;
        entity.control_mode = from_c_control_mode(control_mode);
        entity.object = std::move(misc);
        engine->scenario.entities.push_back(std::move(entity));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_speed_action(scn_engine* engine, const char* entity_id,
                                       double target_speed, double at_time) {
    return scn_engine_add_speed_action_ex(engine, entity_id, target_speed, at_time,
                                          SCN_PRIORITY_PARALLEL, 1);
}

scn_status scn_engine_add_speed_action_ex(scn_engine* engine, const char* entity_id,
                                          double target_speed, double at_time,
                                          scn_event_priority priority,
                                          int maximum_execution_count) {
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    if (engine == nullptr || entity_id == nullptr || maximum_execution_count < 0 ||
        !to_ir_priority(priority, ir_priority)) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        append_storyboard_event(engine, at_time, ir_priority, maximum_execution_count,
                                std::make_shared<scena::ir::SpeedAction>(entity_id, target_speed));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_speed_action_dyn(scn_engine* engine, const char* entity_id,
                                           double target_speed,
                                           const scn_transition_dynamics* dynamics, double at_time,
                                           scn_event_priority priority,
                                           int maximum_execution_count) {
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    if (engine == nullptr || entity_id == nullptr || dynamics == nullptr ||
        maximum_execution_count < 0 || !to_ir_priority(priority, ir_priority) ||
        static_cast<unsigned>(dynamics->shape) > static_cast<unsigned>(SCN_DYNAMICS_SHAPE_STEP) ||
        static_cast<unsigned>(dynamics->dimension) >
            static_cast<unsigned>(SCN_DYNAMICS_DIMENSION_RATE) ||
        static_cast<unsigned>(dynamics->following_mode) >
            static_cast<unsigned>(SCN_FOLLOWING_MODE_FOLLOW)) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        const scena::ir::TransitionDynamics td{
            static_cast<scena::ir::DynamicsShape>(dynamics->shape),
            static_cast<scena::ir::DynamicsDimension>(dynamics->dimension), dynamics->value,
            static_cast<scena::ir::FollowingMode>(dynamics->following_mode)};
        append_storyboard_event(
            engine, at_time, ir_priority, maximum_execution_count,
            std::make_shared<scena::ir::SpeedAction>(entity_id, target_speed, td));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_speed_profile_action(scn_engine* engine, const char* entity_id,
                                               const scn_speed_profile_entry* entries,
                                               size_t entry_count,
                                               scn_following_mode following_mode, double at_time,
                                               scn_event_priority priority,
                                               int maximum_execution_count) {
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    if (engine == nullptr || entity_id == nullptr || entries == nullptr || entry_count == 0 ||
        maximum_execution_count < 0 || !to_ir_priority(priority, ir_priority) ||
        static_cast<unsigned>(following_mode) > static_cast<unsigned>(SCN_FOLLOWING_MODE_FOLLOW)) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        std::vector<scena::ir::SpeedProfileEntry> ir_entries;
        ir_entries.reserve(entry_count);
        for (size_t i = 0; i < entry_count; ++i) {
            scena::ir::SpeedProfileEntry entry;
            entry.speed = entries[i].speed;
            // A negative time means "unspecified" across the ABI.
            if (entries[i].time >= 0.0) {
                entry.time = entries[i].time;
            }
            ir_entries.push_back(entry);
        }
        append_storyboard_event(engine, at_time, ir_priority, maximum_execution_count,
                                std::make_shared<scena::ir::SpeedProfileAction>(
                                    entity_id, std::move(ir_entries),
                                    static_cast<scena::ir::FollowingMode>(following_mode)));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_relative_speed_action(
    scn_engine* engine, const char* entity_id, const char* reference_entity_id, double value,
    scn_speed_target_value_type value_type, int continuous, const scn_transition_dynamics* dynamics,
    double at_time, scn_event_priority priority, int maximum_execution_count) {
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    if (engine == nullptr || entity_id == nullptr || reference_entity_id == nullptr ||
        dynamics == nullptr || maximum_execution_count < 0 ||
        !to_ir_priority(priority, ir_priority) ||
        static_cast<unsigned>(value_type) > static_cast<unsigned>(SCN_SPEED_TARGET_FACTOR) ||
        static_cast<unsigned>(dynamics->shape) > static_cast<unsigned>(SCN_DYNAMICS_SHAPE_STEP) ||
        static_cast<unsigned>(dynamics->dimension) >
            static_cast<unsigned>(SCN_DYNAMICS_DIMENSION_RATE) ||
        static_cast<unsigned>(dynamics->following_mode) >
            static_cast<unsigned>(SCN_FOLLOWING_MODE_FOLLOW)) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        const scena::ir::TransitionDynamics td{
            static_cast<scena::ir::DynamicsShape>(dynamics->shape),
            static_cast<scena::ir::DynamicsDimension>(dynamics->dimension), dynamics->value,
            static_cast<scena::ir::FollowingMode>(dynamics->following_mode)};
        const scena::ir::RelativeTargetSpeed target{
            reference_entity_id, value, static_cast<scena::ir::SpeedTargetValueType>(value_type),
            continuous != 0};
        append_storyboard_event(engine, at_time, ir_priority, maximum_execution_count,
                                std::make_shared<scena::ir::SpeedAction>(entity_id, target, td));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_teleport_action(scn_engine* engine, const char* entity_id, double x,
                                          double y, double z, double at_time,
                                          scn_event_priority priority,
                                          int maximum_execution_count) {
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    if (engine == nullptr || entity_id == nullptr || maximum_execution_count < 0 ||
        !to_ir_priority(priority, ir_priority)) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        append_storyboard_event(engine, at_time, ir_priority, maximum_execution_count,
                                std::make_shared<scena::ir::TeleportAction>(
                                    entity_id, scena::ir::WorldPosition{x, y, z}));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

namespace {

/// Shared argument check for the two lane-change builders: everything but the
/// target, which is what distinguishes them.
bool lane_change_arguments_valid(scn_engine* engine, const char* entity_id,
                                 const scn_transition_dynamics* dynamics,
                                 scn_event_priority priority, int maximum_execution_count,
                                 scena::ir::EventPriority& out_priority) {
    return engine != nullptr && entity_id != nullptr && dynamics != nullptr &&
           maximum_execution_count >= 0 && to_ir_priority(priority, out_priority) &&
           static_cast<unsigned>(dynamics->shape) <=
               static_cast<unsigned>(SCN_DYNAMICS_SHAPE_STEP) &&
           static_cast<unsigned>(dynamics->dimension) <=
               static_cast<unsigned>(SCN_DYNAMICS_DIMENSION_RATE) &&
           static_cast<unsigned>(dynamics->following_mode) <=
               static_cast<unsigned>(SCN_FOLLOWING_MODE_FOLLOW);
}

scena::ir::TransitionDynamics from_c_dynamics(const scn_transition_dynamics& dynamics) {
    return scena::ir::TransitionDynamics{
        static_cast<scena::ir::DynamicsShape>(dynamics.shape),
        static_cast<scena::ir::DynamicsDimension>(dynamics.dimension), dynamics.value,
        static_cast<scena::ir::FollowingMode>(dynamics.following_mode)};
}

} // namespace

scn_status scn_engine_add_relative_lane_change_action(scn_engine* engine, const char* entity_id,
                                                      const char* reference_entity_id,
                                                      int lane_delta, double target_lane_offset,
                                                      const scn_transition_dynamics* dynamics,
                                                      double at_time, scn_event_priority priority,
                                                      int maximum_execution_count) {
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    if (reference_entity_id == nullptr ||
        !lane_change_arguments_valid(engine, entity_id, dynamics, priority, maximum_execution_count,
                                     ir_priority)) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        append_storyboard_event(engine, at_time, ir_priority, maximum_execution_count,
                                std::make_shared<scena::ir::LaneChangeAction>(
                                    entity_id,
                                    scena::ir::RelativeTargetLane{reference_entity_id, lane_delta},
                                    from_c_dynamics(*dynamics), target_lane_offset));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_absolute_lane_change_action(scn_engine* engine, const char* entity_id,
                                                      const char* lane_id,
                                                      double target_lane_offset,
                                                      const scn_transition_dynamics* dynamics,
                                                      double at_time, scn_event_priority priority,
                                                      int maximum_execution_count) {
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    if (lane_id == nullptr || lane_id[0] == '\0' ||
        !lane_change_arguments_valid(engine, entity_id, dynamics, priority, maximum_execution_count,
                                     ir_priority)) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        append_storyboard_event(engine, at_time, ir_priority, maximum_execution_count,
                                std::make_shared<scena::ir::LaneChangeAction>(
                                    entity_id, scena::ir::AbsoluteTargetLane{lane_id},
                                    from_c_dynamics(*dynamics), target_lane_offset));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_lane_offset_action(scn_engine* engine, const char* entity_id,
                                             const char* reference_entity_id, double value,
                                             int continuous, scn_dynamics_shape shape,
                                             double max_lateral_acc, double at_time,
                                             scn_event_priority priority,
                                             int maximum_execution_count) {
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    if (engine == nullptr || entity_id == nullptr || maximum_execution_count < 0 ||
        !to_ir_priority(priority, ir_priority) ||
        static_cast<unsigned>(shape) > static_cast<unsigned>(SCN_DYNAMICS_SHAPE_STEP)) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        // On the ABI "unset" is a negative maxLateralAcc, the same convention
        // the distance/timeGap pair uses: the standard's range is [0..inf[ and
        // a missing value means 'inf'.
        std::optional<double> ir_acc;
        if (max_lateral_acc >= 0.0) {
            ir_acc = max_lateral_acc;
        }
        const auto ir_shape = static_cast<scena::ir::DynamicsShape>(shape);
        std::shared_ptr<scena::ir::LaneOffsetAction> action;
        if (reference_entity_id != nullptr) {
            action = std::make_shared<scena::ir::LaneOffsetAction>(
                entity_id, scena::ir::RelativeTargetLaneOffset{reference_entity_id, value},
                continuous != 0, ir_shape, ir_acc);
        } else {
            action = std::make_shared<scena::ir::LaneOffsetAction>(
                entity_id, scena::ir::AbsoluteTargetLaneOffset{value}, continuous != 0, ir_shape,
                ir_acc);
        }
        append_storyboard_event(engine, at_time, ir_priority, maximum_execution_count,
                                std::move(action));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_lateral_distance_action(
    scn_engine* engine, const char* entity_id, const char* reference_entity_id, double distance,
    int freespace, int continuous, scn_coordinate_system coordinate_system,
    scn_lateral_displacement displacement, const scn_dynamic_constraints* constraints,
    double at_time, scn_event_priority priority, int maximum_execution_count) {
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    if (engine == nullptr || entity_id == nullptr || reference_entity_id == nullptr ||
        maximum_execution_count < 0 || !to_ir_priority(priority, ir_priority) ||
        static_cast<unsigned>(coordinate_system) >
            static_cast<unsigned>(SCN_COORDINATE_SYSTEM_WORLD) ||
        static_cast<unsigned>(displacement) >
            static_cast<unsigned>(SCN_LATERAL_DISPLACEMENT_RIGHT)) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        std::optional<scena::ir::DynamicConstraints> ir_constraints;
        if (constraints != nullptr) {
            ir_constraints = from_c_constraints(*constraints);
        }
        append_storyboard_event(engine, at_time, ir_priority, maximum_execution_count,
                                std::make_shared<scena::ir::LateralDistanceAction>(
                                    entity_id, reference_entity_id, distance, freespace != 0,
                                    continuous != 0,
                                    static_cast<scena::ir::CoordinateSystem>(coordinate_system),
                                    static_cast<scena::ir::LateralDisplacement>(displacement),
                                    std::move(ir_constraints)));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_set_default_lane_width(scn_engine* engine, double width) {
    if (engine == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    return to_c_status(engine->engine.set_default_lane_width(width));
}

scn_status scn_engine_get_default_lane_width(scn_engine* engine, double* out) {
    if (engine == nullptr || out == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    *out = engine->engine.default_lane_width();
    return SCN_OK;
}

scn_status scn_engine_add_longitudinal_distance_action(
    scn_engine* engine, const char* entity_id, const char* reference_entity_id, double distance,
    double time_gap, int freespace, int continuous, scn_coordinate_system coordinate_system,
    scn_longitudinal_displacement displacement, const scn_dynamic_constraints* constraints,
    double at_time, scn_event_priority priority, int maximum_execution_count) {
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    // Exactly one of the two targets: on the ABI "not used" is a negative
    // value, since both are Range [0..inf[ in the standard.
    const bool has_distance = distance >= 0.0;
    const bool has_time_gap = time_gap >= 0.0;
    if (engine == nullptr || entity_id == nullptr || reference_entity_id == nullptr ||
        maximum_execution_count < 0 || has_distance == has_time_gap ||
        !to_ir_priority(priority, ir_priority) ||
        static_cast<unsigned>(coordinate_system) >
            static_cast<unsigned>(SCN_COORDINATE_SYSTEM_WORLD) ||
        static_cast<unsigned>(displacement) >
            static_cast<unsigned>(SCN_LONGITUDINAL_DISPLACEMENT_LEADING)) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        std::optional<scena::ir::DynamicConstraints> ir_constraints;
        if (constraints != nullptr) {
            ir_constraints = from_c_constraints(*constraints);
        }
        append_storyboard_event(engine, at_time, ir_priority, maximum_execution_count,
                                std::make_shared<scena::ir::LongitudinalDistanceAction>(
                                    entity_id, reference_entity_id,
                                    has_distance ? std::optional<double>(distance) : std::nullopt,
                                    has_time_gap ? std::optional<double>(time_gap) : std::nullopt,
                                    freespace != 0, continuous != 0,
                                    static_cast<scena::ir::CoordinateSystem>(coordinate_system),
                                    static_cast<scena::ir::LongitudinalDisplacement>(displacement),
                                    std::move(ir_constraints)));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_assign_route_action(scn_engine* engine, const char* entity_id,
                                              const char* name, const scn_waypoint* waypoints,
                                              size_t waypoint_count, int closed, double at_time,
                                              scn_event_priority priority,
                                              int maximum_execution_count) {
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    if (engine == nullptr || entity_id == nullptr || waypoints == nullptr || waypoint_count < 2 ||
        maximum_execution_count < 0 || !to_ir_priority(priority, ir_priority)) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    for (size_t index = 0; index < waypoint_count; ++index) {
        if (static_cast<unsigned>(waypoints[index].strategy) >
            static_cast<unsigned>(SCN_ROUTE_STRATEGY_SHORTEST)) {
            return SCN_ERROR_INVALID_ARGUMENT;
        }
    }
    try {
        scena::ir::Route route;
        route.name = name != nullptr ? name : "";
        route.closed = closed != 0;
        route.waypoints.reserve(waypoint_count);
        for (size_t index = 0; index < waypoint_count; ++index) {
            route.waypoints.push_back(scena::ir::Waypoint{
                scena::ir::WorldPosition{waypoints[index].x, waypoints[index].y,
                                         waypoints[index].z},
                static_cast<scena::ir::RouteStrategy>(waypoints[index].strategy)});
        }
        append_storyboard_event(
            engine, at_time, ir_priority, maximum_execution_count,
            std::make_shared<scena::ir::AssignRouteAction>(entity_id, std::move(route)));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_acquire_position_action(scn_engine* engine, const char* entity_id,
                                                  double x, double y, double z, double at_time,
                                                  scn_event_priority priority,
                                                  int maximum_execution_count) {
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    if (engine == nullptr || entity_id == nullptr || maximum_execution_count < 0 ||
        !to_ir_priority(priority, ir_priority)) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        append_storyboard_event(engine, at_time, ir_priority, maximum_execution_count,
                                std::make_shared<scena::ir::AcquirePositionAction>(
                                    entity_id, scena::ir::WorldPosition{x, y, z}));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_follow_trajectory_action(
    scn_engine* engine, const char* entity_id, const char* name,
    const scn_trajectory_vertex* vertices, size_t vertex_count, int closed,
    scn_following_mode following_mode, const scn_timing* timing, double initial_distance_offset,
    double at_time, scn_event_priority priority, int maximum_execution_count) {
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    if (engine == nullptr || entity_id == nullptr || vertices == nullptr || vertex_count < 2 ||
        maximum_execution_count < 0 || !to_ir_priority(priority, ir_priority) ||
        static_cast<unsigned>(following_mode) > static_cast<unsigned>(SCN_FOLLOWING_MODE_FOLLOW) ||
        (timing != nullptr && static_cast<unsigned>(timing->domain) >
                                  static_cast<unsigned>(SCN_REFERENCE_CONTEXT_RELATIVE))) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        scena::ir::Trajectory trajectory;
        trajectory.name = name != nullptr ? name : "";
        trajectory.closed = closed != 0;
        trajectory.vertices.reserve(vertex_count);
        for (size_t index = 0; index < vertex_count; ++index) {
            trajectory.vertices.push_back(scena::ir::TrajectoryVertex{
                scena::ir::WorldPosition{vertices[index].x, vertices[index].y, vertices[index].z},
                vertices[index].has_time != 0 ? std::optional<double>(vertices[index].time)
                                              : std::nullopt});
        }
        std::optional<scena::ir::Timing> ir_timing;
        if (timing != nullptr) {
            ir_timing = scena::ir::Timing{static_cast<scena::ir::ReferenceContext>(timing->domain),
                                          timing->scale, timing->offset};
        }
        append_storyboard_event(engine, at_time, ir_priority, maximum_execution_count,
                                std::make_shared<scena::ir::FollowTrajectoryAction>(
                                    entity_id, std::move(trajectory),
                                    static_cast<scena::ir::FollowingMode>(following_mode),
                                    ir_timing, initial_distance_offset));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_assign_controller_action(
    scn_engine* engine, const char* entity_id, const char* name,
    scn_controller_type controller_type, const char* const* property_names,
    const char* const* property_values, size_t property_count, int activate_lateral,
    int activate_longitudinal, double at_time, scn_event_priority priority,
    int maximum_execution_count) {
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    if (engine == nullptr || entity_id == nullptr || name == nullptr ||
        maximum_execution_count < 0 || !to_ir_priority(priority, ir_priority) ||
        static_cast<unsigned>(controller_type) > static_cast<unsigned>(SCN_CONTROLLER_TYPE_ALL) ||
        (property_count > 0 && (property_names == nullptr || property_values == nullptr))) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    for (size_t index = 0; index < property_count; ++index) {
        if (property_names[index] == nullptr || property_values[index] == nullptr) {
            return SCN_ERROR_INVALID_ARGUMENT;
        }
    }
    try {
        scena::ir::Controller controller;
        controller.name = name;
        controller.type = static_cast<scena::ir::ControllerType>(controller_type);
        controller.properties.reserve(property_count);
        for (size_t index = 0; index < property_count; ++index) {
            controller.properties.push_back(
                scena::ir::Property{property_names[index], property_values[index]});
        }
        append_storyboard_event(engine, at_time, ir_priority, maximum_execution_count,
                                std::make_shared<scena::ir::AssignControllerAction>(
                                    entity_id, std::move(controller),
                                    from_c_tristate(activate_lateral),
                                    from_c_tristate(activate_longitudinal)));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_activate_controller_action(scn_engine* engine, const char* entity_id,
                                                     int lateral, int longitudinal, double at_time,
                                                     scn_event_priority priority,
                                                     int maximum_execution_count) {
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    if (engine == nullptr || entity_id == nullptr || maximum_execution_count < 0 ||
        !to_ir_priority(priority, ir_priority)) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        append_storyboard_event(
            engine, at_time, ir_priority, maximum_execution_count,
            std::make_shared<scena::ir::ActivateControllerAction>(
                entity_id, from_c_tristate(lateral), from_c_tristate(longitudinal)));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_visibility_action(scn_engine* engine, const char* entity_id, int graphics,
                                            int sensors, int traffic, double at_time,
                                            scn_event_priority priority,
                                            int maximum_execution_count) {
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    if (engine == nullptr || entity_id == nullptr || maximum_execution_count < 0 ||
        !to_ir_priority(priority, ir_priority)) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        append_storyboard_event(engine, at_time, ir_priority, maximum_execution_count,
                                std::make_shared<scena::ir::VisibilityAction>(
                                    entity_id, graphics != 0, sensors != 0, traffic != 0));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_entity_visibility(scn_engine* engine, const char* id,
                                        scn_entity_visibility* out) {
    if (engine == nullptr || id == nullptr || out == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    const std::optional<scena::EntityVisibility> visibility = engine->engine.visibility_of(id);
    if (!visibility.has_value()) {
        return SCN_ERROR_UNKNOWN_ENTITY;
    }
    out->graphics = visibility->graphics ? 1 : 0;
    out->sensors = visibility->sensors ? 1 : 0;
    out->traffic = visibility->traffic ? 1 : 0;
    return SCN_OK;
}

scn_status scn_engine_entity_controller_activation(scn_engine* engine, const char* id,
                                                   scn_controller_activation* out) {
    if (engine == nullptr || id == nullptr || out == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    const std::optional<scena::ControllerActivation> activation =
        engine->engine.controller_activation_of(id);
    if (!activation.has_value()) {
        return SCN_ERROR_UNKNOWN_ENTITY;
    }
    out->lateral = activation->lateral ? 1 : 0;
    out->longitudinal = activation->longitudinal ? 1 : 0;
    return SCN_OK;
}

scn_status scn_engine_entity_controller_type(scn_engine* engine, const char* id,
                                             scn_controller_type* out) {
    if (engine == nullptr || id == nullptr || out == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    if (!engine_has_entity(engine, id)) {
        return SCN_ERROR_UNKNOWN_ENTITY;
    }
    const scena::ir::Controller* controller = engine->engine.assigned_controller_of(id);
    if (controller == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT; // no controller assigned
    }
    *out = static_cast<scn_controller_type>(controller->type);
    return SCN_OK;
}

scn_status scn_engine_entity_controller_name(scn_engine* engine, const char* id, const char** out) {
    if (engine == nullptr || id == nullptr || out == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    if (!engine_has_entity(engine, id)) {
        return SCN_ERROR_UNKNOWN_ENTITY;
    }
    const scena::ir::Controller* controller = engine->engine.assigned_controller_of(id);
    if (controller == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        // Borrowed through the handle's buffer, like scn_engine_get_variable.
        engine->value_buffer = controller->name;
        *out = engine->value_buffer.c_str();
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_entity_route_info(scn_engine* engine, const char* id,
                                        size_t* out_waypoint_count, int* out_closed) {
    if (engine == nullptr || id == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    if (!engine_has_entity(engine, id)) {
        return SCN_ERROR_UNKNOWN_ENTITY;
    }
    const scena::ir::Route* route = engine->engine.route_of(id);
    if (route == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT; // no route assigned
    }
    if (out_waypoint_count != nullptr) {
        *out_waypoint_count = route->waypoints.size();
    }
    if (out_closed != nullptr) {
        *out_closed = route->closed ? 1 : 0;
    }
    return SCN_OK;
}

scn_status scn_engine_entity_route_waypoint_at(scn_engine* engine, const char* id, size_t index,
                                               scn_waypoint* out) {
    if (engine == nullptr || id == nullptr || out == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    if (!engine_has_entity(engine, id)) {
        return SCN_ERROR_UNKNOWN_ENTITY;
    }
    const scena::ir::Route* route = engine->engine.route_of(id);
    if (route == nullptr || index >= route->waypoints.size()) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    const scena::ir::Waypoint& waypoint = route->waypoints[index];
    out->x = waypoint.position.x;
    out->y = waypoint.position.y;
    out->z = waypoint.position.z;
    out->strategy = static_cast<scn_route_strategy>(waypoint.strategy);
    return SCN_OK;
}

scn_status scn_engine_init(scn_engine* engine) {
    if (engine == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        // Init on a copy so the builder scenario stays intact for re-init
        // after close.
        return to_c_status(engine->engine.init(engine->scenario));
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_step(scn_engine* engine, double dt) {
    if (engine == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        return to_c_status(engine->engine.step(dt));
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_close(scn_engine* engine) {
    if (engine == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        return to_c_status(engine->engine.close());
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_diagnostic_count(scn_engine* engine, size_t* out_count) {
    if (engine == nullptr || out_count == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        *out_count = engine->engine.diagnostics().size();
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_diagnostic_at(scn_engine* engine, size_t index, scn_diagnostic* out) {
    if (engine == nullptr || out == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        const auto& diagnostics = engine->engine.diagnostics();
        if (index >= diagnostics.size()) {
            return SCN_ERROR_INVALID_ARGUMENT; // out left untouched
        }
        const scena::Diagnostic& diagnostic = diagnostics[index];
        // Strings borrow from the diagnostic's std::strings, which live in the
        // engine and are stable until the next mutating call (see capi.h).
        out->severity = to_c_severity(diagnostic.severity);
        out->code = to_c_status(diagnostic.code);
        out->message = diagnostic.message.c_str();
        out->path = diagnostic.path.c_str();
        out->file = diagnostic.location.file.c_str();
        out->line = diagnostic.location.line;
        out->column = diagnostic.location.column;
        out->rule_id = diagnostic.rule_id.c_str();
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_clear_diagnostics(scn_engine* engine) {
    if (engine == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        engine->engine.clear_diagnostics();
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_get_state(scn_engine* engine, const char* entity_id, scn_entity_state* out) {
    if (engine == nullptr || entity_id == nullptr || out == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        if (!engine->engine.initialized()) {
            return SCN_ERROR_NOT_INITIALIZED;
        }
        const auto state = engine->engine.state(entity_id);
        if (!state.has_value()) {
            return SCN_ERROR_UNKNOWN_ENTITY;
        }
        *out = to_c_state(*state);
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_report_state(scn_engine* engine, const char* entity_id,
                                   const scn_entity_state* state) {
    if (engine == nullptr || entity_id == nullptr || state == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        return to_c_status(engine->engine.report_state(entity_id, from_c_state(*state)));
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_entity_object_type(scn_engine* engine, const char* id, scn_object_type* out) {
    if (engine == nullptr || id == nullptr || out == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        const scena::ir::Entity* entity = find_entity(engine, id);
        if (entity == nullptr) {
            return SCN_ERROR_UNKNOWN_ENTITY;
        }
        const std::optional<scena::ir::ObjectType> type = scena::ir::object_type_of(*entity);
        if (!type.has_value()) {
            return SCN_ERROR_INVALID_ARGUMENT;
        }
        switch (*type) {
        case scena::ir::ObjectType::Vehicle:
            *out = SCN_OBJECT_VEHICLE;
            break;
        case scena::ir::ObjectType::Pedestrian:
            *out = SCN_OBJECT_PEDESTRIAN;
            break;
        case scena::ir::ObjectType::MiscObject:
            *out = SCN_OBJECT_MISC;
            break;
        }
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_entity_bounding_box(scn_engine* engine, const char* id,
                                          scn_bounding_box* out) {
    if (engine == nullptr || id == nullptr || out == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        const scena::ir::Entity* entity = find_entity(engine, id);
        if (entity == nullptr) {
            return SCN_ERROR_UNKNOWN_ENTITY;
        }
        const std::optional<scena::ir::BoundingBox> box = scena::ir::bounding_box_of(*entity);
        if (!box.has_value()) {
            return SCN_ERROR_INVALID_ARGUMENT;
        }
        *out = to_c_bounding_box(*box);
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_entity_performance(scn_engine* engine, const char* id, scn_performance* out) {
    if (engine == nullptr || id == nullptr || out == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        const scena::ir::Entity* entity = find_entity(engine, id);
        if (entity == nullptr) {
            return SCN_ERROR_UNKNOWN_ENTITY;
        }
        const scena::ir::Performance* perf = scena::ir::performance_of(*entity);
        if (perf == nullptr) {
            return SCN_ERROR_INVALID_ARGUMENT;
        }
        *out = to_c_performance(*perf);
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_set_parameter(scn_engine* engine, const char* name, const char* value) {
    if (engine == nullptr || name == nullptr || value == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        engine->scenario.parameters[name] = value;
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_declare_variable(scn_engine* engine, const char* name, const char* value) {
    if (engine == nullptr || name == nullptr || value == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        engine->scenario.variables[name] = value;
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_set_variable(scn_engine* engine, const char* name, const char* value) {
    if (engine == nullptr || name == nullptr || value == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        return to_c_status(engine->engine.set_variable(name, value));
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_get_variable(scn_engine* engine, const char* name, const char** out) {
    if (engine == nullptr || name == nullptr || out == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        const auto value = engine->engine.variable(name);
        if (!value.has_value()) {
            return SCN_ERROR_UNKNOWN_NAME; // *out left untouched
        }
        engine->value_buffer = *value;
        *out = engine->value_buffer.c_str();
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_set_user_defined_value(scn_engine* engine, const char* name,
                                             const char* value) {
    if (engine == nullptr || name == nullptr || value == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        return to_c_status(engine->engine.set_user_defined_value(name, value));
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_get_user_defined_value(scn_engine* engine, const char* name,
                                             const char** out) {
    if (engine == nullptr || name == nullptr || out == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        const auto value = engine->engine.user_defined_value(name);
        if (!value.has_value()) {
            return SCN_ERROR_UNKNOWN_NAME; // *out left untouched
        }
        engine->value_buffer = *value;
        *out = engine->value_buffer.c_str();
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

namespace {

/* Maps and range-checks the modify operator arriving across the ABI. */
bool to_ir_modify_operator(scn_modify_operator op, scena::ir::ModifyOperator& out) {
    switch (op) {
    case SCN_MODIFY_ADD:
        out = scena::ir::ModifyOperator::Add;
        return true;
    case SCN_MODIFY_MULTIPLY:
        out = scena::ir::ModifyOperator::Multiply;
        return true;
    }
    return false;
}

/* The guard every global-action builder shares: a non-null engine, a valid
 * priority and a non-negative execution count. */
bool check_event_args(const scn_engine* engine, scn_event_priority priority,
                      int maximum_execution_count, scena::ir::EventPriority& out_priority) {
    return engine != nullptr && maximum_execution_count >= 0 &&
           to_ir_priority(priority, out_priority);
}

} // namespace

scn_status scn_engine_add_variable_set_action(scn_engine* engine, const char* variable_ref,
                                              const char* value, double at_time,
                                              scn_event_priority priority,
                                              int maximum_execution_count) {
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    if (!check_event_args(engine, priority, maximum_execution_count, ir_priority) ||
        variable_ref == nullptr || value == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        append_storyboard_event(
            engine, at_time, ir_priority, maximum_execution_count,
            std::make_shared<scena::ir::VariableSetAction>(variable_ref, value));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_variable_modify_action(scn_engine* engine, const char* variable_ref,
                                                 scn_modify_operator op, double value,
                                                 double at_time, scn_event_priority priority,
                                                 int maximum_execution_count) {
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    scena::ir::ModifyOperator ir_op = scena::ir::ModifyOperator::Add;
    if (!check_event_args(engine, priority, maximum_execution_count, ir_priority) ||
        variable_ref == nullptr || !to_ir_modify_operator(op, ir_op)) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        append_storyboard_event(
            engine, at_time, ir_priority, maximum_execution_count,
            std::make_shared<scena::ir::VariableModifyAction>(variable_ref, ir_op, value));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_parameter_set_action(scn_engine* engine, const char* parameter_ref,
                                               const char* value, double at_time,
                                               scn_event_priority priority,
                                               int maximum_execution_count) {
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    if (!check_event_args(engine, priority, maximum_execution_count, ir_priority) ||
        parameter_ref == nullptr || value == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        append_storyboard_event(
            engine, at_time, ir_priority, maximum_execution_count,
            std::make_shared<scena::ir::ParameterSetAction>(parameter_ref, value));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_parameter_modify_action(scn_engine* engine, const char* parameter_ref,
                                                  scn_modify_operator op, double value,
                                                  double at_time, scn_event_priority priority,
                                                  int maximum_execution_count) {
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    scena::ir::ModifyOperator ir_op = scena::ir::ModifyOperator::Add;
    if (!check_event_args(engine, priority, maximum_execution_count, ir_priority) ||
        parameter_ref == nullptr || !to_ir_modify_operator(op, ir_op)) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        append_storyboard_event(
            engine, at_time, ir_priority, maximum_execution_count,
            std::make_shared<scena::ir::ParameterModifyAction>(parameter_ref, ir_op, value));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_add_entity_action(scn_engine* engine, const char* entity_ref, double x,
                                            double y, double z, double at_time,
                                            scn_event_priority priority,
                                            int maximum_execution_count) {
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    if (!check_event_args(engine, priority, maximum_execution_count, ir_priority) ||
        entity_ref == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        append_storyboard_event(engine, at_time, ir_priority, maximum_execution_count,
                                std::make_shared<scena::ir::AddEntityAction>(
                                    entity_ref, scena::ir::WorldPosition{x, y, z}));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_delete_entity_action(scn_engine* engine, const char* entity_ref,
                                               double at_time, scn_event_priority priority,
                                               int maximum_execution_count) {
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    if (!check_event_args(engine, priority, maximum_execution_count, ir_priority) ||
        entity_ref == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        append_storyboard_event(engine, at_time, ir_priority, maximum_execution_count,
                                std::make_shared<scena::ir::DeleteEntityAction>(entity_ref));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_environment_action(scn_engine* engine, const scn_environment* environment,
                                             double at_time, scn_event_priority priority,
                                             int maximum_execution_count) {
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    if (!check_event_args(engine, priority, maximum_execution_count, ir_priority) ||
        environment == nullptr ||
        static_cast<unsigned>(environment->precipitation_type) >
            static_cast<unsigned>(SCN_PRECIPITATION_SNOW)) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        scena::ir::Environment update;
        if (environment->name != nullptr) {
            update.name = environment->name;
        }
        if (environment->has_time_of_day != 0) {
            scena::ir::TimeOfDay time_of_day;
            time_of_day.animation = environment->time_of_day_animation != 0;
            time_of_day.date_time =
                scena::ir::DateTime{environment->year,        environment->month,
                                    environment->day,         environment->hour,
                                    environment->minute,      environment->second,
                                    environment->millisecond, environment->utc_offset_minutes};
            update.time_of_day = time_of_day;
        }
        if (environment->has_weather != 0) {
            scena::ir::Weather weather;
            if (environment->has_sun != 0) {
                weather.sun = scena::ir::Sun{environment->sun_azimuth, environment->sun_elevation,
                                             environment->sun_illuminance};
            }
            if (environment->has_fog != 0) {
                weather.fog = scena::ir::Fog{environment->fog_visual_range};
            }
            if (environment->has_precipitation != 0) {
                weather.precipitation = scena::ir::Precipitation{
                    static_cast<scena::ir::PrecipitationType>(environment->precipitation_type),
                    environment->precipitation_intensity};
            }
            if (environment->has_wind != 0) {
                weather.wind =
                    scena::ir::Wind{environment->wind_direction, environment->wind_speed};
            }
            if (environment->has_temperature != 0) {
                weather.temperature = environment->temperature;
            }
            if (environment->has_atmospheric_pressure != 0) {
                weather.atmospheric_pressure = environment->atmospheric_pressure;
            }
            if (environment->has_fractional_cloud_cover != 0) {
                weather.fractional_cloud_cover_oktas = environment->fractional_cloud_cover_oktas;
            }
            update.weather = weather;
        }
        if (environment->has_road_condition != 0) {
            update.road_condition = scena::ir::RoadCondition{environment->friction_scale_factor};
        }
        append_storyboard_event(engine, at_time, ir_priority, maximum_execution_count,
                                std::make_shared<scena::ir::EnvironmentAction>(std::move(update)));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_traffic_signal_state_action(scn_engine* engine, const char* name,
                                                      const char* state, double at_time,
                                                      scn_event_priority priority,
                                                      int maximum_execution_count) {
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    if (!check_event_args(engine, priority, maximum_execution_count, ir_priority) ||
        name == nullptr || state == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        append_storyboard_event(engine, at_time, ir_priority, maximum_execution_count,
                                std::make_shared<scena::ir::TrafficSignalStateAction>(name, state));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_traffic_signal_controller_action(
    scn_engine* engine, const char* traffic_signal_controller_ref, const char* phase,
    double at_time, scn_event_priority priority, int maximum_execution_count) {
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    if (!check_event_args(engine, priority, maximum_execution_count, ir_priority) ||
        traffic_signal_controller_ref == nullptr || phase == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        append_storyboard_event(engine, at_time, ir_priority, maximum_execution_count,
                                std::make_shared<scena::ir::TrafficSignalControllerAction>(
                                    traffic_signal_controller_ref, phase));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_custom_command_action(scn_engine* engine, const char* type,
                                                const char* content, double at_time,
                                                scn_event_priority priority,
                                                int maximum_execution_count) {
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    if (!check_event_args(engine, priority, maximum_execution_count, ir_priority) ||
        type == nullptr || content == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        append_storyboard_event(engine, at_time, ir_priority, maximum_execution_count,
                                std::make_shared<scena::ir::CustomCommandAction>(type, content));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_declare_traffic_signal_controller(scn_engine* engine, const char* name,
                                                        double delay, const char* reference,
                                                        const scn_signal_phase* phases,
                                                        size_t phase_count) {
    if (engine == nullptr || name == nullptr || (phases == nullptr && phase_count > 0)) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    for (size_t i = 0; i < phase_count; ++i) {
        if (phases[i].name == nullptr ||
            (phases[i].states == nullptr && phases[i].state_count > 0)) {
            return SCN_ERROR_INVALID_ARGUMENT;
        }
        for (size_t j = 0; j < phases[i].state_count; ++j) {
            if (phases[i].states[j].traffic_signal_id == nullptr ||
                phases[i].states[j].state == nullptr) {
                return SCN_ERROR_INVALID_ARGUMENT;
            }
        }
    }
    try {
        scena::ir::TrafficSignalController controller;
        controller.name = name;
        // A negative delay means "unspecified" on the ABI, matching the
        // scn_performance rate-limit convention.
        if (delay >= 0.0) {
            controller.delay = delay;
        }
        if (reference != nullptr) {
            controller.reference = reference;
        }
        controller.phases.reserve(phase_count);
        for (size_t i = 0; i < phase_count; ++i) {
            scena::ir::Phase phase;
            phase.name = phases[i].name;
            phase.duration = phases[i].duration;
            phase.signal_states.reserve(phases[i].state_count);
            for (size_t j = 0; j < phases[i].state_count; ++j) {
                phase.signal_states.push_back(scena::ir::TrafficSignalState{
                    phases[i].states[j].traffic_signal_id, phases[i].states[j].state});
            }
            controller.phases.push_back(std::move(phase));
        }
        engine->scenario.traffic_signal_controllers.push_back(std::move(controller));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_traffic_signal_state(scn_engine* engine, const char* name, const char** out) {
    if (engine == nullptr || name == nullptr || out == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        const auto state = engine->engine.traffic_signal_state(name);
        if (!state.has_value()) {
            return SCN_ERROR_UNKNOWN_NAME; // *out left untouched
        }
        engine->value_buffer = *state;
        *out = engine->value_buffer.c_str();
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_traffic_signal_controller_phase(scn_engine* engine, const char* name,
                                                      const char** out) {
    if (engine == nullptr || name == nullptr || out == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        const auto phase = engine->engine.traffic_signal_controller_phase(name);
        if (!phase.has_value()) {
            return SCN_ERROR_UNKNOWN_NAME; // *out left untouched
        }
        engine->value_buffer = *phase;
        *out = engine->value_buffer.c_str();
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_entity_active(scn_engine* engine, const char* id, int* out) {
    if (engine == nullptr || id == nullptr || out == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    const std::optional<bool> active = engine->engine.entity_active(id);
    if (!active.has_value()) {
        return SCN_ERROR_UNKNOWN_ENTITY; // not declared at all
    }
    *out = *active ? 1 : 0;
    return SCN_OK;
}

scn_status scn_engine_get_date_time(scn_engine* engine, double* out) {
    if (engine == nullptr || out == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    const std::optional<double> instant = engine->engine.date_time();
    if (!instant.has_value()) {
        return SCN_ERROR_INVALID_ARGUMENT; // no anchor set; *out untouched
    }
    *out = *instant;
    return SCN_OK;
}

scn_status scn_engine_set_date_time(scn_engine* engine, int year, int month, int day, int hour,
                                    int minute, int second, int millisecond,
                                    int utc_offset_minutes) {
    if (engine == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        const scena::ir::DateTime date_time{year,   month,  day,         hour,
                                            minute, second, millisecond, utc_offset_minutes};
        return to_c_status(engine->engine.set_date_time(date_time));
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}
