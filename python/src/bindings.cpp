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

#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/string_view.h>
#include <nanobind/stl/variant.h>
#include <nanobind/stl/vector.h>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "scena/diagnostic.h"
#include "scena/engine.h"
#include "scena/entity_visibility.h"
#include "scena/ir/action.h"
#include "scena/ir/bounding_box.h"
#include "scena/ir/condition.h"
#include "scena/ir/controller.h"
#include "scena/ir/date_time.h"
#include "scena/ir/entity.h"
#include "scena/ir/entity_condition.h"
#include "scena/ir/entity_types.h"
#include "scena/ir/environment.h"
#include "scena/ir/evaluation_context.h"
#include "scena/ir/interaction_condition.h"
#include "scena/ir/position.h"
#include "scena/ir/route.h"
#include "scena/ir/rule.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/traffic_signal.h"
#include "scena/ir/trajectory.h"
#include "scena/ir/trigger.h"
#include "scena/status.h"
#include "scena/version.h"

namespace nb = nanobind;
using namespace nb::literals;

namespace ir = scena::ir;

NB_MODULE(_scena, m) {
    m.doc() = "Scena: scenario execution engine for autonomous-driving simulation";

    m.def("version", &scena::version_string, "Library version as \"major.minor.patch\".");
    m.attr("__version__") = scena::version_string();

    nb::enum_<scena::Status>(m, "Status")
        .value("Ok", scena::Status::Ok)
        .value("AlreadyInitialized", scena::Status::AlreadyInitialized)
        .value("NotInitialized", scena::Status::NotInitialized)
        .value("UnknownEntity", scena::Status::UnknownEntity)
        .value("InvalidControlMode", scena::Status::InvalidControlMode)
        .value("InvalidArgument", scena::Status::InvalidArgument)
        .value("ParseError", scena::Status::ParseError)
        .value("ValidationError", scena::Status::ValidationError)
        .value("SemanticError", scena::Status::SemanticError)
        .value("UnsupportedFeature", scena::Status::UnsupportedFeature)
        .value("UnknownName", scena::Status::UnknownName)
        .value("DeprecatedFeature", scena::Status::DeprecatedFeature);

    nb::enum_<scena::Severity>(m, "Severity")
        .value("Info", scena::Severity::Info)
        .value("Warning", scena::Severity::Warning)
        .value("Error", scena::Severity::Error);

    nb::class_<scena::SourceLocation>(m, "SourceLocation")
        .def_ro("file", &scena::SourceLocation::file)
        .def_ro("line", &scena::SourceLocation::line)
        .def_ro("column", &scena::SourceLocation::column)
        .def("__repr__", [](const scena::SourceLocation& location) {
            return "SourceLocation(file='" + location.file +
                   "', line=" + std::to_string(location.line) +
                   ", column=" + std::to_string(location.column) + ")";
        });

    nb::class_<scena::Diagnostic>(m, "Diagnostic")
        .def_ro("severity", &scena::Diagnostic::severity)
        .def_ro("code", &scena::Diagnostic::code)
        .def_ro("message", &scena::Diagnostic::message)
        .def_ro("path", &scena::Diagnostic::path)
        .def_ro("location", &scena::Diagnostic::location)
        .def_ro("rule_id", &scena::Diagnostic::rule_id)
        .def("__repr__", [](const scena::Diagnostic& diagnostic) {
            return nb::str("Diagnostic(severity={}, code={}, path='{}', message='{}')")
                .format(nb::cast(diagnostic.severity), nb::cast(diagnostic.code), diagnostic.path,
                        diagnostic.message);
        });

    nb::enum_<ir::ControlMode>(m, "ControlMode")
        .value("EngineControlled", ir::ControlMode::EngineControlled)
        .value("HostControlled", ir::ControlMode::HostControlled);

    nb::enum_<scena::runtime::ElementState>(m, "ElementState")
        .value("Standby", scena::runtime::ElementState::Standby)
        .value("Running", scena::runtime::ElementState::Running)
        .value("Complete", scena::runtime::ElementState::Complete);

    nb::enum_<scena::runtime::TransitionKind>(m, "TransitionKind")
        .value("NoTransition", scena::runtime::TransitionKind::None)
        .value("Start", scena::runtime::TransitionKind::Start)
        .value("End", scena::runtime::TransitionKind::End)
        .value("Stop", scena::runtime::TransitionKind::Stop)
        .value("Skip", scena::runtime::TransitionKind::Skip);

    // Minimal entity bounding box (§ BoundingBox, p5-s3): center offset in the
    // entity frame plus dimensions. Must be bound before Entity uses it.
    nb::class_<ir::BoundingBox>(m, "BoundingBox")
        .def(
            "__init__",
            [](ir::BoundingBox* self, double center_x, double center_y, double center_z,
               double length, double width, double height) {
                new (self) ir::BoundingBox{center_x, center_y, center_z, length, width, height};
            },
            "center_x"_a = 0.0, "center_y"_a = 0.0, "center_z"_a = 0.0, "length"_a = 0.0,
            "width"_a = 0.0, "height"_a = 0.0)
        .def_rw("center_x", &ir::BoundingBox::center_x)
        .def_rw("center_y", &ir::BoundingBox::center_y)
        .def_rw("center_z", &ir::BoundingBox::center_z)
        .def_rw("length", &ir::BoundingBox::length)
        .def_rw("width", &ir::BoundingBox::width)
        .def_rw("height", &ir::BoundingBox::height);

    // Entity taxonomy (§7.2.2). The "None" role/category is exposed as "NONE"
    // because Python forbids the keyword `None` as an attribute name.
    nb::enum_<ir::ObjectType>(m, "ObjectType")
        .value("Vehicle", ir::ObjectType::Vehicle)
        .value("Pedestrian", ir::ObjectType::Pedestrian)
        .value("MiscObject", ir::ObjectType::MiscObject);

    nb::enum_<ir::VehicleCategory>(m, "VehicleCategory")
        .value("Aircraft", ir::VehicleCategory::Aircraft)
        .value("Bicycle", ir::VehicleCategory::Bicycle)
        .value("Bus", ir::VehicleCategory::Bus)
        .value("Car", ir::VehicleCategory::Car)
        .value("HeavyTruck", ir::VehicleCategory::HeavyTruck)
        .value("LandVehicle", ir::VehicleCategory::LandVehicle)
        .value("MicromobilityDevice", ir::VehicleCategory::MicromobilityDevice)
        .value("Motorbike", ir::VehicleCategory::Motorbike)
        .value("Motorcycle", ir::VehicleCategory::Motorcycle)
        .value("Other", ir::VehicleCategory::Other)
        .value("Semitractor", ir::VehicleCategory::Semitractor)
        .value("Semitrailer", ir::VehicleCategory::Semitrailer)
        .value("StandupScooter", ir::VehicleCategory::StandupScooter)
        .value("Trailer", ir::VehicleCategory::Trailer)
        .value("Train", ir::VehicleCategory::Train)
        .value("Tram", ir::VehicleCategory::Tram)
        .value("Truck", ir::VehicleCategory::Truck)
        .value("Van", ir::VehicleCategory::Van)
        .value("Watercraft", ir::VehicleCategory::Watercraft)
        .value("Wheelchair", ir::VehicleCategory::Wheelchair)
        .value("WorkMachine", ir::VehicleCategory::WorkMachine);

    nb::enum_<ir::PedestrianCategory>(m, "PedestrianCategory")
        .value("Animal", ir::PedestrianCategory::Animal)
        .value("Pedestrian", ir::PedestrianCategory::Pedestrian)
        .value("Wheelchair", ir::PedestrianCategory::Wheelchair);

    nb::enum_<ir::MiscObjectCategory>(m, "MiscObjectCategory")
        .value("Barrier", ir::MiscObjectCategory::Barrier)
        .value("Building", ir::MiscObjectCategory::Building)
        .value("Crosswalk", ir::MiscObjectCategory::Crosswalk)
        .value("Gantry", ir::MiscObjectCategory::Gantry)
        .value("NONE", ir::MiscObjectCategory::None)
        .value("Obstacle", ir::MiscObjectCategory::Obstacle)
        .value("ParkingSpace", ir::MiscObjectCategory::ParkingSpace)
        .value("Patch", ir::MiscObjectCategory::Patch)
        .value("Pole", ir::MiscObjectCategory::Pole)
        .value("Railing", ir::MiscObjectCategory::Railing)
        .value("RoadMark", ir::MiscObjectCategory::RoadMark)
        .value("SoundBarrier", ir::MiscObjectCategory::SoundBarrier)
        .value("StreetLamp", ir::MiscObjectCategory::StreetLamp)
        .value("TrafficIsland", ir::MiscObjectCategory::TrafficIsland)
        .value("Tree", ir::MiscObjectCategory::Tree)
        .value("Vegetation", ir::MiscObjectCategory::Vegetation)
        .value("Wind", ir::MiscObjectCategory::Wind);

    nb::enum_<ir::Role>(m, "Role")
        .value("NONE", ir::Role::None)
        .value("Agriculture", ir::Role::Agriculture)
        .value("Ambulance", ir::Role::Ambulance)
        .value("Civil", ir::Role::Civil)
        .value("Construction", ir::Role::Construction)
        .value("DangerousGoodsTransport", ir::Role::DangerousGoodsTransport)
        .value("Fire", ir::Role::Fire)
        .value("FireBrigade", ir::Role::FireBrigade)
        .value("FreightTransport", ir::Role::FreightTransport)
        .value("GarbageCollection", ir::Role::GarbageCollection)
        .value("Military", ir::Role::Military)
        .value("Other", ir::Role::Other)
        .value("Police", ir::Role::Police)
        .value("PublicTransport", ir::Role::PublicTransport)
        .value("RoadAssistance", ir::Role::RoadAssistance)
        .value("RoadsideAssistance", ir::Role::RoadsideAssistance)
        .value("SpecialTransport", ir::Role::SpecialTransport)
        .value("TrafficControl", ir::Role::TrafficControl);

    nb::class_<ir::Performance>(m, "Performance")
        .def(
            "__init__",
            [](ir::Performance* self, double max_speed, double max_acceleration,
               double max_deceleration, std::optional<double> max_acceleration_rate,
               std::optional<double> max_deceleration_rate) {
                new (self) ir::Performance{max_speed, max_acceleration, max_deceleration,
                                           max_acceleration_rate, max_deceleration_rate};
            },
            "max_speed"_a = 0.0, "max_acceleration"_a = 0.0, "max_deceleration"_a = 0.0,
            "max_acceleration_rate"_a = nb::none(), "max_deceleration_rate"_a = nb::none())
        .def_rw("max_speed", &ir::Performance::max_speed)
        .def_rw("max_acceleration", &ir::Performance::max_acceleration)
        .def_rw("max_deceleration", &ir::Performance::max_deceleration)
        .def_rw("max_acceleration_rate", &ir::Performance::max_acceleration_rate)
        .def_rw("max_deceleration_rate", &ir::Performance::max_deceleration_rate);

    nb::class_<ir::Axle>(m, "Axle")
        .def(
            "__init__",
            [](ir::Axle* self, double max_steering, double position_x, double position_z,
               double track_width, double wheel_diameter) {
                new (self)
                    ir::Axle{max_steering, position_x, position_z, track_width, wheel_diameter};
            },
            "max_steering"_a = 0.0, "position_x"_a = 0.0, "position_z"_a = 0.0,
            "track_width"_a = 0.0, "wheel_diameter"_a = 0.0)
        .def_rw("max_steering", &ir::Axle::max_steering)
        .def_rw("position_x", &ir::Axle::position_x)
        .def_rw("position_z", &ir::Axle::position_z)
        .def_rw("track_width", &ir::Axle::track_width)
        .def_rw("wheel_diameter", &ir::Axle::wheel_diameter);

    nb::class_<ir::Axles>(m, "Axles")
        .def(
            "__init__",
            [](ir::Axles* self, ir::Axle rear, std::optional<ir::Axle> front,
               std::vector<ir::Axle> additional) {
                new (self) ir::Axles{std::move(rear), std::move(front), std::move(additional)};
            },
            "rear"_a = ir::Axle{}, "front"_a = nb::none(), "additional"_a = std::vector<ir::Axle>{})
        .def_rw("rear", &ir::Axles::rear)
        .def_rw("front", &ir::Axles::front)
        .def_rw("additional", &ir::Axles::additional);

    nb::class_<ir::Property>(m, "Property")
        .def(
            "__init__",
            [](ir::Property* self, std::string name, std::string value) {
                new (self) ir::Property{std::move(name), std::move(value)};
            },
            "name"_a, "value"_a)
        .def_rw("name", &ir::Property::name)
        .def_rw("value", &ir::Property::value);

    nb::class_<ir::Vehicle>(m, "Vehicle")
        .def(
            "__init__",
            [](ir::Vehicle* self, ir::VehicleCategory category, ir::Role role,
               std::optional<double> mass, ir::BoundingBox bounding_box,
               ir::Performance performance, ir::Axles axles, std::vector<ir::Property> properties) {
                new (self) ir::Vehicle{category,
                                       role,
                                       mass,
                                       bounding_box,
                                       performance,
                                       std::move(axles),
                                       std::move(properties)};
            },
            "category"_a = ir::VehicleCategory::Car, "role"_a = ir::Role::None,
            "mass"_a = nb::none(), "bounding_box"_a = ir::BoundingBox{},
            "performance"_a = ir::Performance{}, "axles"_a = ir::Axles{},
            "properties"_a = std::vector<ir::Property>{})
        .def_rw("category", &ir::Vehicle::category)
        .def_rw("role", &ir::Vehicle::role)
        .def_rw("mass", &ir::Vehicle::mass)
        .def_rw("bounding_box", &ir::Vehicle::bounding_box)
        .def_rw("performance", &ir::Vehicle::performance)
        .def_rw("axles", &ir::Vehicle::axles)
        .def_rw("properties", &ir::Vehicle::properties);

    nb::class_<ir::Pedestrian>(m, "Pedestrian")
        .def(
            "__init__",
            [](ir::Pedestrian* self, ir::PedestrianCategory category, ir::Role role,
               std::optional<double> mass, ir::BoundingBox bounding_box,
               std::vector<ir::Property> properties) {
                new (self)
                    ir::Pedestrian{category, role, mass, bounding_box, std::move(properties)};
            },
            "category"_a = ir::PedestrianCategory::Pedestrian, "role"_a = ir::Role::None,
            "mass"_a = nb::none(), "bounding_box"_a = ir::BoundingBox{},
            "properties"_a = std::vector<ir::Property>{})
        .def_rw("category", &ir::Pedestrian::category)
        .def_rw("role", &ir::Pedestrian::role)
        .def_rw("mass", &ir::Pedestrian::mass)
        .def_rw("bounding_box", &ir::Pedestrian::bounding_box)
        .def_rw("properties", &ir::Pedestrian::properties);

    nb::class_<ir::MiscObject>(m, "MiscObject")
        .def(
            "__init__",
            [](ir::MiscObject* self, ir::MiscObjectCategory category, std::optional<double> mass,
               ir::BoundingBox bounding_box, std::vector<ir::Property> properties) {
                new (self) ir::MiscObject{category, mass, bounding_box, std::move(properties)};
            },
            "category"_a = ir::MiscObjectCategory::Obstacle, "mass"_a = nb::none(),
            "bounding_box"_a = ir::BoundingBox{}, "properties"_a = std::vector<ir::Property>{})
        .def_rw("category", &ir::MiscObject::category)
        .def_rw("mass", &ir::MiscObject::mass)
        .def_rw("bounding_box", &ir::MiscObject::bounding_box)
        .def_rw("properties", &ir::MiscObject::properties);

    // A ScenarioObject: identity + control mode + an optional concrete object
    // (Vehicle/Pedestrian/MiscObject). object_type/bounding_box/performance are
    // read-only views derived from the object via the ir helpers.
    nb::class_<ir::Entity>(m, "Entity")
        .def(
            "__init__",
            [](ir::Entity* self, std::string id, std::string name, ir::ControlMode control_mode,
               std::optional<ir::EntityObject> object) {
                new (self)
                    ir::Entity{std::move(id), std::move(name), control_mode, std::move(object)};
            },
            "id"_a, "name"_a, "control_mode"_a = ir::ControlMode::EngineControlled,
            "object"_a = nb::none())
        .def_rw("id", &ir::Entity::id)
        .def_rw("name", &ir::Entity::name)
        .def_rw("control_mode", &ir::Entity::control_mode)
        .def_rw("object", &ir::Entity::object)
        .def_prop_ro("object_type",
                     [](const ir::Entity& entity) { return ir::object_type_of(entity); })
        .def_prop_ro("bounding_box",
                     [](const ir::Entity& entity) { return ir::bounding_box_of(entity); })
        .def_prop_ro("performance",
                     [](const ir::Entity& entity) -> std::optional<ir::Performance> {
                         const ir::Performance* perf = ir::performance_of(entity);
                         if (perf == nullptr) {
                             return std::nullopt;
                         }
                         return *perf;
                     })
        .def("__repr__", [](const ir::Entity& entity) {
            return "Entity(id='" + entity.id + "', name='" + entity.name + "')";
        });

    // Rule comparator and by-value building blocks.
    nb::enum_<ir::Rule>(m, "Rule", "Comparison operator shared by the by-value conditions.")
        .value("EqualTo", ir::Rule::EqualTo)
        .value("GreaterThan", ir::Rule::GreaterThan)
        .value("LessThan", ir::Rule::LessThan)
        .value("GreaterOrEqual", ir::Rule::GreaterOrEqual)
        .value("LessOrEqual", ir::Rule::LessOrEqual)
        .value("NotEqualTo", ir::Rule::NotEqualTo);

    nb::enum_<ir::StoryboardElementType>(m, "StoryboardElementType")
        .value("Story", ir::StoryboardElementType::Story)
        .value("Act", ir::StoryboardElementType::Act)
        .value("ManeuverGroup", ir::StoryboardElementType::ManeuverGroup)
        .value("Maneuver", ir::StoryboardElementType::Maneuver)
        .value("Event", ir::StoryboardElementType::Event)
        .value("Action", ir::StoryboardElementType::Action);

    nb::enum_<ir::StoryboardElementState>(m, "StoryboardElementState")
        .value("StandbyState", ir::StoryboardElementState::StandbyState)
        .value("RunningState", ir::StoryboardElementState::RunningState)
        .value("CompleteState", ir::StoryboardElementState::CompleteState)
        .value("StartTransition", ir::StoryboardElementState::StartTransition)
        .value("EndTransition", ir::StoryboardElementState::EndTransition)
        .value("StopTransition", ir::StoryboardElementState::StopTransition)
        .value("SkipTransition", ir::StoryboardElementState::SkipTransition);

    nb::class_<ir::DateTime>(m, "DateTime")
        .def(
            "__init__",
            [](ir::DateTime* self, int year, int month, int day, int hour, int minute, int second,
               int millisecond, int utc_offset_minutes) {
                new (self) ir::DateTime{year,   month,  day,         hour,
                                        minute, second, millisecond, utc_offset_minutes};
            },
            "year"_a = 1970, "month"_a = 1, "day"_a = 1, "hour"_a = 0, "minute"_a = 0,
            "second"_a = 0, "millisecond"_a = 0, "utc_offset_minutes"_a = 0)
        .def_rw("year", &ir::DateTime::year)
        .def_rw("month", &ir::DateTime::month)
        .def_rw("day", &ir::DateTime::day)
        .def_rw("hour", &ir::DateTime::hour)
        .def_rw("minute", &ir::DateTime::minute)
        .def_rw("second", &ir::DateTime::second)
        .def_rw("millisecond", &ir::DateTime::millisecond)
        .def_rw("utc_offset_minutes", &ir::DateTime::utc_offset_minutes)
        .def("valid", &ir::DateTime::valid)
        .def("to_epoch_seconds", &ir::DateTime::to_epoch_seconds)
        .def("__repr__", [](const ir::DateTime& dt) {
            return nb::str("DateTime(year={}, month={}, day={}, hour={}, minute={}, second={}, "
                           "millisecond={}, utc_offset_minutes={})")
                .format(dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second, dt.millisecond,
                        dt.utc_offset_minutes);
        });

    nb::class_<ir::Condition>(m, "Condition");
    nb::class_<ir::SimulationTimeCondition, ir::Condition>(m, "SimulationTimeCondition")
        .def(
            "__init__",
            [](ir::SimulationTimeCondition* self, double at_time, ir::Rule rule) {
                new (self) ir::SimulationTimeCondition(at_time, rule);
            },
            "at_time"_a, "rule"_a = ir::Rule::GreaterOrEqual)
        .def_prop_ro("at_time", &ir::SimulationTimeCondition::at_time)
        .def_prop_ro("value", &ir::SimulationTimeCondition::value)
        .def_prop_ro("rule", &ir::SimulationTimeCondition::rule);

    nb::class_<ir::ParameterCondition, ir::Condition>(m, "ParameterCondition")
        .def(nb::init<std::string, ir::Rule, std::string>(), "parameter_ref"_a, "rule"_a, "value"_a)
        .def_prop_ro("parameter_ref", &ir::ParameterCondition::parameter_ref)
        .def_prop_ro("rule", &ir::ParameterCondition::rule)
        .def_prop_ro("value", &ir::ParameterCondition::value);

    nb::class_<ir::VariableCondition, ir::Condition>(m, "VariableCondition")
        .def(nb::init<std::string, ir::Rule, std::string>(), "variable_ref"_a, "rule"_a, "value"_a)
        .def_prop_ro("variable_ref", &ir::VariableCondition::variable_ref)
        .def_prop_ro("rule", &ir::VariableCondition::rule)
        .def_prop_ro("value", &ir::VariableCondition::value);

    nb::class_<ir::UserDefinedValueCondition, ir::Condition>(m, "UserDefinedValueCondition")
        .def(nb::init<std::string, ir::Rule, std::string>(), "name"_a, "rule"_a, "value"_a)
        .def_prop_ro("name", &ir::UserDefinedValueCondition::name)
        .def_prop_ro("rule", &ir::UserDefinedValueCondition::rule)
        .def_prop_ro("value", &ir::UserDefinedValueCondition::value);

    nb::class_<ir::TimeOfDayCondition, ir::Condition>(m, "TimeOfDayCondition")
        .def(nb::init<ir::DateTime, ir::Rule>(), "date_time"_a, "rule"_a)
        .def_prop_ro("date_time", &ir::TimeOfDayCondition::date_time)
        .def_prop_ro("rule", &ir::TimeOfDayCondition::rule);

    nb::class_<ir::StoryboardElementStateCondition, ir::Condition>(
        m, "StoryboardElementStateCondition")
        .def(nb::init<ir::StoryboardElementType, std::string, ir::StoryboardElementState>(),
             "element_type"_a, "element_ref"_a, "state"_a)
        .def_prop_ro("element_type", &ir::StoryboardElementStateCondition::element_type)
        .def_prop_ro("element_ref", &ir::StoryboardElementStateCondition::element_ref)
        .def_prop_ro("state", &ir::StoryboardElementStateCondition::state);

    // By-entity conditions (§7.6.5.1): triggering-entities frame + kinematics.
    nb::enum_<ir::TriggeringEntitiesRule>(m, "TriggeringEntitiesRule")
        .value("Any", ir::TriggeringEntitiesRule::Any)
        .value("All", ir::TriggeringEntitiesRule::All);

    nb::enum_<ir::DirectionalDimension>(m, "DirectionalDimension")
        .value("Longitudinal", ir::DirectionalDimension::Longitudinal)
        .value("Lateral", ir::DirectionalDimension::Lateral)
        .value("Vertical", ir::DirectionalDimension::Vertical);

    nb::class_<ir::TriggeringEntities>(m, "TriggeringEntities")
        .def(
            "__init__",
            [](ir::TriggeringEntities* self, std::vector<std::string> entity_refs,
               ir::TriggeringEntitiesRule rule) {
                new (self) ir::TriggeringEntities{rule, std::move(entity_refs)};
            },
            "entity_refs"_a, "rule"_a = ir::TriggeringEntitiesRule::Any)
        .def_rw("rule", &ir::TriggeringEntities::rule)
        .def_rw("entity_refs", &ir::TriggeringEntities::entity_refs);

    nb::class_<ir::WorldPosition>(m, "WorldPosition")
        .def(
            "__init__",
            [](ir::WorldPosition* self, double x, double y, double z, double h, double p,
               double r) { new (self) ir::WorldPosition{x, y, z, h, p, r}; },
            "x"_a = 0.0, "y"_a = 0.0, "z"_a = 0.0, "h"_a = 0.0, "p"_a = 0.0, "r"_a = 0.0)
        .def_rw("x", &ir::WorldPosition::x)
        .def_rw("y", &ir::WorldPosition::y)
        .def_rw("z", &ir::WorldPosition::z)
        .def_rw("h", &ir::WorldPosition::h)
        .def_rw("p", &ir::WorldPosition::p)
        .def_rw("r", &ir::WorldPosition::r);

    nb::class_<ir::SpeedCondition, ir::Condition>(m, "SpeedCondition")
        .def(nb::init<ir::TriggeringEntities, double, ir::Rule,
                      std::optional<ir::DirectionalDimension>>(),
             "triggering_entities"_a, "value"_a, "rule"_a, "direction"_a = nb::none())
        .def_prop_ro("triggering_entities", &ir::SpeedCondition::triggering_entities)
        .def_prop_ro("value", &ir::SpeedCondition::value)
        .def_prop_ro("rule", &ir::SpeedCondition::rule)
        .def_prop_ro("direction", &ir::SpeedCondition::direction);

    nb::class_<ir::RelativeSpeedCondition, ir::Condition>(m, "RelativeSpeedCondition")
        .def(nb::init<ir::TriggeringEntities, std::string, double, ir::Rule,
                      std::optional<ir::DirectionalDimension>>(),
             "triggering_entities"_a, "entity_ref"_a, "value"_a, "rule"_a,
             "direction"_a = nb::none())
        .def_prop_ro("triggering_entities", &ir::RelativeSpeedCondition::triggering_entities)
        .def_prop_ro("entity_ref", &ir::RelativeSpeedCondition::entity_ref)
        .def_prop_ro("value", &ir::RelativeSpeedCondition::value)
        .def_prop_ro("rule", &ir::RelativeSpeedCondition::rule)
        .def_prop_ro("direction", &ir::RelativeSpeedCondition::direction);

    nb::class_<ir::AccelerationCondition, ir::Condition>(m, "AccelerationCondition")
        .def(nb::init<ir::TriggeringEntities, double, ir::Rule,
                      std::optional<ir::DirectionalDimension>>(),
             "triggering_entities"_a, "value"_a, "rule"_a, "direction"_a = nb::none())
        .def_prop_ro("triggering_entities", &ir::AccelerationCondition::triggering_entities)
        .def_prop_ro("value", &ir::AccelerationCondition::value)
        .def_prop_ro("rule", &ir::AccelerationCondition::rule)
        .def_prop_ro("direction", &ir::AccelerationCondition::direction);

    nb::class_<ir::StandStillCondition, ir::Condition>(m, "StandStillCondition")
        .def(nb::init<ir::TriggeringEntities, double>(), "triggering_entities"_a, "duration"_a)
        .def_prop_ro("triggering_entities", &ir::StandStillCondition::triggering_entities)
        .def_prop_ro("duration", &ir::StandStillCondition::duration);

    nb::class_<ir::TraveledDistanceCondition, ir::Condition>(m, "TraveledDistanceCondition")
        .def(nb::init<ir::TriggeringEntities, double>(), "triggering_entities"_a, "value"_a)
        .def_prop_ro("triggering_entities", &ir::TraveledDistanceCondition::triggering_entities)
        .def_prop_ro("value", &ir::TraveledDistanceCondition::value);

    nb::class_<ir::ReachPositionCondition, ir::Condition>(m, "ReachPositionCondition")
        .def(nb::init<ir::TriggeringEntities, ir::WorldPosition, double>(), "triggering_entities"_a,
             "position"_a, "tolerance"_a)
        .def_prop_ro("triggering_entities", &ir::ReachPositionCondition::triggering_entities)
        .def_prop_ro("position", &ir::ReachPositionCondition::position)
        .def_prop_ro("tolerance", &ir::ReachPositionCondition::tolerance);

    // Interaction conditions (§7.6.5.1, §6.4): the two-entity / entity-to-
    // position metrics and their shared distance parameters.
    nb::enum_<ir::CoordinateSystem>(m, "CoordinateSystem")
        .value("Entity", ir::CoordinateSystem::Entity)
        .value("Lane", ir::CoordinateSystem::Lane)
        .value("Road", ir::CoordinateSystem::Road)
        .value("Trajectory", ir::CoordinateSystem::Trajectory)
        .value("World", ir::CoordinateSystem::World);

    nb::enum_<ir::RelativeDistanceType>(m, "RelativeDistanceType")
        .value("Longitudinal", ir::RelativeDistanceType::Longitudinal)
        .value("Lateral", ir::RelativeDistanceType::Lateral)
        .value("CartesianDistance", ir::RelativeDistanceType::CartesianDistance)
        .value("EuclidianDistance", ir::RelativeDistanceType::EuclidianDistance);

    nb::enum_<ir::RoutingAlgorithm>(m, "RoutingAlgorithm")
        .value("AssignedRoute", ir::RoutingAlgorithm::AssignedRoute)
        .value("Fastest", ir::RoutingAlgorithm::Fastest)
        .value("LeastIntersections", ir::RoutingAlgorithm::LeastIntersections)
        .value("Shortest", ir::RoutingAlgorithm::Shortest)
        .value("Undefined", ir::RoutingAlgorithm::Undefined);

    // `from` is a Python keyword, so the fields are exposed as from_lane/to_lane.
    nb::class_<ir::RelativeLaneRange>(m, "RelativeLaneRange")
        .def(
            "__init__",
            [](ir::RelativeLaneRange* self, std::optional<int> from_lane,
               std::optional<int> to_lane) {
                new (self) ir::RelativeLaneRange{from_lane, to_lane};
            },
            "from_lane"_a = nb::none(), "to_lane"_a = nb::none())
        .def_rw("from_lane", &ir::RelativeLaneRange::from)
        .def_rw("to_lane", &ir::RelativeLaneRange::to);

    nb::class_<ir::DistanceCondition, ir::Condition>(m, "DistanceCondition")
        .def(nb::init<ir::TriggeringEntities, ir::WorldPosition, double, bool, ir::Rule,
                      std::optional<ir::CoordinateSystem>, std::optional<ir::RelativeDistanceType>,
                      std::optional<ir::RoutingAlgorithm>, std::optional<bool>>(),
             "triggering_entities"_a, "position"_a, "value"_a, "freespace"_a, "rule"_a,
             "coordinate_system"_a = nb::none(), "relative_distance_type"_a = nb::none(),
             "routing_algorithm"_a = nb::none(), "along_route"_a = nb::none())
        .def_prop_ro("triggering_entities", &ir::DistanceCondition::triggering_entities)
        .def_prop_ro("position", &ir::DistanceCondition::position)
        .def_prop_ro("value", &ir::DistanceCondition::value)
        .def_prop_ro("freespace", &ir::DistanceCondition::freespace)
        .def_prop_ro("rule", &ir::DistanceCondition::rule)
        .def_prop_ro("coordinate_system", &ir::DistanceCondition::coordinate_system)
        .def_prop_ro("relative_distance_type", &ir::DistanceCondition::relative_distance_type)
        .def_prop_ro("routing_algorithm", &ir::DistanceCondition::routing_algorithm)
        .def_prop_ro("along_route", &ir::DistanceCondition::along_route);

    nb::class_<ir::RelativeDistanceCondition, ir::Condition>(m, "RelativeDistanceCondition")
        .def(nb::init<ir::TriggeringEntities, std::string, double, bool, ir::RelativeDistanceType,
                      ir::Rule, std::optional<ir::CoordinateSystem>,
                      std::optional<ir::RoutingAlgorithm>>(),
             "triggering_entities"_a, "entity_ref"_a, "value"_a, "freespace"_a,
             "relative_distance_type"_a, "rule"_a, "coordinate_system"_a = nb::none(),
             "routing_algorithm"_a = nb::none())
        .def_prop_ro("triggering_entities", &ir::RelativeDistanceCondition::triggering_entities)
        .def_prop_ro("entity_ref", &ir::RelativeDistanceCondition::entity_ref)
        .def_prop_ro("value", &ir::RelativeDistanceCondition::value)
        .def_prop_ro("freespace", &ir::RelativeDistanceCondition::freespace)
        .def_prop_ro("relative_distance_type",
                     &ir::RelativeDistanceCondition::relative_distance_type)
        .def_prop_ro("rule", &ir::RelativeDistanceCondition::rule)
        .def_prop_ro("coordinate_system", &ir::RelativeDistanceCondition::coordinate_system)
        .def_prop_ro("routing_algorithm", &ir::RelativeDistanceCondition::routing_algorithm);

    nb::class_<ir::TimeHeadwayCondition, ir::Condition>(m, "TimeHeadwayCondition")
        .def(nb::init<ir::TriggeringEntities, std::string, double, bool, ir::Rule,
                      std::optional<ir::CoordinateSystem>, std::optional<ir::RelativeDistanceType>,
                      std::optional<ir::RoutingAlgorithm>, std::optional<bool>>(),
             "triggering_entities"_a, "entity_ref"_a, "value"_a, "freespace"_a, "rule"_a,
             "coordinate_system"_a = nb::none(), "relative_distance_type"_a = nb::none(),
             "routing_algorithm"_a = nb::none(), "along_route"_a = nb::none())
        .def_prop_ro("triggering_entities", &ir::TimeHeadwayCondition::triggering_entities)
        .def_prop_ro("entity_ref", &ir::TimeHeadwayCondition::entity_ref)
        .def_prop_ro("value", &ir::TimeHeadwayCondition::value)
        .def_prop_ro("freespace", &ir::TimeHeadwayCondition::freespace)
        .def_prop_ro("rule", &ir::TimeHeadwayCondition::rule)
        .def_prop_ro("coordinate_system", &ir::TimeHeadwayCondition::coordinate_system)
        .def_prop_ro("relative_distance_type", &ir::TimeHeadwayCondition::relative_distance_type)
        .def_prop_ro("routing_algorithm", &ir::TimeHeadwayCondition::routing_algorithm)
        .def_prop_ro("along_route", &ir::TimeHeadwayCondition::along_route);

    // TimeToCollision takes a reference entity XOR a position; a custom __init__
    // enforces exactly one, and the target is read back as two Optionals.
    nb::class_<ir::TimeToCollisionCondition, ir::Condition>(m, "TimeToCollisionCondition")
        .def(
            "__init__",
            [](ir::TimeToCollisionCondition* self, ir::TriggeringEntities triggering, double value,
               bool freespace, ir::Rule rule, std::optional<std::string> entity_ref,
               std::optional<ir::WorldPosition> position,
               std::optional<ir::CoordinateSystem> coordinate_system,
               std::optional<ir::RelativeDistanceType> relative_distance_type,
               std::optional<ir::RoutingAlgorithm> routing_algorithm,
               std::optional<bool> along_route) {
                if (entity_ref.has_value() == position.has_value()) {
                    throw nb::value_error(
                        "TimeToCollisionCondition requires exactly one of entity_ref or position");
                }
                ir::TimeToCollisionTarget target =
                    entity_ref.has_value() ? ir::TimeToCollisionTarget{std::move(*entity_ref)}
                                           : ir::TimeToCollisionTarget{*position};
                new (self) ir::TimeToCollisionCondition(
                    std::move(triggering), std::move(target), value, freespace, rule,
                    coordinate_system, relative_distance_type, routing_algorithm, along_route);
            },
            "triggering_entities"_a, "value"_a, "freespace"_a, "rule"_a,
            "entity_ref"_a = nb::none(), "position"_a = nb::none(),
            "coordinate_system"_a = nb::none(), "relative_distance_type"_a = nb::none(),
            "routing_algorithm"_a = nb::none(), "along_route"_a = nb::none())
        .def_prop_ro("triggering_entities", &ir::TimeToCollisionCondition::triggering_entities)
        .def_prop_ro("entity_ref",
                     [](const ir::TimeToCollisionCondition& c) -> std::optional<std::string> {
                         if (std::holds_alternative<std::string>(c.target())) {
                             return std::get<std::string>(c.target());
                         }
                         return std::nullopt;
                     })
        .def_prop_ro("position",
                     [](const ir::TimeToCollisionCondition& c) -> std::optional<ir::WorldPosition> {
                         if (std::holds_alternative<ir::WorldPosition>(c.target())) {
                             return std::get<ir::WorldPosition>(c.target());
                         }
                         return std::nullopt;
                     })
        .def_prop_ro("value", &ir::TimeToCollisionCondition::value)
        .def_prop_ro("freespace", &ir::TimeToCollisionCondition::freespace)
        .def_prop_ro("rule", &ir::TimeToCollisionCondition::rule)
        .def_prop_ro("coordinate_system", &ir::TimeToCollisionCondition::coordinate_system)
        .def_prop_ro("relative_distance_type",
                     &ir::TimeToCollisionCondition::relative_distance_type)
        .def_prop_ro("routing_algorithm", &ir::TimeToCollisionCondition::routing_algorithm)
        .def_prop_ro("along_route", &ir::TimeToCollisionCondition::along_route);

    nb::class_<ir::CollisionCondition, ir::Condition>(m, "CollisionCondition")
        .def(nb::init<ir::TriggeringEntities, std::string>(), "triggering_entities"_a,
             "entity_ref"_a)
        .def_prop_ro("triggering_entities", &ir::CollisionCondition::triggering_entities)
        .def_prop_ro("entity_ref", &ir::CollisionCondition::entity_ref);

    nb::class_<ir::EndOfRoadCondition, ir::Condition>(m, "EndOfRoadCondition")
        .def(nb::init<ir::TriggeringEntities, double>(), "triggering_entities"_a, "duration"_a)
        .def_prop_ro("triggering_entities", &ir::EndOfRoadCondition::triggering_entities)
        .def_prop_ro("duration", &ir::EndOfRoadCondition::duration);

    nb::class_<ir::OffroadCondition, ir::Condition>(m, "OffroadCondition")
        .def(nb::init<ir::TriggeringEntities, double>(), "triggering_entities"_a, "duration"_a)
        .def_prop_ro("triggering_entities", &ir::OffroadCondition::triggering_entities)
        .def_prop_ro("duration", &ir::OffroadCondition::duration);

    nb::class_<ir::RelativeClearanceCondition, ir::Condition>(m, "RelativeClearanceCondition")
        .def(nb::init<ir::TriggeringEntities, bool, bool, double, double, std::vector<std::string>,
                      std::vector<ir::RelativeLaneRange>>(),
             "triggering_entities"_a, "free_space"_a, "opposite_lanes"_a,
             "distance_backward"_a = 0.0, "distance_forward"_a = 0.0,
             "entity_refs"_a = std::vector<std::string>{},
             "lane_ranges"_a = std::vector<ir::RelativeLaneRange>{})
        .def_prop_ro("triggering_entities", &ir::RelativeClearanceCondition::triggering_entities)
        .def_prop_ro("free_space", &ir::RelativeClearanceCondition::free_space)
        .def_prop_ro("opposite_lanes", &ir::RelativeClearanceCondition::opposite_lanes)
        .def_prop_ro("distance_backward", &ir::RelativeClearanceCondition::distance_backward)
        .def_prop_ro("distance_forward", &ir::RelativeClearanceCondition::distance_forward)
        .def_prop_ro("entity_refs", &ir::RelativeClearanceCondition::entity_refs)
        .def_prop_ro("lane_ranges", &ir::RelativeClearanceCondition::lane_ranges);

    // Trigger model (§7.6.1): a Trigger is an OR over ConditionGroups,
    // each an AND over TriggerConditions.
    nb::enum_<ir::ConditionEdge>(m, "ConditionEdge")
        .value("NoEdge", ir::ConditionEdge::None)
        .value("Rising", ir::ConditionEdge::Rising)
        .value("Falling", ir::ConditionEdge::Falling)
        .value("RisingOrFalling", ir::ConditionEdge::RisingOrFalling);

    nb::class_<ir::TriggerCondition>(m, "TriggerCondition")
        .def(
            "__init__",
            [](ir::TriggerCondition* self, std::shared_ptr<ir::Condition> expression,
               ir::ConditionEdge edge, double delay, std::string name) {
                new (self)
                    ir::TriggerCondition{std::move(name), delay, edge, std::move(expression)};
            },
            "expression"_a, "edge"_a = ir::ConditionEdge::None, "delay"_a = 0.0,
            "name"_a = std::string(),
            "A condition: a logical expression plus its edge and delay modifiers.")
        .def_rw("name", &ir::TriggerCondition::name)
        .def_rw("delay", &ir::TriggerCondition::delay)
        .def_rw("edge", &ir::TriggerCondition::edge)
        .def_rw("expression", &ir::TriggerCondition::expression);

    nb::class_<ir::ConditionGroup>(m, "ConditionGroup")
        .def(nb::init<>(), "A condition group: the AND of its conditions.")
        .def(
            "add_condition",
            [](ir::ConditionGroup& group, const ir::TriggerCondition& condition) {
                group.conditions.push_back(condition);
            },
            "condition"_a);

    nb::class_<ir::Trigger>(m, "Trigger")
        .def(nb::init<>(),
             "A trigger: the OR of its condition groups. Without groups it is always false.")
        .def(
            "add_group",
            [](ir::Trigger& trigger, const ir::ConditionGroup& group) {
                trigger.groups.push_back(group);
            },
            "group"_a);

    m.def("make_trigger", &ir::make_trigger, "expression"_a, "edge"_a = ir::ConditionEdge::None,
          "delay"_a = 0.0,
          "Wraps one logical expression into a single-group, single-condition trigger.");

    // Longitudinal transition dynamics (§TransitionDynamics / §DynamicsShape).
    nb::enum_<ir::DynamicsShape>(m, "DynamicsShape",
                                 "Shape of a speed transition (§DynamicsShape).")
        .value("Linear", ir::DynamicsShape::Linear)
        .value("Cubic", ir::DynamicsShape::Cubic)
        .value("Sinusoidal", ir::DynamicsShape::Sinusoidal)
        .value("Step", ir::DynamicsShape::Step);
    nb::enum_<ir::DynamicsDimension>(m, "DynamicsDimension",
                                     "How a target value is acquired (§DynamicsDimension).")
        .value("Time", ir::DynamicsDimension::Time)
        .value("Distance", ir::DynamicsDimension::Distance)
        .value("Rate", ir::DynamicsDimension::Rate);
    nb::enum_<ir::FollowingMode>(m, "FollowingMode",
                                 "Shape-following behavior (§FollowingMode). Scena models "
                                 "Position; Follow is accepted but treated as Position.")
        .value("Position", ir::FollowingMode::Position)
        .value("Follow", ir::FollowingMode::Follow);

    nb::class_<ir::TransitionDynamics>(m, "TransitionDynamics")
        .def(
            "__init__",
            [](ir::TransitionDynamics* self, ir::DynamicsShape shape,
               ir::DynamicsDimension dimension, double value, ir::FollowingMode following_mode) {
                new (self) ir::TransitionDynamics{shape, dimension, value, following_mode};
            },
            "shape"_a = ir::DynamicsShape::Step, "dimension"_a = ir::DynamicsDimension::Time,
            "value"_a = 0.0, "following_mode"_a = ir::FollowingMode::Position)
        .def_rw("shape", &ir::TransitionDynamics::shape)
        .def_rw("dimension", &ir::TransitionDynamics::dimension)
        .def_rw("value", &ir::TransitionDynamics::value)
        .def_rw("following_mode", &ir::TransitionDynamics::following_mode);

    nb::enum_<ir::SpeedTargetValueType>(
        m, "SpeedTargetValueType",
        "How a relative speed target combines with the reference (§SpeedTargetValueType).")
        .value("Delta", ir::SpeedTargetValueType::Delta)
        .value("Factor", ir::SpeedTargetValueType::Factor);
    nb::class_<ir::RelativeTargetSpeed>(m, "RelativeTargetSpeed")
        .def(
            "__init__",
            [](ir::RelativeTargetSpeed* self, std::string entity_ref, double value,
               ir::SpeedTargetValueType value_type, bool continuous) {
                new (self)
                    ir::RelativeTargetSpeed{std::move(entity_ref), value, value_type, continuous};
            },
            "entity_ref"_a, "value"_a, "value_type"_a = ir::SpeedTargetValueType::Delta,
            "continuous"_a = false)
        .def_rw("entity_ref", &ir::RelativeTargetSpeed::entity_ref)
        .def_rw("value", &ir::RelativeTargetSpeed::value)
        .def_rw("value_type", &ir::RelativeTargetSpeed::value_type)
        .def_rw("continuous", &ir::RelativeTargetSpeed::continuous);

    nb::class_<ir::Action>(m, "Action")
        .def_prop_ro("kind", &ir::Action::kind,
                     "Stable ASAM element name of the action kind (e.g. 'SpeedAction').");
    nb::class_<ir::GlobalAction, ir::Action>(
        m, "GlobalAction",
        "Base of the actor-less actions (§7.4.2 global, §7.4.3 user-defined). "
        "entity_id is always the empty string; branch on the type, not on that.")
        .def_prop_ro("entity_id", &ir::GlobalAction::entity_id);
    nb::class_<ir::SpeedAction, ir::Action>(m, "SpeedAction")
        .def(nb::init<std::string, double>(), "entity_id"_a, "target_speed"_a)
        .def(nb::init<std::string, double, ir::TransitionDynamics>(), "entity_id"_a,
             "target_speed"_a, "dynamics"_a)
        .def(nb::init<std::string, ir::RelativeTargetSpeed, ir::TransitionDynamics>(),
             "entity_id"_a, "target"_a, "dynamics"_a)
        .def_prop_ro("entity_id", &ir::SpeedAction::entity_id)
        .def_prop_ro("is_relative", &ir::SpeedAction::is_relative)
        .def_prop_ro("target_speed", &ir::SpeedAction::target_speed)
        .def_prop_ro("relative_target", &ir::SpeedAction::relative_target)
        .def_prop_ro("dynamics", &ir::SpeedAction::dynamics);

    nb::class_<ir::SpeedProfileEntry>(m, "SpeedProfileEntry")
        .def(
            "__init__",
            [](ir::SpeedProfileEntry* self, double speed, std::optional<double> time) {
                new (self) ir::SpeedProfileEntry{speed, time};
            },
            "speed"_a = 0.0, "time"_a = nb::none())
        .def_rw("speed", &ir::SpeedProfileEntry::speed)
        .def_rw("time", &ir::SpeedProfileEntry::time);
    nb::class_<ir::SpeedProfileAction, ir::Action>(m, "SpeedProfileAction")
        .def(nb::init<std::string, std::vector<ir::SpeedProfileEntry>, ir::FollowingMode>(),
             "entity_id"_a, "entries"_a, "following_mode"_a = ir::FollowingMode::Position)
        .def_prop_ro("entity_id", &ir::SpeedProfileAction::entity_id)
        .def_prop_ro("entries", &ir::SpeedProfileAction::entries)
        .def_prop_ro("following_mode", &ir::SpeedProfileAction::following_mode);
    nb::class_<ir::TeleportAction, ir::Action>(m, "TeleportAction")
        .def(nb::init<std::string, ir::Position>(), "entity_id"_a, "position"_a)
        .def_prop_ro("entity_id", &ir::TeleportAction::entity_id)
        .def_prop_ro("position", &ir::TeleportAction::position);

    // Routing (§6.8.2 routes, §6.9 trajectories).
    nb::enum_<ir::RouteStrategy>(m, "RouteStrategy",
                                 "How to reach a waypoint (§RouteStrategy). Stored, not "
                                 "interpreted: path selection needs a road network.")
        .value("Fastest", ir::RouteStrategy::Fastest)
        .value("LeastIntersections", ir::RouteStrategy::LeastIntersections)
        .value("Random", ir::RouteStrategy::Random)
        .value("Shortest", ir::RouteStrategy::Shortest);

    nb::class_<ir::Waypoint>(m, "Waypoint")
        .def(
            "__init__",
            [](ir::Waypoint* self, ir::WorldPosition position, ir::RouteStrategy strategy) {
                new (self) ir::Waypoint{position, strategy};
            },
            "position"_a, "strategy"_a = ir::RouteStrategy::Shortest)
        .def_rw("position", &ir::Waypoint::position)
        .def_rw("strategy", &ir::Waypoint::strategy);

    nb::class_<ir::Route>(m, "Route")
        .def(
            "__init__",
            [](ir::Route* self, std::string name, bool closed,
               std::vector<ir::Waypoint> waypoints) {
                new (self) ir::Route{std::move(name), closed, std::move(waypoints)};
            },
            "name"_a = std::string(), "closed"_a = false,
            "waypoints"_a = std::vector<ir::Waypoint>{},
            "A route through the road network; at least two waypoints (§Route).")
        .def_rw("name", &ir::Route::name)
        .def_rw("closed", &ir::Route::closed)
        .def_rw("waypoints", &ir::Route::waypoints)
        .def(
            "add_waypoint",
            [](ir::Route& route, const ir::Waypoint& waypoint) {
                route.waypoints.push_back(waypoint);
            },
            "waypoint"_a);

    nb::enum_<ir::ReferenceContext>(m, "ReferenceContext",
                                    "Whether a value is absolute (world frame / simulation-time "
                                    "origin) or relative to a reference (§ReferenceContext).")
        .value("Absolute", ir::ReferenceContext::Absolute)
        .value("Relative", ir::ReferenceContext::Relative);

    nb::class_<ir::Orientation>(m, "Orientation")
        .def(
            "__init__",
            [](ir::Orientation* self, double h, double p, double r, ir::ReferenceContext type) {
                new (self) ir::Orientation{h, p, r, type};
            },
            "h"_a = 0.0, "p"_a = 0.0, "r"_a = 0.0, "type"_a = ir::ReferenceContext::Relative)
        .def_rw("h", &ir::Orientation::h)
        .def_rw("p", &ir::Orientation::p)
        .def_rw("r", &ir::Orientation::r)
        .def_rw("type", &ir::Orientation::type);

    nb::class_<ir::RelativeWorldPosition>(m, "RelativeWorldPosition")
        .def(
            "__init__",
            [](ir::RelativeWorldPosition* self, std::string entity_ref, double dx, double dy,
               double dz, std::optional<ir::Orientation> orientation) {
                new (self) ir::RelativeWorldPosition{std::move(entity_ref), dx, dy, dz,
                                                     std::move(orientation)};
            },
            "entity_ref"_a, "dx"_a = 0.0, "dy"_a = 0.0, "dz"_a = 0.0, "orientation"_a = nb::none())
        .def_rw("entity_ref", &ir::RelativeWorldPosition::entity_ref)
        .def_rw("dx", &ir::RelativeWorldPosition::dx)
        .def_rw("dy", &ir::RelativeWorldPosition::dy)
        .def_rw("dz", &ir::RelativeWorldPosition::dz)
        .def_rw("orientation", &ir::RelativeWorldPosition::orientation);

    nb::class_<ir::RelativeObjectPosition>(m, "RelativeObjectPosition")
        .def(
            "__init__",
            [](ir::RelativeObjectPosition* self, std::string entity_ref, double dx, double dy,
               double dz, std::optional<ir::Orientation> orientation) {
                new (self) ir::RelativeObjectPosition{std::move(entity_ref), dx, dy, dz,
                                                      std::move(orientation)};
            },
            "entity_ref"_a, "dx"_a = 0.0, "dy"_a = 0.0, "dz"_a = 0.0, "orientation"_a = nb::none())
        .def_rw("entity_ref", &ir::RelativeObjectPosition::entity_ref)
        .def_rw("dx", &ir::RelativeObjectPosition::dx)
        .def_rw("dy", &ir::RelativeObjectPosition::dy)
        .def_rw("dz", &ir::RelativeObjectPosition::dz)
        .def_rw("orientation", &ir::RelativeObjectPosition::orientation);

    // GeoPosition is the one deferred (post-v0.0.1) variant exposed to Python:
    // it resolves only against a geodetic datum, so a teleport to it reports the
    // rule asam.net:xosc:1.1.0:positioning.geodetic_datum_defined. The road-,
    // lane-, route- and trajectory-relative variants land with their backends
    // (p3-s4/p2-s5).
    nb::class_<ir::GeoPosition>(m, "GeoPosition")
        .def(
            "__init__",
            [](ir::GeoPosition* self, double latitude_deg, double longitude_deg, double altitude,
               std::optional<ir::Orientation> orientation) {
                new (self)
                    ir::GeoPosition{latitude_deg, longitude_deg, altitude, std::move(orientation)};
            },
            "latitude_deg"_a = 0.0, "longitude_deg"_a = 0.0, "altitude"_a = 0.0,
            "orientation"_a = nb::none())
        .def_rw("latitude_deg", &ir::GeoPosition::latitude_deg)
        .def_rw("longitude_deg", &ir::GeoPosition::longitude_deg)
        .def_rw("altitude", &ir::GeoPosition::altitude)
        .def_rw("orientation", &ir::GeoPosition::orientation);

    nb::class_<ir::Timing>(m, "Timing")
        .def(
            "__init__",
            [](ir::Timing* self, ir::ReferenceContext domain, double scale, double offset) {
                new (self) ir::Timing{domain, scale, offset};
            },
            "domain"_a = ir::ReferenceContext::Absolute, "scale"_a = 1.0, "offset"_a = 0.0,
            "Effective vertex time = time * scale + offset, read in `domain` (§Timing).")
        .def_rw("domain", &ir::Timing::domain)
        .def_rw("scale", &ir::Timing::scale)
        .def_rw("offset", &ir::Timing::offset);

    nb::class_<ir::TrajectoryVertex>(m, "TrajectoryVertex")
        .def(
            "__init__",
            [](ir::TrajectoryVertex* self, ir::WorldPosition position, std::optional<double> time) {
                new (self) ir::TrajectoryVertex{position, time};
            },
            "position"_a, "time"_a = nb::none())
        .def_rw("position", &ir::TrajectoryVertex::position)
        .def_rw("time", &ir::TrajectoryVertex::time);

    nb::class_<ir::Trajectory>(m, "Trajectory")
        .def(
            "__init__",
            [](ir::Trajectory* self, std::string name, bool closed,
               std::vector<ir::TrajectoryVertex> vertices) {
                new (self) ir::Trajectory{std::move(name), closed, std::move(vertices)};
            },
            "name"_a = std::string(), "closed"_a = false,
            "vertices"_a = std::vector<ir::TrajectoryVertex>{},
            "A polyline path; at least two vertices (§Trajectory, §Polyline).")
        .def_rw("name", &ir::Trajectory::name)
        .def_rw("closed", &ir::Trajectory::closed)
        .def_prop_rw(
            "vertices", [](const ir::Trajectory& trajectory) { return trajectory.vertices(); },
            [](ir::Trajectory& trajectory, std::vector<ir::TrajectoryVertex> vertices) {
                trajectory.shape = ir::Polyline{std::move(vertices)};
            },
            "The polyline vertices (requires a polyline trajectory).")
        .def(
            "add_vertex",
            [](ir::Trajectory& trajectory, const ir::TrajectoryVertex& vertex) {
                trajectory.vertices().push_back(vertex);
            },
            "vertex"_a);

    // Distance keeping (§LongitudinalDistanceAction).
    nb::enum_<ir::LongitudinalDisplacement>(
        m, "LongitudinalDisplacement",
        "Which side of the reference entity a longitudinal distance applies to "
        "(§LongitudinalDisplacement).")
        .value("Any", ir::LongitudinalDisplacement::Any)
        .value("TrailingReferencedEntity", ir::LongitudinalDisplacement::TrailingReferencedEntity)
        .value("LeadingReferencedEntity", ir::LongitudinalDisplacement::LeadingReferencedEntity);

    nb::class_<ir::DynamicConstraints>(m, "DynamicConstraints")
        .def(
            "__init__",
            [](ir::DynamicConstraints* self, std::optional<double> max_acceleration,
               std::optional<double> max_acceleration_rate, std::optional<double> max_deceleration,
               std::optional<double> max_deceleration_rate, std::optional<double> max_speed) {
                new (self)
                    ir::DynamicConstraints{max_acceleration, max_acceleration_rate,
                                           max_deceleration, max_deceleration_rate, max_speed};
            },
            "max_acceleration"_a = nb::none(), "max_acceleration_rate"_a = nb::none(),
            "max_deceleration"_a = nb::none(), "max_deceleration_rate"_a = nb::none(),
            "max_speed"_a = nb::none(),
            "Limits a distance controller may use; None means unlimited (§DynamicConstraints).")
        .def_rw("max_acceleration", &ir::DynamicConstraints::max_acceleration)
        .def_rw("max_acceleration_rate", &ir::DynamicConstraints::max_acceleration_rate)
        .def_rw("max_deceleration", &ir::DynamicConstraints::max_deceleration)
        .def_rw("max_deceleration_rate", &ir::DynamicConstraints::max_deceleration_rate)
        .def_rw("max_speed", &ir::DynamicConstraints::max_speed);

    // Controllers (§Controller, §ControllerType).
    nb::enum_<ir::ControllerType>(m, "ControllerType",
                                  "The operational domains a controller acts on (§ControllerType).")
        .value("Lateral", ir::ControllerType::Lateral)
        .value("Longitudinal", ir::ControllerType::Longitudinal)
        .value("Lighting", ir::ControllerType::Lighting)
        .value("Animation", ir::ControllerType::Animation)
        .value("Movement", ir::ControllerType::Movement)
        .value("Appearance", ir::ControllerType::Appearance)
        .value("All", ir::ControllerType::All);

    nb::class_<ir::Controller>(m, "Controller")
        .def(
            "__init__",
            [](ir::Controller* self, std::string name, ir::ControllerType type,
               std::vector<ir::Property> properties) {
                new (self) ir::Controller{std::move(name), type, std::move(properties)};
            },
            "name"_a = std::string(), "type"_a = ir::ControllerType::Movement,
            "properties"_a = std::vector<ir::Property>{},
            "A controller model; its properties are a contract with the host (§Controller).")
        .def_rw("name", &ir::Controller::name)
        .def_rw("type", &ir::Controller::type)
        .def_rw("properties", &ir::Controller::properties);

    nb::class_<scena::EntityVisibility>(m, "EntityVisibility")
        .def(
            "__init__",
            [](scena::EntityVisibility* self, bool graphics, bool sensors, bool traffic) {
                new (self) scena::EntityVisibility{graphics, sensors, traffic};
            },
            "graphics"_a = true, "sensors"_a = true, "traffic"_a = true,
            "Detectability of an entity; visible everywhere by default (§VisibilityAction).")
        .def_rw("graphics", &scena::EntityVisibility::graphics)
        .def_rw("sensors", &scena::EntityVisibility::sensors)
        .def_rw("traffic", &scena::EntityVisibility::traffic)
        .def("__repr__", [](const scena::EntityVisibility& visibility) {
            return std::string("EntityVisibility(graphics=") +
                   (visibility.graphics ? "True" : "False") +
                   ", sensors=" + (visibility.sensors ? "True" : "False") +
                   ", traffic=" + (visibility.traffic ? "True" : "False") + ")";
        });

    nb::class_<scena::ControllerActivation>(m, "ControllerActivation")
        .def(
            "__init__",
            [](scena::ControllerActivation* self, bool lateral, bool longitudinal) {
                new (self) scena::ControllerActivation{lateral, longitudinal};
            },
            "lateral"_a = true, "longitudinal"_a = true,
            "Which movement domains the engine currently controls for an entity.")
        .def_rw("lateral", &scena::ControllerActivation::lateral)
        .def_rw("longitudinal", &scena::ControllerActivation::longitudinal)
        .def("__repr__", [](const scena::ControllerActivation& activation) {
            return std::string("ControllerActivation(lateral=") +
                   (activation.lateral ? "True" : "False") +
                   ", longitudinal=" + (activation.longitudinal ? "True" : "False") + ")";
        });

    // The p5-s5 private actions.
    nb::class_<ir::LongitudinalDistanceAction, ir::Action>(m, "LongitudinalDistanceAction")
        .def(nb::init<std::string, std::string, std::optional<double>, std::optional<double>, bool,
                      bool, ir::CoordinateSystem, ir::LongitudinalDisplacement,
                      std::optional<ir::DynamicConstraints>>(),
             "entity_id"_a, "entity_ref"_a, "distance"_a = nb::none(), "time_gap"_a = nb::none(),
             "freespace"_a = false, "continuous"_a = false,
             "coordinate_system"_a = ir::CoordinateSystem::Entity,
             "displacement"_a = ir::LongitudinalDisplacement::TrailingReferencedEntity,
             "constraints"_a = nb::none())
        .def_prop_ro("entity_id", &ir::LongitudinalDistanceAction::entity_id)
        .def_prop_ro("entity_ref", &ir::LongitudinalDistanceAction::entity_ref)
        .def_prop_ro("distance", &ir::LongitudinalDistanceAction::distance)
        .def_prop_ro("time_gap", &ir::LongitudinalDistanceAction::time_gap)
        .def_prop_ro("freespace", &ir::LongitudinalDistanceAction::freespace)
        .def_prop_ro("continuous", &ir::LongitudinalDistanceAction::continuous)
        .def_prop_ro("coordinate_system", &ir::LongitudinalDistanceAction::coordinate_system)
        .def_prop_ro("displacement", &ir::LongitudinalDistanceAction::displacement)
        .def_prop_ro("constraints", &ir::LongitudinalDistanceAction::constraints);

    // Lateral actions (§7.4.1.4). Positive is the reference entity's +y axis,
    // which points left (§6.3.4, ISO 8855), for all three.
    nb::enum_<ir::LateralDisplacement>(
        m, "LateralDisplacement",
        "Which side of the reference entity a lateral distance applies to "
        "(§LateralDisplacement).")
        .value("Any", ir::LateralDisplacement::Any)
        .value("LeftToReferencedEntity", ir::LateralDisplacement::LeftToReferencedEntity)
        .value("RightToReferencedEntity", ir::LateralDisplacement::RightToReferencedEntity);

    nb::class_<ir::RelativeTargetLane>(m, "RelativeTargetLane")
        .def(
            "__init__",
            [](ir::RelativeTargetLane* self, std::string entity_ref, int value) {
                new (self) ir::RelativeTargetLane{std::move(entity_ref), value};
            },
            "entity_ref"_a, "value"_a = 0,
            "A target lane given as a signed lane count from a reference entity's lane; "
            "positive is to its left (§RelativeTargetLane).")
        .def_rw("entity_ref", &ir::RelativeTargetLane::entity_ref)
        .def_rw("value", &ir::RelativeTargetLane::value);

    nb::class_<ir::AbsoluteTargetLane>(m, "AbsoluteTargetLane")
        .def(
            "__init__",
            [](ir::AbsoluteTargetLane* self, std::string value) {
                new (self) ir::AbsoluteTargetLane{std::move(value)};
            },
            "value"_a,
            "A target lane given by road-network lane id; resolving it needs a road backend "
            "(§AbsoluteTargetLane).")
        .def_rw("value", &ir::AbsoluteTargetLane::value);

    nb::class_<ir::RelativeTargetLaneOffset>(m, "RelativeTargetLaneOffset")
        .def(
            "__init__",
            [](ir::RelativeTargetLaneOffset* self, std::string entity_ref, double value) {
                new (self) ir::RelativeTargetLaneOffset{std::move(entity_ref), value};
            },
            "entity_ref"_a, "value"_a = 0.0,
            "A lane offset [m] measured from a reference entity's lane position; positive is "
            "to its left (§RelativeTargetLaneOffset).")
        .def_rw("entity_ref", &ir::RelativeTargetLaneOffset::entity_ref)
        .def_rw("value", &ir::RelativeTargetLaneOffset::value);

    nb::class_<ir::AbsoluteTargetLaneOffset>(m, "AbsoluteTargetLaneOffset")
        .def(
            "__init__",
            [](ir::AbsoluteTargetLaneOffset* self, double value) {
                new (self) ir::AbsoluteTargetLaneOffset{value};
            },
            "value"_a = 0.0,
            "A lane offset [m] from the actor's own lane centre line, positive to the left "
            "(§AbsoluteTargetLaneOffset).")
        .def_rw("value", &ir::AbsoluteTargetLaneOffset::value);

    nb::class_<ir::LaneChangeAction, ir::Action>(m, "LaneChangeAction")
        .def(nb::init<std::string, ir::RelativeTargetLane, ir::TransitionDynamics, double>(),
             "entity_id"_a, "target"_a, "dynamics"_a, "target_lane_offset"_a = 0.0)
        .def(nb::init<std::string, ir::AbsoluteTargetLane, ir::TransitionDynamics, double>(),
             "entity_id"_a, "target"_a, "dynamics"_a, "target_lane_offset"_a = 0.0)
        .def_prop_ro("entity_id", &ir::LaneChangeAction::entity_id)
        .def_prop_ro("is_relative", &ir::LaneChangeAction::is_relative)
        .def_prop_ro("relative_target", &ir::LaneChangeAction::relative_target)
        .def_prop_ro("absolute_target", &ir::LaneChangeAction::absolute_target)
        .def_prop_ro("dynamics", &ir::LaneChangeAction::dynamics)
        .def_prop_ro("target_lane_offset", &ir::LaneChangeAction::target_lane_offset);

    nb::class_<ir::LaneOffsetAction, ir::Action>(m, "LaneOffsetAction")
        .def(nb::init<std::string, ir::AbsoluteTargetLaneOffset, bool, ir::DynamicsShape,
                      std::optional<double>>(),
             "entity_id"_a, "target"_a, "continuous"_a = false, "shape"_a = ir::DynamicsShape::Step,
             "max_lateral_acc"_a = nb::none())
        .def(nb::init<std::string, ir::RelativeTargetLaneOffset, bool, ir::DynamicsShape,
                      std::optional<double>>(),
             "entity_id"_a, "target"_a, "continuous"_a = false, "shape"_a = ir::DynamicsShape::Step,
             "max_lateral_acc"_a = nb::none())
        .def_prop_ro("entity_id", &ir::LaneOffsetAction::entity_id)
        .def_prop_ro("is_relative", &ir::LaneOffsetAction::is_relative)
        .def_prop_ro("relative_target", &ir::LaneOffsetAction::relative_target)
        .def_prop_ro("absolute_target", &ir::LaneOffsetAction::absolute_target)
        .def_prop_ro("continuous", &ir::LaneOffsetAction::continuous)
        .def_prop_ro("shape", &ir::LaneOffsetAction::shape)
        .def_prop_ro("max_lateral_acc", &ir::LaneOffsetAction::max_lateral_acc);

    nb::class_<ir::LateralDistanceAction, ir::Action>(m, "LateralDistanceAction")
        .def(nb::init<std::string, std::string, double, bool, bool, ir::CoordinateSystem,
                      ir::LateralDisplacement, std::optional<ir::DynamicConstraints>>(),
             "entity_id"_a, "entity_ref"_a, "distance"_a = 0.0, "freespace"_a = false,
             "continuous"_a = false, "coordinate_system"_a = ir::CoordinateSystem::Entity,
             "displacement"_a = ir::LateralDisplacement::Any, "constraints"_a = nb::none())
        .def_prop_ro("entity_id", &ir::LateralDistanceAction::entity_id)
        .def_prop_ro("entity_ref", &ir::LateralDistanceAction::entity_ref)
        .def_prop_ro("distance", &ir::LateralDistanceAction::distance)
        .def_prop_ro("freespace", &ir::LateralDistanceAction::freespace)
        .def_prop_ro("continuous", &ir::LateralDistanceAction::continuous)
        .def_prop_ro("coordinate_system", &ir::LateralDistanceAction::coordinate_system)
        .def_prop_ro("displacement", &ir::LateralDistanceAction::displacement)
        .def_prop_ro("constraints", &ir::LateralDistanceAction::constraints);

    nb::class_<ir::AssignRouteAction, ir::Action>(m, "AssignRouteAction")
        .def(nb::init<std::string, ir::Route>(), "entity_id"_a, "route"_a)
        .def_prop_ro("entity_id", &ir::AssignRouteAction::entity_id)
        .def_prop_ro("route", &ir::AssignRouteAction::route);

    nb::class_<ir::AcquirePositionAction, ir::Action>(m, "AcquirePositionAction")
        .def(nb::init<std::string, ir::Position>(), "entity_id"_a, "position"_a)
        .def_prop_ro("entity_id", &ir::AcquirePositionAction::entity_id)
        .def_prop_ro("position", &ir::AcquirePositionAction::position);

    nb::class_<ir::FollowTrajectoryAction, ir::Action>(m, "FollowTrajectoryAction")
        .def(nb::init<std::string, ir::Trajectory, ir::FollowingMode, std::optional<ir::Timing>,
                      double>(),
             "entity_id"_a, "trajectory"_a, "following_mode"_a = ir::FollowingMode::Position,
             "time_reference"_a = nb::none(), "initial_distance_offset"_a = 0.0)
        .def_prop_ro("entity_id", &ir::FollowTrajectoryAction::entity_id)
        .def_prop_ro("trajectory", &ir::FollowTrajectoryAction::trajectory)
        .def_prop_ro("following_mode", &ir::FollowTrajectoryAction::following_mode)
        .def_prop_ro("time_reference", &ir::FollowTrajectoryAction::time_reference)
        .def_prop_ro("initial_distance_offset",
                     &ir::FollowTrajectoryAction::initial_distance_offset);

    nb::class_<ir::AssignControllerAction, ir::Action>(m, "AssignControllerAction")
        .def(nb::init<std::string, ir::Controller, std::optional<bool>, std::optional<bool>>(),
             "entity_id"_a, "controller"_a, "activate_lateral"_a = nb::none(),
             "activate_longitudinal"_a = nb::none())
        .def_prop_ro("entity_id", &ir::AssignControllerAction::entity_id)
        .def_prop_ro("controller", &ir::AssignControllerAction::controller)
        .def_prop_ro("activate_lateral", &ir::AssignControllerAction::activate_lateral)
        .def_prop_ro("activate_longitudinal", &ir::AssignControllerAction::activate_longitudinal);

    nb::class_<ir::ActivateControllerAction, ir::Action>(m, "ActivateControllerAction")
        .def(nb::init<std::string, std::optional<bool>, std::optional<bool>>(), "entity_id"_a,
             "lateral"_a = nb::none(), "longitudinal"_a = nb::none())
        .def_prop_ro("entity_id", &ir::ActivateControllerAction::entity_id)
        .def_prop_ro("lateral", &ir::ActivateControllerAction::lateral)
        .def_prop_ro("longitudinal", &ir::ActivateControllerAction::longitudinal);

    nb::class_<ir::VisibilityAction, ir::Action>(m, "VisibilityAction")
        .def(nb::init<std::string, bool, bool, bool>(), "entity_id"_a, "graphics"_a = true,
             "sensors"_a = true, "traffic"_a = true)
        .def_prop_ro("entity_id", &ir::VisibilityAction::entity_id)
        .def_prop_ro("graphics", &ir::VisibilityAction::graphics)
        .def_prop_ro("sensors", &ir::VisibilityAction::sensors)
        .def_prop_ro("traffic", &ir::VisibilityAction::traffic);

    // Storyboard hierarchy (ASAM OpenSCENARIO XML 1.4.0 §8.3.2 nesting).
    // --- Global and infrastructure actions (p5-s6, §7.4.2 / §7.4.3) --------

    nb::enum_<ir::ModifyOperator>(m, "ModifyOperator",
                                  "The arithmetic a modify action applies (§VariableModifyRule).")
        .value("Add", ir::ModifyOperator::Add)
        .value("Multiply", ir::ModifyOperator::Multiply);

    nb::class_<ir::VariableSetAction, ir::GlobalAction>(m, "VariableSetAction")
        .def(nb::init<std::string, std::string>(), "variable_ref"_a, "value"_a)
        .def_prop_ro("variable_ref", &ir::VariableSetAction::variable_ref)
        .def_prop_ro("value", &ir::VariableSetAction::value);

    nb::class_<ir::VariableModifyAction, ir::GlobalAction>(m, "VariableModifyAction")
        .def(nb::init<std::string, ir::ModifyOperator, double>(), "variable_ref"_a, "op"_a,
             "value"_a)
        .def_prop_ro("variable_ref", &ir::VariableModifyAction::variable_ref)
        .def_prop_ro("op", &ir::VariableModifyAction::op)
        .def_prop_ro("value", &ir::VariableModifyAction::value);

    nb::class_<ir::ParameterSetAction, ir::GlobalAction>(
        m, "ParameterSetAction",
        "Deprecated with 1.2 in favour of VariableSetAction; executed against a "
        "runtime overlay so a 1.0/1.1 file's ParameterCondition observes it.")
        .def(nb::init<std::string, std::string>(), "parameter_ref"_a, "value"_a)
        .def_prop_ro("parameter_ref", &ir::ParameterSetAction::parameter_ref)
        .def_prop_ro("value", &ir::ParameterSetAction::value);

    nb::class_<ir::ParameterModifyAction, ir::GlobalAction>(
        m, "ParameterModifyAction", "Deprecated with 1.2; same overlay as ParameterSetAction.")
        .def(nb::init<std::string, ir::ModifyOperator, double>(), "parameter_ref"_a, "op"_a,
             "value"_a)
        .def_prop_ro("parameter_ref", &ir::ParameterModifyAction::parameter_ref)
        .def_prop_ro("op", &ir::ParameterModifyAction::op)
        .def_prop_ro("value", &ir::ParameterModifyAction::value);

    nb::class_<ir::AddEntityAction, ir::GlobalAction>(m, "AddEntityAction")
        .def(nb::init<std::string, ir::Position>(), "entity_ref"_a, "position"_a)
        .def_prop_ro("entity_ref", &ir::AddEntityAction::entity_ref)
        .def_prop_ro("position", &ir::AddEntityAction::position);

    nb::class_<ir::DeleteEntityAction, ir::GlobalAction>(m, "DeleteEntityAction")
        .def(nb::init<std::string>(), "entity_ref"_a)
        .def_prop_ro("entity_ref", &ir::DeleteEntityAction::entity_ref);

    // Environment value types (§Environment and friends).
    nb::enum_<ir::PrecipitationType>(m, "PrecipitationType", "Types of precipitation.")
        .value("Dry", ir::PrecipitationType::Dry)
        .value("Rain", ir::PrecipitationType::Rain)
        .value("Snow", ir::PrecipitationType::Snow);

    nb::class_<ir::TimeOfDay>(m, "TimeOfDay")
        .def(
            "__init__",
            [](ir::TimeOfDay* self, bool animation, ir::DateTime date_time) {
                new (self) ir::TimeOfDay{animation, date_time};
            },
            "animation"_a = false, "date_time"_a = ir::DateTime{},
            "The simulated day and time; animation=False freezes the clock at date_time.")
        .def_rw("animation", &ir::TimeOfDay::animation)
        .def_rw("date_time", &ir::TimeOfDay::date_time);

    nb::class_<ir::Sun>(m, "Sun")
        .def(
            "__init__",
            [](ir::Sun* self, double azimuth, double elevation, double illuminance) {
                new (self) ir::Sun{azimuth, elevation, illuminance};
            },
            "azimuth"_a = 0.0, "elevation"_a = 0.0, "illuminance"_a = 0.0)
        .def_rw("azimuth", &ir::Sun::azimuth)
        .def_rw("elevation", &ir::Sun::elevation)
        .def_rw("illuminance", &ir::Sun::illuminance);

    nb::class_<ir::Fog>(m, "Fog")
        .def(
            "__init__",
            [](ir::Fog* self, double visual_range) { new (self) ir::Fog{visual_range}; },
            "visual_range"_a = 0.0)
        .def_rw("visual_range", &ir::Fog::visual_range);

    nb::class_<ir::Precipitation>(m, "Precipitation")
        .def(
            "__init__",
            [](ir::Precipitation* self, ir::PrecipitationType type, double intensity) {
                new (self) ir::Precipitation{type, intensity};
            },
            "type"_a = ir::PrecipitationType::Dry, "intensity"_a = 0.0)
        .def_rw("type", &ir::Precipitation::type)
        .def_rw("intensity", &ir::Precipitation::intensity);

    nb::class_<ir::Wind>(m, "Wind")
        .def(
            "__init__",
            [](ir::Wind* self, double direction, double speed) {
                new (self) ir::Wind{direction, speed};
            },
            "direction"_a = 0.0, "speed"_a = 0.0)
        .def_rw("direction", &ir::Wind::direction)
        .def_rw("speed", &ir::Wind::speed);

    nb::class_<ir::Weather>(m, "Weather",
                            "Weather state; every member is optional and None means "
                            "'this condition doesn't change' (§Weather).")
        .def(nb::init<>())
        .def_rw("sun", &ir::Weather::sun)
        .def_rw("fog", &ir::Weather::fog)
        .def_rw("precipitation", &ir::Weather::precipitation)
        .def_rw("wind", &ir::Weather::wind)
        .def_rw("temperature", &ir::Weather::temperature)
        .def_rw("atmospheric_pressure", &ir::Weather::atmospheric_pressure)
        .def_rw("fractional_cloud_cover_oktas", &ir::Weather::fractional_cloud_cover_oktas);

    nb::class_<ir::RoadCondition>(m, "RoadCondition")
        .def(
            "__init__",
            [](ir::RoadCondition* self, double friction_scale_factor) {
                new (self) ir::RoadCondition{friction_scale_factor};
            },
            "friction_scale_factor"_a = 1.0)
        .def_rw("friction_scale_factor", &ir::RoadCondition::friction_scale_factor);

    nb::class_<ir::Environment>(
        m, "Environment", "Environment state; an absent member does not change (§Environment).")
        .def(nb::init<>())
        .def_rw("name", &ir::Environment::name)
        .def_rw("time_of_day", &ir::Environment::time_of_day)
        .def_rw("weather", &ir::Environment::weather)
        .def_rw("road_condition", &ir::Environment::road_condition);

    nb::class_<ir::EnvironmentAction, ir::GlobalAction>(m, "EnvironmentAction")
        .def(nb::init<ir::Environment>(), "environment"_a)
        .def_prop_ro("environment", &ir::EnvironmentAction::environment);

    // Traffic signals (§6.11).
    nb::class_<ir::TrafficSignalState>(m, "TrafficSignalState")
        .def(
            "__init__",
            [](ir::TrafficSignalState* self, std::string traffic_signal_id, std::string state) {
                new (self) ir::TrafficSignalState{std::move(traffic_signal_id), std::move(state)};
            },
            "traffic_signal_id"_a, "state"_a,
            "The observable state of one signal during a phase; both strings are "
            "opaque to the engine (§6.11.4).")
        .def_rw("traffic_signal_id", &ir::TrafficSignalState::traffic_signal_id)
        .def_rw("state", &ir::TrafficSignalState::state);

    nb::class_<ir::Phase>(m, "Phase")
        .def(
            "__init__",
            [](ir::Phase* self, std::string name, double duration,
               std::vector<ir::TrafficSignalState> signal_states) {
                new (self) ir::Phase{std::move(name), duration, std::move(signal_states)};
            },
            "name"_a, "duration"_a = 0.0, "signal_states"_a = std::vector<ir::TrafficSignalState>{})
        .def_rw("name", &ir::Phase::name)
        .def_rw("duration", &ir::Phase::duration)
        .def_rw("signal_states", &ir::Phase::signal_states);

    nb::class_<ir::TrafficSignalController>(m, "TrafficSignalController")
        .def(
            "__init__",
            [](ir::TrafficSignalController* self, std::string name, std::optional<double> delay,
               std::optional<std::string> reference, std::vector<ir::Phase> phases) {
                new (self) ir::TrafficSignalController{std::move(name), delay, std::move(reference),
                                                       std::move(phases)};
            },
            "name"_a, "delay"_a = nb::none(), "reference"_a = nb::none(),
            "phases"_a = std::vector<ir::Phase>{},
            "A signal cycle (§6.11.2); delay requires reference (§6.11.3).")
        .def_rw("name", &ir::TrafficSignalController::name)
        .def_rw("delay", &ir::TrafficSignalController::delay)
        .def_rw("reference", &ir::TrafficSignalController::reference)
        .def_rw("phases", &ir::TrafficSignalController::phases);

    nb::class_<ir::TrafficSignalStateAction, ir::GlobalAction>(m, "TrafficSignalStateAction")
        .def(nb::init<std::string, std::string>(), "name"_a, "state"_a)
        .def_prop_ro("name", &ir::TrafficSignalStateAction::name)
        .def_prop_ro("state", &ir::TrafficSignalStateAction::state);

    nb::class_<ir::TrafficSignalControllerAction, ir::GlobalAction>(m,
                                                                    "TrafficSignalControllerAction")
        .def(nb::init<std::string, std::string>(), "traffic_signal_controller_ref"_a, "phase"_a)
        .def_prop_ro("traffic_signal_controller_ref",
                     &ir::TrafficSignalControllerAction::traffic_signal_controller_ref)
        .def_prop_ro("phase", &ir::TrafficSignalControllerAction::phase);

    nb::class_<ir::TrafficSignalCondition, ir::Condition>(
        m, "TrafficSignalCondition",
        "True while the signal is in `state`; combine with ConditionEdge.Rising "
        "for the standard's 'reaches' wording.")
        .def(nb::init<std::string, std::string>(), "name"_a, "state"_a)
        .def_prop_ro("name", &ir::TrafficSignalCondition::name)
        .def_prop_ro("state", &ir::TrafficSignalCondition::state);

    nb::class_<ir::TrafficSignalControllerCondition, ir::Condition>(
        m, "TrafficSignalControllerCondition",
        "True while the controller is in `phase`; false before a delayed start.")
        .def(nb::init<std::string, std::string>(), "traffic_signal_controller_ref"_a, "phase"_a)
        .def_prop_ro("traffic_signal_controller_ref",
                     &ir::TrafficSignalControllerCondition::traffic_signal_controller_ref)
        .def_prop_ro("phase", &ir::TrafficSignalControllerCondition::phase);

    nb::class_<ir::CustomCommandAction, ir::GlobalAction>(
        m, "CustomCommandAction",
        "A host-author contract handed to the gateway verbatim (§7.4.3); a "
        "silent no-op without a gateway.")
        .def(nb::init<std::string, std::string>(), "type"_a, "content"_a)
        .def_prop_ro("type", &ir::CustomCommandAction::type)
        .def_prop_ro("content", &ir::CustomCommandAction::content);

    nb::enum_<ir::EventPriority>(m, "EventPriority",
                                 "How a starting event interacts with the other events of its "
                                 "Maneuver (§7.3.2, §8.4.2.2). Distinct from TransitionKind.Skip, "
                                 "which is the transition a skipped start performs.")
        .value("Override", ir::EventPriority::Override)
        .value("Parallel", ir::EventPriority::Parallel)
        .value("Skip", ir::EventPriority::Skip);

    nb::class_<ir::Event>(m, "Event")
        .def(
            "__init__",
            [](ir::Event* self, std::string name, std::optional<ir::Trigger> start_trigger,
               ir::EventPriority priority, int maximum_execution_count) {
                new (self) ir::Event{.name = std::move(name),
                                     .start_trigger = std::move(start_trigger),
                                     .priority = priority,
                                     .maximum_execution_count = maximum_execution_count,
                                     .actions = {}};
            },
            "name"_a, "start_trigger"_a = nb::none(), "priority"_a = ir::EventPriority::Parallel,
            "maximum_execution_count"_a = 1,
            "An event; without a start trigger it starts with its parent.")
        .def_rw("name", &ir::Event::name)
        .def_rw("start_trigger", &ir::Event::start_trigger)
        .def_rw("priority", &ir::Event::priority)
        .def_rw("maximum_execution_count", &ir::Event::maximum_execution_count)
        .def(
            "add_action",
            [](ir::Event& event, std::shared_ptr<ir::Action> action) {
                event.actions.push_back(std::move(action));
            },
            "action"_a);

    nb::class_<ir::Maneuver>(m, "Maneuver")
        .def(
            "__init__",
            [](ir::Maneuver* self, std::string name) {
                new (self) ir::Maneuver{.name = std::move(name), .events = {}};
            },
            "name"_a)
        .def_rw("name", &ir::Maneuver::name)
        .def(
            "add_event",
            [](ir::Maneuver& maneuver, const ir::Event& event) {
                maneuver.events.push_back(event);
            },
            "event"_a);

    nb::class_<ir::ManeuverGroup>(m, "ManeuverGroup")
        .def(
            "__init__",
            [](ir::ManeuverGroup* self, std::string name) {
                new (self)
                    ir::ManeuverGroup{.name = std::move(name), .actors = {}, .maneuvers = {}};
            },
            "name"_a)
        .def_rw("name", &ir::ManeuverGroup::name)
        .def(
            "add_actor",
            [](ir::ManeuverGroup& group, std::string entity_id) {
                group.actors.push_back(std::move(entity_id));
            },
            "entity_id"_a)
        .def(
            "add_maneuver",
            [](ir::ManeuverGroup& group, const ir::Maneuver& maneuver) {
                group.maneuvers.push_back(maneuver);
            },
            "maneuver"_a);

    nb::class_<ir::Act>(m, "Act")
        .def(
            "__init__",
            [](ir::Act* self, std::string name, std::optional<ir::Trigger> start_trigger,
               std::optional<ir::Trigger> stop_trigger) {
                new (self) ir::Act{.name = std::move(name),
                                   .start_trigger = std::move(start_trigger),
                                   .stop_trigger = std::move(stop_trigger),
                                   .groups = {}};
            },
            "name"_a, "start_trigger"_a = nb::none(), "stop_trigger"_a = nb::none(),
            "An act; without a start trigger it starts with the storyboard.")
        .def_rw("name", &ir::Act::name)
        .def_rw("start_trigger", &ir::Act::start_trigger)
        .def_rw("stop_trigger", &ir::Act::stop_trigger)
        .def(
            "set_stop_trigger",
            [](ir::Act& act, ir::Trigger trigger) { act.stop_trigger = std::move(trigger); },
            "trigger"_a,
            "Sets the act stop trigger; when it fires the act and its whole subtree complete.")
        .def(
            "add_group",
            [](ir::Act& act, const ir::ManeuverGroup& group) { act.groups.push_back(group); },
            "group"_a);

    nb::class_<ir::Story>(m, "Story")
        .def(
            "__init__",
            [](ir::Story* self, std::string name) {
                new (self) ir::Story{.name = std::move(name), .acts = {}};
            },
            "name"_a)
        .def_rw("name", &ir::Story::name)
        .def(
            "add_act", [](ir::Story& story, const ir::Act& act) { story.acts.push_back(act); },
            "act"_a);

    nb::class_<ir::Scenario>(m, "Scenario")
        .def(
            "__init__",
            [](ir::Scenario* self, std::string name) {
                new (self) ir::Scenario();
                self->name = std::move(name);
            },
            "name"_a = std::string())
        .def_rw("name", &ir::Scenario::name)
        .def(
            "set_parameter",
            [](ir::Scenario& scenario, std::string name, std::string value) {
                scenario.parameters[std::move(name)] = std::move(value);
            },
            "name"_a, "value"_a,
            "Declares a global parameter (immutable at runtime); effective at the next init().")
        .def(
            "declare_variable",
            [](ir::Scenario& scenario, std::string name, std::string value) {
                scenario.variables[std::move(name)] = std::move(value);
            },
            "name"_a, "value"_a,
            "Declares a global variable with its initialization value; effective at the next "
            "init().")
        .def(
            "add_entity",
            [](ir::Scenario& scenario, const ir::Entity& entity) {
                scenario.entities.push_back(entity);
            },
            "entity"_a)
        .def(
            "add_init_action",
            [](ir::Scenario& scenario, std::shared_ptr<ir::Action> action) {
                scenario.init_actions.push_back(std::move(action));
            },
            "action"_a, "Appends an init-phase action, applied before simulation time starts.")
        .def(
            "add_traffic_signal_controller",
            [](ir::Scenario& scenario, const ir::TrafficSignalController& controller) {
                scenario.traffic_signal_controllers.push_back(controller);
            },
            "controller"_a,
            "Declares a TrafficSignalController (§6.11.2); document order is the "
            "order the engine ticks them in.")
        .def(
            "add_story",
            [](ir::Scenario& scenario, const ir::Story& story) {
                scenario.storyboard.stories.push_back(story);
            },
            "story"_a)
        .def(
            "set_stop_trigger",
            [](ir::Scenario& scenario, ir::Trigger trigger) {
                scenario.storyboard.stop_trigger = std::move(trigger);
            },
            "trigger"_a,
            "Sets the storyboard stop trigger; without one the storyboard never completes.");

    nb::class_<scena::EntityState>(m, "EntityState")
        .def(
            "__init__",
            [](scena::EntityState* self, double x, double y, double z, double heading, double speed,
               double pitch, double roll) {
                new (self) scena::EntityState{x, y, z, heading, speed, pitch, roll};
            },
            "x"_a = 0.0, "y"_a = 0.0, "z"_a = 0.0, "heading"_a = 0.0, "speed"_a = 0.0,
            "pitch"_a = 0.0, "roll"_a = 0.0)
        .def_rw("x", &scena::EntityState::x)
        .def_rw("y", &scena::EntityState::y)
        .def_rw("z", &scena::EntityState::z)
        .def_rw("heading", &scena::EntityState::heading)
        .def_rw("speed", &scena::EntityState::speed)
        .def_rw("pitch", &scena::EntityState::pitch)
        .def_rw("roll", &scena::EntityState::roll)
        .def("__repr__", [](const scena::EntityState& state) {
            return "EntityState(x=" + std::to_string(state.x) + ", y=" + std::to_string(state.y) +
                   ", z=" + std::to_string(state.z) + ", heading=" + std::to_string(state.heading) +
                   ", speed=" + std::to_string(state.speed) +
                   ", pitch=" + std::to_string(state.pitch) +
                   ", roll=" + std::to_string(state.roll) + ")";
        });

    nb::class_<scena::Engine>(m, "Engine")
        .def(nb::init<>())
        .def("init", &scena::Engine::init, "scenario"_a,
             "Loads a scenario, applies init actions, and starts the storyboard at t = 0.")
        .def("step", &scena::Engine::step, "dt"_a, "Advances simulation time by dt seconds.")
        .def("state", &scena::Engine::state, "entity_id"_a,
             "Current state of an entity, or None when unknown.")
        .def("report_state", &scena::Engine::report_state, "entity_id"_a, "state"_a,
             "Reports the authoritative state of a host-controlled entity.")
        .def(
            "route_of",
            [](const scena::Engine& engine,
               const std::string& entity_id) -> std::optional<ir::Route> {
                // Returns a copy: the engine's route is replaced by the next
                // routing action, and a Python reference must not dangle.
                const ir::Route* route = engine.route_of(entity_id);
                if (route == nullptr) {
                    return std::nullopt;
                }
                return *route;
            },
            "entity_id"_a,
            "The route assigned to an entity (a copy), or None when it has none (§6.8.2).")
        .def(
            "assigned_controller_of",
            [](const scena::Engine& engine,
               const std::string& entity_id) -> std::optional<ir::Controller> {
                const ir::Controller* controller = engine.assigned_controller_of(entity_id);
                if (controller == nullptr) {
                    return std::nullopt;
                }
                return *controller;
            },
            "entity_id"_a,
            "The controller assigned to an entity (a copy), or None when it has none.")
        .def("entity_active", &scena::Engine::entity_active, "entity_id"_a,
             "Whether a declared entity is currently in the scenario (§EntityAction); "
             "None when the id is not declared at all.")
        .def("traffic_signal_state", &scena::Engine::traffic_signal_state, "name"_a,
             "Current observable state of a traffic signal, or None when nothing has "
             "written it yet.")
        .def("traffic_signal_controller_phase", &scena::Engine::traffic_signal_controller_phase,
             "name"_a,
             "Name of the phase a controller is in, or None when it is unknown, has no "
             "phases, or has not started yet.")
        .def_prop_ro(
            "environment",
            [](const scena::Engine& engine) {
                // A copy: the store is replaced by the next EnvironmentAction
                // and by init/close, and a Python reference must not dangle.
                return engine.environment();
            },
            "Environment state merged from the EnvironmentActions so far (a copy).")
        .def("controller_activation_of", &scena::Engine::controller_activation_of, "entity_id"_a,
             "Which movement domains the engine controls for an entity, or None when unknown.")
        .def("visibility_of", &scena::Engine::visibility_of, "entity_id"_a,
             "Current detectability of an entity (§VisibilityAction), or None when unknown.")
        .def("set_variable", &scena::Engine::set_variable, "name"_a, "value"_a,
             "Sets a declared variable; UnknownName for an undeclared name.")
        .def("variable", &scena::Engine::variable, "name"_a,
             "Current value of a variable, or None when undeclared or before init.")
        .def("set_user_defined_value", &scena::Engine::set_user_defined_value, "name"_a, "value"_a,
             "Creates or updates an external user-defined value.")
        .def("user_defined_value", &scena::Engine::user_defined_value, "name"_a,
             "Current value of a user-defined value, or None when unset.")
        .def("set_date_time", &scena::Engine::set_date_time, "date_time"_a,
             "Anchors the simulated time of day; InvalidArgument for an out-of-range DateTime.")
        .def_prop_ro("date_time", &scena::Engine::date_time,
                     "Simulated instant as epoch seconds, or None when no anchor is set.")
        .def("set_default_lane_width", &scena::Engine::set_default_lane_width, "width"_a,
             "Sets the lane width a LaneChangeAction assumes without a road network [m]; "
             "InvalidArgument for a non-finite or non-positive width. A method returning a "
             "Status rather than a property setter, matching every other Engine setter.")
        .def_prop_ro("default_lane_width", &scena::Engine::default_lane_width,
                     "The lane width the flat-world lane model currently uses [m].")
        .def("storyboard_element_state", &scena::Engine::storyboard_element_state, "path"_a,
             "Lifecycle state of the element at 'story/act/group/maneuver/event'; '' is the "
             "storyboard itself; None for unknown paths.")
        .def("storyboard_element_transition", &scena::Engine::storyboard_element_transition,
             "path"_a, "Last monitorable transition of the element at the given path.")
        .def("close", &scena::Engine::close,
             "Releases the scenario; the engine can be re-initialized afterwards.")
        .def(
            "diagnostics",
            [](const scena::Engine& engine) {
                // Returns a list of copies: safe across close / re-init and
                // independent of the engine's borrowed backing store.
                return engine.diagnostics();
            },
            "Structured findings from the current scenario, in report order (a copy).")
        .def("clear_diagnostics", &scena::Engine::clear_diagnostics,
             "Drops every collected diagnostic.")
        .def_prop_ro("time", &scena::Engine::time, "Simulation time in seconds since init().")
        .def_prop_ro("initialized", &scena::Engine::initialized);
}
