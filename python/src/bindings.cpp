// SPDX-License-Identifier: MIT
#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/string_view.h>
#include <nanobind/stl/vector.h>

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "scena/diagnostic.h"
#include "scena/engine.h"
#include "scena/ir/action.h"
#include "scena/ir/bounding_box.h"
#include "scena/ir/condition.h"
#include "scena/ir/date_time.h"
#include "scena/ir/entity_condition.h"
#include "scena/ir/evaluation_context.h"
#include "scena/ir/interaction_condition.h"
#include "scena/ir/position.h"
#include "scena/ir/rule.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"
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

    nb::class_<ir::Entity>(m, "Entity")
        .def(
            "__init__",
            [](ir::Entity* self, std::string id, std::string name, ir::ControlMode control_mode,
               std::optional<ir::BoundingBox> bounding_box) {
                new (self) ir::Entity{std::move(id), std::move(name), control_mode, bounding_box};
            },
            "id"_a, "name"_a, "control_mode"_a = ir::ControlMode::EngineControlled,
            "bounding_box"_a = nb::none())
        .def_rw("id", &ir::Entity::id)
        .def_rw("name", &ir::Entity::name)
        .def_rw("control_mode", &ir::Entity::control_mode)
        .def_rw("bounding_box", &ir::Entity::bounding_box)
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
            [](ir::WorldPosition* self, double x, double y, double z) {
                new (self) ir::WorldPosition{x, y, z};
            },
            "x"_a = 0.0, "y"_a = 0.0, "z"_a = 0.0)
        .def_rw("x", &ir::WorldPosition::x)
        .def_rw("y", &ir::WorldPosition::y)
        .def_rw("z", &ir::WorldPosition::z);

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

    nb::class_<ir::Action>(m, "Action")
        .def_prop_ro("kind", &ir::Action::kind,
                     "Stable ASAM element name of the action kind (e.g. 'SpeedAction').");
    nb::class_<ir::SpeedAction, ir::Action>(m, "SpeedAction")
        .def(nb::init<std::string, double>(), "entity_id"_a, "target_speed"_a)
        .def_prop_ro("entity_id", &ir::SpeedAction::entity_id)
        .def_prop_ro("target_speed", &ir::SpeedAction::target_speed);

    // Storyboard hierarchy (ASAM OpenSCENARIO XML 1.4.0 §8.3.2 nesting).
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
            [](scena::EntityState* self, double x, double y, double z, double heading,
               double speed) { new (self) scena::EntityState{x, y, z, heading, speed}; },
            "x"_a = 0.0, "y"_a = 0.0, "z"_a = 0.0, "heading"_a = 0.0, "speed"_a = 0.0)
        .def_rw("x", &scena::EntityState::x)
        .def_rw("y", &scena::EntityState::y)
        .def_rw("z", &scena::EntityState::z)
        .def_rw("heading", &scena::EntityState::heading)
        .def_rw("speed", &scena::EntityState::speed)
        .def("__repr__", [](const scena::EntityState& state) {
            return "EntityState(x=" + std::to_string(state.x) + ", y=" + std::to_string(state.y) +
                   ", z=" + std::to_string(state.z) + ", heading=" + std::to_string(state.heading) +
                   ", speed=" + std::to_string(state.speed) + ")";
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
