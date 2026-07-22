// SPDX-License-Identifier: MIT
#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/string_view.h>
#include <nanobind/stl/vector.h>

#include <memory>
#include <string>
#include <utility>

#include "scena/diagnostic.h"
#include "scena/engine.h"
#include "scena/ir/action.h"
#include "scena/ir/condition.h"
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
        .value("UnknownName", scena::Status::UnknownName);

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

    nb::class_<ir::Entity>(m, "Entity")
        .def(
            "__init__",
            [](ir::Entity* self, std::string id, std::string name, ir::ControlMode control_mode) {
                new (self) ir::Entity{std::move(id), std::move(name), control_mode};
            },
            "id"_a, "name"_a, "control_mode"_a = ir::ControlMode::EngineControlled)
        .def_rw("id", &ir::Entity::id)
        .def_rw("name", &ir::Entity::name)
        .def_rw("control_mode", &ir::Entity::control_mode)
        .def("__repr__", [](const ir::Entity& entity) {
            return "Entity(id='" + entity.id + "', name='" + entity.name + "')";
        });

    nb::class_<ir::Condition>(m, "Condition");
    nb::class_<ir::SimulationTimeCondition, ir::Condition>(m, "SimulationTimeCondition")
        .def(nb::init<double>(), "at_time"_a)
        .def_prop_ro("at_time", &ir::SimulationTimeCondition::at_time);

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
                new (self) ir::Scenario{std::move(name), {}, {}, {}};
            },
            "name"_a = std::string())
        .def_rw("name", &ir::Scenario::name)
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
