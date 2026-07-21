// SPDX-License-Identifier: MIT
#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include <memory>
#include <string>
#include <utility>

#include "kinema/engine.h"
#include "kinema/ir/action.h"
#include "kinema/ir/condition.h"
#include "kinema/ir/scenario.h"
#include "kinema/version.h"

namespace nb = nanobind;
using namespace nb::literals;

namespace ir = kinema::ir;

NB_MODULE(_kinema, m) {
    m.doc() = "Kinema: scenario execution engine for autonomous-driving simulation";

    m.def("version", &kinema::version_string, "Library version as \"major.minor.patch\".");
    m.attr("__version__") = kinema::version_string();

    nb::enum_<kinema::Status>(m, "Status")
        .value("Ok", kinema::Status::Ok)
        .value("AlreadyInitialized", kinema::Status::AlreadyInitialized)
        .value("NotInitialized", kinema::Status::NotInitialized)
        .value("UnknownEntity", kinema::Status::UnknownEntity)
        .value("InvalidControlMode", kinema::Status::InvalidControlMode)
        .value("InvalidArgument", kinema::Status::InvalidArgument);

    nb::enum_<ir::ControlMode>(m, "ControlMode")
        .value("EngineControlled", ir::ControlMode::EngineControlled)
        .value("HostControlled", ir::ControlMode::HostControlled);

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

    nb::class_<ir::Action>(m, "Action");
    nb::class_<ir::SpeedAction, ir::Action>(m, "SpeedAction")
        .def(nb::init<std::string, double>(), "entity_id"_a, "target_speed"_a)
        .def_prop_ro("entity_id", &ir::SpeedAction::entity_id)
        .def_prop_ro("target_speed", &ir::SpeedAction::target_speed);

    nb::class_<ir::Scenario>(m, "Scenario")
        .def(
            "__init__",
            [](ir::Scenario* self, std::string name) {
                new (self) ir::Scenario{std::move(name), {}, {}};
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
            "add_entry",
            [](ir::Scenario& scenario, std::shared_ptr<ir::Condition> condition,
               std::shared_ptr<ir::Action> action) {
                scenario.storyboard.entries.push_back({std::move(condition), std::move(action)});
            },
            "condition"_a, "action"_a, "Appends a (condition, action) pair to the storyboard.");

    nb::class_<kinema::EntityState>(m, "EntityState")
        .def(
            "__init__",
            [](kinema::EntityState* self, double x, double y, double z, double heading,
               double speed) { new (self) kinema::EntityState{x, y, z, heading, speed}; },
            "x"_a = 0.0, "y"_a = 0.0, "z"_a = 0.0, "heading"_a = 0.0, "speed"_a = 0.0)
        .def_rw("x", &kinema::EntityState::x)
        .def_rw("y", &kinema::EntityState::y)
        .def_rw("z", &kinema::EntityState::z)
        .def_rw("heading", &kinema::EntityState::heading)
        .def_rw("speed", &kinema::EntityState::speed)
        .def("__repr__", [](const kinema::EntityState& state) {
            return "EntityState(x=" + std::to_string(state.x) + ", y=" + std::to_string(state.y) +
                   ", z=" + std::to_string(state.z) + ", heading=" + std::to_string(state.heading) +
                   ", speed=" + std::to_string(state.speed) + ")";
        });

    nb::class_<kinema::Engine>(m, "Engine")
        .def(nb::init<>())
        .def("init", &kinema::Engine::init, "scenario"_a,
             "Loads a scenario and resets simulation time to zero.")
        .def("step", &kinema::Engine::step, "dt"_a, "Advances simulation time by dt seconds.")
        .def("state", &kinema::Engine::state, "entity_id"_a,
             "Current state of an entity, or None when unknown.")
        .def("report_state", &kinema::Engine::report_state, "entity_id"_a, "state"_a,
             "Reports the authoritative state of a host-controlled entity.")
        .def("close", &kinema::Engine::close,
             "Releases the scenario; the engine can be re-initialized afterwards.")
        .def_prop_ro("time", &kinema::Engine::time, "Simulation time in seconds since init().")
        .def_prop_ro("initialized", &kinema::Engine::initialized);
}
