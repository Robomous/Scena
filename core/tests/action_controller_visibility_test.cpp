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

//
// The controller and visibility actions at the engine level (p5-s5):
// AssignControllerAction, ActivateControllerAction and VisibilityAction all
// complete immediately (Annex A Table 10), reach the gateway at a fixed point
// in the step, and — for the activation flags — decide whether the engine still
// drives a movement domain (ADR-0014).

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "scena/diagnostic.h"
#include "scena/engine.h"
#include "scena/entity_visibility.h"
#include "scena/gateway/simulator_gateway.h"
#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/controller.h"
#include "scena/ir/dynamics.h"
#include "scena/ir/entity.h"
#include "scena/ir/entity_types.h"
#include "scena/ir/position.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/trajectory.h"
#include "scena/ir/trigger.h"

namespace {

using scena::ControllerActivation;
using scena::Engine;
using scena::EntityState;
using scena::EntityVisibility;
using scena::Severity;
using scena::Status;
using scena::ir::ActivateControllerAction;
using scena::ir::AssignControllerAction;
using scena::ir::Controller;
using scena::ir::ControllerType;
using scena::ir::ControlMode;
using scena::ir::DynamicsDimension;
using scena::ir::DynamicsShape;
using scena::ir::Entity;
using scena::ir::FollowTrajectoryAction;
using scena::ir::Property;
using scena::ir::SimulationTimeCondition;
using scena::ir::SpeedAction;
using scena::ir::Trajectory;
using scena::ir::TrajectoryVertex;
using scena::ir::TransitionDynamics;
using scena::ir::VisibilityAction;
using scena::ir::WorldPosition;

constexpr double kTol = 1e-9;

/// A gateway that records every hand-off in order. Hand-written rather than a
/// gmock: the interface is four methods and the recording reads better.
class RecordingGateway final : public scena::gateway::ISimulatorGateway {
public:
    struct ControllerCall {
        std::string entity_id;
        Controller controller;
    };
    struct VisibilityCall {
        std::string entity_id;
        EntityVisibility visibility;
    };

    void publish_state(const std::string& entity_id, const EntityState& state) override {
        (void)entity_id;
        (void)state;
    }

    bool poll_state(const std::string& entity_id, EntityState& out) override {
        (void)entity_id;
        (void)out;
        return false;
    }

    scena::gateway::IRoadQuery* road_query() override { return nullptr; }

    void on_controller_assigned(const std::string& entity_id,
                                const Controller& controller) override {
        controllers.push_back(ControllerCall{entity_id, controller});
    }

    void on_visibility_changed(const std::string& entity_id,
                               const EntityVisibility& visibility) override {
        visibilities.push_back(VisibilityCall{entity_id, visibility});
    }

    std::vector<ControllerCall> controllers;
    std::vector<VisibilityCall> visibilities;
};

/// A gateway that overrides nothing beyond the pure virtuals — the proof that
/// the two p5-s5 hand-offs are defaulted and a pre-existing host still
/// compiles (the ADR-0003 amendment in ADR-0014).
class LegacyGateway final : public scena::gateway::ISimulatorGateway {
public:
    void publish_state(const std::string& entity_id, const EntityState& state) override {
        (void)entity_id;
        published = state;
    }
    bool poll_state(const std::string& entity_id, EntityState& out) override {
        (void)entity_id;
        (void)out;
        return false;
    }
    scena::gateway::IRoadQuery* road_query() override { return nullptr; }

    EntityState published;
};

scena::ir::Event timed_event(const std::string& name, double at_time,
                             std::shared_ptr<scena::ir::Action> action) {
    scena::ir::Event event;
    event.name = name;
    event.start_trigger = scena::ir::make_trigger(
        std::make_shared<SimulationTimeCondition>(at_time), scena::ir::ConditionEdge::None, 0.0);
    event.actions.push_back(std::move(action));
    return event;
}

scena::ir::Scenario make_scenario(double speed, std::vector<scena::ir::Event> events) {
    scena::ir::Scenario scenario;
    scenario.name = "controller-visibility";
    Entity ego;
    ego.id = "ego";
    ego.name = "ego";
    ego.control_mode = ControlMode::EngineControlled;
    scenario.entities.push_back(std::move(ego));
    scenario.init_actions.push_back(std::make_shared<SpeedAction>("ego", speed));

    scena::ir::Maneuver maneuver;
    maneuver.name = "maneuver";
    maneuver.events = std::move(events);
    scena::ir::ManeuverGroup group;
    group.name = "group";
    group.maneuvers.push_back(std::move(maneuver));
    scena::ir::Act act;
    act.name = "act";
    act.groups.push_back(std::move(group));
    scena::ir::Story story;
    story.name = "story";
    story.acts.push_back(std::move(act));
    scenario.storyboard.stories.push_back(std::move(story));
    return scenario;
}

std::string path_of(const std::string& event_name) {
    return "story/act/group/maneuver/" + event_name;
}

Controller make_controller(ControllerType type = ControllerType::Movement) {
    Controller controller;
    controller.name = "driver";
    controller.type = type;
    controller.properties.push_back(Property{"model", "idm"});
    controller.properties.push_back(Property{"aggressiveness", "0.7"});
    return controller;
}

// --- AssignControllerAction ------------------------------------------------

TEST(ControllerActionTest, AssignmentReachesTheGatewayWithItsPropertiesInOrder) {
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event(
        "assign", 1.0, std::make_shared<AssignControllerAction>("ego", make_controller())));
    RecordingGateway gateway;
    Engine engine(&gateway);
    ASSERT_EQ(engine.init(make_scenario(10.0, std::move(events))), Status::Ok);
    EXPECT_TRUE(gateway.controllers.empty());
    EXPECT_EQ(engine.assigned_controller_of("ego"), nullptr);

    ASSERT_EQ(engine.step(1.0), Status::Ok);
    // Table 10: "immediately (does not consume simulation time)".
    EXPECT_EQ(*engine.storyboard_element_state(path_of("assign")),
              scena::runtime::ElementState::Complete);
    ASSERT_EQ(gateway.controllers.size(), 1U);
    EXPECT_EQ(gateway.controllers[0].entity_id, "ego");
    EXPECT_EQ(gateway.controllers[0].controller.name, "driver");
    EXPECT_EQ(gateway.controllers[0].controller.type, ControllerType::Movement);
    ASSERT_EQ(gateway.controllers[0].controller.properties.size(), 2U);
    EXPECT_EQ(gateway.controllers[0].controller.properties[0].name, "model");
    EXPECT_EQ(gateway.controllers[0].controller.properties[1].name, "aggressiveness");

    const Controller* stored = engine.assigned_controller_of("ego");
    ASSERT_NE(stored, nullptr);
    EXPECT_EQ(stored->name, "driver");
}

TEST(ControllerActionTest, AssignmentActivationFlagsApplyAndDefaultToNoChange) {
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event("assign", 1.0,
                                 std::make_shared<AssignControllerAction>(
                                     "ego", make_controller(), /*activate_lateral=*/std::nullopt,
                                     /*activate_longitudinal=*/false)));
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(10.0, std::move(events))), Status::Ok);
    const std::optional<ControllerActivation> before = engine.controller_activation_of("ego");
    ASSERT_TRUE(before.has_value());
    EXPECT_TRUE(before->lateral);
    EXPECT_TRUE(before->longitudinal);

    ASSERT_EQ(engine.step(1.0), Status::Ok);
    const std::optional<ControllerActivation> after = engine.controller_activation_of("ego");
    ASSERT_TRUE(after.has_value());
    EXPECT_TRUE(after->lateral); // unset ⇒ no change
    EXPECT_FALSE(after->longitudinal);
}

TEST(ControllerActionTest, ActivatingADomainTheControllerTypeLacksIsRejected) {
    // per rule asam.net:xosc:1.2.0:scenario_logic.controller_activation
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event("assign", 0.0,
                                 std::make_shared<AssignControllerAction>(
                                     "ego", make_controller(ControllerType::Longitudinal),
                                     /*activate_lateral=*/true)));
    Engine engine;
    EXPECT_EQ(engine.init(make_scenario(10.0, std::move(events))), Status::ValidationError);
    ASSERT_FALSE(engine.diagnostics().empty());
    EXPECT_EQ(engine.diagnostics().front().rule_id,
              "asam.net:xosc:1.2.0:scenario_logic.controller_activation");
}

TEST(ControllerActionTest, ALightingControllerMayNotActivateAMovementDomain) {
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event(
        "assign", 0.0,
        std::make_shared<AssignControllerAction>("ego", make_controller(ControllerType::Lighting),
                                                 /*activate_lateral=*/std::nullopt,
                                                 /*activate_longitudinal=*/true)));
    Engine engine;
    EXPECT_EQ(engine.init(make_scenario(10.0, std::move(events))), Status::ValidationError);
}

TEST(ControllerActionTest, DeactivationIsAlwaysAllowedRegardlessOfControllerType) {
    // The rule constrains activation only; switching a domain off never
    // references a domain the controller claims.
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event(
        "assign", 0.0,
        std::make_shared<AssignControllerAction>("ego", make_controller(ControllerType::Lighting),
                                                 /*activate_lateral=*/false,
                                                 /*activate_longitudinal=*/false)));
    Engine engine;
    EXPECT_EQ(engine.init(make_scenario(10.0, std::move(events))), Status::Ok);
}

// --- ActivateControllerAction ---------------------------------------------

TEST(ActivateControllerTest, DeactivatingLongitudinalRetiresTheRunningOwnerAndHoldsSpeed) {
    std::vector<scena::ir::Event> events;
    events.push_back(
        timed_event("ramp", 0.0,
                    std::make_shared<SpeedAction>(
                        "ego", 30.0,
                        TransitionDynamics{DynamicsShape::Linear, DynamicsDimension::Time, 20.0})));
    events.push_back(timed_event("off", 4.0,
                                 std::make_shared<ActivateControllerAction>(
                                     "ego", /*lateral=*/std::nullopt, /*longitudinal=*/false)));
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(10.0, std::move(events))), Status::Ok);
    for (int i = 0; i < 4; ++i) { // to t = 4: 10 -> 14 m/s so far
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    const double held = engine.state("ego")->speed;
    EXPECT_NEAR(held, 14.0, kTol);
    EXPECT_EQ(*engine.storyboard_element_state(path_of("off")),
              scena::runtime::ElementState::Complete);

    // The ramp reports Complete on its next re-poll and stops driving; the
    // entity holds the speed it had when the domain was released.
    for (int i = 0; i < 3; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
        EXPECT_NEAR(engine.state("ego")->speed, held, kTol);
    }
    EXPECT_EQ(*engine.storyboard_element_state(path_of("ramp")),
              scena::runtime::ElementState::Complete);
}

TEST(ActivateControllerTest, ALongitudinalActionFiredWhileInactiveIsSkippedWithAWarning) {
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event("off", 1.0,
                                 std::make_shared<ActivateControllerAction>(
                                     "ego", /*lateral=*/std::nullopt, /*longitudinal=*/false)));
    events.push_back(timed_event("speed", 2.0, std::make_shared<SpeedAction>("ego", 25.0)));
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(10.0, std::move(events))), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    // The action was skipped (§7.5.2.2 missing prerequisite), so the speed is
    // untouched, and the event still completed.
    EXPECT_NEAR(engine.state("ego")->speed, 10.0, kTol);
    EXPECT_EQ(*engine.storyboard_element_state(path_of("speed")),
              scena::runtime::ElementState::Complete);
    bool warned = false;
    for (const scena::Diagnostic& diagnostic : engine.diagnostics()) {
        warned = warned || (diagnostic.severity == Severity::Warning &&
                            diagnostic.code == Status::InvalidControlMode);
    }
    EXPECT_TRUE(warned);
}

TEST(ActivateControllerTest, ReactivationResumesNormalDispatch) {
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event(
        "off", 1.0,
        std::make_shared<ActivateControllerAction>("ego", std::nullopt, /*longitudinal=*/false)));
    events.push_back(timed_event(
        "on", 2.0,
        std::make_shared<ActivateControllerAction>("ego", std::nullopt, /*longitudinal=*/true)));
    events.push_back(timed_event("speed", 3.0, std::make_shared<SpeedAction>("ego", 25.0)));
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(10.0, std::move(events))), Status::Ok);
    for (int i = 0; i < 3; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    ASSERT_TRUE(engine.controller_activation_of("ego").has_value());
    EXPECT_TRUE(engine.controller_activation_of("ego")->longitudinal);
    EXPECT_NEAR(engine.state("ego")->speed, 25.0, kTol);
}

TEST(ActivateControllerTest, DeactivatingLateralStopsAndSuppressesTrajectoryFollowing) {
    Trajectory trajectory;
    trajectory.vertices().push_back(TrajectoryVertex{WorldPosition{0.0, 0.0, 0.0}, std::nullopt});
    trajectory.vertices().push_back(TrajectoryVertex{WorldPosition{0.0, 500.0, 0.0}, std::nullopt});
    std::vector<scena::ir::Event> events;
    events.push_back(
        timed_event("follow", 1.0, std::make_shared<FollowTrajectoryAction>("ego", trajectory)));
    events.push_back(timed_event("off", 3.0,
                                 std::make_shared<ActivateControllerAction>(
                                     "ego", /*lateral=*/false, /*longitudinal=*/std::nullopt)));
    events.push_back(
        timed_event("again", 5.0, std::make_shared<FollowTrajectoryAction>("ego", trajectory)));
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(10.0, std::move(events))), Status::Ok);
    for (int i = 0; i < 3; ++i) { // the follower has steered the entity onto +y
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    EXPECT_NEAR(engine.state("ego")->y, 20.0, kTol);
    const double y_when_released = engine.state("ego")->y;

    // Released: the straight-line integrator takes over from the last pose
    // (heading pi/2, so it keeps going along +y — but under its own steam).
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_GT(engine.state("ego")->y, y_when_released);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(*engine.storyboard_element_state(path_of("follow")),
              scena::runtime::ElementState::Complete);

    // A new trajectory fired while lateral is inactive is skipped.
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(*engine.storyboard_element_state(path_of("again")),
              scena::runtime::ElementState::Complete);
    EXPECT_NEAR(engine.state("ego")->x, 0.0, kTol);
}

// --- VisibilityAction ------------------------------------------------------

TEST(VisibilityActionTest, DefaultsToVisibleAndTheActionFlipsTheFlags) {
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event("hide", 1.0,
                                 std::make_shared<VisibilityAction>("ego", /*graphics=*/false,
                                                                    /*sensors=*/true,
                                                                    /*traffic=*/false)));
    RecordingGateway gateway;
    Engine engine(&gateway);
    ASSERT_EQ(engine.init(make_scenario(10.0, std::move(events))), Status::Ok);
    // §VisibilityAction: "The default for entities is that they are visible
    // everywhere."
    const std::optional<EntityVisibility> before = engine.visibility_of("ego");
    ASSERT_TRUE(before.has_value());
    EXPECT_TRUE(before->graphics);
    EXPECT_TRUE(before->sensors);
    EXPECT_TRUE(before->traffic);
    EXPECT_TRUE(gateway.visibilities.empty());

    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(*engine.storyboard_element_state(path_of("hide")),
              scena::runtime::ElementState::Complete);
    const std::optional<EntityVisibility> after = engine.visibility_of("ego");
    ASSERT_TRUE(after.has_value());
    EXPECT_FALSE(after->graphics);
    EXPECT_TRUE(after->sensors);
    EXPECT_FALSE(after->traffic);

    ASSERT_EQ(gateway.visibilities.size(), 1U);
    EXPECT_EQ(gateway.visibilities[0].entity_id, "ego");
    EXPECT_FALSE(gateway.visibilities[0].visibility.graphics);
    EXPECT_TRUE(gateway.visibilities[0].visibility.sensors);
}

TEST(VisibilityActionTest, VisibilityDoesNotAffectMotion) {
    std::vector<scena::ir::Event> events;
    events.push_back(
        timed_event("hide", 1.0, std::make_shared<VisibilityAction>("ego", false, false, false)));
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(10.0, std::move(events))), Status::Ok);
    for (int i = 0; i < 3; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    EXPECT_NEAR(engine.state("ego")->x, 30.0, kTol);
    EXPECT_NEAR(engine.state("ego")->speed, 10.0, kTol);
}

TEST(VisibilityActionTest, UnknownQueryTargetsReportNothing) {
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(10.0, {})), Status::Ok);
    EXPECT_FALSE(engine.visibility_of("ghost").has_value());
    EXPECT_FALSE(engine.controller_activation_of("ghost").has_value());
    EXPECT_EQ(engine.assigned_controller_of("ghost"), nullptr);
}

// --- Gateway compatibility -------------------------------------------------

TEST(ControllerActionTest, AGatewayThatOverridesNothingNewStillRuns) {
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event(
        "assign", 1.0, std::make_shared<AssignControllerAction>("ego", make_controller())));
    events.push_back(
        timed_event("hide", 2.0, std::make_shared<VisibilityAction>("ego", false, false, false)));
    LegacyGateway gateway;
    Engine engine(&gateway);
    ASSERT_EQ(engine.init(make_scenario(10.0, std::move(events))), Status::Ok);
    for (int i = 0; i < 3; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    // The defaulted hand-offs are no-ops; everything else is unchanged.
    EXPECT_NEAR(gateway.published.x, 30.0, kTol);
    ASSERT_TRUE(engine.visibility_of("ego").has_value());
    EXPECT_FALSE(engine.visibility_of("ego")->graphics);
}

} // namespace
