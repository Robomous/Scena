// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
//
// Private longitudinal actions at the action level (p5-s4): SpeedAction with a
// relative target (§RelativeTargetSpeed) resolved against a reference entity,
// both one-shot and continuous, plus multi-action supersession. The controller
// math itself is covered by longitudinal_test.cpp; here the focus is the IR
// surface and the engine's cross-entity resolution.

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "scena/engine.h"
#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/dynamics.h"
#include "scena/ir/entity.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/trigger.h"

namespace {

using scena::Engine;
using scena::EntityState;
using scena::Status;
using scena::ir::ControlMode;
using scena::ir::DynamicsDimension;
using scena::ir::DynamicsShape;
using scena::ir::Entity;
using scena::ir::RelativeTargetSpeed;
using scena::ir::Scenario;
using scena::ir::SimulationTimeCondition;
using scena::ir::SpeedAction;
using scena::ir::SpeedTargetValueType;
using scena::ir::TransitionDynamics;
using scena::ir::Trigger;

constexpr double kTol = 1e-12;

// --- IR surface -----------------------------------------------------------

TEST(SpeedActionIrTest, AbsoluteTargetIsNotRelative) {
    SpeedAction action("ego", 12.0);
    EXPECT_FALSE(action.is_relative());
    EXPECT_DOUBLE_EQ(action.target_speed(), 12.0);
    EXPECT_FALSE(action.relative_target().has_value());
    EXPECT_EQ(action.kind(), "SpeedAction");
    // The default short form is a Step transition.
    EXPECT_EQ(action.dynamics().shape, DynamicsShape::Step);
}

TEST(SpeedActionIrTest, RelativeTargetCarriesReferenceAndValueType) {
    RelativeTargetSpeed target{"lead", 5.0, SpeedTargetValueType::Delta, /*continuous=*/false};
    SpeedAction action("ego", target, {DynamicsShape::Linear, DynamicsDimension::Time, 3.0});
    ASSERT_TRUE(action.is_relative());
    ASSERT_TRUE(action.relative_target().has_value());
    EXPECT_EQ(action.relative_target()->entity_ref, "lead");
    EXPECT_DOUBLE_EQ(action.relative_target()->value, 5.0);
    EXPECT_EQ(action.relative_target()->value_type, SpeedTargetValueType::Delta);
    EXPECT_FALSE(action.relative_target()->continuous);
    EXPECT_EQ(action.dynamics().shape, DynamicsShape::Linear);
}

TEST(SpeedActionIrTest, ContinuousRelativeTargetIsFlagged) {
    RelativeTargetSpeed target{"lead", 1.1, SpeedTargetValueType::Factor, /*continuous=*/true};
    SpeedAction action("ego", target, {});
    ASSERT_TRUE(action.relative_target().has_value());
    EXPECT_TRUE(action.relative_target()->continuous);
    EXPECT_EQ(action.relative_target()->value_type, SpeedTargetValueType::Factor);
    EXPECT_NEAR(action.relative_target()->value, 1.1, kTol);
}

// --- Engine integration: relative targets and supersession ----------------

// One event carrying a single action, fired by a SimulationTimeCondition at
// `at_time` (0 ⇒ fires at the t=0 evaluation).
scena::ir::Event timed_event(const std::string& name, double at_time,
                             std::shared_ptr<scena::ir::Action> action) {
    scena::ir::Event event;
    event.name = name;
    event.start_trigger = scena::ir::make_trigger(
        std::make_shared<SimulationTimeCondition>(at_time), scena::ir::ConditionEdge::None, 0.0);
    event.actions.push_back(std::move(action));
    return event;
}

// A scenario with a "lead" reference vehicle and an "ego" vehicle (both
// engine-controlled), each given an initial speed, plus one maneuver holding
// `events` in document order. The lead maneuver is placed first so a same-step
// re-poll reads the lead's already-updated speed.
Scenario make_relative_scenario(double lead_speed, double ego_speed,
                                std::vector<scena::ir::Event> lead_events,
                                std::vector<scena::ir::Event> ego_events) {
    Scenario scenario;
    scenario.name = "relative-integration";
    for (const char* id : {"lead", "ego"}) {
        Entity entity;
        entity.id = id;
        entity.name = id;
        entity.control_mode = ControlMode::EngineControlled;
        scenario.entities.push_back(std::move(entity));
    }
    scenario.init_actions.push_back(std::make_shared<SpeedAction>("lead", lead_speed));
    if (ego_speed != 0.0) {
        scenario.init_actions.push_back(std::make_shared<SpeedAction>("ego", ego_speed));
    }

    auto maneuver = [](const char* name, std::vector<scena::ir::Event> events) {
        scena::ir::Maneuver m;
        m.name = name;
        m.events = std::move(events);
        return m;
    };
    scena::ir::ManeuverGroup group;
    group.name = "group";
    group.maneuvers.push_back(maneuver("lead_maneuver", std::move(lead_events)));
    group.maneuvers.push_back(maneuver("ego_maneuver", std::move(ego_events)));
    scena::ir::Act act;
    act.name = "act";
    act.groups.push_back(std::move(group));
    scena::ir::Story story;
    story.name = "story";
    story.acts.push_back(std::move(act));
    scenario.storyboard.stories.push_back(std::move(story));
    return scenario;
}

double speed_of(const Engine& engine, const std::string& id) {
    const std::optional<EntityState> state = engine.state(id);
    return state.has_value() ? state->speed : std::nan("");
}

scena::runtime::ElementState event_state(const Engine& engine, const std::string& maneuver,
                                         const std::string& event) {
    return *engine.storyboard_element_state("story/act/group/" + maneuver + "/" + event);
}

TEST(RelativeSpeedEngineTest, DeltaTargetResolvesAgainstReferenceAtStart) {
    // lead cruises at 8 m/s; ego ramps to lead + 5 = 13 over 2 s (linear).
    RelativeTargetSpeed target{"lead", 5.0, SpeedTargetValueType::Delta, /*continuous=*/false};
    Engine engine;
    ASSERT_EQ(engine.init(make_relative_scenario(
                  8.0, 0.0, {},
                  {timed_event("go", 0.0,
                               std::make_shared<SpeedAction>(
                                   "ego", target,
                                   TransitionDynamics{DynamicsShape::Linear,
                                                      DynamicsDimension::Time, 2.0}))})),
              Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_NEAR(speed_of(engine, "ego"), 6.5, kTol); // halfway up a 0->13 ramp
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_DOUBLE_EQ(speed_of(engine, "ego"), 13.0); // reached exactly
    EXPECT_EQ(event_state(engine, "ego_maneuver", "go"), scena::runtime::ElementState::Complete);
}

TEST(RelativeSpeedEngineTest, FactorTargetIsInstantaneousWithStepShape) {
    // ego jumps to 1.5x the lead's 10 m/s = 15 m/s at t=0 (Step).
    RelativeTargetSpeed target{"lead", 1.5, SpeedTargetValueType::Factor, /*continuous=*/false};
    Engine engine;
    ASSERT_EQ(
        engine.init(make_relative_scenario(
            10.0, 0.0, {},
            {timed_event("go", 0.0,
                         std::make_shared<SpeedAction>("ego", target, TransitionDynamics{}))})),
        Status::Ok);
    EXPECT_DOUBLE_EQ(speed_of(engine, "ego"), 15.0);
    EXPECT_EQ(event_state(engine, "ego_maneuver", "go"), scena::runtime::ElementState::Complete);
}

TEST(RelativeSpeedEngineTest, ContinuousTargetTracksReferenceAndNeverCompletes) {
    // lead constant at 8; ego continuously holds lead + 3 = 11. The event stays
    // Running across steps and the speed re-matches every step.
    RelativeTargetSpeed target{"lead", 3.0, SpeedTargetValueType::Delta, /*continuous=*/true};
    Engine engine;
    ASSERT_EQ(
        engine.init(make_relative_scenario(
            8.0, 0.0, {},
            {timed_event("hold", 0.0,
                         std::make_shared<SpeedAction>("ego", target, TransitionDynamics{}))})),
        Status::Ok);
    for (int i = 0; i < 3; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
        EXPECT_DOUBLE_EQ(speed_of(engine, "ego"), 11.0);
        EXPECT_EQ(event_state(engine, "ego_maneuver", "hold"),
                  scena::runtime::ElementState::Running);
    }
}

TEST(RelativeSpeedEngineTest, ContinuousTargetFollowsAChangingReference) {
    // lead ramps 8 -> 16 over 4 s; ego continuously holds lead + 2. Because the
    // lead maneuver is applied before ego's each step, ego reads the lead's
    // current speed with no lag.
    RelativeTargetSpeed target{"lead", 2.0, SpeedTargetValueType::Delta, /*continuous=*/true};
    Engine engine;
    ASSERT_EQ(
        engine.init(make_relative_scenario(
            8.0, 0.0,
            {timed_event(
                "lead_ramp", 0.0,
                std::make_shared<SpeedAction>(
                    "lead", 16.0,
                    TransitionDynamics{DynamicsShape::Linear, DynamicsDimension::Time, 4.0}))},
            {timed_event("hold", 0.0,
                         std::make_shared<SpeedAction>("ego", target, TransitionDynamics{}))})),
        Status::Ok);
    ASSERT_EQ(engine.step(2.0), Status::Ok);
    EXPECT_DOUBLE_EQ(speed_of(engine, "lead"), 12.0); // 2 s into a 4 s ramp
    EXPECT_DOUBLE_EQ(speed_of(engine, "ego"), 14.0);  // lead + 2, same step
}

TEST(RelativeSpeedEngineTest, LaterAbsoluteActionSupersedesARunningRamp) {
    // ego ramps toward 20 over 10 s, but at t=2 an absolute Step action jumps it
    // to 5. The ramp is retired: it neither reinstalls nor drives the entity.
    Engine engine;
    ASSERT_EQ(engine.init(make_relative_scenario(
                  8.0, 0.0, {},
                  {timed_event("ramp", 0.0,
                               std::make_shared<SpeedAction>(
                                   "ego", 20.0,
                                   TransitionDynamics{DynamicsShape::Linear,
                                                      DynamicsDimension::Time, 10.0})),
                   timed_event("override", 2.0, std::make_shared<SpeedAction>("ego", 5.0))})),
              Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_NEAR(speed_of(engine, "ego"), 2.0, kTol); // 1 s into a 0->20 / 10 s ramp
    ASSERT_EQ(engine.step(1.0), Status::Ok);         // t=2: override fires
    EXPECT_DOUBLE_EQ(speed_of(engine, "ego"), 5.0);
    EXPECT_EQ(event_state(engine, "ego_maneuver", "override"),
              scena::runtime::ElementState::Complete);
    // The ramp does not claw the entity back on the following step.
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_DOUBLE_EQ(speed_of(engine, "ego"), 5.0);
    EXPECT_EQ(event_state(engine, "ego_maneuver", "ramp"), scena::runtime::ElementState::Complete);
}

TEST(RelativeSpeedEngineTest, ContinuousWithTimedTransitionIsRejectedAtInit) {
    // §RelativeTargetSpeed: continuous must not combine with Dynamics.time.
    RelativeTargetSpeed target{"lead", 3.0, SpeedTargetValueType::Delta, /*continuous=*/true};
    Engine engine;
    const Status status = engine.init(make_relative_scenario(
        8.0, 0.0, {},
        {timed_event(
            "hold", 0.0,
            std::make_shared<SpeedAction>(
                "ego", target,
                TransitionDynamics{DynamicsShape::Linear, DynamicsDimension::Time, 2.0}))}));
    EXPECT_EQ(status, Status::ValidationError);
}

TEST(RelativeSpeedEngineTest, UnknownReferenceEntityIsRejectedAtInit) {
    RelativeTargetSpeed target{"ghost", 3.0, SpeedTargetValueType::Delta, /*continuous=*/false};
    Engine engine;
    const Status status = engine.init(make_relative_scenario(
        8.0, 0.0, {},
        {timed_event("go", 0.0,
                     std::make_shared<SpeedAction>("ego", target, TransitionDynamics{}))}));
    EXPECT_EQ(status, Status::SemanticError);
}

} // namespace
