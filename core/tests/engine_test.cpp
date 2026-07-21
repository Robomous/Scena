// SPDX-License-Identifier: MIT
#include "scena/engine.h"

#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/trigger.h"

using scena::Engine;
using scena::EntityState;
using scena::Status;
using scena::ir::ControlMode;
using scena::ir::Scenario;
using scena::ir::SimulationTimeCondition;
using scena::ir::SpeedAction;

namespace {

scena::ir::Event make_speed_event(std::string name, double at_time, std::string entity_id,
                                  double target_speed) {
    scena::ir::Event event;
    event.name = std::move(name);
    event.start_trigger =
        scena::ir::make_trigger(std::make_shared<SimulationTimeCondition>(at_time));
    event.actions.push_back(std::make_shared<SpeedAction>(std::move(entity_id), target_speed));
    return event;
}

/// One story/act/group/maneuver chain (all trigger-less) around the events.
Scenario make_scenario(std::vector<scena::ir::Event> events = {
                           make_speed_event("event-1", 0.0, "ego", 10.0)}) {
    Scenario scenario;
    scenario.name = "engine-test";
    scenario.entities.push_back({"ego", "ego vehicle", ControlMode::EngineControlled});
    scenario.entities.push_back({"npc", "host vehicle", ControlMode::HostControlled});

    scena::ir::Maneuver maneuver;
    maneuver.name = "maneuver";
    maneuver.events = std::move(events);
    scena::ir::ManeuverGroup group;
    group.name = "group";
    group.actors.push_back("ego");
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

} // namespace

TEST(EngineTest, LifecycleInitStepClose) {
    Engine engine;
    EXPECT_FALSE(engine.initialized());
    EXPECT_EQ(engine.init(make_scenario()), Status::Ok);
    EXPECT_TRUE(engine.initialized());
    EXPECT_EQ(engine.step(0.01), Status::Ok);
    EXPECT_EQ(engine.close(), Status::Ok);
    EXPECT_FALSE(engine.initialized());
}

TEST(EngineTest, DoubleInitFails) {
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario()), Status::Ok);
    EXPECT_EQ(engine.init(make_scenario()), Status::AlreadyInitialized);
}

TEST(EngineTest, StepBeforeInitFails) {
    Engine engine;
    EXPECT_EQ(engine.step(0.01), Status::NotInitialized);
}

TEST(EngineTest, CloseBeforeInitFails) {
    Engine engine;
    EXPECT_EQ(engine.close(), Status::NotInitialized);
}

TEST(EngineTest, ReinitAfterCloseWorks) {
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario({make_speed_event("event-1", 1.0, "ego", 10.0)})),
              Status::Ok);
    ASSERT_EQ(engine.step(2.0), Status::Ok);
    ASSERT_EQ(engine.close(), Status::Ok);
    ASSERT_EQ(engine.init(make_scenario({make_speed_event("event-1", 1.0, "ego", 10.0)})),
              Status::Ok);
    EXPECT_EQ(engine.time(), 0.0);
    const auto state = engine.state("ego");
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->speed, 0.0); // fresh zero-initialized state
}

TEST(EngineTest, NegativeOrNanDtIsRejected) {
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario()), Status::Ok);
    EXPECT_EQ(engine.step(-0.01), Status::InvalidArgument);
    EXPECT_EQ(engine.step(std::nan("")), Status::InvalidArgument);
    EXPECT_EQ(engine.step(0.0), Status::Ok); // zero dt is a valid no-advance step
}

TEST(EngineTest, StateQueries) {
    Engine engine;
    EXPECT_FALSE(engine.state("ego").has_value()); // before init
    ASSERT_EQ(engine.init(make_scenario()), Status::Ok);
    EXPECT_TRUE(engine.state("ego").has_value());
    EXPECT_TRUE(engine.state("npc").has_value());
    EXPECT_FALSE(engine.state("missing").has_value());
}

TEST(EngineTest, InitValidatesScenario) {
    {
        Engine engine;
        Scenario scenario = make_scenario();
        scenario.entities.push_back({"ego", "duplicate id", ControlMode::EngineControlled});
        EXPECT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
        EXPECT_FALSE(engine.initialized());
    }
    {
        Engine engine;
        Scenario scenario = make_scenario();
        scenario.entities.push_back({"", "empty id", ControlMode::EngineControlled});
        EXPECT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    }
    {
        // Action targeting an unknown entity anywhere in the tree.
        Engine engine;
        Scenario scenario = make_scenario({make_speed_event("event-1", 1.0, "missing", 5.0)});
        EXPECT_EQ(engine.init(std::move(scenario)), Status::SemanticError);
    }
    {
        // Null action.
        Engine engine;
        Scenario scenario = make_scenario();
        scenario.storyboard.stories[0].acts[0].groups[0].maneuvers[0].events[0].actions.push_back(
            nullptr);
        EXPECT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    }
    {
        // An event must carry at least one action (§8.3.2 cardinality).
        Engine engine;
        Scenario scenario = make_scenario();
        scenario.storyboard.stories[0].acts[0].groups[0].maneuvers[0].events[0].actions.clear();
        EXPECT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    }
    {
        // Sibling element names must be unique: they address the state query.
        Engine engine;
        Scenario scenario = make_scenario({make_speed_event("event-1", 1.0, "ego", 5.0),
                                           make_speed_event("event-1", 2.0, "ego", 7.0)});
        EXPECT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    }
    {
        // Element names must be non-empty.
        Engine engine;
        Scenario scenario = make_scenario();
        scenario.storyboard.stories[0].name.clear();
        EXPECT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    }
    {
        // Maneuver-group actors must reference known entities.
        Engine engine;
        Scenario scenario = make_scenario();
        scenario.storyboard.stories[0].acts[0].groups[0].actors.push_back("missing");
        EXPECT_EQ(engine.init(std::move(scenario)), Status::SemanticError);
    }
    {
        // Init actions are validated like storyboard actions.
        Engine engine;
        Scenario scenario = make_scenario();
        scenario.init_actions.push_back(std::make_shared<SpeedAction>("missing", 5.0));
        EXPECT_EQ(engine.init(std::move(scenario)), Status::SemanticError);
    }
}

TEST(EngineTest, InitRejectsNegativeConditionDelay) {
    // per rule asam.net:xosc:1.0.0:data_type.condition_delay_not_negative
    for (const double delay : {-0.5, std::numeric_limits<double>::quiet_NaN()}) {
        Engine engine;
        Scenario scenario = make_scenario();
        scenario.storyboard.stories[0]
            .acts[0]
            .groups[0]
            .maneuvers[0]
            .events[0]
            .start_trigger->groups[0]
            .conditions[0]
            .delay = delay;
        EXPECT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
        EXPECT_FALSE(engine.initialized());
    }
}

TEST(EngineTest, InitRejectsNegativeMaximumExecutionCount) {
    // §8.3.3.2: the attribute is an unsignedInt, so a negative budget has no
    // meaning. Zero is schema-valid and stays accepted (§8.4.2.1).
    Engine engine;
    Scenario scenario = make_scenario();
    scenario.storyboard.stories[0]
        .acts[0]
        .groups[0]
        .maneuvers[0]
        .events[0]
        .maximum_execution_count = -1;
    EXPECT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    EXPECT_FALSE(engine.initialized());
}

TEST(EngineTest, InitAcceptsZeroMaximumExecutionCount) {
    Engine engine;
    Scenario scenario = make_scenario();
    scenario.storyboard.stories[0]
        .acts[0]
        .groups[0]
        .maneuvers[0]
        .events[0]
        .maximum_execution_count = 0;
    EXPECT_EQ(engine.init(std::move(scenario)), Status::Ok);
}

TEST(EngineTest, InitRejectsNullTriggerExpression) {
    Engine engine;
    Scenario scenario = make_scenario();
    scenario.storyboard.stories[0]
        .acts[0]
        .groups[0]
        .maneuvers[0]
        .events[0]
        .start_trigger->groups[0]
        .conditions[0]
        .expression = nullptr;
    EXPECT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
}

TEST(EngineTest, InitRejectsEmptyConditionGroup) {
    // A condition group holds 1..* conditions; an empty one would be a
    // vacuously true conjunction (§7.6.1 and the ConditionGroup class
    // reference). An empty *trigger* stays legal — it is always false.
    {
        Engine engine;
        Scenario scenario = make_scenario();
        scenario.storyboard.stories[0]
            .acts[0]
            .groups[0]
            .maneuvers[0]
            .events[0]
            .start_trigger->groups[0]
            .conditions.clear();
        EXPECT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    }
    {
        Engine engine;
        Scenario scenario = make_scenario();
        scenario.storyboard.stop_trigger = scena::ir::Trigger{};
        scenario.storyboard.stories[0].acts[0].stop_trigger = scena::ir::Trigger{};
        EXPECT_EQ(engine.init(std::move(scenario)), Status::Ok);
    }
}

TEST(EngineTest, TimeAccumulates) {
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario()), Status::Ok);
    double expected = 0.0;
    for (int i = 0; i < 100; ++i) {
        ASSERT_EQ(engine.step(0.01), Status::Ok);
        expected += 0.01;
    }
    EXPECT_EQ(engine.time(), expected);
}

TEST(EngineTest, EngineControlledKinematicIntegration) {
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario()), Status::Ok);

    // The speed action's condition holds at t = 0, so it fires during init
    // (§8.4.7: the storyboard starts with the simulation); the entity then
    // advances along its zero heading (+X) at 10 m/s from the first step.
    const double dt = 0.01;
    double expected_x = 0.0;
    for (int i = 0; i < 100; ++i) {
        ASSERT_EQ(engine.step(dt), Status::Ok);
        expected_x += 10.0 * 1.0 * dt; // speed * cos(0) * dt, same op order as the engine
    }

    const auto state = engine.state("ego");
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->speed, 10.0);
    EXPECT_DOUBLE_EQ(state->x, expected_x);
    EXPECT_EQ(state->y, 0.0);
    EXPECT_EQ(state->z, 0.0);
}

TEST(EngineTest, HostControlledStateRoundTrip) {
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario()), Status::Ok);

    const EntityState reported{1.0, 2.0, 3.0, 0.5, 7.0};
    ASSERT_EQ(engine.report_state("npc", reported), Status::Ok);

    // The engine never integrates host-controlled entities: the reported state
    // survives a step bit-for-bit.
    ASSERT_EQ(engine.step(0.01), Status::Ok);
    const auto state = engine.state("npc");
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->x, reported.x);
    EXPECT_EQ(state->y, reported.y);
    EXPECT_EQ(state->z, reported.z);
    EXPECT_EQ(state->heading, reported.heading);
    EXPECT_EQ(state->speed, reported.speed);
}

TEST(EngineTest, ReportStateErrors) {
    Engine engine;
    const EntityState state{};
    EXPECT_EQ(engine.report_state("npc", state), Status::NotInitialized);
    ASSERT_EQ(engine.init(make_scenario()), Status::Ok);
    EXPECT_EQ(engine.report_state("missing", state), Status::UnknownEntity);
    EXPECT_EQ(engine.report_state("ego", state), Status::InvalidControlMode);
}

TEST(EngineTest, InitActionsApplyBeforeSimulationTime) {
    // An init action sets the initial speed before the storyboard runs
    // (§8.5.1): the entity has the init speed at t = 0 without any step, and
    // has not moved.
    Engine engine;
    Scenario scenario = make_scenario({make_speed_event("event-1", 1.0, "ego", 10.0)});
    scenario.init_actions.push_back(std::make_shared<SpeedAction>("ego", 5.0));
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);

    auto state = engine.state("ego");
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->speed, 5.0);
    EXPECT_EQ(state->x, 0.0); // init consumed no simulation time

    // The storyboard event later overrides the init speed.
    for (int i = 0; i < 150; ++i) {
        ASSERT_EQ(engine.step(0.01), Status::Ok);
    }
    state = engine.state("ego");
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->speed, 10.0);
}

TEST(EngineTest, StoryboardElementStateQuery) {
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario({make_speed_event("event-1", 1.0, "ego", 10.0)})),
              Status::Ok);

    // The storyboard itself is addressed by the empty path.
    ASSERT_TRUE(engine.storyboard_element_state("").has_value());
    EXPECT_EQ(*engine.storyboard_element_state(""), scena::runtime::ElementState::Running);

    ASSERT_TRUE(engine.storyboard_element_state("story/act/group/maneuver/event-1").has_value());
    EXPECT_EQ(*engine.storyboard_element_state("story/act/group/maneuver/event-1"),
              scena::runtime::ElementState::Standby);
    EXPECT_FALSE(engine.storyboard_element_state("story/no-such-act").has_value());

    for (int i = 0; i < 150; ++i) {
        ASSERT_EQ(engine.step(0.01), Status::Ok);
    }
    EXPECT_EQ(*engine.storyboard_element_state("story/act/group/maneuver/event-1"),
              scena::runtime::ElementState::Complete);
    EXPECT_EQ(*engine.storyboard_element_transition("story/act/group/maneuver/event-1"),
              scena::runtime::TransitionKind::End);
}
