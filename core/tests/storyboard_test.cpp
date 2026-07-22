// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
// Engine-level storyboard semantics: init phase, element lifecycle
// observation, stop trigger, and multi-story parallelism, per ASAM
// OpenSCENARIO XML 1.4.0 §8.
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "scena/engine.h"
#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/trigger.h"

using scena::Engine;
using scena::Status;
using scena::ir::Act;
using scena::ir::ControlMode;
using scena::ir::Event;
using scena::ir::Maneuver;
using scena::ir::ManeuverGroup;
using scena::ir::Scenario;
using scena::ir::SimulationTimeCondition;
using scena::ir::SpeedAction;
using scena::ir::Story;
using scena::runtime::ElementState;
using scena::runtime::TransitionKind;

namespace {

Event make_speed_event(std::string name, std::shared_ptr<scena::ir::Condition> trigger,
                       std::string entity_id, double target_speed) {
    Event event;
    event.name = std::move(name);
    if (trigger != nullptr) {
        event.start_trigger = scena::ir::make_trigger(std::move(trigger));
    }
    event.actions.push_back(std::make_shared<SpeedAction>(std::move(entity_id), target_speed));
    return event;
}

Story make_story(std::string name, std::string act_name,
                 std::shared_ptr<scena::ir::Condition> act_trigger, std::vector<Event> events) {
    Maneuver maneuver;
    maneuver.name = "maneuver";
    maneuver.events = std::move(events);
    ManeuverGroup group;
    group.name = "group";
    group.maneuvers.push_back(std::move(maneuver));
    Act act;
    act.name = std::move(act_name);
    if (act_trigger != nullptr) {
        act.start_trigger = scena::ir::make_trigger(std::move(act_trigger));
    }
    act.groups.push_back(std::move(group));
    Story story;
    story.name = std::move(name);
    story.acts.push_back(std::move(act));
    return story;
}

Scenario make_base_scenario() {
    Scenario scenario;
    scenario.name = "storyboard-test";
    scenario.entities.push_back(
        {.id = "ego", .name = "ego vehicle", .control_mode = ControlMode::EngineControlled});
    scenario.entities.push_back(
        {.id = "lead", .name = "lead vehicle", .control_mode = ControlMode::EngineControlled});
    return scenario;
}

} // namespace

TEST(StoryboardTest, TimeZeroTriggersFireDuringInit) {
    // A start condition that already holds at t = 0 fires during init: the
    // storyboard starts with the simulation (§8.4.7).
    Scenario scenario = make_base_scenario();
    scenario.storyboard.stories.push_back(make_story(
        "story", "act", nullptr,
        {make_speed_event("event", std::make_shared<SimulationTimeCondition>(0.0), "ego", 10.0)}));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    const auto state = engine.state("ego");
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->speed, 10.0); // fired at init, before any step
    EXPECT_EQ(state->x, 0.0);      // no simulation time consumed
}

TEST(StoryboardTest, ElementLifecycleIsObservable) {
    Scenario scenario = make_base_scenario();
    Event event =
        make_speed_event("event", std::make_shared<SimulationTimeCondition>(2.0), "ego", 10.0);
    scenario.storyboard.stories.push_back(make_story(
        "story", "act", std::make_shared<SimulationTimeCondition>(1.0), {std::move(event)}));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);

    // After init: storyboard + story running, act waiting on its trigger.
    EXPECT_EQ(*engine.storyboard_element_state(""), ElementState::Running);
    EXPECT_EQ(*engine.storyboard_element_state("story"), ElementState::Running);
    EXPECT_EQ(*engine.storyboard_element_state("story/act"), ElementState::Standby);
    EXPECT_EQ(*engine.storyboard_element_transition("story/act"), TransitionKind::None);

    // t = 1.5: act started (its child event now waits in standby).
    ASSERT_EQ(engine.step(1.5), Status::Ok);
    EXPECT_EQ(*engine.storyboard_element_state("story/act"), ElementState::Running);
    EXPECT_EQ(*engine.storyboard_element_transition("story/act"), TransitionKind::Start);
    EXPECT_EQ(*engine.storyboard_element_state("story/act/group/maneuver/event"),
              ElementState::Standby);

    // t = 2.5: event fired and the whole chain completed regularly; the
    // storyboard keeps running (§8.4.7).
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    for (const char* path : {"story/act/group/maneuver/event", "story/act/group/maneuver",
                             "story/act/group", "story/act", "story"}) {
        EXPECT_EQ(*engine.storyboard_element_state(path), ElementState::Complete) << path;
        EXPECT_EQ(*engine.storyboard_element_transition(path), TransitionKind::End) << path;
    }
    EXPECT_EQ(*engine.storyboard_element_state(""), ElementState::Running);
    EXPECT_EQ(engine.state("ego")->speed, 10.0);
}

TEST(StoryboardTest, StopTriggerPreventsPendingEvents) {
    Scenario scenario = make_base_scenario();
    scenario.storyboard.stories.push_back(make_story(
        "story", "act", nullptr,
        {make_speed_event("late", std::make_shared<SimulationTimeCondition>(5.0), "ego", 20.0)}));
    scenario.storyboard.stop_trigger =
        scena::ir::make_trigger(std::make_shared<SimulationTimeCondition>(2.0));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);

    for (int i = 0; i < 600; ++i) { // 6 simulated seconds
        ASSERT_EQ(engine.step(0.01), Status::Ok);
    }

    // The storyboard stopped at t = 2.0; the t = 5.0 event never fired.
    EXPECT_EQ(*engine.storyboard_element_state(""), ElementState::Complete);
    EXPECT_EQ(*engine.storyboard_element_transition(""), TransitionKind::Stop);
    EXPECT_EQ(*engine.storyboard_element_transition("story/act/group/maneuver/late"),
              TransitionKind::Stop);
    EXPECT_EQ(engine.state("ego")->speed, 0.0);

    // Stepping past a completed storyboard stays valid; entities keep their
    // last state's motion (the engine still integrates kinematics).
    ASSERT_EQ(engine.step(0.01), Status::Ok);
}

TEST(StoryboardTest, ParallelStoriesRunIndependently) {
    // Two stories execute in parallel (§8.3.3.1); each completes on its own.
    Scenario scenario = make_base_scenario();
    scenario.storyboard.stories.push_back(make_story(
        "story-a", "act", nullptr,
        {make_speed_event("event", std::make_shared<SimulationTimeCondition>(1.0), "ego", 10.0)}));
    scenario.storyboard.stories.push_back(make_story(
        "story-b", "act", nullptr,
        {make_speed_event("event", std::make_shared<SimulationTimeCondition>(3.0), "lead", 8.0)}));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);

    ASSERT_EQ(engine.step(2.0), Status::Ok);
    EXPECT_EQ(*engine.storyboard_element_state("story-a"), ElementState::Complete);
    EXPECT_EQ(*engine.storyboard_element_state("story-b"), ElementState::Running);
    EXPECT_EQ(engine.state("ego")->speed, 10.0);
    EXPECT_EQ(engine.state("lead")->speed, 0.0);

    ASSERT_EQ(engine.step(2.0), Status::Ok);
    EXPECT_EQ(*engine.storyboard_element_state("story-b"), ElementState::Complete);
    EXPECT_EQ(engine.state("lead")->speed, 8.0);
    EXPECT_EQ(*engine.storyboard_element_state(""), ElementState::Running);
}

TEST(StoryboardTest, MultipleEventsInManeuverRunToCompletion) {
    Scenario scenario = make_base_scenario();
    Event accelerate =
        make_speed_event("accelerate", std::make_shared<SimulationTimeCondition>(1.0), "ego", 15.0);
    Event settle =
        make_speed_event("settle", std::make_shared<SimulationTimeCondition>(2.0), "ego", 9.0);
    scenario.storyboard.stories.push_back(
        make_story("story", "act", nullptr, {std::move(accelerate), std::move(settle)}));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);

    const std::string maneuver_path = "story/act/group/maneuver";
    ASSERT_EQ(engine.step(1.5), Status::Ok);
    EXPECT_EQ(engine.state("ego")->speed, 15.0);
    EXPECT_EQ(*engine.storyboard_element_state(maneuver_path), ElementState::Running);

    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(engine.state("ego")->speed, 9.0);
    EXPECT_EQ(*engine.storyboard_element_state(maneuver_path), ElementState::Complete);
}

TEST(StoryboardTest, EventReExecutesUpToMaximumExecutionCount) {
    // §8.3.3.2: executions are performed sequentially — the event ends,
    // re-arms to standbyState and starts again while its trigger holds.
    Scenario scenario = make_base_scenario();
    Event repeated =
        make_speed_event("repeat", std::make_shared<SimulationTimeCondition>(1.0), "ego", 7.0);
    repeated.maximum_execution_count = 3;
    scenario.storyboard.stories.push_back(
        make_story("story", "act", nullptr, {std::move(repeated)}));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);

    const char* const kPath = "story/act/group/maneuver/repeat";
    ASSERT_EQ(engine.storyboard_element_state(kPath), ElementState::Standby);

    // Three evaluations at or beyond the trigger time spend the budget; the
    // event stands by between them.
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(engine.storyboard_element_state(kPath), ElementState::Standby);
    EXPECT_EQ(engine.storyboard_element_transition(kPath), TransitionKind::End);

    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(engine.storyboard_element_state(kPath), ElementState::Standby);

    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(engine.storyboard_element_state(kPath), ElementState::Complete);
    EXPECT_EQ(engine.storyboard_element_transition(kPath), TransitionKind::End);
}

TEST(StoryboardTest, ZeroExecutionCountEventNeverFires) {
    // §8.4.2.1 read with a budget of zero: the event is exhausted in
    // standbyState and completes with a skipTransition without executing.
    Scenario scenario = make_base_scenario();
    Event never =
        make_speed_event("never", std::make_shared<SimulationTimeCondition>(0.0), "ego", 42.0);
    never.maximum_execution_count = 0;
    scenario.storyboard.stories.push_back(make_story("story", "act", nullptr, {std::move(never)}));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);

    const char* const kPath = "story/act/group/maneuver/never";
    EXPECT_EQ(engine.storyboard_element_state(kPath), ElementState::Complete);
    EXPECT_EQ(engine.storyboard_element_transition(kPath), TransitionKind::Skip);
    const auto state = engine.state("ego");
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->speed, 0.0);
}

TEST(StoryboardTest, PriorityDefaultsPreserveSingleExecutionBehaviour) {
    // Every action the engine can apply is instantaneous (§7.4.1.2), so no
    // event is ever in runningState when a sibling starts and the default
    // parallel priority leaves the p1-s1/p1-s2 behaviour untouched.
    Scenario scenario = make_base_scenario();
    scenario.storyboard.stories.push_back(make_story(
        "story", "act", nullptr,
        {make_speed_event("first", std::make_shared<SimulationTimeCondition>(0.0), "ego", 11.0),
         make_speed_event("second", std::make_shared<SimulationTimeCondition>(0.0), "lead", 9.0)}));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);

    for (const char* path : {"story/act/group/maneuver/first", "story/act/group/maneuver/second"}) {
        EXPECT_EQ(engine.storyboard_element_state(path), ElementState::Complete) << path;
        EXPECT_EQ(engine.storyboard_element_transition(path), TransitionKind::End) << path;
    }
    EXPECT_EQ(engine.storyboard_element_state("story"), ElementState::Complete);
}
