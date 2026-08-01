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
// ManeuverGroup maximumExecutionCount (#52, ADR-0026): §8.4.4's re-arm rule,
// the subtree reset that makes each execution independent, the zero and
// negative budgets, and the stop trigger that completes a group "regardless of
// the number of execution counts left".

#include <memory>
#include <optional>
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

namespace {

using scena::Engine;
using scena::Status;
using scena::ir::Act;
using scena::ir::ControlMode;
using scena::ir::Entity;
using scena::ir::Event;
using scena::ir::Maneuver;
using scena::ir::ManeuverGroup;
using scena::ir::Scenario;
using scena::ir::SimulationTimeCondition;
using scena::ir::SpeedAction;
using scena::ir::Story;
using scena::runtime::ElementState;

constexpr const char* kGroupPath = "story/act/group";
constexpr const char* kEventPath = "story/act/group/maneuver/event";

/// A one-event group whose event fires immediately (no start trigger of its
/// own) and completes in the evaluation it fires, so the group ends regularly
/// on every evaluation and its execution budget is the only thing pacing it.
Scenario make_scenario(int group_executions, int event_executions = 1,
                       std::optional<double> stop_at = std::nullopt) {
    Scenario scenario;
    scenario.name = "group-executions";

    Entity ego;
    ego.id = "ego";
    ego.name = "ego";
    ego.control_mode = ControlMode::EngineControlled;
    scenario.entities.push_back(std::move(ego));

    Event event;
    event.name = "event";
    event.maximum_execution_count = event_executions;
    event.actions.push_back(std::make_shared<SpeedAction>("ego", 5.0));

    Maneuver maneuver;
    maneuver.name = "maneuver";
    maneuver.events.push_back(std::move(event));

    ManeuverGroup group;
    group.name = "group";
    group.maximum_execution_count = group_executions;
    group.maneuvers.push_back(std::move(maneuver));

    Act act;
    act.name = "act";
    if (stop_at.has_value()) {
        act.stop_trigger =
            scena::ir::make_trigger(std::make_shared<SimulationTimeCondition>(*stop_at));
    }
    act.groups.push_back(std::move(group));

    Story story;
    story.name = "story";
    story.acts.push_back(std::move(act));
    scenario.storyboard.stories.push_back(std::move(story));
    return scenario;
}

ElementState state_of(const Engine& engine, const char* path) {
    const std::optional<ElementState> state = engine.storyboard_element_state(path);
    EXPECT_TRUE(state.has_value()) << path;
    return state.value_or(ElementState::Standby);
}

TEST(ManeuverGroupExecutionsTest, ASingleExecutionCompletesTheGroupAsBefore) {
    // The default budget: unchanged behaviour, which is what keeps every other
    // storyboard test meaningful.
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(1)), Status::Ok);
    EXPECT_EQ(state_of(engine, kGroupPath), ElementState::Complete);
}

TEST(ManeuverGroupExecutionsTest, AGroupWithExecutionsLeftReArmsInsteadOfCompleting) {
    // §8.4.4: "if the number of executions is smaller than
    // maximumExecutionCount, the ManeuverGroup transfers from runningState into
    // standbyState and waits until the start trigger is executed". The trigger
    // is the Act's, which has already fired, so the group starts again on the
    // next evaluation — one execution per evaluation.
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(3)), Status::Ok);
    // init evaluates the storyboard once at t = 0: execution 1 ran and the
    // group re-armed.
    EXPECT_EQ(state_of(engine, kGroupPath), ElementState::Standby);

    ASSERT_EQ(engine.step(1.0), Status::Ok); // execution 2
    EXPECT_EQ(state_of(engine, kGroupPath), ElementState::Standby);
    ASSERT_EQ(engine.step(1.0), Status::Ok); // execution 3 — the budget is spent
    EXPECT_EQ(state_of(engine, kGroupPath), ElementState::Complete);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(state_of(engine, kGroupPath), ElementState::Complete);
}

TEST(ManeuverGroupExecutionsTest, EachExecutionRunsTheSubtreeAfresh) {
    // The event's own budget is 1, and it is spent by the group's first
    // execution. Without the subtree reset the group's later executions would
    // find an exhausted event and complete instantly having done nothing; with
    // it, every execution behaves like the first.
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(/*group_executions=*/3, /*event_executions=*/1)),
              Status::Ok);
    EXPECT_EQ(state_of(engine, kGroupPath), ElementState::Standby);
    // The event completed inside execution 1 and was reset with the subtree, so
    // it is back in standbyState rather than stuck at Complete.
    EXPECT_EQ(state_of(engine, kEventPath), ElementState::Standby);

    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(state_of(engine, kEventPath), ElementState::Standby);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    // Last execution: the group completes and its subtree completes with it.
    EXPECT_EQ(state_of(engine, kGroupPath), ElementState::Complete);
    EXPECT_EQ(state_of(engine, kEventPath), ElementState::Complete);
}

TEST(ManeuverGroupExecutionsTest, EveryExecutionRunsItsTriggeredEventAgain) {
    // A triggered event, not one that starts with its parent: the reset has to
    // put its trigger back in a state where it can fire again. The condition is
    // a level one (edge None) on t >= 1, so once it holds it holds for every
    // later execution and each of them ends one evaluation after it started.
    Scenario scenario = make_scenario(/*group_executions=*/3);
    Event& event =
        scenario.storyboard.stories.at(0).acts.at(0).groups.at(0).maneuvers.at(0).events.at(0);
    event.start_trigger = scena::ir::make_trigger(std::make_shared<SimulationTimeCondition>(1.0));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    // t = 0: the condition is false, so execution 1 is still waiting.
    EXPECT_EQ(state_of(engine, kGroupPath), ElementState::Running);

    // t = 1: the condition holds, the event fires and completes, and the group
    // ends execution 1 and re-arms.
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(state_of(engine, kGroupPath), ElementState::Standby);
    // t = 2: execution 2 starts, finds its event back in standbyState with its
    // budget restored, fires it and ends in the same evaluation.
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(state_of(engine, kGroupPath), ElementState::Standby);
    // t = 3: execution 3 does the same and exhausts the budget.
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(state_of(engine, kGroupPath), ElementState::Complete);
    EXPECT_EQ(state_of(engine, kEventPath), ElementState::Complete);
}

TEST(ManeuverGroupExecutionsTest, ClearedHistoriesMeanARisingEdgeDoesNotRefire) {
    // The reset clears each condition's edge history, so a later execution
    // starts with no previous sample. §7.6.4 makes the first check false, and a
    // condition that is already true never produces a rising edge afterwards —
    // so a rising-edge trigger fires in exactly one execution. That is the
    // deterministic consequence of resetting histories rather than carrying
    // them over, and it is documented in ADR-0026 rather than worked around.
    Scenario scenario = make_scenario(/*group_executions=*/3);
    Event& event =
        scenario.storyboard.stories.at(0).acts.at(0).groups.at(0).maneuvers.at(0).events.at(0);
    scena::ir::TriggerCondition condition;
    condition.expression = std::make_shared<SimulationTimeCondition>(1.0);
    condition.edge = scena::ir::ConditionEdge::Rising;
    scena::ir::ConditionGroup condition_group;
    condition_group.conditions.push_back(std::move(condition));
    scena::ir::Trigger trigger;
    trigger.groups.push_back(std::move(condition_group));
    event.start_trigger = std::move(trigger);

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    int ends = 0;
    ElementState previous = ElementState::Running;
    for (int i = 0; i < 12; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
        const ElementState now = state_of(engine, kGroupPath);
        if (previous == ElementState::Running && now != ElementState::Running) {
            ++ends;
        }
        previous = now;
    }
    EXPECT_EQ(ends, 1);
    // The group is still in its second execution, waiting for an edge that
    // cannot come again.
    EXPECT_EQ(state_of(engine, kGroupPath), ElementState::Running);
}

TEST(ManeuverGroupExecutionsTest, AZeroBudgetMeansTheGroupNeverRuns) {
    // Schema-valid, and §8.4.4 gives it a coherent reading: no startTransition
    // is ever performed. A group has no priority, so there is no skipTransition
    // to take — it completes.
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(0)), Status::Ok);
    EXPECT_EQ(state_of(engine, kGroupPath), ElementState::Complete);
    // Its subtree never ran either.
    EXPECT_EQ(state_of(engine, kEventPath), ElementState::Standby);
}

TEST(ManeuverGroupExecutionsTest, ANegativeBudgetIsRejectedAtInit) {
    Engine engine;
    EXPECT_EQ(engine.init(make_scenario(-1)), Status::ValidationError);
}

TEST(ManeuverGroupExecutionsTest, AStopTriggerCompletesTheGroupWithExecutionsLeft) {
    // §8.4.4: "when stopped, the ManeuverGroup transfers to completeState
    // regardless of the number of execution counts left".
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(/*group_executions=*/10, /*event_executions=*/1,
                                        /*stop_at=*/2.0)),
              Status::Ok);
    EXPECT_EQ(state_of(engine, kGroupPath), ElementState::Standby);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(state_of(engine, kGroupPath), ElementState::Standby);
    ASSERT_EQ(engine.step(1.0), Status::Ok); // t = 2: the act's stop trigger fires
    EXPECT_EQ(state_of(engine, kGroupPath), ElementState::Complete);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(state_of(engine, kGroupPath), ElementState::Complete);
}

TEST(ManeuverGroupExecutionsTest, ARepeatedRunIsBitIdentical) {
    const auto run = []() {
        Engine engine;
        EXPECT_EQ(engine.init(make_scenario(/*group_executions=*/5)), Status::Ok);
        std::vector<int> trace;
        for (int i = 0; i < 20; ++i) {
            EXPECT_EQ(engine.step(0.5), Status::Ok);
            trace.push_back(static_cast<int>(state_of(engine, kGroupPath)));
            trace.push_back(static_cast<int>(state_of(engine, kEventPath)));
        }
        return trace;
    };
    EXPECT_EQ(run(), run());
}

} // namespace
