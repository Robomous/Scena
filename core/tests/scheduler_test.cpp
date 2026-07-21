// SPDX-License-Identifier: MIT
#include "scena/runtime/scheduler.h"

#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/scenario.h"

using scena::ir::SimulationTimeCondition;
using scena::ir::SpeedAction;
using scena::ir::Storyboard;
using scena::runtime::ActionState;
using scena::runtime::Scheduler;

namespace {

Storyboard make_storyboard(const std::vector<std::pair<double, std::string>>& triggers) {
    Storyboard storyboard;
    for (const auto& [at_time, entity_id] : triggers) {
        storyboard.entries.push_back({std::make_shared<SimulationTimeCondition>(at_time),
                                      std::make_shared<SpeedAction>(entity_id, 10.0)});
    }
    return storyboard;
}

} // namespace

TEST(SchedulerTest, NoFiringBeforeTriggerTime) {
    const Storyboard storyboard = make_storyboard({{1.0, "ego"}});
    Scheduler scheduler;
    scheduler.bind(storyboard);

    int fired = 0;
    scheduler.step(0.5, [&](const scena::ir::Action&) { ++fired; });
    scheduler.step(0.999, [&](const scena::ir::Action&) { ++fired; });

    EXPECT_EQ(fired, 0);
    EXPECT_EQ(scheduler.action_state(0), ActionState::Pending);
}

TEST(SchedulerTest, FiresExactlyOnce) {
    const Storyboard storyboard = make_storyboard({{1.0, "ego"}});
    Scheduler scheduler;
    scheduler.bind(storyboard);

    int fired = 0;
    for (const double t : {0.5, 1.0, 1.5, 2.0, 100.0}) {
        scheduler.step(t, [&](const scena::ir::Action&) { ++fired; });
    }

    EXPECT_EQ(fired, 1);
    EXPECT_EQ(scheduler.action_state(0), ActionState::Complete);
}

TEST(SchedulerTest, StateTransitionsPendingToComplete) {
    const Storyboard storyboard = make_storyboard({{1.0, "ego"}});
    Scheduler scheduler;
    scheduler.bind(storyboard);

    ASSERT_EQ(scheduler.action_state(0), ActionState::Pending);

    // Instantaneous actions pass through Running within the firing step: the
    // action observes its entry in the Running state from the fire callback.
    ActionState state_during_fire = ActionState::Pending;
    scheduler.step(
        1.0, [&](const scena::ir::Action&) { state_during_fire = scheduler.action_state(0); });

    EXPECT_EQ(state_during_fire, ActionState::Running);
    EXPECT_EQ(scheduler.action_state(0), ActionState::Complete);
}

TEST(SchedulerTest, EntriesAreIndependent) {
    const Storyboard storyboard = make_storyboard({{1.0, "ego"}, {2.0, "npc"}});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    ASSERT_EQ(scheduler.entry_count(), 2U);

    std::vector<std::string> fired;
    const auto record = [&](const scena::ir::Action& action) {
        fired.push_back(action.entity_id());
    };

    scheduler.step(1.0, record);
    EXPECT_EQ(fired, (std::vector<std::string>{"ego"}));
    EXPECT_EQ(scheduler.action_state(0), ActionState::Complete);
    EXPECT_EQ(scheduler.action_state(1), ActionState::Pending);

    scheduler.step(2.5, record);
    EXPECT_EQ(fired, (std::vector<std::string>{"ego", "npc"}));
    EXPECT_EQ(scheduler.action_state(1), ActionState::Complete);
}

TEST(SchedulerTest, StepWithoutBindIsNoOp) {
    Scheduler scheduler;
    int fired = 0;
    scheduler.step(100.0, [&](const scena::ir::Action&) { ++fired; });
    EXPECT_EQ(fired, 0);
    EXPECT_EQ(scheduler.entry_count(), 0U);
}

TEST(SchedulerTest, RebindResetsStates) {
    const Storyboard storyboard = make_storyboard({{1.0, "ego"}});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    scheduler.step(1.0, nullptr);
    ASSERT_EQ(scheduler.action_state(0), ActionState::Complete);

    scheduler.bind(storyboard);
    EXPECT_EQ(scheduler.action_state(0), ActionState::Pending);
}
