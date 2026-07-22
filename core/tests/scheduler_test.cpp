// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#include "scena/runtime/scheduler.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/trigger.h"

using scena::ir::Act;
using scena::ir::Event;
using scena::ir::Maneuver;
using scena::ir::ManeuverGroup;
using scena::ir::SimulationTimeCondition;
using scena::ir::SpeedAction;
using scena::ir::Story;
using scena::ir::Storyboard;
using scena::runtime::ActionOutcome;
using scena::runtime::ElementState;
using scena::runtime::Scheduler;
using scena::runtime::TransitionKind;

namespace {

/// Helpers keep taking a bare logical expression and wrap it in a
/// one-group, one-condition trigger; a null expression means "no trigger".
Event make_event(std::string name, std::shared_ptr<scena::ir::Condition> trigger,
                 std::string entity_id, double target_speed) {
    Event event;
    event.name = std::move(name);
    if (trigger != nullptr) {
        event.start_trigger = scena::ir::make_trigger(std::move(trigger));
    }
    event.actions.push_back(std::make_shared<SpeedAction>(std::move(entity_id), target_speed));
    return event;
}

/// story/act/group/maneuver wrapper; the act's trigger is optional.
Storyboard make_storyboard(std::vector<Event> events,
                           std::shared_ptr<scena::ir::Condition> act_trigger = nullptr) {
    Maneuver maneuver;
    maneuver.name = "maneuver";
    maneuver.events = std::move(events);
    ManeuverGroup group;
    group.name = "group";
    group.maneuvers.push_back(std::move(maneuver));
    Act act;
    act.name = "act";
    if (act_trigger != nullptr) {
        act.start_trigger = scena::ir::make_trigger(std::move(act_trigger));
    }
    act.groups.push_back(std::move(group));
    Story story;
    story.name = "story";
    story.acts.push_back(std::move(act));
    Storyboard storyboard;
    storyboard.stories.push_back(std::move(story));
    return storyboard;
}

std::vector<std::string> fired_entities(Scheduler& scheduler, double t) {
    std::vector<std::string> fired;
    scheduler.step(t, [&](const scena::ir::Action& action) {
        fired.push_back(action.entity_id());
        return ActionOutcome::Complete;
    });
    return fired;
}

} // namespace

TEST(SchedulerTest, StepWithoutBindIsNoOp) {
    Scheduler scheduler;
    int fired = 0;
    scheduler.step(100.0, [&](const scena::ir::Action&) {
        ++fired;
        return ActionOutcome::Complete;
    });
    EXPECT_EQ(fired, 0);
    EXPECT_FALSE(scheduler.element_state("").has_value());
    EXPECT_FALSE(scheduler.storyboard_complete());
}

TEST(SchedulerTest, FirstStepStartsStoryboardAndTriggerlessChain) {
    // Event without a start trigger: the whole trigger-less chain starts and
    // fires on the first evaluation (child start rule, §8.3).
    const Storyboard storyboard = make_storyboard({make_event("event", nullptr, "ego", 10.0)});
    Scheduler scheduler;
    scheduler.bind(storyboard);

    EXPECT_EQ(fired_entities(scheduler, 0.0), (std::vector<std::string>{"ego"}));
    EXPECT_EQ(*scheduler.element_state(""), ElementState::Running);
    EXPECT_EQ(*scheduler.element_state("story/act/group/maneuver/event"), ElementState::Complete);
    EXPECT_EQ(*scheduler.element_transition("story/act/group/maneuver/event"), TransitionKind::End);
}

TEST(SchedulerTest, NoFiringBeforeTriggerTime) {
    const Storyboard storyboard = make_storyboard(
        {make_event("event", std::make_shared<SimulationTimeCondition>(1.0), "ego", 10.0)});
    Scheduler scheduler;
    scheduler.bind(storyboard);

    EXPECT_TRUE(fired_entities(scheduler, 0.0).empty());
    EXPECT_TRUE(fired_entities(scheduler, 0.999).empty());
    EXPECT_EQ(*scheduler.element_state("story/act/group/maneuver/event"), ElementState::Standby);
}

TEST(SchedulerTest, FiresExactlyOnce) {
    const Storyboard storyboard = make_storyboard(
        {make_event("event", std::make_shared<SimulationTimeCondition>(1.0), "ego", 10.0)});
    Scheduler scheduler;
    scheduler.bind(storyboard);

    int fired = 0;
    for (const double t : {0.5, 1.0, 1.5, 2.0, 100.0}) {
        scheduler.step(t, [&](const scena::ir::Action&) {
            ++fired;
            return ActionOutcome::Complete;
        });
    }
    EXPECT_EQ(fired, 1);
    EXPECT_EQ(*scheduler.element_state("story/act/group/maneuver/event"), ElementState::Complete);
}

TEST(SchedulerTest, SimultaneousEventsFireInDocumentOrder) {
    const Storyboard storyboard = make_storyboard(
        {make_event("first", std::make_shared<SimulationTimeCondition>(1.0), "a", 1.0),
         make_event("second", std::make_shared<SimulationTimeCondition>(1.0), "b", 2.0),
         make_event("third", std::make_shared<SimulationTimeCondition>(1.0), "c", 3.0)});
    Scheduler scheduler;
    scheduler.bind(storyboard);

    EXPECT_EQ(fired_entities(scheduler, 1.0), (std::vector<std::string>{"a", "b", "c"}));
}

TEST(SchedulerTest, ChildStartRule) {
    // An act with a start trigger waits in standby; its subtree only exists
    // once the act starts (§8.3).
    const Storyboard storyboard = make_storyboard({make_event("event", nullptr, "ego", 10.0)},
                                                  std::make_shared<SimulationTimeCondition>(2.0));
    Scheduler scheduler;
    scheduler.bind(storyboard);

    EXPECT_TRUE(fired_entities(scheduler, 0.0).empty());
    EXPECT_EQ(*scheduler.element_state("story"), ElementState::Running);
    EXPECT_EQ(*scheduler.element_state("story/act"), ElementState::Standby);

    // Act trigger holds: the trigger-less event fires in the same step.
    EXPECT_EQ(fired_entities(scheduler, 2.0), (std::vector<std::string>{"ego"}));
    EXPECT_EQ(*scheduler.element_state("story/act"), ElementState::Complete);
}

TEST(SchedulerTest, CompletionPropagatesUpTheHierarchy) {
    const Storyboard storyboard = make_storyboard(
        {make_event("event", std::make_shared<SimulationTimeCondition>(1.0), "ego", 10.0)});
    Scheduler scheduler;
    scheduler.bind(storyboard);

    (void)fired_entities(scheduler, 0.0);
    EXPECT_EQ(*scheduler.element_state("story"), ElementState::Running);

    (void)fired_entities(scheduler, 1.0);
    // Event -> Maneuver -> ManeuverGroup -> Act -> Story all complete with
    // regular endTransitions (§8.4.3–8.4.6)...
    for (const char* path : {"story/act/group/maneuver/event", "story/act/group/maneuver",
                             "story/act/group", "story/act", "story"}) {
        EXPECT_EQ(*scheduler.element_state(path), ElementState::Complete) << path;
        EXPECT_EQ(*scheduler.element_transition(path), TransitionKind::End) << path;
    }
    // ...but the storyboard never completes on its own (§8.4.7).
    EXPECT_EQ(*scheduler.element_state(""), ElementState::Running);
    EXPECT_FALSE(scheduler.storyboard_complete());
}

TEST(SchedulerTest, EmptyManeuverGroupCompletesInstantly) {
    // §8.4.4: an empty ManeuverGroup ends instantly.
    Storyboard storyboard = make_storyboard({make_event("event", nullptr, "ego", 10.0)});
    ManeuverGroup empty_group;
    empty_group.name = "empty-group";
    storyboard.stories[0].acts[0].groups.push_back(std::move(empty_group));

    Scheduler scheduler;
    scheduler.bind(storyboard);
    (void)fired_entities(scheduler, 0.0);
    EXPECT_EQ(*scheduler.element_state("story/act/empty-group"), ElementState::Complete);
}

TEST(SchedulerTest, StopTriggerStopsEverythingExecuting) {
    Storyboard storyboard = make_storyboard(
        {make_event("early", std::make_shared<SimulationTimeCondition>(1.0), "ego", 10.0),
         make_event("late", std::make_shared<SimulationTimeCondition>(5.0), "ego", 20.0)});
    storyboard.stop_trigger =
        scena::ir::make_trigger(std::make_shared<SimulationTimeCondition>(2.0));

    Scheduler scheduler;
    scheduler.bind(storyboard);
    (void)fired_entities(scheduler, 0.0);
    EXPECT_EQ(fired_entities(scheduler, 1.0), (std::vector<std::string>{"ego"}));

    // Stop trigger holds: the storyboard and every still-executing element
    // complete with stopTransition; the pending event never fires (§8.4.7).
    EXPECT_TRUE(fired_entities(scheduler, 2.0).empty());
    EXPECT_TRUE(scheduler.storyboard_complete());
    EXPECT_EQ(*scheduler.element_transition(""), TransitionKind::Stop);
    EXPECT_EQ(*scheduler.element_state("story/act/group/maneuver/late"), ElementState::Complete);
    EXPECT_EQ(*scheduler.element_transition("story/act/group/maneuver/late"), TransitionKind::Stop);
    // The regularly ended event keeps its endTransition.
    EXPECT_EQ(*scheduler.element_transition("story/act/group/maneuver/early"), TransitionKind::End);

    // A completed storyboard is inert.
    EXPECT_TRUE(fired_entities(scheduler, 10.0).empty());
}

TEST(SchedulerTest, UnknownPathsReturnNullopt) {
    const Storyboard storyboard = make_storyboard({make_event("event", nullptr, "ego", 10.0)});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    EXPECT_FALSE(scheduler.element_state("nope").has_value());
    EXPECT_FALSE(scheduler.element_state("story/act/group/maneuver/nope").has_value());
    EXPECT_FALSE(scheduler.element_transition("story/nope").has_value());
}

TEST(SchedulerTest, RebindResetsStates) {
    const Storyboard storyboard = make_storyboard(
        {make_event("event", std::make_shared<SimulationTimeCondition>(1.0), "ego", 10.0)});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    (void)fired_entities(scheduler, 1.0);
    ASSERT_EQ(*scheduler.element_state("story/act/group/maneuver/event"), ElementState::Complete);

    scheduler.bind(storyboard);
    EXPECT_EQ(fired_entities(scheduler, 1.0), (std::vector<std::string>{"ego"}));
}

TEST(SchedulerTest, ResetUnbinds) {
    const Storyboard storyboard = make_storyboard({make_event("event", nullptr, "ego", 10.0)});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    scheduler.reset();
    EXPECT_TRUE(fired_entities(scheduler, 0.0).empty());
    EXPECT_FALSE(scheduler.element_state("").has_value());
}

TEST(SchedulerTest, EventStaysRunningWhileAnActionIsOngoing) {
    // §8.4.2: the event "ends regularly when every nested Action is
    // completed", so an action that cannot end by itself (§7.5.3) keeps its
    // event in runningState across evaluations.
    const Storyboard storyboard = make_storyboard(
        {make_event("event", std::make_shared<SimulationTimeCondition>(1.0), "ego", 12.0)});
    Scheduler scheduler;
    scheduler.bind(storyboard);

    const auto ongoing = [](const scena::ir::Action&) { return ActionOutcome::Ongoing; };
    scheduler.step(1.0, ongoing);
    EXPECT_EQ(*scheduler.element_state("story/act/group/maneuver/event"), ElementState::Running);
    scheduler.step(2.0, ongoing);
    EXPECT_EQ(*scheduler.element_state("story/act/group/maneuver/event"), ElementState::Running);
    EXPECT_EQ(*scheduler.element_transition("story/act/group/maneuver/event"),
              TransitionKind::Start);
}

TEST(SchedulerTest, CompletionRollUpWaitsForOngoingEvents) {
    // An event has no children, so the child->parent roll-up of §8.4.3–8.4.6
    // must not treat a running event as vacuously complete.
    const Storyboard storyboard = make_storyboard(
        {make_event("event", std::make_shared<SimulationTimeCondition>(1.0), "ego", 12.0)});
    Scheduler scheduler;
    scheduler.bind(storyboard);

    scheduler.step(1.0, [](const scena::ir::Action&) { return ActionOutcome::Ongoing; });
    EXPECT_EQ(*scheduler.element_state("story/act/group/maneuver"), ElementState::Running);
    EXPECT_EQ(*scheduler.element_state("story/act"), ElementState::Running);
    EXPECT_EQ(*scheduler.element_state("story"), ElementState::Running);
}

TEST(SchedulerTest, TransitionActionIsRePolledUntilComplete) {
    // An action reporting Running (transition dynamics) keeps its event in
    // runningState and is re-polled each step until it reports Complete, at
    // which point the event ends regularly (§8.4.2).
    const Storyboard storyboard = make_storyboard({make_event("event", nullptr, "ego", 10.0)});
    Scheduler scheduler;
    scheduler.bind(storyboard);

    int calls = 0;
    auto ramp = [&](const scena::ir::Action&) {
        ++calls;
        return calls < 3 ? ActionOutcome::Running : ActionOutcome::Complete;
    };

    scheduler.step(0.0, ramp); // call 1: initial fire -> Running
    EXPECT_EQ(*scheduler.element_state("story/act/group/maneuver/event"), ElementState::Running);
    scheduler.step(1.0, ramp); // call 2: re-poll -> Running
    EXPECT_EQ(*scheduler.element_state("story/act/group/maneuver/event"), ElementState::Running);
    scheduler.step(2.0, ramp); // call 3: re-poll -> Complete, event ends regularly
    EXPECT_EQ(calls, 3);
    EXPECT_EQ(*scheduler.element_state("story/act/group/maneuver/event"), ElementState::Complete);
    EXPECT_EQ(*scheduler.element_transition("story/act/group/maneuver/event"), TransitionKind::End);
    EXPECT_EQ(*scheduler.element_state("story/act/group/maneuver"), ElementState::Complete);

    // Once complete, the action is no longer re-polled.
    scheduler.step(3.0, ramp);
    EXPECT_EQ(calls, 3);
}

TEST(SchedulerTest, NeverEndingActionIsNotRePolled) {
    // Ongoing (never-ending, §7.5.3) is distinct from Running: it fires once
    // and is never re-polled, so a stopTransition is the only way out.
    const Storyboard storyboard = make_storyboard({make_event("event", nullptr, "ego", 10.0)});
    Scheduler scheduler;
    scheduler.bind(storyboard);

    int calls = 0;
    auto ongoing = [&](const scena::ir::Action&) {
        ++calls;
        return ActionOutcome::Ongoing;
    };
    scheduler.step(0.0, ongoing);
    scheduler.step(1.0, ongoing);
    scheduler.step(2.0, ongoing);
    EXPECT_EQ(calls, 1);
    EXPECT_EQ(*scheduler.element_state("story/act/group/maneuver/event"), ElementState::Running);
}

TEST(SchedulerTest, RunningAlongsideNeverEndingStaysRunning) {
    // An event with both a transition (Running) and a never-ending (Ongoing)
    // action: completing the transition does not end the event — the
    // never-ending action holds it in runningState (§8.4.2, "every nested
    // Action is completed").
    Event event;
    event.name = "event";
    event.actions.push_back(std::make_shared<SpeedAction>("ego", 10.0));  // transition
    event.actions.push_back(std::make_shared<SpeedAction>("other", 5.0)); // never-ending
    std::vector<Event> events;
    events.push_back(std::move(event));
    const Storyboard storyboard = make_storyboard(std::move(events));
    Scheduler scheduler;
    scheduler.bind(storyboard);

    // ego ramps and completes after one re-poll; other never ends.
    int ego_calls = 0;
    auto drive_two = [&](const scena::ir::Action& action) {
        if (action.entity_id() == "other") {
            return ActionOutcome::Ongoing;
        }
        ++ego_calls;
        return ego_calls < 2 ? ActionOutcome::Running : ActionOutcome::Complete;
    };

    scheduler.step(0.0, drive_two); // ego Running, other Ongoing
    EXPECT_EQ(*scheduler.element_state("story/act/group/maneuver/event"), ElementState::Running);
    scheduler.step(1.0, drive_two); // ego Complete, other still Ongoing
    EXPECT_EQ(*scheduler.element_state("story/act/group/maneuver/event"), ElementState::Running);
}
