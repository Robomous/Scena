// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
// Event lifecycle driven directly through the scheduler: action lifetime,
// event priority, execution counts and skip transitions, per ASAM
// OpenSCENARIO XML 1.4.0 §7.3.2, §8.2, §8.3.3.2 and §8.4.2.
#include "scena/runtime/scheduler.h"

#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/evaluation_context.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/trigger.h"

using scena::ir::Act;
using scena::ir::Condition;
using scena::ir::ConditionEdge;
using scena::ir::Event;
using scena::ir::EventPriority;
using scena::ir::Maneuver;
using scena::ir::ManeuverGroup;
using scena::ir::SimulationTimeCondition;
using scena::ir::SpeedAction;
using scena::ir::Story;
using scena::ir::Storyboard;
using scena::ir::Trigger;
using scena::runtime::ActionOutcome;
using scena::runtime::ElementState;
using scena::runtime::Scheduler;
using scena::runtime::TransitionKind;

namespace {

/// True on [from, until): a logical expression that both rises and falls,
/// which SimulationTimeCondition alone cannot express.
class TimeWindowCondition final : public Condition {
public:
    TimeWindowCondition(double from, double until) : from_(from), until_(until) {}

    [[nodiscard]] bool evaluate(const scena::ir::EvaluationContext& context) const override {
        const double simulation_time = context.simulation_time();
        return from_ <= simulation_time && simulation_time < until_;
    }

private:
    double from_;
    double until_;
};

std::shared_ptr<Condition> at_least(double at_time) {
    return std::make_shared<SimulationTimeCondition>(at_time);
}

std::shared_ptr<Condition> window(double from, double until) {
    return std::make_shared<TimeWindowCondition>(from, until);
}

/// Trigger holding one condition, with the optional edge and delay of
/// §7.6.2/§7.6.3.
Trigger when(std::shared_ptr<Condition> expression, ConditionEdge edge = ConditionEdge::None,
             double delay = 0.0) {
    return scena::ir::make_trigger(std::move(expression), edge, delay);
}

/// Applier that records every firing and reports ActionOutcome::Ongoing for
/// a chosen set of entity ids — the §7.5.3 never-ending case, which is what
/// keeps an event in runningState long enough for a sibling to interact
/// with it. The set is ordered so the fixture itself cannot introduce
/// iteration-order nondeterminism.
class Recorder {
public:
    explicit Recorder(std::set<std::string> ongoing = {}) : ongoing_(std::move(ongoing)) {}

    ActionOutcome operator()(const scena::ir::Action& action) {
        fired_.push_back(action.entity_id());
        return ongoing_.count(action.entity_id()) != 0 ? ActionOutcome::Ongoing
                                                       : ActionOutcome::Complete;
    }

    [[nodiscard]] const std::vector<std::string>& fired() const { return fired_; }
    void clear() { fired_.clear(); }

private:
    std::set<std::string> ongoing_;
    std::vector<std::string> fired_;
};

/// One step; returns what fired during it.
std::vector<std::string> step(Scheduler& scheduler, Recorder& recorder, double t) {
    recorder.clear();
    scheduler.step(t, [&recorder](const scena::ir::Action& action) { return recorder(action); });
    return recorder.fired();
}

/// An event whose single action targets an entity of the same name, so a
/// firing transcript reads as a list of event names.
Event make_event(std::string name, std::optional<Trigger> start_trigger,
                 EventPriority priority = EventPriority::Parallel,
                 int maximum_execution_count = 1) {
    Event event;
    event.name = name;
    event.start_trigger = std::move(start_trigger);
    event.priority = priority;
    event.maximum_execution_count = maximum_execution_count;
    event.actions.push_back(std::make_shared<SpeedAction>(std::move(name), 10.0));
    return event;
}

/// story/act/group/maneuver wrapper; the act and storyboard triggers are
/// optional.
Storyboard make_storyboard(std::vector<Event> events,
                           std::optional<Trigger> act_stop = std::nullopt,
                           std::optional<Trigger> storyboard_stop = std::nullopt) {
    Maneuver maneuver;
    maneuver.name = "maneuver";
    maneuver.events = std::move(events);
    ManeuverGroup group;
    group.name = "group";
    group.maneuvers.push_back(std::move(maneuver));
    Act act;
    act.name = "act";
    act.stop_trigger = std::move(act_stop);
    act.groups.push_back(std::move(group));
    Story story;
    story.name = "story";
    story.acts.push_back(std::move(act));
    Storyboard storyboard;
    storyboard.stories.push_back(std::move(story));
    storyboard.stop_trigger = std::move(storyboard_stop);
    return storyboard;
}

/// Two maneuvers under one group — the scope boundary priority must not
/// cross (§7.3.3).
Storyboard make_two_maneuver_storyboard(std::vector<Event> first, std::vector<Event> second) {
    Maneuver maneuver_a;
    maneuver_a.name = "first";
    maneuver_a.events = std::move(first);
    Maneuver maneuver_b;
    maneuver_b.name = "second";
    maneuver_b.events = std::move(second);
    ManeuverGroup group;
    group.name = "group";
    group.maneuvers.push_back(std::move(maneuver_a));
    group.maneuvers.push_back(std::move(maneuver_b));
    Act act;
    act.name = "act";
    act.groups.push_back(std::move(group));
    Story story;
    story.name = "story";
    story.acts.push_back(std::move(act));
    Storyboard storyboard;
    storyboard.stories.push_back(std::move(story));
    return storyboard;
}

std::string path(const std::string& event) {
    return "story/act/group/maneuver/" + event;
}

std::string path(const std::string& maneuver, const std::string& event) {
    return "story/act/group/" + maneuver + "/" + event;
}

} // namespace

// --- Action lifetime (§7.4.1.2, §7.5.3, §8.4.2) -----------------------------

TEST(EventPriorityTest, InstantaneousActionsEndTheEventInTheSameEvaluation) {
    const Storyboard storyboard = make_storyboard({make_event("a", when(at_least(1.0)))});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder;

    EXPECT_TRUE(step(scheduler, recorder, 0.5).empty());
    EXPECT_EQ(step(scheduler, recorder, 1.0), (std::vector<std::string>{"a"}));
    EXPECT_EQ(*scheduler.element_state(path("a")), ElementState::Complete);
    EXPECT_EQ(*scheduler.element_transition(path("a")), TransitionKind::End);
}

TEST(EventPriorityTest, OngoingActionKeepsTheEventRunningAcrossEvaluations) {
    const Storyboard storyboard = make_storyboard({make_event("a", when(at_least(1.0)))});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder({"a"});

    step(scheduler, recorder, 1.0);
    EXPECT_EQ(*scheduler.element_state(path("a")), ElementState::Running);
    EXPECT_EQ(*scheduler.element_transition(path("a")), TransitionKind::Start);

    step(scheduler, recorder, 2.0);
    EXPECT_EQ(*scheduler.element_state(path("a")), ElementState::Running);
}

TEST(EventPriorityTest, EventWithOneOngoingAmongInstantaneousActionsStaysRunning) {
    Event event = make_event("a", when(at_least(1.0)));
    event.actions.push_back(std::make_shared<SpeedAction>("lingering", 5.0));
    const Storyboard storyboard = make_storyboard({std::move(event)});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder({"lingering"});

    // Both actions fire, and the one that cannot end by itself keeps the
    // event in runningState (§8.4.2).
    EXPECT_EQ(step(scheduler, recorder, 1.0), (std::vector<std::string>{"a", "lingering"}));
    EXPECT_EQ(*scheduler.element_state(path("a")), ElementState::Running);
}

TEST(EventPriorityTest, EveryActionIsFiredEvenWhenAnEarlierOneIsOngoing) {
    Event event = make_event("a", when(at_least(1.0)));
    event.actions.push_back(std::make_shared<SpeedAction>("second", 5.0));
    const Storyboard storyboard = make_storyboard({std::move(event)});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder({"a"}); // the *first* action is the ongoing one

    EXPECT_EQ(step(scheduler, recorder, 1.0), (std::vector<std::string>{"a", "second"}));
}

TEST(EventPriorityTest, RunningEventDoesNotEvaluateItsStartTriggerAgain) {
    // §7.3.2: "there shall not be multiple instantiations of the same event
    // running simultaneously", and start triggers "only make sense for
    // events in standbyState". The trigger stays true, but the running event
    // must not start a second time even with executions left.
    const Storyboard storyboard =
        make_storyboard({make_event("a", when(at_least(1.0)), EventPriority::Parallel, /*max=*/3)});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder({"a"});

    EXPECT_EQ(step(scheduler, recorder, 1.0), (std::vector<std::string>{"a"}));
    EXPECT_TRUE(step(scheduler, recorder, 2.0).empty());
    EXPECT_TRUE(step(scheduler, recorder, 3.0).empty());
}

TEST(EventPriorityTest, RunningEventIsStoppedByTheActStopTrigger) {
    const Storyboard storyboard =
        make_storyboard({make_event("a", when(at_least(1.0)))}, when(at_least(2.0)));
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder({"a"});

    step(scheduler, recorder, 1.0);
    ASSERT_EQ(*scheduler.element_state(path("a")), ElementState::Running);

    step(scheduler, recorder, 2.0);
    EXPECT_EQ(*scheduler.element_state(path("a")), ElementState::Complete);
    EXPECT_EQ(*scheduler.element_transition(path("a")), TransitionKind::Stop);
}

// --- Priority: parallel (§8.4.2.2) -----------------------------------------

TEST(EventPriorityTest, ParallelIsTheDefaultPriority) {
    const Event event;
    EXPECT_EQ(event.priority, EventPriority::Parallel);
    EXPECT_EQ(event.maximum_execution_count, 1);
}

TEST(EventPriorityTest, ParallelEventStartsAlongsideARunningSibling) {
    const Storyboard storyboard = make_storyboard({
        make_event("a", when(at_least(1.0))),
        make_event("b", when(at_least(2.0))),
    });
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder({"a", "b"});

    step(scheduler, recorder, 1.0);
    EXPECT_EQ(step(scheduler, recorder, 2.0), (std::vector<std::string>{"b"}));
    EXPECT_EQ(*scheduler.element_state(path("a")), ElementState::Running);
    EXPECT_EQ(*scheduler.element_state(path("b")), ElementState::Running);
}

TEST(EventPriorityTest, ParallelEventsRunSimultaneouslyInOneManeuver) {
    const Storyboard storyboard = make_storyboard({
        make_event("a", when(at_least(1.0))),
        make_event("b", when(at_least(1.0))),
    });
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder({"a", "b"});

    EXPECT_EQ(step(scheduler, recorder, 1.0), (std::vector<std::string>{"a", "b"}));
    EXPECT_EQ(*scheduler.element_state(path("a")), ElementState::Running);
    EXPECT_EQ(*scheduler.element_state(path("b")), ElementState::Running);
}

// --- Priority: override (§8.4.2.2) -----------------------------------------

TEST(EventPriorityTest, OverrideStopsARunningSiblingWithStopTransition) {
    const Storyboard storyboard = make_storyboard({
        make_event("a", when(at_least(1.0))),
        make_event("b", when(at_least(2.0)), EventPriority::Override),
    });
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder({"a"});

    step(scheduler, recorder, 1.0);
    ASSERT_EQ(*scheduler.element_state(path("a")), ElementState::Running);

    EXPECT_EQ(step(scheduler, recorder, 2.0), (std::vector<std::string>{"b"}));
    EXPECT_EQ(*scheduler.element_state(path("a")), ElementState::Complete);
    EXPECT_EQ(*scheduler.element_transition(path("a")), TransitionKind::Stop);
}

TEST(EventPriorityTest, OverrideStartsItselfAfterStoppingTheSibling) {
    const Storyboard storyboard = make_storyboard({
        make_event("a", when(at_least(1.0))),
        make_event("b", when(at_least(2.0)), EventPriority::Override),
    });
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder({"a", "b"});

    step(scheduler, recorder, 1.0);
    step(scheduler, recorder, 2.0);
    EXPECT_EQ(*scheduler.element_state(path("b")), ElementState::Running);
    EXPECT_EQ(*scheduler.element_transition(path("b")), TransitionKind::Start);
}

TEST(EventPriorityTest, OverrideLeavesStandbySiblingsUntouched) {
    // §8.4.2.2 and the Priority class reference both say "running": a
    // standby sibling keeps its own start trigger and its executions.
    const Storyboard storyboard = make_storyboard({
        make_event("running", when(at_least(1.0))),
        make_event("standby", when(at_least(5.0))),
        make_event("overriding", when(at_least(2.0)), EventPriority::Override),
    });
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder({"running"});

    step(scheduler, recorder, 1.0);
    step(scheduler, recorder, 2.0);
    EXPECT_EQ(*scheduler.element_state(path("running")), ElementState::Complete);
    EXPECT_EQ(*scheduler.element_state(path("standby")), ElementState::Standby);

    EXPECT_EQ(step(scheduler, recorder, 5.0), (std::vector<std::string>{"standby"}));
}

TEST(EventPriorityTest, OverrideDoesNotReachAnotherManeuver) {
    // The scope is the Maneuver (§7.3.3), not the maneuver group.
    const Storyboard storyboard = make_two_maneuver_storyboard(
        {make_event("a", when(at_least(1.0)))},
        {make_event("b", when(at_least(2.0)), EventPriority::Override)});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder({"a"});

    step(scheduler, recorder, 1.0);
    step(scheduler, recorder, 2.0);
    EXPECT_EQ(*scheduler.element_state(path("first", "a")), ElementState::Running);
    EXPECT_EQ(*scheduler.element_state(path("second", "b")), ElementState::Complete);
}

TEST(EventPriorityTest, OverrideDoesNotStopTheTriggeringEventItself) {
    // "all events in running state ... as the starting event" excludes the
    // starting event, and the case cannot arise anyway: a running event has
    // no reachable start trigger.
    const Storyboard storyboard =
        make_storyboard({make_event("a", when(at_least(1.0)), EventPriority::Override)});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder({"a"});

    step(scheduler, recorder, 1.0);
    step(scheduler, recorder, 2.0);
    EXPECT_EQ(*scheduler.element_state(path("a")), ElementState::Running);
    EXPECT_EQ(*scheduler.element_transition(path("a")), TransitionKind::Start);
}

TEST(EventPriorityTest, OverrideClearsRemainingExecutionsOfTheStoppedSibling) {
    // §8.4.2.2: a terminated event completes "regardless of the number of
    // executions left", so the victim never runs again even though its
    // trigger keeps holding and it had two executions to spare.
    const Storyboard storyboard = make_storyboard({
        make_event("a", when(at_least(1.0)), EventPriority::Parallel, /*max=*/3),
        make_event("b", when(at_least(2.0)), EventPriority::Override),
    });
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder({"a"});

    step(scheduler, recorder, 1.0);
    step(scheduler, recorder, 2.0);
    ASSERT_EQ(*scheduler.element_state(path("a")), ElementState::Complete);

    EXPECT_TRUE(step(scheduler, recorder, 3.0).empty());
    EXPECT_TRUE(step(scheduler, recorder, 4.0).empty());
}

TEST(EventPriorityTest, OverrideWithoutARunningSiblingIsAPlainStart) {
    const Storyboard storyboard = make_storyboard({
        make_event("a", when(at_least(5.0))),
        make_event("b", when(at_least(1.0)), EventPriority::Override),
    });
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder;

    EXPECT_EQ(step(scheduler, recorder, 1.0), (std::vector<std::string>{"b"}));
    EXPECT_EQ(*scheduler.element_state(path("a")), ElementState::Standby);
}

// --- Priority: skip (§8.2, §8.4.2.1, §8.4.2.2) ------------------------------

TEST(EventPriorityTest, SkipEventDoesNotStartWhileASiblingRuns) {
    const Storyboard storyboard = make_storyboard({
        make_event("a", when(at_least(1.0))),
        make_event("b", when(at_least(2.0)), EventPriority::Skip),
    });
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder({"a"});

    step(scheduler, recorder, 1.0);
    EXPECT_TRUE(step(scheduler, recorder, 2.0).empty());
    EXPECT_EQ(*scheduler.element_transition(path("b")), TransitionKind::Skip);
}

TEST(EventPriorityTest, SkipWithDefaultCountCompletesTheEvent) {
    // With the default budget of one execution, the skipTransition exhausts
    // the event immediately (§8.4.2.1).
    const Storyboard storyboard = make_storyboard({
        make_event("a", when(at_least(1.0))),
        make_event("b", when(at_least(2.0)), EventPriority::Skip),
    });
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder({"a"});

    step(scheduler, recorder, 1.0);
    step(scheduler, recorder, 2.0);
    EXPECT_EQ(*scheduler.element_state(path("b")), ElementState::Complete);
    EXPECT_EQ(*scheduler.element_transition(path("b")), TransitionKind::Skip);
}

TEST(EventPriorityTest, SkipWithARemainingExecutionStaysInStandby) {
    const Storyboard storyboard = make_storyboard({
        make_event("a", when(at_least(1.0))),
        make_event("b", when(at_least(2.0)), EventPriority::Skip, /*max=*/2),
    });
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder({"a"});

    step(scheduler, recorder, 1.0);
    step(scheduler, recorder, 2.0);
    EXPECT_EQ(*scheduler.element_state(path("b")), ElementState::Standby);
    EXPECT_EQ(*scheduler.element_transition(path("b")), TransitionKind::Skip);
}

TEST(EventPriorityTest, SkipTransitionCountsAsAnExecution) {
    // §8.4.2.1: executions are startTransitions plus skipTransitions, so two
    // skips exhaust a budget of two without the event ever firing.
    const Storyboard storyboard = make_storyboard({
        make_event("a", when(at_least(1.0))),
        make_event("b", when(at_least(2.0)), EventPriority::Skip, /*max=*/2),
    });
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder({"a"});

    step(scheduler, recorder, 1.0);
    step(scheduler, recorder, 2.0);
    ASSERT_EQ(*scheduler.element_state(path("b")), ElementState::Standby);

    EXPECT_TRUE(step(scheduler, recorder, 3.0).empty());
    EXPECT_EQ(*scheduler.element_state(path("b")), ElementState::Complete);
    EXPECT_EQ(*scheduler.element_transition(path("b")), TransitionKind::Skip);
}

TEST(EventPriorityTest, SkipStartsNormallyWhenNoSiblingIsRunning) {
    // Priority class reference: skip only withholds the start "if there is
    // any other event in the same scope (maneuver) in the running state".
    const Storyboard storyboard = make_storyboard({
        make_event("a", when(at_least(5.0))),
        make_event("b", when(at_least(1.0)), EventPriority::Skip),
    });
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder;

    EXPECT_EQ(step(scheduler, recorder, 1.0), (std::vector<std::string>{"b"}));
    EXPECT_EQ(*scheduler.element_transition(path("b")), TransitionKind::End);
}

TEST(EventPriorityTest, SkipDoesNotSeeRunningEventsInAnotherManeuver) {
    const Storyboard storyboard =
        make_two_maneuver_storyboard({make_event("a", when(at_least(1.0)))},
                                     {make_event("b", when(at_least(2.0)), EventPriority::Skip)});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder({"a"});

    step(scheduler, recorder, 1.0);
    EXPECT_EQ(step(scheduler, recorder, 2.0), (std::vector<std::string>{"b"}));
}

// --- Execution counts (§8.3.3.2, §8.4.2.1) ---------------------------------

TEST(EventPriorityTest, DefaultCountExecutesTheEventOnce) {
    const Storyboard storyboard = make_storyboard({make_event("a", when(at_least(1.0)))});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder;

    EXPECT_EQ(step(scheduler, recorder, 1.0), (std::vector<std::string>{"a"}));
    EXPECT_TRUE(step(scheduler, recorder, 2.0).empty());
    EXPECT_TRUE(step(scheduler, recorder, 3.0).empty());
}

TEST(EventPriorityTest, TriggeredEventReExecutesOnEachTriggerUpToTheCount) {
    // The executions are performed sequentially (§8.3.3.2): the event ends,
    // re-arms, and starts again the next time its trigger holds.
    const Storyboard storyboard =
        make_storyboard({make_event("a", when(at_least(1.0)), EventPriority::Parallel, /*max=*/3)});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder;

    EXPECT_EQ(step(scheduler, recorder, 1.0), (std::vector<std::string>{"a"}));
    EXPECT_EQ(step(scheduler, recorder, 2.0), (std::vector<std::string>{"a"}));
    EXPECT_EQ(step(scheduler, recorder, 3.0), (std::vector<std::string>{"a"}));
    EXPECT_EQ(*scheduler.element_state(path("a")), ElementState::Complete);
    EXPECT_TRUE(step(scheduler, recorder, 4.0).empty());
}

TEST(EventPriorityTest, ReArmedEventSitsInStandbyBetweenExecutions) {
    const Storyboard storyboard =
        make_storyboard({make_event("a", when(at_least(1.0)), EventPriority::Parallel, /*max=*/2)});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder;

    step(scheduler, recorder, 1.0);
    EXPECT_EQ(*scheduler.element_state(path("a")), ElementState::Standby);
    EXPECT_EQ(*scheduler.element_transition(path("a")), TransitionKind::End);
}

TEST(EventPriorityTest, ExhaustedEventLeavesRunningWithEndTransition) {
    const Storyboard storyboard = make_storyboard({make_event("a", when(at_least(1.0)))});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder;

    step(scheduler, recorder, 1.0);
    EXPECT_EQ(*scheduler.element_state(path("a")), ElementState::Complete);
    EXPECT_EQ(*scheduler.element_transition(path("a")), TransitionKind::End);
}

TEST(EventPriorityTest, ReExecutionDoesNotResetConditionHistory) {
    // Re-arming returns the event to standbyState but leaves its condition
    // histories alone (only bind() rebuilds them), so a rising edge has to
    // rise again — it does not re-fire just because the expression is still
    // true.
    const Storyboard storyboard = make_storyboard({make_event(
        "a", when(window(1.0, 2.0), ConditionEdge::Rising), EventPriority::Parallel, /*max=*/3)});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder;

    EXPECT_TRUE(step(scheduler, recorder, 0.5).empty());
    EXPECT_EQ(step(scheduler, recorder, 1.0), (std::vector<std::string>{"a"}));
    EXPECT_TRUE(step(scheduler, recorder, 1.5).empty());
    EXPECT_TRUE(step(scheduler, recorder, 2.0).empty());
    EXPECT_EQ(*scheduler.element_state(path("a")), ElementState::Standby);
}

TEST(EventPriorityTest, EventWithoutItsOwnStartTriggerExecutesOnce) {
    // §8.4.2: such an event inherits the enclosing Act's start trigger,
    // which has already fired by the time the event re-arms and is not
    // re-evaluated while the act runs. It therefore executes once whatever
    // its budget says — no same-evaluation restart loop is invented.
    const Storyboard storyboard =
        make_storyboard({make_event("a", std::nullopt, EventPriority::Parallel, /*max=*/3)});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder;

    EXPECT_EQ(step(scheduler, recorder, 0.0), (std::vector<std::string>{"a"}));
    EXPECT_TRUE(step(scheduler, recorder, 1.0).empty());
    EXPECT_EQ(*scheduler.element_state(path("a")), ElementState::Standby);
}

TEST(EventPriorityTest, ZeroCountEventCompletesImmediatelyWithSkipTransition) {
    // §8.4.2.1's standby rule read with a budget of zero: the event is
    // exhausted before it ever starts, so it never fires.
    const Storyboard storyboard =
        make_storyboard({make_event("a", when(at_least(1.0)), EventPriority::Parallel, /*max=*/0)});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder;

    EXPECT_TRUE(step(scheduler, recorder, 0.0).empty());
    EXPECT_EQ(*scheduler.element_state(path("a")), ElementState::Complete);
    EXPECT_EQ(*scheduler.element_transition(path("a")), TransitionKind::Skip);
    EXPECT_TRUE(step(scheduler, recorder, 1.0).empty());
}

TEST(EventPriorityTest, ZeroCountEventStillLetsItsManeuverComplete) {
    const Storyboard storyboard =
        make_storyboard({make_event("a", when(at_least(1.0)), EventPriority::Parallel, /*max=*/0)});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder;

    step(scheduler, recorder, 0.0);
    EXPECT_EQ(*scheduler.element_state("story/act/group/maneuver"), ElementState::Complete);
    EXPECT_EQ(*scheduler.element_state("story/act"), ElementState::Complete);
}

TEST(EventPriorityTest, ZeroCountEventWithoutAStartTriggerNeverFires) {
    const Storyboard storyboard =
        make_storyboard({make_event("a", std::nullopt, EventPriority::Parallel, /*max=*/0)});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder;

    EXPECT_TRUE(step(scheduler, recorder, 0.0).empty());
    EXPECT_EQ(*scheduler.element_transition(path("a")), TransitionKind::Skip);
}

TEST(EventPriorityTest, ActStopTriggerCancelsRemainingExecutions) {
    const Storyboard storyboard =
        make_storyboard({make_event("a", when(at_least(1.0)), EventPriority::Parallel, /*max=*/3)},
                        when(at_least(2.0)));
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder;

    EXPECT_EQ(step(scheduler, recorder, 1.0), (std::vector<std::string>{"a"}));
    EXPECT_TRUE(step(scheduler, recorder, 2.0).empty());
    EXPECT_EQ(*scheduler.element_state(path("a")), ElementState::Complete);
    EXPECT_EQ(*scheduler.element_transition(path("a")), TransitionKind::Stop);
    EXPECT_TRUE(step(scheduler, recorder, 3.0).empty());
}

TEST(EventPriorityTest, StoryboardStopTriggerCancelsRemainingExecutionsFromStandby) {
    const Storyboard storyboard =
        make_storyboard({make_event("a", when(at_least(5.0)), EventPriority::Parallel, /*max=*/3)},
                        std::nullopt, when(at_least(2.0)));
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder;

    step(scheduler, recorder, 1.0);
    ASSERT_EQ(*scheduler.element_state(path("a")), ElementState::Standby);

    step(scheduler, recorder, 2.0);
    EXPECT_EQ(*scheduler.element_state(path("a")), ElementState::Complete);
    EXPECT_EQ(*scheduler.element_transition(path("a")), TransitionKind::Stop);
    EXPECT_TRUE(step(scheduler, recorder, 5.0).empty());
}

TEST(EventPriorityTest, RebindResetsExecutionCounts) {
    const Storyboard storyboard =
        make_storyboard({make_event("a", when(at_least(1.0)), EventPriority::Parallel, /*max=*/2)});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder;

    step(scheduler, recorder, 1.0);
    step(scheduler, recorder, 2.0);
    ASSERT_EQ(*scheduler.element_state(path("a")), ElementState::Complete);

    scheduler.bind(storyboard);
    EXPECT_EQ(step(scheduler, recorder, 1.0), (std::vector<std::string>{"a"}));
    EXPECT_EQ(step(scheduler, recorder, 2.0), (std::vector<std::string>{"a"}));
}

// --- Ordering and determinism ----------------------------------------------

TEST(EventPriorityTest, SimultaneousOverrideAndParallelResolveInDocumentOrder) {
    // The standard gives no ordering rule; Scena resolves a single pass in
    // document order, so the override wins only over what already started.
    {
        const Storyboard storyboard = make_storyboard({
            make_event("first", when(at_least(1.0))),
            make_event("second", when(at_least(1.0)), EventPriority::Override),
        });
        Scheduler scheduler;
        scheduler.bind(storyboard);
        Recorder recorder({"first", "second"});

        EXPECT_EQ(step(scheduler, recorder, 1.0), (std::vector<std::string>{"first", "second"}));
        EXPECT_EQ(*scheduler.element_state(path("first")), ElementState::Complete);
        EXPECT_EQ(*scheduler.element_transition(path("first")), TransitionKind::Stop);
        EXPECT_EQ(*scheduler.element_state(path("second")), ElementState::Running);
    }
    {
        const Storyboard storyboard = make_storyboard({
            make_event("first", when(at_least(1.0)), EventPriority::Override),
            make_event("second", when(at_least(1.0))),
        });
        Scheduler scheduler;
        scheduler.bind(storyboard);
        Recorder recorder({"first", "second"});

        EXPECT_EQ(step(scheduler, recorder, 1.0), (std::vector<std::string>{"first", "second"}));
        EXPECT_EQ(*scheduler.element_state(path("first")), ElementState::Running);
        EXPECT_EQ(*scheduler.element_state(path("second")), ElementState::Running);
    }
}

TEST(EventPriorityTest, SimultaneousSkipAndOverrideResolveInDocumentOrder) {
    {
        // The skip event comes first: nothing is running yet when its turn
        // comes, so it starts — and is then overridden.
        const Storyboard storyboard = make_storyboard({
            make_event("first", when(at_least(1.0)), EventPriority::Skip),
            make_event("second", when(at_least(1.0)), EventPriority::Override),
        });
        Scheduler scheduler;
        scheduler.bind(storyboard);
        Recorder recorder({"first", "second"});

        EXPECT_EQ(step(scheduler, recorder, 1.0), (std::vector<std::string>{"first", "second"}));
        EXPECT_EQ(*scheduler.element_transition(path("first")), TransitionKind::Stop);
    }
    {
        const Storyboard storyboard = make_storyboard({
            make_event("first", when(at_least(1.0)), EventPriority::Override),
            make_event("second", when(at_least(1.0)), EventPriority::Skip),
        });
        Scheduler scheduler;
        scheduler.bind(storyboard);
        Recorder recorder({"first", "second"});

        EXPECT_EQ(step(scheduler, recorder, 1.0), (std::vector<std::string>{"first"}));
        EXPECT_EQ(*scheduler.element_transition(path("second")), TransitionKind::Skip);
    }
}

TEST(EventPriorityTest, EveryStandbyTriggerIsEvaluatedEvenWhenAnEarlierOverrideFires) {
    // Regression guard for the no-short-circuit rule: resolving priority
    // must never replace a trigger evaluation. The delayed condition of
    // "late" needs the sample taken at t = 1.0 to fire at t = 1.5; if the
    // override at t = 1.0 caused its evaluation to be skipped, its firing
    // would slip to t = 2.0.
    const auto run = [](bool with_override) {
        std::vector<Event> events;
        events.push_back(make_event("a", when(at_least(1.0))));
        if (with_override) {
            events.push_back(make_event("b", when(at_least(1.0)), EventPriority::Override));
        }
        events.push_back(make_event("late", when(at_least(1.0), ConditionEdge::None, 0.5)));

        const Storyboard storyboard = make_storyboard(std::move(events));
        Scheduler scheduler;
        scheduler.bind(storyboard);
        Recorder recorder({"a", "b"});

        std::vector<double> late_firings;
        for (const double t : {0.5, 1.0, 1.5, 2.0}) {
            for (const std::string& entity : step(scheduler, recorder, t)) {
                if (entity == "late") {
                    late_firings.push_back(t);
                }
            }
        }
        return late_firings;
    };

    EXPECT_EQ(run(/*with_override=*/false), (std::vector<double>{1.5}));
    EXPECT_EQ(run(/*with_override=*/true), (std::vector<double>{1.5}));
}
