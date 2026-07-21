// SPDX-License-Identifier: MIT
// Trigger semantics driven directly through the scheduler: condition
// groups, condition edges, delays and act stop triggers, per ASAM
// OpenSCENARIO XML 1.4.0 §7.6.
#include "scena/runtime/scheduler.h"

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/trigger.h"

using scena::ir::Act;
using scena::ir::Condition;
using scena::ir::ConditionEdge;
using scena::ir::ConditionGroup;
using scena::ir::Event;
using scena::ir::Maneuver;
using scena::ir::ManeuverGroup;
using scena::ir::SimulationTimeCondition;
using scena::ir::SpeedAction;
using scena::ir::Story;
using scena::ir::Storyboard;
using scena::ir::Trigger;
using scena::ir::TriggerCondition;
using scena::runtime::ActionOutcome;
using scena::runtime::ElementState;
using scena::runtime::Scheduler;
using scena::runtime::TransitionKind;

namespace {

/// Logical expression with a fixed value — the §7.6.4 "constantly true"
/// case, among others.
class ConstantCondition final : public Condition {
public:
    explicit ConstantCondition(bool value) : value_(value) {}

    [[nodiscard]] bool evaluate(double /*simulation_time*/) const override { return value_; }

private:
    bool value_;
};

/// True on [from, until): a logical expression that both rises and falls,
/// which SimulationTimeCondition alone cannot express.
class TimeWindowCondition final : public Condition {
public:
    TimeWindowCondition(double from, double until) : from_(from), until_(until) {}

    [[nodiscard]] bool evaluate(double simulation_time) const override {
        return from_ <= simulation_time && simulation_time < until_;
    }

private:
    double from_;
    double until_;
};

std::shared_ptr<Condition> constant(bool value) {
    return std::make_shared<ConstantCondition>(value);
}

std::shared_ptr<Condition> window(double from, double until) {
    return std::make_shared<TimeWindowCondition>(from, until);
}

std::shared_ptr<Condition> at_least(double at_time) {
    return std::make_shared<SimulationTimeCondition>(at_time);
}

TriggerCondition cond(std::shared_ptr<Condition> expression,
                      ConditionEdge edge = ConditionEdge::None, double delay = 0.0) {
    TriggerCondition condition;
    condition.delay = delay;
    condition.edge = edge;
    condition.expression = std::move(expression);
    return condition;
}

/// Builds the OR-of-ANDs of §7.6.1 from a list of groups.
Trigger trigger_of(std::vector<std::vector<TriggerCondition>> groups) {
    Trigger trigger;
    for (std::vector<TriggerCondition>& conditions : groups) {
        ConditionGroup group;
        group.conditions = std::move(conditions);
        trigger.groups.push_back(std::move(group));
    }
    return trigger;
}

Event make_event(std::string name, std::optional<Trigger> start_trigger, std::string entity_id) {
    Event event;
    event.name = std::move(name);
    event.start_trigger = std::move(start_trigger);
    event.actions.push_back(std::make_shared<SpeedAction>(std::move(entity_id), 10.0));
    return event;
}

/// story/act/group/maneuver wrapper; both act triggers are optional.
Storyboard make_storyboard(std::vector<Event> events,
                           std::optional<Trigger> act_start = std::nullopt,
                           std::optional<Trigger> act_stop = std::nullopt) {
    Maneuver maneuver;
    maneuver.name = "maneuver";
    maneuver.events = std::move(events);
    ManeuverGroup group;
    group.name = "group";
    group.maneuvers.push_back(std::move(maneuver));
    Act act;
    act.name = "act";
    act.start_trigger = std::move(act_start);
    act.stop_trigger = std::move(act_stop);
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

/// Steps the scheduler over `times` and returns the times at which the
/// event fired — one entry per action firing, so an event with a
/// maximumExecutionCount above one contributes one entry per execution.
std::vector<double> firing_times(Scheduler& scheduler, const std::vector<double>& times) {
    std::vector<double> fired;
    for (const double t : times) {
        scheduler.step(t, [&](const scena::ir::Action&) {
            fired.push_back(t);
            return ActionOutcome::Complete;
        });
    }
    return fired;
}

const char* const kEventPath = "story/act/group/maneuver/event";

} // namespace

// ---------------------------------------------------------------------------
// Trigger structure (§7.6.1)
// ---------------------------------------------------------------------------

TEST(TriggerTest, EmptyStartTriggerNeverStartsElement) {
    // §7.6.1: an empty trigger always evaluates to false. That is not the
    // same as having no trigger, which starts the element with its parent.
    Scheduler empty_scheduler;
    const Storyboard with_empty = make_storyboard({make_event("event", Trigger{}, "ego")});
    empty_scheduler.bind(with_empty);
    EXPECT_TRUE(firing_times(empty_scheduler, {0.0, 1.0, 100.0}).empty());
    EXPECT_EQ(*empty_scheduler.element_state(kEventPath), ElementState::Standby);

    Scheduler absent_scheduler;
    const Storyboard with_none = make_storyboard({make_event("event", std::nullopt, "ego")});
    absent_scheduler.bind(with_none);
    EXPECT_EQ(fired_entities(absent_scheduler, 0.0), (std::vector<std::string>{"ego"}));
}

TEST(TriggerTest, EmptyStopTriggerNeverStops) {
    Storyboard storyboard = make_storyboard(
        {make_event("event", trigger_of({{cond(at_least(1.0))}}), "ego")}, std::nullopt, Trigger{});
    storyboard.stop_trigger = Trigger{};

    Scheduler scheduler;
    scheduler.bind(storyboard);
    EXPECT_EQ(firing_times(scheduler, {0.0, 0.5, 1.0}), (std::vector<double>{1.0}));
    EXPECT_FALSE(scheduler.storyboard_complete());
    EXPECT_EQ(*scheduler.element_transition(kEventPath), TransitionKind::End);
}

TEST(TriggerTest, GroupIsConjunction) {
    // One group, two conditions: fires only once both hold (§7.6.1).
    const Storyboard storyboard = make_storyboard(
        {make_event("event", trigger_of({{cond(at_least(1.0)), cond(at_least(2.0))}}), "ego")});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    EXPECT_EQ(firing_times(scheduler, {0.0, 1.0, 1.5, 2.0}), (std::vector<double>{2.0}));
}

TEST(TriggerTest, GroupsAreDisjunction) {
    // Two groups: the earlier one alone is enough (§7.6.1).
    const Storyboard storyboard = make_storyboard(
        {make_event("event", trigger_of({{cond(at_least(5.0))}, {cond(at_least(2.0))}}), "ego")});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    EXPECT_EQ(firing_times(scheduler, {0.0, 1.0, 2.0, 3.0}), (std::vector<double>{2.0}));
}

TEST(TriggerTest, RisingEdgeInGroupMustCoincideWithOtherConditions) {
    // An edge is a one-evaluation-wide pulse: the conjunction only holds if
    // the other conditions hold at that very evaluation (§7.6.1, §7.6.2).
    const Storyboard storyboard = make_storyboard({make_event(
        "event", trigger_of({{cond(at_least(1.0), ConditionEdge::Rising), cond(at_least(2.0))}}),
        "ego")});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    // The rise happens at t = 1.0, when the second condition is still
    // false; by t = 2.0 the pulse is long gone.
    EXPECT_TRUE(firing_times(scheduler, {0.0, 1.0, 2.0, 3.0, 100.0}).empty());
}

// ---------------------------------------------------------------------------
// Condition edges (§7.6.2, §7.6.4)
// ---------------------------------------------------------------------------

TEST(TriggerTest, NoneEdgeCanFireOnFirstEvaluation) {
    // C_N(t_d) = LE(t_d) (§7.6.2.4): no history involved, so the very first
    // evaluation can fire — the §7.6.4 first-check rule applies to edges only.
    const Storyboard storyboard =
        make_storyboard({make_event("event", trigger_of({{cond(constant(true))}}), "ego")});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    EXPECT_EQ(fired_entities(scheduler, 0.0), (std::vector<std::string>{"ego"}));
}

TEST(TriggerTest, RisingEdgeFiresOnExactTransitionStep) {
    // C_R(t_d) = LE(t_d) AND NOT LE(t_{d-1}) (§7.6.2.1).
    const Storyboard storyboard = make_storyboard(
        {make_event("event", trigger_of({{cond(at_least(1.0), ConditionEdge::Rising)}}), "ego")});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    EXPECT_EQ(firing_times(scheduler, {0.0, 0.5, 1.0, 1.5}), (std::vector<double>{1.0}));
}

TEST(TriggerTest, RisingEdgeConstantTrueNeverFires) {
    // §7.6.4: "constantly true logical expressions do NOT trigger a
    // conditionEdge=rising" — the first check is false and there is never a
    // false-to-true transition afterwards.
    const Storyboard storyboard = make_storyboard(
        {make_event("event", trigger_of({{cond(constant(true), ConditionEdge::Rising)}}), "ego")});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    EXPECT_TRUE(firing_times(scheduler, {0.0, 1.0, 2.0, 3.0, 100.0}).empty());
    EXPECT_EQ(*scheduler.element_state(kEventPath), ElementState::Standby);
}

TEST(TriggerTest, FallingEdgeFiresWhenExpressionDrops) {
    // C_F(t_d) = NOT LE(t_d) AND LE(t_{d-1}) (§7.6.2.2).
    const Storyboard storyboard = make_storyboard({make_event(
        "event", trigger_of({{cond(window(0.5, 1.5), ConditionEdge::Falling)}}), "ego")});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    EXPECT_EQ(firing_times(scheduler, {0.0, 0.5, 1.0, 1.5, 2.0}), (std::vector<double>{1.5}));
}

TEST(TriggerTest, RisingOrFallingFiresOnEitherTransition) {
    // C_RoF = C_R OR C_F (§7.6.2.3): the rise fires when the element has
    // been watching since before the window...
    const Storyboard rising_case = make_storyboard({make_event(
        "event", trigger_of({{cond(window(0.5, 1.5), ConditionEdge::RisingOrFalling)}}), "ego")});
    Scheduler rising_scheduler;
    rising_scheduler.bind(rising_case);
    EXPECT_EQ(firing_times(rising_scheduler, {0.0, 0.5, 1.0}), (std::vector<double>{0.5}));

    // ...and the fall fires when the act only starts inside the window, so
    // the rise happened before the condition was ever checked.
    const Storyboard falling_case = make_storyboard(
        {make_event("event", trigger_of({{cond(window(0.5, 1.5), ConditionEdge::RisingOrFalling)}}),
                    "ego")},
        trigger_of({{cond(at_least(1.0))}}));
    Scheduler falling_scheduler;
    falling_scheduler.bind(falling_case);
    EXPECT_EQ(firing_times(falling_scheduler, {0.0, 1.0, 1.5, 2.0}), (std::vector<double>{1.5}));
}

TEST(TriggerTest, FirstEvaluationOfEdgeConditionIsFalse) {
    // §7.6.4: the first check of an edge condition is always false, and the
    // first check happens when the enclosing element enters standbyState —
    // here at t = 2.0, when the act starts. The expression has been true
    // since t = 1.0, so no rise is ever observed.
    const Storyboard storyboard = make_storyboard(
        {make_event("event", trigger_of({{cond(at_least(1.0), ConditionEdge::Rising)}}), "ego")},
        trigger_of({{cond(at_least(2.0))}}));
    Scheduler scheduler;
    scheduler.bind(storyboard);
    EXPECT_TRUE(firing_times(scheduler, {0.0, 1.0, 2.0, 2.5, 100.0}).empty());
    EXPECT_EQ(*scheduler.element_state("story/act"), ElementState::Running);
    EXPECT_EQ(*scheduler.element_state(kEventPath), ElementState::Standby);
}

TEST(TriggerTest, EdgeDetectedAfterElementEntersStandby) {
    // Contrast with the case above: a rise that happens after the enclosing
    // act started is detected normally.
    const Storyboard storyboard = make_storyboard(
        {make_event("event", trigger_of({{cond(at_least(2.5), ConditionEdge::Rising)}}), "ego")},
        trigger_of({{cond(at_least(2.0))}}));
    Scheduler scheduler;
    scheduler.bind(storyboard);
    EXPECT_EQ(firing_times(scheduler, {0.0, 1.0, 2.0, 2.5, 3.0}), (std::vector<double>{2.5}));
}

// ---------------------------------------------------------------------------
// Condition delays (§7.6.3, §7.6.4)
// ---------------------------------------------------------------------------

TEST(TriggerTest, DelayShiftsFiringByDelta) {
    // C_D(t_d) = C(t_d - delta) (§7.6.3).
    const Storyboard storyboard = make_storyboard({make_event(
        "event", trigger_of({{cond(at_least(1.0), ConditionEdge::None, 0.5)}}), "ego")});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    EXPECT_EQ(firing_times(scheduler, {0.0, 0.5, 1.0, 1.5, 2.0}), (std::vector<double>{1.5}));
}

TEST(TriggerTest, DelayedConditionFalseWhileTimeLessThanDelta) {
    // §7.6.4: while t_d < delta the delayed condition is false, even though
    // its logical expression holds from the very first evaluation.
    const Storyboard storyboard = make_storyboard({make_event(
        "event", trigger_of({{cond(constant(true), ConditionEdge::None, 1.0)}}), "ego")});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    EXPECT_EQ(firing_times(scheduler, {0.0, 0.5, 1.0}), (std::vector<double>{1.0}));
}

TEST(TriggerTest, DelayLookupUsesMostRecentSampleAtOrBefore) {
    // The host chooses the step times, so t_d - delta usually falls between
    // two evaluations. The condition is sampled and held: the most recent
    // evaluation at or before t_d - delta is the answer.
    const Storyboard storyboard = make_storyboard({make_event(
        "event", trigger_of({{cond(at_least(0.75), ConditionEdge::None, 0.5)}}), "ego")});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    // At t = 1.2 the lookup lands on t = 0.7, held from the evaluation at
    // t = 0.4 (false). At t = 1.6 it lands on t = 1.1, held from t = 0.8
    // (true).
    EXPECT_EQ(firing_times(scheduler, {0.0, 0.4, 0.8, 1.2, 1.6}), (std::vector<double>{1.6}));
}

TEST(TriggerTest, ExactBoundarySampleIsUsed) {
    // When an evaluation exists at exactly t_d - delta it is the one used;
    // the comparison is exact, with no tolerance.
    const Storyboard storyboard = make_storyboard({make_event(
        "event", trigger_of({{cond(at_least(0.5), ConditionEdge::None, 0.5)}}), "ego")});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    EXPECT_EQ(firing_times(scheduler, {0.0, 0.5, 1.0, 1.5}), (std::vector<double>{1.0}));
}

TEST(TriggerTest, DelayedRisingEdgeFiresExactlyOnce) {
    // The delay applies to the post-edge value, so the one-evaluation-wide
    // pulse is shifted, not widened.
    const Storyboard storyboard = make_storyboard({make_event(
        "event", trigger_of({{cond(at_least(1.0), ConditionEdge::Rising, 0.5)}}), "ego")});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    EXPECT_EQ(firing_times(scheduler, {0.0, 0.5, 1.0, 1.5, 2.0, 2.5}), (std::vector<double>{1.5}));
}

TEST(TriggerTest, DelayedEventsFiringSameStepFireInDocumentOrder) {
    // Three different routes to the same discrete time: a delayed
    // condition, an undelayed one, and a longer delay on an earlier
    // condition. Document order decides, exactly as for undelayed events.
    const Storyboard storyboard = make_storyboard({
        make_event("first", trigger_of({{cond(at_least(1.0), ConditionEdge::None, 0.5)}}), "a"),
        make_event("second", trigger_of({{cond(at_least(1.5))}}), "b"),
        make_event("third", trigger_of({{cond(at_least(0.5), ConditionEdge::None, 1.0)}}), "c"),
    });
    Scheduler scheduler;
    scheduler.bind(storyboard);
    for (const double t : {0.0, 0.5, 1.0}) {
        EXPECT_TRUE(fired_entities(scheduler, t).empty()) << t;
    }
    EXPECT_EQ(fired_entities(scheduler, 1.5), (std::vector<std::string>{"a", "b", "c"}));
}

TEST(TriggerTest, RepeatedEvaluationAtSameTimeUsesLatestSample) {
    // A host may step with dt = 0. Each call is a new discrete evaluation
    // t_d, so the second one sees the first as t_{d-1}: the rising pulse
    // produced at t = 1.0 is immediately followed by a false at the same
    // time, and the delayed lookup — which takes the most recent sample at
    // or before its target — reads that false.
    const Storyboard storyboard = make_storyboard({make_event(
        "event", trigger_of({{cond(at_least(1.0), ConditionEdge::Rising, 1.0)}}), "ego")});
    Scheduler repeated;
    repeated.bind(storyboard);
    EXPECT_TRUE(firing_times(repeated, {0.0, 1.0, 1.0, 2.0, 3.0}).empty());

    // Without the repeated evaluation the same trigger fires at t = 2.0.
    Scheduler single;
    single.bind(storyboard);
    EXPECT_EQ(firing_times(single, {0.0, 1.0, 2.0, 3.0}), (std::vector<double>{2.0}));
}

// ---------------------------------------------------------------------------
// Stop triggers (§7.6.1.2)
// ---------------------------------------------------------------------------

TEST(TriggerTest, ActStopTriggerStopsItsSubtreeOnly) {
    // Stop triggers are inherited downwards, never sideways or upwards: a
    // stopped act takes its own subtree to completeState and leaves the
    // rest of the storyboard running.
    Storyboard storyboard =
        make_storyboard({make_event("event", trigger_of({{cond(at_least(3.0))}}), "stopped")},
                        std::nullopt, trigger_of({{cond(at_least(2.0))}}));
    Storyboard other =
        make_storyboard({make_event("event", trigger_of({{cond(at_least(3.0))}}), "surviving")});
    other.stories[0].name = "other-story";
    storyboard.stories.push_back(std::move(other.stories[0]));

    Scheduler scheduler;
    scheduler.bind(storyboard);
    EXPECT_TRUE(firing_times(scheduler, {0.0, 1.0, 2.0}).empty());
    EXPECT_EQ(*scheduler.element_state("story/act"), ElementState::Complete);
    EXPECT_EQ(*scheduler.element_transition("story/act"), TransitionKind::Stop);
    EXPECT_EQ(*scheduler.element_transition(kEventPath), TransitionKind::Stop);
    // The other story is untouched, and the storyboard itself keeps running.
    EXPECT_EQ(*scheduler.element_state("other-story/act"), ElementState::Running);
    EXPECT_FALSE(scheduler.storyboard_complete());

    // Only the surviving story's event still fires.
    EXPECT_EQ(fired_entities(scheduler, 3.0), (std::vector<std::string>{"surviving"}));
}

TEST(TriggerTest, ActStopTriggerFromStandbyCompletesWithoutRunning) {
    // §7.6.1.2: a stop trigger takes an element out of standbyState too —
    // the act here never starts.
    const Storyboard storyboard =
        make_storyboard({make_event("event", std::nullopt, "ego")},
                        trigger_of({{cond(at_least(5.0))}}), trigger_of({{cond(at_least(1.0))}}));
    Scheduler scheduler;
    scheduler.bind(storyboard);
    EXPECT_TRUE(firing_times(scheduler, {0.0, 1.0, 5.0, 10.0}).empty());
    EXPECT_EQ(*scheduler.element_state("story/act"), ElementState::Complete);
    EXPECT_EQ(*scheduler.element_transition("story/act"), TransitionKind::Stop);
}

TEST(TriggerTest, StopWinsOverStartInSameStep) {
    // Both triggers hold at t = 1.0. Stop is checked first at every level,
    // so the act is stopped rather than started and its event never fires.
    const Storyboard storyboard =
        make_storyboard({make_event("event", std::nullopt, "ego")},
                        trigger_of({{cond(at_least(1.0))}}), trigger_of({{cond(at_least(1.0))}}));
    Scheduler scheduler;
    scheduler.bind(storyboard);
    EXPECT_TRUE(firing_times(scheduler, {0.0, 1.0, 2.0}).empty());
    EXPECT_EQ(*scheduler.element_transition("story/act"), TransitionKind::Stop);
}

TEST(TriggerTest, StopTriggerSupportsEdgeAndDelay) {
    // Stop triggers are ordinary triggers: same edge and delay machinery.
    const Storyboard storyboard = make_storyboard(
        {make_event("event", trigger_of({{cond(at_least(3.0))}}), "ego")}, std::nullopt,
        trigger_of({{cond(at_least(1.0), ConditionEdge::Rising, 0.5)}}));
    Scheduler scheduler;
    scheduler.bind(storyboard);
    for (const double t : {0.0, 0.5, 1.0}) {
        scheduler.step(t, [](const scena::ir::Action&) { return ActionOutcome::Complete; });
        EXPECT_EQ(*scheduler.element_state("story/act"), ElementState::Running) << t;
    }
    // The rise at t = 1.0 arrives delayed by 0.5.
    scheduler.step(1.5, [](const scena::ir::Action&) { return ActionOutcome::Complete; });
    EXPECT_EQ(*scheduler.element_transition("story/act"), TransitionKind::Stop);
}

TEST(TriggerTest, RebindClearsTriggerState) {
    // bind() rebuilds every condition's history, so a rising edge already
    // consumed by a previous run is detected again.
    const Storyboard storyboard = make_storyboard(
        {make_event("event", trigger_of({{cond(at_least(1.0), ConditionEdge::Rising)}}), "ego")});
    Scheduler scheduler;
    scheduler.bind(storyboard);
    ASSERT_EQ(firing_times(scheduler, {0.0, 1.0, 2.0}), (std::vector<double>{1.0}));

    scheduler.bind(storyboard);
    EXPECT_EQ(firing_times(scheduler, {0.0, 1.0, 2.0}), (std::vector<double>{1.0}));

    // And a fresh binding starts its history from scratch: an expression
    // that already holds at the first evaluation shows no rise (§7.6.4).
    scheduler.bind(storyboard);
    EXPECT_TRUE(firing_times(scheduler, {2.0, 3.0}).empty());
}
