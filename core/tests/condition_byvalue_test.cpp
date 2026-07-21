// SPDX-License-Identifier: MIT
// By-value conditions and their shared building blocks, per ASAM
// OpenSCENARIO XML 1.4.0: the Rule comparator (§ enum Rule), the dateTime
// value type (preface "Data types"), and the SimulationTime, Parameter,
// Variable, UserDefinedValue, TimeOfDay and StoryboardElementState
// conditions (ByValueCondition group).
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "scena/engine.h"
#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/date_time.h"
#include "scena/ir/rule.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/trigger.h"
#include "scena/status.h"

using scena::Engine;
using scena::Status;
using scena::ir::compare;
using scena::ir::compare_values;
using scena::ir::Condition;
using scena::ir::ConditionEdge;
using scena::ir::DateTime;
using scena::ir::parse_scalar;
using scena::ir::Rule;
using scena::ir::SimulationTimeCondition;
using scena::ir::TriggerCondition;
using scena::runtime::ActionOutcome;
using scena::runtime::ElementState;
using scena::runtime::Scheduler;

namespace {

constexpr double kEpoch2000 = 946684800.0; // 2000-01-01T00:00:00Z, seconds.

// --- Scheduler-driven helpers (trigger_test idioms) ------------------------

TriggerCondition cond(std::shared_ptr<Condition> expression,
                      ConditionEdge edge = ConditionEdge::None, double delay = 0.0) {
    TriggerCondition condition;
    condition.delay = delay;
    condition.edge = edge;
    condition.expression = std::move(expression);
    return condition;
}

scena::ir::Trigger trigger_of(std::vector<std::vector<TriggerCondition>> groups) {
    scena::ir::Trigger trigger;
    for (std::vector<TriggerCondition>& conditions : groups) {
        scena::ir::ConditionGroup group;
        group.conditions = std::move(conditions);
        trigger.groups.push_back(std::move(group));
    }
    return trigger;
}

scena::ir::Event make_event(std::string name, std::optional<scena::ir::Trigger> start_trigger,
                            std::string entity_id) {
    scena::ir::Event event;
    event.name = std::move(name);
    event.start_trigger = std::move(start_trigger);
    event.actions.push_back(std::make_shared<scena::ir::SpeedAction>(std::move(entity_id), 10.0));
    return event;
}

/// story/act/group/maneuver wrapper (no act triggers).
scena::ir::Storyboard make_storyboard(std::vector<scena::ir::Event> events) {
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
    scena::ir::Storyboard storyboard;
    storyboard.stories.push_back(std::move(story));
    return storyboard;
}

/// Steps the scheduler over `times`, returning the times at which the single
/// "event" fired.
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

std::shared_ptr<Condition> sim_time(double value, Rule rule = Rule::GreaterOrEqual) {
    return std::make_shared<SimulationTimeCondition>(value, rule);
}

// --- Engine-driven helpers -------------------------------------------------

/// A one-entity ("ego", engine-controlled) scenario whose single event fires
/// when `expression` holds, driving ego to 10 m/s — a firing shows up as a
/// non-zero ego speed. `edge`/`delay` carry into the wrapping trigger.
scena::ir::Scenario make_scenario(std::shared_ptr<Condition> expression,
                                  ConditionEdge edge = ConditionEdge::None, double delay = 0.0) {
    scena::ir::Scenario scenario;
    scenario.name = "byvalue";
    scena::ir::Entity ego;
    ego.id = "ego";
    ego.name = "ego";
    scenario.entities.push_back(std::move(ego));
    scena::ir::Event event =
        make_event("event", scena::ir::make_trigger(std::move(expression), edge, delay), "ego");
    scenario.storyboard = make_storyboard({std::move(event)});
    return scenario;
}

/// True once ego has been driven off its initial zero speed — i.e. the event
/// fired at some point.
bool ego_fired(const Engine& engine) {
    const auto state = engine.state("ego");
    return state.has_value() && state->speed != 0.0;
}

} // namespace

// ---------------------------------------------------------------------------
// Rule comparator (§ enum Rule)
// ---------------------------------------------------------------------------

TEST(RuleComparatorTest, NumericComparisonsCoverAllSixRules) {
    EXPECT_TRUE(compare(2.0, Rule::EqualTo, 2.0));
    EXPECT_FALSE(compare(2.0, Rule::EqualTo, 3.0));

    EXPECT_TRUE(compare(3.0, Rule::GreaterThan, 2.0));
    EXPECT_FALSE(compare(2.0, Rule::GreaterThan, 2.0));

    EXPECT_TRUE(compare(1.0, Rule::LessThan, 2.0));
    EXPECT_FALSE(compare(2.0, Rule::LessThan, 2.0));

    EXPECT_TRUE(compare(2.0, Rule::GreaterOrEqual, 2.0));
    EXPECT_TRUE(compare(3.0, Rule::GreaterOrEqual, 2.0));
    EXPECT_FALSE(compare(1.0, Rule::GreaterOrEqual, 2.0));

    EXPECT_TRUE(compare(2.0, Rule::LessOrEqual, 2.0));
    EXPECT_TRUE(compare(1.0, Rule::LessOrEqual, 2.0));
    EXPECT_FALSE(compare(3.0, Rule::LessOrEqual, 2.0));

    EXPECT_TRUE(compare(2.0, Rule::NotEqualTo, 3.0));
    EXPECT_FALSE(compare(2.0, Rule::NotEqualTo, 2.0));
}

TEST(RuleComparatorTest, EqualToOnDoublesIsExact) {
    // No tolerance: 0.1 + 0.2 is not 0.3 in IEEE-754, and the comparator must
    // reflect that rather than smoothing it over with an epsilon.
    EXPECT_FALSE(compare(0.1 + 0.2, Rule::EqualTo, 0.3));
    EXPECT_TRUE(compare(0.3, Rule::EqualTo, 0.3));
    // The orientation of the ordering rules is stable regardless of magnitude.
    EXPECT_TRUE(compare(1e308, Rule::GreaterThan, 1.0));
}

TEST(RuleComparatorTest, StringEqualityAndInequality) {
    // Neither operand is a scalar, so only equality/inequality apply, byte
    // for byte (ParameterCondition scalar-convertibility clause).
    EXPECT_TRUE(compare_values("left", Rule::EqualTo, "left"));
    EXPECT_FALSE(compare_values("left", Rule::EqualTo, "right"));
    EXPECT_TRUE(compare_values("left", Rule::NotEqualTo, "right"));
    EXPECT_FALSE(compare_values("left", Rule::NotEqualTo, "left"));
    // No boolean coercion in this phase: "true" and "1" are distinct strings.
    EXPECT_FALSE(compare_values("true", Rule::EqualTo, "1"));
}

TEST(RuleComparatorTest, NumericStringsCompareNumerically) {
    // Both sides parse: the comparison is numeric, so "5" == "5.0" and
    // ordering is meaningful. Left operand is the stored value.
    EXPECT_TRUE(compare_values("5", Rule::EqualTo, "5.0"));
    EXPECT_TRUE(compare_values("16.667", Rule::GreaterThan, "5"));
    EXPECT_TRUE(compare_values("+5", Rule::EqualTo, "5"));
    EXPECT_TRUE(compare_values("5", Rule::LessOrEqual, "5"));
}

TEST(RuleComparatorTest, OrderingOnNonNumericStringsIsFalse) {
    // "Less and greater operators will only be supported if the value ... can
    // unambiguously be converted into a scalar" — otherwise every ordering
    // rule is false, even when the strings differ.
    EXPECT_FALSE(compare_values("apple", Rule::LessThan, "banana"));
    EXPECT_FALSE(compare_values("apple", Rule::GreaterThan, "banana"));
    EXPECT_FALSE(compare_values("apple", Rule::GreaterOrEqual, "apple"));
    EXPECT_FALSE(compare_values("apple", Rule::LessOrEqual, "apple"));
    // One numeric, one not: still not both-scalar, so ordering is false.
    EXPECT_FALSE(compare_values("5", Rule::GreaterThan, "five"));
}

TEST(RuleComparatorTest, PartialAndLocaleLikeTokensAreNotScalars) {
    EXPECT_FALSE(parse_scalar("").has_value());
    EXPECT_FALSE(parse_scalar("1,5").has_value());  // locale decimal comma
    EXPECT_FALSE(parse_scalar("1.5x").has_value()); // trailing remainder
    EXPECT_FALSE(parse_scalar(" 1.5").has_value()); // leading whitespace
    EXPECT_FALSE(parse_scalar("+").has_value());
    EXPECT_FALSE(parse_scalar("+-5").has_value());
    ASSERT_TRUE(parse_scalar("16.667").has_value());
    EXPECT_DOUBLE_EQ(*parse_scalar("16.667"), 16.667);
    ASSERT_TRUE(parse_scalar("+5").has_value());
    EXPECT_DOUBLE_EQ(*parse_scalar("+5"), 5.0);
    ASSERT_TRUE(parse_scalar("-2.5").has_value());
    EXPECT_DOUBLE_EQ(*parse_scalar("-2.5"), -2.5);
}

TEST(RuleComparatorTest, NanOperandsFollowIeeeSemantics) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(compare(nan, Rule::EqualTo, nan));
    EXPECT_TRUE(compare(nan, Rule::NotEqualTo, nan));
    EXPECT_FALSE(compare(nan, Rule::GreaterThan, 0.0));
    EXPECT_FALSE(compare(nan, Rule::LessThan, 0.0));
    EXPECT_FALSE(compare(nan, Rule::GreaterOrEqual, 0.0));
    EXPECT_FALSE(compare(nan, Rule::LessOrEqual, 0.0));
    EXPECT_FALSE(compare(0.0, Rule::LessThan, nan));
}

// ---------------------------------------------------------------------------
// DateTime value type (preface "Data types", Table 5)
// ---------------------------------------------------------------------------

TEST(DateTimeTest, EpochAndFractionAreExact) {
    EXPECT_DOUBLE_EQ(DateTime{}.to_epoch_seconds(), 0.0); // 1970-01-01T00:00:00Z
    const DateTime y2k{2000, 1, 1, 0, 0, 0, 0, 0};
    EXPECT_DOUBLE_EQ(y2k.to_epoch_seconds(), kEpoch2000);
    const DateTime half_second{1970, 1, 1, 0, 0, 0, 500, 0};
    EXPECT_DOUBLE_EQ(half_second.to_epoch_seconds(), 0.5);
}

TEST(DateTimeTest, CanonicalizationHandlesLeapYearsAndOffsets) {
    // 2000 is a leap year (divisible by 400): Feb 29 exists and March 1 is
    // one day after it.
    const DateTime leap_day{2000, 2, 29, 0, 0, 0, 0, 0};
    const DateTime march_first{2000, 3, 1, 0, 0, 0, 0, 0};
    EXPECT_DOUBLE_EQ(march_first.to_epoch_seconds() - leap_day.to_epoch_seconds(), 86400.0);
    EXPECT_DOUBLE_EQ(march_first.to_epoch_seconds(), kEpoch2000 + 60.0 * 86400.0);

    // A +01:00 zone is one hour ahead of UTC, so the same wall reading is an
    // hour earlier in epoch terms.
    const DateTime utc_one_am{1970, 1, 1, 1, 0, 0, 0, 0};
    const DateTime cet_one_am{1970, 1, 1, 1, 0, 0, 0, 60};
    EXPECT_DOUBLE_EQ(utc_one_am.to_epoch_seconds(), 3600.0);
    EXPECT_DOUBLE_EQ(cet_one_am.to_epoch_seconds(), 0.0);
    // A western offset moves the instant later.
    const DateTime west{1970, 1, 1, 0, 0, 0, 0, -300}; // -05:00
    EXPECT_DOUBLE_EQ(west.to_epoch_seconds(), 5.0 * 3600.0);
}

TEST(DateTimeTest, ValidatesFieldRangesAndDayInMonth) {
    EXPECT_TRUE((DateTime{2000, 2, 29, 23, 59, 59, 999, 0}.valid()));
    EXPECT_TRUE((DateTime{}.valid()));
    // 2001 is not a leap year, so Feb 29 does not exist.
    EXPECT_FALSE((DateTime{2001, 2, 29, 0, 0, 0, 0, 0}.valid()));
    EXPECT_FALSE((DateTime{2000, 4, 31, 0, 0, 0, 0, 0}.valid()));      // April has 30
    EXPECT_FALSE((DateTime{2000, 0, 1, 0, 0, 0, 0, 0}.valid()));       // month underflow
    EXPECT_FALSE((DateTime{2000, 13, 1, 0, 0, 0, 0, 0}.valid()));      // month overflow
    EXPECT_FALSE((DateTime{2000, 1, 1, 24, 0, 0, 0, 0}.valid()));      // hour overflow
    EXPECT_FALSE((DateTime{2000, 1, 1, 0, 60, 0, 0, 0}.valid()));      // minute overflow
    EXPECT_FALSE((DateTime{2000, 1, 1, 0, 0, 60, 0, 0}.valid()));      // second overflow
    EXPECT_FALSE((DateTime{2000, 1, 1, 0, 0, 0, 1000, 0}.valid()));    // ms overflow
    EXPECT_FALSE((DateTime{2000, 1, 1, 0, 0, 0, 0, 15 * 60}.valid())); // offset too wide
}

// ---------------------------------------------------------------------------
// SimulationTimeCondition (§ SimulationTimeCondition, §8.4.7)
// ---------------------------------------------------------------------------

TEST(SimulationTimeConditionTest, SimulationTimeStartsWithStoryboardRunning) {
    // §8.4.7: the storyboard enters runningState when the simulation starts,
    // and that marks t = 0. An event guarded by SimulationTime >= 0 therefore
    // fires in the init evaluation, before any host step, and the storyboard
    // is already running with time at zero.
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(sim_time(0.0, Rule::GreaterOrEqual))), Status::Ok);
    EXPECT_EQ(engine.time(), 0.0);
    EXPECT_EQ(*engine.storyboard_element_state(""), ElementState::Running);
    EXPECT_TRUE(ego_fired(engine));
}

TEST(SimulationTimeConditionTest, GreaterOrEqualFiresExactlyAtBoundary) {
    // The default rule is greaterOrEqual: the condition holds from the value
    // on, so it fires at exactly t = value and not before.
    Scheduler scheduler;
    const scena::ir::Storyboard storyboard =
        make_storyboard({make_event("event", trigger_of({{cond(sim_time(1.0))}}), "ego")});
    scheduler.bind(storyboard);
    EXPECT_EQ(firing_times(scheduler, {0.0, 0.5, 1.0, 1.5}), (std::vector<double>{1.0}));
}

TEST(SimulationTimeConditionTest, RuleFamilyOnSimulationTime) {
    const auto first_fire = [](Rule rule, const std::vector<double>& times) {
        Scheduler scheduler;
        const scena::ir::Storyboard storyboard = make_storyboard(
            {make_event("event", trigger_of({{cond(sim_time(1.0, rule))}}), "ego")});
        scheduler.bind(storyboard);
        return firing_times(scheduler, times);
    };
    EXPECT_EQ(first_fire(Rule::EqualTo, {0.0, 0.5, 1.0, 2.0}), (std::vector<double>{1.0}));
    EXPECT_EQ(first_fire(Rule::NotEqualTo, {0.0, 1.0}), (std::vector<double>{0.0}));
    EXPECT_EQ(first_fire(Rule::GreaterThan, {0.0, 1.0, 1.5}), (std::vector<double>{1.5}));
    EXPECT_EQ(first_fire(Rule::GreaterOrEqual, {0.0, 0.5, 1.0}), (std::vector<double>{1.0}));
    EXPECT_EQ(first_fire(Rule::LessThan, {0.0, 0.5}), (std::vector<double>{0.0}));
    EXPECT_EQ(first_fire(Rule::LessOrEqual, {0.0, 0.5}), (std::vector<double>{0.0}));
}

TEST(SimulationTimeConditionTest, SimulationTimeWithRisingEdgeAndDelay) {
    // A SimulationTimeCondition is an ordinary logical expression: it composes
    // with the edge and delay machinery. The rise at t = 1.0 arrives shifted
    // by the 0.5 s delay, firing exactly once at t = 1.5.
    Scheduler scheduler;
    const scena::ir::Storyboard storyboard = make_storyboard({make_event(
        "event", trigger_of({{cond(sim_time(1.0), ConditionEdge::Rising, 0.5)}}), "ego")});
    scheduler.bind(storyboard);
    EXPECT_EQ(firing_times(scheduler, {0.0, 0.5, 1.0, 1.5, 2.0, 2.5}), (std::vector<double>{1.5}));
}
