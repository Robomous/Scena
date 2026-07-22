// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
// By-value conditions and their shared building blocks, per ASAM
// OpenSCENARIO XML 1.4.0: the Rule comparator (§ enum Rule), the dateTime
// value type (preface "Data types"), and the SimulationTime, Parameter,
// Variable, UserDefinedValue, TimeOfDay and StoryboardElementState
// conditions (ByValueCondition group).
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "scena/engine.h"
#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/date_time.h"
#include "scena/ir/environment.h"
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
using scena::ir::StoryboardElementState;
using scena::ir::StoryboardElementType;
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

std::shared_ptr<Condition> parameter(std::string ref, Rule rule, std::string value) {
    return std::make_shared<scena::ir::ParameterCondition>(std::move(ref), rule, std::move(value));
}

std::shared_ptr<Condition> variable(std::string ref, Rule rule, std::string value) {
    return std::make_shared<scena::ir::VariableCondition>(std::move(ref), rule, std::move(value));
}

std::shared_ptr<Condition> user_value(std::string name, Rule rule, std::string value) {
    return std::make_shared<scena::ir::UserDefinedValueCondition>(std::move(name), rule,
                                                                  std::move(value));
}

std::shared_ptr<Condition> time_of_day(DateTime date_time, Rule rule) {
    return std::make_shared<scena::ir::TimeOfDayCondition>(date_time, rule);
}

std::shared_ptr<Condition> element_state(StoryboardElementType type, std::string ref,
                                         StoryboardElementState state) {
    return std::make_shared<scena::ir::StoryboardElementStateCondition>(type, std::move(ref),
                                                                        state);
}

/// An event whose single action targets an entity of its own name, so a firing
/// transcript reads as a list of event names. Optional priority/count for the
/// skip cases.
scena::ir::Event named_event(std::string name, std::optional<scena::ir::Trigger> trigger,
                             scena::ir::EventPriority priority = scena::ir::EventPriority::Parallel,
                             int maximum_execution_count = 1) {
    scena::ir::Event event;
    event.name = name;
    event.start_trigger = std::move(trigger);
    event.priority = priority;
    event.maximum_execution_count = maximum_execution_count;
    event.actions.push_back(std::make_shared<scena::ir::SpeedAction>(std::move(name), 10.0));
    return event;
}

/// Fire callback that records firings and reports Ongoing for a chosen set of
/// entity ids (§7.5.3 never-ending actions) so an event stays in runningState
/// long enough to be observed. The set is ordered — no fixture-introduced
/// nondeterminism.
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

/// One step; returns the event names that fired during it.
std::vector<std::string> step_events(Scheduler& scheduler, Recorder& recorder, double t) {
    recorder.clear();
    scheduler.step(t, [&recorder](const scena::ir::Action& action) { return recorder(action); });
    return recorder.fired();
}

/// The times at which `event` fired while stepping over `times`.
std::vector<double> observer_firings(Scheduler& scheduler, Recorder& recorder,
                                     const std::string& event, const std::vector<double>& times) {
    std::vector<double> fires;
    for (const double t : times) {
        for (const std::string& fired : step_events(scheduler, recorder, t)) {
            if (fired == event) {
                fires.push_back(t);
            }
        }
    }
    return fires;
}

/// Two maneuvers under one group (the scope boundary priority must not cross,
/// §7.3.3) — also two distinct enclosing scopes for name resolution.
scena::ir::Storyboard make_two_maneuver_storyboard(std::string first_name,
                                                   std::vector<scena::ir::Event> first,
                                                   std::string second_name,
                                                   std::vector<scena::ir::Event> second) {
    scena::ir::Maneuver maneuver_a;
    maneuver_a.name = std::move(first_name);
    maneuver_a.events = std::move(first);
    scena::ir::Maneuver maneuver_b;
    maneuver_b.name = std::move(second_name);
    maneuver_b.events = std::move(second);
    scena::ir::ManeuverGroup group;
    group.name = "group";
    group.maneuvers.push_back(std::move(maneuver_a));
    group.maneuvers.push_back(std::move(maneuver_b));
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

/// Number of Warning diagnostics anchored to `path`.
int warning_count(const Engine& engine, const std::string& path) {
    int count = 0;
    for (const scena::Diagnostic& diagnostic : engine.diagnostics()) {
        if (diagnostic.severity == scena::Severity::Warning && diagnostic.path == path) {
            ++count;
        }
    }
    return count;
}

/// True when an Error diagnostic with `code` and `rule_id` was reported.
bool has_error(const Engine& engine, Status code, const std::string& rule_id) {
    for (const scena::Diagnostic& diagnostic : engine.diagnostics()) {
        if (diagnostic.severity == scena::Severity::Error && diagnostic.code == code &&
            diagnostic.rule_id == rule_id) {
            return true;
        }
    }
    return false;
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

/// Wraps `storyboard` in a scenario declaring one entity per id, so the engine
/// accepts the named_event action targets.
scena::ir::Scenario engine_scenario(scena::ir::Storyboard storyboard,
                                    std::vector<std::string> entity_ids) {
    scena::ir::Scenario scenario;
    scenario.name = "element-state";
    for (std::string& id : entity_ids) {
        scena::ir::Entity entity;
        entity.id = id;
        entity.name = id;
        scenario.entities.push_back(std::move(entity));
    }
    scenario.storyboard = std::move(storyboard);
    return scenario;
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

// ---------------------------------------------------------------------------
// ParameterCondition (§ ParameterCondition, §9.1)
// ---------------------------------------------------------------------------

TEST(ParameterConditionTest, ParameterEqualToMatchesScenarioValue) {
    scena::ir::Scenario matching = make_scenario(parameter("speedLimit", Rule::EqualTo, "30"));
    matching.parameters["speedLimit"] = "30";
    Engine hit;
    ASSERT_EQ(hit.init(std::move(matching)), Status::Ok);
    EXPECT_TRUE(ego_fired(hit)); // 30 == 30, fires at t = 0

    scena::ir::Scenario mismatching = make_scenario(parameter("speedLimit", Rule::EqualTo, "30"));
    mismatching.parameters["speedLimit"] = "50";
    Engine miss;
    ASSERT_EQ(miss.init(std::move(mismatching)), Status::Ok);
    EXPECT_FALSE(ego_fired(miss));
}

TEST(ParameterConditionTest, ParameterNumericOrderingWhenConvertible) {
    scena::ir::Scenario over = make_scenario(parameter("v", Rule::GreaterThan, "10"));
    over.parameters["v"] = "16.667"; // unambiguously scalar
    Engine above;
    ASSERT_EQ(above.init(std::move(over)), Status::Ok);
    EXPECT_TRUE(ego_fired(above));

    scena::ir::Scenario under = make_scenario(parameter("v", Rule::GreaterThan, "10"));
    under.parameters["v"] = "5";
    Engine below;
    ASSERT_EQ(below.init(std::move(under)), Status::Ok);
    EXPECT_FALSE(ego_fired(below));
}

TEST(ParameterConditionTest, ParameterConditionIsConstantAcrossTheRun) {
    // §9.1: parameters cannot change at runtime, so a condition that does not
    // hold at t = 0 never holds — there is no setter to make it true.
    scena::ir::Scenario scenario = make_scenario(parameter("mode", Rule::EqualTo, "go"));
    scenario.parameters["mode"] = "stop";
    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    for (const double dt : {1.0, 1.0, 1.0, 1.0}) {
        ASSERT_EQ(engine.step(dt), Status::Ok);
    }
    EXPECT_FALSE(ego_fired(engine));
}

TEST(ParameterConditionTest, DanglingParameterRefFailsInitWithSemanticError) {
    scena::ir::Scenario scenario = make_scenario(parameter("missing", Rule::EqualTo, "1"));
    Engine engine;
    EXPECT_EQ(engine.init(std::move(scenario)), Status::SemanticError);
    // The standard names no checker rule for parameter resolvability.
    EXPECT_TRUE(has_error(engine, Status::SemanticError, ""));
}

TEST(ParameterConditionTest, OrderingRuleOnNonNumericParameterWarnsAtInitAndIsFalse) {
    scena::ir::Scenario scenario = make_scenario(parameter("mode", Rule::GreaterThan, "5"));
    scenario.parameters["mode"] = "fast"; // not scalar-convertible
    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok); // warning, not error
    EXPECT_GE(warning_count(engine, "story/act/group/maneuver/event/startTrigger/group[0]/"
                                    "condition[0]"),
              1);
    EXPECT_FALSE(ego_fired(engine));
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_FALSE(ego_fired(engine));
}

// ---------------------------------------------------------------------------
// VariableCondition (§ VariableCondition, §6.12)
// ---------------------------------------------------------------------------

TEST(VariableConditionTest, VariableInitializationValueVisibleAtTimeZero) {
    // §6.12: variables take their initialization value at load time, so a
    // VariableCondition sees it in the t = 0 evaluation.
    scena::ir::Scenario scenario = make_scenario(variable("trigger", Rule::EqualTo, "on"));
    scenario.variables["trigger"] = "on";
    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    EXPECT_TRUE(ego_fired(engine));
}

TEST(VariableConditionTest, SetVariableDuringRunFiresRisingEdgeTrigger) {
    scena::ir::Scenario scenario =
        make_scenario(variable("go", Rule::EqualTo, "1"), ConditionEdge::Rising);
    scenario.variables["go"] = "0";
    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    EXPECT_FALSE(ego_fired(engine));
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_FALSE(ego_fired(engine));
    ASSERT_EQ(engine.set_variable("go", "1"), Status::Ok);
    ASSERT_EQ(engine.step(2.0), Status::Ok); // false -> true: rising edge fires
    EXPECT_TRUE(ego_fired(engine));
}

TEST(VariableConditionTest, SetVariableOnUndeclaredNameReturnsUnknownName) {
    scena::ir::Scenario scenario = make_scenario(sim_time(100.0));
    scenario.variables["declared"] = "0";
    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    EXPECT_EQ(engine.set_variable("declared", "1"), Status::Ok);
    EXPECT_EQ(engine.variable("declared"), std::optional<std::string>("1"));
    EXPECT_EQ(engine.set_variable("undeclared", "1"), Status::UnknownName);
    EXPECT_FALSE(engine.variable("undeclared").has_value());
}

TEST(VariableConditionTest, DanglingVariableRefFailsInitWithRuleC79) {
    scena::ir::Scenario scenario = make_scenario(variable("missing", Rule::EqualTo, "1"));
    Engine engine;
    EXPECT_EQ(engine.init(std::move(scenario)), Status::SemanticError);
    EXPECT_TRUE(has_error(engine, Status::SemanticError,
                          "asam.net:xosc:1.2.0:reference_control.resolvable_variable_reference"));
}

TEST(VariableConditionTest, VariableNumericOrderingAcrossUpdates) {
    scena::ir::Scenario scenario = make_scenario(variable("v", Rule::GreaterOrEqual, "10"));
    scenario.variables["v"] = "0";
    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    EXPECT_FALSE(ego_fired(engine));
    ASSERT_EQ(engine.set_variable("v", "5"), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_FALSE(ego_fired(engine)); // 5 >= 10 is false
    ASSERT_EQ(engine.set_variable("v", "12"), Status::Ok);
    ASSERT_EQ(engine.step(2.0), Status::Ok);
    EXPECT_TRUE(ego_fired(engine)); // 12 >= 10 holds
}

// ---------------------------------------------------------------------------
// UserDefinedValueCondition (§ UserDefinedValueCondition)
// ---------------------------------------------------------------------------

TEST(UserDefinedValueConditionTest, UserValueSetBeforeInitVisibleAtTimeZero) {
    Engine engine;
    ASSERT_EQ(engine.set_user_defined_value("ext", "ready"), Status::Ok); // staged pre-init
    ASSERT_EQ(engine.init(make_scenario(user_value("ext", Rule::EqualTo, "ready"))), Status::Ok);
    EXPECT_TRUE(ego_fired(engine));
}

TEST(UserDefinedValueConditionTest, UserValueDrivesTriggerWithEdgeAndDelay) {
    Engine engine;
    ASSERT_EQ(engine.set_user_defined_value("sig", "0"), Status::Ok);
    ASSERT_EQ(engine.init(
                  make_scenario(user_value("sig", Rule::EqualTo, "1"), ConditionEdge::Rising, 0.5)),
              Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok); // t = 1, still 0
    ASSERT_EQ(engine.set_user_defined_value("sig", "1"), Status::Ok);
    ASSERT_EQ(engine.step(0.5), Status::Ok); // t = 1.5, rise; delayed lookup still false
    EXPECT_FALSE(ego_fired(engine));
    ASSERT_EQ(engine.step(0.5), Status::Ok); // t = 2.0, the 0.5 s-delayed rise fires
    EXPECT_TRUE(ego_fired(engine));
}

TEST(UserDefinedValueConditionTest, UnknownUserValueIsFalseAndWarnsOnce) {
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(user_value("missing", Rule::EqualTo, "1"))), Status::Ok);
    EXPECT_FALSE(ego_fired(engine));
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    ASSERT_EQ(engine.step(2.0), Status::Ok);
    // The unset value is queried in every evaluation but warned about once.
    EXPECT_EQ(warning_count(engine, "values/userDefined/missing"), 1);
}

TEST(UserDefinedValueConditionTest, UserValueNumericComparison) {
    Engine engine;
    ASSERT_EQ(engine.set_user_defined_value("count", "2"), Status::Ok);
    ASSERT_EQ(engine.init(make_scenario(user_value("count", Rule::GreaterThan, "3"))), Status::Ok);
    EXPECT_FALSE(ego_fired(engine)); // 2 > 3 is false
    ASSERT_EQ(engine.set_user_defined_value("count", "5"), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_TRUE(ego_fired(engine)); // 5 > 3 holds
}

// ---------------------------------------------------------------------------
// TimeOfDayCondition (§ TimeOfDayCondition, dateTime data type)
// ---------------------------------------------------------------------------

TEST(TimeOfDayConditionTest, UnsetTimeOfDayIsFalseAndWarnsOnce) {
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(
                  time_of_day(DateTime{2000, 1, 1, 12, 0, 0, 0, 0}, Rule::GreaterOrEqual))),
              Status::Ok);
    EXPECT_FALSE(ego_fired(engine)); // no anchor set
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    ASSERT_EQ(engine.step(2.0), Status::Ok);
    EXPECT_EQ(warning_count(engine, "timeOfDay"), 1);
}

TEST(TimeOfDayConditionTest, TimeOfDayAdvancesWithSimulationTime) {
    Engine engine;
    // Anchor noon UTC at t = 0; the reference is five seconds later.
    ASSERT_EQ(engine.set_date_time(DateTime{2000, 1, 1, 12, 0, 0, 0, 0}), Status::Ok);
    ASSERT_EQ(engine.init(make_scenario(
                  time_of_day(DateTime{2000, 1, 1, 12, 0, 5, 0, 0}, Rule::GreaterOrEqual))),
              Status::Ok);
    EXPECT_FALSE(ego_fired(engine)); // t = 0
    ASSERT_EQ(engine.step(3.0), Status::Ok);
    EXPECT_FALSE(ego_fired(engine)); // t = 3, still before the reference
    ASSERT_EQ(engine.step(2.0), Status::Ok);
    EXPECT_TRUE(ego_fired(engine)); // t = 5, simulated time reaches the reference
    // The getter reflects the advanced simulated instant.
    ASSERT_TRUE(engine.date_time().has_value());
    EXPECT_DOUBLE_EQ(*engine.date_time(),
                     DateTime({2000, 1, 1, 12, 0, 5, 0, 0}).to_epoch_seconds());
}

TEST(TimeOfDayConditionTest, FrozenAnchorHoldsTheConditionFalseForever) {
    // The condition itself is unchanged by p5-s6: it reads whatever instant
    // the context reports. What an EnvironmentAction with
    // §TimeOfDay animation="false" changes is that the instant stops moving,
    // so a reference in the future is never reached (docs/user-guide/
    // global-actions.md).
    Engine engine;
    scena::ir::Environment environment;
    environment.time_of_day =
        scena::ir::TimeOfDay{/*animation=*/false, DateTime{2000, 1, 1, 12, 0, 0, 0, 0}};
    scena::ir::Scenario scenario =
        make_scenario(time_of_day(DateTime{2000, 1, 1, 12, 0, 5, 0, 0}, Rule::GreaterOrEqual));
    scenario.init_actions.push_back(
        std::make_shared<scena::ir::EnvironmentAction>(std::move(environment)));
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);

    ASSERT_TRUE(engine.date_time().has_value());
    const double anchor = *engine.date_time();
    for (int i = 0; i < 10; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
        EXPECT_EQ(*engine.date_time(), anchor); // bit-identical every step
        EXPECT_FALSE(ego_fired(engine));
    }
}

TEST(TimeOfDayConditionTest, TimeOfDayRuleFamilyAtBoundary) {
    // now == reference exactly at t = 0 (anchor equals the reference).
    const auto fires_at_zero = [](Rule rule) {
        Engine engine;
        engine.set_date_time(DateTime{2000, 1, 1, 12, 0, 0, 0, 0});
        engine.init(make_scenario(time_of_day(DateTime{2000, 1, 1, 12, 0, 0, 0, 0}, rule)));
        return ego_fired(engine);
    };
    EXPECT_TRUE(fires_at_zero(Rule::EqualTo));
    EXPECT_TRUE(fires_at_zero(Rule::GreaterOrEqual));
    EXPECT_TRUE(fires_at_zero(Rule::LessOrEqual));
    EXPECT_FALSE(fires_at_zero(Rule::GreaterThan)); // strictly later only
    EXPECT_FALSE(fires_at_zero(Rule::LessThan));    // time never runs backwards

    // greaterThan holds once simulation time carries past the reference.
    Engine later;
    later.set_date_time(DateTime{2000, 1, 1, 12, 0, 0, 0, 0});
    later.init(make_scenario(time_of_day(DateTime{2000, 1, 1, 12, 0, 0, 0, 0}, Rule::GreaterThan)));
    ASSERT_FALSE(ego_fired(later));
    ASSERT_EQ(later.step(1.0), Status::Ok);
    EXPECT_TRUE(ego_fired(later));
}

TEST(TimeOfDayConditionTest, TimezoneOffsetNormalizesToUtc) {
    // 13:00+01:00 is the same instant as 12:00 UTC, so an equalTo against the
    // noon-UTC anchor holds — the offset is folded into the epoch comparison.
    Engine engine;
    ASSERT_EQ(engine.set_date_time(DateTime{2000, 1, 1, 12, 0, 0, 0, 0}), Status::Ok);
    ASSERT_EQ(engine.init(
                  make_scenario(time_of_day(DateTime{2000, 1, 1, 13, 0, 0, 0, 60}, Rule::EqualTo))),
              Status::Ok);
    EXPECT_TRUE(ego_fired(engine));
}

TEST(TimeOfDayConditionTest, InvalidTimeOfDayDateTimeFailsInit) {
    // April has 30 days, so the fields do not name a real instant.
    Engine engine;
    EXPECT_EQ(engine.init(
                  make_scenario(time_of_day(DateTime{2000, 4, 31, 0, 0, 0, 0, 0}, Rule::EqualTo))),
              Status::ValidationError);
}

TEST(TimeOfDayConditionTest, SetDateTimeRejectsInvalidDateTime) {
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(sim_time(100.0))), Status::Ok);
    EXPECT_EQ(engine.set_date_time(DateTime{2001, 2, 29, 0, 0, 0, 0, 0}), Status::InvalidArgument);
    EXPECT_FALSE(engine.date_time().has_value());
}

// ---------------------------------------------------------------------------
// StoryboardElementStateCondition (§ StoryboardElementStateCondition, §8.1-8.2)
// ---------------------------------------------------------------------------

TEST(StoryboardElementStateConditionTest, StateLiteralsObserveStandbyRunningComplete) {
    // Standby: a not-yet-triggered element is observed in standbyState.
    {
        Scheduler scheduler;
        const scena::ir::Storyboard storyboard =
            make_storyboard({named_event("source", trigger_of({{cond(sim_time(100.0))}})),
                             named_event("obs", trigger_of({{cond(element_state(
                                                    StoryboardElementType::Event, "source",
                                                    StoryboardElementState::StandbyState))}}))});
        scheduler.bind(storyboard);
        Recorder recorder;
        EXPECT_EQ(step_events(scheduler, recorder, 0.0), (std::vector<std::string>{"obs"}));
    }
    // Running: an event kept running by an ongoing action is observed running.
    {
        Scheduler scheduler;
        const scena::ir::Storyboard storyboard =
            make_storyboard({named_event("source", trigger_of({{cond(sim_time(1.0))}})),
                             named_event("obs", trigger_of({{cond(element_state(
                                                    StoryboardElementType::Event, "source",
                                                    StoryboardElementState::RunningState))}}))});
        scheduler.bind(storyboard);
        Recorder recorder({"source"}); // source stays running
        EXPECT_TRUE(step_events(scheduler, recorder, 0.0).empty());
        EXPECT_EQ(step_events(scheduler, recorder, 1.0),
                  (std::vector<std::string>{"source", "obs"}));
    }
    // Complete: an instantaneously-finishing event is observed complete.
    {
        Scheduler scheduler;
        const scena::ir::Storyboard storyboard =
            make_storyboard({named_event("source", trigger_of({{cond(sim_time(1.0))}})),
                             named_event("obs", trigger_of({{cond(element_state(
                                                    StoryboardElementType::Event, "source",
                                                    StoryboardElementState::CompleteState))}}))});
        scheduler.bind(storyboard);
        Recorder recorder;
        EXPECT_TRUE(step_events(scheduler, recorder, 0.0).empty());
        EXPECT_EQ(step_events(scheduler, recorder, 1.0),
                  (std::vector<std::string>{"source", "obs"}));
    }
}

TEST(StoryboardElementStateConditionTest, StartTransitionHoldsOnlyInItsEvaluation) {
    Scheduler scheduler;
    const scena::ir::Storyboard storyboard = make_storyboard(
        {named_event("source", trigger_of({{cond(sim_time(1.0))}})),
         named_event("observer",
                     trigger_of({{cond(element_state(StoryboardElementType::Event, "source",
                                                     StoryboardElementState::StartTransition))}}),
                     scena::ir::EventPriority::Parallel, /*max=*/3)});
    scheduler.bind(storyboard);
    Recorder recorder({"source"}); // source stays running, so it is not re-started
    // Even though the observer re-arms (max 3) and the source stays running,
    // the startTransition is a one-evaluation pulse: observed only at t = 1.
    EXPECT_EQ(observer_firings(scheduler, recorder, "observer", {0.0, 1.0, 2.0, 3.0}),
              (std::vector<double>{1.0}));
}

TEST(StoryboardElementStateConditionTest, EndTransitionObservedOnEventCompletion) {
    Scheduler scheduler;
    const scena::ir::Storyboard storyboard = make_storyboard(
        {named_event("source", trigger_of({{cond(sim_time(1.0))}})),
         named_event("observer",
                     trigger_of({{cond(element_state(StoryboardElementType::Event, "source",
                                                     StoryboardElementState::EndTransition))}}))});
    scheduler.bind(storyboard);
    Recorder recorder;
    EXPECT_EQ(observer_firings(scheduler, recorder, "observer", {0.0, 1.0, 2.0}),
              (std::vector<double>{1.0}));
}

TEST(StoryboardElementStateConditionTest, StopTransitionObservedUnderStopBeforeStartOrdering) {
    // Two acts in document order: act1 hosts the stopped source, act2 the
    // observer. Stop is processed before act2's start triggers in the same
    // evaluation, so the stopTransition is visible to the later observer.
    scena::ir::Maneuver source_maneuver;
    source_maneuver.name = "sm";
    source_maneuver.events.push_back(named_event("source", trigger_of({{cond(sim_time(1.0))}})));
    scena::ir::ManeuverGroup source_group;
    source_group.name = "sg";
    source_group.maneuvers.push_back(std::move(source_maneuver));
    scena::ir::Act act1;
    act1.name = "act1";
    act1.stop_trigger = trigger_of({{cond(sim_time(2.0))}});
    act1.groups.push_back(std::move(source_group));

    scena::ir::Maneuver obs_maneuver;
    obs_maneuver.name = "om";
    obs_maneuver.events.push_back(named_event(
        "observer", trigger_of({{cond(element_state(StoryboardElementType::Event, "source",
                                                    StoryboardElementState::StopTransition))}})));
    scena::ir::ManeuverGroup obs_group;
    obs_group.name = "og";
    obs_group.maneuvers.push_back(std::move(obs_maneuver));
    scena::ir::Act act2;
    act2.name = "act2";
    act2.groups.push_back(std::move(obs_group));

    scena::ir::Story story;
    story.name = "story";
    story.acts.push_back(std::move(act1));
    story.acts.push_back(std::move(act2));
    scena::ir::Storyboard storyboard;
    storyboard.stories.push_back(std::move(story));

    Scheduler scheduler;
    scheduler.bind(storyboard);
    Recorder recorder({"source"}); // source runs until stopped at t = 2
    EXPECT_EQ(observer_firings(scheduler, recorder, "observer", {0.0, 1.0, 2.0, 3.0}),
              (std::vector<double>{2.0}));
}

TEST(StoryboardElementStateConditionTest, SkipTransitionObservedOnSkippedEvent) {
    Scheduler scheduler;
    const scena::ir::Storyboard storyboard = make_storyboard(
        {named_event("runner", trigger_of({{cond(sim_time(1.0))}})),
         named_event("skipper", trigger_of({{cond(sim_time(2.0))}}),
                     scena::ir::EventPriority::Skip),
         named_event("observer",
                     trigger_of({{cond(element_state(StoryboardElementType::Event, "skipper",
                                                     StoryboardElementState::SkipTransition))}}))});
    scheduler.bind(storyboard);
    Recorder recorder({"runner"}); // runner keeps running, so skipper is skipped at t = 2
    EXPECT_EQ(observer_firings(scheduler, recorder, "observer", {0.0, 1.0, 2.0, 3.0}),
              (std::vector<double>{2.0}));
}

TEST(StoryboardElementStateConditionTest, UniqueNameResolvesAcrossHierarchy) {
    // A globally-unique name is used bare and resolves though it lives in a
    // different maneuver from the referencing element.
    Scheduler scheduler;
    const scena::ir::Storyboard storyboard = make_two_maneuver_storyboard(
        "first", {named_event("src", trigger_of({{cond(sim_time(1.0))}}))}, "second",
        {named_event("watcher",
                     trigger_of({{cond(element_state(StoryboardElementType::Event, "src",
                                                     StoryboardElementState::CompleteState))}}))});
    scheduler.bind(storyboard);
    Recorder recorder;
    EXPECT_EQ(observer_firings(scheduler, recorder, "watcher", {0.0, 1.0, 2.0}),
              (std::vector<double>{1.0}));
}

TEST(StoryboardElementStateConditionTest, DoubleColonPrefixDisambiguates) {
    // Two events named "leg"; "first::leg" selects the one in maneuver "first"
    // (completes at t = 1), not the one in "second" (t = 5).
    Scheduler scheduler;
    const scena::ir::Storyboard storyboard = make_two_maneuver_storyboard(
        "first",
        {named_event("leg", trigger_of({{cond(sim_time(1.0))}})),
         named_event("watch",
                     trigger_of({{cond(element_state(StoryboardElementType::Event, "first::leg",
                                                     StoryboardElementState::CompleteState))}}))},
        "second", {named_event("leg", trigger_of({{cond(sim_time(5.0))}}))});
    scheduler.bind(storyboard);
    Recorder recorder;
    EXPECT_EQ(observer_firings(scheduler, recorder, "watch", {0.0, 1.0, 2.0, 5.0}),
              (std::vector<double>{1.0}));
}

TEST(StoryboardElementStateConditionTest, TransitionPulseWithDelayReplaysShifted) {
    // The endTransition pulse at t = 1 is one evaluation wide; a 0.5 s delay
    // replays it shifted, firing exactly once at t = 1.5.
    Scheduler scheduler;
    const scena::ir::Storyboard storyboard = make_storyboard(
        {named_event("source", trigger_of({{cond(sim_time(1.0))}})),
         named_event("observer",
                     trigger_of({{cond(element_state(StoryboardElementType::Event, "source",
                                                     StoryboardElementState::EndTransition),
                                       ConditionEdge::None, 0.5)}}))});
    scheduler.bind(storyboard);
    Recorder recorder;
    EXPECT_EQ(observer_firings(scheduler, recorder, "observer", {0.0, 1.0, 1.5, 2.0}),
              (std::vector<double>{1.5}));
}

TEST(StoryboardElementStateConditionTest, AmbiguousRefFailsInitWithRuleC720) {
    scena::ir::Storyboard storyboard = make_two_maneuver_storyboard(
        "first",
        {named_event("leg", trigger_of({{cond(sim_time(1.0))}})),
         named_event("observer",
                     trigger_of({{cond(element_state(StoryboardElementType::Event, "leg",
                                                     StoryboardElementState::CompleteState))}}))},
        "second", {named_event("leg", trigger_of({{cond(sim_time(5.0))}}))});
    Engine engine;
    EXPECT_EQ(engine.init(engine_scenario(std::move(storyboard), {"leg", "observer"})),
              Status::SemanticError);
    EXPECT_TRUE(
        has_error(engine, Status::SemanticError,
                  "asam.net:xosc:1.0.0:reference_control.resolvable_storyboard_element_ref"));
}

TEST(StoryboardElementStateConditionTest, DanglingOrTypeMismatchedRefFailsInit) {
    {
        // Dangling: no element named "ghost".
        scena::ir::Storyboard storyboard = make_storyboard({named_event(
            "observer",
            trigger_of({{cond(element_state(StoryboardElementType::Event, "ghost",
                                            StoryboardElementState::CompleteState))}}))});
        Engine engine;
        EXPECT_EQ(engine.init(engine_scenario(std::move(storyboard), {"observer"})),
                  Status::SemanticError);
        EXPECT_TRUE(
            has_error(engine, Status::SemanticError,
                      "asam.net:xosc:1.0.0:reference_control.resolvable_storyboard_element_ref"));
    }
    {
        // Type mismatch: "src" is an Event, referenced here as a Maneuver.
        scena::ir::Storyboard storyboard = make_storyboard(
            {named_event("src", trigger_of({{cond(sim_time(1.0))}})),
             named_event(
                 "observer",
                 trigger_of({{cond(element_state(StoryboardElementType::Maneuver, "src",
                                                 StoryboardElementState::CompleteState))}}))});
        Engine engine;
        EXPECT_EQ(engine.init(engine_scenario(std::move(storyboard), {"src", "observer"})),
                  Status::SemanticError);
    }
}

TEST(StoryboardElementStateConditionTest, ActionTypeEmitsUnsupportedFeatureWarning) {
    // Per-action nodes do not exist yet: an action reference warns and the
    // condition is always false, but init succeeds.
    scena::ir::Storyboard storyboard = make_storyboard({named_event(
        "observer", trigger_of({{cond(element_state(StoryboardElementType::Action, "whatever",
                                                    StoryboardElementState::CompleteState))}}))});
    Engine engine;
    ASSERT_EQ(engine.init(engine_scenario(std::move(storyboard), {"observer"})), Status::Ok);
    EXPECT_GE(warning_count(engine, "story/act/group/maneuver/observer/startTrigger/group[0]/"
                                    "condition[0]"),
              1);
}
