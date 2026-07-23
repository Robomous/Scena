// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
//
// Traffic signals (p5-s6, ASAM OpenSCENARIO XML 1.4.0 §6.11): the controller
// cycle clock and its phase transitions, the chained-controller delay of
// §6.11.3, the two infrastructure actions (§7.4.2) and the two signal
// conditions (§7.6.5.2), plus the load-time reference and range rules of
// Annex C (C.2.3, C.7.11, C.7.12, C.7.13).

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "scena/diagnostic.h"
#include "scena/engine.h"
#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/entity.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/traffic_signal.h"
#include "scena/ir/trigger.h"

namespace {

using scena::Engine;
using scena::Severity;
using scena::Status;
using scena::ir::Act;
using scena::ir::Action;
using scena::ir::ControlMode;
using scena::ir::Entity;
using scena::ir::Event;
using scena::ir::make_trigger;
using scena::ir::Maneuver;
using scena::ir::ManeuverGroup;
using scena::ir::Phase;
using scena::ir::Scenario;
using scena::ir::SimulationTimeCondition;
using scena::ir::Story;
using scena::ir::TrafficSignalCondition;
using scena::ir::TrafficSignalController;
using scena::ir::TrafficSignalControllerAction;
using scena::ir::TrafficSignalControllerCondition;
using scena::ir::TrafficSignalState;
using scena::ir::TrafficSignalStateAction;
using scena::ir::VariableSetAction;

/// A controller with one signal per phase, the shape every timing test uses:
/// phase i drives signal `signal` into state `<name>`.
TrafficSignalController make_controller(std::string name, const char* signal,
                                        const std::vector<std::pair<const char*, double>>& phases) {
    TrafficSignalController controller;
    controller.name = std::move(name);
    for (const auto& [phase_name, duration] : phases) {
        Phase phase;
        phase.name = phase_name;
        phase.duration = duration;
        phase.signal_states.push_back(TrafficSignalState{signal, phase_name});
        controller.phases.push_back(std::move(phase));
    }
    return controller;
}

/// A scenario with one entity and an empty single-maneuver storyboard.
Scenario make_scenario() {
    Scenario scenario;
    scenario.name = "traffic-signals";
    Entity ego;
    ego.id = "ego";
    ego.name = "ego";
    ego.control_mode = ControlMode::EngineControlled;
    scenario.entities.push_back(std::move(ego));

    Maneuver maneuver;
    maneuver.name = "maneuver";
    ManeuverGroup group;
    group.name = "group";
    group.maneuvers.push_back(std::move(maneuver));
    Act act;
    act.name = "act";
    act.groups.push_back(std::move(group));
    Story story;
    story.name = "story";
    story.acts.push_back(std::move(act));
    scenario.storyboard.stories.push_back(std::move(story));
    return scenario;
}

void add_event(Scenario& scenario, std::string name, double at_time,
               std::shared_ptr<Action> action) {
    Event event;
    event.name = std::move(name);
    event.start_trigger = make_trigger(std::make_shared<SimulationTimeCondition>(at_time));
    event.actions.push_back(std::move(action));
    scenario.storyboard.stories.front()
        .acts.front()
        .groups.front()
        .maneuvers.front()
        .events.push_back(std::move(event));
}

bool has_diagnostic(const Engine& engine, Severity severity, const std::string& needle) {
    for (const scena::Diagnostic& diagnostic : engine.diagnostics()) {
        if (diagnostic.severity == severity &&
            diagnostic.message.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

bool cites_rule(const Engine& engine, const std::string& rule_id) {
    for (const scena::Diagnostic& diagnostic : engine.diagnostics()) {
        if (diagnostic.rule_id == rule_id) {
            return true;
        }
    }
    return false;
}

// --- The cycle clock (§6.11.4) ---------------------------------------------

TEST(TrafficSignalTest, FirstPhaseAppliesAtStoryboardStart) {
    // §6.11.4: "The first Phase starts with the execution of the storyboard."
    Scenario scenario = make_scenario();
    scenario.traffic_signal_controllers.push_back(
        make_controller("group1", "signal1", {{"stop", 20.0}, {"go", 15.0}}));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    EXPECT_EQ(engine.traffic_signal_state("signal1"), std::optional<std::string>("stop"));
    EXPECT_EQ(engine.traffic_signal_controller_phase("group1"), std::optional<std::string>("stop"));
    // An unwritten signal and an unknown controller both report nothing.
    EXPECT_FALSE(engine.traffic_signal_state("signal9").has_value());
    EXPECT_FALSE(engine.traffic_signal_controller_phase("group9").has_value());
}

TEST(TrafficSignalTest, PhaseBoundariesFollowDurations) {
    Scenario scenario = make_scenario();
    scenario.traffic_signal_controllers.push_back(
        make_controller("group1", "signal1", {{"stop", 10.0}, {"caution", 3.0}, {"go", 12.0}}));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    const auto state_at = [&engine](double target) {
        while (engine.time() < target) {
            const double remaining = target - engine.time();
            engine.step(remaining < 0.5 ? remaining : 0.5);
        }
        return engine.traffic_signal_state("signal1");
    };
    EXPECT_EQ(state_at(9.5), std::optional<std::string>("stop"));
    // The boundary belongs to the next phase: at exactly t = 10 the cycle has
    // spent the whole "stop" duration.
    EXPECT_EQ(state_at(10.0), std::optional<std::string>("caution"));
    EXPECT_EQ(state_at(12.5), std::optional<std::string>("caution"));
    EXPECT_EQ(state_at(13.0), std::optional<std::string>("go"));
    EXPECT_EQ(state_at(24.5), std::optional<std::string>("go"));
}

TEST(TrafficSignalTest, CycleRepeatsAfterTotalDuration) {
    // §6.11.4: "the first Phase repeats after the last Phase has ended."
    Scenario scenario = make_scenario();
    scenario.traffic_signal_controllers.push_back(
        make_controller("group1", "signal1", {{"stop", 4.0}, {"go", 6.0}}));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    std::vector<std::string> transcript;
    for (int i = 0; i < 50; ++i) { // 25 s: two and a half cycles
        ASSERT_EQ(engine.step(0.5), Status::Ok);
        transcript.push_back(*engine.traffic_signal_state("signal1"));
    }
    // t = 4 (index 7) is the first "go"; t = 10 (index 19) is back to "stop";
    // t = 14 (index 27) "go" again — the cycle length is 10 s exactly.
    EXPECT_EQ(transcript[6], "stop");  // t = 3.5
    EXPECT_EQ(transcript[7], "go");    // t = 4.0
    EXPECT_EQ(transcript[18], "go");   // t = 9.5
    EXPECT_EQ(transcript[19], "stop"); // t = 10.0, cycle restarts
    EXPECT_EQ(transcript[27], "go");   // t = 14.0
}

TEST(TrafficSignalTest, ZeroTotalDurationPinsFirstPhase) {
    // A cycle of zero length has no next phase to move to; the first phase is
    // held rather than dividing by the cycle duration, and the load warns.
    Scenario scenario = make_scenario();
    scenario.traffic_signal_controllers.push_back(
        make_controller("group1", "signal1", {{"stop", 0.0}, {"go", 0.0}}));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    EXPECT_TRUE(has_diagnostic(engine, Severity::Warning, "zero total duration"));
    for (int i = 0; i < 10; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
        EXPECT_EQ(engine.traffic_signal_state("signal1"), std::optional<std::string>("stop"));
    }
}

TEST(TrafficSignalTest, ControllerWithNoPhasesDrivesNothing) {
    Scenario scenario = make_scenario();
    TrafficSignalController empty;
    empty.name = "group1";
    scenario.traffic_signal_controllers.push_back(std::move(empty));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    ASSERT_EQ(engine.step(5.0), Status::Ok);
    EXPECT_FALSE(engine.traffic_signal_controller_phase("group1").has_value());
}

// --- Chained controllers (§6.11.3) -----------------------------------------

TEST(TrafficSignalTest, DelayedControllerStartsAfterReferenceChain) {
    // §6.11.3: a controller's "first phase virtually starts delaytime seconds
    // after the start of the reference's first phase", and the delays compose
    // along the chain.
    Scenario scenario = make_scenario();
    scenario.traffic_signal_controllers.push_back(
        make_controller("first", "s1", {{"a", 10.0}, {"b", 10.0}}));
    TrafficSignalController second = make_controller("second", "s2", {{"a", 10.0}, {"b", 10.0}});
    second.delay = 4.0;
    second.reference = "first";
    scenario.traffic_signal_controllers.push_back(std::move(second));
    TrafficSignalController third = make_controller("third", "s3", {{"a", 10.0}, {"b", 10.0}});
    third.delay = 3.0;
    third.reference = "second"; // 4 + 3 = 7 s behind "first"
    scenario.traffic_signal_controllers.push_back(std::move(third));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    // At t = 0 only the unchained controller has a phase at all.
    EXPECT_EQ(engine.traffic_signal_controller_phase("first"), std::optional<std::string>("a"));
    EXPECT_FALSE(engine.traffic_signal_controller_phase("second").has_value());
    EXPECT_FALSE(engine.traffic_signal_controller_phase("third").has_value());
    EXPECT_FALSE(engine.traffic_signal_state("s2").has_value());

    for (int i = 0; i < 8; ++i) { // t = 4
        ASSERT_EQ(engine.step(0.5), Status::Ok);
    }
    EXPECT_EQ(engine.traffic_signal_controller_phase("second"), std::optional<std::string>("a"));
    EXPECT_FALSE(engine.traffic_signal_controller_phase("third").has_value());

    for (int i = 0; i < 6; ++i) { // t = 7
        ASSERT_EQ(engine.step(0.5), Status::Ok);
    }
    EXPECT_EQ(engine.traffic_signal_controller_phase("third"), std::optional<std::string>("a"));
    // "first" is already 7 s into its 10 s opening phase, "third" just started:
    // the offset is a genuine progressive-system skew, not a shared clock.
    EXPECT_EQ(engine.traffic_signal_controller_phase("first"), std::optional<std::string>("a"));
    for (int i = 0; i < 6; ++i) { // t = 10: "first" flips, "third" does not
        ASSERT_EQ(engine.step(0.5), Status::Ok);
    }
    EXPECT_EQ(engine.traffic_signal_controller_phase("first"), std::optional<std::string>("b"));
    EXPECT_EQ(engine.traffic_signal_controller_phase("third"), std::optional<std::string>("a"));
}

TEST(TrafficSignalTest, DelayWithoutReferenceFailsInit) {
    Scenario scenario = make_scenario();
    TrafficSignalController controller = make_controller("group1", "s1", {{"a", 5.0}});
    controller.delay = 3.0; // no reference
    scenario.traffic_signal_controllers.push_back(std::move(controller));

    Engine engine;
    EXPECT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    EXPECT_TRUE(has_diagnostic(engine, Severity::Error, "delay requires a reference"));
}

TEST(TrafficSignalTest, UnknownControllerReferenceFailsInitC713) {
    Scenario scenario = make_scenario();
    TrafficSignalController controller = make_controller("group1", "s1", {{"a", 5.0}});
    controller.delay = 3.0;
    controller.reference = "nobody";
    scenario.traffic_signal_controllers.push_back(std::move(controller));

    Engine engine;
    EXPECT_EQ(engine.init(std::move(scenario)), Status::SemanticError);
    EXPECT_TRUE(cites_rule(
        engine, "asam.net:xosc:1.0.0:reference_control.traffic_signal_controller_references"));
}

TEST(TrafficSignalTest, ReferenceCycleFailsInit) {
    Scenario scenario = make_scenario();
    TrafficSignalController a = make_controller("a", "s1", {{"p", 5.0}});
    a.delay = 1.0;
    a.reference = "b";
    TrafficSignalController b = make_controller("b", "s2", {{"p", 5.0}});
    b.delay = 1.0;
    b.reference = "a";
    scenario.traffic_signal_controllers.push_back(std::move(a));
    scenario.traffic_signal_controllers.push_back(std::move(b));

    Engine engine;
    EXPECT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    EXPECT_TRUE(has_diagnostic(engine, Severity::Error, "is cyclic"));
}

TEST(TrafficSignalTest, SelfReferenceFailsInit) {
    Scenario scenario = make_scenario();
    TrafficSignalController controller = make_controller("group1", "s1", {{"a", 5.0}});
    controller.delay = 2.0;
    controller.reference = "group1";
    scenario.traffic_signal_controllers.push_back(std::move(controller));

    Engine engine;
    EXPECT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    EXPECT_TRUE(has_diagnostic(engine, Severity::Error, "references itself"));
}

// --- Load-time validation --------------------------------------------------

TEST(TrafficSignalTest, NegativePhaseDurationFailsInitC23) {
    Scenario scenario = make_scenario();
    scenario.traffic_signal_controllers.push_back(
        make_controller("group1", "s1", {{"a", 5.0}, {"b", -1.0}}));

    Engine engine;
    EXPECT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    EXPECT_TRUE(cites_rule(engine, "asam.net:xosc:1.0.0:data_type.phase_duration_positive"));
}

TEST(TrafficSignalTest, DuplicatePhaseNamesFailInit) {
    // §6.11.4: a phase name is "unique within its controller" — it is what an
    // action and a condition address the phase by.
    Scenario scenario = make_scenario();
    scenario.traffic_signal_controllers.push_back(
        make_controller("group1", "s1", {{"go", 5.0}, {"go", 5.0}}));

    Engine engine;
    EXPECT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    EXPECT_TRUE(has_diagnostic(engine, Severity::Error, "duplicate traffic signal phase name"));
}

TEST(TrafficSignalTest, DuplicateControllerNamesFailInit) {
    Scenario scenario = make_scenario();
    scenario.traffic_signal_controllers.push_back(make_controller("group1", "s1", {{"a", 5.0}}));
    scenario.traffic_signal_controllers.push_back(make_controller("group1", "s2", {{"a", 5.0}}));

    Engine engine;
    EXPECT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    EXPECT_TRUE(
        has_diagnostic(engine, Severity::Error, "duplicate traffic signal controller name"));
}

TEST(TrafficSignalTest, EmptySignalIdFailsInit) {
    Scenario scenario = make_scenario();
    TrafficSignalController controller;
    controller.name = "group1";
    Phase phase;
    phase.name = "go";
    phase.duration = 5.0;
    phase.signal_states.push_back(TrafficSignalState{"", "on"});
    controller.phases.push_back(std::move(phase));
    scenario.traffic_signal_controllers.push_back(std::move(controller));

    Engine engine;
    EXPECT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
}

// --- The infrastructure actions (§7.4.2) -----------------------------------

TEST(TrafficSignalTest, ControllerActionRestartsCycleAtNamedPhase) {
    Scenario scenario = make_scenario();
    scenario.traffic_signal_controllers.push_back(
        make_controller("group1", "s1", {{"stop", 20.0}, {"caution", 4.0}, {"go", 16.0}}));
    add_event(scenario, "jump", 5.0,
              std::make_shared<TrafficSignalControllerAction>("group1", "caution"));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    EXPECT_EQ(engine.traffic_signal_state("s1"), std::optional<std::string>("stop"));
    for (int i = 0; i < 10; ++i) { // t = 5, the action fires
        ASSERT_EQ(engine.step(0.5), Status::Ok);
    }
    // Observable in the very evaluation the action ran: the controller is
    // ticked immediately so later conditions in the same walk agree.
    EXPECT_EQ(engine.traffic_signal_controller_phase("group1"),
              std::optional<std::string>("caution"));
    EXPECT_EQ(engine.traffic_signal_state("s1"), std::optional<std::string>("caution"));

    // The cycle continues in declared order from there: "caution" lasts its
    // full 4 s from t = 5, then "go".
    for (int i = 0; i < 7; ++i) { // t = 8.5
        ASSERT_EQ(engine.step(0.5), Status::Ok);
    }
    EXPECT_EQ(engine.traffic_signal_state("s1"), std::optional<std::string>("caution"));
    ASSERT_EQ(engine.step(0.5), Status::Ok); // t = 9.0 = 5 + 4
    EXPECT_EQ(engine.traffic_signal_state("s1"), std::optional<std::string>("go"));
}

TEST(TrafficSignalTest, ControllerActionUnknownRefsFailInitC711) {
    Scenario unknown_controller = make_scenario();
    unknown_controller.traffic_signal_controllers.push_back(
        make_controller("group1", "s1", {{"go", 5.0}}));
    add_event(unknown_controller, "jump", 1.0,
              std::make_shared<TrafficSignalControllerAction>("group9", "go"));
    Engine engine_a;
    EXPECT_EQ(engine_a.init(std::move(unknown_controller)), Status::SemanticError);
    EXPECT_TRUE(cites_rule(
        engine_a,
        "asam.net:xosc:1.0.0:reference_control.traffic_signal_controller_action_references"));

    Scenario unknown_phase = make_scenario();
    unknown_phase.traffic_signal_controllers.push_back(
        make_controller("group1", "s1", {{"go", 5.0}}));
    add_event(unknown_phase, "jump", 1.0,
              std::make_shared<TrafficSignalControllerAction>("group1", "nope"));
    Engine engine_b;
    EXPECT_EQ(engine_b.init(std::move(unknown_phase)), Status::SemanticError);
    EXPECT_TRUE(has_diagnostic(engine_b, Severity::Error, "has no phase 'nope'"));
}

TEST(TrafficSignalTest, StateActionOverridesUntilNextPhaseTransition) {
    // The §11.12 traffic-light-failure shape: a signal is forced into a state
    // its controller never produces, and holds it until the cycle moves on.
    Scenario scenario = make_scenario();
    scenario.traffic_signal_controllers.push_back(
        make_controller("group1", "s1", {{"stop", 10.0}, {"go", 10.0}}));
    add_event(scenario, "break", 4.0,
              std::make_shared<TrafficSignalStateAction>("s1", "red;green"));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    for (int i = 0; i < 8; ++i) { // t = 4, the failure is injected
        ASSERT_EQ(engine.step(0.5), Status::Ok);
    }
    EXPECT_EQ(engine.traffic_signal_state("s1"), std::optional<std::string>("red;green"));
    // Still broken most of the way through the phase: the controller writes on
    // transition only, so it does not overwrite the forced state every step.
    for (int i = 0; i < 11; ++i) { // t = 9.5
        ASSERT_EQ(engine.step(0.5), Status::Ok);
    }
    EXPECT_EQ(engine.traffic_signal_state("s1"), std::optional<std::string>("red;green"));
    // The controller's phase is unaffected by the state override.
    EXPECT_EQ(engine.traffic_signal_controller_phase("group1"), std::optional<std::string>("stop"));

    ASSERT_EQ(engine.step(0.5), Status::Ok); // t = 10: the phase transition wins
    EXPECT_EQ(engine.traffic_signal_state("s1"), std::optional<std::string>("go"));
}

TEST(TrafficSignalTest, StateActionInInitPhaseOverridesTheSeed) {
    // Init ordering: controllers seed their first phase before the init
    // actions run, so an init-phase state action is the one that stands.
    Scenario scenario = make_scenario();
    scenario.traffic_signal_controllers.push_back(
        make_controller("group1", "s1", {{"stop", 10.0}, {"go", 10.0}}));
    scenario.init_actions.push_back(std::make_shared<TrafficSignalStateAction>("s1", "dark"));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    EXPECT_EQ(engine.traffic_signal_state("s1"), std::optional<std::string>("dark"));
    EXPECT_EQ(engine.traffic_signal_controller_phase("group1"), std::optional<std::string>("stop"));
}

TEST(TrafficSignalTest, SignalStoresAreClearedByInitAndClose) {
    Scenario scenario = make_scenario();
    scenario.traffic_signal_controllers.push_back(make_controller("group1", "s1", {{"stop", 5.0}}));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    EXPECT_TRUE(engine.traffic_signal_state("s1").has_value());
    ASSERT_EQ(engine.close(), Status::Ok);
    EXPECT_FALSE(engine.traffic_signal_state("s1").has_value());
    EXPECT_FALSE(engine.traffic_signal_controller_phase("group1").has_value());

    ASSERT_EQ(engine.init(make_scenario()), Status::Ok); // no controllers at all
    EXPECT_FALSE(engine.traffic_signal_state("s1").has_value());
}

// --- The signal conditions (§7.6.5.2) --------------------------------------

TEST(TrafficSignalTest, TrafficSignalConditionLevelSemantics) {
    // A level predicate: true for as long as the state matches, not only in
    // the evaluation it is reached. The "reaches" wording of the standard is
    // supplied by conditionEdge rising, tested separately below.
    Scenario scenario = make_scenario();
    scenario.variables["seen"] = "no";
    scenario.traffic_signal_controllers.push_back(
        make_controller("group1", "s1", {{"stop", 5.0}, {"go", 5.0}}));

    Event gated;
    gated.name = "gated";
    gated.start_trigger = make_trigger(std::make_shared<TrafficSignalCondition>("s1", "go"));
    gated.actions.push_back(std::make_shared<VariableSetAction>("seen", "yes"));
    scenario.storyboard.stories.front()
        .acts.front()
        .groups.front()
        .maneuvers.front()
        .events.push_back(std::move(gated));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    EXPECT_EQ(engine.variable("seen"), "no");
    for (int i = 0; i < 8; ++i) { // t = 4, still "stop"
        ASSERT_EQ(engine.step(0.5), Status::Ok);
    }
    EXPECT_EQ(engine.variable("seen"), "no");
    for (int i = 0; i < 4; ++i) { // past t = 5, "go"
        ASSERT_EQ(engine.step(0.5), Status::Ok);
    }
    EXPECT_EQ(engine.variable("seen"), "yes");
}

TEST(TrafficSignalTest, UnknownSignalConditionIsFalseAndWarnsOnce) {
    Scenario scenario = make_scenario();
    scenario.variables["seen"] = "no";
    Event gated;
    gated.name = "gated";
    gated.start_trigger = make_trigger(std::make_shared<TrafficSignalCondition>("ghost", "go"));
    gated.actions.push_back(std::make_shared<VariableSetAction>("seen", "yes"));
    scenario.storyboard.stories.front()
        .acts.front()
        .groups.front()
        .maneuvers.front()
        .events.push_back(std::move(gated));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    for (int i = 0; i < 5; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    EXPECT_EQ(engine.variable("seen"), "no");
    int warnings = 0;
    for (const scena::Diagnostic& diagnostic : engine.diagnostics()) {
        if (diagnostic.message.find("traffic signal 'ghost'") != std::string::npos) {
            ++warnings;
        }
    }
    EXPECT_EQ(warnings, 1);
}

TEST(TrafficSignalTest, TrafficSignalControllerConditionFollowsThePhase) {
    Scenario scenario = make_scenario();
    scenario.variables["seen"] = "no";
    scenario.traffic_signal_controllers.push_back(
        make_controller("group1", "s1", {{"stop", 6.0}, {"go", 6.0}}));

    Event gated;
    gated.name = "gated";
    gated.start_trigger =
        make_trigger(std::make_shared<TrafficSignalControllerCondition>("group1", "go"));
    gated.actions.push_back(std::make_shared<VariableSetAction>("seen", "yes"));
    scenario.storyboard.stories.front()
        .acts.front()
        .groups.front()
        .maneuvers.front()
        .events.push_back(std::move(gated));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    ASSERT_EQ(engine.step(5.0), Status::Ok);
    EXPECT_EQ(engine.variable("seen"), "no");
    ASSERT_EQ(engine.step(1.5), Status::Ok);
    EXPECT_EQ(engine.variable("seen"), "yes");
}

TEST(TrafficSignalTest, ControllerConditionIsFalseBeforeADelayedStart) {
    // A controller waiting out its §6.11.3 delay has no phase, so a condition
    // naming its first phase is a deterministic false until it starts.
    Scenario scenario = make_scenario();
    scenario.variables["seen"] = "no";
    scenario.traffic_signal_controllers.push_back(make_controller("first", "s1", {{"a", 20.0}}));
    TrafficSignalController late = make_controller("late", "s2", {{"a", 20.0}});
    late.delay = 5.0;
    late.reference = "first";
    scenario.traffic_signal_controllers.push_back(std::move(late));

    Event gated;
    gated.name = "gated";
    gated.start_trigger =
        make_trigger(std::make_shared<TrafficSignalControllerCondition>("late", "a"));
    gated.actions.push_back(std::make_shared<VariableSetAction>("seen", "yes"));
    scenario.storyboard.stories.front()
        .acts.front()
        .groups.front()
        .maneuvers.front()
        .events.push_back(std::move(gated));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    ASSERT_EQ(engine.step(4.0), Status::Ok);
    EXPECT_EQ(engine.variable("seen"), "no");
    ASSERT_EQ(engine.step(1.5), Status::Ok);
    EXPECT_EQ(engine.variable("seen"), "yes");
}

TEST(TrafficSignalTest, TrafficSignalControllerConditionC712Validation) {
    Scenario unknown_controller = make_scenario();
    unknown_controller.traffic_signal_controllers.push_back(
        make_controller("group1", "s1", {{"go", 5.0}}));
    Event a;
    a.name = "gated";
    a.start_trigger =
        make_trigger(std::make_shared<TrafficSignalControllerCondition>("group9", "go"));
    a.actions.push_back(std::make_shared<VariableSetAction>("seen", "yes"));
    unknown_controller.variables["seen"] = "no";
    unknown_controller.storyboard.stories.front()
        .acts.front()
        .groups.front()
        .maneuvers.front()
        .events.push_back(std::move(a));
    Engine engine_a;
    EXPECT_EQ(engine_a.init(std::move(unknown_controller)), Status::SemanticError);
    EXPECT_TRUE(cites_rule(
        engine_a,
        "asam.net:xosc:1.0.0:reference_control.traffic_signal_controller_condition_references"));

    Scenario unknown_phase = make_scenario();
    unknown_phase.traffic_signal_controllers.push_back(
        make_controller("group1", "s1", {{"go", 5.0}}));
    unknown_phase.variables["seen"] = "no";
    Event b;
    b.name = "gated";
    b.start_trigger =
        make_trigger(std::make_shared<TrafficSignalControllerCondition>("group1", "amber"));
    b.actions.push_back(std::make_shared<VariableSetAction>("seen", "yes"));
    unknown_phase.storyboard.stories.front()
        .acts.front()
        .groups.front()
        .maneuvers.front()
        .events.push_back(std::move(b));
    Engine engine_b;
    EXPECT_EQ(engine_b.init(std::move(unknown_phase)), Status::SemanticError);
    EXPECT_TRUE(has_diagnostic(engine_b, Severity::Error, "has no phase 'amber'"));
}

TEST(TrafficSignalTest, SignalConditionsGateEventsThroughRisingEdge) {
    // The §11.12 shape in miniature: a controller action moves the cycle, a
    // rising-edge TrafficSignalControllerCondition releases the cross traffic,
    // and a forced broken state is picked up by a TrafficSignalCondition.
    Scenario scenario = make_scenario();
    scenario.variables["released"] = "no";
    scenario.variables["failure"] = "no";
    scenario.traffic_signal_controllers.push_back(
        make_controller("intersection", "straight", {{"stop", 30.0}, {"go", 20.0}}));

    add_event(scenario, "vehicle-at-stop-line", 2.0,
              std::make_shared<TrafficSignalControllerAction>("intersection", "go"));

    Event release;
    release.name = "release-cross-traffic";
    release.start_trigger =
        make_trigger(std::make_shared<TrafficSignalControllerCondition>("intersection", "go"),
                     scena::ir::ConditionEdge::Rising);
    release.actions.push_back(std::make_shared<VariableSetAction>("released", "yes"));
    scenario.storyboard.stories.front()
        .acts.front()
        .groups.front()
        .maneuvers.front()
        .events.push_back(std::move(release));

    add_event(scenario, "bulb-failure", 5.0,
              std::make_shared<TrafficSignalStateAction>("straight", "red;green"));

    Event notice;
    notice.name = "notice-failure";
    notice.start_trigger =
        make_trigger(std::make_shared<TrafficSignalCondition>("straight", "red;green"),
                     scena::ir::ConditionEdge::Rising);
    notice.actions.push_back(std::make_shared<VariableSetAction>("failure", "yes"));
    scenario.storyboard.stories.front()
        .acts.front()
        .groups.front()
        .maneuvers.front()
        .events.push_back(std::move(notice));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    EXPECT_EQ(engine.variable("released"), "no");
    for (int i = 0; i < 8; ++i) { // t = 4: past the controller action
        ASSERT_EQ(engine.step(0.5), Status::Ok);
    }
    EXPECT_EQ(engine.variable("released"), "yes");
    EXPECT_EQ(engine.variable("failure"), "no");
    for (int i = 0; i < 6; ++i) { // t = 7: past the forced failure
        ASSERT_EQ(engine.step(0.5), Status::Ok);
    }
    EXPECT_EQ(engine.variable("failure"), "yes");
    EXPECT_EQ(engine.traffic_signal_state("straight"), std::optional<std::string>("red;green"));
}

// --- Determinism -----------------------------------------------------------

TEST(TrafficSignalTest, PhaseTimingIsBitIdenticalAcrossRuns) {
    // The phase index is derived from simulation time with fmod rather than
    // accumulated, so it cannot drift with the host's step pattern.
    const auto build = []() {
        Scenario scenario = make_scenario();
        scenario.traffic_signal_controllers.push_back(
            make_controller("main", "s1", {{"stop", 7.0}, {"caution", 2.5}, {"go", 9.5}}));
        TrafficSignalController cross =
            make_controller("cross", "s2", {{"go", 9.5}, {"caution", 2.5}, {"stop", 7.0}});
        cross.delay = 3.25;
        cross.reference = "main";
        scenario.traffic_signal_controllers.push_back(std::move(cross));
        return scenario;
    };

    Engine engine_a;
    Engine engine_b;
    ASSERT_EQ(engine_a.init(build()), Status::Ok);
    ASSERT_EQ(engine_b.init(build()), Status::Ok);

    const double pattern[] = {0.05, 0.13, 0.09, 0.07};
    for (int i = 0; i < 400; ++i) {
        const double dt = pattern[i % 4];
        SCOPED_TRACE("step " + std::to_string(i));
        ASSERT_EQ(engine_a.step(dt), Status::Ok);
        ASSERT_EQ(engine_b.step(dt), Status::Ok);
        EXPECT_EQ(engine_a.traffic_signal_state("s1"), engine_b.traffic_signal_state("s1"));
        EXPECT_EQ(engine_a.traffic_signal_state("s2"), engine_b.traffic_signal_state("s2"));
        EXPECT_EQ(engine_a.traffic_signal_controller_phase("main"),
                  engine_b.traffic_signal_controller_phase("main"));
        EXPECT_EQ(engine_a.traffic_signal_controller_phase("cross"),
                  engine_b.traffic_signal_controller_phase("cross"));
    }
    // Guard against a dead scenario: both cycles really did run.
    EXPECT_TRUE(engine_a.traffic_signal_controller_phase("cross").has_value());
}

TEST(TrafficSignalTest, PhaseIsIndependentOfTheStepPattern) {
    // Two engines reaching the same simulation time through different step
    // sequences must be in the same phase — the arithmetic-clock guarantee.
    const auto build = []() {
        Scenario scenario = make_scenario();
        scenario.traffic_signal_controllers.push_back(
            make_controller("main", "s1", {{"stop", 7.0}, {"caution", 2.5}, {"go", 9.5}}));
        return scenario;
    };
    Engine coarse;
    Engine fine;
    ASSERT_EQ(coarse.init(build()), Status::Ok);
    ASSERT_EQ(fine.init(build()), Status::Ok);
    // Both sequences sum to exactly 20 s (4.0 and 0.125 are exact binary
    // fractions), so the clocks agree bit for bit and only the phase
    // derivation is under test.
    for (int i = 0; i < 5; ++i) {
        ASSERT_EQ(coarse.step(4.0), Status::Ok);
    }
    for (int i = 0; i < 160; ++i) {
        ASSERT_EQ(fine.step(0.125), Status::Ok);
    }
    ASSERT_EQ(coarse.time(), fine.time());
    EXPECT_EQ(coarse.traffic_signal_state("s1"), fine.traffic_signal_state("s1"));
    EXPECT_EQ(coarse.traffic_signal_controller_phase("main"),
              fine.traffic_signal_controller_phase("main"));
}

} // namespace
