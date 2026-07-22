// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
//
// The global actions at the engine level (p5-s6, ASAM OpenSCENARIO XML 1.4.0
// §7.4.2 and §7.4.3): variable and deprecated parameter actions over the
// runtime named-value stores, the add/delete entity lifecycle, the environment
// store and its time-of-day anchor, and the CustomCommandAction gateway
// callback. Every one of them "completes immediately" (Annex A Tables 11, 12).
//
// The traffic-signal half of the sprint lives in traffic_signal_test.cpp.

#include <memory>
#include <limits>
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
#include "scena/ir/rule.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"
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
using scena::ir::ModifyOperator;
using scena::ir::ParameterCondition;
using scena::ir::ParameterModifyAction;
using scena::ir::ParameterSetAction;
using scena::ir::Rule;
using scena::ir::Scenario;
using scena::ir::SimulationTimeCondition;
using scena::ir::Story;
using scena::ir::VariableCondition;
using scena::ir::VariableModifyAction;
using scena::ir::VariableSetAction;

/// A scenario with one engine-controlled entity and a single-maneuver
/// storyboard, the shape every test here builds on.
Scenario make_scenario() {
    Scenario scenario;
    scenario.name = "global-actions";
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

/// Appends an event firing `action` at `at_time`, into the fixture's maneuver.
void add_event(Scenario& scenario, std::string name, double at_time,
               std::shared_ptr<Action> action) {
    Event event;
    event.name = std::move(name);
    event.start_trigger = make_trigger(std::make_shared<SimulationTimeCondition>(at_time));
    event.actions.push_back(std::move(action));
    scenario.storyboard.stories.front().acts.front().groups.front().maneuvers.front().events
        .push_back(std::move(event));
}

/// True when some diagnostic of `severity` mentions `needle`.
bool has_diagnostic(const Engine& engine, Severity severity, const std::string& needle) {
    for (const scena::Diagnostic& diagnostic : engine.diagnostics()) {
        if (diagnostic.severity == severity && diagnostic.message.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

/// Number of diagnostics whose message mentions `needle` (warn-once checks).
int count_diagnostics(const Engine& engine, const std::string& needle) {
    int count = 0;
    for (const scena::Diagnostic& diagnostic : engine.diagnostics()) {
        if (diagnostic.message.find(needle) != std::string::npos) {
            ++count;
        }
    }
    return count;
}

// --- VariableSetAction / VariableModifyAction (§6.12) ----------------------

TEST(GlobalActionTest, VariableSetActionUpdatesRuntimeStore) {
    Scenario scenario = make_scenario();
    scenario.variables["trigger"] = "false";
    add_event(scenario, "set", 1.0, std::make_shared<VariableSetAction>("trigger", "true"));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    EXPECT_EQ(engine.variable("trigger"), "false");
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(engine.variable("trigger"), "true");
}

TEST(GlobalActionTest, VariableModifyAddIsExact) {
    Scenario scenario = make_scenario();
    // 0.1 + 0.2 is the canonical IEEE case: the store must hold the exact
    // double, not a rounded rendering of it.
    scenario.variables["counter"] = "0.1";
    add_event(scenario, "add", 1.0,
              std::make_shared<VariableModifyAction>("counter", ModifyOperator::Add, 0.2));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    const std::optional<std::string> stored = engine.variable("counter");
    ASSERT_TRUE(stored.has_value());
    EXPECT_EQ(*stored, "0.30000000000000004");
    const std::optional<double> parsed = scena::ir::parse_scalar(*stored);
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(*parsed, 0.1 + 0.2);
}

TEST(GlobalActionTest, VariableModifyMultiplyIsExact) {
    Scenario scenario = make_scenario();
    scenario.variables["gain"] = "2.5";
    add_event(scenario, "scale", 1.0,
              std::make_shared<VariableModifyAction>("gain", ModifyOperator::Multiply, 4.0));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(engine.variable("gain"), "10");
}

TEST(GlobalActionTest, VariableModifyNonNumericWarnsC26AndNoOps) {
    Scenario scenario = make_scenario();
    scenario.variables["mode"] = "cruise";
    // Two firings of the same action: the C.2.6 warning is emitted once.
    add_event(scenario, "modify", 1.0,
              std::make_shared<VariableModifyAction>("mode", ModifyOperator::Add, 1.0));
    add_event(scenario, "modify-again", 2.0,
              std::make_shared<VariableModifyAction>("mode", ModifyOperator::Add, 1.0));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(engine.variable("mode"), "cruise"); // untouched
    EXPECT_TRUE(has_diagnostic(engine, Severity::Warning, "non-numeric value 'mode'"));
    EXPECT_EQ(count_diagnostics(engine, "non-numeric value 'mode'"), 1);
}

TEST(GlobalActionTest, VariableActionUnknownRefFailsInit) {
    Scenario scenario = make_scenario();
    add_event(scenario, "set", 1.0, std::make_shared<VariableSetAction>("missing", "1"));

    Engine engine;
    EXPECT_EQ(engine.init(std::move(scenario)), Status::SemanticError);
    EXPECT_TRUE(has_diagnostic(engine, Severity::Error, "variable 'missing' is not declared"));
    // The rule id is the same one the VariableCondition cites.
    bool cited = false;
    for (const scena::Diagnostic& diagnostic : engine.diagnostics()) {
        if (diagnostic.rule_id ==
            "asam.net:xosc:1.2.0:reference_control.resolvable_variable_reference") {
            cited = true;
        }
    }
    EXPECT_TRUE(cited);
}

TEST(GlobalActionTest, VariableModifyNonFiniteValueFailsInit) {
    Scenario scenario = make_scenario();
    scenario.variables["counter"] = "1";
    add_event(scenario, "modify", 1.0,
              std::make_shared<VariableModifyAction>(
                  "counter", ModifyOperator::Add, std::numeric_limits<double>::infinity()));

    Engine engine;
    EXPECT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
}

TEST(GlobalActionTest, VariableActionVisibleToVariableCondition) {
    Scenario scenario = make_scenario();
    scenario.variables["armed"] = "false";
    add_event(scenario, "arm", 1.0, std::make_shared<VariableSetAction>("armed", "true"));

    // A second event gated on the variable the first one writes — the §6.12
    // "trigger actions with a VariableCondition" loop, closed inside the
    // scenario rather than through the host.
    Event gated;
    gated.name = "gated";
    gated.start_trigger =
        make_trigger(std::make_shared<VariableCondition>("armed", Rule::EqualTo, "true"));
    gated.actions.push_back(std::make_shared<VariableSetAction>("armed", "consumed"));
    scenario.storyboard.stories.front().acts.front().groups.front().maneuvers.front().events
        .push_back(std::move(gated));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok); // "arm" fires, writing "true"
    // Both events are evaluated in the same walk; whether the gated one sees
    // the write in this step or the next, it must fire without any host help.
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(engine.variable("armed"), "consumed");
}

// --- Deprecated parameter actions (the 1.0/1.1 overlay) --------------------

TEST(GlobalActionTest, ParameterSetActionOverlaysImmutableDeclaration) {
    Scenario scenario = make_scenario();
    scenario.parameters["speedLimit"] = "30";
    add_event(scenario, "raise", 1.0, std::make_shared<ParameterSetAction>("speedLimit", "50"));

    // A ParameterCondition on the same name observes the overlay, which is the
    // whole point of executing a deprecated action instead of rejecting it.
    Event gated;
    gated.name = "gated";
    gated.start_trigger =
        make_trigger(std::make_shared<ParameterCondition>("speedLimit", Rule::EqualTo, "50"));
    gated.actions.push_back(std::make_shared<VariableSetAction>("seen", "yes"));
    scenario.variables["seen"] = "no";
    scenario.storyboard.stories.front().acts.front().groups.front().maneuvers.front().events
        .push_back(std::move(gated));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(engine.variable("seen"), "yes");
    EXPECT_TRUE(has_diagnostic(engine, Severity::Warning, "deprecated with version 1.2"));
}

TEST(GlobalActionTest, ParameterModifyActionOnOverlay) {
    Scenario scenario = make_scenario();
    scenario.parameters["gap"] = "8";
    // The first modify reads the declared §9.1 value into the overlay.
    add_event(scenario, "widen", 1.0,
              std::make_shared<ParameterModifyAction>("gap", ModifyOperator::Add, 4.0));
    add_event(scenario, "widen-more", 2.0,
              std::make_shared<ParameterModifyAction>("gap", ModifyOperator::Multiply, 2.0));

    Event gated;
    gated.name = "gated";
    gated.start_trigger =
        make_trigger(std::make_shared<ParameterCondition>("gap", Rule::EqualTo, "24"));
    gated.actions.push_back(std::make_shared<VariableSetAction>("seen", "yes"));
    scenario.variables["seen"] = "no";
    scenario.storyboard.stories.front().acts.front().groups.front().maneuvers.front().events
        .push_back(std::move(gated));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    for (int i = 0; i < 4; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    EXPECT_EQ(engine.variable("seen"), "yes");
}

TEST(GlobalActionTest, ParameterActionUnknownRefFailsInit) {
    Scenario scenario = make_scenario();
    add_event(scenario, "set", 1.0, std::make_shared<ParameterSetAction>("nope", "1"));

    Engine engine;
    EXPECT_EQ(engine.init(std::move(scenario)), Status::SemanticError);
    EXPECT_TRUE(has_diagnostic(engine, Severity::Error, "parameter 'nope' is not declared"));
}

TEST(GlobalActionTest, ParameterOverlayResetOnReInit) {
    // The overlay is per-run state: re-initializing the same engine restores
    // the declared §9.1 value.
    const auto build = []() {
        Scenario scenario = make_scenario();
        scenario.parameters["speedLimit"] = "30";
        scenario.variables["seen"] = "no";
        add_event(scenario, "raise", 1.0, std::make_shared<ParameterSetAction>("speedLimit", "50"));
        Event gated;
        gated.name = "gated";
        gated.start_trigger =
            make_trigger(std::make_shared<ParameterCondition>("speedLimit", Rule::EqualTo, "30"));
        gated.actions.push_back(std::make_shared<VariableSetAction>("seen", "declared"));
        scenario.storyboard.stories.front().acts.front().groups.front().maneuvers.front().events
            .push_back(std::move(gated));
        return scenario;
    };

    Engine engine;
    ASSERT_EQ(engine.init(build()), Status::Ok);
    // At t = 0 the declaration still holds, so the gated event fires at once.
    EXPECT_EQ(engine.variable("seen"), "declared");
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    ASSERT_EQ(engine.close(), Status::Ok);

    ASSERT_EQ(engine.init(build()), Status::Ok);
    EXPECT_EQ(engine.variable("seen"), "declared");
}

// --- Init phase and the unknown-kind fallback ------------------------------

TEST(GlobalActionTest, GlobalActionsRunInInitPhase) {
    // §8.5: init actions are applied before simulation time starts, and a
    // global action carries no actor, so the actor-existence check must not
    // reject it.
    Scenario scenario = make_scenario();
    scenario.variables["phase"] = "start";
    scenario.parameters["mode"] = "a";
    scenario.init_actions.push_back(std::make_shared<VariableSetAction>("phase", "init"));
    scenario.init_actions.push_back(std::make_shared<ParameterSetAction>("mode", "b"));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    EXPECT_EQ(engine.variable("phase"), "init");
}

/// A global action the engine does not implement, to exercise the fallback.
class UnknownGlobalAction final : public scena::ir::GlobalAction {
public:
    [[nodiscard]] std::string_view kind() const noexcept override { return "TrafficSourceAction"; }
};

TEST(GlobalActionTest, UnknownGlobalKindWarnsWithActionPath) {
    Scenario scenario = make_scenario();
    add_event(scenario, "spawn", 1.0, std::make_shared<UnknownGlobalAction>());

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    bool found = false;
    for (const scena::Diagnostic& diagnostic : engine.diagnostics()) {
        if (diagnostic.path == "actions/TrafficSourceAction") {
            found = true;
            EXPECT_EQ(diagnostic.severity, Severity::Warning);
            EXPECT_EQ(diagnostic.code, Status::UnsupportedFeature);
            // No entity is named: an actor-less action has none.
            EXPECT_EQ(diagnostic.message.find("entity"), std::string::npos);
        }
    }
    EXPECT_TRUE(found);
}

} // namespace
