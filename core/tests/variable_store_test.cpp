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

// The runtime variable store (§6.12): variables are scenario state that
// changes while the scenario runs, unlike parameters, which are fixed at load
// time. This suite covers the store itself — seeding, host writes, action
// writes, and what happens to an undeclared name.

#include <memory>
#include <string>

#include <gtest/gtest.h>

#include "scena/engine.h"
#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/rule.h"
#include "scena/ir/scenario.h"
#include "scena/ir/trigger.h"
#include "scena/runtime/scheduler.h"

namespace {

using scena::Status;

/// A scenario with the given variables and one entity, and no storyboard
/// content beyond what a test adds.
scena::ir::Scenario make_scenario() {
    scena::ir::Scenario scenario;
    scenario.name = "variables";
    scenario.entities.push_back(scena::ir::Entity{"ego", "ego"});
    return scenario;
}

/// Runs one init action and returns the engine, initialized.
void init_with(scena::Engine& engine, scena::ir::Scenario scenario) {
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
}

/// Builds a one-event storyboard whose trigger is `condition` and whose only
/// action sets "observed" to "yes", so the event's state reports whether the
/// condition saw what the test expects.
void add_probe_event(scena::ir::Scenario& scenario,
                     std::shared_ptr<scena::ir::Condition> condition) {
    scena::ir::Event event;
    event.name = "probe";
    event.start_trigger = scena::ir::make_trigger(std::move(condition));
    event.actions.push_back(std::make_shared<scena::ir::VariableSetAction>("observed", "yes"));

    scena::ir::Maneuver maneuver;
    maneuver.name = "m";
    maneuver.events.push_back(std::move(event));
    scena::ir::ManeuverGroup group;
    group.name = "g";
    group.actors.emplace_back("ego");
    group.maneuvers.push_back(std::move(maneuver));
    scena::ir::Act act;
    act.name = "a";
    act.groups.push_back(std::move(group));
    scena::ir::Story story;
    story.name = "s";
    story.acts.push_back(std::move(act));
    scenario.storyboard.stories.push_back(std::move(story));
    scenario.variables["observed"] = "no";
}

TEST(VariableStore, DeclarationsAreSeededAtInit) {
    scena::ir::Scenario scenario = make_scenario();
    scenario.variables["counter"] = "3";
    add_probe_event(scenario, std::make_shared<scena::ir::VariableCondition>(
                                  "counter", scena::ir::Rule::EqualTo, "3"));

    scena::Engine engine;
    init_with(engine, std::move(scenario));
    ASSERT_EQ(engine.step(0.1), Status::Ok);
    // The declared value is in the store before the first step, so a
    // condition comparing against it is true immediately.
    EXPECT_EQ(engine.storyboard_element_state("s/a/g/m/probe"),
              scena::runtime::ElementState::Complete);
}

TEST(VariableStore, AnUndeclaredNameIsRejected) {
    scena::Engine engine;
    init_with(engine, make_scenario());
    // Host API misuse, not a scenario defect: the name was never declared.
    EXPECT_EQ(engine.set_variable("ghost", "1"), Status::UnknownName);
}

TEST(VariableStore, WritesAreRejectedBeforeInit) {
    scena::Engine engine;
    EXPECT_EQ(engine.set_variable("counter", "1"), Status::NotInitialized);
}

TEST(VariableStore, AHostWriteIsObservedByTheNextEvaluation) {
    scena::ir::Scenario scenario = make_scenario();
    scenario.variables["armed"] = "no";
    add_probe_event(scenario, std::make_shared<scena::ir::VariableCondition>(
                                  "armed", scena::ir::Rule::EqualTo, "yes"));

    scena::Engine engine;
    init_with(engine, std::move(scenario));
    ASSERT_EQ(engine.step(0.1), Status::Ok);
    EXPECT_EQ(engine.storyboard_element_state("s/a/g/m/probe"),
              scena::runtime::ElementState::Standby);

    ASSERT_EQ(engine.set_variable("armed", "yes"), Status::Ok);
    ASSERT_EQ(engine.step(0.1), Status::Ok);
    EXPECT_EQ(engine.storyboard_element_state("s/a/g/m/probe"),
              scena::runtime::ElementState::Complete);
}

TEST(VariableStore, ASetActionWritesTheStore) {
    scena::ir::Scenario scenario = make_scenario();
    scenario.variables["counter"] = "0";
    scenario.init_actions.push_back(std::make_shared<scena::ir::VariableSetAction>("counter", "7"));
    add_probe_event(scenario, std::make_shared<scena::ir::VariableCondition>(
                                  "counter", scena::ir::Rule::EqualTo, "7"));

    scena::Engine engine;
    init_with(engine, std::move(scenario));
    ASSERT_EQ(engine.step(0.1), Status::Ok);
    EXPECT_EQ(engine.storyboard_element_state("s/a/g/m/probe"),
              scena::runtime::ElementState::Complete);
}

TEST(VariableStore, AModifyActionAppliesItsOperator) {
    scena::ir::Scenario scenario = make_scenario();
    scenario.variables["counter"] = "10";
    scenario.init_actions.push_back(std::make_shared<scena::ir::VariableModifyAction>(
        "counter", scena::ir::ModifyOperator::Add, 5.0));
    // 10 + 5 stored back through the shortest round-tripping decimal, so the
    // comparison against "15" is exact rather than "15.000000".
    add_probe_event(scenario, std::make_shared<scena::ir::VariableCondition>(
                                  "counter", scena::ir::Rule::EqualTo, "15"));

    scena::Engine engine;
    init_with(engine, std::move(scenario));
    ASSERT_EQ(engine.step(0.1), Status::Ok);
    EXPECT_EQ(engine.storyboard_element_state("s/a/g/m/probe"),
              scena::runtime::ElementState::Complete);
}

TEST(VariableStore, VariablesAreScenarioStateNotLoadTimeConstants) {
    // The distinction from a parameter: a variable's value changes while the
    // scenario runs, so the same condition flips from false to true within
    // one run.
    scena::ir::Scenario scenario = make_scenario();
    scenario.variables["phase"] = "idle";
    add_probe_event(scenario, std::make_shared<scena::ir::VariableCondition>(
                                  "phase", scena::ir::Rule::EqualTo, "running"));

    scena::Engine engine;
    init_with(engine, std::move(scenario));
    for (int step = 0; step < 3; ++step) {
        ASSERT_EQ(engine.step(0.1), Status::Ok);
        EXPECT_EQ(engine.storyboard_element_state("s/a/g/m/probe"),
                  scena::runtime::ElementState::Standby);
    }
    ASSERT_EQ(engine.set_variable("phase", "running"), Status::Ok);
    ASSERT_EQ(engine.step(0.1), Status::Ok);
    EXPECT_EQ(engine.storyboard_element_state("s/a/g/m/probe"),
              scena::runtime::ElementState::Complete);
}

} // namespace
