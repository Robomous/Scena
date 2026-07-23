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
// The global actions at the engine level (p5-s6, ASAM OpenSCENARIO XML 1.4.0
// §7.4.2 and §7.4.3): variable and deprecated parameter actions over the
// runtime named-value stores, the add/delete entity lifecycle, the environment
// store and its time-of-day anchor, and the CustomCommandAction gateway
// callback. Every one of them "completes immediately" (Annex A Tables 11, 12).
//
// The traffic-signal half of the sprint lives in traffic_signal_test.cpp.

#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "scena/diagnostic.h"
#include "scena/engine.h"
#include "scena/entity_state.h"
#include "scena/gateway/simulator_gateway.h"
#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/date_time.h"
#include "scena/ir/dynamics.h"
#include "scena/ir/entity.h"
#include "scena/ir/entity_condition.h"
#include "scena/ir/environment.h"
#include "scena/ir/position.h"
#include "scena/ir/rule.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/trigger.h"

namespace {

using scena::Engine;
using scena::EntityState;
using scena::Severity;
using scena::Status;
using scena::ir::Act;
using scena::ir::Action;
using scena::ir::AddEntityAction;
using scena::ir::ControlMode;
using scena::ir::CustomCommandAction;
using scena::ir::DateTime;
using scena::ir::DeleteEntityAction;
using scena::ir::DynamicsDimension;
using scena::ir::DynamicsShape;
using scena::ir::Entity;
using scena::ir::Environment;
using scena::ir::EnvironmentAction;
using scena::ir::Event;
using scena::ir::Fog;
using scena::ir::LongitudinalDistanceAction;
using scena::ir::make_trigger;
using scena::ir::Maneuver;
using scena::ir::ManeuverGroup;
using scena::ir::ModifyOperator;
using scena::ir::ParameterCondition;
using scena::ir::ParameterModifyAction;
using scena::ir::ParameterSetAction;
using scena::ir::Precipitation;
using scena::ir::PrecipitationType;
using scena::ir::RelativeTargetSpeed;
using scena::ir::RoadCondition;
using scena::ir::Rule;
using scena::ir::Scenario;
using scena::ir::SimulationTimeCondition;
using scena::ir::SpeedAction;
using scena::ir::SpeedCondition;
using scena::ir::SpeedTargetValueType;
using scena::ir::Story;
using scena::ir::TeleportAction;
using scena::ir::TimeOfDay;
using scena::ir::TimeOfDayCondition;
using scena::ir::TransitionDynamics;
using scena::ir::TraveledDistanceCondition;
using scena::ir::TriggeringEntities;
using scena::ir::TriggeringEntitiesRule;
using scena::ir::VariableCondition;
using scena::ir::VariableModifyAction;
using scena::ir::VariableSetAction;
using scena::ir::Weather;
using scena::ir::Wind;
using scena::ir::WorldPosition;

constexpr double kTol = 1e-9;

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
    scenario.storyboard.stories.front()
        .acts.front()
        .groups.front()
        .maneuvers.front()
        .events.push_back(std::move(event));
}

/// True when some diagnostic of `severity` mentions `needle`.
bool has_diagnostic(const Engine& engine, Severity severity, const std::string& needle) {
    for (const scena::Diagnostic& diagnostic : engine.diagnostics()) {
        if (diagnostic.severity == severity &&
            diagnostic.message.find(needle) != std::string::npos) {
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
              std::make_shared<VariableModifyAction>("counter", ModifyOperator::Add,
                                                     std::numeric_limits<double>::infinity()));

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
    scenario.storyboard.stories.front()
        .acts.front()
        .groups.front()
        .maneuvers.front()
        .events.push_back(std::move(gated));

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
    scenario.storyboard.stories.front()
        .acts.front()
        .groups.front()
        .maneuvers.front()
        .events.push_back(std::move(gated));

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
    scenario.storyboard.stories.front()
        .acts.front()
        .groups.front()
        .maneuvers.front()
        .events.push_back(std::move(gated));

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
        scenario.storyboard.stories.front()
            .acts.front()
            .groups.front()
            .maneuvers.front()
            .events.push_back(std::move(gated));
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

// --- AddEntityAction / DeleteEntityAction (the active lifecycle) -----------

TEST(GlobalActionTest, DeleteEntityHidesStateAndConditions) {
    Scenario scenario = make_scenario();
    scenario.variables["seen"] = "no";
    scenario.init_actions.push_back(std::make_shared<SpeedAction>("ego", 10.0));
    add_event(scenario, "remove", 1.0, std::make_shared<DeleteEntityAction>("ego"));

    // A by-entity condition on the deleted entity must go deterministically
    // false rather than keep observing a stale snapshot. The threshold is one
    // the ego would pass at t = 5 s if it kept moving, and the delete lands at
    // t = 1 s — so the condition can only hold if the entity is still observed.
    Event gated;
    gated.name = "gated";
    gated.start_trigger = make_trigger(std::make_shared<TraveledDistanceCondition>(
        TriggeringEntities{TriggeringEntitiesRule::Any, {"ego"}}, 50.0));
    gated.actions.push_back(std::make_shared<VariableSetAction>("seen", "yes"));
    scenario.storyboard.stories.front()
        .acts.front()
        .groups.front()
        .maneuvers.front()
        .events.push_back(std::move(gated));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    ASSERT_TRUE(engine.state("ego").has_value());
    EXPECT_EQ(engine.entity_active("ego"), std::optional<bool>(true));

    ASSERT_EQ(engine.step(1.0), Status::Ok); // deletes the entity
    EXPECT_EQ(engine.entity_active("ego"), std::optional<bool>(false));
    EXPECT_FALSE(engine.state("ego").has_value());
    EXPECT_FALSE(engine.visibility_of("ego").has_value());
    EXPECT_FALSE(engine.controller_activation_of("ego").has_value());
    EXPECT_EQ(engine.route_of("ego"), nullptr);

    for (int i = 0; i < 20; ++i) {
        ASSERT_EQ(engine.step(0.5), Status::Ok);
    }
    // Well past t = 5 s: the odometer froze at the delete and the facet is
    // absent, so the condition never held.
    EXPECT_EQ(engine.variable("seen"), "no");
    // An id that was never declared is distinct from one that is inactive.
    EXPECT_FALSE(engine.entity_active("nobody").has_value());
}

TEST(GlobalActionTest, AddEntityWhenActiveIsNoOp) {
    // §EntityAction: "Adding an already active entity will have no effect" —
    // in particular it must not reset the entity's position or speed.
    Scenario scenario = make_scenario();
    scenario.init_actions.push_back(std::make_shared<SpeedAction>("ego", 10.0));
    add_event(scenario, "add-again", 1.0,
              std::make_shared<AddEntityAction>("ego", WorldPosition{500.0, 0.0, 0.0}));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    const std::optional<EntityState> state = engine.state("ego");
    ASSERT_TRUE(state.has_value());
    EXPECT_NEAR(state->x, 10.0, kTol); // moved by its own speed, not teleported
    EXPECT_NEAR(state->speed, 10.0, kTol);
}

TEST(GlobalActionTest, DeleteWhenInactiveIsNoOp) {
    Scenario scenario = make_scenario();
    add_event(scenario, "remove", 1.0, std::make_shared<DeleteEntityAction>("ego"));
    add_event(scenario, "remove-again", 2.0, std::make_shared<DeleteEntityAction>("ego"));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(engine.entity_active("ego"), std::optional<bool>(false));
    // A second delete is silent: no diagnostic at all.
    EXPECT_EQ(count_diagnostics(engine, "ego"), 0);
}

TEST(GlobalActionTest, DeleteThenAddRespawnsAtPositionWithFreshObservations) {
    Scenario scenario = make_scenario();
    scenario.init_actions.push_back(std::make_shared<SpeedAction>("ego", 20.0));
    add_event(scenario, "remove", 1.0, std::make_shared<DeleteEntityAction>("ego"));
    add_event(scenario, "restore", 2.0,
              std::make_shared<AddEntityAction>("ego", WorldPosition{-40.0, 7.0, 0.0}));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok); // deleted
    ASSERT_EQ(engine.step(1.0), Status::Ok); // re-added at the action's position

    const std::optional<EntityState> state = engine.state("ego");
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(engine.entity_active("ego"), std::optional<bool>(true));
    EXPECT_NEAR(state->x, -40.0, kTol);
    EXPECT_NEAR(state->y, 7.0, kTol);
    // A re-added entity starts from rest: the speed action that drove it was
    // released by the delete, and the fresh state carries no velocity.
    EXPECT_NEAR(state->speed, 0.0, kTol);

    // The next step must not report a phantom acceleration from the position
    // jump: the baseline was seeded at the add.
    ASSERT_EQ(engine.step(0.5), Status::Ok);
    const std::optional<EntityState> after = engine.state("ego");
    ASSERT_TRUE(after.has_value());
    EXPECT_NEAR(after->x, -40.0, kTol); // still at rest, nothing drives it
}

TEST(GlobalActionTest, AddEntitySeedsOrientationFromResolvedPose) {
    // AddEntity now accepts any §6.3.8 Position and seeds the full pose: a
    // world-frame target with heading/pitch/roll places the re-added entity
    // facing that way (ADR-0017).
    Scenario scenario = make_scenario();
    add_event(scenario, "remove", 1.0, std::make_shared<DeleteEntityAction>("ego"));
    add_event(
        scenario, "restore", 2.0,
        std::make_shared<AddEntityAction>("ego", WorldPosition{1.0, 2.0, 0.0, 0.6, 0.0, 0.0}));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok); // deleted
    ASSERT_EQ(engine.step(1.0), Status::Ok); // re-added with orientation

    const std::optional<EntityState> state = engine.state("ego");
    ASSERT_TRUE(state.has_value());
    EXPECT_NEAR(state->heading, 0.6, kTol);
}

TEST(GlobalActionTest, DeleteEntityStopsRunningPrivateActions) {
    // §7.5.2.2: a private action whose actor disappears is missing a
    // prerequisite and stops. Scena surfaces that as the owning event ending
    // one evaluation later (ADR-0015) — there are no per-action transitions.
    Scenario scenario = make_scenario();
    Event ramp;
    ramp.name = "ramp";
    ramp.start_trigger = make_trigger(std::make_shared<SimulationTimeCondition>(0.0));
    ramp.actions.push_back(std::make_shared<SpeedAction>(
        "ego", 30.0, TransitionDynamics{DynamicsShape::Linear, DynamicsDimension::Time, 20.0}));
    scenario.storyboard.stories.front()
        .acts.front()
        .groups.front()
        .maneuvers.front()
        .events.push_back(std::move(ramp));
    add_event(scenario, "remove", 1.0, std::make_shared<DeleteEntityAction>("ego"));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    EXPECT_EQ(engine.storyboard_element_state("story/act/group/maneuver/ramp"),
              scena::runtime::ElementState::Running);
    ASSERT_EQ(engine.step(1.0), Status::Ok); // delete fires
    ASSERT_EQ(engine.step(1.0), Status::Ok); // the ramp is re-polled and stops
    EXPECT_EQ(engine.storyboard_element_state("story/act/group/maneuver/ramp"),
              scena::runtime::ElementState::Complete);
    EXPECT_TRUE(has_diagnostic(engine, Severity::Warning, "targets inactive entity 'ego'"));
    // Warn-once: the ramp is re-polled every step until it drops out.
    EXPECT_EQ(count_diagnostics(engine, "targets inactive entity 'ego'"), 1);
}

TEST(GlobalActionTest, DeleteReferenceEntityCompletesDistanceKeeping) {
    // §7.5.2.1 names this case exactly: "if the referenced entity of an
    // instance of a LongitudinalDistanceAction disappears".
    Scenario scenario = make_scenario();
    Entity lead;
    lead.id = "lead";
    lead.name = "lead";
    lead.control_mode = ControlMode::EngineControlled;
    scenario.entities.push_back(std::move(lead));
    scenario.init_actions.push_back(
        std::make_shared<TeleportAction>("lead", WorldPosition{50.0, 0.0, 0.0}));
    scenario.init_actions.push_back(std::make_shared<SpeedAction>("lead", 10.0));
    scenario.init_actions.push_back(std::make_shared<SpeedAction>("ego", 10.0));

    Event keep;
    keep.name = "keep";
    keep.start_trigger = make_trigger(std::make_shared<SimulationTimeCondition>(0.0));
    keep.actions.push_back(std::make_shared<LongitudinalDistanceAction>(
        "ego", "lead", 20.0, std::nullopt, /*freespace=*/false, /*continuous=*/true));
    scenario.storyboard.stories.front()
        .acts.front()
        .groups.front()
        .maneuvers.front()
        .events.push_back(std::move(keep));
    add_event(scenario, "remove-lead", 1.0, std::make_shared<DeleteEntityAction>("lead"));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    ASSERT_EQ(engine.step(0.5), Status::Ok);
    // A continuous keeping action never ends on its own, so it is still running.
    EXPECT_EQ(engine.storyboard_element_state("story/act/group/maneuver/keep"),
              scena::runtime::ElementState::Running);
    ASSERT_EQ(engine.step(0.5), Status::Ok); // lead deleted
    ASSERT_EQ(engine.step(0.5), Status::Ok); // keeping re-polled, gives up
    EXPECT_EQ(engine.storyboard_element_state("story/act/group/maneuver/keep"),
              scena::runtime::ElementState::Complete);
    EXPECT_TRUE(has_diagnostic(engine, Severity::Warning, "no longer in the scenario"));
}

TEST(GlobalActionTest, DeleteReferenceEntityStopsRelativeSpeedTracking) {
    Scenario scenario = make_scenario();
    Entity lead;
    lead.id = "lead";
    lead.name = "lead";
    lead.control_mode = ControlMode::EngineControlled;
    scenario.entities.push_back(std::move(lead));
    scenario.init_actions.push_back(std::make_shared<SpeedAction>("lead", 12.0));

    Event track;
    track.name = "track";
    track.start_trigger = make_trigger(std::make_shared<SimulationTimeCondition>(0.0));
    track.actions.push_back(std::make_shared<SpeedAction>(
        "ego", RelativeTargetSpeed{"lead", 0.0, SpeedTargetValueType::Delta, /*continuous=*/true},
        TransitionDynamics{}));
    scenario.storyboard.stories.front()
        .acts.front()
        .groups.front()
        .maneuvers.front()
        .events.push_back(std::move(track));
    add_event(scenario, "remove-lead", 1.0, std::make_shared<DeleteEntityAction>("lead"));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(engine.storyboard_element_state("story/act/group/maneuver/track"),
              scena::runtime::ElementState::Complete);
    EXPECT_TRUE(has_diagnostic(engine, Severity::Warning, "is not in the scenario"));
    // The ego holds the speed it had; nothing commands it any more.
    ASSERT_TRUE(engine.state("ego").has_value());
    EXPECT_NEAR(engine.state("ego")->speed, 12.0, kTol);
}

/// Counts the per-entity gateway traffic, so the phase skips are observable.
class CountingGateway final : public scena::gateway::ISimulatorGateway {
public:
    void publish_state(const std::string& entity_id, const EntityState& state) override {
        (void)state;
        ++published[entity_id];
    }

    bool poll_state(const std::string& entity_id, EntityState& out) override {
        ++polled[entity_id];
        out = reported;
        return true;
    }

    scena::gateway::IRoadQuery* road_query() override { return nullptr; }

    std::map<std::string, int> published;
    std::map<std::string, int> polled;
    EntityState reported;
};

TEST(GlobalActionTest, InactiveEntitySkipsPollIntegratePublish) {
    Scenario scenario = make_scenario();
    Entity host_entity;
    host_entity.id = "host";
    host_entity.name = "host";
    host_entity.control_mode = ControlMode::HostControlled;
    scenario.entities.push_back(std::move(host_entity));
    scenario.init_actions.push_back(std::make_shared<SpeedAction>("ego", 10.0));
    add_event(scenario, "remove-ego", 1.0, std::make_shared<DeleteEntityAction>("ego"));
    add_event(scenario, "remove-host", 1.0, std::make_shared<DeleteEntityAction>("host"));

    CountingGateway gateway;
    gateway.reported.x = 3.0;
    Engine engine(&gateway);
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    // One step short of the delete, so the baseline traffic is real.
    ASSERT_EQ(engine.step(0.5), Status::Ok);
    EXPECT_GT(gateway.published["ego"], 0);
    EXPECT_GT(gateway.polled["host"], 0);

    // The step that fires the delete still polls: the host-state poll is
    // phase 2 and the storyboard is phase 3, so the entity is only removed
    // afterwards — and that same step must no longer publish it.
    const int published_at_delete = gateway.published["ego"];
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(gateway.published["ego"], published_at_delete);
    const int polled_after_delete = gateway.polled["host"];

    // Both entities are gone now: no poll, no integration, no publish.
    const std::optional<EntityState> frozen = engine.state("ego");
    EXPECT_FALSE(frozen.has_value());
    for (int i = 0; i < 3; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    EXPECT_EQ(gateway.published["ego"], published_at_delete);
    EXPECT_EQ(gateway.polled["host"], polled_after_delete);
}

TEST(GlobalActionTest, ReportStateOnInactiveEntityRejected) {
    Scenario scenario = make_scenario();
    Entity host_entity;
    host_entity.id = "host";
    host_entity.name = "host";
    host_entity.control_mode = ControlMode::HostControlled;
    scenario.entities.push_back(std::move(host_entity));
    add_event(scenario, "remove", 1.0, std::make_shared<DeleteEntityAction>("host"));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    EXPECT_EQ(engine.report_state("host", EntityState{}), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(engine.report_state("host", EntityState{}), Status::UnknownEntity);
}

TEST(GlobalActionTest, EntityActionUnknownRefFailsInit) {
    Scenario add_scenario = make_scenario();
    add_event(add_scenario, "add", 1.0,
              std::make_shared<AddEntityAction>("ghost", WorldPosition{}));
    Engine add_engine;
    EXPECT_EQ(add_engine.init(std::move(add_scenario)), Status::SemanticError);
    EXPECT_TRUE(has_diagnostic(add_engine, Severity::Error, "undeclared entity 'ghost'"));

    Scenario delete_scenario = make_scenario();
    add_event(delete_scenario, "remove", 1.0, std::make_shared<DeleteEntityAction>("ghost"));
    Engine delete_engine;
    EXPECT_EQ(delete_engine.init(std::move(delete_scenario)), Status::SemanticError);
    EXPECT_TRUE(has_diagnostic(delete_engine, Severity::Error, "undeclared entity 'ghost'"));
}

// --- EnvironmentAction (§Environment, the merge and the clock) -------------

TEST(GlobalActionTest, EnvironmentActionMergesPartialUpdate) {
    // §Environment / §Weather: "If one of the conditions is missing it means
    // that it doesn't change." Two partial updates must compose, not replace.
    Scenario scenario = make_scenario();

    Environment first;
    first.name = "dusk";
    Weather weather;
    weather.fog = Fog{120.0};
    weather.temperature = 288.0;
    first.weather = weather;
    first.road_condition = RoadCondition{0.8};
    add_event(scenario, "dusk", 1.0, std::make_shared<EnvironmentAction>(first));

    Environment second;
    Weather rain;
    rain.precipitation = Precipitation{PrecipitationType::Rain, 4.5};
    rain.temperature = 281.0; // this member does change
    second.weather = rain;
    add_event(scenario, "rain", 2.0, std::make_shared<EnvironmentAction>(second));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    EXPECT_FALSE(engine.environment().weather.has_value()); // nothing set yet

    ASSERT_EQ(engine.step(1.0), Status::Ok);
    ASSERT_TRUE(engine.environment().weather.has_value());
    ASSERT_TRUE(engine.environment().weather->fog.has_value());
    EXPECT_NEAR(engine.environment().weather->fog->visual_range, 120.0, kTol);
    ASSERT_TRUE(engine.environment().road_condition.has_value());
    EXPECT_NEAR(engine.environment().road_condition->friction_scale_factor, 0.8, kTol);

    ASSERT_EQ(engine.step(1.0), Status::Ok);
    const scena::ir::Environment& merged = engine.environment();
    // Carried over from the first action.
    EXPECT_EQ(merged.name, "dusk");
    ASSERT_TRUE(merged.weather->fog.has_value());
    EXPECT_NEAR(merged.weather->fog->visual_range, 120.0, kTol);
    ASSERT_TRUE(merged.road_condition.has_value());
    EXPECT_NEAR(merged.road_condition->friction_scale_factor, 0.8, kTol);
    // Added and updated by the second.
    ASSERT_TRUE(merged.weather->precipitation.has_value());
    EXPECT_EQ(merged.weather->precipitation->type, PrecipitationType::Rain);
    EXPECT_NEAR(merged.weather->precipitation->intensity, 4.5, kTol);
    ASSERT_TRUE(merged.weather->temperature.has_value());
    EXPECT_NEAR(*merged.weather->temperature, 281.0, kTol);
    // Never mentioned by either action.
    EXPECT_FALSE(merged.weather->wind.has_value());
    EXPECT_FALSE(merged.weather->sun.has_value());
}

TEST(GlobalActionTest, EnvironmentTimeOfDayAnimatedFeedsTimeOfDayCondition) {
    Scenario scenario = make_scenario();
    scenario.variables["seen"] = "no";

    Environment environment;
    environment.time_of_day =
        TimeOfDay{/*animation=*/true, DateTime{2026, 7, 22, 11, 59, 58, 0, 0}};
    scenario.init_actions.push_back(std::make_shared<EnvironmentAction>(environment));

    Event noon;
    noon.name = "noon";
    noon.start_trigger = make_trigger(std::make_shared<TimeOfDayCondition>(
        DateTime{2026, 7, 22, 12, 0, 0, 0, 0}, Rule::GreaterOrEqual));
    noon.actions.push_back(std::make_shared<VariableSetAction>("seen", "yes"));
    scenario.storyboard.stories.front()
        .acts.front()
        .groups.front()
        .maneuvers.front()
        .events.push_back(std::move(noon));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    ASSERT_TRUE(engine.date_time().has_value());
    const double anchor = *engine.date_time();
    EXPECT_EQ(engine.variable("seen"), "no");

    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_NEAR(*engine.date_time(), anchor + 1.0, kTol); // advances one-for-one
    EXPECT_EQ(engine.variable("seen"), "no");
    ASSERT_EQ(engine.step(1.5), Status::Ok);
    EXPECT_EQ(engine.variable("seen"), "yes"); // crossed noon
}

TEST(GlobalActionTest, EnvironmentTimeOfDayFrozenHoldsClock) {
    // §TimeOfDay animation="false": the simulated instant does not move, so a
    // TimeOfDayCondition past it never becomes true no matter how long the
    // scenario runs.
    Scenario scenario = make_scenario();
    scenario.variables["seen"] = "no";

    Environment environment;
    environment.time_of_day =
        TimeOfDay{/*animation=*/false, DateTime{2026, 7, 22, 11, 59, 58, 0, 0}};
    scenario.init_actions.push_back(std::make_shared<EnvironmentAction>(environment));

    Event noon;
    noon.name = "noon";
    noon.start_trigger = make_trigger(std::make_shared<TimeOfDayCondition>(
        DateTime{2026, 7, 22, 12, 0, 0, 0, 0}, Rule::GreaterOrEqual));
    noon.actions.push_back(std::make_shared<VariableSetAction>("seen", "yes"));
    scenario.storyboard.stories.front()
        .acts.front()
        .groups.front()
        .maneuvers.front()
        .events.push_back(std::move(noon));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    ASSERT_TRUE(engine.date_time().has_value());
    const double anchor = *engine.date_time();
    for (int i = 0; i < 20; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
        EXPECT_EQ(*engine.date_time(), anchor); // bit-identical, never moves
    }
    EXPECT_EQ(engine.variable("seen"), "no");
}

TEST(GlobalActionTest, HostDateTimeSetterResumesAnimationAfterAFrozenAnchor) {
    // The host setter always means an advancing clock, so it un-freezes an
    // anchor a non-animated TimeOfDay pinned.
    Scenario scenario = make_scenario();
    Environment environment;
    environment.time_of_day = TimeOfDay{/*animation=*/false, DateTime{2026, 7, 22, 8, 0, 0, 0, 0}};
    scenario.init_actions.push_back(std::make_shared<EnvironmentAction>(environment));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    const double frozen = *engine.date_time();
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(*engine.date_time(), frozen);

    ASSERT_EQ(engine.set_date_time(DateTime{2026, 7, 22, 9, 0, 0, 0, 0}), Status::Ok);
    const double resumed = *engine.date_time();
    ASSERT_EQ(engine.step(2.0), Status::Ok);
    EXPECT_NEAR(*engine.date_time(), resumed + 2.0, kTol);
}

TEST(GlobalActionTest, EnvironmentInvalidDateTimeFailsInitC24) {
    Scenario scenario = make_scenario();
    Environment environment;
    // February 30 does not exist in any year.
    environment.time_of_day = TimeOfDay{true, DateTime{2026, 2, 30, 12, 0, 0, 0, 0}};
    add_event(scenario, "bad-time", 1.0, std::make_shared<EnvironmentAction>(environment));

    Engine engine;
    EXPECT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    bool cited = false;
    for (const scena::Diagnostic& diagnostic : engine.diagnostics()) {
        if (diagnostic.rule_id == "asam.net:xosc:1.0.0:data_type.time_format") {
            cited = true;
        }
    }
    EXPECT_TRUE(cited);
}

TEST(GlobalActionTest, EnvironmentOutOfRangeWeatherFailsInit) {
    const auto reject = [](Environment environment) {
        Scenario scenario = make_scenario();
        add_event(scenario, "weather", 1.0,
                  std::make_shared<EnvironmentAction>(std::move(environment)));
        Engine engine;
        return engine.init(std::move(scenario));
    };

    Environment cold;
    Weather cold_weather;
    cold_weather.temperature = 100.0; // below the §Weather range [170..340] K
    cold.weather = cold_weather;
    EXPECT_EQ(reject(cold), Status::ValidationError);

    Environment oktas;
    Weather cloudy;
    cloudy.fractional_cloud_cover_oktas = 12; // §FractionalCloudCover is 0..9
    oktas.weather = cloudy;
    EXPECT_EQ(reject(oktas), Status::ValidationError);

    Environment fog;
    Weather foggy;
    foggy.fog = Fog{-5.0}; // §Fog visualRange is [0..inf[
    fog.weather = foggy;
    EXPECT_EQ(reject(fog), Status::ValidationError);

    Environment road;
    road.road_condition = RoadCondition{-1.0}; // [0..inf[
    EXPECT_EQ(reject(road), Status::ValidationError);
}

TEST(GlobalActionTest, EnvironmentStoreIsClearedByInitAndClose) {
    Scenario scenario = make_scenario();
    Environment environment;
    environment.name = "storm";
    Weather weather;
    weather.wind = Wind{1.5, 22.0};
    environment.weather = weather;
    scenario.init_actions.push_back(std::make_shared<EnvironmentAction>(environment));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    EXPECT_EQ(engine.environment().name, "storm");
    ASSERT_EQ(engine.close(), Status::Ok);
    EXPECT_TRUE(engine.environment().name.empty());
    EXPECT_FALSE(engine.environment().weather.has_value());

    // A scenario with no environment action leaves the store empty.
    ASSERT_EQ(engine.init(make_scenario()), Status::Ok);
    EXPECT_TRUE(engine.environment().name.empty());
}

// --- CustomCommandAction (§7.4.3) ------------------------------------------

/// A gateway that records the custom commands it is handed, in order.
class CommandGateway final : public scena::gateway::ISimulatorGateway {
public:
    void publish_state(const std::string& entity_id, const EntityState& state) override {
        (void)entity_id;
        (void)state;
    }

    bool poll_state(const std::string& entity_id, EntityState& out) override {
        (void)entity_id;
        (void)out;
        return false;
    }

    scena::gateway::IRoadQuery* road_query() override { return nullptr; }

    void on_custom_command(const std::string& type, const std::string& content) override {
        commands.push_back({type, content});
    }

    std::vector<std::pair<std::string, std::string>> commands;
};

TEST(GlobalActionTest, CustomCommandActionReachesGatewayInOrder) {
    Scenario scenario = make_scenario();
    scenario.init_actions.push_back(
        std::make_shared<CustomCommandAction>("script", "prepare.py --warm"));
    add_event(scenario, "first", 1.0, std::make_shared<CustomCommandAction>("log", "checkpoint A"));
    add_event(scenario, "second", 2.0,
              std::make_shared<CustomCommandAction>("log", "checkpoint B"));

    CommandGateway gateway;
    Engine engine(&gateway);
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    // The init action is handed over during init (§8.5), before any step.
    ASSERT_EQ(gateway.commands.size(), 1U);
    EXPECT_EQ(gateway.commands[0].first, "script");
    EXPECT_EQ(gateway.commands[0].second, "prepare.py --warm");

    ASSERT_EQ(engine.step(1.0), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    ASSERT_EQ(gateway.commands.size(), 3U);
    // Verbatim and in the order the storyboard fired them.
    EXPECT_EQ(gateway.commands[1].second, "checkpoint A");
    EXPECT_EQ(gateway.commands[2].second, "checkpoint B");

    // Each action fires once, so a further step hands nothing over.
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(gateway.commands.size(), 3U);
}

TEST(GlobalActionTest, CustomCommandActionWithoutGatewayIsSilentNoOp) {
    // §7.4.3 makes executability depend on the environment recognizing the
    // action, so no host means no effect — and no diagnostic.
    Scenario scenario = make_scenario();
    add_event(scenario, "command", 1.0, std::make_shared<CustomCommandAction>("script", "noop"));

    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(engine.diagnostics().size(), 0U);
    // The action still completed, so its event ended.
    EXPECT_EQ(engine.storyboard_element_state("story/act/group/maneuver/command"),
              scena::runtime::ElementState::Complete);
}

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
