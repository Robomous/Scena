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
// followingMode=follow, DynamicConstraints and entity-relative speed profiles
// (#62, ADR-0024): the jerk-aware duration math, the shape a follow-mode
// transition is realised with, the precedence of a SpeedProfileAction's
// DynamicConstraints over the actor's Performance envelope, and the advisory
// maxSpeed rules. Position mode is covered by longitudinal_test.cpp and is
// asserted here only where the two must differ.

#include <cmath>
#include <limits>
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
#include "scena/ir/dynamics.h"
#include "scena/ir/entity.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/trigger.h"
#include "scena/runtime/longitudinal.h"

namespace {

using scena::Engine;
using scena::EntityState;
using scena::Severity;
using scena::Status;
using scena::ir::ControlMode;
using scena::ir::DynamicConstraints;
using scena::ir::DynamicsDimension;
using scena::ir::DynamicsShape;
using scena::ir::Entity;
using scena::ir::FollowingMode;
using scena::ir::Performance;
using scena::ir::Scenario;
using scena::ir::SimulationTimeCondition;
using scena::ir::SpeedAction;
using scena::ir::SpeedProfileAction;
using scena::ir::SpeedProfileEntry;
using scena::ir::TransitionDynamics;
using scena::ir::Vehicle;

constexpr double kTol = 1e-9;
constexpr double kInf = std::numeric_limits<double>::infinity();

double speed_of(const Engine& engine, const std::string& id) {
    const std::optional<EntityState> state = engine.state(id);
    return state.has_value() ? state->speed : std::nan("");
}

Entity make_entity(std::string id, std::optional<Performance> perf) {
    Entity entity;
    entity.id = id;
    entity.name = std::move(id);
    entity.control_mode = ControlMode::EngineControlled;
    if (perf.has_value()) {
        Vehicle vehicle;
        vehicle.performance = *perf;
        entity.object = vehicle;
    }
    return entity;
}

/// A one-event scenario whose single action fires at t = 0.
Scenario wrap(std::vector<Entity> entities, std::shared_ptr<scena::ir::Action> action,
              std::vector<std::shared_ptr<scena::ir::Action>> init_actions = {}) {
    Scenario scenario;
    scenario.name = "follow-mode";
    scenario.entities = std::move(entities);
    scenario.init_actions = std::move(init_actions);

    scena::ir::Event event;
    event.name = "event";
    event.actions.push_back(std::move(action));
    scena::ir::Maneuver maneuver;
    maneuver.name = "maneuver";
    maneuver.events.push_back(std::move(event));
    scena::ir::ManeuverGroup group;
    group.name = "group";
    group.maneuvers.push_back(std::move(maneuver));
    scena::ir::Act act;
    act.name = "act";
    act.groups.push_back(std::move(group));
    scena::ir::Story story;
    story.name = "story";
    story.acts.push_back(std::move(act));
    scenario.storyboard.stories.push_back(std::move(story));
    return scenario;
}

// --- the shape factors -----------------------------------------------------

TEST(FollowModeShapeTest, JerkFactorIsFiniteOnlyForTheSmoothShapes) {
    EXPECT_DOUBLE_EQ(scena::runtime::shape_peak_jerk_factor(DynamicsShape::Cubic), 6.0);
    // pi^2/2, formed exactly as the implementation forms it.
    const double pi = 3.14159265358979311599796346854;
    EXPECT_DOUBLE_EQ(scena::runtime::shape_peak_jerk_factor(DynamicsShape::Sinusoidal),
                     pi * pi * 0.5);
    // Linear and Step step their gradient at the endpoints: no finite duration
    // bounds their jerk, so no jerk limit can be met by stretching them.
    EXPECT_FALSE(std::isfinite(scena::runtime::shape_peak_jerk_factor(DynamicsShape::Linear)));
    EXPECT_FALSE(std::isfinite(scena::runtime::shape_peak_jerk_factor(DynamicsShape::Step)));
}

TEST(FollowModeShapeTest, FollowShapeHasZeroGradientAtBothEnds) {
    // §SpeedProfileAction: "for mode=follow the acceleration is zero at the
    // start and end of the profile".
    EXPECT_EQ(scena::runtime::follow_shape(DynamicsShape::Linear), DynamicsShape::Cubic);
    EXPECT_EQ(scena::runtime::follow_shape(DynamicsShape::Step), DynamicsShape::Cubic);
    EXPECT_EQ(scena::runtime::follow_shape(DynamicsShape::Cubic), DynamicsShape::Cubic);
    // Sinusoidal already satisfies the requirement and is kept as authored.
    EXPECT_EQ(scena::runtime::follow_shape(DynamicsShape::Sinusoidal), DynamicsShape::Sinusoidal);
    for (const DynamicsShape shape : {DynamicsShape::Linear, DynamicsShape::Step,
                                      DynamicsShape::Cubic, DynamicsShape::Sinusoidal}) {
        const DynamicsShape realised = scena::runtime::follow_shape(shape);
        EXPECT_TRUE(std::isfinite(scena::runtime::shape_peak_jerk_factor(realised)));
    }
}

// --- constrained_duration --------------------------------------------------

TEST(FollowModeDurationTest, AccelerationLimitStretchesTheAuthoredDuration) {
    // Cubic peak gradient 1.5: T >= 1.5 * 10 / 2 = 7.5 s.
    EXPECT_DOUBLE_EQ(
        scena::runtime::constrained_duration(DynamicsShape::Cubic, 10.0, 2.0, kInf, 2.0), 7.5);
}

TEST(FollowModeDurationTest, JerkLimitStretchesTheAuthoredDuration) {
    // Cubic peak curvature 6: T >= sqrt(6 * 10 / 3) = sqrt(20).
    EXPECT_DOUBLE_EQ(
        scena::runtime::constrained_duration(DynamicsShape::Cubic, 10.0, kInf, 3.0, 0.0),
        std::sqrt(20.0));
}

TEST(FollowModeDurationTest, TheTightestLimitWins) {
    // Acceleration asks for 7.5 s, jerk for sqrt(20) ~ 4.47 s, the author for 2 s.
    EXPECT_DOUBLE_EQ(
        scena::runtime::constrained_duration(DynamicsShape::Cubic, 10.0, 2.0, 3.0, 2.0), 7.5);
    // Loosen the acceleration limit and the jerk bound takes over.
    EXPECT_DOUBLE_EQ(
        scena::runtime::constrained_duration(DynamicsShape::Cubic, 10.0, 100.0, 3.0, 2.0),
        std::sqrt(20.0));
}

TEST(FollowModeDurationTest, ConstraintsNeverSpeedATransitionUp) {
    // The authored duration is a floor: an unreachably loose limit leaves it be.
    EXPECT_DOUBLE_EQ(
        scena::runtime::constrained_duration(DynamicsShape::Cubic, 10.0, 1e6, 1e6, 12.0), 12.0);
    // No limits at all, no authored duration: instantaneous.
    EXPECT_DOUBLE_EQ(
        scena::runtime::constrained_duration(DynamicsShape::Cubic, 10.0, kInf, kInf, 0.0), 0.0);
    // Nothing to acquire.
    EXPECT_DOUBLE_EQ(scena::runtime::constrained_duration(DynamicsShape::Cubic, 0.0, 1.0, 1.0, 0.0),
                     0.0);
}

TEST(FollowModeDurationTest, AnUnboundedShapeIgnoresTheJerkLimitRatherThanDivergent) {
    // Linear has an infinite curvature factor; the jerk term is skipped rather
    // than producing an infinite (or NaN) duration. Reached only defensively —
    // follow_shape keeps Linear out of follow mode.
    const double duration =
        scena::runtime::constrained_duration(DynamicsShape::Linear, 10.0, 2.0, 3.0, 0.0);
    EXPECT_TRUE(std::isfinite(duration));
    EXPECT_DOUBLE_EQ(duration, 5.0); // acceleration term only: 1.0 * 10 / 2
}

// --- SpeedAction with followingMode=follow ----------------------------------

std::shared_ptr<SpeedAction> follow_speed_action(double target, DynamicsShape shape,
                                                 DynamicsDimension dimension, double value) {
    TransitionDynamics dynamics;
    dynamics.shape = shape;
    dynamics.dimension = dimension;
    dynamics.value = value;
    dynamics.following_mode = FollowingMode::Follow;
    return std::make_shared<SpeedAction>("ego", target, dynamics);
}

TEST(FollowModeSpeedActionTest, FollowModeStretchesForTheAccelerationEnvelope) {
    // 0 -> 10 m/s authored over 2 s, but max_acceleration is 2 m/s^2 and the
    // realised shape is Cubic: T = 1.5 * 10 / 2 = 7.5 s.
    const Performance perf{60.0, 2.0, 2.0, std::nullopt, std::nullopt};
    Engine engine;
    ASSERT_EQ(engine.init(wrap(
                  {make_entity("ego", perf)},
                  follow_speed_action(10.0, DynamicsShape::Linear, DynamicsDimension::Time, 2.0))),
              Status::Ok);
    ASSERT_EQ(engine.step(3.75), Status::Ok);        // half the stretched span
    EXPECT_NEAR(speed_of(engine, "ego"), 5.0, kTol); // Cubic g(0.5) = 0.5
    ASSERT_EQ(engine.step(3.75), Status::Ok);
    EXPECT_DOUBLE_EQ(speed_of(engine, "ego"), 10.0);
}

TEST(FollowModeSpeedActionTest, FollowModeStretchesForTheJerkEnvelope) {
    // Acceleration is effectively unbounded (100 m/s^2), the jerk rate limit is
    // 3 m/s^3: T = sqrt(6 * 10 / 3) = sqrt(20).
    const Performance perf{60.0, 100.0, 100.0, 3.0, 3.0};
    Engine engine;
    ASSERT_EQ(engine.init(wrap(
                  {make_entity("ego", perf)},
                  follow_speed_action(10.0, DynamicsShape::Linear, DynamicsDimension::Time, 1.0))),
              Status::Ok);
    const double span = std::sqrt(20.0);
    ASSERT_EQ(engine.step(span * 0.5), Status::Ok);
    EXPECT_NEAR(speed_of(engine, "ego"), 5.0, kTol);
    ASSERT_EQ(engine.step(span * 0.5), Status::Ok);
    EXPECT_DOUBLE_EQ(speed_of(engine, "ego"), 10.0);
}

TEST(FollowModeSpeedActionTest, PositionModeKeepsTheAuthoredLinearShape) {
    // The same scenario in position mode: the shape stays Linear, so at half
    // the (acceleration-clamped) span the speed is exactly half — the Cubic
    // midpoint coincides, so this asserts the quarter point instead, where the
    // two shapes differ: Linear 0.25 vs Cubic 3(1/16) - 2(1/64) = 0.15625.
    const Performance perf{60.0, 2.0, 2.0, std::nullopt, std::nullopt};
    TransitionDynamics dynamics;
    dynamics.shape = DynamicsShape::Linear;
    dynamics.dimension = DynamicsDimension::Time;
    dynamics.value = 2.0;
    dynamics.following_mode = FollowingMode::Position;
    Engine engine;
    ASSERT_EQ(engine.init(wrap({make_entity("ego", perf)},
                               std::make_shared<SpeedAction>("ego", 10.0, dynamics))),
              Status::Ok);
    // Position mode clamps to 1.0 * 10 / 2 = 5 s; a quarter of it is 1.25 s.
    ASSERT_EQ(engine.step(1.25), Status::Ok);
    EXPECT_NEAR(speed_of(engine, "ego"), 2.5, kTol);

    Engine follow_engine;
    ASSERT_EQ(follow_engine.init(wrap(
                  {make_entity("ego", perf)},
                  follow_speed_action(10.0, DynamicsShape::Linear, DynamicsDimension::Time, 2.0))),
              Status::Ok);
    ASSERT_EQ(follow_engine.step(7.5 * 0.25), Status::Ok);
    EXPECT_NEAR(speed_of(follow_engine, "ego"), 1.5625, kTol);
}

TEST(FollowModeSpeedActionTest, TargetAboveMaxSpeedWarnsWithItsRuleId) {
    const Performance perf{20.0, 2.0, 2.0, std::nullopt, std::nullopt};
    Engine engine;
    ASSERT_EQ(engine.init(wrap(
                  {make_entity("ego", perf)},
                  follow_speed_action(30.0, DynamicsShape::Linear, DynamicsDimension::Time, 2.0))),
              Status::Ok); // advisory: the scenario still initializes
    bool found = false;
    for (const scena::Diagnostic& diagnostic : engine.diagnostics()) {
        if (diagnostic.rule_id ==
            "asam.net:xosc:1.2.0:scenario_logic.targetspeed_maxspeed_general") {
            EXPECT_EQ(diagnostic.severity, Severity::Warning);
            found = true;
        }
    }
    EXPECT_TRUE(found);
    // And the target is still clamped to maxSpeed, as position mode clamps it.
    ASSERT_EQ(engine.step(100.0), Status::Ok);
    EXPECT_DOUBLE_EQ(speed_of(engine, "ego"), 20.0);
}

TEST(FollowModeSpeedActionTest, PositionModeDoesNotRaiseTheMaxSpeedAdvice) {
    // Rule targetspeed_maxspeed_general is scoped to followingMode=follow.
    TransitionDynamics dynamics;
    dynamics.shape = DynamicsShape::Linear;
    dynamics.dimension = DynamicsDimension::Time;
    dynamics.value = 2.0;
    const Performance perf{20.0, 2.0, 2.0, std::nullopt, std::nullopt};
    Engine engine;
    ASSERT_EQ(engine.init(wrap({make_entity("ego", perf)},
                               std::make_shared<SpeedAction>("ego", 30.0, dynamics))),
              Status::Ok);
    for (const scena::Diagnostic& diagnostic : engine.diagnostics()) {
        EXPECT_NE(diagnostic.rule_id,
                  "asam.net:xosc:1.2.0:scenario_logic.targetspeed_maxspeed_general");
    }
}

// --- SpeedProfileAction: follow mode, constraints and entityRef -------------

TEST(FollowModeProfileTest, FollowModeUsesTheSmoothShapeAndTheJerkLimit) {
    // One leg 0 -> 10 with a 3 m/s^3 jerk constraint and no acceleration bound:
    // T = sqrt(6 * 10 / 3) = sqrt(20), with a Cubic profile over it.
    DynamicConstraints constraints;
    constraints.max_acceleration_rate = 3.0;
    constraints.max_deceleration_rate = 3.0;
    Engine engine;
    ASSERT_EQ(engine.init(wrap({make_entity("ego", std::nullopt)},
                               std::make_shared<SpeedProfileAction>(
                                   "ego", std::vector<SpeedProfileEntry>{{10.0, 1.0}},
                                   FollowingMode::Follow, std::string{}, constraints))),
              Status::Ok);
    const double span = std::sqrt(20.0);
    ASSERT_EQ(engine.step(span * 0.25), Status::Ok);
    EXPECT_NEAR(speed_of(engine, "ego"), 1.5625, kTol); // Cubic, not Linear
    ASSERT_EQ(engine.step(span * 0.75), Status::Ok);
    EXPECT_DOUBLE_EQ(speed_of(engine, "ego"), 10.0);
}

TEST(FollowModeProfileTest, ConstraintsTakePrecedenceOverThePerformanceEnvelope) {
    // §SpeedProfileAction: "these settings has precedence over any Performance
    // settings" — so a LOOSER constraint wins over a tighter envelope, which a
    // tighter-of-both reading would get wrong.
    const Performance perf{60.0, 1.0, 1.0, std::nullopt, std::nullopt};
    DynamicConstraints constraints;
    constraints.max_acceleration = 5.0; // looser than Performance's 1.0
    Engine engine;
    ASSERT_EQ(engine.init(wrap({make_entity("ego", perf)},
                               std::make_shared<SpeedProfileAction>(
                                   "ego", std::vector<SpeedProfileEntry>{{10.0, std::nullopt}},
                                   FollowingMode::Position, std::string{}, constraints))),
              Status::Ok);
    // Position mode, no authored time: delta / limit = 10 / 5 = 2 s, not 10 s.
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_NEAR(speed_of(engine, "ego"), 5.0, kTol);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_DOUBLE_EQ(speed_of(engine, "ego"), 10.0);
}

TEST(FollowModeProfileTest, EntityRelativeEntriesAreDeltasOnTheReferenceSpeed) {
    // The reference holds 12 m/s; an entry of -2 targets 10 m/s.
    std::vector<Entity> entities;
    entities.push_back(make_entity("ego", std::nullopt));
    entities.push_back(make_entity("lead", std::nullopt));
    std::vector<std::shared_ptr<scena::ir::Action>> init;
    init.push_back(std::make_shared<SpeedAction>("lead", 12.0));
    Engine engine;
    ASSERT_EQ(engine.init(wrap(std::move(entities),
                               std::make_shared<SpeedProfileAction>(
                                   "ego", std::vector<SpeedProfileEntry>{{-2.0, 2.0}},
                                   FollowingMode::Position, "lead", std::nullopt),
                               std::move(init))),
              Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_NEAR(speed_of(engine, "ego"), 5.0, kTol); // halfway from 0 to 10
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_DOUBLE_EQ(speed_of(engine, "ego"), 10.0);
}

TEST(FollowModeProfileTest, UnknownReferenceEntityIsRejectedAtInit) {
    Engine engine;
    EXPECT_EQ(engine.init(wrap({make_entity("ego", std::nullopt)},
                               std::make_shared<SpeedProfileAction>(
                                   "ego", std::vector<SpeedProfileEntry>{{1.0, 1.0}},
                                   FollowingMode::Position, "ghost", std::nullopt))),
              Status::SemanticError);
}

TEST(FollowModeProfileTest, NegativeConstraintIsRejectedAtInit) {
    DynamicConstraints constraints;
    constraints.max_acceleration = -1.0;
    Engine engine;
    EXPECT_EQ(engine.init(wrap({make_entity("ego", std::nullopt)},
                               std::make_shared<SpeedProfileAction>(
                                   "ego", std::vector<SpeedProfileEntry>{{1.0, 1.0}},
                                   FollowingMode::Position, std::string{}, constraints))),
              Status::ValidationError);
}

TEST(FollowModeProfileTest, EntryAboveMaxSpeedWarnsOnlyForAnAbsoluteFollowProfile) {
    const Performance perf{20.0, 2.0, 2.0, std::nullopt, std::nullopt};
    const char* const rule =
        "asam.net:xosc:1.2.0:scenario_logic.targetspeed_maxspeed_speedprofileaction";

    Engine absolute;
    ASSERT_EQ(absolute.init(wrap({make_entity("ego", perf)},
                                 std::make_shared<SpeedProfileAction>(
                                     "ego", std::vector<SpeedProfileEntry>{{30.0, 1.0}},
                                     FollowingMode::Follow, std::string{}, std::nullopt))),
              Status::Ok);
    bool found = false;
    for (const scena::Diagnostic& diagnostic : absolute.diagnostics()) {
        if (diagnostic.rule_id == rule) {
            EXPECT_EQ(diagnostic.severity, Severity::Warning);
            found = true;
        }
    }
    EXPECT_TRUE(found);

    // With an entityRef the entry is a delta, not a speed: the rule text scopes
    // itself to "no entityRef is specified", so no advice is due.
    std::vector<Entity> entities;
    entities.push_back(make_entity("ego", perf));
    entities.push_back(make_entity("lead", std::nullopt));
    Engine relative;
    ASSERT_EQ(relative.init(
                  wrap(std::move(entities), std::make_shared<SpeedProfileAction>(
                                                "ego", std::vector<SpeedProfileEntry>{{30.0, 1.0}},
                                                FollowingMode::Follow, "lead", std::nullopt))),
              Status::Ok);
    for (const scena::Diagnostic& diagnostic : relative.diagnostics()) {
        EXPECT_NE(diagnostic.rule_id, rule);
    }
}

TEST(FollowModeProfileTest, AFollowModeRunIsBitIdenticalAcrossRepeats) {
    // The determinism contract reaches the new arithmetic: sqrt and the Cubic
    // polynomial are IEEE-exact, and no transcendental enters unless the author
    // asked for Sinusoidal (which routes through det_cos like every other one).
    const Performance perf{60.0, 2.0, 2.0, 1.5, 1.5};
    const auto run = [&perf]() {
        Engine engine;
        EXPECT_EQ(engine.init(wrap({make_entity("ego", perf)},
                                   follow_speed_action(23.5, DynamicsShape::Sinusoidal,
                                                       DynamicsDimension::Time, 1.0))),
                  Status::Ok);
        std::vector<double> trace;
        for (int i = 0; i < 200; ++i) {
            EXPECT_EQ(engine.step(0.05), Status::Ok);
            trace.push_back(speed_of(engine, "ego"));
        }
        return trace;
    };
    const std::vector<double> first = run();
    const std::vector<double> second = run();
    ASSERT_EQ(first.size(), second.size());
    for (std::size_t i = 0; i < first.size(); ++i) {
        EXPECT_DOUBLE_EQ(first[i], second[i]);
    }
}

} // namespace
