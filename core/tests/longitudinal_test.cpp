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
// Longitudinal-dynamics evaluator and controller (p2-s2). The transition math
// is closed form, so the shape x dimension matrix is checked against its own
// analytic reference rather than a tolerance sweep. Sinusoidal references are
// computed through det_cos so the assertions are exact bit-for-bit.

#include "scena/runtime/longitudinal.h"

#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "scena/engine.h"
#include "scena/ir/action.h"
#include "scena/ir/dynamics.h"
#include "scena/ir/entity.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"
#include "scena/runtime/detmath.h"

namespace {

using scena::Engine;
using scena::Status;
using scena::ir::ControlMode;
using scena::ir::DynamicsDimension;
using scena::ir::DynamicsShape;
using scena::ir::Entity;
using scena::ir::Performance;
using scena::ir::Scenario;
using scena::ir::SpeedAction;
using scena::ir::TransitionDynamics;
using scena::ir::Vehicle;
using scena::runtime::LongitudinalController;
using scena::runtime::shape_fraction;
using scena::runtime::shape_peak_gradient_factor;
using scena::runtime::transition_duration;
using scena::runtime::transition_value;

constexpr double kPi = 3.14159265358979311599796346854;
constexpr double kTol = 1e-12;

// --- shape_fraction: endpoints, midpoints, monotonic bounds ---------------

TEST(LongitudinalShapeTest, EndpointsAreZeroAndOne) {
    for (DynamicsShape shape :
         {DynamicsShape::Linear, DynamicsShape::Cubic, DynamicsShape::Sinusoidal}) {
        EXPECT_DOUBLE_EQ(shape_fraction(shape, 0.0), 0.0);
        EXPECT_DOUBLE_EQ(shape_fraction(shape, 1.0), 1.0);
    }
}

TEST(LongitudinalShapeTest, ProgressIsClampedOutsideUnitInterval) {
    EXPECT_DOUBLE_EQ(shape_fraction(DynamicsShape::Linear, -0.5), 0.0);
    EXPECT_DOUBLE_EQ(shape_fraction(DynamicsShape::Cubic, 2.0), 1.0);
    // NaN progress clamps to 0, never propagates.
    EXPECT_DOUBLE_EQ(shape_fraction(DynamicsShape::Sinusoidal, std::nan("")), 0.0);
}

TEST(LongitudinalShapeTest, MidpointsMatchClosedForm) {
    EXPECT_NEAR(shape_fraction(DynamicsShape::Linear, 0.5), 0.5, kTol);
    // 3(0.25) - 2(0.125) = 0.5.
    EXPECT_NEAR(shape_fraction(DynamicsShape::Cubic, 0.5), 0.5, kTol);
    // (1 - cos(pi/2)) / 2 = 0.5.
    EXPECT_NEAR(shape_fraction(DynamicsShape::Sinusoidal, 0.5), 0.5, kTol);
}

TEST(LongitudinalShapeTest, SinusoidalIsExactThroughDetCos) {
    // The evaluator must route through det_cos, so recomputing the reference
    // the same way is bit-identical, not merely close.
    const double p = 0.25;
    const double expected = (1.0 - scena::runtime::det_cos(kPi * p)) * 0.5;
    EXPECT_EQ(shape_fraction(DynamicsShape::Sinusoidal, p), expected);
}

TEST(LongitudinalShapeTest, StepIsInstantaneous) {
    EXPECT_DOUBLE_EQ(shape_fraction(DynamicsShape::Step, 0.0), 0.0);
    EXPECT_DOUBLE_EQ(shape_fraction(DynamicsShape::Step, 1e-9), 1.0);
    EXPECT_DOUBLE_EQ(shape_fraction(DynamicsShape::Step, 1.0), 1.0);
}

TEST(LongitudinalShapeTest, TransitionValueScalesBetweenEndpoints) {
    // from = 4, to = 10, cubic midpoint = 4 + 6*0.5 = 7.
    EXPECT_NEAR(transition_value(DynamicsShape::Cubic, 4.0, 10.0, 0.5), 7.0, kTol);
    // Decreasing transitions work identically.
    EXPECT_NEAR(transition_value(DynamicsShape::Linear, 10.0, 0.0, 0.25), 7.5, kTol);
}

// --- transition_duration: time, rate, distance, degenerate ----------------

TEST(LongitudinalDurationTest, TimeDimensionReturnsValueDirectly) {
    for (DynamicsShape shape :
         {DynamicsShape::Linear, DynamicsShape::Cubic, DynamicsShape::Sinusoidal}) {
        const TransitionDynamics td{shape, DynamicsDimension::Time, 5.0};
        EXPECT_DOUBLE_EQ(transition_duration(td, 0.0, 10.0), 5.0);
    }
}

TEST(LongitudinalDurationTest, RateDimensionUsesShapePeakGradient) {
    const double delta = 10.0; // 0 -> 10 m/s
    const double rate = 2.0;   // m/s per s (peak gradient)
    EXPECT_NEAR(
        transition_duration({DynamicsShape::Linear, DynamicsDimension::Rate, rate}, 0.0, 10.0),
        delta / rate, kTol); // 5 s
    EXPECT_NEAR(
        transition_duration({DynamicsShape::Cubic, DynamicsDimension::Rate, rate}, 0.0, 10.0),
        1.5 * delta / rate, kTol); // 7.5 s
    EXPECT_NEAR(
        transition_duration({DynamicsShape::Sinusoidal, DynamicsDimension::Rate, rate}, 0.0, 10.0),
        (kPi / 2.0) * delta / rate, kTol); // ~7.854 s
}

TEST(LongitudinalDurationTest, DistanceDimensionHasNoTimeDuration) {
    const TransitionDynamics td{DynamicsShape::Linear, DynamicsDimension::Distance, 20.0};
    EXPECT_TRUE(std::isnan(transition_duration(td, 0.0, 10.0)));
}

TEST(LongitudinalDurationTest, DegenerateTransitionsConsumeNoTime) {
    // Step is always instantaneous.
    EXPECT_DOUBLE_EQ(
        transition_duration({DynamicsShape::Step, DynamicsDimension::Time, 0.0}, 0.0, 9.0), 0.0);
    // No value change ⇒ nothing to acquire.
    EXPECT_DOUBLE_EQ(
        transition_duration({DynamicsShape::Linear, DynamicsDimension::Time, 5.0}, 3.0, 3.0), 0.0);
    // Non-positive value ⇒ instantaneous.
    EXPECT_DOUBLE_EQ(
        transition_duration({DynamicsShape::Linear, DynamicsDimension::Time, 0.0}, 0.0, 9.0), 0.0);
    EXPECT_DOUBLE_EQ(
        transition_duration({DynamicsShape::Linear, DynamicsDimension::Rate, -1.0}, 0.0, 9.0), 0.0);
}

TEST(LongitudinalDurationTest, PeakGradientFactors) {
    EXPECT_DOUBLE_EQ(shape_peak_gradient_factor(DynamicsShape::Linear), 1.0);
    EXPECT_DOUBLE_EQ(shape_peak_gradient_factor(DynamicsShape::Cubic), 1.5);
    EXPECT_DOUBLE_EQ(shape_peak_gradient_factor(DynamicsShape::Sinusoidal), kPi / 2.0);
    EXPECT_DOUBLE_EQ(shape_peak_gradient_factor(DynamicsShape::Step), 0.0);
}

// --- LongitudinalController: single-segment (SpeedAction) drive ------------

// Builds a one-segment time controller (the SpeedAction shape) with span T.
LongitudinalController time_controller(double from, double to, DynamicsShape shape, double span) {
    LongitudinalController c;
    c.segments.push_back({from, to, shape, /*by_distance=*/false, span});
    return c;
}

TEST(LongitudinalControllerTest, LinearTimeRampReachesTargetExactly) {
    LongitudinalController c = time_controller(0.0, 10.0, DynamicsShape::Linear, 5.0);
    // Ten steps of 0.5 s; midpoint is exactly half.
    for (int i = 0; i < 5; ++i) {
        c.advance(0.5, 0.0);
    }
    EXPECT_NEAR(c.speed(), 5.0, kTol);
    EXPECT_FALSE(c.done());
    for (int i = 0; i < 5; ++i) {
        c.advance(0.5, 0.0);
    }
    EXPECT_TRUE(c.done());
    EXPECT_DOUBLE_EQ(c.speed(), 10.0); // snapped exactly to target
}

TEST(LongitudinalControllerTest, SinusoidalEndpointSnapsExactlyToTarget) {
    LongitudinalController c = time_controller(0.0, 12.0, DynamicsShape::Sinusoidal, 4.0);
    c.advance(4.0, 0.0);
    ASSERT_TRUE(c.done());
    // Exactly the target despite det_cos(pi) not being exactly -1.
    EXPECT_DOUBLE_EQ(c.speed(), 12.0);
}

TEST(LongitudinalControllerTest, StepSegmentIsInstantaneous) {
    LongitudinalController c = time_controller(3.0, 8.0, DynamicsShape::Step, 0.0);
    const double s = c.advance(0.0, 0.0);
    EXPECT_TRUE(c.done());
    EXPECT_DOUBLE_EQ(s, 8.0);
}

TEST(LongitudinalControllerTest, DistanceSegmentAdvancesByTravel) {
    // Reach 10 m/s over 20 m, linearly in distance. After 10 m travelled the
    // speed is halfway.
    LongitudinalController c;
    c.segments.push_back({0.0, 10.0, DynamicsShape::Linear, /*by_distance=*/true, 20.0});
    c.advance(1.0, 10.0);
    EXPECT_NEAR(c.speed(), 5.0, kTol);
    EXPECT_FALSE(c.done());
    c.advance(1.0, 10.0);
    EXPECT_TRUE(c.done());
    EXPECT_DOUBLE_EQ(c.speed(), 10.0);
}

TEST(LongitudinalControllerTest, MultiSegmentProfileCarriesRemainderAcrossBoundaries) {
    // A two-entry profile: 0 -> 10 over 2 s, then 10 -> 4 over 2 s, both linear.
    LongitudinalController c;
    c.segments.push_back({0.0, 10.0, DynamicsShape::Linear, false, 2.0});
    c.segments.push_back({10.0, 4.0, DynamicsShape::Linear, false, 2.0});
    // One 3 s step crosses the first boundary: 2 s finishes segment 0, the
    // remaining 1 s puts segment 1 at its midpoint (10 -> 7).
    c.advance(3.0, 0.0);
    EXPECT_EQ(c.index, 1u);
    EXPECT_NEAR(c.speed(), 7.0, kTol);
    c.advance(1.0, 0.0);
    EXPECT_TRUE(c.done());
    EXPECT_DOUBLE_EQ(c.speed(), 4.0);
}

TEST(LongitudinalControllerTest, EmptyControllerIsDone) {
    LongitudinalController c;
    EXPECT_TRUE(c.done());
    EXPECT_DOUBLE_EQ(c.speed(), 0.0);
}

} // namespace

// --- Engine integration: SpeedAction through the default controller --------

// Builds a scenario with one engine-controlled entity and a triggerless
// SpeedAction to `target` shaped by `td`. `initial_speed` is set by a step init
// action; `perf`, when present, makes the entity a Vehicle with that envelope.
Scenario make_speed_scenario(double target, TransitionDynamics td, double initial_speed = 0.0,
                             std::optional<Performance> perf = std::nullopt) {
    Scenario scenario;
    scenario.name = "longitudinal-integration";

    Entity ego;
    ego.id = "ego";
    ego.name = "ego";
    ego.control_mode = ControlMode::EngineControlled;
    if (perf.has_value()) {
        Vehicle vehicle;
        vehicle.performance = *perf;
        ego.object = vehicle;
    }
    scenario.entities.push_back(std::move(ego));

    if (initial_speed != 0.0) {
        scenario.init_actions.push_back(std::make_shared<SpeedAction>("ego", initial_speed));
    }

    scena::ir::Event event;
    event.name = "event";
    event.actions.push_back(std::make_shared<SpeedAction>("ego", target, td));
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

double speed_of(const Engine& engine, const std::string& id) {
    const std::optional<scena::EntityState> state = engine.state(id);
    return state.has_value() ? state->speed : std::nan("");
}

TEST(LongitudinalEngineTest, LinearTimeRampDrivesSpeedAcrossSteps) {
    Engine engine;
    ASSERT_EQ(engine.init(
                  make_speed_scenario(10.0, {DynamicsShape::Linear, DynamicsDimension::Time, 5.0})),
              Status::Ok);
    // Installed at t=0, still at the origin speed.
    EXPECT_DOUBLE_EQ(speed_of(engine, "ego"), 0.0);

    for (int i = 0; i < 2; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    EXPECT_NEAR(speed_of(engine, "ego"), 4.0, kTol); // 2 s of a 5 s ramp
    EXPECT_EQ(*engine.storyboard_element_state("story/act/group/maneuver/event"),
              scena::runtime::ElementState::Running);

    for (int i = 0; i < 3; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    EXPECT_DOUBLE_EQ(speed_of(engine, "ego"), 10.0); // reached target exactly
    EXPECT_EQ(*engine.storyboard_element_state("story/act/group/maneuver/event"),
              scena::runtime::ElementState::Complete);
}

TEST(LongitudinalEngineTest, StepShapeSpeedActionIsInstantaneous) {
    Engine engine;
    // The 3-arg step form and the legacy 2-arg form behave identically.
    ASSERT_EQ(
        engine.init(make_speed_scenario(7.5, {DynamicsShape::Step, DynamicsDimension::Time, 0.0})),
        Status::Ok);
    // Reached in the t=0 evaluation; the event has already completed.
    EXPECT_DOUBLE_EQ(speed_of(engine, "ego"), 7.5);
    EXPECT_EQ(*engine.storyboard_element_state("story/act/group/maneuver/event"),
              scena::runtime::ElementState::Complete);
}

TEST(LongitudinalEngineTest, PerformanceAccelerationClampExtendsTheRamp) {
    // Ask for 0 -> 10 in 1 s (10 m/s^2) but cap acceleration at 2 m/s^2: the
    // transition is stretched to 5 s, reaching the target no faster than the
    // envelope allows.
    Engine engine;
    const Performance perf{60.0, 2.0, 2.0, std::nullopt, std::nullopt};
    ASSERT_EQ(engine.init(make_speed_scenario(
                  10.0, {DynamicsShape::Linear, DynamicsDimension::Time, 1.0}, 0.0, perf)),
              Status::Ok);

    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_NEAR(speed_of(engine, "ego"), 2.0, kTol); // clamped slope, not 10
    for (int i = 0; i < 4; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    EXPECT_DOUBLE_EQ(speed_of(engine, "ego"), 10.0); // target reached at 5 s
}

TEST(LongitudinalEngineTest, PerformanceMaxSpeedClampsTheTarget) {
    // Target 30 m/s but the vehicle tops out at 20 m/s.
    Engine engine;
    const Performance perf{20.0, 100.0, 100.0, std::nullopt, std::nullopt};
    ASSERT_EQ(engine.init(make_speed_scenario(
                  30.0, {DynamicsShape::Linear, DynamicsDimension::Time, 4.0}, 0.0, perf)),
              Status::Ok);
    for (int i = 0; i < 4; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    EXPECT_DOUBLE_EQ(speed_of(engine, "ego"), 20.0);
}

TEST(LongitudinalEngineTest, InvalidTransitionValueIsRejectedAtInit) {
    Engine engine;
    const Status status = engine.init(
        make_speed_scenario(10.0, {DynamicsShape::Linear, DynamicsDimension::Time, -1.0}));
    EXPECT_EQ(status, Status::ValidationError);
}

// --- SpeedProfileAction: piecewise position-mode profile -------------------

Scenario make_profile_scenario(std::vector<scena::ir::SpeedProfileEntry> entries,
                               double initial_speed = 0.0,
                               std::optional<Performance> perf = std::nullopt) {
    Scenario scenario;
    scenario.name = "profile-integration";

    Entity ego;
    ego.id = "ego";
    ego.name = "ego";
    ego.control_mode = ControlMode::EngineControlled;
    if (perf.has_value()) {
        Vehicle vehicle;
        vehicle.performance = *perf;
        ego.object = vehicle;
    }
    scenario.entities.push_back(std::move(ego));
    if (initial_speed != 0.0) {
        scenario.init_actions.push_back(std::make_shared<SpeedAction>("ego", initial_speed));
    }

    scena::ir::Event event;
    event.name = "event";
    event.actions.push_back(
        std::make_shared<scena::ir::SpeedProfileAction>("ego", std::move(entries)));
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

TEST(LongitudinalProfileTest, PiecewiseLinearProfileFollowsTargets) {
    // 0 -> 10 over 2 s, then 10 -> 4 over 3 s (linear interpolation each leg).
    Engine engine;
    ASSERT_EQ(engine.init(make_profile_scenario({{10.0, 2.0}, {4.0, 3.0}})), Status::Ok);

    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_NEAR(speed_of(engine, "ego"), 5.0, kTol); // midway up the first leg
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_NEAR(speed_of(engine, "ego"), 10.0, kTol); // first target reached
    ASSERT_EQ(engine.step(1.5), Status::Ok);
    EXPECT_NEAR(speed_of(engine, "ego"), 7.0, kTol); // midway down the second leg
    ASSERT_EQ(engine.step(1.5), Status::Ok);
    EXPECT_DOUBLE_EQ(speed_of(engine, "ego"), 4.0); // final target, exact
    EXPECT_EQ(*engine.storyboard_element_state("story/act/group/maneuver/event"),
              scena::runtime::ElementState::Complete);
}

TEST(LongitudinalProfileTest, EntryWithoutTimeUsesPerformanceAcceleration) {
    // No time on the entry ⇒ reach 6 m/s as fast as max_acceleration (2 m/s^2)
    // allows: 3 s.
    Engine engine;
    const Performance perf{60.0, 2.0, 2.0, std::nullopt, std::nullopt};
    ASSERT_EQ(engine.init(make_profile_scenario({{6.0, std::nullopt}}, 0.0, perf)), Status::Ok);
    ASSERT_EQ(engine.step(1.5), Status::Ok);
    EXPECT_NEAR(speed_of(engine, "ego"), 3.0, kTol); // halfway there at 1.5 s
    ASSERT_EQ(engine.step(1.5), Status::Ok);
    EXPECT_DOUBLE_EQ(speed_of(engine, "ego"), 6.0);
}

TEST(LongitudinalProfileTest, EmptyProfileIsRejectedAtInit) {
    Engine engine;
    EXPECT_EQ(engine.init(make_profile_scenario({})), Status::ValidationError);
}

TEST(LongitudinalProfileTest, NegativeEntryTimeIsRejectedAtInit) {
    Engine engine;
    EXPECT_EQ(engine.init(make_profile_scenario({{5.0, -1.0}})), Status::ValidationError);
}
