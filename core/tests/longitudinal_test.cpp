// SPDX-License-Identifier: MIT
//
// Longitudinal-dynamics evaluator and controller (p2-s2). The transition math
// is closed form, so the shape x dimension matrix is checked against its own
// analytic reference rather than a tolerance sweep. Sinusoidal references are
// computed through det_cos so the assertions are exact bit-for-bit.

#include "scena/runtime/longitudinal.h"

#include <cmath>

#include <gtest/gtest.h>

#include "scena/ir/dynamics.h"
#include "scena/runtime/detmath.h"

namespace {

using scena::ir::DynamicsDimension;
using scena::ir::DynamicsShape;
using scena::ir::TransitionDynamics;
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
    EXPECT_NEAR(transition_duration({DynamicsShape::Linear, DynamicsDimension::Rate, rate}, 0.0, 10.0),
                delta / rate, kTol); // 5 s
    EXPECT_NEAR(transition_duration({DynamicsShape::Cubic, DynamicsDimension::Rate, rate}, 0.0, 10.0),
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
    EXPECT_DOUBLE_EQ(transition_duration({DynamicsShape::Step, DynamicsDimension::Time, 0.0}, 0.0, 9.0),
                     0.0);
    // No value change ⇒ nothing to acquire.
    EXPECT_DOUBLE_EQ(transition_duration({DynamicsShape::Linear, DynamicsDimension::Time, 5.0}, 3.0, 3.0),
                     0.0);
    // Non-positive value ⇒ instantaneous.
    EXPECT_DOUBLE_EQ(transition_duration({DynamicsShape::Linear, DynamicsDimension::Time, 0.0}, 0.0, 9.0),
                     0.0);
    EXPECT_DOUBLE_EQ(transition_duration({DynamicsShape::Linear, DynamicsDimension::Rate, -1.0}, 0.0, 9.0),
                     0.0);
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
