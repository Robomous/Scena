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
// The lateral motion machinery (p2-s3): the maxLateralAcc duration derivation
// of ADR-0016, the shaped offset ramps the lateral actions run on (the shared
// value-transition sequencer of runtime/longitudinal.h), and the overshoot and
// endpoint-exactness invariants those ramps must hold. Engine-level behavior
// lives in action_lateral_test.cpp.

#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include "scena/ir/dynamics.h"
#include "scena/runtime/lateral.h"
#include "scena/runtime/longitudinal.h"

namespace {

using scena::ir::DynamicsShape;
using scena::runtime::lane_offset_duration;
using scena::runtime::LongitudinalController;
using scena::runtime::shape_fraction;
using scena::runtime::shape_peak_curvature_factor;

constexpr double kTol = 1e-12;

/// pi, matching the literal the runtime uses.
constexpr double kPi = 3.14159265358979311599796346854;

/// The shapes that describe a transition over time; Step is handled separately
/// everywhere because it consumes none.
const std::vector<DynamicsShape> kSmoothShapes = {DynamicsShape::Linear, DynamicsShape::Cubic,
                                                  DynamicsShape::Sinusoidal};

/// A one-segment offset ramp from `from` to `to` over `span` seconds.
LongitudinalController offset_ramp(DynamicsShape shape, double from, double to, double span) {
    LongitudinalController controller;
    LongitudinalController::Segment segment;
    segment.from = from;
    segment.to = to;
    segment.shape = shape;
    segment.by_distance = false;
    segment.span = span;
    controller.segments.push_back(segment);
    return controller;
}

// --- Duration derivation ---------------------------------------------------

TEST(LateralDurationTest, PerShapeClosedForms) {
    // T = sqrt(k * |delta| / a), with k from the peak curvature of the shape.
    const double delta = 3.5; // one lane width
    const double acc = 2.0;
    EXPECT_NEAR(lane_offset_duration(DynamicsShape::Sinusoidal, delta, acc),
                kPi * std::sqrt(delta / (2.0 * acc)), kTol);
    EXPECT_NEAR(lane_offset_duration(DynamicsShape::Cubic, delta, acc),
                std::sqrt(6.0 * delta / acc), kTol);
    EXPECT_NEAR(lane_offset_duration(DynamicsShape::Linear, delta, acc),
                2.0 * std::sqrt(delta / acc), kTol);
}

TEST(LateralDurationTest, PeakCurvatureFactorsAreTheDocumentedConstants) {
    EXPECT_DOUBLE_EQ(shape_peak_curvature_factor(DynamicsShape::Step), 0.0);
    EXPECT_DOUBLE_EQ(shape_peak_curvature_factor(DynamicsShape::Linear), 4.0);
    EXPECT_DOUBLE_EQ(shape_peak_curvature_factor(DynamicsShape::Cubic), 6.0);
    EXPECT_NEAR(shape_peak_curvature_factor(DynamicsShape::Sinusoidal), kPi * kPi * 0.5, kTol);
    // The linear bound sits continuously below the smooth shapes.
    EXPECT_LT(shape_peak_curvature_factor(DynamicsShape::Linear),
              shape_peak_curvature_factor(DynamicsShape::Sinusoidal));
    EXPECT_LT(shape_peak_curvature_factor(DynamicsShape::Sinusoidal),
              shape_peak_curvature_factor(DynamicsShape::Cubic));
}

TEST(LateralDurationTest, InstantaneousCases) {
    // Step is instantaneous by definition (§7.4.1.4).
    EXPECT_DOUBLE_EQ(lane_offset_duration(DynamicsShape::Step, 3.5, 2.0), 0.0);
    for (const DynamicsShape shape : kSmoothShapes) {
        // Nothing to acquire.
        EXPECT_DOUBLE_EQ(lane_offset_duration(shape, 0.0, 2.0), 0.0);
        // "Missing value is interpreted as 'inf'": no time needed.
        EXPECT_DOUBLE_EQ(lane_offset_duration(shape, 3.5, std::nullopt), 0.0);
        EXPECT_DOUBLE_EQ(
            lane_offset_duration(shape, 3.5, std::numeric_limits<double>::infinity()), 0.0);
        // Defensive: init rejects these, they must not divide by zero or NaN.
        EXPECT_DOUBLE_EQ(lane_offset_duration(shape, 3.5, 0.0), 0.0);
        EXPECT_DOUBLE_EQ(lane_offset_duration(shape, std::nan(""), 2.0), 0.0);
    }
}

TEST(LateralDurationTest, DurationIsSignAgnosticAndScalesAsTheSquareRoot) {
    for (const DynamicsShape shape : kSmoothShapes) {
        // Moving left or right takes the same time.
        EXPECT_DOUBLE_EQ(lane_offset_duration(shape, 3.5, 2.0),
                         lane_offset_duration(shape, -3.5, 2.0));
        // Four times the offset at the same limit takes twice as long.
        EXPECT_NEAR(lane_offset_duration(shape, 4.0, 2.0),
                    2.0 * lane_offset_duration(shape, 1.0, 2.0), kTol);
    }
}

TEST(LateralDurationTest, ThePeakLateralAccelerationMatchesTheLimit) {
    // The derivation's whole point: a shaped ramp over the derived duration
    // peaks at exactly the requested lateral acceleration. Measured as a
    // centred second difference of the offset over a dense sweep.
    //
    // Both smooth shapes take their peak curvature at the endpoints, where a
    // centred difference has no room, so the sampled peak approaches the limit
    // from below rather than straddling it — hence a one-sided bound rather
    // than EXPECT_NEAR. Linear is excluded: it has no interior curvature at
    // all, and its factor comes from the bang-bang bound instead.
    const double delta = 3.5;
    const double acc = 1.6;
    for (const DynamicsShape shape : {DynamicsShape::Cubic, DynamicsShape::Sinusoidal}) {
        const double span = lane_offset_duration(shape, delta, acc);
        ASSERT_GT(span, 0.0);
        const double h = span * 1e-4;
        double peak = 0.0;
        for (int i = 0; i <= 1000; ++i) {
            const double t = span * static_cast<double>(i) / 1000.0;
            if (t - h < 0.0 || t + h > span) {
                continue;
            }
            const double before = delta * shape_fraction(shape, (t - h) / span);
            const double at = delta * shape_fraction(shape, t / span);
            const double after = delta * shape_fraction(shape, (t + h) / span);
            peak = std::fmax(peak, std::fabs((after - 2.0 * at + before) / (h * h)));
        }
        EXPECT_LE(peak, acc);
        EXPECT_GT(peak, acc * 0.99);
    }
}

// --- Shaped offset ramps ---------------------------------------------------

TEST(LateralRampTest, ShapeSweepReachesTheTargetExactly) {
    // A completed transition lands on the target bit-exactly, not a det_cos ulp
    // short — the invariant that lets a lane change end on the lane centre.
    for (const DynamicsShape shape : kSmoothShapes) {
        LongitudinalController ramp = offset_ramp(shape, 0.0, 3.5, 4.0);
        EXPECT_DOUBLE_EQ(ramp.advance(0.0, 0.0), 0.0); // g(0) = 0 for every shape
        ramp.advance(2.0, 0.0);
        ramp.advance(2.0, 0.0);
        EXPECT_TRUE(ramp.done());
        EXPECT_DOUBLE_EQ(ramp.speed(), 3.5);
    }
}

TEST(LateralRampTest, MidpointsMatchTheClosedForms) {
    // Half way through, each shape sits where its g(0.5) says it does.
    const double delta = 3.5;
    LongitudinalController linear = offset_ramp(DynamicsShape::Linear, 0.0, delta, 4.0);
    linear.advance(2.0, 0.0);
    EXPECT_NEAR(linear.speed(), delta * 0.5, kTol);

    LongitudinalController cubic = offset_ramp(DynamicsShape::Cubic, 0.0, delta, 4.0);
    cubic.advance(2.0, 0.0);
    EXPECT_NEAR(cubic.speed(), delta * 0.5, kTol); // 3(0.25) - 2(0.125) = 0.5

    LongitudinalController sinusoidal = offset_ramp(DynamicsShape::Sinusoidal, 0.0, delta, 4.0);
    sinusoidal.advance(2.0, 0.0);
    EXPECT_NEAR(sinusoidal.speed(), delta * 0.5, kTol); // (1 - cos(pi/2))/2 = 0.5

    // A quarter of the way in the shapes genuinely differ.
    LongitudinalController quarter_cubic = offset_ramp(DynamicsShape::Cubic, 0.0, delta, 4.0);
    quarter_cubic.advance(1.0, 0.0);
    EXPECT_NEAR(quarter_cubic.speed(), delta * (3.0 * 0.0625 - 2.0 * 0.015625), kTol);
    LongitudinalController quarter_sin = offset_ramp(DynamicsShape::Sinusoidal, 0.0, delta, 4.0);
    quarter_sin.advance(1.0, 0.0);
    EXPECT_NEAR(quarter_sin.speed(), delta * (1.0 - std::cos(kPi * 0.25)) * 0.5, 1e-9);
}

TEST(LateralRampTest, OffsetNeverOvershootsTheDelta) {
    // A dense sweep in both directions: the offset stays within [0, delta] the
    // whole way, so a lane change never swings past the target lane.
    for (const DynamicsShape shape : kSmoothShapes) {
        for (const double delta : {3.5, -3.5, 0.75, -0.75}) {
            LongitudinalController ramp = offset_ramp(shape, 0.0, delta, 4.0);
            const double lo = std::fmin(0.0, delta);
            const double hi = std::fmax(0.0, delta);
            for (int i = 0; i < 400; ++i) {
                const double value = ramp.advance(0.01, 0.0);
                EXPECT_GE(value, lo - kTol);
                EXPECT_LE(value, hi + kTol);
            }
        }
    }
}

TEST(LateralRampTest, DistanceDimensionedRampsAdvanceOnTravelledMetres) {
    // A distance-dimensioned lane change is tied to the odometer, not the
    // clock: the same metres give the same offset whatever the step pattern.
    LongitudinalController by_metres = offset_ramp(DynamicsShape::Linear, 0.0, 3.5, 50.0);
    by_metres.segments[0].by_distance = true;
    by_metres.advance(1.0, 25.0);
    EXPECT_NEAR(by_metres.speed(), 1.75, kTol);

    LongitudinalController other_pattern = offset_ramp(DynamicsShape::Linear, 0.0, 3.5, 50.0);
    other_pattern.segments[0].by_distance = true;
    for (int i = 0; i < 5; ++i) {
        other_pattern.advance(0.2, 5.0); // same 25 m, five steps
    }
    EXPECT_NEAR(other_pattern.speed(), 1.75, kTol);
}

TEST(LateralRampTest, AZeroSpanRampIsInstantaneous) {
    // The Step / derived-zero-duration path: consumed by the settling advance.
    LongitudinalController step = offset_ramp(DynamicsShape::Step, 0.0, 3.5, 0.0);
    EXPECT_DOUBLE_EQ(step.advance(0.0, 0.0), 3.5);
    EXPECT_TRUE(step.done());
}

} // namespace
