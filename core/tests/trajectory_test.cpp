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

// Analytic-fidelity and determinism tests for the trajectory evaluator
// (§6.9, risk R3). Analytic references (circles, straight lines) are matched
// to 1e-9; hex-pinned bit patterns, together with the 3-OS CI matrix, prove
// the evaluation is bit-identical across platforms. `std::sin`/`std::cos` are
// used freely here as an INDEPENDENT reference — test code is exempt from the
// detmath guard.

#include <cmath>

#include <gtest/gtest.h>

#include "scena/ir/trajectory.h"
#include "scena/runtime/trajectory_eval.h"
#include "support/trace_recorder.h"

namespace {

using scena::ir::Clothoid;
using scena::ir::Polyline;
using scena::ir::Trajectory;
using scena::ir::TrajectoryVertex;
using scena::ir::WorldPosition;
using scena::runtime::Pose;
using scena::runtime::TrajectoryEvaluator;
using scena::testsupport::hex_bits;

constexpr double kPi = 3.14159265358979323846;

Trajectory polyline_of(std::vector<WorldPosition> points) {
    Polyline polyline;
    for (const auto& p : points) {
        polyline.vertices.push_back(TrajectoryVertex{p, std::nullopt});
    }
    return Trajectory{"", false, polyline};
}

TEST(TrajectoryEvaluatorTest, PolylineInterpolatesSegmentsAndTangents) {
    const Trajectory trajectory =
        polyline_of({WorldPosition{0.0, 0.0, 0.0}, WorldPosition{10.0, 0.0, 0.0},
                     WorldPosition{10.0, 4.0, 2.0}});
    const TrajectoryEvaluator evaluator(trajectory);
    ASSERT_TRUE(evaluator.ok());
    EXPECT_DOUBLE_EQ(evaluator.total_length(), 10.0 + std::sqrt(16.0 + 4.0));

    const Pose mid_first = evaluator.pose_at_arclength(5.0);
    EXPECT_DOUBLE_EQ(mid_first.x, 5.0);
    EXPECT_DOUBLE_EQ(mid_first.y, 0.0);
    EXPECT_DOUBLE_EQ(mid_first.heading, 0.0);

    // Endpoint lands exactly on the final vertex (arc-length round-trip).
    const Pose end = evaluator.pose_at_arclength(evaluator.total_length());
    EXPECT_DOUBLE_EQ(end.x, 10.0);
    EXPECT_DOUBLE_EQ(end.y, 4.0);
    EXPECT_DOUBLE_EQ(end.z, 2.0);

    // Clamps beyond the ends.
    EXPECT_DOUBLE_EQ(evaluator.pose_at_arclength(-3.0).x, 0.0);
    EXPECT_DOUBLE_EQ(evaluator.pose_at_arclength(1e9).y, 4.0);
}

TEST(TrajectoryEvaluatorTest, DegeneratePolylineReportsAndDoesNotResolve) {
    const Trajectory trajectory = polyline_of({WorldPosition{0.0, 0.0, 0.0}});
    const TrajectoryEvaluator evaluator(trajectory);
    EXPECT_FALSE(evaluator.ok());
}

TEST(TrajectoryEvaluatorTest, ClothoidWithZeroCurvatureIsAStraightLine) {
    Clothoid clothoid;
    clothoid.start = WorldPosition{2.0, -1.0, 0.5};
    clothoid.start.h = kPi / 6.0; // 30 degrees
    clothoid.curvature = 0.0;
    clothoid.curvature_prime = 0.0;
    clothoid.length = 12.0;
    const TrajectoryEvaluator evaluator(Trajectory{"line", false, clothoid});
    ASSERT_TRUE(evaluator.ok());
    EXPECT_DOUBLE_EQ(evaluator.total_length(), 12.0);

    for (const double s : {0.0, 3.0, 7.5, 12.0}) {
        const Pose pose = evaluator.pose_at_arclength(s);
        EXPECT_NEAR(pose.x, 2.0 + s * std::cos(kPi / 6.0), 1e-9);
        EXPECT_NEAR(pose.y, -1.0 + s * std::sin(kPi / 6.0), 1e-9);
        EXPECT_DOUBLE_EQ(pose.z, 0.5);
        EXPECT_NEAR(pose.heading, kPi / 6.0, 1e-12);
    }
}

TEST(TrajectoryEvaluatorTest, ClothoidWithConstantCurvatureIsAnExactCircle) {
    // curvaturePrime == 0 -> circular arc of radius R = 1/kappa.
    const double kappa = 0.05;
    const double radius = 1.0 / kappa;
    Clothoid clothoid;
    clothoid.start = WorldPosition{0.0, 0.0, 0.0};
    clothoid.start.h = 0.0;
    clothoid.curvature = kappa;
    clothoid.curvature_prime = 0.0;
    clothoid.length = radius * (kPi / 2.0); // quarter circle
    const TrajectoryEvaluator evaluator(Trajectory{"arc", false, clothoid});
    ASSERT_TRUE(evaluator.ok());

    // Centre of curvature is a left-normal step of R from the start.
    const double cx = 0.0;
    const double cy = radius;
    for (int i = 0; i <= 20; ++i) {
        const double s = evaluator.total_length() * (static_cast<double>(i) / 20.0);
        const Pose pose = evaluator.pose_at_arclength(s);
        const double dx = pose.x - cx;
        const double dy = pose.y - cy;
        EXPECT_NEAR(std::sqrt(dx * dx + dy * dy), radius, 1e-9); // exactly on the circle
        EXPECT_NEAR(pose.heading, kappa * s, 1e-12);             // tangent turns with s
    }
    // Quarter circle ends a quarter turn round: heading == pi/2, at (R, R).
    const Pose end = evaluator.pose_at_arclength(evaluator.total_length());
    EXPECT_NEAR(end.x, radius, 1e-9);
    EXPECT_NEAR(end.y, radius, 1e-9);
    EXPECT_NEAR(end.heading, kPi / 2.0, 1e-9);
}

TEST(TrajectoryEvaluatorTest, GeneralClothoidMatchesAnIndependentFineReference) {
    // curvaturePrime != 0 -> Euler spiral. Compare the evaluator to an
    // independent, very fine composite-Simpson reference built with libm.
    Clothoid clothoid;
    clothoid.start = WorldPosition{1.0, 2.0, 0.0};
    clothoid.start.h = 0.1;
    clothoid.curvature = 0.02;
    clothoid.curvature_prime = 0.001;
    clothoid.length = 40.0;
    const TrajectoryEvaluator evaluator(Trajectory{"spiral", false, clothoid});
    ASSERT_TRUE(evaluator.ok());

    const auto theta = [&](double u) { return 0.1 + 0.02 * u + 0.5 * 0.001 * u * u; };
    for (const double s : {5.0, 17.3, 33.0, 40.0}) {
        // Reference: 200000-panel Simpson over [0, s].
        const int panels = 200000;
        const double h = s / panels;
        double ref_x = 1.0;
        double ref_y = 2.0;
        for (int k = 0; k < panels; ++k) {
            const double a = k * h;
            const double m = a + h * 0.5;
            const double b = a + h;
            ref_x +=
                (h / 6.0) * (std::cos(theta(a)) + 4.0 * std::cos(theta(m)) + std::cos(theta(b)));
            ref_y +=
                (h / 6.0) * (std::sin(theta(a)) + 4.0 * std::sin(theta(m)) + std::sin(theta(b)));
        }
        const Pose pose = evaluator.pose_at_arclength(s);
        EXPECT_NEAR(pose.x, ref_x, 1e-8);
        EXPECT_NEAR(pose.y, ref_y, 1e-8);
        EXPECT_NEAR(pose.heading, theta(s), 1e-12);
    }
}

Trajectory quarter_circle_nurbs(double radius) {
    // Standard exact rational quadratic quarter circle: control polygon
    // (R,0)-(R,R)-(0,R), middle weight cos(45deg) = 1/sqrt(2), knots {0,0,0,1,1,1}.
    scena::ir::Nurbs nurbs;
    nurbs.order = 3;
    const double w = 1.0 / std::sqrt(2.0);
    nurbs.control_points.push_back({WorldPosition{radius, 0.0, 0.0}, std::nullopt, 1.0});
    nurbs.control_points.push_back({WorldPosition{radius, radius, 0.0}, std::nullopt, w});
    nurbs.control_points.push_back({WorldPosition{0.0, radius, 0.0}, std::nullopt, 1.0});
    nurbs.knots = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
    return Trajectory{"circle", false, nurbs};
}

TEST(TrajectoryEvaluatorTest, NurbsQuadraticTracesAnExactCircle) {
    const double radius = 20.0;
    const TrajectoryEvaluator evaluator(quarter_circle_nurbs(radius));
    ASSERT_TRUE(evaluator.ok());
    // Arc length of a quarter circle is (pi/2) * R.
    EXPECT_NEAR(evaluator.total_length(), (kPi / 2.0) * radius, 1e-3);

    for (int i = 0; i <= 40; ++i) {
        const double s = evaluator.total_length() * (static_cast<double>(i) / 40.0);
        const Pose pose = evaluator.pose_at_arclength(s);
        // Every point lies exactly on the circle of radius R about the origin.
        EXPECT_NEAR(std::sqrt(pose.x * pose.x + pose.y * pose.y), radius, 1e-9);
    }
    // Endpoints hit the first and last control points exactly.
    const Pose begin = evaluator.pose_at_arclength(0.0);
    EXPECT_NEAR(begin.x, radius, 1e-12);
    EXPECT_NEAR(begin.y, 0.0, 1e-12);
    const Pose end = evaluator.pose_at_arclength(evaluator.total_length());
    EXPECT_NEAR(end.x, 0.0, 1e-9);
    EXPECT_NEAR(end.y, radius, 1e-9);
}

TEST(TrajectoryEvaluatorTest, NurbsDegreeOneIsAStraightLine) {
    scena::ir::Nurbs nurbs;
    nurbs.order = 2;
    nurbs.control_points.push_back({WorldPosition{0.0, 0.0, 0.0}, std::nullopt, 1.0});
    nurbs.control_points.push_back({WorldPosition{10.0, 0.0, 0.0}, std::nullopt, 1.0});
    nurbs.knots = {0.0, 0.0, 1.0, 1.0};
    const TrajectoryEvaluator evaluator(Trajectory{"seg", false, nurbs});
    ASSERT_TRUE(evaluator.ok());
    EXPECT_NEAR(evaluator.total_length(), 10.0, 1e-9);
    const Pose mid = evaluator.pose_at_arclength(5.0);
    EXPECT_NEAR(mid.x, 5.0, 1e-9);
    EXPECT_NEAR(mid.y, 0.0, 1e-12);
    EXPECT_NEAR(mid.heading, 0.0, 1e-12);
}

TEST(TrajectoryEvaluatorTest, MalformedNurbsIsRejected) {
    scena::ir::Nurbs nurbs;
    nurbs.order = 3;
    // Only two control points for order 3 -> violates the cardinality rule.
    nurbs.control_points.push_back({WorldPosition{0.0, 0.0, 0.0}, std::nullopt, 1.0});
    nurbs.control_points.push_back({WorldPosition{1.0, 0.0, 0.0}, std::nullopt, 1.0});
    nurbs.knots = {0.0, 0.0, 0.0, 1.0, 1.0};
    const TrajectoryEvaluator evaluator(Trajectory{"bad", false, nurbs});
    EXPECT_FALSE(evaluator.ok());
    EXPECT_EQ(evaluator.status().rule_id,
              "asam.net:xosc:1.0.0:routing.cardinality_of_control_points_in_nurbs");
}

TEST(TrajectoryEvaluatorTest, EvaluatorSamplesAreHexPinned) {
    // Bit-identity anchors captured once from the local build; the 3-OS CI
    // matrix proves they are universal. Regenerate only on an intentional
    // numeric change (release-noted, breaks trace goldens).
    const Trajectory poly =
        polyline_of({WorldPosition{0.0, 0.0, 0.0}, WorldPosition{10.0, 3.0, 0.0},
                     WorldPosition{4.0, 9.0, 1.0}});
    const TrajectoryEvaluator poly_eval(poly);
    const Pose p = poly_eval.pose_at_arclength(7.5);
    EXPECT_EQ(hex_bits(p.x), "401cbc1b1a54386b");
    EXPECT_EQ(hex_bits(p.y), "40013da9dc98eea7");
    EXPECT_EQ(hex_bits(p.heading), "3fd2a73a661eaf06");

    Clothoid clothoid;
    clothoid.start = WorldPosition{1.0, 2.0, 0.0};
    clothoid.start.h = 0.1;
    clothoid.curvature = 0.02;
    clothoid.curvature_prime = 0.001;
    clothoid.length = 40.0;
    const TrajectoryEvaluator spiral(Trajectory{"spiral", false, clothoid});
    const Pose c = spiral.pose_at_arclength(33.0);
    EXPECT_EQ(hex_bits(c.x), "403a6fb8696a3ff6");
    EXPECT_EQ(hex_bits(c.y), "4033bf9a264b29de");
    EXPECT_EQ(hex_bits(c.heading), "3ff4df3b645a1cac");

    const TrajectoryEvaluator circle(quarter_circle_nurbs(20.0));
    const Pose n = circle.pose_at_arclength(11.0);
    EXPECT_EQ(hex_bits(n.x), "40310cecf0d8881c");
    EXPECT_EQ(hex_bits(n.y), "4024e851363c0fe9");
    EXPECT_EQ(hex_bits(n.heading), "4000f76410ae1817");
}

} // namespace
