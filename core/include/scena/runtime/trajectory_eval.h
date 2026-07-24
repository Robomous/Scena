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

#pragma once

#include <string>
#include <vector>

#include "scena/ir/trajectory.h"
#include "scena/runtime/position_resolver.h" // Pose
#include "scena/status.h"

namespace scena::runtime {

/// Outcome of constructing a TrajectoryEvaluator. `status == Status::Ok` means
/// the shape is well formed and queryable; otherwise `message` explains why
/// (deterministic — element names and indices only, never a floating-point
/// value) and `rule_id` carries the ASAM checker rule UID when one applies
/// (NURBS cardinality).
struct TrajectoryEvalStatus {
    Status status = Status::Ok;
    std::string message;
    std::string rule_id;
};

/// Evaluates a trajectory's geometry as a function of arc length, per §6.9,
/// deterministically and with analytic fidelity (risk R3).
///
/// Standalone by design — it depends only on the IR `ir::Trajectory`, not on
/// the engine — so the clothoid quadrature and the NURBS de Boor recursion can
/// be unit-tested against analytic ground truth (circles, straights) without
/// booting a scenario. Construct one per FollowTrajectoryAction install, then
/// query `pose_at_arclength()` each step.
///
/// Numerical methods:
///  - Polyline: exact per-segment linear interpolation; heading is the segment
///    tangent via deterministic `det_atan2`.
///  - Clothoid: closed form for the straight-line and circular-arc degenerate
///    cases (bit-exact to the analytic reference); general spirals use a
///    fixed-step composite Simpson quadrature of the Fresnel-type integrand
///    `cos/sin theta(u)` through deterministic `det_sincos`. Error bound
///    O(h^4) per unit length with `h == kClothoidStep`; see trajectory_eval.cpp.
///  - NURBS: rational de Boor recursion (IEEE ops only, deterministic by
///    construction); arc length via a fixed-resolution cumulative table with a
///    linear s->u inversion; exact tangents from the rational derivative.
class TrajectoryEvaluator {
public:
    explicit TrajectoryEvaluator(const ir::Trajectory& trajectory);

    [[nodiscard]] bool ok() const noexcept { return status_.status == Status::Ok; }
    [[nodiscard]] const TrajectoryEvalStatus& status() const noexcept { return status_; }

    /// Total arc length of the path. Unit: [m]. Zero for an ill-formed shape.
    [[nodiscard]] double total_length() const noexcept { return total_length_; }

    /// The world pose at arc length `s`, clamped to `[0, total_length()]`.
    /// Heading is the curve tangent. Deterministic (`det_sincos` / IEEE-only).
    /// For an ill-formed shape the start pose (or the origin) is returned.
    [[nodiscard]] Pose pose_at_arclength(double s) const noexcept;

private:
    enum class Kind { Polyline, Clothoid, Nurbs };

    void build_polyline(const ir::Polyline& polyline);
    void build_clothoid(const ir::Clothoid& clothoid);
    void build_nurbs(const ir::Nurbs& nurbs);

    [[nodiscard]] Pose polyline_pose(double s) const noexcept;
    [[nodiscard]] Pose clothoid_pose(double s) const noexcept;
    [[nodiscard]] Pose nurbs_pose(double s) const noexcept;

    TrajectoryEvalStatus status_{};
    Kind kind_ = Kind::Polyline;
    double total_length_ = 0.0;

    // --- polyline state ---
    std::vector<Pose> vertices_;      ///< vertex poses (x/y/z, heading unused per-vertex)
    std::vector<double> arc_;         ///< cumulative arc length per vertex, arc_[0]==0
    std::vector<double> seg_heading_; ///< per-segment tangent heading, size vertices-1

    // --- clothoid state ---
    double c_x0_ = 0.0;
    double c_y0_ = 0.0;
    double c_z0_ = 0.0;
    double c_theta0_ = 0.0;
    double c_kappa0_ = 0.0;
    double c_kappa_prime_ = 0.0;
    double c_ds_ = 0.0;           ///< quadrature node spacing (0 for arc/line)
    std::vector<double> c_cum_x_; ///< cumulative integral of cosine-theta at each node
    std::vector<double> c_cum_y_; ///< cumulative integral of sine-theta at each node

    // --- nurbs state ---
    struct HControlPoint {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 1.0;
    };
    /// de Boor evaluation of a homogeneous B-spline of the given `order` at
    /// parameter `u`. IEEE ops only, so bit-identical across platforms.
    [[nodiscard]] static HControlPoint de_boor(const std::vector<HControlPoint>& cp,
                                               const std::vector<double>& knots, unsigned int order,
                                               double u) noexcept;
    unsigned int n_order_ = 2;
    std::vector<HControlPoint> n_cp_;  ///< homogeneous control points (w-scaled)
    std::vector<double> n_knots_;      ///< knot vector
    std::vector<HControlPoint> n_dcp_; ///< derivative homogeneous control points
    std::vector<double> n_dknots_;     ///< derivative knot vector
    std::vector<double> n_u_;          ///< arc-length table: parameter samples
    std::vector<double> n_arc_;        ///< arc-length table: cumulative length at n_u_
};

} // namespace scena::runtime
