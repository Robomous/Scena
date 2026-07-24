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

#include "scena/runtime/trajectory_eval.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <variant>

#include "scena/runtime/detmath.h"

namespace scena::runtime {
namespace {

// Composite-Simpson node spacing for the general clothoid quadrature. At 1 cm
// the per-interval error is O(ds^5 * |d4/du4 cos theta|); for the curvatures
// and lengths of realistic drivable spirals the accumulated error stays well
// under 1e-9 m (see trajectory_test.cpp), while a degenerate arc or line never
// reaches this path (closed form). See §6.9 and ADR-0018.
constexpr double kClothoidStep = 0.01;

// Upper bound on quadrature nodes so a pathological length cannot exhaust
// memory; beyond it the step grows (coarser, still deterministic).
constexpr std::size_t kMaxClothoidNodes = 1u << 20;

[[nodiscard]] bool is_finite_position(const ir::WorldPosition& p) noexcept {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

// Index of the segment [bounds[i], bounds[i+1]] containing value, clamped into
// [0, bounds.size()-2]. bounds is ascending with at least two entries.
[[nodiscard]] std::size_t segment_index(const std::vector<double>& bounds, double value) noexcept {
    if (value <= bounds.front()) {
        return 0;
    }
    if (value >= bounds.back()) {
        return bounds.size() - 2;
    }
    std::size_t index = 0;
    while (index + 2 < bounds.size() && value > bounds[index + 1]) {
        ++index;
    }
    return index;
}

} // namespace

TrajectoryEvaluator::TrajectoryEvaluator(const ir::Trajectory& trajectory) {
    std::visit(
        [this](const auto& shape) {
            using T = std::decay_t<decltype(shape)>;
            if constexpr (std::is_same_v<T, ir::Polyline>) {
                kind_ = Kind::Polyline;
                build_polyline(shape);
            } else if constexpr (std::is_same_v<T, ir::Clothoid>) {
                kind_ = Kind::Clothoid;
                build_clothoid(shape);
            } else {
                kind_ = Kind::Nurbs;
                build_nurbs(shape);
            }
        },
        trajectory.shape);
}

void TrajectoryEvaluator::build_polyline(const ir::Polyline& polyline) {
    // §Polyline: an ordered chain of at least two vertices.
    if (polyline.vertices.size() < 2) {
        status_ = {Status::ValidationError, "trajectory needs at least two vertices", {}};
        return;
    }
    vertices_.reserve(polyline.vertices.size());
    arc_.reserve(polyline.vertices.size());
    seg_heading_.reserve(polyline.vertices.size() - 1);
    double cumulative = 0.0;
    for (std::size_t index = 0; index < polyline.vertices.size(); ++index) {
        const ir::WorldPosition& position = polyline.vertices[index].position;
        if (!is_finite_position(position)) {
            status_ = {Status::ValidationError, "trajectory vertex position must be finite", {}};
            return;
        }
        Pose pose;
        pose.x = position.x;
        pose.y = position.y;
        pose.z = position.z;
        if (index > 0) {
            const double dx = position.x - vertices_[index - 1].x;
            const double dy = position.y - vertices_[index - 1].y;
            const double dz = position.z - vertices_[index - 1].z;
            cumulative += std::sqrt(dx * dx + dy * dy + dz * dz);
            // Segment tangent; the single det_atan2 site for the polyline path.
            seg_heading_.push_back(det_atan2(dy, dx));
        }
        vertices_.push_back(pose);
        arc_.push_back(cumulative);
    }
    total_length_ = arc_.back();
}

Pose TrajectoryEvaluator::polyline_pose(double s) const noexcept {
    if (vertices_.empty()) {
        return Pose{};
    }
    const double clamped = std::clamp(s, 0.0, total_length_);
    const std::size_t index = segment_index(arc_, clamped);
    const double span = arc_[index + 1] - arc_[index];
    const double fraction = span > 0.0 ? (clamped - arc_[index]) / span : 0.0;
    const Pose& a = vertices_[index];
    const Pose& b = vertices_[index + 1];
    Pose pose;
    pose.x = a.x + (b.x - a.x) * fraction;
    pose.y = a.y + (b.y - a.y) * fraction;
    pose.z = a.z + (b.z - a.z) * fraction;
    pose.heading = seg_heading_[index];
    return pose;
}

void TrajectoryEvaluator::build_clothoid(const ir::Clothoid& clothoid) {
    if (!is_finite_position(clothoid.start) || !std::isfinite(clothoid.start.h) ||
        !std::isfinite(clothoid.curvature) || !std::isfinite(clothoid.curvature_prime) ||
        !std::isfinite(clothoid.length) || clothoid.length < 0.0) {
        status_ = {Status::ValidationError,
                   "clothoid needs finite curvature, curvaturePrime and a non-negative length",
                   {}};
        return;
    }
    c_x0_ = clothoid.start.x;
    c_y0_ = clothoid.start.y;
    c_z0_ = clothoid.start.z;
    c_theta0_ = clothoid.start.h;
    c_kappa0_ = clothoid.curvature;
    c_kappa_prime_ = clothoid.curvature_prime;
    total_length_ = clothoid.length;

    // Straight line and circular arc are closed form (see clothoid_pose); no
    // quadrature table needed for them.
    if (c_kappa_prime_ == 0.0 || total_length_ == 0.0) {
        c_ds_ = 0.0;
        return;
    }

    // General Euler spiral: precompute the cumulative Fresnel-type integrals at
    // fixed nodes with composite Simpson, so per-query cost is O(1) plus a tiny
    // residual panel.
    std::size_t nodes = static_cast<std::size_t>(std::ceil(total_length_ / kClothoidStep));
    nodes = std::clamp<std::size_t>(nodes, 1, kMaxClothoidNodes);
    c_ds_ = total_length_ / static_cast<double>(nodes);
    c_cum_x_.assign(nodes + 1, 0.0);
    c_cum_y_.assign(nodes + 1, 0.0);
    double cum_x = 0.0;
    double cum_y = 0.0;
    for (std::size_t k = 0; k < nodes; ++k) {
        const double a = static_cast<double>(k) * c_ds_;
        const double m = a + c_ds_ * 0.5;
        const double b = a + c_ds_;
        const SinCos fa = det_sincos(c_theta0_ + c_kappa0_ * a + 0.5 * c_kappa_prime_ * a * a);
        const SinCos fm = det_sincos(c_theta0_ + c_kappa0_ * m + 0.5 * c_kappa_prime_ * m * m);
        const SinCos fb = det_sincos(c_theta0_ + c_kappa0_ * b + 0.5 * c_kappa_prime_ * b * b);
        cum_x += (c_ds_ / 6.0) * (fa.cos + 4.0 * fm.cos + fb.cos);
        cum_y += (c_ds_ / 6.0) * (fa.sin + 4.0 * fm.sin + fb.sin);
        c_cum_x_[k + 1] = cum_x;
        c_cum_y_[k + 1] = cum_y;
    }
}

Pose TrajectoryEvaluator::clothoid_pose(double s) const noexcept {
    const double clamped = std::clamp(s, 0.0, total_length_);
    Pose pose;
    pose.z = c_z0_;
    // Heading is the tangent theta(s) = theta0 + kappa0*s + 0.5*kappa'*s^2.
    pose.heading = c_theta0_ + c_kappa0_ * clamped + 0.5 * c_kappa_prime_ * clamped * clamped;

    if (c_kappa_prime_ == 0.0 && c_kappa0_ == 0.0) {
        // Straight line.
        const SinCos t0 = det_sincos(c_theta0_);
        pose.x = c_x0_ + clamped * t0.cos;
        pose.y = c_y0_ + clamped * t0.sin;
        return pose;
    }
    if (c_kappa_prime_ == 0.0) {
        // Circular arc: exact integral of the constant-curvature tangent.
        const SinCos t0 = det_sincos(c_theta0_);
        const SinCos ts = det_sincos(c_theta0_ + c_kappa0_ * clamped);
        pose.x = c_x0_ + (ts.sin - t0.sin) / c_kappa0_;
        pose.y = c_y0_ - (ts.cos - t0.cos) / c_kappa0_;
        return pose;
    }

    // General spiral: cumulative table plus a residual Simpson panel over the
    // partial interval [node, s].
    if (c_ds_ <= 0.0 || c_cum_x_.empty()) {
        pose.x = c_x0_;
        pose.y = c_y0_;
        return pose;
    }
    std::size_t k = static_cast<std::size_t>(clamped / c_ds_);
    if (k >= c_cum_x_.size() - 1) {
        k = c_cum_x_.size() - 2;
    }
    const double sk = static_cast<double>(k) * c_ds_;
    const double r = clamped - sk;
    double res_x = 0.0;
    double res_y = 0.0;
    if (r > 0.0) {
        const double a = sk;
        const double m = sk + r * 0.5;
        const double b = sk + r;
        const SinCos fa = det_sincos(c_theta0_ + c_kappa0_ * a + 0.5 * c_kappa_prime_ * a * a);
        const SinCos fm = det_sincos(c_theta0_ + c_kappa0_ * m + 0.5 * c_kappa_prime_ * m * m);
        const SinCos fb = det_sincos(c_theta0_ + c_kappa0_ * b + 0.5 * c_kappa_prime_ * b * b);
        res_x = (r / 6.0) * (fa.cos + 4.0 * fm.cos + fb.cos);
        res_y = (r / 6.0) * (fa.sin + 4.0 * fm.sin + fb.sin);
    }
    pose.x = c_x0_ + c_cum_x_[k] + res_x;
    pose.y = c_y0_ + c_cum_y_[k] + res_y;
    return pose;
}

void TrajectoryEvaluator::build_nurbs(const ir::Nurbs& nurbs) {
    // Filled in the NURBS commit (de Boor + arc-length table).
    n_order_ = nurbs.order;
    status_ = {Status::UnsupportedFeature, "NURBS evaluation not implemented yet", {}};
}

Pose TrajectoryEvaluator::nurbs_pose(double /*s*/) const noexcept {
    return Pose{c_x0_, c_y0_, c_z0_, 0.0, 0.0, 0.0};
}

Pose TrajectoryEvaluator::pose_at_arclength(double s) const noexcept {
    if (!ok()) {
        return Pose{};
    }
    switch (kind_) {
    case Kind::Polyline:
        return polyline_pose(s);
    case Kind::Clothoid:
        return clothoid_pose(s);
    case Kind::Nurbs:
        return nurbs_pose(s);
    }
    return Pose{};
}

} // namespace scena::runtime
