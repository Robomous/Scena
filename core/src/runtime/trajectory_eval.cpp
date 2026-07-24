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

// Samples in the NURBS arc-length table. The s->u inversion is linear within a
// cell, so the arc-length error is O(1/kNurbsArcSamples^2); the evaluated point
// is always exactly on the curve regardless. See ADR-0018.
constexpr std::size_t kNurbsArcSamples = 4096;

// per rule asam.net:xosc:1.0.0:routing.cardinality_of_control_points_in_nurbs
constexpr const char* kRuleNurbsCardinality =
    "asam.net:xosc:1.0.0:routing.cardinality_of_control_points_in_nurbs";

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

TrajectoryEvaluator::HControlPoint
TrajectoryEvaluator::de_boor(const std::vector<HControlPoint>& cp, const std::vector<double>& knots,
                             unsigned int order, double u) noexcept {
    const unsigned int p = order - 1;
    const std::size_t n = cp.size() - 1;
    // Clamp into the valid parameter domain [knots[p], knots[n+1]].
    const double u0 = knots[p];
    const double u1 = knots[n + 1];
    u = std::clamp(u, u0, u1);
    // Knot span l with knots[l] <= u <= knots[l+1], l in [p, n].
    std::size_t l = p;
    if (u >= u1) {
        l = n;
    } else {
        while (l < n && u >= knots[l + 1]) {
            ++l;
        }
    }
    HControlPoint d[64];
    for (unsigned int j = 0; j <= p; ++j) {
        d[j] = cp[l - p + j];
    }
    for (unsigned int r = 1; r <= p; ++r) {
        for (unsigned int j = p; j >= r; --j) {
            const std::size_t i = l - p + j;
            const double denom = knots[i + p - r + 1] - knots[i];
            const double alpha = denom > 0.0 ? (u - knots[i]) / denom : 0.0;
            const double beta = 1.0 - alpha;
            d[j].x = beta * d[j - 1].x + alpha * d[j].x;
            d[j].y = beta * d[j - 1].y + alpha * d[j].y;
            d[j].z = beta * d[j - 1].z + alpha * d[j].z;
            d[j].w = beta * d[j - 1].w + alpha * d[j].w;
        }
    }
    return d[p];
}

void TrajectoryEvaluator::build_nurbs(const ir::Nurbs& nurbs) {
    n_order_ = nurbs.order;
    const unsigned int order = nurbs.order;
    const std::size_t cp_count = nurbs.control_points.size();
    // §Nurbs cardinality: order >= 2, control_points >= order, knots ==
    // control_points + order.
    if (order < 2 || cp_count < order || nurbs.knots.size() != cp_count + order) {
        status_ = {Status::ValidationError,
                   "NURBS needs order>=2, control_points>=order and knots==control_points+order",
                   kRuleNurbsCardinality};
        return;
    }
    if (order - 1u >= 64u) { // de Boor scratch bound; realistic orders are 2..4
        status_ = {Status::UnsupportedFeature, "NURBS order is too high", {}};
        return;
    }
    for (std::size_t i = 0; i < nurbs.knots.size(); ++i) {
        if (!std::isfinite(nurbs.knots[i]) || (i > 0 && nurbs.knots[i] < nurbs.knots[i - 1])) {
            status_ = {Status::ValidationError, "NURBS knot vector must be finite and ascending",
                       kRuleNurbsCardinality};
            return;
        }
    }
    n_cp_.reserve(cp_count);
    for (const ir::ControlPoint& cp : nurbs.control_points) {
        if (!is_finite_position(cp.position) || !std::isfinite(cp.weight) || cp.weight <= 0.0) {
            status_ = {Status::ValidationError,
                       "NURBS control point needs a finite position and a positive weight",
                       {}};
            return;
        }
        n_cp_.push_back(HControlPoint{cp.position.x * cp.weight, cp.position.y * cp.weight,
                                      cp.position.z * cp.weight, cp.weight});
    }
    n_knots_ = nurbs.knots;

    const unsigned int p = order - 1;
    const double u0 = n_knots_[p];
    const double u1 = n_knots_[cp_count]; // knots[n+1], n = cp_count - 1
    if (!(u1 > u0)) {
        status_ = {Status::ValidationError, "NURBS parameter domain is empty",
                   kRuleNurbsCardinality};
        return;
    }

    // Derivative homogeneous control points (a degree p-1 B-spline), for exact
    // tangents: Q_i = p * (P_{i+1} - P_i) / (knots[i+p+1] - knots[i+1]).
    if (p >= 1) {
        n_dcp_.reserve(cp_count - 1);
        for (std::size_t i = 0; i + 1 < cp_count; ++i) {
            const double denom = n_knots_[i + p + 1] - n_knots_[i + 1];
            const double scale = denom > 0.0 ? static_cast<double>(p) / denom : 0.0;
            n_dcp_.push_back(HControlPoint{
                (n_cp_[i + 1].x - n_cp_[i].x) * scale, (n_cp_[i + 1].y - n_cp_[i].y) * scale,
                (n_cp_[i + 1].z - n_cp_[i].z) * scale, (n_cp_[i + 1].w - n_cp_[i].w) * scale});
        }
        n_dknots_.assign(n_knots_.begin() + 1, n_knots_.end() - 1);
    }

    // Arc-length table: fine uniform sampling in u, cumulative chord length.
    // The s->u inversion is linear within a table cell, so the error is
    // O(1/kNurbsArcSamples^2) in arc length; the evaluated point is always
    // exactly on the curve (de Boor is exact). See ADR-0018.
    n_u_.assign(kNurbsArcSamples + 1, 0.0);
    n_arc_.assign(kNurbsArcSamples + 1, 0.0);
    HControlPoint start = de_boor(n_cp_, n_knots_, order, u0);
    double prev_x = start.x / start.w;
    double prev_y = start.y / start.w;
    double prev_z = start.z / start.w;
    n_u_[0] = u0;
    double cumulative = 0.0;
    for (std::size_t j = 1; j <= kNurbsArcSamples; ++j) {
        const double u =
            u0 + (u1 - u0) * (static_cast<double>(j) / static_cast<double>(kNurbsArcSamples));
        const HControlPoint h = de_boor(n_cp_, n_knots_, order, u);
        const double x = h.x / h.w;
        const double y = h.y / h.w;
        const double z = h.z / h.w;
        const double dx = x - prev_x;
        const double dy = y - prev_y;
        const double dz = z - prev_z;
        cumulative += std::sqrt(dx * dx + dy * dy + dz * dz);
        n_u_[j] = u;
        n_arc_[j] = cumulative;
        prev_x = x;
        prev_y = y;
        prev_z = z;
    }
    total_length_ = cumulative;
}

Pose TrajectoryEvaluator::nurbs_pose(double s) const noexcept {
    Pose pose;
    if (n_cp_.empty() || n_u_.size() < 2) {
        return pose;
    }
    const double clamped = std::clamp(s, 0.0, total_length_);
    // Invert the arc-length table linearly to get the curve parameter u.
    const std::size_t j = segment_index(n_arc_, clamped);
    const double span = n_arc_[j + 1] - n_arc_[j];
    const double fraction = span > 0.0 ? (clamped - n_arc_[j]) / span : 0.0;
    const double u = n_u_[j] + (n_u_[j + 1] - n_u_[j]) * fraction;

    const HControlPoint c = de_boor(n_cp_, n_knots_, n_order_, u);
    pose.x = c.x / c.w;
    pose.y = c.y / c.w;
    pose.z = c.z / c.w;

    // Tangent of the rational curve: C' = (A'*w - A*w') / w^2, heading from the
    // planar (x, y) components.
    if (!n_dcp_.empty()) {
        const HControlPoint d = de_boor(n_dcp_, n_dknots_, n_order_ - 1, u);
        const double w = c.w;
        const double dx = (d.x * w - c.x * d.w) / (w * w);
        const double dy = (d.y * w - c.y * d.w) / (w * w);
        pose.heading = det_atan2(dy, dx);
    }
    return pose;
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
