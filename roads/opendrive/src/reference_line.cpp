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

#include "scena/opendrive/reference_line.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "scena/ir/trajectory.h"
#include "scena/runtime/detmath.h"

namespace scena::opendrive {

namespace {

using runtime::det_atan2;
using runtime::det_sincos;
using runtime::SinCos;

/// Nodes per poly3/paramPoly3 parameter table. 1024 intervals keep the
/// chord-length error of the s->parameter mapping O((L/1024)^2) — well under
/// a millimeter for any road-scale element — at a fixed, platform-identical
/// cost.
constexpr std::size_t kPolyIntervals = 1024;

/// Fixed iteration count of the projection refinement. Ternary search
/// shrinks the bracket by 2/3 per step: 64 steps reduce any road-scale
/// bracket far below double resolution, and a fixed count (never an epsilon
/// exit) keeps the loop shape platform-independent.
constexpr int kProjectIterations = 64;

/// Local (u, v) of the poly3 cubic at parameter u.
struct UV {
    double u = 0.0;
    double v = 0.0;
};

UV poly3_at(const Geometry& geometry, double u) {
    // v(u) = a + b u + c u^2 + d u^3 (§9.7), Horner form.
    return {u, geometry.a + u * (geometry.b + u * (geometry.c + u * geometry.d))};
}

UV param_poly3_at(const Geometry& geometry, double p) {
    // u(p), v(p) cubics (§9.6.1), Horner form.
    return {geometry.a_u + p * (geometry.b_u + p * (geometry.c_u + p * geometry.d_u)),
            geometry.a_v + p * (geometry.b_v + p * (geometry.c_v + p * geometry.d_v))};
}

UV poly_derivative_at(const Geometry& geometry, double param) {
    if (geometry.kind == GeometryKind::Poly3) {
        // d/du of v(u); du/du = 1.
        return {1.0, geometry.b + param * (2.0 * geometry.c + param * (3.0 * geometry.d))};
    }
    return {geometry.b_u + param * (2.0 * geometry.c_u + param * (3.0 * geometry.d_u)),
            geometry.b_v + param * (2.0 * geometry.c_v + param * (3.0 * geometry.d_v))};
}

UV poly_at(const Geometry& geometry, double param) {
    return geometry.kind == GeometryKind::Poly3 ? poly3_at(geometry, param)
                                                : param_poly3_at(geometry, param);
}

/// Rotates the local (u, v) into the inertial frame anchored at the element
/// start (§9.6.1 / §9.7.2 transformation).
RefPose to_inertial(const Geometry& geometry, const UV& local, const UV& derivative) {
    const SinCos rot = det_sincos(geometry.hdg);
    RefPose pose;
    pose.x = geometry.x + local.u * rot.cos - local.v * rot.sin;
    pose.y = geometry.y + local.u * rot.sin + local.v * rot.cos;
    const double tx = derivative.u * rot.cos - derivative.v * rot.sin;
    const double ty = derivative.u * rot.sin + derivative.v * rot.cos;
    pose.heading = det_atan2(ty, tx);
    return pose;
}

} // namespace

ReferenceLine::ReferenceLine(const Road& road) {
    double cumulative = 0.0;
    for (const Geometry& geometry : road.plan_view) {
        if (!(geometry.length > 0.0) || !std::isfinite(geometry.length)) {
            elements_.clear();
            total_length_ = 0.0;
            return;
        }
        Element element;
        element.geom = geometry;
        element.start_s = cumulative;
        switch (geometry.kind) {
        case GeometryKind::Line:
        case GeometryKind::Arc:
        case GeometryKind::Spiral: {
            // The kappa-linear family maps 1:1 onto the IR clothoid: a line
            // is kappa = kappa' = 0, an arc kappa' = 0, and an OpenDRIVE
            // spiral has kappa' = (curvEnd - curvStart) / length (§9.4:
            // curvature is linear along the arc length). Evaluating through
            // TrajectoryEvaluator shares its closed forms and deterministic
            // Simpson quadrature instead of duplicating them here.
            ir::Clothoid clothoid;
            clothoid.start.x = geometry.x;
            clothoid.start.y = geometry.y;
            clothoid.start.h = geometry.hdg;
            clothoid.length = geometry.length;
            if (geometry.kind == GeometryKind::Arc) {
                clothoid.curvature = geometry.curvature;
            } else if (geometry.kind == GeometryKind::Spiral) {
                clothoid.curvature = geometry.curv_start;
                clothoid.curvature_prime =
                    (geometry.curv_end - geometry.curv_start) / geometry.length;
            }
            element.clothoid.emplace(ir::Trajectory{{}, false, clothoid});
            if (!element.clothoid->ok()) {
                elements_.clear();
                total_length_ = 0.0;
                return;
            }
            break;
        }
        case GeometryKind::Poly3:
        case GeometryKind::ParamPoly3: {
            // Fixed-resolution cumulative chord table for the s <-> parameter
            // mapping. For paramPoly3 the parameter range is prescribed
            // (§9.6 @pRange); for the deprecated poly3 the u-extent is not
            // stored in the file, so u is grown in fixed steps until the
            // chord length reaches @length — arc length grows at least as
            // fast as u (|dP/du| >= 1), so the loop is bounded.
            PolyTable& table = element.table;
            table.param.reserve(kPolyIntervals + 1);
            table.arc.reserve(kPolyIntervals + 1);
            table.param.push_back(0.0);
            table.arc.push_back(0.0);
            UV previous = poly_at(geometry, 0.0);
            if (geometry.kind == GeometryKind::ParamPoly3) {
                const double p_end = geometry.p_range == PRange::Normalized ? 1.0 : geometry.length;
                for (std::size_t i = 1; i <= kPolyIntervals; ++i) {
                    const double p =
                        p_end * (static_cast<double>(i) / static_cast<double>(kPolyIntervals));
                    const UV current = poly_at(geometry, p);
                    const double du = current.u - previous.u;
                    const double dv = current.v - previous.v;
                    table.param.push_back(p);
                    table.arc.push_back(table.arc.back() + std::sqrt(du * du + dv * dv));
                    previous = current;
                }
            } else {
                const double du = geometry.length / static_cast<double>(kPolyIntervals);
                double u = 0.0;
                // 8x headroom over the minimum step count, then truncation:
                // a pathological cubic ends early rather than looping on.
                for (std::size_t i = 0; i < 8 * kPolyIntervals; ++i) {
                    if (table.arc.back() >= geometry.length) {
                        break;
                    }
                    u += du;
                    const UV current = poly_at(geometry, u);
                    const double step_u = current.u - previous.u;
                    const double step_v = current.v - previous.v;
                    table.param.push_back(u);
                    table.arc.push_back(table.arc.back() +
                                        std::sqrt(step_u * step_u + step_v * step_v));
                    previous = current;
                }
            }
            if (table.arc.back() <= 0.0 || !std::isfinite(table.arc.back())) {
                elements_.clear();
                total_length_ = 0.0;
                return;
            }
            break;
        }
        }
        cumulative += geometry.length;
        elements_.push_back(std::move(element));
    }
    total_length_ = cumulative;
    ok_ = !elements_.empty();
}

RefPose ReferenceLine::poly_pose(const Element& element, double local_s) noexcept {
    const PolyTable& table = element.table;
    // Scale the queried arc length so the element's declared @length spans
    // the whole table: the spec ties the parameter end to @length, and the
    // chord total carries the discretization remainder
    // (asam.net:xodr:1.7.0:road.geometry.paramPoly3.length_match is a
    // "should", not an exact equality).
    const double target = std::clamp(local_s / element.geom.length, 0.0, 1.0) * table.arc.back();
    const auto upper = std::upper_bound(table.arc.begin(), table.arc.end(), target);
    const std::size_t hi = std::min<std::size_t>(
        static_cast<std::size_t>(std::max<std::ptrdiff_t>(upper - table.arc.begin(), 1)),
        table.arc.size() - 1);
    const std::size_t lo = hi - 1;
    const double span = table.arc[hi] - table.arc[lo];
    const double fraction = span > 0.0 ? (target - table.arc[lo]) / span : 0.0;
    const double param = table.param[lo] + fraction * (table.param[hi] - table.param[lo]);
    return to_inertial(element.geom, poly_at(element.geom, param),
                       poly_derivative_at(element.geom, param));
}

RefPose ReferenceLine::element_pose(const Element& element, double local_s) const noexcept {
    if (element.clothoid.has_value()) {
        const runtime::Pose pose = element.clothoid->pose_at_arclength(local_s);
        return {pose.x, pose.y, pose.heading};
    }
    return poly_pose(element, local_s);
}

RefPose ReferenceLine::pose_at(double s) const noexcept {
    if (!ok_ || !std::isfinite(s)) {
        return {};
    }
    const double clamped = std::clamp(s, 0.0, total_length_);
    // Find the element whose [start_s, start_s + length] contains s; the
    // last element also owns s == length() (clamped upper edge).
    std::size_t index = elements_.size() - 1;
    for (std::size_t i = 0; i < elements_.size(); ++i) {
        if (clamped < elements_[i].start_s + elements_[i].geom.length) {
            index = i;
            break;
        }
    }
    const Element& element = elements_[index];
    return element_pose(element, clamped - element.start_s);
}

std::optional<TrackPosition> ReferenceLine::project(double x, double y) const noexcept {
    if (!ok_ || !std::isfinite(x) || !std::isfinite(y)) {
        return std::nullopt;
    }

    const auto distance_squared = [&](double s) {
        const RefPose pose = pose_at(s);
        const double dx = x - pose.x;
        const double dy = y - pose.y;
        return dx * dx + dy * dy;
    };

    // Coarse pass: fixed-stride samples over the whole line. Strict '<'
    // keeps the first (smallest-s) candidate on exact ties — the documented
    // deterministic tie-break.
    const std::size_t samples = std::min<std::size_t>(
        4096, std::max<std::size_t>(64, static_cast<std::size_t>(total_length_) * 4));
    double best_s = 0.0;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i <= samples; ++i) {
        const double s = total_length_ * (static_cast<double>(i) / static_cast<double>(samples));
        const double d2 = distance_squared(s);
        if (d2 < best_d2) {
            best_d2 = d2;
            best_s = s;
        }
    }

    // Refinement: fixed-iteration ternary search on the bracket around the
    // best coarse sample. The distance to a smooth curve is locally
    // unimodal; a fixed count keeps the loop platform-identical.
    const double stride = total_length_ / static_cast<double>(samples);
    double lo = std::max(0.0, best_s - stride);
    double hi = std::min(total_length_, best_s + stride);
    for (int i = 0; i < kProjectIterations; ++i) {
        const double third = (hi - lo) / 3.0;
        const double m1 = lo + third;
        const double m2 = hi - third;
        if (distance_squared(m1) <= distance_squared(m2)) {
            hi = m2;
        } else {
            lo = m1;
        }
    }
    const double s = (lo + hi) / 2.0;

    // Signed lateral offset: component of (point - foot) along the left
    // normal of the s-axis tangent — t is positive to the left (§8.3).
    const RefPose foot = pose_at(s);
    const SinCos tangent = det_sincos(foot.heading);
    const double t = -(x - foot.x) * tangent.sin + (y - foot.y) * tangent.cos;
    return TrackPosition{s, t};
}

} // namespace scena::opendrive
