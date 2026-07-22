// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#include "scena/runtime/obb2.h"

#include <array>
#include <cmath>

namespace scena::runtime {

namespace {

/// Projected radius (half-width) of box `box` onto the unit axis (ux, uy): the
/// sum of each half extent times the absolute projection of that body axis.
/// Body x axis is (cos_h, sin_h), body y axis is (-sin_h, cos_h). Fixed
/// operand order; std::fabs is IEEE-exact.
double projected_radius(const Obb2& box, double ux, double uy) {
    const double ax = box.cos_h * ux + box.sin_h * uy;  // body x axis . u
    const double ay = -box.sin_h * ux + box.cos_h * uy; // body y axis . u
    return box.hx * std::fabs(ax) + box.hy * std::fabs(ay);
}

/// True when the boxes are separated along the unit axis (ux, uy): the gap
/// between projected centers exceeds the sum of projected radii. Strict `>` so
/// touching boxes are NOT separated (freespace distance 0 on contact,
/// §6.4.7.2).
bool separated_on_axis(const Obb2& a, const Obb2& b, double ux, double uy) {
    const double center_gap = std::fabs((b.cx - a.cx) * ux + (b.cy - a.cy) * uy);
    return center_gap > projected_radius(a, ux, uy) + projected_radius(b, ux, uy);
}

/// The four world corners of `box` in the fixed corner order documented on
/// Obb2. Signed local half-extents (kx, ky) rotate by the heading:
/// (cx + kx*cos_h - ky*sin_h, cy + kx*sin_h + ky*cos_h).
std::array<double, 8> world_corners(const Obb2& box) {
    const std::array<double, 8> local = {
        box.hx,  box.hy,  // 0: (+hx, +hy)
        box.hx,  -box.hy, // 1: (+hx, -hy)
        -box.hx, -box.hy, // 2: (-hx, -hy)
        -box.hx, box.hy,  // 3: (-hx, +hy)
    };
    std::array<double, 8> world = {};
    for (int i = 0; i < 4; ++i) {
        const double kx = local[static_cast<std::size_t>(2 * i)];
        const double ky = local[static_cast<std::size_t>(2 * i + 1)];
        world[static_cast<std::size_t>(2 * i)] = box.cx + kx * box.cos_h - ky * box.sin_h;
        world[static_cast<std::size_t>(2 * i + 1)] = box.cy + kx * box.sin_h + ky * box.cos_h;
    }
    return world;
}

/// Squared Euclidean distance from a world point to the filled box `b`, 0 when
/// inside. The point is expressed in b's body frame (rotate by -heading),
/// clamped to the box extents, and the residual is the offset to the nearest
/// surface point. std::fmin/std::fmax are IEEE-exact; fixed operand order.
double point_obb_distance_sq(double px, double py, const Obb2& b) {
    const double rx = px - b.cx;
    const double ry = py - b.cy;
    // Body-frame coordinates: R^T applied to (rx, ry).
    const double lx = rx * b.cos_h + ry * b.sin_h;
    const double ly = -rx * b.sin_h + ry * b.cos_h;
    // Nearest point inside the extents, then the residual outside them.
    const double ox = lx - std::fmax(-b.hx, std::fmin(lx, b.hx));
    const double oy = ly - std::fmax(-b.hy, std::fmin(ly, b.hy));
    return ox * ox + oy * oy;
}

} // namespace

bool obb_intersects(const Obb2& a, const Obb2& b) {
    // Four axes in fixed order: a.x, a.y, b.x, b.y. A single separating axis
    // proves disjointness.
    if (separated_on_axis(a, b, a.cos_h, a.sin_h)) {
        return false;
    }
    if (separated_on_axis(a, b, -a.sin_h, a.cos_h)) {
        return false;
    }
    if (separated_on_axis(a, b, b.cos_h, b.sin_h)) {
        return false;
    }
    if (separated_on_axis(a, b, -b.sin_h, b.cos_h)) {
        return false;
    }
    return true;
}

double obb_distance(const Obb2& a, const Obb2& b) {
    if (obb_intersects(a, b)) {
        return 0.0; // touching or overlapping ⇒ 0 (§6.4.7.2)
    }
    // Disjoint convex rectangles: the closest pair includes a vertex of one
    // box, so the minimum is over each box's corners against the other box.
    // Accumulate squared distances (min via std::fmin) and take one std::sqrt.
    const std::array<double, 8> a_corners = world_corners(a);
    const std::array<double, 8> b_corners = world_corners(b);
    double min_sq = point_obb_distance_sq(a_corners[0], a_corners[1], b);
    for (int i = 1; i < 4; ++i) {
        min_sq = std::fmin(
            min_sq, point_obb_distance_sq(a_corners[static_cast<std::size_t>(2 * i)],
                                          a_corners[static_cast<std::size_t>(2 * i + 1)], b));
    }
    for (int i = 0; i < 4; ++i) {
        min_sq = std::fmin(
            min_sq, point_obb_distance_sq(b_corners[static_cast<std::size_t>(2 * i)],
                                          b_corners[static_cast<std::size_t>(2 * i + 1)], a));
    }
    return std::sqrt(min_sq);
}

double point_obb_distance(double px, double py, const Obb2& b) {
    return std::sqrt(point_obb_distance_sq(px, py, b));
}

void obb_project(const Obb2& a, double ux, double uy, double& lo, double& hi) {
    const double center = a.cx * ux + a.cy * uy;
    const double radius = projected_radius(a, ux, uy);
    lo = center - radius;
    hi = center + radius;
}

} // namespace scena::runtime
