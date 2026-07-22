// SPDX-License-Identifier: Apache-2.0
#pragma once

namespace scena::runtime {

/// A 2D oriented bounding box on the ground plane, the freespace primitive the
/// interaction conditions (p5-s3) reduce entity geometry to.
///
/// An entity's spec bounding box (center offset + length/width in the body
/// frame) projects exactly to a heading-rotated rectangle because EntityState
/// carries yaw only — height and the z center offset never enter the planar
/// math (ADR-0009). The caller builds an Obb2 from an entity's world position,
/// heading, and box, performing the ONLY trigonometry (through det_sincos)
/// once; every routine here is pure multiply/add/compare plus sqrt/fabs/
/// fmin/fmax, all IEEE-exact, so results are bit-identical across platforms
/// (the determinism contract) and raw-distance outputs can be hex-pinned.
///
/// `cx`/`cy` is the world-frame box center = entity position + R(heading)·(box
/// center offset). `cos_h`/`sin_h` come from det_sincos(heading) — the box's
/// body x axis is (cos_h, sin_h), its body y axis is (-sin_h, cos_h).
/// `hx`/`hy` are the half extents: length/2 along body x, width/2 along body y.
///
/// The four corners are enumerated in this fixed order everywhere in the
/// implementation so operand order never varies:
///   0: (+hx, +hy)  1: (+hx, -hy)  2: (-hx, -hy)  3: (-hx, +hy)
/// with world corner (cx + kx*cos_h - ky*sin_h, cy + kx*sin_h + ky*cos_h) for
/// signed local half-extents (kx, ky).
struct Obb2 {
    double cx = 0.0;    ///< World center x, m.
    double cy = 0.0;    ///< World center y, m.
    double cos_h = 1.0; ///< cosine of heading from det_sincos; body x axis is (cos_h, sin_h).
    double sin_h = 0.0; ///< sine of heading from det_sincos; body y axis is (-sin_h, cos_h).
    double hx = 0.0;    ///< Half extent along body x (length/2), m.
    double hy = 0.0;    ///< Half extent along body y (width/2), m.
};

/// True when the two boxes overlap or touch, by the separating-axis test over
/// the four box axes (a.x, a.y, b.x, b.y) in that fixed order. Touching boxes
/// count as intersecting (strict `>` separation test): two entities whose
/// bounding boxes touch have freespace distance zero, per ASAM OpenSCENARIO
/// XML 1.4.0 §6.4.7.2.
[[nodiscard]] bool obb_intersects(const Obb2& a, const Obb2& b);

/// Smallest Euclidean distance between the two (filled) boxes, in meters, or
/// 0.0 when they intersect or touch (§6.4.7.2). For disjoint convex rectangles
/// the closest pair always includes a vertex, so this is the minimum over each
/// box's four world corners against the other box; squared distances are
/// reduced with std::fmin and a single std::sqrt is taken at the end (no
/// std::hypot, no reassociation).
[[nodiscard]] double obb_distance(const Obb2& a, const Obb2& b);

/// Smallest Euclidean distance from a world point to the (filled) box `b`, in
/// meters; 0.0 when the point is on or inside the box. This is the freespace
/// distance between a reference point and an entity (§6.4.7.2).
[[nodiscard]] double point_obb_distance(double px, double py, const Obb2& b);

/// Projects box `a` onto the unit axis (ux, uy), writing the signed interval
/// [lo, hi] the box covers. Used for the longitudinal/lateral freespace gap:
/// the axis-interval separation between two boxes on a shared body axis. The
/// axis must be a unit vector for the interval to be metric.
void obb_project(const Obb2& a, double ux, double uy, double& lo, double& hi);

} // namespace scena::runtime
