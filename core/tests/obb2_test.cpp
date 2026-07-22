// SPDX-License-Identifier: MIT
// The deterministic 2D OBB freespace kernel (scena/runtime/obb2.h, p5-s3).
// Axis-aligned cases assert exact distances; a heading-rotated case hex-pins
// the raw bit pattern, since every op is IEEE-exact and the only trig routes
// through det_sincos — the distance must be identical on every platform.
#include <gtest/gtest.h>

#include "scena/runtime/detmath.h"
#include "scena/runtime/obb2.h"
#include "support/trace_recorder.h"

using scena::runtime::Obb2;
using scena::runtime::obb_distance;
using scena::runtime::obb_intersects;
using scena::runtime::obb_project;
using scena::runtime::point_obb_distance;
using scena::runtime::SinCos;
using scena::testsupport::hex_bits;

namespace {

/// A heading-0 axis-aligned box: center (cx, cy), half extents (hx, hy).
Obb2 aligned(double cx, double cy, double hx, double hy) {
    return Obb2{cx, cy, /*cos_h=*/1.0, /*sin_h=*/0.0, hx, hy};
}

} // namespace

TEST(Obb2Test, OverlapTouchAndDisjointClassification) {
    const Obb2 unit = aligned(0.0, 0.0, 1.0, 1.0);
    // Overlapping (centers 1.5 apart, combined half-widths 2).
    EXPECT_TRUE(obb_intersects(unit, aligned(1.5, 0.0, 1.0, 1.0)));
    EXPECT_EQ(obb_distance(unit, aligned(1.5, 0.0, 1.0, 1.0)), 0.0);
    // Exactly touching (centers 2 apart): still intersecting, distance 0.
    EXPECT_TRUE(obb_intersects(unit, aligned(2.0, 0.0, 1.0, 1.0)));
    EXPECT_EQ(obb_distance(unit, aligned(2.0, 0.0, 1.0, 1.0)), 0.0);
    // Disjoint (centers 3 apart): a gap of exactly 1 between the facing edges.
    EXPECT_FALSE(obb_intersects(unit, aligned(3.0, 0.0, 1.0, 1.0)));
    EXPECT_EQ(obb_distance(unit, aligned(3.0, 0.0, 1.0, 1.0)), 1.0);
}

TEST(Obb2Test, DisjointDistanceIsExactForAlignedBoxes) {
    // A's corner (1, 1) to B's corner (4, 5): a 3-4-5 right triangle, distance
    // exactly 5. B centered at (5, 6) with unit half extents has its nearest
    // corner at (4, 5).
    const Obb2 a = aligned(0.0, 0.0, 1.0, 1.0);
    const Obb2 b = aligned(5.0, 6.0, 1.0, 1.0);
    EXPECT_FALSE(obb_intersects(a, b));
    EXPECT_EQ(obb_distance(a, b), 5.0);
}

TEST(Obb2Test, PointDistanceInsideIsZeroOutsideIsExact) {
    const Obb2 unit = aligned(0.0, 0.0, 1.0, 1.0);
    EXPECT_EQ(point_obb_distance(0.0, 0.0, unit), 0.0); // center: inside
    EXPECT_EQ(point_obb_distance(1.0, 0.5, unit), 0.0); // on the boundary
    EXPECT_EQ(point_obb_distance(4.0, 0.0, unit), 3.0); // beyond the +x edge
    EXPECT_EQ(point_obb_distance(4.0, 5.0, unit), 5.0); // beyond the +x,+y corner
}

TEST(Obb2Test, ProjectionIntervalIsExactOnBodyAxes) {
    const Obb2 box = aligned(3.0, 0.0, 2.0, 1.0);
    double lo = 0.0;
    double hi = 0.0;
    obb_project(box, 1.0, 0.0, lo, hi); // world x
    EXPECT_EQ(lo, 1.0);
    EXPECT_EQ(hi, 5.0);
    obb_project(box, 0.0, 1.0, lo, hi); // world y
    EXPECT_EQ(lo, -1.0);
    EXPECT_EQ(hi, 1.0);
}

TEST(Obb2Test, RotatedBoxFreespaceDistanceIsHexPinned) {
    // A unit box at the origin and a rotated box: the freespace distance is a
    // stable IEEE-754 bit pattern because the only trig is det_sincos and
    // every downstream op is exact. A drift here is a determinism regression.
    const Obb2 a = aligned(0.0, 0.0, 1.0, 1.0);
    const SinCos sc = scena::runtime::det_sincos(0.7);
    const Obb2 b{/*cx=*/4.0, /*cy=*/1.0, sc.cos, sc.sin, /*hx=*/1.5, /*hy=*/0.5};
    ASSERT_FALSE(obb_intersects(a, b));
    EXPECT_EQ(hex_bits(obb_distance(a, b)), "3ff87d73a7b6c364");
}

TEST(Obb2Test, TouchThroughRotationIsIntersecting) {
    // A rotated box whose corner just reaches the origin box: contact ⇒
    // intersecting ⇒ distance 0 (§6.4.7.2 strict-touch rule).
    const Obb2 a = aligned(0.0, 0.0, 1.0, 1.0);
    const SinCos sc = scena::runtime::det_sincos(0.3);
    // Placed so a rotated corner meets a's +x edge region.
    const Obb2 b{/*cx=*/1.5, /*cy=*/0.0, sc.cos, sc.sin, /*hx=*/0.6, /*hy=*/0.6};
    EXPECT_TRUE(obb_intersects(a, b));
    EXPECT_EQ(obb_distance(a, b), 0.0);
}
