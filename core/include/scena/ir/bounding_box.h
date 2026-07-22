// SPDX-License-Identifier: MIT
#pragma once

namespace scena::ir {

/// A simplified three-dimensional bounding box, per ASAM OpenSCENARIO XML
/// 1.4.0 BoundingBox: the geometric center expressed in the entity's own
/// coordinate frame plus the width/length/height dimensions.
///
/// This is a minimal forward-pull from the full entity taxonomy (p2-s1,
/// #15): the interaction conditions (Distance/TimeToCollision/Collision, …)
/// need geometry for freespace math before that taxonomy lands, so only the
/// fields freespace calculations consume live here. p2-s1 later adds the
/// categories, Performance and axles on top of this same box (ADR-0009).
///
/// Frame: the entity body frame (§6.4.4) — +x forward along the heading, +y
/// left, +z up — with the origin at the entity's reference point. The 2D
/// ground-plane freespace kernel (runtime/obb2.h) uses `length`, `width`, and
/// the x/y center offset; `center_z` and `height` are stored for p2-s1 but do
/// not enter the planar math (EntityState carries yaw only, so the box
/// projects exactly to a heading-rotated rectangle).
///
/// Flat doubles with default member initializers, matching WorldPosition:
/// zero dimensions are a valid degenerate point box, so there is no invariant
/// a constructor must enforce.
struct BoundingBox {
    double center_x = 0.0; ///< m, entity frame, +x forward.
    double center_y = 0.0; ///< m, entity frame, +y left.
    double center_z = 0.0; ///< m, entity frame, +z up (stored for p2-s1; unused in 2D math).
    double length = 0.0;   ///< m, extent along the body x axis.
    double width = 0.0;    ///< m, extent along the body y axis.
    double height = 0.0;   ///< m, extent along the body z axis (unused this phase).
};

} // namespace scena::ir
