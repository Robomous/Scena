// SPDX-License-Identifier: MIT
#pragma once

namespace scena::ir {

/// A world-frame Cartesian position, meters (§6.3.2). This is the minimal
/// Position variant the kernel needs today; the ten §6.3.8 Position variants
/// (lane, road, relative, …) and the PositionResolver arrive with p2-s4/p3-s4.
/// ReachPositionCondition compares against it directly in world coordinates
/// until then.
struct WorldPosition {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

} // namespace scena::ir
