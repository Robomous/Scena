// SPDX-License-Identifier: MIT
#pragma once

namespace scena {

/// Kinematic state of one entity in the world frame.
struct EntityState {
    double x = 0.0;       ///< World position, meters.
    double y = 0.0;       ///< World position, meters.
    double z = 0.0;       ///< World position, meters.
    double heading = 0.0; ///< Yaw around +Z, radians; 0 points along +X.
    double speed = 0.0;   ///< Longitudinal speed along the heading, m/s.
};

} // namespace scena
