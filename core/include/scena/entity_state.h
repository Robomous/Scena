// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#pragma once

namespace scena {

/// Kinematic state of one entity in the world frame.
///
/// The orientation is a full Z-up, right-handed pose in radians: `heading`
/// (yaw about +Z), `pitch` (about the entity's +Y), `roll` (about the entity's
/// +X), matching the OpenSCENARIO Orientation convention (ASAM OpenSCENARIO
/// XML 1.4.0 §Orientation, h/p/r). `speed` is the scalar longitudinal speed
/// along the heading. The straight-line runtime integrates position from
/// speed and heading only, leaving pitch and roll at 0; a host may report any
/// pose for a host-controlled entity.
///
/// Field order: `pitch`/`roll` are appended after `speed` so existing
/// positional `EntityState{x, y, z, heading, speed}` initializers keep
/// compiling (the new members default to 0), matching the append-only
/// discipline the mirrored C-ABI struct requires.
struct EntityState {
    double x = 0.0;       ///< World position, meters.
    double y = 0.0;       ///< World position, meters.
    double z = 0.0;       ///< World position, meters.
    double heading = 0.0; ///< Yaw around +Z, radians; 0 points along +X.
    double speed = 0.0;   ///< Longitudinal speed along the heading, m/s.
    double pitch = 0.0;   ///< Pitch around the entity's +Y, radians.
    double roll = 0.0;    ///< Roll around the entity's +X, radians.
};

} // namespace scena
