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
