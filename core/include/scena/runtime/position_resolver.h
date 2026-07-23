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

#include <functional>
#include <optional>
#include <string>
#include <string_view>

#include "scena/entity_state.h"
#include "scena/ir/position.h"
#include "scena/status.h"

namespace scena::runtime {

/// A resolved world pose: inertial-frame position (meters) and orientation
/// (radians). Distinct from EntityState because a position carries no speed —
/// resolving a Position answers "where and facing which way", nothing about
/// motion.
struct Pose {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double heading = 0.0; ///< Yaw about +Z, radians (§Orientation h).
    double pitch = 0.0;   ///< Pitch about +Y, radians (§Orientation p).
    double roll = 0.0;    ///< Roll about +X, radians (§Orientation r).
};

/// Outcome of resolving one Position. `status == Status::Ok` means `out` was
/// filled; otherwise `message` explains why (deterministic — entity ids and
/// element names only, never a floating-point value) and `rule_id` carries the
/// ASAM checker rule UID when the standard defines one (GeoPosition).
struct PositionResolution {
    Status status = Status::Ok;
    std::string message;
    std::string rule_id;
};

/// Maps any of the ten ASAM OpenSCENARIO §6.3.8 Position variants to a world
/// pose, per §6.3 / §Orientation, with the corrected (≥1.3) calculations
/// applied uniformly to all input versions (§5).
///
/// The resolver is deliberately standalone — it depends only on a lookup of
/// reference-entity poses, not on the engine — so every variant and every
/// orientation-composition case can be unit-tested without booting a scenario.
///
/// Coverage this sprint (p2-s4): the self-contained variants resolve fully —
/// WorldPosition, RelativeWorldPosition (world-axis deltas), and
/// RelativeObjectPosition (deltas rotated into the reference entity's frame,
/// via deterministic `det_sincos`). The road-family variants (Road,
/// RelativeRoad, Lane, RelativeLane, Route) require a road network and its
/// s-axis tangents for orientation; they report Status::UnsupportedFeature
/// until the road backend lands (p3-s4). TrajectoryPosition reports unsupported
/// until trajectory shapes land (p2-s5). GeoPosition reports unsupported with
/// the rule `asam.net:xosc:1.1.0:positioning.geodetic_datum_defined`. Every
/// variant either resolves or reports — none is silently wrong.
class PositionResolver {
public:
    /// Returns the current pose of the reference entity `id`, or nullptr when
    /// there is no such active entity.
    using PoseLookup = std::function<const EntityState*(std::string_view id)>;

    explicit PositionResolver(PoseLookup lookup) noexcept;

    /// Resolves `position` into `out`. See PositionResolution for the contract.
    [[nodiscard]] PositionResolution resolve(const ir::Position& position, Pose& out) const;

private:
    PoseLookup lookup_;
};

} // namespace scena::runtime
