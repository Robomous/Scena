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

namespace scena::gateway {
class IRoadQuery;
} // namespace scena::gateway

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
/// Coverage: the self-contained variants resolve fully — WorldPosition,
/// RelativeWorldPosition (world-axis deltas), RelativeObjectPosition (deltas
/// rotated into the reference entity's frame via deterministic
/// `det_sincos`), and TrajectoryPosition (through the trajectory
/// evaluator). The road-family variants (Road, RelativeRoad, Lane,
/// RelativeLane, Route) resolve against the road network handed to the
/// constructor (p3-s4): positions map to the world through the frozen
/// `IRoadQuery` v1 conversions, and the orientation base of every
/// road-family variant is the road s-axis tangent at the target
/// (`road_heading`; for a route traversed against the s-axis the base is
/// turned by pi), with pitch/roll 0 — the standard leaves them to the road
/// surface, which is the z = 0 plane in the v0.0.1 subset. Without a road
/// network they keep reporting Status::UnsupportedFeature. Subset limits
/// (each reported, never silent): RelativeRoad/RelativeLane deltas stay on
/// the reference entity's road (no continuation onto linked roads), and
/// `ds_lane` (along the lane centre line) is not supported — `ds` along the
/// road reference line is. GeoPosition reports unsupported with the rule
/// `asam.net:xosc:1.1.0:positioning.geodetic_datum_defined`. Every variant
/// either resolves or reports — none is silently wrong.
class PositionResolver {
public:
    /// Returns the current pose of the reference entity `id`, or nullptr when
    /// there is no such active entity.
    using PoseLookup = std::function<const EntityState*(std::string_view id)>;

    /// `road` may be null: road-family variants then report unsupported. The
    /// pointer is borrowed and must outlive the resolver.
    explicit PositionResolver(PoseLookup lookup,
                              const gateway::IRoadQuery* road = nullptr) noexcept;

    /// Resolves `position` into `out`. See PositionResolution for the contract.
    [[nodiscard]] PositionResolution resolve(const ir::Position& position, Pose& out) const;

private:
    PoseLookup lookup_;
    const gateway::IRoadQuery* road_ = nullptr;
};

} // namespace scena::runtime
