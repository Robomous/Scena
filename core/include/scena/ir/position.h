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

#include <memory>
#include <optional>
#include <string>
#include <variant>

namespace scena::ir {

// A TrajectoryPosition references a trajectory whose shape lives in
// trajectory.h; that header includes this one for WorldPosition, so the
// reference is held by (forward-declared) shared_ptr to break the cycle.
struct Trajectory;

/// Whether a value is stated in absolute (world-frame) or relative terms, per
/// ASAM OpenSCENARIO XML 1.4.0 §ReferenceContext. On an Orientation it selects
/// whether the angles are world-frame or a shift on the reference frame; on a
/// trajectory Timing it selects the origin of the vertex time values. This is
/// the one canonical definition (trajectory.h and others include this header).
enum class ReferenceContext {
    Absolute, ///< World coordinate system / simulation-time origin.
    Relative, ///< Relative to a reference frame / the action's start instant.
};

/// A heading/pitch/roll orientation, radians (§Orientation). Rotations apply in
/// the order Z (heading), then Y (pitch), then X (roll); positive is
/// counter-clockwise (§6.3). In the `Relative` context the angles are an
/// angular shift on top of the reference orientation (`h=p=r=0` copies it); in
/// the `Absolute` context they are the world-frame orientation and the
/// reference orientation is ignored. When referencing a road/lane/trajectory
/// s-axis only heading is meaningful — pitch/roll come from the road surface and
/// are undefined by the standard.
struct Orientation {
    double h = 0.0;
    double p = 0.0;
    double r = 0.0;
    ReferenceContext type = ReferenceContext::Relative;
};

/// A world-frame position with orientation, meters and radians (§WorldPosition,
/// §6.3.1). `x/y/z` is the inertial-frame position; `h/p/r` is the world
/// orientation (WorldPosition carries no separate `Orientation` element — its
/// angles are inherently absolute-world).
///
/// Field order: `h/p/r` are appended after `x/y/z` so existing `WorldPosition{x,
/// y, z}` initializers keep compiling (the angles default to 0), matching the
/// append-only discipline the mirrored C-ABI struct requires.
struct WorldPosition {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double h = 0.0; ///< Heading (yaw about +Z), radians; 0 points along world +X.
    double p = 0.0; ///< Pitch about +Y, radians.
    double r = 0.0; ///< Roll about +X, radians.
};

/// Position given as world-axis deltas from a reference entity
/// (§RelativeWorldPosition). `dx/dy/dz` are along the WORLD axes — they are NOT
/// rotated by the reference entity's heading (contrast RelativeObjectPosition).
struct RelativeWorldPosition {
    std::string entity_ref;
    double dx = 0.0;
    double dy = 0.0;
    double dz = 0.0;
    std::optional<Orientation> orientation;
};

/// Position given as deltas in a reference entity's LOCAL frame
/// (§RelativeObjectPosition). `dx/dy/dz` are expressed in the reference
/// entity's coordinate system, so they ARE rotated by the entity's orientation.
struct RelativeObjectPosition {
    std::string entity_ref;
    double dx = 0.0;
    double dy = 0.0;
    double dz = 0.0;
    std::optional<Orientation> orientation;
};

/// Absolute road-coordinate position (§RoadPosition): distance `s` along the
/// reference line of road `road_id`, signed lateral offset `t` (t-axis points
/// left, §6.3.2). Resolving it needs a road network (`IRoadQuery`); until a
/// backend exists (p3-s4) the resolver reports it unsupported.
struct RoadPosition {
    std::string road_id;
    double s = 0.0;
    double t = 0.0;
    std::optional<Orientation> orientation;
};

/// Road-coordinate position relative to a reference entity's road position
/// (§RelativeRoadPosition): `ds` along the reference line, `dt` laterally,
/// following the entity's routing onto connecting roads. Needs a road network
/// (deferred to p3-s4).
struct RelativeRoadPosition {
    std::string entity_ref;
    double ds = 0.0;
    double dt = 0.0;
    std::optional<Orientation> orientation;
};

/// Absolute lane-coordinate position (§LanePosition): lane `lane_id` of road
/// `road_id`, distance `s` along the ROAD reference line, `offset` along the
/// lane centre-line t-axis. Needs a road network (p3-s4).
struct LanePosition {
    std::string road_id;
    std::string lane_id;
    double s = 0.0;
    double offset = 0.0;
    std::optional<Orientation> orientation;
};

/// Lane-coordinate position relative to a reference entity's lane
/// (§RelativeLanePosition): `d_lane` lanes over, and either `ds` (along the road
/// reference line) or `ds_lane` (along the lane centre line) — mutually
/// exclusive — plus a lateral `offset`. Needs a road network (p3-s4).
struct RelativeLanePosition {
    std::string entity_ref;
    int d_lane = 0;
    std::optional<double> ds;
    std::optional<double> ds_lane;
    double offset = 0.0;
    std::optional<Orientation> orientation;
};

/// Position along a route (§RoutePosition). The route reference and the
/// in-route coordinate arrive with the road/route backend (p3-s4); the variant
/// exists so the resolver can dispatch it to an unsupported-feature diagnostic
/// rather than silently mishandle it.
struct RoutePosition {
    std::optional<Orientation> orientation;
};

/// Position on the geographic sphere (§GeoPosition): `latitude_deg`,
/// `longitude_deg`, and `altitude` above the road surface. Resolving it needs
/// the referenced road network's geodetic datum / map projection; without one
/// the resolver reports the ASAM rule
/// `asam.net:xosc:1.1.0:positioning.geodetic_datum_defined`. Post-v0.0.1.
struct GeoPosition {
    double latitude_deg = 0.0;
    double longitude_deg = 0.0;
    double altitude = 0.0;
    std::optional<Orientation> orientation;
};

/// Position relative to a trajectory (§TrajectoryPosition): arclength `s` along
/// the referenced trajectory and lateral offset `t` across it (positive to the
/// left of the direction of travel). The resolver evaluates the trajectory
/// geometry at `s` and steps `t` along the left-normal of the tangent.
struct TrajectoryPosition {
    double s = 0.0;
    double t = 0.0;
    std::optional<Orientation> orientation;
    /// The referenced trajectory; null is reported as a content defect.
    std::shared_ptr<Trajectory> trajectory;
};

/// The ten ASAM OpenSCENARIO §6.3.8 position variants, in the spec's `Position`
/// `xsd:choice` order. `WorldPosition` is the self-contained absolute variant a
/// `WorldPosition` value converts to implicitly. The PositionResolver
/// (`runtime/position_resolver.h`) maps any of these to a world pose or a
/// rule-cited diagnostic; per §5 the corrected (≥1.3) position/orientation
/// calculations are applied uniformly to all input versions.
using Position = std::variant<WorldPosition, RelativeWorldPosition, RelativeObjectPosition,
                              RoadPosition, RelativeRoadPosition, LanePosition,
                              RelativeLanePosition, RoutePosition, GeoPosition, TrajectoryPosition>;

} // namespace scena::ir
