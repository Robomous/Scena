// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>
#include <vector>

#include "scena/ir/position.h"

namespace scena::ir {

/// Strategy for path selection between waypoints in a route, per ASAM
/// OpenSCENARIO XML 1.4.0 §RouteStrategy (§6.8.2).
///
/// Scena stores the strategy but does not interpret it: choosing a path
/// between waypoints needs a road network (IRoadQuery, p3-s4). `Random` is
/// stored like any other value — it never reaches a random number generator,
/// which the determinism contract forbids in the runtime.
enum class RouteStrategy {
    Fastest,            ///< Shortest traveling time between start and target.
    LeastIntersections, ///< As few junctions as possible.
    Random,             ///< Up to the simulator how to reach the target.
    Shortest,           ///< Shortest path between start and target.
};

/// A reference position used to form a route, per §Waypoint: a position plus
/// the strategy for reaching it (§6.8.2, "the RouteStrategy specified in the
/// target Waypoint shall be used").
///
/// Scena models the world-frame position only (§WorldPosition); the road- and
/// lane-relative Position variants arrive with p2-s4/p3-s4, together with the
/// `routing.route_waypoints_locations` prerequisite check that waypoints are
/// valid positions on roads (§7.5.2.2).
struct Waypoint {
    WorldPosition position;
    RouteStrategy strategy = RouteStrategy::Shortest;
};

/// A continuous path through the road network defined by an ordered series of
/// waypoints, per §Route (§6.8.2).
///
/// A route requires at least two waypoints. Once assigned it stays in place
/// "until another action overwrites them" (§6.8.2), which is what makes the
/// engine hold it as per-entity state rather than as action-local data.
struct Route {
    std::string name;
    /// In a closed route the last waypoint is followed by the first (§Route).
    bool closed = false;
    std::vector<Waypoint> waypoints; ///< At least two, in document order.
};

} // namespace scena::ir
