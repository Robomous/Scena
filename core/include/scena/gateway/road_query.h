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

#include <string>
#include <vector>

namespace scena::gateway {

/// Lane-relative position in road coordinates, following the ASAM OpenDRIVE
/// s/t convention (ASAM OpenDRIVE 1.9.0 §8.3): `s` runs along the road
/// reference line, `t` is the signed lateral offset from it — positive to the
/// left — and `lane_id` selects the lane relative to the reference line
/// (centre lane 0, descending to the right, ascending to the left, per
/// OpenDRIVE 1.9.0 §11.1).
struct LanePosition {
    std::string road_id;
    int lane_id = 0;
    double s = 0.0; ///< Longitudinal position along the road reference line, meters.
    double t = 0.0; ///< Signed lateral offset from the reference line, meters.
};

/// A contiguous stretch of a single lane traversed by a route.
///
/// A route through the road network is an ordered sequence of spans. Each span
/// covers the s-interval between `s_begin` and `s_end` of lane `lane_id` on
/// road `road_id`, traversed from `s_begin` towards `s_end`: `s_end < s_begin`
/// means the route runs against the road's s-axis. `lane_id` is never 0 — the
/// centre lane has no width and cannot be driven (OpenDRIVE 1.9.0 §11.1).
struct RouteSpan {
    std::string road_id;
    int lane_id = 0;
    double s_begin = 0.0; ///< s where the route enters the span, meters.
    double s_end = 0.0;   ///< s where the route leaves the span, meters.
};

/// Abstract road-network query interface — **frozen v1** (p3-s1).
///
/// The runtime needs lane-relative positioning to evaluate road-based
/// conditions and actions, but it must not depend on any concrete road-network
/// representation. The host simulator (or the separate OpenDRIVE backend
/// library, injected by the host) provides an implementation through the
/// simulator gateway.
///
/// This is the v1 surface every downstream sprint codes against; the runtime
/// reaches road data only through this header. Extending or changing it is an
/// ADR-level decision (ADR-0003, amendment p3-s1).
///
/// ## Unsupported-reporting semantics
///
/// Every query answers through its `bool` return: `true` fills the out
/// parameters with a definitive answer, `false` means *no answer* — because
/// the entity is off-road, the id names nothing, the input is non-finite or
/// otherwise invalid, or the backend does not support the query at all.
/// "Unsupported" and "no such element" are deliberately the same answer:
/// the caller does the same thing either way (fall back to the flat-world
/// model or report the operation unsupported, ADR-0016/ADR-0017). Backends
/// must never throw across this boundary and must answer deterministically:
/// identical query arguments against identical road data give bit-identical
/// results, on every platform.
///
/// All new-in-v1 queries are defaulted to `false` rather than pure so that
/// implementations written against earlier snapshots keep compiling; the two
/// coordinate conversions predate the freeze and stay pure.
class IRoadQuery {
public:
    virtual ~IRoadQuery() = default;

    // --- Lane-relative <-> world conversions -------------------------------

    /// Maps a world-frame position to lane-relative road coordinates.
    /// Returns false when the position is not on any known road.
    [[nodiscard]] virtual bool to_lane_position(double x, double y, double z,
                                                LanePosition& out) const = 0;

    /// Maps lane-relative road coordinates back to a world-frame position.
    /// Returns false when the coordinates do not identify a valid location.
    [[nodiscard]] virtual bool to_world_position(const LanePosition& position, double& x, double& y,
                                                 double& z) const = 0;

    /// Inertial heading of the road's s-axis at `s`, in radians (0 = world
    /// +x, counter-clockwise positive, per OpenDRIVE 1.9.0 §8.2/§8.3). This
    /// is what orients lane-relative poses in the world frame. Returns false
    /// when the road is unknown or `s` lies outside it.
    [[nodiscard]] virtual bool road_heading(const std::string& road_id, double s,
                                            double& out_heading) const {
        (void)road_id;
        (void)s;
        (void)out_heading;
        return false;
    }

    // --- Lane queries ------------------------------------------------------
    //
    // lane_width / lane_center_offset / relative_lane landed ahead of the
    // freeze as the p2-s3 forward-pull (ADR-0016); their signatures are
    // unchanged here and are now part of the frozen surface.

    /// True iff lane `lane_id` of road `road_id` exists at `s`. This is the
    /// existence probe behind the other lane queries: when it answers false,
    /// every other query about the same (road, lane, s) also answers false.
    [[nodiscard]] virtual bool lane_exists(const std::string& road_id, int lane_id,
                                           double s) const {
        (void)road_id;
        (void)lane_id;
        (void)s;
        return false;
    }

    /// Width of lane `lane_id` of road `road_id` at `s`, in metres. Returns
    /// false when the lane is unknown or widths are not available.
    [[nodiscard]] virtual bool lane_width(const std::string& road_id, int lane_id, double s,
                                          double& out_width) const {
        (void)road_id;
        (void)lane_id;
        (void)s;
        (void)out_width;
        return false;
    }

    /// Signed t-coordinate of the centre line of lane `lane_id` of road
    /// `road_id` at `s`, on the road t-axis, which points left (§6.3.2).
    /// Returns false when the lane is unknown.
    [[nodiscard]] virtual bool lane_center_offset(const std::string& road_id, int lane_id, double s,
                                                  double& out_t) const {
        (void)road_id;
        (void)lane_id;
        (void)s;
        (void)out_t;
        return false;
    }

    /// Type of lane `lane_id` of road `road_id` at `s`, as the road network's
    /// type string handed over verbatim (for OpenDRIVE, the `<lane>` @type
    /// value, e.g. "driving", per OpenDRIVE 1.9.0 §11.4). The engine stores
    /// and compares these strings; it interprets none of them. Returns false
    /// when the lane is unknown or has no type.
    [[nodiscard]] virtual bool lane_type(const std::string& road_id, int lane_id, double s,
                                         std::string& out_type) const {
        (void)road_id;
        (void)lane_id;
        (void)s;
        (void)out_type;
        return false;
    }

    /// The lane `count` lanes away from lane `lane_id` of road `road_id`,
    /// counting positive towards +t (left) and skipping the road centre lane,
    /// which "is not counted as a lane and thus omitted" (§7.4.1.4). Returns
    /// false when there is no such lane.
    [[nodiscard]] virtual bool relative_lane(const std::string& road_id, int lane_id, int count,
                                             int& out_lane_id) const {
        (void)road_id;
        (void)lane_id;
        (void)count;
        (void)out_lane_id;
        return false;
    }

    // --- s-range queries ---------------------------------------------------

    /// Total length of road `road_id`'s reference line, in metres; valid s
    /// values lie in [0, length] (OpenDRIVE 1.9.0 §8.3: s is measured from
    /// the beginning of the reference line). Returns false when the road is
    /// unknown.
    [[nodiscard]] virtual bool road_length(const std::string& road_id, double& out_length) const {
        (void)road_id;
        (void)out_length;
        return false;
    }

    /// The maximal contiguous s-interval containing `s` over which lane
    /// `lane_id` of road `road_id` exists, in metres. Returns false when the
    /// lane does not exist at `s`.
    [[nodiscard]] virtual bool lane_s_range(const std::string& road_id, int lane_id, double s,
                                            double& out_s_begin, double& out_s_end) const {
        (void)road_id;
        (void)lane_id;
        (void)s;
        (void)out_s_begin;
        (void)out_s_end;
        return false;
    }

    // --- Routes ------------------------------------------------------------

    /// Computes the ordered road/lane spans connecting `waypoints` in order.
    /// Waypoints are lane positions already resolved against this road
    /// network; a closed route is expressed by repeating the first waypoint
    /// at the end. Path selection between consecutive waypoints is the
    /// backend's deterministic routing (documented tie-break, p3-s3). Returns
    /// false when fewer than two waypoints are given, a waypoint is off the
    /// network, or no connected path exists.
    [[nodiscard]] virtual bool build_route(const std::vector<LanePosition>& waypoints,
                                           std::vector<RouteSpan>& out_spans) const {
        (void)waypoints;
        (void)out_spans;
        return false;
    }

    /// Distance from the start of `route` to the projection of `position`
    /// onto it, measured along the route in metres. Returns false when
    /// `position` does not lie on any span of the route.
    [[nodiscard]] virtual bool position_along_route(const std::vector<RouteSpan>& route,
                                                    const LanePosition& position,
                                                    double& out_distance) const {
        (void)route;
        (void)position;
        (void)out_distance;
        return false;
    }
};

} // namespace scena::gateway
