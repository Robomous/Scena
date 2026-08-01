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

#include <map>
#include <string>
#include <vector>

#include "scena/gateway/road_query.h"
#include "scena/opendrive/map.h"
#include "scena/opendrive/reference_line.h"

namespace scena::opendrive {

/// The OpenDRIVE-backed implementation of the frozen `gateway::IRoadQuery`
/// v1 (p3-s3). Hosts construct it from a parsed `Map` and hand it to the
/// engine through the gateway; the core never links this library.
///
/// Semantics on top of the v1 contract:
///  - Flat world: elevation is scoped out for v0.0.1, so every road lies in
///    the z = 0 plane. `to_lane_position` treats a point with |z| beyond a
///    hair's tolerance as off-road, and `to_world_position` returns z = 0 —
///    this keeps the conversions exact mutual inverses.
///  - On-road means the lateral offset t lies within the lane span of the
///    cross-section at s (cumulative width records, §11.7.1). The lane id is
///    the lane whose t-interval contains the point; exactly t = 0 is the
///    centre lane 0.
///  - Ties between equally near roads keep the smallest road id (the
///    ordered-map iteration order); within one reference line the smallest s
///    wins (p3-s2). Documented deterministic tie-breaks, never an address or
///    hash order.
///  - Routing (`build_route`) is a Dijkstra shortest path over the
///    (road, lane) graph formed by lane linkage across road links and
///    junction connections, weighted by driven length. Driving direction
///    follows the lane-id sign convention: negative lanes run along +s,
///    positive lanes against it (§11.3.1, right-hand traffic). The frontier
///    is an ordered set keyed (distance, road id, lane id), so equal-length
///    paths deterministically prefer the lexicographically smallest
///    (road id, lane id) — the documented tie-break.
class OpenDriveRoadQuery final : public gateway::IRoadQuery {
public:
    /// Builds the query over a loaded map: constructs each road's reference
    /// line and the routing adjacency. Roads whose plan view is unusable are
    /// skipped (their queries answer false).
    explicit OpenDriveRoadQuery(Map map);

    // --- conversions -------------------------------------------------------
    [[nodiscard]] bool to_lane_position(double x, double y, double z,
                                        gateway::LanePosition& out) const override;
    [[nodiscard]] bool to_world_position(const gateway::LanePosition& position, double& x,
                                         double& y, double& z) const override;
    [[nodiscard]] bool road_heading(const std::string& road_id, double s,
                                    double& out_heading) const override;

    // --- lane queries ------------------------------------------------------
    [[nodiscard]] bool lane_exists(const std::string& road_id, int lane_id,
                                   double s) const override;
    [[nodiscard]] bool lane_width(const std::string& road_id, int lane_id, double s,
                                  double& out_width) const override;
    [[nodiscard]] bool lane_center_offset(const std::string& road_id, int lane_id, double s,
                                          double& out_t) const override;
    [[nodiscard]] bool lane_type(const std::string& road_id, int lane_id, double s,
                                 std::string& out_type) const override;
    [[nodiscard]] bool relative_lane(const std::string& road_id, int lane_id, int count,
                                     int& out_lane_id) const override;

    // --- s-range queries ---------------------------------------------------
    [[nodiscard]] bool road_length(const std::string& road_id, double& out_length) const override;
    [[nodiscard]] bool lane_s_range(const std::string& road_id, int lane_id, double s,
                                    double& out_s_begin, double& out_s_end) const override;

    // --- routes ------------------------------------------------------------
    [[nodiscard]] bool build_route(const std::vector<gateway::LanePosition>& waypoints,
                                   std::vector<gateway::RouteSpan>& out_spans) const override;
    [[nodiscard]] bool position_along_route(const std::vector<gateway::RouteSpan>& route,
                                            const gateway::LanePosition& position,
                                            double& out_distance) const override;

private:
    struct RoadEntry {
        Road road;
        ReferenceLine line;
        explicit RoadEntry(Road r) : road(std::move(r)), line(road) {}
    };

    /// A node of the routing graph: one lane of one road, traversed in its
    /// driving direction.
    struct RouteNode {
        std::string road_id;
        int lane_id = 0;
        [[nodiscard]] bool operator<(const RouteNode& other) const {
            return road_id != other.road_id ? road_id < other.road_id : lane_id < other.lane_id;
        }
        [[nodiscard]] bool operator==(const RouteNode& other) const {
            return road_id == other.road_id && lane_id == other.lane_id;
        }
    };

    [[nodiscard]] const RoadEntry* entry_of(const std::string& road_id) const;
    [[nodiscard]] const LaneSection* section_at(const Road& road, double s) const;
    [[nodiscard]] const Lane* lane_at(const Road& road, int lane_id, double s) const;
    /// Successor nodes of `node` in driving direction, in deterministic
    /// (road id, lane id) order.
    [[nodiscard]] std::vector<RouteNode> neighbors_of(const RouteNode& node) const;
    /// Route one waypoint pair; appends spans. Returns false when no path
    /// exists.
    [[nodiscard]] bool route_pair(const gateway::LanePosition& from,
                                  const gateway::LanePosition& to,
                                  std::vector<gateway::RouteSpan>& out_spans) const;

    std::map<std::string, RoadEntry> roads_;
    std::map<std::string, Junction> junctions_;
};

} // namespace scena::opendrive
