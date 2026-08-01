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

#include "scena/opendrive/road_query.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <tuple>
#include <utility>

#include "scena/runtime/detmath.h"

namespace scena::opendrive {

namespace {

/// Elevation is scoped out for v0.0.1: roads live in the z = 0 plane, and a
/// point off that plane is off-road. Keeping this at numeric-noise scale is
/// what makes to_lane_position / to_world_position exact mutual inverses.
constexpr double kZTolerance = 1e-6;

/// Lateral/longitudinal acceptance slack for boundary points, meters.
constexpr double kEdgeTolerance = 1e-9;

/// Width of `lane` at `ds` metres past its section start (§11.7.1): the
/// active record is the last one with s_offset <= ds; the polynomial is
/// evaluated in Horner form and clamped at zero
/// (asam.net:xodr:1.4.0:road.lane.width.lane_width_validity).
double width_at(const Lane& lane, double ds) {
    const WidthRecord* active = nullptr;
    for (const WidthRecord& record : lane.widths) {
        if (record.s_offset <= ds + kEdgeTolerance) {
            active = &record;
        } else {
            break;
        }
    }
    if (active == nullptr) {
        return 0.0;
    }
    const double x = ds - active->s_offset;
    const double w = active->a + x * (active->b + x * (active->c + x * active->d));
    return w > 0.0 ? w : 0.0;
}

/// Signed t of the outer edge of `lane_id` in `section` at `ds`: cumulative
/// widths outward from the centre lane (§11.1 numbering).
double outer_edge(const LaneSection& section, int lane_id, double ds) {
    double edge = 0.0;
    if (lane_id > 0) {
        for (int i = 1; i <= lane_id; ++i) {
            const auto it = section.lanes.find(i);
            if (it != section.lanes.end()) {
                edge += width_at(it->second, ds);
            }
        }
    } else if (lane_id < 0) {
        for (int i = -1; i >= lane_id; --i) {
            const auto it = section.lanes.find(i);
            if (it != section.lanes.end()) {
                edge -= width_at(it->second, ds);
            }
        }
    }
    return edge;
}

/// Relative-lane arithmetic per ASAM OpenSCENARIO XML 1.4.0 §7.4.1.4: count
/// positive towards +t, the centre lane 0 "is not counted as a lane and thus
/// omitted".
int skip_center(int lane_id, int count) {
    int target = lane_id + count;
    if (lane_id > 0 && target <= 0) {
        --target;
    } else if (lane_id < 0 && target >= 0) {
        ++target;
    }
    return target;
}

/// Negative lanes drive along +s, positive lanes against it (§11.3.1,
/// right-hand traffic — the subset's driving-direction convention).
bool drives_forward(int lane_id) {
    return lane_id < 0;
}

} // namespace

OpenDriveRoadQuery::OpenDriveRoadQuery(Map map) : junctions_(std::move(map.junctions)) {
    for (auto& [id, road] : map.roads) {
        RoadEntry entry(std::move(road));
        if (entry.line.ok()) {
            roads_.emplace(id, std::move(entry));
        }
    }
}

const OpenDriveRoadQuery::RoadEntry*
OpenDriveRoadQuery::entry_of(const std::string& road_id) const {
    const auto it = roads_.find(road_id);
    return it != roads_.end() ? &it->second : nullptr;
}

const LaneSection* OpenDriveRoadQuery::section_at(const Road& road, double s) const {
    const LaneSection* active = nullptr;
    for (const LaneSection& section : road.sections) {
        if (section.s <= s + kEdgeTolerance) {
            active = &section;
        } else {
            break;
        }
    }
    return active;
}

const Lane* OpenDriveRoadQuery::lane_at(const Road& road, int lane_id, double s) const {
    const LaneSection* section = section_at(road, s);
    if (section == nullptr) {
        return nullptr;
    }
    const auto it = section->lanes.find(lane_id);
    return it != section->lanes.end() ? &it->second : nullptr;
}

// --- conversions ------------------------------------------------------------

bool OpenDriveRoadQuery::to_lane_position(double x, double y, double z,
                                          gateway::LanePosition& out) const {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || std::fabs(z) > kZTolerance) {
        return false;
    }
    bool found = false;
    double best_d2 = std::numeric_limits<double>::infinity();
    for (const auto& [id, entry] : roads_) {
        if (entry.road.sections.empty()) {
            continue; // geometry-only road: no lanes, nothing to stand on
        }
        const auto track = entry.line.project(x, y);
        if (!track.has_value()) {
            continue;
        }
        const LaneSection* section = section_at(entry.road, track->s);
        if (section == nullptr || section->lanes.empty()) {
            continue;
        }
        const double ds = track->s - section->s;
        // The lanes map is ordered by id, so its ends are the outermost lanes.
        const int max_id = std::max(0, section->lanes.rbegin()->first);
        const int min_id = std::min(0, section->lanes.begin()->first);
        const double left_edge = outer_edge(*section, max_id, ds);
        const double right_edge = outer_edge(*section, min_id, ds);
        if (track->t > left_edge + kEdgeTolerance || track->t < right_edge - kEdgeTolerance) {
            continue; // laterally outside the cross-section: not on this road
        }
        const RefPose foot = entry.line.pose_at(track->s);
        const double dx = x - foot.x;
        const double dy = y - foot.y;
        const double d2 = dx * dx + dy * dy;
        // Beyond the reference line's extent the projection clamps to the
        // road end and the planar distance exceeds |t| by the longitudinal
        // overhang: such a point is past the road, not on it. The slack
        // covers the projection's own convergence error (~1e-8 s).
        const double lateral_abs = std::fabs(track->t);
        if (std::sqrt(d2) - lateral_abs > 1e-6) {
            continue;
        }
        if (d2 < best_d2) {
            best_d2 = d2;
            found = true;
            out.road_id = id;
            out.s = track->s;
            out.t = track->t;
            // The lane whose t-interval contains the point; exactly t = 0 is
            // the centre lane.
            out.lane_id = 0;
            if (track->t > kEdgeTolerance) {
                for (int i = 1;; ++i) {
                    const auto it = section->lanes.find(i);
                    if (it == section->lanes.end()) {
                        break;
                    }
                    if (track->t <= outer_edge(*section, i, ds) + kEdgeTolerance) {
                        out.lane_id = i;
                        break;
                    }
                }
            } else if (track->t < -kEdgeTolerance) {
                for (int i = -1;; --i) {
                    const auto it = section->lanes.find(i);
                    if (it == section->lanes.end()) {
                        break;
                    }
                    if (track->t >= outer_edge(*section, i, ds) - kEdgeTolerance) {
                        out.lane_id = i;
                        break;
                    }
                }
            }
        }
    }
    return found;
}

bool OpenDriveRoadQuery::to_world_position(const gateway::LanePosition& position, double& x,
                                           double& y, double& z) const {
    if (!std::isfinite(position.s) || !std::isfinite(position.t)) {
        return false;
    }
    const RoadEntry* entry = entry_of(position.road_id);
    if (entry == nullptr || entry->road.sections.empty()) {
        return false;
    }
    if (position.s < -kEdgeTolerance || position.s > entry->line.length() + kEdgeTolerance) {
        return false;
    }
    const double s = std::clamp(position.s, 0.0, entry->line.length());
    const LaneSection* section = section_at(entry->road, s);
    if (section == nullptr || section->lanes.count(position.lane_id) == 0) {
        return false;
    }
    // t must stay within the cross-section: a kilometre-offset "lane
    // position" does not identify a location on this road.
    const double ds = s - section->s;
    const int max_id = std::max(0, section->lanes.rbegin()->first);
    const int min_id = std::min(0, section->lanes.begin()->first);
    if (position.t > outer_edge(*section, max_id, ds) + kEdgeTolerance ||
        position.t < outer_edge(*section, min_id, ds) - kEdgeTolerance) {
        return false;
    }
    const RefPose foot = entry->line.pose_at(s);
    const runtime::SinCos tangent = runtime::det_sincos(foot.heading);
    x = foot.x - position.t * tangent.sin;
    y = foot.y + position.t * tangent.cos;
    z = 0.0; // flat world: the road surface plane
    return true;
}

bool OpenDriveRoadQuery::road_heading(const std::string& road_id, double s,
                                      double& out_heading) const {
    const RoadEntry* entry = entry_of(road_id);
    if (entry == nullptr || !std::isfinite(s) || s < -kEdgeTolerance ||
        s > entry->line.length() + kEdgeTolerance) {
        return false;
    }
    out_heading = entry->line.pose_at(std::clamp(s, 0.0, entry->line.length())).heading;
    return true;
}

// --- lane queries ------------------------------------------------------------

bool OpenDriveRoadQuery::lane_exists(const std::string& road_id, int lane_id, double s) const {
    const RoadEntry* entry = entry_of(road_id);
    if (entry == nullptr || !std::isfinite(s) || s < -kEdgeTolerance ||
        s > entry->line.length() + kEdgeTolerance) {
        return false;
    }
    return lane_at(entry->road, lane_id, std::clamp(s, 0.0, entry->line.length())) != nullptr;
}

bool OpenDriveRoadQuery::lane_width(const std::string& road_id, int lane_id, double s,
                                    double& out_width) const {
    const RoadEntry* entry = entry_of(road_id);
    if (entry == nullptr || !std::isfinite(s) || s < -kEdgeTolerance ||
        s > entry->line.length() + kEdgeTolerance) {
        return false;
    }
    const double clamped = std::clamp(s, 0.0, entry->line.length());
    const LaneSection* section = section_at(entry->road, clamped);
    if (section == nullptr) {
        return false;
    }
    const auto it = section->lanes.find(lane_id);
    if (it == section->lanes.end() || it->second.widths.empty()) {
        return false; // unknown lane, or the width-less centre lane
    }
    out_width = width_at(it->second, clamped - section->s);
    return true;
}

bool OpenDriveRoadQuery::lane_center_offset(const std::string& road_id, int lane_id, double s,
                                            double& out_t) const {
    const RoadEntry* entry = entry_of(road_id);
    if (entry == nullptr || !std::isfinite(s) || s < -kEdgeTolerance ||
        s > entry->line.length() + kEdgeTolerance) {
        return false;
    }
    const double clamped = std::clamp(s, 0.0, entry->line.length());
    const LaneSection* section = section_at(entry->road, clamped);
    if (section == nullptr || section->lanes.count(lane_id) == 0) {
        return false;
    }
    if (lane_id == 0) {
        out_t = 0.0; // the centre lane is the reference line
        return true;
    }
    const double ds = clamped - section->s;
    const double outer = outer_edge(*section, lane_id, ds);
    const double inner = outer_edge(*section, lane_id > 0 ? lane_id - 1 : lane_id + 1, ds);
    out_t = (outer + inner) / 2.0;
    return true;
}

bool OpenDriveRoadQuery::lane_type(const std::string& road_id, int lane_id, double s,
                                   std::string& out_type) const {
    const RoadEntry* entry = entry_of(road_id);
    if (entry == nullptr || !std::isfinite(s) || s < -kEdgeTolerance ||
        s > entry->line.length() + kEdgeTolerance) {
        return false;
    }
    const Lane* lane = lane_at(entry->road, lane_id, std::clamp(s, 0.0, entry->line.length()));
    if (lane == nullptr || lane->type.empty()) {
        return false;
    }
    out_type = lane->type;
    return true;
}

bool OpenDriveRoadQuery::relative_lane(const std::string& road_id, int lane_id, int count,
                                       int& out_lane_id) const {
    const RoadEntry* entry = entry_of(road_id);
    if (entry == nullptr) {
        return false;
    }
    const int target = skip_center(lane_id, count);
    // Both endpoints must name lanes the road actually has (in any of its
    // sections; relative_lane carries no s in the frozen interface).
    const auto lane_known = [&](int id) {
        return std::any_of(
            entry->road.sections.begin(), entry->road.sections.end(),
            [&](const LaneSection& section) { return section.lanes.count(id) != 0; });
    };
    if (!lane_known(lane_id) || !lane_known(target)) {
        return false;
    }
    out_lane_id = target;
    return true;
}

// --- s-range queries ---------------------------------------------------------

bool OpenDriveRoadQuery::road_length(const std::string& road_id, double& out_length) const {
    const RoadEntry* entry = entry_of(road_id);
    if (entry == nullptr) {
        return false;
    }
    out_length = entry->line.length();
    return true;
}

bool OpenDriveRoadQuery::lane_s_range(const std::string& road_id, int lane_id, double s,
                                      double& out_s_begin, double& out_s_end) const {
    const RoadEntry* entry = entry_of(road_id);
    if (entry == nullptr || !std::isfinite(s) || s < -kEdgeTolerance ||
        s > entry->line.length() + kEdgeTolerance) {
        return false;
    }
    const double clamped = std::clamp(s, 0.0, entry->line.length());
    const std::vector<LaneSection>& sections = entry->road.sections;
    std::size_t index = sections.size();
    for (std::size_t i = 0; i < sections.size(); ++i) {
        if (sections[i].s <= clamped + kEdgeTolerance) {
            index = i;
        }
    }
    if (index == sections.size() || sections[index].lanes.count(lane_id) == 0) {
        return false;
    }
    // Sections partition [0, length]: extend over adjacent sections that
    // still carry this lane id — the maximal contiguous interval.
    std::size_t first = index;
    while (first > 0 && sections[first - 1].lanes.count(lane_id) != 0) {
        --first;
    }
    std::size_t last = index;
    while (last + 1 < sections.size() && sections[last + 1].lanes.count(lane_id) != 0) {
        ++last;
    }
    out_s_begin = sections[first].s;
    out_s_end = last + 1 < sections.size() ? sections[last + 1].s : entry->line.length();
    return true;
}

// --- routes ------------------------------------------------------------------

std::vector<OpenDriveRoadQuery::RouteNode>
OpenDriveRoadQuery::neighbors_of(const RouteNode& node) const {
    std::vector<RouteNode> neighbors;
    const RoadEntry* entry = entry_of(node.road_id);
    if (entry == nullptr || entry->road.sections.empty()) {
        return neighbors;
    }
    const bool forward = drives_forward(node.lane_id);
    // The boundary section in driving direction, and the lane's link across
    // it (§11.6: the successor/predecessor id names the lane on the linked
    // element).
    const LaneSection& boundary =
        forward ? entry->road.sections.back() : entry->road.sections.front();
    const auto lane_it = boundary.lanes.find(node.lane_id);
    if (lane_it == boundary.lanes.end()) {
        return neighbors;
    }
    const std::optional<int> linked_lane =
        forward ? lane_it->second.successor : lane_it->second.predecessor;
    const std::optional<RoadLink>& link = forward ? entry->road.successor : entry->road.predecessor;
    if (!link.has_value()) {
        return neighbors;
    }
    if (link->kind == RoadLink::Kind::Road) {
        if (linked_lane.has_value()) {
            neighbors.push_back({link->element_id, *linked_lane});
        }
    } else {
        const auto junction_it = junctions_.find(link->element_id);
        if (junction_it != junctions_.end()) {
            for (const JunctionConnection& connection : junction_it->second.connections) {
                if (connection.incoming_road != node.road_id) {
                    continue;
                }
                for (const JunctionLaneLink& lane_link : connection.lane_links) {
                    if (lane_link.from == node.lane_id) {
                        neighbors.push_back({connection.connecting_road, lane_link.to});
                    }
                }
            }
        }
    }
    std::sort(neighbors.begin(), neighbors.end());
    neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
    return neighbors;
}

bool OpenDriveRoadQuery::route_pair(const gateway::LanePosition& from,
                                    const gateway::LanePosition& to,
                                    std::vector<gateway::RouteSpan>& out_spans) const {
    const RoadEntry* from_entry = entry_of(from.road_id);
    const RoadEntry* to_entry = entry_of(to.road_id);
    if (from_entry == nullptr || to_entry == nullptr || from.lane_id == 0 || to.lane_id == 0 ||
        !std::isfinite(from.s) || !std::isfinite(to.s)) {
        return false;
    }
    if (lane_at(from_entry->road, from.lane_id, from.s) == nullptr ||
        lane_at(to_entry->road, to.lane_id, to.s) == nullptr) {
        return false;
    }

    const RouteNode start{from.road_id, from.lane_id};
    const RouteNode goal{to.road_id, to.lane_id};
    const bool start_forward = drives_forward(from.lane_id);

    // Direct span when the target lies ahead on the same lane; behind on the
    // same lane is unreachable in the subset (no loop back onto the entry
    // node — the node model keys distance per (road, lane), so a cycle back
    // to the start never relaxes it).
    if (start == goal) {
        const double delta = to.s - from.s;
        if ((start_forward && delta >= -kEdgeTolerance) ||
            (!start_forward && delta <= kEdgeTolerance)) {
            out_spans.push_back({from.road_id, from.lane_id, from.s, to.s});
            return true;
        }
        return false;
    }

    // Dijkstra over (road, lane) nodes, weighted by driven length; the goal
    // edge weighs only the final leg up to to.s. The frontier is an ordered
    // set keyed (distance, road id, lane id): equal distances pop in
    // lexicographic (road id, lane id) order — the documented deterministic
    // tie-break. The goal is accepted when POPPED, so the emitted path is
    // shortest, not first-discovered.
    std::map<RouteNode, double> distance;
    std::map<RouteNode, RouteNode> parent;
    std::set<std::tuple<double, std::string, int>> frontier;
    distance[start] = start_forward ? from_entry->line.length() - from.s : from.s;
    frontier.insert({distance[start], start.road_id, start.lane_id});

    while (!frontier.empty()) {
        const auto [dist, road_id, lane_id] = *frontier.begin();
        frontier.erase(frontier.begin());
        const RouteNode node{road_id, lane_id};
        if (distance.at(node) < dist) {
            continue; // stale frontier entry
        }
        if (node == goal) {
            std::vector<RouteNode> path{node};
            RouteNode walk = node;
            while (!(walk == start)) {
                walk = parent.at(walk);
                path.push_back(walk);
            }
            std::reverse(path.begin(), path.end());
            for (std::size_t i = 0; i < path.size(); ++i) {
                const RoadEntry* road = entry_of(path[i].road_id);
                const bool fwd = drives_forward(path[i].lane_id);
                const double entry_s = i == 0 ? from.s : (fwd ? 0.0 : road->line.length());
                const double exit_s =
                    i + 1 == path.size() ? to.s : (fwd ? road->line.length() : 0.0);
                out_spans.push_back({path[i].road_id, path[i].lane_id, entry_s, exit_s});
            }
            return true;
        }
        for (const RouteNode& next : neighbors_of(node)) {
            const RoadEntry* next_entry = entry_of(next.road_id);
            if (next_entry == nullptr ||
                lane_at(next_entry->road, next.lane_id,
                        drives_forward(next.lane_id) ? 0.0 : next_entry->line.length()) ==
                    nullptr) {
                continue;
            }
            const bool next_forward = drives_forward(next.lane_id);
            const double leg = next == goal
                                   ? (next_forward ? to.s : next_entry->line.length() - to.s)
                                   : next_entry->line.length();
            const double next_dist = dist + leg;
            const auto known = distance.find(next);
            if (known == distance.end() || next_dist < known->second) {
                if (known != distance.end()) {
                    frontier.erase({known->second, next.road_id, next.lane_id});
                }
                distance[next] = next_dist;
                parent[next] = node;
                frontier.insert({next_dist, next.road_id, next.lane_id});
            }
        }
    }
    return false;
}

bool OpenDriveRoadQuery::build_route(const std::vector<gateway::LanePosition>& waypoints,
                                     std::vector<gateway::RouteSpan>& out_spans) const {
    if (waypoints.size() < 2) {
        return false;
    }
    std::vector<gateway::RouteSpan> spans;
    for (std::size_t i = 0; i + 1 < waypoints.size(); ++i) {
        if (!route_pair(waypoints[i], waypoints[i + 1], spans)) {
            return false;
        }
    }
    out_spans = std::move(spans);
    return true;
}

bool OpenDriveRoadQuery::position_along_route(const std::vector<gateway::RouteSpan>& route,
                                              const gateway::LanePosition& position,
                                              double& out_distance) const {
    if (!std::isfinite(position.s)) {
        return false;
    }
    double travelled = 0.0;
    for (const gateway::RouteSpan& span : route) {
        const double lo = std::min(span.s_begin, span.s_end) - kEdgeTolerance;
        const double hi = std::max(span.s_begin, span.s_end) + kEdgeTolerance;
        if (span.road_id == position.road_id && span.lane_id == position.lane_id &&
            position.s >= lo && position.s <= hi) {
            out_distance = travelled + std::fabs(position.s - span.s_begin);
            return true;
        }
        travelled += std::fabs(span.s_end - span.s_begin);
    }
    return false;
}

} // namespace scena::opendrive
