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

#include "scena/runtime/position_resolver.h"

#include <charconv>
#include <cmath>
#include <utility>
#include <variant>
#include <vector>

#include "scena/gateway/road_query.h"
#include "scena/ir/route.h"
#include "scena/ir/trajectory.h"
#include "scena/runtime/detmath.h"
#include "scena/runtime/trajectory_eval.h"

namespace scena::runtime {
namespace {

/// Composes the orientation of a resolved pose against the reference frame's
/// orientation (base_*), per §Orientation. A missing Orientation copies the
/// reference orientation; an Absolute one replaces it with world-frame angles;
/// a Relative one is an additive counter-clockwise shift on top of it.
void compose_orientation(double base_h, double base_p, double base_r,
                         const std::optional<ir::Orientation>& orientation, Pose& out) {
    if (!orientation.has_value()) {
        out.heading = base_h;
        out.pitch = base_p;
        out.roll = base_r;
        return;
    }
    if (orientation->type == ir::ReferenceContext::Absolute) {
        out.heading = orientation->h;
        out.pitch = orientation->p;
        out.roll = orientation->r;
        return;
    }
    out.heading = base_h + orientation->h;
    out.pitch = base_p + orientation->p;
    out.roll = base_r + orientation->r;
}

PositionResolution ok() {
    return PositionResolution{Status::Ok, {}, {}};
}

PositionResolution unresolved_reference(const std::string& entity_ref) {
    return PositionResolution{Status::SemanticError,
                              "position references entity '" + entity_ref +
                                  "', which is not an active entity",
                              {}};
}

PositionResolution unsupported(std::string message, std::string rule_id = {}) {
    return PositionResolution{Status::UnsupportedFeature, std::move(message), std::move(rule_id)};
}

PositionResolution semantic(std::string message) {
    return PositionResolution{Status::SemanticError, std::move(message), {}};
}

PositionResolution no_road_network() {
    return unsupported("road-, lane- and route-relative positions require a road network "
                       "(IRoadQuery) attached to the resolver");
}

/// pi to double precision, for turning a route-against-s orientation base.
constexpr double kPi = 3.14159265358979323846;

/// Parses an OpenDRIVE-style signed integer lane id (locale-independent,
/// whole-token, per the cross-platform rule).
std::optional<int> parse_lane_id(const std::string& text) {
    int value = 0;
    const char* const first = text.data();
    const char* const last = text.data() + text.size();
    const std::from_chars_result result = std::from_chars(first, last, value);
    if (result.ec != std::errc{} || result.ptr != last || text.empty()) {
        return std::nullopt;
    }
    return value;
}

/// Fills `out` from road coordinates: the world point of (road, s, t) with
/// the road s-axis tangent as the orientation base (§6.3.2 corrected
/// calculation; pitch/roll stay 0 — the road surface is the z = 0 plane in
/// the v0.0.1 subset). `lane_for_validation` is the lane id handed to
/// to_world_position, whose backend validates it exists at `s`.
PositionResolution resolve_on_road(const gateway::IRoadQuery& road, const std::string& road_id,
                                   int lane_for_validation, double s, double t,
                                   double extra_heading,
                                   const std::optional<ir::Orientation>& orientation, Pose& out) {
    double base_heading = 0.0;
    if (!road.road_heading(road_id, s, base_heading)) {
        return semantic("position names road '" + road_id +
                        "', which the road network cannot answer at the requested s");
    }
    gateway::LanePosition lane;
    lane.road_id = road_id;
    lane.lane_id = lane_for_validation;
    lane.s = s;
    lane.t = t;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    if (!road.to_world_position(lane, x, y, z)) {
        return semantic("position does not identify a location on road '" + road_id + "'");
    }
    out.x = x;
    out.y = y;
    out.z = z;
    compose_orientation(base_heading + extra_heading, 0.0, 0.0, orientation, out);
    return ok();
}

} // namespace

PositionResolver::PositionResolver(PoseLookup lookup, const gateway::IRoadQuery* road) noexcept
    : lookup_(std::move(lookup)), road_(road) {}

PositionResolution PositionResolver::resolve(const ir::Position& position, Pose& out) const {
    // WorldPosition (§WorldPosition, §6.3.1): the pose is taken directly. Its
    // h/p/r are the world-frame orientation (WorldPosition has no separate
    // Orientation element), so they are inherently absolute.
    if (const auto* world = std::get_if<ir::WorldPosition>(&position)) {
        out.x = world->x;
        out.y = world->y;
        out.z = world->z;
        out.heading = world->h;
        out.pitch = world->p;
        out.roll = world->r;
        return ok();
    }

    // RelativeWorldPosition (§RelativeWorldPosition): deltas along the WORLD
    // axes added to the reference entity's position — not rotated by its
    // heading.
    if (const auto* rel_world = std::get_if<ir::RelativeWorldPosition>(&position)) {
        const EntityState* ref = lookup_(rel_world->entity_ref);
        if (ref == nullptr) {
            return unresolved_reference(rel_world->entity_ref);
        }
        out.x = ref->x + rel_world->dx;
        out.y = ref->y + rel_world->dy;
        out.z = ref->z + rel_world->dz;
        compose_orientation(ref->heading, ref->pitch, ref->roll, rel_world->orientation, out);
        return ok();
    }

    // RelativeObjectPosition (§RelativeObjectPosition): deltas expressed in the
    // reference entity's LOCAL frame, so they rotate with its orientation. The
    // straight-line runtime keeps pitch and roll at 0 (entity_state.h), so the
    // rotation is a yaw about +Z — exact for every state the runtime produces;
    // the full Z→Y→X frame rotation lands when the runtime carries a non-zero
    // pitch/roll. Uses det_sincos for bit-identical results (ADR-0006).
    if (const auto* rel_obj = std::get_if<ir::RelativeObjectPosition>(&position)) {
        const EntityState* ref = lookup_(rel_obj->entity_ref);
        if (ref == nullptr) {
            return unresolved_reference(rel_obj->entity_ref);
        }
        const SinCos rot = det_sincos(ref->heading);
        out.x = ref->x + rel_obj->dx * rot.cos - rel_obj->dy * rot.sin;
        out.y = ref->y + rel_obj->dx * rot.sin + rel_obj->dy * rot.cos;
        out.z = ref->z + rel_obj->dz;
        compose_orientation(ref->heading, ref->pitch, ref->roll, rel_obj->orientation, out);
        return ok();
    }

    // RoadPosition (§RoadPosition, §6.3.2): (road, s, t) maps to the world
    // through the backend; the orientation base is the s-axis tangent.
    if (const auto* road_pos = std::get_if<ir::RoadPosition>(&position)) {
        if (road_ == nullptr) {
            return no_road_network();
        }
        // Lane 0 (the centre lane, always present in a laned road) carries
        // the validation; t is authoritative for the lateral offset.
        return resolve_on_road(*road_, road_pos->road_id, 0, road_pos->s, road_pos->t, 0.0,
                               road_pos->orientation, out);
    }

    // RelativeRoadPosition (§RelativeRoadPosition): ds/dt on the reference
    // entity's own road coordinates. The v0.0.1 subset keeps the target on
    // the same road: a delta that leaves the road is reported, not followed
    // onto linked roads.
    if (const auto* rel_road = std::get_if<ir::RelativeRoadPosition>(&position)) {
        if (road_ == nullptr) {
            return no_road_network();
        }
        const EntityState* ref = lookup_(rel_road->entity_ref);
        if (ref == nullptr) {
            return unresolved_reference(rel_road->entity_ref);
        }
        gateway::LanePosition ref_lane;
        if (!road_->to_lane_position(ref->x, ref->y, ref->z, ref_lane)) {
            return semantic("position references entity '" + rel_road->entity_ref +
                            "', which is not on the road network");
        }
        const double target_s = ref_lane.s + rel_road->ds;
        double length = 0.0;
        if (!road_->road_length(ref_lane.road_id, length) || target_s < 0.0 || target_s > length) {
            return unsupported("relative road position leaves road '" + ref_lane.road_id +
                               "'; continuing onto linked roads is beyond the v0.0.1 subset");
        }
        return resolve_on_road(*road_, ref_lane.road_id, 0, target_s, ref_lane.t + rel_road->dt,
                               0.0, rel_road->orientation, out);
    }

    // LanePosition (§LanePosition, §6.3.2): the lateral position is the
    // target lane's centre line plus `offset`.
    if (const auto* lane_pos = std::get_if<ir::LanePosition>(&position)) {
        if (road_ == nullptr) {
            return no_road_network();
        }
        const std::optional<int> lane_id = parse_lane_id(lane_pos->lane_id);
        if (!lane_id.has_value()) {
            return PositionResolution{Status::ValidationError,
                                      "lane position lane id '" + lane_pos->lane_id +
                                          "' is not a signed integer lane id",
                                      {}};
        }
        double center_t = 0.0;
        if (!road_->lane_center_offset(lane_pos->road_id, *lane_id, lane_pos->s, center_t)) {
            return semantic("lane position names lane '" + lane_pos->lane_id + "' of road '" +
                            lane_pos->road_id + "', which the road network cannot answer");
        }
        return resolve_on_road(*road_, lane_pos->road_id, *lane_id, lane_pos->s,
                               center_t + lane_pos->offset, 0.0, lane_pos->orientation, out);
    }

    // RelativeLanePosition (§RelativeLanePosition): d_lane lanes over from
    // the reference entity's lane (centre lane skipped, §7.4.1.4 semantics in
    // the backend), then ds along the ROAD reference line. `ds_lane` (along
    // the lane centre line) is beyond the v0.0.1 subset and reported.
    if (const auto* rel_lane = std::get_if<ir::RelativeLanePosition>(&position)) {
        if (road_ == nullptr) {
            return no_road_network();
        }
        if (rel_lane->ds_lane.has_value()) {
            return unsupported("relative lane position dsLane (distance along the lane centre "
                               "line) is beyond the v0.0.1 subset; use ds");
        }
        const EntityState* ref = lookup_(rel_lane->entity_ref);
        if (ref == nullptr) {
            return unresolved_reference(rel_lane->entity_ref);
        }
        gateway::LanePosition ref_lane;
        if (!road_->to_lane_position(ref->x, ref->y, ref->z, ref_lane)) {
            return semantic("position references entity '" + rel_lane->entity_ref +
                            "', which is not on the road network");
        }
        int target_lane = 0;
        if (!road_->relative_lane(ref_lane.road_id, ref_lane.lane_id, rel_lane->d_lane,
                                  target_lane)) {
            return semantic("position references entity '" + rel_lane->entity_ref +
                            "' with a relative lane the road network does not have");
        }
        const double target_s = ref_lane.s + rel_lane->ds.value_or(0.0);
        double length = 0.0;
        if (!road_->road_length(ref_lane.road_id, length) || target_s < 0.0 || target_s > length) {
            return unsupported("relative lane position leaves road '" + ref_lane.road_id +
                               "'; continuing onto linked roads is beyond the v0.0.1 subset");
        }
        double center_t = 0.0;
        if (!road_->lane_center_offset(ref_lane.road_id, target_lane, target_s, center_t)) {
            return semantic("relative lane position target lane does not exist at the "
                            "requested s on road '" +
                            ref_lane.road_id + "'");
        }
        return resolve_on_road(*road_, ref_lane.road_id, target_lane, target_s,
                               center_t + rel_lane->offset, 0.0, rel_lane->orientation, out);
    }

    // RoutePosition (§RoutePosition): the route's waypoints are resolved onto
    // the road network and connected with the backend's deterministic
    // routing; the in-route coordinate (§InRoutePosition) then selects a
    // point along the resulting spans.
    if (const auto* route_pos = std::get_if<ir::RoutePosition>(&position)) {
        if (road_ == nullptr) {
            return no_road_network();
        }
        if (route_pos->route == nullptr || route_pos->route->waypoints.size() < 2) {
            return PositionResolution{
                Status::ValidationError,
                "route position references no route with at least two waypoints",
                {}};
        }
        const bool lane_form = route_pos->lane_id.has_value();
        const int in_route_forms =
            (route_pos->from_entity.has_value() ? 1 : 0) + (route_pos->path_s.has_value() ? 1 : 0);
        if (in_route_forms != 1) {
            return PositionResolution{Status::ValidationError,
                                      "route position needs exactly one in-route coordinate: "
                                      "an entity, or a pathS",
                                      {}};
        }
        std::vector<gateway::LanePosition> waypoints;
        waypoints.reserve(route_pos->route->waypoints.size());
        for (const ir::Waypoint& waypoint : route_pos->route->waypoints) {
            gateway::LanePosition lane;
            if (!road_->to_lane_position(waypoint.position.x, waypoint.position.y,
                                         waypoint.position.z, lane)) {
                return semantic("route position waypoint is not on the road network");
            }
            waypoints.push_back(lane);
        }
        std::vector<gateway::RouteSpan> spans;
        if (!road_->build_route(waypoints, spans)) {
            return semantic("route position route has no connected path through the road "
                            "network");
        }
        double path_s = 0.0;
        if (route_pos->from_entity.has_value()) {
            const EntityState* ref = lookup_(*route_pos->from_entity);
            if (ref == nullptr) {
                return unresolved_reference(*route_pos->from_entity);
            }
            gateway::LanePosition entity_lane;
            if (!road_->to_lane_position(ref->x, ref->y, ref->z, entity_lane) ||
                !road_->position_along_route(spans, entity_lane, path_s)) {
                return semantic("route position entity '" + *route_pos->from_entity +
                                "' is not on the route");
            }
        } else {
            path_s = *route_pos->path_s;
        }
        double total = 0.0;
        for (const gateway::RouteSpan& span : spans) {
            total += std::fabs(span.s_end - span.s_begin);
        }
        if (!(path_s >= 0.0) || path_s > total) {
            return semantic("route position pathS lies beyond the route");
        }
        double remaining = path_s;
        const gateway::RouteSpan* on_span = &spans.back();
        for (const gateway::RouteSpan& span : spans) {
            const double span_length = std::fabs(span.s_end - span.s_begin);
            if (remaining <= span_length) {
                on_span = &span;
                break;
            }
            remaining -= span_length;
        }
        const bool reversed = on_span->s_end < on_span->s_begin;
        const double road_s =
            reversed ? on_span->s_begin - remaining : on_span->s_begin + remaining;
        int validation_lane = on_span->lane_id;
        double target_t = 0.0;
        if (lane_form) {
            const std::optional<int> lane_id = parse_lane_id(*route_pos->lane_id);
            if (!lane_id.has_value()) {
                return PositionResolution{Status::ValidationError,
                                          "route position lane id '" + *route_pos->lane_id +
                                              "' is not a signed integer lane id",
                                          {}};
            }
            double center_t = 0.0;
            if (!road_->lane_center_offset(on_span->road_id, *lane_id, road_s, center_t)) {
                return semantic("route position names lane '" + *route_pos->lane_id +
                                "', which road '" + on_span->road_id +
                                "' does not have at the requested pathS");
            }
            validation_lane = *lane_id;
            target_t = center_t + route_pos->lane_offset;
        } else {
            target_t = route_pos->t.value_or(0.0);
        }
        // A span traversed against the road s-axis turns the orientation
        // base by pi: the route's forward direction is the reference.
        return resolve_on_road(*road_, on_span->road_id, validation_lane, road_s, target_t,
                               reversed ? kPi : 0.0, route_pos->orientation, out);
    }

    // GeoPosition (§GeoPosition): the geographic→world projection is defined by
    // the road network's geodetic datum, which Scena does not yet consume.
    if (std::holds_alternative<ir::GeoPosition>(position)) {
        return unsupported("geographic positions require a geodetic datum in the road network",
                           "asam.net:xosc:1.1.0:positioning.geodetic_datum_defined");
    }

    // TrajectoryPosition (§TrajectoryPosition, §6.9.5): evaluate the referenced
    // trajectory's geometry at arc length s, then step the lateral offset t
    // along the left-normal of the tangent. Road-projected lateral distance
    // (§6.4.6) is not needed here — the trajectory is its own reference line.
    if (const auto* traj_pos = std::get_if<ir::TrajectoryPosition>(&position)) {
        if (traj_pos->trajectory == nullptr) {
            return PositionResolution{
                Status::ValidationError, "trajectory position references no trajectory", {}};
        }
        const TrajectoryEvaluator evaluator(*traj_pos->trajectory);
        if (!evaluator.ok()) {
            return PositionResolution{evaluator.status().status, evaluator.status().message,
                                      evaluator.status().rule_id};
        }
        const Pose on_curve = evaluator.pose_at_arclength(traj_pos->s);
        // Left-normal of the tangent (heading + pi/2): (-sin h, cos h).
        const SinCos tangent = det_sincos(on_curve.heading);
        out.x = on_curve.x - traj_pos->t * tangent.sin;
        out.y = on_curve.y + traj_pos->t * tangent.cos;
        out.z = on_curve.z;
        compose_orientation(on_curve.heading, 0.0, 0.0, traj_pos->orientation, out);
        return ok();
    }

    // Unreachable: the variant has ten alternatives and all are handled above.
    return unsupported("unhandled position variant");
}

} // namespace scena::runtime
