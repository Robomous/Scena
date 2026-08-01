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

#include "scena/runtime/distance_measure.h"

#include <cmath>
#include <optional>

#include "scena/gateway/road_query.h"
#include "scena/runtime/detmath.h"
#include "scena/runtime/obb2.h"

namespace scena::runtime {

namespace {

/// The s- and t-intervals an entity occupies on its road, linearized into
/// the road tangent frame at its reference point (p3-s4 subset): corner
/// projections use the s-axis direction at the entity's own s, which is
/// exact on straight roads and first-order on curved ones (ADR-0019). For a
/// reference-point measure the intervals are degenerate.
struct RoadExtent {
    gateway::LanePosition lane;
    double s_lo = 0.0;
    double s_hi = 0.0;
    double t_lo = 0.0;
    double t_hi = 0.0;
};

std::optional<RoadExtent> road_extent(const gateway::IRoadQuery& road, const EntityState& state,
                                      const ir::BoundingBox* box) {
    RoadExtent extent;
    if (!road.to_lane_position(state.x, state.y, state.z, extent.lane)) {
        return std::nullopt;
    }
    extent.s_lo = extent.s_hi = extent.lane.s;
    extent.t_lo = extent.t_hi = extent.lane.t;
    if (box == nullptr) {
        return extent;
    }
    double tangent_heading = 0.0;
    if (!road.road_heading(extent.lane.road_id, extent.lane.s, tangent_heading)) {
        return std::nullopt;
    }
    gateway::LanePosition foot_position;
    foot_position.road_id = extent.lane.road_id;
    foot_position.lane_id = 0;
    foot_position.s = extent.lane.s;
    foot_position.t = 0.0;
    double foot_x = 0.0;
    double foot_y = 0.0;
    double foot_z = 0.0;
    if (!road.to_world_position(foot_position, foot_x, foot_y, foot_z)) {
        return std::nullopt;
    }
    const SinCos tangent = det_sincos(tangent_heading);
    const Obb2 obb = make_obb(state, *box);
    // Corner projections onto the s-axis (u) and the left normal (n).
    double u_lo = 0.0;
    double u_hi = 0.0;
    obb_project(obb, tangent.cos, tangent.sin, u_lo, u_hi);
    const double foot_u = foot_x * tangent.cos + foot_y * tangent.sin;
    extent.s_lo = extent.lane.s + (u_lo - foot_u);
    extent.s_hi = extent.lane.s + (u_hi - foot_u);
    double n_lo = 0.0;
    double n_hi = 0.0;
    obb_project(obb, -tangent.sin, tangent.cos, n_lo, n_hi);
    const double foot_n = foot_x * -tangent.sin + foot_y * tangent.cos;
    extent.t_lo = n_lo - foot_n;
    extent.t_hi = n_hi - foot_n;
    return extent;
}

/// The velocity components of a state in its road tangent frame: rate of s
/// (along the s-axis) and rate of t (along the left normal), from the
/// scalar-velocity model speed and the heading-to-tangent angle.
std::optional<SinCos> road_velocity_frame(const gateway::IRoadQuery& road,
                                          const gateway::LanePosition& lane, double heading) {
    double tangent_heading = 0.0;
    if (!road.road_heading(lane.road_id, lane.s, tangent_heading)) {
        return std::nullopt;
    }
    return det_sincos(heading - tangent_heading);
}

} // namespace

Obb2 make_obb(const EntityState& state, const ir::BoundingBox& box) {
    const SinCos sc = det_sincos(state.heading);
    Obb2 obb;
    obb.cx = state.x + box.center_x * sc.cos - box.center_y * sc.sin;
    obb.cy = state.y + box.center_x * sc.sin + box.center_y * sc.cos;
    obb.cos_h = sc.cos;
    obb.sin_h = sc.sin;
    obb.hx = box.length * 0.5;
    obb.hy = box.width * 0.5;
    return obb;
}

void projection_axis(ir::CoordinateSystem cs, ir::RelativeDistanceType rdt, double heading,
                     double& ux, double& uy) {
    if (cs == ir::CoordinateSystem::World) {
        // Longitudinal ⇒ world X, lateral ⇒ world Y.
        ux = (rdt == ir::RelativeDistanceType::Longitudinal) ? 1.0 : 0.0;
        uy = (rdt == ir::RelativeDistanceType::Longitudinal) ? 0.0 : 1.0;
        return;
    }
    const SinCos sc = det_sincos(heading);
    if (rdt == ir::RelativeDistanceType::Longitudinal) {
        ux = sc.cos; // body x̂
        uy = sc.sin;
    } else {
        ux = -sc.sin; // body ŷ
        uy = sc.cos;
    }
}

std::optional<double> measure_distance(const ir::EntityKinematics& trigger,
                                       const ir::EntityKinematics* target_entity,
                                       const ir::WorldPosition& target_point,
                                       const DistanceSpec& spec) {
    if (spec.cs == ir::CoordinateSystem::Trajectory) {
        return std::nullopt; // deferred: needs the followed trajectory's geometry
    }
    const bool euclidean = spec.rdt == ir::RelativeDistanceType::EuclidianDistance ||
                           spec.rdt == ir::RelativeDistanceType::CartesianDistance;
    // Euclidean distance is coordinate-system independent (§6.4.3), so a
    // Road/Lane CS with a euclidean type measures below like any other.
    if (!euclidean &&
        (spec.cs == ir::CoordinateSystem::Lane || spec.cs == ir::CoordinateSystem::Road)) {
        if (spec.road == nullptr) {
            return std::nullopt; // no road network: deferred (pre-p3-s4 behaviour)
        }
        const bool lateral = spec.rdt == ir::RelativeDistanceType::Lateral;
        const ir::BoundingBox* trigger_box =
            spec.freespace && trigger.bounding_box.has_value() ? &*trigger.bounding_box : nullptr;
        if (spec.freespace && trigger_box == nullptr) {
            return std::nullopt;
        }
        const std::optional<RoadExtent> a = road_extent(*spec.road, trigger.state, trigger_box);
        if (!a.has_value()) {
            return std::nullopt;
        }
        std::optional<RoadExtent> b;
        if (target_entity != nullptr) {
            const ir::BoundingBox* target_box =
                spec.freespace && target_entity->bounding_box.has_value()
                    ? &*target_entity->bounding_box
                    : nullptr;
            if (spec.freespace && target_box == nullptr) {
                return std::nullopt;
            }
            b = road_extent(*spec.road, target_entity->state, target_box);
        } else {
            EntityState point;
            point.x = target_point.x;
            point.y = target_point.y;
            point.z = target_point.z;
            b = road_extent(*spec.road, point, nullptr);
        }
        if (!b.has_value() || a->lane.road_id != b->lane.road_id) {
            return std::nullopt; // off the network, or not on the same road
        }
        if (!spec.freespace) {
            return lateral ? std::fabs(b->lane.t - a->lane.t) : std::fabs(b->lane.s - a->lane.s);
        }
        const double a_lo = lateral ? a->t_lo : a->s_lo;
        const double a_hi = lateral ? a->t_hi : a->s_hi;
        const double b_lo = lateral ? b->t_lo : b->s_lo;
        const double b_hi = lateral ? b->t_hi : b->s_hi;
        return std::fmax(0.0, std::fmax(b_lo - a_hi, a_lo - b_hi));
    }

    // The target's reference point (its origin for an entity, else the position).
    const double tx = target_entity != nullptr ? target_entity->state.x : target_point.x;
    const double ty = target_entity != nullptr ? target_entity->state.y : target_point.y;
    const double tz = target_entity != nullptr ? target_entity->state.z : target_point.z;

    if (!spec.freespace) {
        // Reference-point distance: entity origins (§6.4.7.1).
        const double dx = tx - trigger.state.x;
        const double dy = ty - trigger.state.y;
        if (euclidean) {
            const double dz = tz - trigger.state.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz); // §6.4.3, 3D, CS-independent
        }
        double ux = 0.0;
        double uy = 0.0;
        projection_axis(spec.cs, spec.rdt, trigger.state.heading, ux, uy);
        return std::fabs(dx * ux + dy * uy); // §6.4.4
    }

    // Freespace distance (§6.4.7.2): needs the triggering entity's geometry.
    if (!trigger.bounding_box.has_value()) {
        return std::nullopt;
    }
    const Obb2 trigger_obb = make_obb(trigger.state, *trigger.bounding_box);

    // A target entity's geometry, when the target is an entity.
    std::optional<Obb2> target_obb;
    if (target_entity != nullptr) {
        if (!target_entity->bounding_box.has_value()) {
            return std::nullopt;
        }
        target_obb = make_obb(target_entity->state, *target_entity->bounding_box);
    }

    if (euclidean) {
        if (target_obb.has_value()) {
            return obb_distance(trigger_obb, *target_obb);
        }
        return point_obb_distance(tx, ty, trigger_obb);
    }

    // Longitudinal/lateral freespace: the gap between the boxes' projections on
    // the effective axis (a point target projects to a degenerate interval).
    double ux = 0.0;
    double uy = 0.0;
    projection_axis(spec.cs, spec.rdt, trigger.state.heading, ux, uy);
    double a_lo = 0.0;
    double a_hi = 0.0;
    obb_project(trigger_obb, ux, uy, a_lo, a_hi);
    double b_lo = tx * ux + ty * uy;
    double b_hi = b_lo;
    if (target_obb.has_value()) {
        obb_project(*target_obb, ux, uy, b_lo, b_hi);
    }
    return std::fmax(0.0, std::fmax(b_lo - a_hi, a_lo - b_hi));
}

std::optional<double> closing_speed(const ir::EntityKinematics& trigger,
                                    const ir::EntityKinematics* target_entity,
                                    const ir::WorldPosition& target_point,
                                    const DistanceSpec& spec) {
    const SinCos a = det_sincos(trigger.state.heading);
    const double vax = trigger.state.speed * a.cos;
    const double vay = trigger.state.speed * a.sin;
    double vbx = 0.0;
    double vby = 0.0;
    if (target_entity != nullptr) {
        const SinCos b = det_sincos(target_entity->state.heading);
        vbx = target_entity->state.speed * b.cos;
        vby = target_entity->state.speed * b.sin;
    }

    const double tx = target_entity != nullptr ? target_entity->state.x : target_point.x;
    const double ty = target_entity != nullptr ? target_entity->state.y : target_point.y;
    const double tz = target_entity != nullptr ? target_entity->state.z : target_point.z;
    const double rx = tx - trigger.state.x;
    const double ry = ty - trigger.state.y;

    const bool euclidean = spec.rdt == ir::RelativeDistanceType::EuclidianDistance ||
                           spec.rdt == ir::RelativeDistanceType::CartesianDistance;
    if (euclidean) {
        const double rz = tz - trigger.state.z;
        const double d_ref = std::sqrt(rx * rx + ry * ry + rz * rz);
        if (d_ref == 0.0) {
            return std::nullopt; // coincident reference points
        }
        // Rate of approach along the line of sight; the z relative velocity is
        // zero in the scalar-velocity model. Fixed operand order.
        return (rx * (vax - vbx) + ry * (vay - vby)) / d_ref;
    }

    if (spec.cs == ir::CoordinateSystem::Trajectory) {
        return std::nullopt;
    }
    if (spec.cs == ir::CoordinateSystem::Lane || spec.cs == ir::CoordinateSystem::Road) {
        // Road-coordinate closing speed (§6.4.5): each entity's speed is
        // decomposed into the road tangent frame at its own s; the closing
        // rate reduces the s- (longitudinal) or t- (lateral) separation.
        if (spec.road == nullptr) {
            return std::nullopt;
        }
        gateway::LanePosition a_lane;
        if (!spec.road->to_lane_position(trigger.state.x, trigger.state.y, trigger.state.z,
                                         a_lane)) {
            return std::nullopt;
        }
        gateway::LanePosition b_lane;
        double vb_s = 0.0;
        double vb_t = 0.0;
        if (target_entity != nullptr) {
            if (!spec.road->to_lane_position(target_entity->state.x, target_entity->state.y,
                                             target_entity->state.z, b_lane)) {
                return std::nullopt;
            }
            const std::optional<SinCos> b_frame =
                road_velocity_frame(*spec.road, b_lane, target_entity->state.heading);
            if (!b_frame.has_value()) {
                return std::nullopt;
            }
            vb_s = target_entity->state.speed * b_frame->cos;
            vb_t = target_entity->state.speed * b_frame->sin;
        } else if (!spec.road->to_lane_position(target_point.x, target_point.y, target_point.z,
                                                b_lane)) {
            return std::nullopt; // a position target is stationary
        }
        if (a_lane.road_id != b_lane.road_id) {
            return std::nullopt;
        }
        const std::optional<SinCos> a_frame =
            road_velocity_frame(*spec.road, a_lane, trigger.state.heading);
        if (!a_frame.has_value()) {
            return std::nullopt;
        }
        const double va_s = trigger.state.speed * a_frame->cos;
        const double va_t = trigger.state.speed * a_frame->sin;
        const bool lateral = spec.rdt == ir::RelativeDistanceType::Lateral;
        const double separation = lateral ? b_lane.t - a_lane.t : b_lane.s - a_lane.s;
        if (separation == 0.0) {
            return std::nullopt;
        }
        const double relative = lateral ? vb_t - va_t : vb_s - va_s;
        return -std::copysign(1.0, separation) * relative;
    }

    // Longitudinal/lateral: the component of relative velocity that reduces the
    // signed axis separation s = u·r.
    double ux = 0.0;
    double uy = 0.0;
    projection_axis(spec.cs, spec.rdt, trigger.state.heading, ux, uy);
    const double s = rx * ux + ry * uy;
    if (s == 0.0) {
        return std::nullopt; // coincident along the axis
    }
    return -std::copysign(1.0, s) * ((vbx - vax) * ux + (vby - vay) * uy);
}

std::optional<double>
road_signed_longitudinal_gap(const gateway::IRoadQuery* road, const EntityState& actor,
                             const std::optional<ir::BoundingBox>& actor_box,
                             const EntityState& reference,
                             const std::optional<ir::BoundingBox>& reference_box, bool freespace) {
    if (road == nullptr) {
        return std::nullopt;
    }
    const ir::BoundingBox* a_box = freespace && actor_box.has_value() ? &*actor_box : nullptr;
    const ir::BoundingBox* b_box =
        freespace && reference_box.has_value() ? &*reference_box : nullptr;
    if (freespace && (a_box == nullptr || b_box == nullptr)) {
        return std::nullopt;
    }
    const std::optional<RoadExtent> a = road_extent(*road, actor, a_box);
    const std::optional<RoadExtent> b = road_extent(*road, reference, b_box);
    if (!a.has_value() || !b.has_value() || a->lane.road_id != b->lane.road_id) {
        return std::nullopt;
    }
    const double reference_point_gap = b->lane.s - a->lane.s;
    if (!freespace) {
        return reference_point_gap;
    }
    // Reference ahead on s: front-of-actor to rear-of-reference; behind:
    // mirrored and negated — the signed_longitudinal_gap convention.
    return reference_point_gap >= 0.0 ? b->s_lo - a->s_hi : b->s_hi - a->s_lo;
}

std::optional<double> road_signed_lateral_gap(const gateway::IRoadQuery* road,
                                              const EntityState& actor,
                                              const std::optional<ir::BoundingBox>& actor_box,
                                              const EntityState& reference,
                                              const std::optional<ir::BoundingBox>& reference_box,
                                              bool freespace) {
    if (road == nullptr) {
        return std::nullopt;
    }
    const ir::BoundingBox* a_box = freespace && actor_box.has_value() ? &*actor_box : nullptr;
    const ir::BoundingBox* b_box =
        freespace && reference_box.has_value() ? &*reference_box : nullptr;
    if (freespace && (a_box == nullptr || b_box == nullptr)) {
        return std::nullopt;
    }
    const std::optional<RoadExtent> a = road_extent(*road, actor, a_box);
    const std::optional<RoadExtent> b = road_extent(*road, reference, b_box);
    if (!a.has_value() || !b.has_value() || a->lane.road_id != b->lane.road_id) {
        return std::nullopt;
    }
    const double reference_point_gap = a->lane.t - b->lane.t;
    if (!freespace) {
        return reference_point_gap;
    }
    const bool actor_left = reference_point_gap >= 0.0;
    double clearance = actor_left ? a->t_lo - b->t_hi : b->t_lo - a->t_hi;
    if (clearance < 0.0) {
        clearance = 0.0; // overlapping flanks: no clearance, sign kept
    }
    return actor_left ? clearance : -clearance;
}

} // namespace scena::runtime
