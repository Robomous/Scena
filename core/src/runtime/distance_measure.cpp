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

#include "scena/runtime/detmath.h"
#include "scena/runtime/obb2.h"

namespace scena::runtime {

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
    if (spec.cs == ir::CoordinateSystem::Lane || spec.cs == ir::CoordinateSystem::Road ||
        spec.cs == ir::CoordinateSystem::Trajectory) {
        return std::nullopt; // deferred: no road network (p3-s4)
    }
    const bool euclidean = spec.rdt == ir::RelativeDistanceType::EuclidianDistance ||
                           spec.rdt == ir::RelativeDistanceType::CartesianDistance;

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

} // namespace scena::runtime
