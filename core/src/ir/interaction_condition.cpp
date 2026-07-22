// SPDX-License-Identifier: MIT
#include "scena/ir/interaction_condition.h"

#include <cmath>
#include <optional>
#include <utility>

#include "scena/ir/evaluation_context.h"
#include "scena/ir/rule.h"
#include "scena/runtime/detmath.h"
#include "scena/runtime/obb2.h"

namespace scena::ir {

namespace {

/// Builds the world-frame 2D oriented box for an entity from its state and its
/// body-frame bounding box. The heading's sine/cosine come from det_sincos —
/// the single trig site for the whole distance calculation — so the geometry
/// is bit-identical across platforms. Fixed operand order; length/width halved
/// by an exact multiply.
runtime::Obb2 make_obb(const EntityState& state, const BoundingBox& box) {
    const runtime::SinCos sc = runtime::det_sincos(state.heading);
    runtime::Obb2 obb;
    obb.cx = state.x + box.center_x * sc.cos - box.center_y * sc.sin;
    obb.cy = state.y + box.center_x * sc.sin + box.center_y * sc.cos;
    obb.cos_h = sc.cos;
    obb.sin_h = sc.sin;
    obb.hx = box.length * 0.5;
    obb.hy = box.width * 0.5;
    return obb;
}

/// The unit axis a longitudinal/lateral distance projects onto, in world
/// coordinates. Entity CS uses the triggering entity's body axes (§6.4.4, "the
/// coordinate system relates to the triggering entity") via det_sincos; World
/// CS uses the world X/Y axes. Only Entity and World reach here — road-based
/// systems are deferred before this is called.
void projection_axis(CoordinateSystem cs, RelativeDistanceType rdt, double heading, double& ux,
                     double& uy) {
    if (cs == CoordinateSystem::World) {
        // Longitudinal ⇒ world X, lateral ⇒ world Y.
        ux = (rdt == RelativeDistanceType::Longitudinal) ? 1.0 : 0.0;
        uy = (rdt == RelativeDistanceType::Longitudinal) ? 0.0 : 1.0;
        return;
    }
    const runtime::SinCos sc = runtime::det_sincos(heading);
    if (rdt == RelativeDistanceType::Longitudinal) {
        ux = sc.cos; // body x̂
        uy = sc.sin;
    } else {
        ux = -sc.sin; // body ŷ
        uy = sc.cos;
    }
}

/// The effective distance parameters after defaults/deprecations are resolved.
struct DistanceSpec {
    CoordinateSystem cs;      ///< Effective coordinate system.
    RelativeDistanceType rdt; ///< Effective relative-distance type.
    bool freespace;
};

/// Measures the distance from the triggering entity to a target — another
/// entity (`target_entity` non-null) or a fixed position (`target_point`) —
/// under `spec`, per ASAM OpenSCENARIO XML 1.4.0 §6.4. Returns std::nullopt
/// when the measurement cannot be made and the per-entity predicate is
/// therefore a deterministic false: a road-based coordinate system (deferred
/// to p3-s4) or a freespace request with a missing bounding box (geometry is
/// optional until p2-s1). All arithmetic is IEEE-exact with fixed operand
/// order; the only trig is det_sincos inside make_obb / projection_axis.
std::optional<double> measure_distance(const EntityKinematics& trigger,
                                       const EntityKinematics* target_entity,
                                       const WorldPosition& target_point, const DistanceSpec& spec) {
    if (spec.cs == CoordinateSystem::Lane || spec.cs == CoordinateSystem::Road ||
        spec.cs == CoordinateSystem::Trajectory) {
        return std::nullopt; // deferred: no road network (p3-s4)
    }
    const bool euclidean = spec.rdt == RelativeDistanceType::EuclidianDistance ||
                           spec.rdt == RelativeDistanceType::CartesianDistance;

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
    const runtime::Obb2 trigger_obb = make_obb(trigger.state, *trigger.bounding_box);

    // A target entity's geometry, when the target is an entity.
    std::optional<runtime::Obb2> target_obb;
    if (target_entity != nullptr) {
        if (!target_entity->bounding_box.has_value()) {
            return std::nullopt;
        }
        target_obb = make_obb(target_entity->state, *target_entity->bounding_box);
    }

    if (euclidean) {
        if (target_obb.has_value()) {
            return runtime::obb_distance(trigger_obb, *target_obb);
        }
        return runtime::point_obb_distance(tx, ty, trigger_obb);
    }

    // Longitudinal/lateral freespace: the gap between the boxes' projections on
    // the effective axis (a point target projects to a degenerate interval).
    double ux = 0.0;
    double uy = 0.0;
    projection_axis(spec.cs, spec.rdt, trigger.state.heading, ux, uy);
    double a_lo = 0.0;
    double a_hi = 0.0;
    runtime::obb_project(trigger_obb, ux, uy, a_lo, a_hi);
    double b_lo = tx * ux + ty * uy;
    double b_hi = b_lo;
    if (target_obb.has_value()) {
        runtime::obb_project(*target_obb, ux, uy, b_lo, b_hi);
    }
    return std::fmax(0.0, std::fmax(b_lo - a_hi, a_lo - b_hi));
}

} // namespace

// --- DistanceCondition ------------------------------------------------------

DistanceCondition::DistanceCondition(TriggeringEntities triggering, WorldPosition position,
                                     double value, bool freespace, Rule rule,
                                     std::optional<CoordinateSystem> coordinate_system,
                                     std::optional<RelativeDistanceType> relative_distance_type,
                                     std::optional<RoutingAlgorithm> routing_algorithm,
                                     std::optional<bool> along_route)
    : ByEntityCondition(std::move(triggering)), position_(position), value_(value),
      freespace_(freespace), rule_(rule), coordinate_system_(coordinate_system),
      relative_distance_type_(relative_distance_type), routing_algorithm_(routing_algorithm),
      along_route_(along_route) {}

bool DistanceCondition::evaluate_for_entity(const EvaluationContext& context,
                                            std::string_view entity_id) const {
    const std::optional<EntityKinematics> trigger = context.entity_kinematics(entity_id);
    if (!trigger.has_value()) {
        return false;
    }
    const DistanceSpec spec{effective_coordinate_system(), effective_relative_distance_type(),
                            freespace_};
    const std::optional<double> measured = measure_distance(*trigger, nullptr, position_, spec);
    if (!measured.has_value()) {
        return false;
    }
    return compare(*measured, rule_, value_);
}

const WorldPosition& DistanceCondition::position() const {
    return position_;
}

double DistanceCondition::value() const {
    return value_;
}

bool DistanceCondition::freespace() const {
    return freespace_;
}

Rule DistanceCondition::rule() const {
    return rule_;
}

const std::optional<CoordinateSystem>& DistanceCondition::coordinate_system() const {
    return coordinate_system_;
}

const std::optional<RelativeDistanceType>& DistanceCondition::relative_distance_type() const {
    return relative_distance_type_;
}

const std::optional<RoutingAlgorithm>& DistanceCondition::routing_algorithm() const {
    return routing_algorithm_;
}

const std::optional<bool>& DistanceCondition::along_route() const {
    return along_route_;
}

CoordinateSystem DistanceCondition::effective_coordinate_system() const {
    if (coordinate_system_.has_value()) {
        return *coordinate_system_;
    }
    // alongRoute is honored only when neither coordinateSystem nor
    // relativeDistanceType is authored (§ DistanceCondition); alongRoute=true
    // then selects road-based routing, which is deferred.
    if (!relative_distance_type_.has_value() && along_route_.value_or(false)) {
        return CoordinateSystem::Road;
    }
    return CoordinateSystem::Entity;
}

RelativeDistanceType DistanceCondition::effective_relative_distance_type() const {
    return relative_distance_type_.value_or(RelativeDistanceType::EuclidianDistance);
}

// --- RelativeDistanceCondition ----------------------------------------------

RelativeDistanceCondition::RelativeDistanceCondition(
    TriggeringEntities triggering, std::string entity_ref, double value, bool freespace,
    RelativeDistanceType relative_distance_type, Rule rule,
    std::optional<CoordinateSystem> coordinate_system,
    std::optional<RoutingAlgorithm> routing_algorithm)
    : ByEntityCondition(std::move(triggering)), entity_ref_(std::move(entity_ref)), value_(value),
      freespace_(freespace), relative_distance_type_(relative_distance_type), rule_(rule),
      coordinate_system_(coordinate_system), routing_algorithm_(routing_algorithm) {}

bool RelativeDistanceCondition::evaluate_for_entity(const EvaluationContext& context,
                                                    std::string_view entity_id) const {
    const std::optional<EntityKinematics> trigger = context.entity_kinematics(entity_id);
    const std::optional<EntityKinematics> reference = context.entity_kinematics(entity_ref_);
    if (!trigger.has_value() || !reference.has_value()) {
        return false; // a missing reference makes every triggering entity false
    }
    const DistanceSpec spec{effective_coordinate_system(), relative_distance_type_, freespace_};
    const std::optional<double> measured =
        measure_distance(*trigger, &reference.value(), WorldPosition{}, spec);
    if (!measured.has_value()) {
        return false;
    }
    return compare(*measured, rule_, value_);
}

const std::string& RelativeDistanceCondition::entity_ref() const {
    return entity_ref_;
}

double RelativeDistanceCondition::value() const {
    return value_;
}

bool RelativeDistanceCondition::freespace() const {
    return freespace_;
}

RelativeDistanceType RelativeDistanceCondition::relative_distance_type() const {
    return relative_distance_type_;
}

Rule RelativeDistanceCondition::rule() const {
    return rule_;
}

const std::optional<CoordinateSystem>& RelativeDistanceCondition::coordinate_system() const {
    return coordinate_system_;
}

const std::optional<RoutingAlgorithm>& RelativeDistanceCondition::routing_algorithm() const {
    return routing_algorithm_;
}

CoordinateSystem RelativeDistanceCondition::effective_coordinate_system() const {
    return coordinate_system_.value_or(CoordinateSystem::Entity);
}

} // namespace scena::ir
