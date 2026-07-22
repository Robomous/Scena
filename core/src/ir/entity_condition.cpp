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

#include "scena/ir/entity_condition.h"

#include <cmath>
#include <optional>
#include <utility>

#include "scena/ir/evaluation_context.h"
#include "scena/ir/rule.h"
#include "scena/runtime/detmath.h"

namespace scena::ir {

namespace {

/// Projects a scalar quantity defined along the body longitudinal (+x) axis —
/// under Scena's scalar-velocity model both speed and acceleration are such
/// quantities — onto the requested direction (§7.6.5.1). No direction ⇒ the
/// total magnitude; longitudinal ⇒ the signed value; lateral/vertical ⇒
/// exactly 0.0, because the scalar model carries no sideslip or vertical
/// motion (this generalizes when a velocity vector lands, p2-s*). `std::fabs`
/// is exact, so no determinism concern.
double project_longitudinal(double value, std::optional<DirectionalDimension> direction) {
    if (!direction.has_value()) {
        return std::fabs(value);
    }
    switch (*direction) {
    case DirectionalDimension::Longitudinal:
        return value;
    case DirectionalDimension::Lateral:
    case DirectionalDimension::Vertical:
        return 0.0;
    }
    return 0.0;
}

} // namespace

ByEntityCondition::ByEntityCondition(TriggeringEntities triggering_entities)
    : triggering_entities_(std::move(triggering_entities)) {}

bool ByEntityCondition::evaluate(const EvaluationContext& context) const {
    const std::vector<std::string>& refs = triggering_entities_.entity_refs;
    // An empty set has no entity to satisfy the expression; validation rejects
    // it at init, so this only guards a hand-built IR.
    if (refs.empty()) {
        return false;
    }
    // Evaluate every entity — no short-circuit — so the outcome never depends
    // on evaluation order and a future per-entity diagnostic stays stable.
    // Then reduce per §7.6.5.1: Any is OR, All is AND.
    bool any = false;
    bool all = true;
    for (const std::string& entity_id : refs) {
        const bool held = evaluate_for_entity(context, entity_id);
        any = any || held;
        all = all && held;
    }
    return triggering_entities_.rule == TriggeringEntitiesRule::All ? all : any;
}

const TriggeringEntities& ByEntityCondition::triggering_entities() const {
    return triggering_entities_;
}

// --- SpeedCondition ---------------------------------------------------------

SpeedCondition::SpeedCondition(TriggeringEntities triggering, double value, Rule rule,
                               std::optional<DirectionalDimension> direction)
    : ByEntityCondition(std::move(triggering)), value_(value), rule_(rule), direction_(direction) {}

bool SpeedCondition::evaluate_for_entity(const EvaluationContext& context,
                                         std::string_view entity_id) const {
    const std::optional<EntityKinematics> kinematics = context.entity_kinematics(entity_id);
    if (!kinematics.has_value()) {
        return false;
    }
    // The model's `speed` is longitudinal along heading, so the projection of
    // the velocity onto a body axis reduces to project_longitudinal(speed).
    const double measured = project_longitudinal(kinematics->state.speed, direction_);
    return compare(measured, rule_, value_);
}

double SpeedCondition::value() const {
    return value_;
}

Rule SpeedCondition::rule() const {
    return rule_;
}

const std::optional<DirectionalDimension>& SpeedCondition::direction() const {
    return direction_;
}

// --- RelativeSpeedCondition -------------------------------------------------

RelativeSpeedCondition::RelativeSpeedCondition(TriggeringEntities triggering,
                                               std::string entity_ref, double value, Rule rule,
                                               std::optional<DirectionalDimension> direction)
    : ByEntityCondition(std::move(triggering)), entity_ref_(std::move(entity_ref)), value_(value),
      rule_(rule), direction_(direction) {}

bool RelativeSpeedCondition::evaluate_for_entity(const EvaluationContext& context,
                                                 std::string_view entity_id) const {
    const std::optional<EntityKinematics> triggering = context.entity_kinematics(entity_id);
    const std::optional<EntityKinematics> reference = context.entity_kinematics(entity_ref_);
    if (!triggering.has_value() || !reference.has_value()) {
        return false; // a missing reference makes every triggering entity false
    }
    const double s_trig = triggering->state.speed;
    const double s_ref = reference->state.speed;

    double relative = 0.0;
    if (!direction_.has_value()) {
        // Spec: "speed_rel = speed(triggering entity) - speed(reference
        // entity)", range ]-inf..inf[ — a signed difference of the entities'
        // total speeds, NOT the magnitude of the relative velocity vector.
        relative = std::fabs(s_trig) - std::fabs(s_ref);
    } else {
        // Directional: project the relative velocity onto the *triggering*
        // entity's axes (§ RelativeSpeedCondition). This is the sprint's only
        // trigonometry, so det_sincos is mandatory (raw libm trig is
        // CI-blocked). Operand order is fixed — no reassociation.
        const runtime::SinCos t = runtime::det_sincos(triggering->state.heading);
        const runtime::SinCos r = runtime::det_sincos(reference->state.heading);
        const double vx = s_trig * t.cos - s_ref * r.cos;
        const double vy = s_trig * t.sin - s_ref * r.sin;
        switch (*direction_) {
        case DirectionalDimension::Longitudinal:
            relative = vx * t.cos + vy * t.sin;
            break;
        case DirectionalDimension::Lateral:
            relative = vx * (-t.sin) + vy * t.cos;
            break;
        case DirectionalDimension::Vertical:
            relative = 0.0; // no vertical component in the scalar model
            break;
        }
    }
    return compare(relative, rule_, value_);
}

const std::string& RelativeSpeedCondition::entity_ref() const {
    return entity_ref_;
}

double RelativeSpeedCondition::value() const {
    return value_;
}

Rule RelativeSpeedCondition::rule() const {
    return rule_;
}

const std::optional<DirectionalDimension>& RelativeSpeedCondition::direction() const {
    return direction_;
}

// --- AccelerationCondition --------------------------------------------------

AccelerationCondition::AccelerationCondition(TriggeringEntities triggering, double value, Rule rule,
                                             std::optional<DirectionalDimension> direction)
    : ByEntityCondition(std::move(triggering)), value_(value), rule_(rule), direction_(direction) {}

bool AccelerationCondition::evaluate_for_entity(const EvaluationContext& context,
                                                std::string_view entity_id) const {
    const std::optional<EntityKinematics> kinematics = context.entity_kinematics(entity_id);
    if (!kinematics.has_value() || !kinematics->acceleration.has_value()) {
        // No entity, or fewer than two samples so far: absent ⇒ false for
        // every rule (including LessThan / NotEqualTo — the quantity does not
        // exist, so no comparison holds).
        return false;
    }
    const double measured = project_longitudinal(*kinematics->acceleration, direction_);
    return compare(measured, rule_, value_);
}

double AccelerationCondition::value() const {
    return value_;
}

Rule AccelerationCondition::rule() const {
    return rule_;
}

const std::optional<DirectionalDimension>& AccelerationCondition::direction() const {
    return direction_;
}

// --- StandStillCondition ----------------------------------------------------

StandStillCondition::StandStillCondition(TriggeringEntities triggering, double duration)
    : ByEntityCondition(std::move(triggering)), duration_(duration) {}

bool StandStillCondition::evaluate_for_entity(const EvaluationContext& context,
                                              std::string_view entity_id) const {
    const std::optional<EntityKinematics> kinematics = context.entity_kinematics(entity_id);
    if (!kinematics.has_value()) {
        return false;
    }
    // No `rule`: holds once at-rest time reaches the duration. Exact >= so a
    // duration of 0 holds the instant the entity is at rest.
    return kinematics->standstill_seconds >= duration_;
}

double StandStillCondition::duration() const {
    return duration_;
}

// --- TraveledDistanceCondition ----------------------------------------------

TraveledDistanceCondition::TraveledDistanceCondition(TriggeringEntities triggering, double value)
    : ByEntityCondition(std::move(triggering)), value_(value) {}

bool TraveledDistanceCondition::evaluate_for_entity(const EvaluationContext& context,
                                                    std::string_view entity_id) const {
    const std::optional<EntityKinematics> kinematics = context.entity_kinematics(entity_id);
    if (!kinematics.has_value()) {
        return false;
    }
    // No `rule`: holds once the odometer reaches the value (value 0 holds
    // immediately). Exact >=, no tolerance.
    return kinematics->traveled_distance >= value_;
}

double TraveledDistanceCondition::value() const {
    return value_;
}

// --- ReachPositionCondition -------------------------------------------------

ReachPositionCondition::ReachPositionCondition(TriggeringEntities triggering,
                                               WorldPosition position, double tolerance)
    : ByEntityCondition(std::move(triggering)), position_(position), tolerance_(tolerance) {}

bool ReachPositionCondition::evaluate_for_entity(const EvaluationContext& context,
                                                 std::string_view entity_id) const {
    const std::optional<EntityKinematics> kinematics = context.entity_kinematics(entity_id);
    if (!kinematics.has_value()) {
        return false;
    }
    // 2D horizontal distance: the spec calls `tolerance` the "radius of
    // tolerance circle", so z is ignored (documented spec silence, ADR-0008).
    // std::sqrt is IEEE-exact (unlike std::hypot), fixed operand order.
    const double dx = kinematics->state.x - position_.x;
    const double dy = kinematics->state.y - position_.y;
    const double distance = std::sqrt(dx * dx + dy * dy);
    return distance <= tolerance_;
}

const WorldPosition& ReachPositionCondition::position() const {
    return position_;
}

double ReachPositionCondition::tolerance() const {
    return tolerance_;
}

} // namespace scena::ir
