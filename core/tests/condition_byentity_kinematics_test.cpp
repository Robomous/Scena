// SPDX-License-Identifier: MIT
// By-entity conditions (ASAM OpenSCENARIO XML 1.4.0 §7.6.5.1): the
// triggering-entities any/all frame and the six kinematic/position
// conditions. This file covers the shared frame and the pure-context
// per-entity predicates; engine-driven derived-state tests live alongside.
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "scena/entity_state.h"
#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/entity_condition.h"
#include "scena/ir/evaluation_context.h"
#include "scena/ir/rule.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/trigger.h"
#include "scena/runtime/detmath.h"
#include "scena/runtime/scheduler.h"

namespace ir = scena::ir;
using ir::DirectionalDimension;
using ir::EntityKinematics;
using ir::Rule;
using ir::TriggeringEntities;
using ir::TriggeringEntitiesRule;
using scena::EntityState;
using scena::runtime::ActionOutcome;
using scena::runtime::Scheduler;

namespace {

/// A context that answers entity_kinematics from a fixed table — the pure-unit
/// driver for the by-entity predicates (no engine, no storyboard).
class KinematicsContext final : public ir::EvaluationContext {
public:
    explicit KinematicsContext(std::map<std::string, EntityKinematics> entities)
        : entities_(std::move(entities)) {}

    [[nodiscard]] double simulation_time() const override { return 0.0; }

    [[nodiscard]] std::optional<EntityKinematics>
    entity_kinematics(std::string_view id) const override {
        const auto it = entities_.find(std::string(id));
        if (it == entities_.end()) {
            return std::nullopt;
        }
        return it->second;
    }

private:
    std::map<std::string, EntityKinematics> entities_;
};

/// Minimal by-entity predicate for exercising the any/all frame: an entity
/// "holds" when it is observable and moving.
class MovingProbe final : public ir::ByEntityCondition {
public:
    explicit MovingProbe(TriggeringEntities triggering)
        : ir::ByEntityCondition(std::move(triggering)) {}

protected:
    [[nodiscard]] bool evaluate_for_entity(const ir::EvaluationContext& context,
                                           std::string_view entity_id) const override {
        const auto kinematics = context.entity_kinematics(entity_id);
        return kinematics.has_value() && kinematics->state.speed > 0.0;
    }
};

TriggeringEntities triggering(TriggeringEntitiesRule rule, std::vector<std::string> refs) {
    return TriggeringEntities{rule, std::move(refs)};
}

EntityKinematics with_speed(double speed) {
    EntityKinematics kinematics;
    kinematics.state.speed = speed;
    return kinematics;
}

/// Kinematics with speed, heading and (optionally) an acceleration sample.
EntityKinematics kin(double speed, double heading = 0.0,
                     std::optional<double> acceleration = std::nullopt) {
    EntityKinematics kinematics;
    kinematics.state.speed = speed;
    kinematics.state.heading = heading;
    kinematics.acceleration = acceleration;
    return kinematics;
}

/// A single-entity "ego" triggering set — the common case for the per-entity
/// measurement tests.
TriggeringEntities ego_only() {
    return triggering(TriggeringEntitiesRule::Any, {"ego"});
}

bool holds(const ir::Condition& condition, const ir::EvaluationContext& context) {
    return condition.evaluate(context);
}

} // namespace

// ---------------------------------------------------------------------------
// The entity-kinematics facet (ADR-0007 default-absent contract)
// ---------------------------------------------------------------------------

TEST(EntityKinematicsFacetTest, TimeOnlyContextHasNoEntityFacet) {
    const ir::TimeOnlyEvaluationContext context{0.0};
    EXPECT_FALSE(context.entity_kinematics("ego").has_value());
    // A by-entity condition over a facet-less context is deterministically false.
    const MovingProbe probe{triggering(TriggeringEntitiesRule::Any, {"ego"})};
    EXPECT_FALSE(holds(probe, context));
}

// ---------------------------------------------------------------------------
// TriggeringEntities any/all reduction (§7.6.5.1)
// ---------------------------------------------------------------------------

TEST(TriggeringEntitiesTest, AnyAllReductionMatrix) {
    const KinematicsContext context{{
        {"mover", with_speed(5.0)},
        {"still", with_speed(0.0)},
        {"other", with_speed(3.0)},
    }};

    // Any: at least one entity moving.
    EXPECT_TRUE(
        holds(MovingProbe{triggering(TriggeringEntitiesRule::Any, {"mover", "still"})}, context));
    EXPECT_FALSE(holds(MovingProbe{triggering(TriggeringEntitiesRule::Any, {"still"})}, context));
    // All: every entity moving.
    EXPECT_TRUE(
        holds(MovingProbe{triggering(TriggeringEntitiesRule::All, {"mover", "other"})}, context));
    EXPECT_FALSE(
        holds(MovingProbe{triggering(TriggeringEntitiesRule::All, {"mover", "still"})}, context));
    // Single entity: any and all agree.
    EXPECT_TRUE(holds(MovingProbe{triggering(TriggeringEntitiesRule::Any, {"mover"})}, context));
    EXPECT_TRUE(holds(MovingProbe{triggering(TriggeringEntitiesRule::All, {"mover"})}, context));
}

TEST(TriggeringEntitiesTest, EmptyRefsIsAlwaysFalse) {
    const KinematicsContext context{{{"mover", with_speed(5.0)}}};
    EXPECT_FALSE(holds(MovingProbe{triggering(TriggeringEntitiesRule::Any, {})}, context));
    EXPECT_FALSE(holds(MovingProbe{triggering(TriggeringEntitiesRule::All, {})}, context));
}

TEST(TriggeringEntitiesTest, UnobservableEntityContributesFalsePerEntity) {
    // An entity absent from the context is false for that entity only: Any can
    // still hold through another ref; All cannot (ADR-0007 finest grain).
    const KinematicsContext context{{{"mover", with_speed(5.0)}}};
    EXPECT_TRUE(
        holds(MovingProbe{triggering(TriggeringEntitiesRule::Any, {"mover", "ghost"})}, context));
    EXPECT_FALSE(
        holds(MovingProbe{triggering(TriggeringEntitiesRule::All, {"mover", "ghost"})}, context));
}

// ---------------------------------------------------------------------------
// BoundContext forwards the facet to the host context
// ---------------------------------------------------------------------------

TEST(EntityKinematicsFacetTest, BoundContextForwardsEntityKinematics) {
    // A by-entity trigger inside a bound storyboard must observe the entity
    // state the host context supplies — i.e. Scheduler::BoundContext forwards
    // the facet rather than swallowing it.
    ir::Event event;
    event.name = "event";
    ir::Trigger trigger;
    ir::ConditionGroup group;
    ir::TriggerCondition condition;
    condition.expression =
        std::make_shared<MovingProbe>(triggering(TriggeringEntitiesRule::Any, {"ego"}));
    group.conditions.push_back(std::move(condition));
    trigger.groups.push_back(std::move(group));
    event.start_trigger = std::move(trigger);
    event.actions.push_back(std::make_shared<ir::SpeedAction>("ego", 10.0));

    ir::Maneuver maneuver;
    maneuver.name = "maneuver";
    maneuver.events.push_back(std::move(event));
    ir::ManeuverGroup mgroup;
    mgroup.name = "group";
    mgroup.maneuvers.push_back(std::move(maneuver));
    ir::Act act;
    act.name = "act";
    act.groups.push_back(std::move(mgroup));
    ir::Story story;
    story.name = "story";
    story.acts.push_back(std::move(act));
    ir::Storyboard storyboard;
    storyboard.stories.push_back(std::move(story));

    Scheduler scheduler;
    scheduler.bind(storyboard);

    // Host context where ego is not moving: the event stays in standby.
    const KinematicsContext still{{{"ego", with_speed(0.0)}}};
    bool fired = false;
    const auto fire = [&fired](const ir::Action&) {
        fired = true;
        return ActionOutcome::Complete;
    };
    scheduler.step(still, fire);
    EXPECT_FALSE(fired);

    // Host context where ego is moving: the forwarded facet makes it fire.
    const KinematicsContext moving{{{"ego", with_speed(4.0)}}};
    scheduler.step(moving, fire);
    EXPECT_TRUE(fired);
}

// ---------------------------------------------------------------------------
// SpeedCondition (§ SpeedCondition)
// ---------------------------------------------------------------------------

TEST(SpeedConditionTest, TotalSpeedIsMagnitude) {
    // No direction ⇒ total speed = |speed|, so a negative (reversing) speed
    // still compares by magnitude.
    const KinematicsContext context{{{"ego", kin(-8.0)}}};
    EXPECT_TRUE(holds(ir::SpeedCondition{ego_only(), 8.0, Rule::EqualTo}, context));
    EXPECT_TRUE(holds(ir::SpeedCondition{ego_only(), 5.0, Rule::GreaterThan}, context));
    EXPECT_FALSE(holds(ir::SpeedCondition{ego_only(), 8.0, Rule::GreaterThan}, context));
}

TEST(SpeedConditionTest, DirectionalProjection) {
    const KinematicsContext context{{{"ego", kin(-8.0)}}};
    // Longitudinal ⇒ the *signed* speed, so -8 is < 0 and != 8.
    EXPECT_TRUE(holds(
        ir::SpeedCondition{ego_only(), -8.0, Rule::EqualTo, DirectionalDimension::Longitudinal},
        context));
    EXPECT_TRUE(holds(
        ir::SpeedCondition{ego_only(), 0.0, Rule::LessThan, DirectionalDimension::Longitudinal},
        context));
    // Lateral / vertical ⇒ exactly 0.0 in the scalar model.
    EXPECT_TRUE(
        holds(ir::SpeedCondition{ego_only(), 0.0, Rule::EqualTo, DirectionalDimension::Lateral},
              context));
    EXPECT_TRUE(
        holds(ir::SpeedCondition{ego_only(), 0.0, Rule::EqualTo, DirectionalDimension::Vertical},
              context));
}

TEST(SpeedConditionTest, PerEntityAbsentStillFiresUnderAny) {
    // "ghost" is not observable, so it is false, but "ego" satisfies the
    // predicate and Any still holds.
    const KinematicsContext context{{{"ego", kin(10.0)}}};
    const TriggeringEntities any_two = triggering(TriggeringEntitiesRule::Any, {"ego", "ghost"});
    EXPECT_TRUE(holds(ir::SpeedCondition{any_two, 5.0, Rule::GreaterThan}, context));
}

// ---------------------------------------------------------------------------
// RelativeSpeedCondition (§ RelativeSpeedCondition)
// ---------------------------------------------------------------------------

TEST(RelativeSpeedConditionTest, SpecFormulaIsSignedDifferenceOfTotals) {
    // speed_rel = |speed(triggering)| - |speed(reference)|; a slower
    // triggering entity gives a negative relative speed.
    const KinematicsContext context{{{"ego", kin(4.0)}, {"lead", kin(10.0)}}};
    EXPECT_TRUE(
        holds(ir::RelativeSpeedCondition{ego_only(), "lead", -6.0, Rule::EqualTo}, context));
    EXPECT_TRUE(
        holds(ir::RelativeSpeedCondition{ego_only(), "lead", 0.0, Rule::LessThan}, context));
}

TEST(RelativeSpeedConditionTest, AbsentReferenceIsFalse) {
    const KinematicsContext context{{{"ego", kin(4.0)}}}; // no "lead"
    EXPECT_FALSE(
        holds(ir::RelativeSpeedCondition{ego_only(), "lead", 0.0, Rule::NotEqualTo}, context));
}

TEST(RelativeSpeedConditionTest, DirectionalProjectionInTriggeringFrame) {
    // Triggering ego heads along +x (heading 0, exact under det_sincos); the
    // reference heads at 1.0 rad. The projection is in ego's frame, so the
    // longitudinal/lateral split follows the reference's heading only.
    const KinematicsContext context{{{"ego", kin(10.0, 0.0)}, {"lead", kin(4.0, 1.0)}}};
    const scena::runtime::SinCos r = scena::runtime::det_sincos(1.0);
    const double longitudinal = 10.0 - 4.0 * r.cos; // vx, since ego cos=1 sin=0
    const double lateral = -4.0 * r.sin;            // vy

    EXPECT_TRUE(holds(ir::RelativeSpeedCondition{ego_only(), "lead", longitudinal, Rule::EqualTo,
                                                 DirectionalDimension::Longitudinal},
                      context));
    EXPECT_TRUE(holds(ir::RelativeSpeedCondition{ego_only(), "lead", lateral, Rule::EqualTo,
                                                 DirectionalDimension::Lateral},
                      context));
    EXPECT_TRUE(holds(ir::RelativeSpeedCondition{ego_only(), "lead", 0.0, Rule::EqualTo,
                                                 DirectionalDimension::Vertical},
                      context));
}

// ---------------------------------------------------------------------------
// AccelerationCondition (§ AccelerationCondition)
// ---------------------------------------------------------------------------

TEST(AccelerationConditionTest, AbsentAccelerationIsFalseForEveryRule) {
    // Fewer than two samples: acceleration is absent ⇒ false for every rule,
    // including LessThan and NotEqualTo (the quantity does not exist).
    const KinematicsContext context{{{"ego", kin(5.0, 0.0, std::nullopt)}}};
    for (const Rule rule : {Rule::EqualTo, Rule::GreaterThan, Rule::LessThan, Rule::GreaterOrEqual,
                            Rule::LessOrEqual, Rule::NotEqualTo}) {
        EXPECT_FALSE(holds(ir::AccelerationCondition{ego_only(), 0.0, rule}, context)) << "rule";
    }
}

TEST(AccelerationConditionTest, PresentAccelerationCompares) {
    const KinematicsContext context{{{"ego", kin(5.0, 0.0, -3.0)}}};
    // Total ⇒ magnitude.
    EXPECT_TRUE(holds(ir::AccelerationCondition{ego_only(), 3.0, Rule::EqualTo}, context));
    // Longitudinal ⇒ signed (braking is negative).
    EXPECT_TRUE(holds(ir::AccelerationCondition{ego_only(), -3.0, Rule::EqualTo,
                                                DirectionalDimension::Longitudinal},
                      context));
    EXPECT_TRUE(holds(ir::AccelerationCondition{ego_only(), 0.0, Rule::LessThan,
                                                DirectionalDimension::Longitudinal},
                      context));
    // Lateral ⇒ 0.0.
    EXPECT_TRUE(holds(
        ir::AccelerationCondition{ego_only(), 0.0, Rule::EqualTo, DirectionalDimension::Lateral},
        context));
}

// ---------------------------------------------------------------------------
// StandStillCondition (§ StandStillCondition)
// ---------------------------------------------------------------------------

TEST(StandStillConditionTest, HoldsOnceRestTimeReachesDuration) {
    EntityKinematics almost = with_speed(0.0);
    almost.standstill_seconds = 1.5;
    EntityKinematics enough = with_speed(0.0);
    enough.standstill_seconds = 2.0;
    const KinematicsContext before{{{"ego", almost}}};
    const KinematicsContext after{{{"ego", enough}}};

    EXPECT_FALSE(holds(ir::StandStillCondition{ego_only(), 2.0}, before));
    EXPECT_TRUE(holds(ir::StandStillCondition{ego_only(), 2.0}, after)); // exact boundary >=
}

TEST(StandStillConditionTest, DurationZeroHoldsImmediatelyAtRest) {
    const KinematicsContext at_rest{{{"ego", with_speed(0.0)}}}; // standstill_seconds == 0
    EXPECT_TRUE(holds(ir::StandStillCondition{ego_only(), 0.0}, at_rest));
}

// ---------------------------------------------------------------------------
// TraveledDistanceCondition (§ TraveledDistanceCondition)
// ---------------------------------------------------------------------------

TEST(TraveledDistanceConditionTest, HoldsOnceOdometerReachesValue) {
    EntityKinematics near = with_speed(5.0);
    near.traveled_distance = 9.0;
    EntityKinematics reached = with_speed(5.0);
    reached.traveled_distance = 10.0;
    EXPECT_FALSE(
        holds(ir::TraveledDistanceCondition{ego_only(), 10.0}, KinematicsContext{{{"ego", near}}}));
    EXPECT_TRUE(holds(ir::TraveledDistanceCondition{ego_only(), 10.0},
                      KinematicsContext{{{"ego", reached}}}));
}

TEST(TraveledDistanceConditionTest, ValueZeroHoldsImmediately) {
    const KinematicsContext fresh{{{"ego", with_speed(0.0)}}}; // odometer 0
    EXPECT_TRUE(holds(ir::TraveledDistanceCondition{ego_only(), 0.0}, fresh));
}

// ---------------------------------------------------------------------------
// ReachPositionCondition (§ ReachPositionCondition, deprecated 1.2)
// ---------------------------------------------------------------------------

namespace {
EntityKinematics at_xy(double x, double y, double z = 0.0) {
    EntityKinematics k;
    k.state.x = x;
    k.state.y = y;
    k.state.z = z;
    return k;
}
} // namespace

TEST(ReachPositionConditionTest, InsideOutsideAndExactBoundary) {
    const ir::WorldPosition target{10.0, 0.0, 0.0};
    // Inside the 2 m circle (distance 1.5).
    EXPECT_TRUE(holds(ir::ReachPositionCondition{ego_only(), target, 2.0},
                      KinematicsContext{{{"ego", at_xy(11.5, 0.0)}}}));
    // Outside (distance 3).
    EXPECT_FALSE(holds(ir::ReachPositionCondition{ego_only(), target, 2.0},
                       KinematicsContext{{{"ego", at_xy(13.0, 0.0)}}}));
    // Exactly on the boundary: distance == tolerance ⇒ true (3-4-5 triangle,
    // distance exactly 5).
    EXPECT_TRUE(holds(ir::ReachPositionCondition{ego_only(), target, 5.0},
                      KinematicsContext{{{"ego", at_xy(13.0, 4.0)}}}));
}

TEST(ReachPositionConditionTest, DistanceIs2DAndIgnoresZ) {
    const ir::WorldPosition target{0.0, 0.0, 0.0};
    // A large z offset must not matter — the tolerance is a horizontal circle.
    EXPECT_TRUE(holds(ir::ReachPositionCondition{ego_only(), target, 0.5},
                      KinematicsContext{{{"ego", at_xy(0.0, 0.0, 100.0)}}}));
}

TEST(ReachPositionConditionTest, ToleranceZeroNeedsExactXy) {
    const ir::WorldPosition target{5.0, 5.0, 0.0};
    EXPECT_TRUE(holds(ir::ReachPositionCondition{ego_only(), target, 0.0},
                      KinematicsContext{{{"ego", at_xy(5.0, 5.0)}}}));
    EXPECT_FALSE(holds(ir::ReachPositionCondition{ego_only(), target, 0.0},
                       KinematicsContext{{{"ego", at_xy(5.0, 5.001)}}}));
}
