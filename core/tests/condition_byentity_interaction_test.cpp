// SPDX-License-Identifier: MIT
// Interaction conditions (ASAM OpenSCENARIO XML 1.4.0 §7.6.5.1, §6.4): the
// two-entity and entity-to-position metrics. This file covers the pure-context
// distance semantics (coordinate system × relative-distance type × freespace)
// and the init-time validation/diagnostics. TimeHeadway/TimeToCollision and
// the collision/road-deferred conditions extend it in later commits.
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include <gtest/gtest.h>

#include "scena/engine.h"
#include "scena/entity_state.h"
#include "scena/ir/action.h"
#include "scena/ir/bounding_box.h"
#include "scena/ir/condition.h"
#include "scena/ir/entity_condition.h"
#include "scena/ir/evaluation_context.h"
#include "scena/ir/interaction_condition.h"
#include "scena/ir/position.h"
#include "scena/ir/rule.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/trigger.h"
#include "scena/status.h"

namespace ir = scena::ir;
using ir::BoundingBox;
using ir::CollisionCondition;
using ir::CoordinateSystem;
using ir::DistanceCondition;
using ir::EndOfRoadCondition;
using ir::EntityKinematics;
using ir::OffroadCondition;
using ir::RelativeClearanceCondition;
using ir::RelativeDistanceCondition;
using ir::RelativeDistanceType;
using ir::RelativeLaneRange;
using ir::Rule;
using ir::TimeHeadwayCondition;
using ir::TimeToCollisionCondition;
using ir::TimeToCollisionTarget;
using ir::TriggeringEntities;
using ir::TriggeringEntitiesRule;
using ir::WorldPosition;
using scena::Engine;
using scena::EntityState;
using scena::Status;

namespace {

/// A context answering entity_kinematics from a fixed table (pure-unit driver).
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

TriggeringEntities ego_only() {
    return TriggeringEntities{TriggeringEntitiesRule::Any, {"ego"}};
}

/// Kinematics at a world pose, optionally with a body-frame bounding box.
EntityKinematics pose(double x, double y, double z = 0.0, double heading = 0.0,
                      std::optional<BoundingBox> box = std::nullopt) {
    EntityKinematics k;
    k.state.x = x;
    k.state.y = y;
    k.state.z = z;
    k.state.heading = heading;
    k.bounding_box = box;
    return k;
}

/// A box centered on the entity origin with the given half extents.
BoundingBox centered_box(double half_length, double half_width) {
    BoundingBox box;
    box.length = 2.0 * half_length;
    box.width = 2.0 * half_width;
    return box;
}

/// Kinematics at a pose with a speed along the heading (for headway/TTC).
EntityKinematics moving(double x, double y, double heading, double speed,
                        std::optional<BoundingBox> box = std::nullopt) {
    EntityKinematics k = pose(x, y, 0.0, heading, box);
    k.state.speed = speed;
    return k;
}

bool holds(const ir::Condition& condition, const ir::EvaluationContext& context) {
    return condition.evaluate(context);
}

// --- Engine-driven validation helpers ---------------------------------------

/// A scenario whose event fires on `condition`; validation diagnostics are
/// inspected via engine.diagnostics(). Entities: a host `ego`, optional host
/// `lead`, plus an engine-controlled probe.
ir::Scenario interaction_scenario(std::shared_ptr<ir::Condition> condition,
                                  bool with_lead = false) {
    ir::Scenario scenario;
    scenario.name = "interaction";
    for (const char* id : {"ego"}) {
        ir::Entity entity;
        entity.id = id;
        entity.name = id;
        entity.control_mode = ir::ControlMode::HostControlled;
        scenario.entities.push_back(std::move(entity));
    }
    if (with_lead) {
        ir::Entity lead;
        lead.id = "lead";
        lead.name = "lead";
        lead.control_mode = ir::ControlMode::HostControlled;
        scenario.entities.push_back(std::move(lead));
    }
    ir::Entity probe;
    probe.id = "probe";
    probe.name = "probe";
    probe.control_mode = ir::ControlMode::EngineControlled;
    scenario.entities.push_back(std::move(probe));

    ir::Event event;
    event.name = "event";
    event.start_trigger = ir::make_trigger(std::move(condition));
    event.actions.push_back(std::make_shared<ir::SpeedAction>("probe", 99.0));
    ir::Maneuver maneuver;
    maneuver.name = "maneuver";
    maneuver.events.push_back(std::move(event));
    ir::ManeuverGroup group;
    group.name = "group";
    group.maneuvers.push_back(std::move(maneuver));
    ir::Act act;
    act.name = "act";
    act.groups.push_back(std::move(group));
    ir::Story story;
    story.name = "story";
    story.acts.push_back(std::move(act));
    scenario.storyboard.stories.push_back(std::move(story));
    return scenario;
}

int diagnostic_count(const Engine& engine, scena::Severity severity, Status code) {
    int count = 0;
    for (const scena::Diagnostic& diagnostic : engine.diagnostics()) {
        if (diagnostic.severity == severity && diagnostic.code == code) {
            ++count;
        }
    }
    return count;
}

} // namespace

// ---------------------------------------------------------------------------
// DistanceCondition — entity to position (§ DistanceCondition, §6.4)
// ---------------------------------------------------------------------------

TEST(DistanceConditionTest, EuclideanReferencePointIs3D) {
    // Default coordinate system (entity) and default RDT (euclidian): the
    // reference-point distance is the 3D straight line (§6.4.3), independent of
    // heading. ego at origin, target at (3, 4, 12): distance 13.
    const KinematicsContext context{{{"ego", pose(0.0, 0.0, 0.0, 1.0)}}};
    const WorldPosition target{3.0, 4.0, 12.0};
    EXPECT_TRUE(holds(DistanceCondition{ego_only(), target, 13.0, false, Rule::EqualTo}, context));
    EXPECT_TRUE(
        holds(DistanceCondition{ego_only(), target, 13.0, false, Rule::LessOrEqual}, context));
    EXPECT_FALSE(
        holds(DistanceCondition{ego_only(), target, 12.9, false, Rule::LessOrEqual}, context));
}

TEST(DistanceConditionTest, EntityLongitudinalAndLateralProject) {
    // Entity CS, heading 0 (body x̂ = world x): longitudinal = |Δx|, lateral =
    // |Δy| (§6.4.4).
    const KinematicsContext context{{{"ego", pose(0.0, 0.0)}}};
    const WorldPosition target{3.0, 4.0, 0.0};
    EXPECT_TRUE(
        holds(DistanceCondition{ego_only(), target, 3.0, false, Rule::EqualTo,
                                CoordinateSystem::Entity, RelativeDistanceType::Longitudinal},
              context));
    EXPECT_TRUE(holds(DistanceCondition{ego_only(), target, 4.0, false, Rule::EqualTo,
                                        CoordinateSystem::Entity, RelativeDistanceType::Lateral},
                      context));
}

TEST(DistanceConditionTest, WorldLongitudinalIgnoresHeading) {
    // World CS longitudinal is |Δx| on the world axis regardless of the
    // entity's heading, unlike the Entity CS which rotates with it.
    const KinematicsContext context{{{"ego", pose(0.0, 0.0, 0.0, 1.0)}}};
    const WorldPosition target{3.0, 4.0, 0.0};
    EXPECT_TRUE(
        holds(DistanceCondition{ego_only(), target, 3.0, false, Rule::EqualTo,
                                CoordinateSystem::World, RelativeDistanceType::Longitudinal},
              context));
    EXPECT_TRUE(holds(DistanceCondition{ego_only(), target, 4.0, false, Rule::EqualTo,
                                        CoordinateSystem::World, RelativeDistanceType::Lateral},
                      context));
}

TEST(DistanceConditionTest, FreespaceEuclideanUsesBoundingBox) {
    // ego is a 2x2 box at the origin; the target point at (3, 0) is 2 m from
    // the box surface (§6.4.7.2).
    const KinematicsContext context{{{"ego", pose(0.0, 0.0, 0.0, 0.0, centered_box(1.0, 1.0))}}};
    const WorldPosition target{3.0, 0.0, 0.0};
    EXPECT_TRUE(holds(DistanceCondition{ego_only(), target, 2.0, true, Rule::EqualTo}, context));
}

TEST(DistanceConditionTest, FreespaceLongitudinalIsAxisGap) {
    const KinematicsContext context{{{"ego", pose(0.0, 0.0, 0.0, 0.0, centered_box(1.0, 1.0))}}};
    const WorldPosition target{3.0, 0.0, 0.0};
    EXPECT_TRUE(
        holds(DistanceCondition{ego_only(), target, 2.0, true, Rule::EqualTo,
                                CoordinateSystem::Entity, RelativeDistanceType::Longitudinal},
              context));
}

TEST(DistanceConditionTest, FreespaceWithoutBoundingBoxIsFalse) {
    // freespace requested but ego has no geometry ⇒ per-entity false, silently.
    const KinematicsContext context{{{"ego", pose(0.0, 0.0)}}};
    const WorldPosition target{3.0, 0.0, 0.0};
    EXPECT_FALSE(
        holds(DistanceCondition{ego_only(), target, 0.0, true, Rule::GreaterOrEqual}, context));
}

TEST(DistanceConditionTest, RoadCoordinateSystemEvaluatesFalse) {
    // Road CS is deferred (no road network): the metric is a deterministic
    // false even for a rule a real distance would satisfy.
    const KinematicsContext context{{{"ego", pose(0.0, 0.0)}}};
    const WorldPosition target{3.0, 4.0, 0.0};
    EXPECT_FALSE(holds(DistanceCondition{ego_only(), target, 0.0, false, Rule::GreaterOrEqual,
                                         CoordinateSystem::Road},
                       context));
}

TEST(DistanceConditionTest, CartesianDistanceIsTreatedAsEuclidean) {
    const KinematicsContext context{{{"ego", pose(0.0, 0.0)}}};
    const WorldPosition target{3.0, 4.0, 0.0};
    EXPECT_TRUE(holds(DistanceCondition{ego_only(), target, 5.0, false, Rule::EqualTo, std::nullopt,
                                        RelativeDistanceType::CartesianDistance},
                      context));
}

TEST(DistanceConditionTest, EffectiveCoordinateSystemHonorsAlongRouteRule) {
    // Neither CS nor RDT authored, alongRoute true ⇒ effective CS is Road.
    const DistanceCondition promoted{ego_only(),   WorldPosition{}, 1.0,
                                     false,        Rule::LessThan,  std::nullopt,
                                     std::nullopt, std::nullopt,    true};
    EXPECT_EQ(promoted.effective_coordinate_system(), CoordinateSystem::Road);
    // alongRoute is ignored once relativeDistanceType is set ⇒ default Entity.
    const DistanceCondition ignored{ego_only(),
                                    WorldPosition{},
                                    1.0,
                                    false,
                                    Rule::LessThan,
                                    std::nullopt,
                                    RelativeDistanceType::Longitudinal,
                                    std::nullopt,
                                    true};
    EXPECT_EQ(ignored.effective_coordinate_system(), CoordinateSystem::Entity);
}

// ---------------------------------------------------------------------------
// RelativeDistanceCondition — entity to entity (§ RelativeDistanceCondition)
// ---------------------------------------------------------------------------

TEST(RelativeDistanceConditionTest, EuclideanBetweenEntities) {
    const KinematicsContext context{{{"ego", pose(0.0, 0.0)}, {"lead", pose(6.0, 8.0)}}};
    EXPECT_TRUE(
        holds(RelativeDistanceCondition{ego_only(), "lead", 10.0, false,
                                        RelativeDistanceType::EuclidianDistance, Rule::EqualTo},
              context));
}

TEST(RelativeDistanceConditionTest, LongitudinalProjection) {
    const KinematicsContext context{{{"ego", pose(0.0, 0.0)}, {"lead", pose(6.0, 8.0)}}};
    EXPECT_TRUE(holds(RelativeDistanceCondition{ego_only(), "lead", 6.0, false,
                                                RelativeDistanceType::Longitudinal, Rule::EqualTo},
                      context));
}

TEST(RelativeDistanceConditionTest, FreespaceUsesBothBoundingBoxes) {
    const KinematicsContext context{{{"ego", pose(0.0, 0.0, 0.0, 0.0, centered_box(1.0, 1.0))},
                                     {"lead", pose(5.0, 0.0, 0.0, 0.0, centered_box(1.0, 1.0))}}};
    // Gap between the two unit boxes 5 m apart: 5 - 1 - 1 = 3.
    EXPECT_TRUE(
        holds(RelativeDistanceCondition{ego_only(), "lead", 3.0, true,
                                        RelativeDistanceType::EuclidianDistance, Rule::EqualTo},
              context));
}

TEST(RelativeDistanceConditionTest, AbsentReferenceIsFalse) {
    const KinematicsContext context{{{"ego", pose(0.0, 0.0)}}}; // no "lead"
    EXPECT_FALSE(holds(RelativeDistanceCondition{ego_only(), "lead", 0.0, false,
                                                 RelativeDistanceType::EuclidianDistance,
                                                 Rule::GreaterOrEqual},
                       context));
}

// ---------------------------------------------------------------------------
// TimeHeadwayCondition (§ TimeHeadwayCondition)
// ---------------------------------------------------------------------------

TEST(TimeHeadwayConditionTest, DistanceOverTriggeringSpeed) {
    // ego at 5 m/s, lead 10 m ahead: headway = 10 / 5 = 2 s. Only the
    // triggering entity's speed matters, so the lead's motion is irrelevant.
    const KinematicsContext context{
        {{"ego", moving(0.0, 0.0, 0.0, 5.0)}, {"lead", pose(10.0, 0.0)}}};
    EXPECT_TRUE(
        holds(TimeHeadwayCondition{ego_only(), "lead", 2.0, false, Rule::EqualTo}, context));
    EXPECT_TRUE(
        holds(TimeHeadwayCondition{ego_only(), "lead", 1.0, false, Rule::GreaterThan}, context));
}

TEST(TimeHeadwayConditionTest, StoppedOrReversingFollowerIsFalse) {
    const KinematicsContext stopped{
        {{"ego", moving(0.0, 0.0, 0.0, 0.0)}, {"lead", pose(10.0, 0.0)}}};
    EXPECT_FALSE(
        holds(TimeHeadwayCondition{ego_only(), "lead", 100.0, false, Rule::LessThan}, stopped));
    const KinematicsContext reversing{
        {{"ego", moving(0.0, 0.0, 0.0, -5.0)}, {"lead", pose(10.0, 0.0)}}};
    EXPECT_FALSE(
        holds(TimeHeadwayCondition{ego_only(), "lead", 100.0, false, Rule::LessThan}, reversing));
}

// ---------------------------------------------------------------------------
// TimeToCollisionCondition (§ TimeToCollisionCondition)
// ---------------------------------------------------------------------------

TEST(TimeToCollisionConditionTest, ApproachingEntityHasFiniteTtc) {
    // ego at 10 m/s toward a stationary lead 20 m ahead: closing speed 10,
    // distance 20, TTC = 2 s.
    const KinematicsContext context{
        {{"ego", moving(0.0, 0.0, 0.0, 10.0)}, {"lead", moving(20.0, 0.0, 0.0, 0.0)}}};
    EXPECT_TRUE(
        holds(TimeToCollisionCondition{ego_only(), TimeToCollisionTarget{std::string{"lead"}}, 2.0,
                                       false, Rule::EqualTo},
              context));
}

TEST(TimeToCollisionConditionTest, MovingApartIsFalse) {
    // Negative closing speed ⇒ no predicted collision (§ TimeToCollisionCondition).
    const KinematicsContext context{
        {{"ego", moving(0.0, 0.0, 0.0, -10.0)}, {"lead", moving(20.0, 0.0, 0.0, 0.0)}}};
    EXPECT_FALSE(
        holds(TimeToCollisionCondition{ego_only(), TimeToCollisionTarget{std::string{"lead"}},
                                       100.0, false, Rule::LessThan},
              context));
}

TEST(TimeToCollisionConditionTest, BothEntitiesClosingSumsRelativeSpeed) {
    // ego +5 m/s, lead reversing at -5 m/s (toward ego): closing speed 10,
    // distance 20, TTC = 2 s.
    const KinematicsContext context{
        {{"ego", moving(0.0, 0.0, 0.0, 5.0)}, {"lead", moving(20.0, 0.0, 0.0, -5.0)}}};
    EXPECT_TRUE(
        holds(TimeToCollisionCondition{ego_only(), TimeToCollisionTarget{std::string{"lead"}}, 2.0,
                                       false, Rule::EqualTo},
              context));
}

TEST(TimeToCollisionConditionTest, PositionTargetIsStationary) {
    const KinematicsContext context{{{"ego", moving(0.0, 0.0, 0.0, 10.0)}}};
    EXPECT_TRUE(holds(TimeToCollisionCondition{ego_only(),
                                               TimeToCollisionTarget{WorldPosition{20.0, 0.0, 0.0}},
                                               2.0, false, Rule::EqualTo},
                      context));
}

TEST(TimeToCollisionConditionTest, CoincidentReferencePointIsFalse) {
    const KinematicsContext context{{{"ego", moving(0.0, 0.0, 0.0, 10.0)}}};
    EXPECT_FALSE(holds(TimeToCollisionCondition{ego_only(),
                                                TimeToCollisionTarget{WorldPosition{0.0, 0.0, 0.0}},
                                                100.0, false, Rule::GreaterOrEqual},
                       context));
}

TEST(TimeToCollisionConditionTest, LongitudinalClosingSpeed) {
    // Longitudinal RDT, Entity CS: distance = |Δx| = 20, closing speed along
    // x̂ = 10, TTC = 2. The lead's lateral offset does not change the axis TTC.
    const KinematicsContext context{
        {{"ego", moving(0.0, 0.0, 0.0, 10.0)}, {"lead", moving(20.0, 5.0, 0.0, 0.0)}}};
    EXPECT_TRUE(
        holds(TimeToCollisionCondition{ego_only(), TimeToCollisionTarget{std::string{"lead"}}, 2.0,
                                       false, Rule::EqualTo, CoordinateSystem::Entity,
                                       RelativeDistanceType::Longitudinal},
              context));
}

// ---------------------------------------------------------------------------
// CollisionCondition (§ CollisionCondition, §6.4.7.2)
// ---------------------------------------------------------------------------

TEST(CollisionConditionTest, IntersectingBoxesCollide) {
    const KinematicsContext context{{{"ego", pose(0.0, 0.0, 0.0, 0.0, centered_box(1.0, 1.0))},
                                     {"lead", pose(1.5, 0.0, 0.0, 0.0, centered_box(1.0, 1.0))}}};
    EXPECT_TRUE(holds(CollisionCondition{ego_only(), "lead"}, context));
}

TEST(CollisionConditionTest, TouchingBoxesCollide) {
    // Exactly touching (centers 2 apart, unit boxes): freespace distance 0 ⇒
    // collision (§6.4.7.2).
    const KinematicsContext context{{{"ego", pose(0.0, 0.0, 0.0, 0.0, centered_box(1.0, 1.0))},
                                     {"lead", pose(2.0, 0.0, 0.0, 0.0, centered_box(1.0, 1.0))}}};
    EXPECT_TRUE(holds(CollisionCondition{ego_only(), "lead"}, context));
}

TEST(CollisionConditionTest, DisjointBoxesDoNotCollide) {
    const KinematicsContext context{{{"ego", pose(0.0, 0.0, 0.0, 0.0, centered_box(1.0, 1.0))},
                                     {"lead", pose(3.0, 0.0, 0.0, 0.0, centered_box(1.0, 1.0))}}};
    EXPECT_FALSE(holds(CollisionCondition{ego_only(), "lead"}, context));
}

TEST(CollisionConditionTest, MissingBoxOrReferenceIsFalse) {
    // Overlapping positions but no geometry on the reference ⇒ false.
    const KinematicsContext no_ref_box{
        {{"ego", pose(0.0, 0.0, 0.0, 0.0, centered_box(1.0, 1.0))}, {"lead", pose(0.0, 0.0)}}};
    EXPECT_FALSE(holds(CollisionCondition{ego_only(), "lead"}, no_ref_box));
    // No trigger geometry ⇒ false.
    const KinematicsContext no_trigger_box{
        {{"ego", pose(0.0, 0.0)}, {"lead", pose(0.0, 0.0, 0.0, 0.0, centered_box(1.0, 1.0))}}};
    EXPECT_FALSE(holds(CollisionCondition{ego_only(), "lead"}, no_trigger_box));
    // Absent reference entirely ⇒ false.
    const KinematicsContext no_ref{{{"ego", pose(0.0, 0.0, 0.0, 0.0, centered_box(1.0, 1.0))}}};
    EXPECT_FALSE(holds(CollisionCondition{ego_only(), "lead"}, no_ref));
}

TEST(CollisionConditionTest, AllReductionRequiresEveryTriggeringEntityToCollide) {
    // Two triggering entities against "lead": under All, both must collide.
    const KinematicsContext context{{{"a", pose(0.0, 0.0, 0.0, 0.0, centered_box(1.0, 1.0))},
                                     {"b", pose(10.0, 0.0, 0.0, 0.0, centered_box(1.0, 1.0))},
                                     {"lead", pose(0.5, 0.0, 0.0, 0.0, centered_box(1.0, 1.0))}}};
    const TriggeringEntities both_all{TriggeringEntitiesRule::All, {"a", "b"}};
    const TriggeringEntities both_any{TriggeringEntitiesRule::Any, {"a", "b"}};
    EXPECT_FALSE(holds(CollisionCondition{both_all, "lead"}, context)); // only "a" overlaps
    EXPECT_TRUE(holds(CollisionCondition{both_any, "lead"}, context));
}

// ---------------------------------------------------------------------------
// Road-deferred conditions: deterministic false until p3-s4 (ADR-0009)
// ---------------------------------------------------------------------------

TEST(RoadDeferredConditionTest, EndOfRoadOffroadAndClearanceEvaluateFalse) {
    const KinematicsContext context{
        {{"ego", moving(0.0, 0.0, 0.0, 10.0)}, {"lead", pose(1.0, 0.0)}}};
    EXPECT_FALSE(holds(EndOfRoadCondition{ego_only(), 0.0}, context));
    EXPECT_FALSE(holds(OffroadCondition{ego_only(), 0.0}, context));
    EXPECT_FALSE(holds(RelativeClearanceCondition{ego_only(), true, false}, context));
    // Even with all the optional fields populated, still false.
    EXPECT_FALSE(holds(
        RelativeClearanceCondition{
            ego_only(), false, true, 5.0, 10.0, {"lead"}, {RelativeLaneRange{-1, 1}}},
        context));
}

// ---------------------------------------------------------------------------
// Validation (engine.cpp validate_condition_expression)
// ---------------------------------------------------------------------------

TEST(InteractionValidationTest, NegativeDistanceValueFailsInit) {
    Engine engine;
    EXPECT_EQ(engine.init(interaction_scenario(std::make_shared<DistanceCondition>(
                  ego_only(), WorldPosition{}, -1.0, false, Rule::LessThan))),
              Status::ValidationError);
}

TEST(InteractionValidationTest, NanDistanceValueOrPositionFailsInit) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    {
        Engine engine;
        EXPECT_EQ(engine.init(interaction_scenario(std::make_shared<DistanceCondition>(
                      ego_only(), WorldPosition{}, nan, false, Rule::LessThan))),
                  Status::ValidationError);
    }
    {
        Engine engine;
        EXPECT_EQ(engine.init(interaction_scenario(std::make_shared<DistanceCondition>(
                      ego_only(), WorldPosition{nan, 0.0, 0.0}, 1.0, false, Rule::LessThan))),
                  Status::ValidationError);
    }
}

TEST(InteractionValidationTest, RelativeDistanceUnknownReferenceFailsInit) {
    Engine engine;
    EXPECT_EQ(engine.init(interaction_scenario(std::make_shared<RelativeDistanceCondition>(
                  ego_only(), "ghost", 1.0, false, RelativeDistanceType::EuclidianDistance,
                  Rule::LessThan))),
              Status::SemanticError);
}

TEST(InteractionValidationTest, RoadCoordinateSystemWarnsButInitSucceeds) {
    Engine engine;
    ASSERT_EQ(
        engine.init(interaction_scenario(std::make_shared<DistanceCondition>(
            ego_only(), WorldPosition{}, 1.0, false, Rule::LessThan, CoordinateSystem::Road))),
        Status::Ok);
    EXPECT_EQ(diagnostic_count(engine, scena::Severity::Warning, Status::UnsupportedFeature), 1);
}

TEST(InteractionValidationTest, AlongRouteDeprecationWarns) {
    Engine engine;
    // alongRoute authored (false, so no Road promotion): one deprecation warning.
    ASSERT_EQ(engine.init(interaction_scenario(std::make_shared<DistanceCondition>(
                  ego_only(), WorldPosition{}, 1.0, false, Rule::LessThan, std::nullopt,
                  std::nullopt, std::nullopt, false))),
              Status::Ok);
    EXPECT_EQ(diagnostic_count(engine, scena::Severity::Warning, Status::DeprecatedFeature), 1);
    EXPECT_EQ(diagnostic_count(engine, scena::Severity::Warning, Status::UnsupportedFeature), 0);
}

TEST(InteractionValidationTest, AlongRouteTrueWithoutModesPromotesToRoad) {
    Engine engine;
    // alongRoute true and no CS/RDT ⇒ effective Road ⇒ both a deprecation and
    // an unsupported-feature warning.
    ASSERT_EQ(engine.init(interaction_scenario(std::make_shared<DistanceCondition>(
                  ego_only(), WorldPosition{}, 1.0, false, Rule::LessThan, std::nullopt,
                  std::nullopt, std::nullopt, true))),
              Status::Ok);
    EXPECT_EQ(diagnostic_count(engine, scena::Severity::Warning, Status::DeprecatedFeature), 1);
    EXPECT_EQ(diagnostic_count(engine, scena::Severity::Warning, Status::UnsupportedFeature), 1);
}

TEST(InteractionValidationTest, CartesianDistanceDeprecationWarns) {
    Engine engine;
    ASSERT_EQ(engine.init(interaction_scenario(std::make_shared<DistanceCondition>(
                  ego_only(), WorldPosition{}, 1.0, false, Rule::LessThan, std::nullopt,
                  RelativeDistanceType::CartesianDistance))),
              Status::Ok);
    EXPECT_EQ(diagnostic_count(engine, scena::Severity::Warning, Status::DeprecatedFeature), 1);
}

TEST(InteractionValidationTest, NegativeHeadwayOrTtcValueFailsInit) {
    {
        Engine engine;
        EXPECT_EQ(
            engine.init(interaction_scenario(std::make_shared<TimeHeadwayCondition>(
                                                 ego_only(), "lead", -1.0, false, Rule::LessThan),
                                             /*with_lead=*/true)),
            Status::ValidationError);
    }
    {
        Engine engine;
        EXPECT_EQ(
            engine.init(interaction_scenario(std::make_shared<TimeToCollisionCondition>(
                ego_only(), TimeToCollisionTarget{WorldPosition{}}, -1.0, false, Rule::LessThan))),
            Status::ValidationError);
    }
}

TEST(InteractionValidationTest, HeadwayUnknownReferenceFailsInit) {
    Engine engine;
    EXPECT_EQ(engine.init(interaction_scenario(std::make_shared<TimeHeadwayCondition>(
                  ego_only(), "ghost", 1.0, false, Rule::LessThan))),
              Status::SemanticError);
}

TEST(InteractionValidationTest, TtcUnknownEntityTargetFailsInit) {
    Engine engine;
    EXPECT_EQ(
        engine.init(interaction_scenario(std::make_shared<TimeToCollisionCondition>(
            ego_only(), TimeToCollisionTarget{std::string{"ghost"}}, 1.0, false, Rule::LessThan))),
        Status::SemanticError);
}

TEST(InteractionValidationTest, TtcNanPositionTargetFailsInit) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    Engine engine;
    EXPECT_EQ(engine.init(interaction_scenario(std::make_shared<TimeToCollisionCondition>(
                  ego_only(), TimeToCollisionTarget{WorldPosition{nan, 0.0, 0.0}}, 1.0, false,
                  Rule::LessThan))),
              Status::ValidationError);
}

TEST(InteractionValidationTest, CollisionUnknownReferenceFailsInit) {
    Engine engine;
    EXPECT_EQ(engine.init(
                  interaction_scenario(std::make_shared<CollisionCondition>(ego_only(), "ghost"))),
              Status::SemanticError);
}

TEST(InteractionValidationTest, EndOfRoadAndOffroadWarnUnsupportedButInitSucceeds) {
    {
        Engine engine;
        ASSERT_EQ(engine.init(
                      interaction_scenario(std::make_shared<EndOfRoadCondition>(ego_only(), 1.0))),
                  Status::Ok);
        EXPECT_EQ(diagnostic_count(engine, scena::Severity::Warning, Status::UnsupportedFeature),
                  1);
    }
    {
        Engine engine;
        ASSERT_EQ(
            engine.init(interaction_scenario(std::make_shared<OffroadCondition>(ego_only(), 1.0))),
            Status::Ok);
        EXPECT_EQ(diagnostic_count(engine, scena::Severity::Warning, Status::UnsupportedFeature),
                  1);
    }
}

TEST(InteractionValidationTest, EndOfRoadNegativeDurationFailsInit) {
    Engine engine;
    EXPECT_EQ(
        engine.init(interaction_scenario(std::make_shared<EndOfRoadCondition>(ego_only(), -1.0))),
        Status::ValidationError);
}

TEST(InteractionValidationTest, RelativeClearanceWarnsUnsupportedButInitSucceeds) {
    Engine engine;
    ASSERT_EQ(engine.init(interaction_scenario(
                  std::make_shared<RelativeClearanceCondition>(ego_only(), true, false))),
              Status::Ok);
    EXPECT_EQ(diagnostic_count(engine, scena::Severity::Warning, Status::UnsupportedFeature), 1);
}

TEST(InteractionValidationTest, RelativeClearanceNegativeDistanceFailsInit) {
    Engine engine;
    EXPECT_EQ(engine.init(interaction_scenario(std::make_shared<RelativeClearanceCondition>(
                  ego_only(), true, false, -1.0, 0.0))),
              Status::ValidationError);
}

TEST(InteractionValidationTest, RelativeClearanceInvertedLaneRangeFailsInit) {
    Engine engine;
    EXPECT_EQ(engine.init(interaction_scenario(std::make_shared<RelativeClearanceCondition>(
                  ego_only(), true, false, 0.0, 0.0, std::vector<std::string>{},
                  std::vector<RelativeLaneRange>{RelativeLaneRange{2, -2}}))),
              Status::ValidationError);
}

TEST(InteractionValidationTest, RelativeClearanceUnknownEntityFailsInit) {
    Engine engine;
    EXPECT_EQ(engine.init(interaction_scenario(std::make_shared<RelativeClearanceCondition>(
                  ego_only(), true, false, 0.0, 0.0, std::vector<std::string>{"ghost"}))),
              Status::SemanticError);
}
