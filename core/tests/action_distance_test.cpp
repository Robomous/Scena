// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
//
// LongitudinalDistanceAction at the engine level (p5-s5): the deadbeat
// distance-keeping controller of ADR-0014 — distance and timeGap targets,
// freespace gaps, DynamicConstraints/Performance clamping, displacement,
// supersession against the other longitudinal actions, the never-ending
// continuous mode, and the validation diagnostics. The GS-4 golden body lives
// at the end of the file.

#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "scena/diagnostic.h"
#include "scena/engine.h"
#include "scena/ir/action.h"
#include "scena/ir/bounding_box.h"
#include "scena/ir/condition.h"
#include "scena/ir/coordinate_system.h"
#include "scena/ir/dynamics.h"
#include "scena/ir/entity.h"
#include "scena/ir/entity_types.h"
#include "scena/ir/interaction_condition.h"
#include "scena/ir/position.h"
#include "scena/ir/rule.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/trigger.h"

namespace {

using scena::Engine;
using scena::EntityState;
using scena::Severity;
using scena::Status;
using scena::ir::BoundingBox;
using scena::ir::ControlMode;
using scena::ir::CoordinateSystem;
using scena::ir::DynamicConstraints;
using scena::ir::DynamicsDimension;
using scena::ir::DynamicsShape;
using scena::ir::Entity;
using scena::ir::LongitudinalDisplacement;
using scena::ir::LongitudinalDistanceAction;
using scena::ir::Performance;
using scena::ir::Scenario;
using scena::ir::SimulationTimeCondition;
using scena::ir::SpeedAction;
using scena::ir::TeleportAction;
using scena::ir::TransitionDynamics;
using scena::ir::Vehicle;
using scena::ir::WorldPosition;

constexpr double kTol = 1e-9;

// --- Scenario scaffolding -------------------------------------------------

scena::ir::Event timed_event(const std::string& name, double at_time,
                             std::shared_ptr<scena::ir::Action> action) {
    scena::ir::Event event;
    event.name = name;
    event.start_trigger = scena::ir::make_trigger(
        std::make_shared<SimulationTimeCondition>(at_time), scena::ir::ConditionEdge::None, 0.0);
    event.actions.push_back(std::move(action));
    return event;
}

/// A plain (unclassified) engine-controlled entity: no bounding box, no
/// Performance envelope, so nothing clamps the distance controller.
Entity plain_entity(const std::string& id) {
    Entity entity;
    entity.id = id;
    entity.name = id;
    entity.control_mode = ControlMode::EngineControlled;
    return entity;
}

/// An engine-controlled vehicle with a `length` x 2 m box centred on its
/// reference point and the given Performance envelope.
Entity vehicle_entity(const std::string& id, double length,
                      std::optional<Performance> performance = std::nullopt) {
    Entity entity = plain_entity(id);
    Vehicle vehicle;
    vehicle.bounding_box = BoundingBox{0.0, 0.0, 0.75, length, 2.0, 1.5};
    vehicle.performance =
        performance.value_or(Performance{60.0, 5.0, 5.0, std::nullopt, std::nullopt});
    entity.object = vehicle;
    return entity;
}

/// Two entities on the +x axis (`lead` ahead of `ego`) at their given speeds,
/// plus one maneuver holding `events` in document order. The lead is teleported
/// to `lead_x`, the ego to the origin; both head along +x.
Scenario make_following_scenario(Entity ego, Entity lead, double lead_x, double ego_speed,
                                 double lead_speed, std::vector<scena::ir::Event> events) {
    Scenario scenario;
    scenario.name = "distance-keeping";
    scenario.entities.push_back(std::move(lead));
    scenario.entities.push_back(std::move(ego));
    scenario.init_actions.push_back(
        std::make_shared<TeleportAction>("lead", WorldPosition{lead_x, 0.0, 0.0}));
    scenario.init_actions.push_back(
        std::make_shared<TeleportAction>("ego", WorldPosition{0.0, 0.0, 0.0}));
    scenario.init_actions.push_back(std::make_shared<SpeedAction>("lead", lead_speed));
    scenario.init_actions.push_back(std::make_shared<SpeedAction>("ego", ego_speed));

    scena::ir::Maneuver maneuver;
    maneuver.name = "maneuver";
    maneuver.events = std::move(events);
    scena::ir::ManeuverGroup group;
    group.name = "group";
    group.maneuvers.push_back(std::move(maneuver));
    scena::ir::Act act;
    act.name = "act";
    act.groups.push_back(std::move(group));
    scena::ir::Story story;
    story.name = "story";
    story.acts.push_back(std::move(act));
    scenario.storyboard.stories.push_back(std::move(story));
    return scenario;
}

/// Reference-point gap from ego to lead along +x.
double gap_of(const Engine& engine) {
    return engine.state("lead")->x - engine.state("ego")->x;
}

// --- Distance and time-gap targets ----------------------------------------

TEST(DistanceKeepingTest, ReachesAnAbsoluteDistanceTargetInOneUnclampedStep) {
    // Unconstrained entities: the deadbeat command closes the whole error over
    // one step, so the gap is exactly on target at the next measurement and the
    // action ends there ("by reaching the targeted distance", Table 10).
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event("keep", 0.0,
                                 std::make_shared<LongitudinalDistanceAction>(
                                     "ego", "lead", 50.0, std::nullopt, /*freespace=*/false,
                                     /*continuous=*/false)));
    Engine engine;
    ASSERT_EQ(engine.init(make_following_scenario(plain_entity("ego"), plain_entity("lead"), 100.0,
                                                  10.0, 10.0, std::move(events))),
              Status::Ok);
    EXPECT_NEAR(gap_of(engine), 100.0, kTol);

    // v_cmd = v_lead + (gap - target)/dt = 10 + 50/1.
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_NEAR(engine.state("ego")->speed, 60.0, kTol);
    EXPECT_NEAR(gap_of(engine), 50.0, kTol);
    EXPECT_EQ(*engine.storyboard_element_state("story/act/group/maneuver/keep"),
              scena::runtime::ElementState::Running);

    // The next re-poll finds the gap on target and ends the action.
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(*engine.storyboard_element_state("story/act/group/maneuver/keep"),
              scena::runtime::ElementState::Complete);
    // Ownership released: the entity keeps its last commanded speed.
    EXPECT_NEAR(engine.state("ego")->speed, 60.0, kTol);
}

TEST(DistanceKeepingTest, CompletesImmediatelyWhenTheGapAlreadyHolds) {
    // §7.5.2.1: "An action ends regularly if its goal is accomplished upon the
    // start of the action."
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event("keep", 1.0,
                                 std::make_shared<LongitudinalDistanceAction>(
                                     "ego", "lead", 40.0, std::nullopt, /*freespace=*/false,
                                     /*continuous=*/false)));
    Engine engine;
    ASSERT_EQ(engine.init(make_following_scenario(plain_entity("ego"), plain_entity("lead"), 40.0,
                                                  10.0, 10.0, std::move(events))),
              Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok); // both moved 10 m: the gap is still 40
    EXPECT_EQ(*engine.storyboard_element_state("story/act/group/maneuver/keep"),
              scena::runtime::ElementState::Complete);
    EXPECT_NEAR(engine.state("ego")->speed, 10.0, kTol); // never commanded
}

TEST(DistanceKeepingTest, TimeGapTargetScalesWithTheActorSpeed) {
    // Headway arithmetic: target = timeGap * own speed = 2 s * 10 m/s = 20 m.
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event("keep", 0.0,
                                 std::make_shared<LongitudinalDistanceAction>(
                                     "ego", "lead", std::nullopt, 2.0, /*freespace=*/false,
                                     /*continuous=*/true)));
    Engine engine;
    ASSERT_EQ(engine.init(make_following_scenario(plain_entity("ego"), plain_entity("lead"), 50.0,
                                                  10.0, 10.0, std::move(events))),
              Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    // v_cmd = 10 + (50 - 20)/1 = 40, so the gap closes to 20 m this step.
    EXPECT_NEAR(engine.state("ego")->speed, 40.0, kTol);
    EXPECT_NEAR(gap_of(engine), 20.0, kTol);

    // Now the target follows the new speed (2 s * 40 m/s = 80 m): the actor is
    // far too close and falls back.
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_LT(engine.state("ego")->speed, 40.0);
}

TEST(DistanceKeepingTest, FreespaceGapIsMeasuredBumperToBumper) {
    // Boxes are 6 m and 4 m long, centred on the reference points, so the
    // freespace gap is the reference-point gap minus 3 m minus 2 m.
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event("keep", 0.0,
                                 std::make_shared<LongitudinalDistanceAction>(
                                     "ego", "lead", 10.0, std::nullopt, /*freespace=*/true,
                                     /*continuous=*/false)));
    Engine engine;
    ASSERT_EQ(
        engine.init(make_following_scenario(vehicle_entity("ego", 4.0), vehicle_entity("lead", 6.0),
                                            60.0, 10.0, 10.0, std::move(events))),
        Status::Ok);
    // Both vehicles carry a Performance envelope, so the approach is
    // acceleration-limited and the glide law brings the gap in over ~7.5 s.
    for (int i = 0; i < 15; ++i) {
        SCOPED_TRACE(i);
        ASSERT_EQ(engine.step(0.5), Status::Ok);
        // The boxes never touch on the way in (6 m and 4 m long ⇒ 5 m of
        // reference-point gap is bumper contact).
        EXPECT_GT(gap_of(engine), 5.0);
    }
    // Bumper-to-bumper 10 m ⇒ reference-point gap 10 + 2 + 3 = 15 m.
    EXPECT_NEAR(gap_of(engine), 15.0, 1e-6);
    // The next re-poll observes the on-target gap and ends the action.
    ASSERT_EQ(engine.step(0.5), Status::Ok);
    EXPECT_EQ(*engine.storyboard_element_state("story/act/group/maneuver/keep"),
              scena::runtime::ElementState::Complete);
}

TEST(DistanceKeepingTest, LeadingDisplacementHoldsTheActorAheadOfTheReference) {
    // displacement=leadingReferencedEntity ⇒ the actor stays ahead, so the
    // signed gap converges to -25 m (the reference is behind).
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event("keep", 0.0,
                                 std::make_shared<LongitudinalDistanceAction>(
                                     "ego", "lead", 25.0, std::nullopt, /*freespace=*/false,
                                     /*continuous=*/true, CoordinateSystem::Entity,
                                     LongitudinalDisplacement::LeadingReferencedEntity)));
    Engine engine;
    // The ego starts 10 m ahead of the lead (lead_x is negative).
    ASSERT_EQ(engine.init(make_following_scenario(plain_entity("ego"), plain_entity("lead"), -10.0,
                                                  10.0, 10.0, std::move(events))),
              Status::Ok);
    for (int i = 0; i < 4; ++i) {
        SCOPED_TRACE(i);
        ASSERT_EQ(engine.step(0.5), Status::Ok);
    }
    EXPECT_NEAR(gap_of(engine), -25.0, 1e-6);
    EXPECT_GT(engine.state("ego")->speed, 0.0);
}

// --- Clamping -------------------------------------------------------------

TEST(DistanceKeepingTest, DynamicConstraintsLimitTheApproach) {
    // maxAcceleration 2 m/s^2 over 1 s steps: the command may rise by at most
    // 2 m/s per step no matter how large the error is.
    DynamicConstraints constraints;
    constraints.max_acceleration = 2.0;
    constraints.max_deceleration = 3.0;
    std::vector<scena::ir::Event> events;
    events.push_back(
        timed_event("keep", 0.0,
                    std::make_shared<LongitudinalDistanceAction>(
                        "ego", "lead", 20.0, std::nullopt, /*freespace=*/false, /*continuous=*/true,
                        CoordinateSystem::Entity,
                        LongitudinalDisplacement::TrailingReferencedEntity, constraints)));
    Engine engine;
    ASSERT_EQ(engine.init(make_following_scenario(plain_entity("ego"), plain_entity("lead"), 200.0,
                                                  10.0, 10.0, std::move(events))),
              Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_NEAR(engine.state("ego")->speed, 12.0, kTol);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_NEAR(engine.state("ego")->speed, 14.0, kTol);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_NEAR(engine.state("ego")->speed, 16.0, kTol);
}

TEST(DistanceKeepingTest, PerformanceEnvelopeClampsWhenItIsTheTighterLimit) {
    // No DynamicConstraints at all: the Performance envelope (2 m/s^2) is the
    // only limit, and the max speed caps the command.
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event("keep", 0.0,
                                 std::make_shared<LongitudinalDistanceAction>(
                                     "ego", "lead", 20.0, std::nullopt, /*freespace=*/false,
                                     /*continuous=*/true)));
    Engine engine;
    const Performance envelope{11.0, 2.0, 2.0, std::nullopt, std::nullopt};
    ASSERT_EQ(engine.init(make_following_scenario(vehicle_entity("ego", 4.0, envelope),
                                                  plain_entity("lead"), 200.0, 10.0, 10.0,
                                                  std::move(events))),
              Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    // +2 m/s would be 12, but maxSpeed 11 caps it.
    EXPECT_NEAR(engine.state("ego")->speed, 11.0, kTol);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_NEAR(engine.state("ego")->speed, 11.0, kTol);
}

TEST(DistanceKeepingTest, NeverCommandsANegativeSpeed) {
    // The reference is far behind and the actor must fall back, but the scalar
    // speed model has no reverse: the actor stops instead.
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event("keep", 0.0,
                                 std::make_shared<LongitudinalDistanceAction>(
                                     "ego", "lead", 5.0, std::nullopt, /*freespace=*/false,
                                     /*continuous=*/true)));
    Engine engine;
    ASSERT_EQ(engine.init(make_following_scenario(plain_entity("ego"), plain_entity("lead"), -100.0,
                                                  10.0, 0.0, std::move(events))),
              Status::Ok);
    for (int i = 0; i < 5; ++i) {
        SCOPED_TRACE(i);
        ASSERT_EQ(engine.step(0.5), Status::Ok);
        EXPECT_GE(engine.state("ego")->speed, 0.0);
    }
    EXPECT_NEAR(engine.state("ego")->speed, 0.0, kTol);
}

// --- Lifetime and supersession --------------------------------------------

TEST(DistanceKeepingTest, ContinuousNeverCompletesAndTracksAChangingReference) {
    // §7.5.3: with continuous=true the action has no regular ending. The lead
    // changes speed mid-run and the follower keeps the 30 m gap.
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event("keep", 0.0,
                                 std::make_shared<LongitudinalDistanceAction>(
                                     "ego", "lead", 30.0, std::nullopt, /*freespace=*/false,
                                     /*continuous=*/true)));
    events.push_back(timed_event("lead-slows", 4.0, std::make_shared<SpeedAction>("lead", 4.0)));
    Engine engine;
    ASSERT_EQ(engine.init(make_following_scenario(plain_entity("ego"), plain_entity("lead"), 30.0,
                                                  10.0, 10.0, std::move(events))),
              Status::Ok);
    for (int i = 0; i < 20; ++i) {
        SCOPED_TRACE(i);
        ASSERT_EQ(engine.step(0.5), Status::Ok);
        EXPECT_EQ(*engine.storyboard_element_state("story/act/group/maneuver/keep"),
                  scena::runtime::ElementState::Running);
    }
    EXPECT_NEAR(gap_of(engine), 30.0, 1e-6);
    EXPECT_NEAR(engine.state("ego")->speed, 4.0, 1e-6); // matched the slower lead
}

TEST(DistanceKeepingTest, SupersedesARunningSpeedRampAndIsSupersededInTurn) {
    std::vector<scena::ir::Event> events;
    events.push_back(
        timed_event("ramp", 0.0,
                    std::make_shared<SpeedAction>(
                        "ego", 30.0,
                        TransitionDynamics{DynamicsShape::Linear, DynamicsDimension::Time, 10.0})));
    events.push_back(timed_event("keep", 2.0,
                                 std::make_shared<LongitudinalDistanceAction>(
                                     "ego", "lead", 25.0, std::nullopt, /*freespace=*/false,
                                     /*continuous=*/true)));
    events.push_back(timed_event("cruise", 6.0, std::make_shared<SpeedAction>("ego", 7.0)));
    Engine engine;
    ASSERT_EQ(engine.init(make_following_scenario(plain_entity("ego"), plain_entity("lead"), 60.0,
                                                  10.0, 10.0, std::move(events))),
              Status::Ok);

    for (int i = 0; i < 4; ++i) { // to t = 4: the ramp was superseded at t = 2
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    EXPECT_EQ(*engine.storyboard_element_state("story/act/group/maneuver/ramp"),
              scena::runtime::ElementState::Complete);
    EXPECT_EQ(*engine.storyboard_element_state("story/act/group/maneuver/keep"),
              scena::runtime::ElementState::Running);

    for (int i = 0; i < 3; ++i) { // to t = 7: the step SpeedAction took over
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    EXPECT_EQ(*engine.storyboard_element_state("story/act/group/maneuver/keep"),
              scena::runtime::ElementState::Complete);
    EXPECT_NEAR(engine.state("ego")->speed, 7.0, kTol);
}

// --- Validation -----------------------------------------------------------

Scenario make_validation_scenario(std::shared_ptr<scena::ir::Action> action) {
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event("keep", 0.0, std::move(action)));
    return make_following_scenario(plain_entity("ego"), plain_entity("lead"), 50.0, 10.0, 10.0,
                                   std::move(events));
}

TEST(DistanceKeepingValidationTest, DistanceAndTimeGapAreMutuallyExclusive) {
    Engine engine;
    EXPECT_EQ(engine.init(make_validation_scenario(std::make_shared<LongitudinalDistanceAction>(
                  "ego", "lead", 10.0, 2.0, false, false))),
              Status::ValidationError);
    Engine neither;
    EXPECT_EQ(neither.init(make_validation_scenario(std::make_shared<LongitudinalDistanceAction>(
                  "ego", "lead", std::nullopt, std::nullopt, false, false))),
              Status::ValidationError);
}

TEST(DistanceKeepingValidationTest, NegativeAndNonFiniteTargetsAreRejected) {
    Engine engine;
    EXPECT_EQ(engine.init(make_validation_scenario(std::make_shared<LongitudinalDistanceAction>(
                  "ego", "lead", -1.0, std::nullopt, false, false))),
              Status::ValidationError);
    ASSERT_FALSE(engine.diagnostics().empty());
    EXPECT_EQ(engine.diagnostics().front().rule_id,
              "asam.net:xosc:1.1.0:data_type.distances_are_not_negative");

    Engine nan_gap;
    EXPECT_EQ(nan_gap.init(make_validation_scenario(std::make_shared<LongitudinalDistanceAction>(
                  "ego", "lead", std::nullopt, std::nan(""), false, false))),
              Status::ValidationError);
}

TEST(DistanceKeepingValidationTest, UnknownReferenceEntityIsSemanticError) {
    Engine engine;
    EXPECT_EQ(engine.init(make_validation_scenario(std::make_shared<LongitudinalDistanceAction>(
                  "ego", "ghost", 10.0, std::nullopt, false, false))),
              Status::SemanticError);
}

TEST(DistanceKeepingValidationTest, InvalidDynamicConstraintsAreRejected) {
    DynamicConstraints constraints;
    constraints.max_deceleration = -2.0;
    Engine engine;
    EXPECT_EQ(engine.init(make_validation_scenario(std::make_shared<LongitudinalDistanceAction>(
                  "ego", "lead", 10.0, std::nullopt, false, false, CoordinateSystem::Entity,
                  LongitudinalDisplacement::Any, constraints))),
              Status::ValidationError);
}

TEST(DistanceKeepingValidationTest, RoadCoordinateSystemWarnsAndCompletesImmediately) {
    // Road-based gaps are deferred to p3-s4: the scenario still loads, the
    // action reports UnsupportedFeature and ends without touching the speed.
    Engine engine;
    ASSERT_EQ(engine.init(make_validation_scenario(std::make_shared<LongitudinalDistanceAction>(
                  "ego", "lead", 10.0, std::nullopt, false, true, CoordinateSystem::Lane))),
              Status::Ok);
    bool warned_at_init = false;
    for (const scena::Diagnostic& diagnostic : engine.diagnostics()) {
        warned_at_init = warned_at_init || (diagnostic.severity == Severity::Warning &&
                                            diagnostic.code == Status::UnsupportedFeature);
    }
    EXPECT_TRUE(warned_at_init);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(*engine.storyboard_element_state("story/act/group/maneuver/keep"),
              scena::runtime::ElementState::Complete);
    EXPECT_NEAR(engine.state("ego")->speed, 10.0, kTol);
}

TEST(DistanceKeepingValidationTest, FreespaceWithoutGeometryCompletesWithAWarning) {
    // A freespace gap needs both bounding boxes; without them the prerequisite
    // is missing (§7.5.2.2) and the action ends.
    Engine engine;
    ASSERT_EQ(engine.init(make_validation_scenario(std::make_shared<LongitudinalDistanceAction>(
                  "ego", "lead", 10.0, std::nullopt, /*freespace=*/true, /*continuous=*/true))),
              Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(*engine.storyboard_element_state("story/act/group/maneuver/keep"),
              scena::runtime::ElementState::Complete);
    EXPECT_NEAR(engine.state("ego")->speed, 10.0, kTol);
    bool warned = false;
    for (const scena::Diagnostic& diagnostic : engine.diagnostics()) {
        warned = warned || diagnostic.code == Status::UnsupportedFeature;
    }
    EXPECT_TRUE(warned);
}

// --- GS-4: traffic-jam approach -------------------------------------------

/// GS-4 from docs/roadmap/golden-scenarios.md: the lead vehicle decelerates to
/// near standstill; the ego approaches until a TimeToCollisionCondition arms a
/// strong SpeedAction deceleration; a freespace LongitudinalDistanceAction then
/// holds the gap; a timed trigger dissolves the jam and the ego tracks the lead
/// back up.
///
/// Both vehicles are 5 m long boxes centred on their reference points, so the
/// freespace gap is the reference-point gap minus 5 m.
scena::ir::Scenario make_gs4_scenario() {
    const Performance lead_envelope{40.0, 3.0, 6.0, std::nullopt, std::nullopt};
    const Performance ego_envelope{40.0, 3.0, 8.0, std::nullopt, std::nullopt};

    std::vector<scena::ir::Event> events;
    // The jam forms: the lead brakes from 25 m/s to a crawl over 8 s.
    events.push_back(timed_event(
        "jam-forms", 2.0,
        std::make_shared<SpeedAction>(
            "lead", 0.5, TransitionDynamics{DynamicsShape::Linear, DynamicsDimension::Time, 8.0})));

    // The ego brakes hard when the freespace time to collision drops below 6 s.
    scena::ir::Event brake;
    brake.name = "ego-brakes";
    brake.start_trigger = scena::ir::make_trigger(
        std::make_shared<scena::ir::TimeToCollisionCondition>(
            scena::ir::TriggeringEntities{scena::ir::TriggeringEntitiesRule::Any, {"ego"}},
            scena::ir::TimeToCollisionTarget{std::string("lead")}, 6.0, /*freespace=*/true,
            scena::ir::Rule::LessThan, CoordinateSystem::Entity,
            scena::ir::RelativeDistanceType::Longitudinal),
        scena::ir::ConditionEdge::None, 0.0);
    brake.actions.push_back(std::make_shared<SpeedAction>(
        "ego", 2.0, TransitionDynamics{DynamicsShape::Linear, DynamicsDimension::Time, 5.0}));
    events.push_back(std::move(brake));

    // Once the braking event has ended, distance keeping takes over: a
    // continuous freespace gap of 8 m to the lead.
    scena::ir::Event keep;
    keep.name = "ego-keeps-gap";
    keep.start_trigger =
        scena::ir::make_trigger(std::make_shared<scena::ir::StoryboardElementStateCondition>(
                                    scena::ir::StoryboardElementType::Event, "ego-brakes",
                                    scena::ir::StoryboardElementState::CompleteState),
                                scena::ir::ConditionEdge::None, 0.0);
    keep.actions.push_back(std::make_shared<LongitudinalDistanceAction>(
        "ego", "lead", 8.0, std::nullopt, /*freespace=*/true, /*continuous=*/true));
    events.push_back(std::move(keep));

    // The jam dissolves: the lead accelerates away and the ego follows it.
    events.push_back(
        timed_event("jam-dissolves", 40.0,
                    std::make_shared<SpeedAction>(
                        "lead", 20.0,
                        TransitionDynamics{DynamicsShape::Linear, DynamicsDimension::Time, 8.0})));

    Scenario scenario = make_following_scenario(vehicle_entity("ego", 5.0, ego_envelope),
                                                vehicle_entity("lead", 5.0, lead_envelope),
                                                /*lead_x=*/150.0, /*ego_speed=*/25.0,
                                                /*lead_speed=*/25.0, std::move(events));
    scenario.name = "gs4-traffic-jam-approach";
    return scenario;
}

/// Freespace bumper-to-bumper gap for the GS-4 geometry (5 m boxes).
double gs4_freespace_gap(const Engine& engine) {
    return gap_of(engine) - 5.0;
}

TEST(GoldenScenarioTest, GS4TrafficJamApproachHoldsTheGap) {
    Engine engine;
    ASSERT_EQ(engine.init(make_gs4_scenario()), Status::Ok);

    // Pass criteria (golden-scenarios.md GS-4): the minimum gap never drops
    // below the declared floor and the boxes never intersect.
    constexpr double kGapFloor = 1.0;
    double minimum_gap = gs4_freespace_gap(engine);
    bool braked = false;
    for (int i = 0; i < 600; ++i) { // 60 s at 0.1 s steps
        SCOPED_TRACE(i);
        ASSERT_EQ(engine.step(0.1), Status::Ok);
        const double gap = gs4_freespace_gap(engine);
        minimum_gap = std::fmin(minimum_gap, gap);
        EXPECT_GT(gap, 0.0); // no collision: the boxes never touch
        braked = braked || engine.state("ego")->speed < 5.0;
    }
    EXPECT_GT(minimum_gap, kGapFloor);
    EXPECT_TRUE(braked); // the TTC trigger really did arm the deceleration

    const std::string base = "story/act/group/maneuver/";
    EXPECT_EQ(*engine.storyboard_element_state(base + "jam-forms"),
              scena::runtime::ElementState::Complete);
    EXPECT_EQ(*engine.storyboard_element_state(base + "ego-brakes"),
              scena::runtime::ElementState::Complete);
    // Continuous distance keeping has no regular ending (§7.5.3).
    EXPECT_EQ(*engine.storyboard_element_state(base + "ego-keeps-gap"),
              scena::runtime::ElementState::Running);

    // After the jam dissolves the ego tracks the lead again: it is back up to
    // speed and holding the commanded gap.
    EXPECT_NEAR(engine.state("ego")->speed, engine.state("lead")->speed, 0.5);
    EXPECT_NEAR(gs4_freespace_gap(engine), 8.0, 0.5);
}

} // namespace
