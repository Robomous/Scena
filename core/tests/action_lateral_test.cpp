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
//
// The three lateral actions at the engine level (p2-s3): LaneChangeAction,
// LaneOffsetAction and LateralDistanceAction over the lateral-axis kinematic
// model of ADR-0016 — flat-world lane widths and the forward-pulled IRoadQuery
// lane queries, the offset transitions, lateral distance keeping, lateral
// supersession (§7.5.1, Annex A Table 10), and the validation diagnostics.

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
#include "scena/ir/condition.h"
#include "scena/ir/coordinate_system.h"
#include "scena/ir/dynamics.h"
#include "scena/ir/entity.h"
#include "scena/ir/position.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/trajectory.h"
#include "scena/ir/trigger.h"

namespace {

using scena::Engine;
using scena::Severity;
using scena::Status;
using scena::ir::AbsoluteTargetLane;
using scena::ir::AbsoluteTargetLaneOffset;
using scena::ir::ControlMode;
using scena::ir::CoordinateSystem;
using scena::ir::DynamicConstraints;
using scena::ir::DynamicsDimension;
using scena::ir::DynamicsShape;
using scena::ir::Entity;
using scena::ir::FollowingMode;
using scena::ir::LaneChangeAction;
using scena::ir::LaneOffsetAction;
using scena::ir::LateralDisplacement;
using scena::ir::LateralDistanceAction;
using scena::ir::RelativeTargetLane;
using scena::ir::RelativeTargetLaneOffset;
using scena::ir::Scenario;
using scena::ir::SimulationTimeCondition;
using scena::ir::SpeedAction;
using scena::ir::TeleportAction;
using scena::ir::TransitionDynamics;
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
/// Performance envelope, so nothing clamps a lateral controller.
Entity plain_entity(const std::string& id) {
    Entity entity;
    entity.id = id;
    entity.name = id;
    entity.control_mode = ControlMode::EngineControlled;
    return entity;
}

/// Two entities heading along +x: `ego` at the origin and `other` at
/// (other_x, other_y), both at their given speeds, plus one maneuver holding
/// `events` in document order.
Scenario make_lateral_scenario(Entity ego, Entity other, double other_x, double other_y,
                               double ego_speed, double other_speed,
                               std::vector<scena::ir::Event> events) {
    Scenario scenario;
    scenario.name = "lateral";
    const std::string other_id = other.id;
    scenario.entities.push_back(std::move(ego));
    scenario.entities.push_back(std::move(other));
    scenario.init_actions.push_back(
        std::make_shared<TeleportAction>("ego", WorldPosition{0.0, 0.0, 0.0}));
    scenario.init_actions.push_back(
        std::make_shared<TeleportAction>(other_id, WorldPosition{other_x, other_y, 0.0}));
    scenario.init_actions.push_back(std::make_shared<SpeedAction>("ego", ego_speed));
    scenario.init_actions.push_back(std::make_shared<SpeedAction>(other_id, other_speed));

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

/// The single-event scenario every validation case below uses: `ego` plus a
/// reference entity named "lead" one lane to the left.
Scenario make_validation_scenario(std::shared_ptr<scena::ir::Action> action) {
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event("lateral", 0.0, std::move(action)));
    return make_lateral_scenario(plain_entity("ego"), plain_entity("lead"), 20.0, 3.5, 10.0, 10.0,
                                 std::move(events));
}

bool has_warning(const Engine& engine, Status code) {
    for (const scena::Diagnostic& diagnostic : engine.diagnostics()) {
        if (diagnostic.severity == Severity::Warning && diagnostic.code == code) {
            return true;
        }
    }
    return false;
}

const char* const kEventPath = "story/act/group/maneuver/lateral";

scena::runtime::ElementState state_of(const Engine& engine, const char* path = kEventPath) {
    return *engine.storyboard_element_state(path);
}

/// Runs `count` steps of `dt`.
void run(Engine& engine, int count, double dt) {
    for (int i = 0; i < count; ++i) {
        ASSERT_EQ(engine.step(dt), Status::Ok);
    }
}

// --- LaneOffsetAction ------------------------------------------------------

TEST(LaneOffsetTest, ReachesAnAbsoluteTargetOverTheDerivedDuration) {
    // Cubic over 3 m at 2 m/s^2: T = sqrt(6*3/2) = 3 s exactly (ADR-0016). The
    // axis is captured along the ego's +x heading, so a positive offset is +y
    // (§7.4.1.4, ISO 8855).
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event(
        "lateral", 0.0,
        std::make_shared<LaneOffsetAction>("ego", AbsoluteTargetLaneOffset{3.0},
                                           /*continuous=*/false, DynamicsShape::Cubic, 2.0)));
    Engine engine;
    ASSERT_EQ(engine.init(make_lateral_scenario(plain_entity("ego"), plain_entity("lead"), 20.0,
                                                3.5, 10.0, 10.0, std::move(events))),
              Status::Ok);
    // The event fires at the t = 0 evaluation, so the first step already
    // advances the ramp. The cubic starts flat: half a second in, well under a
    // sixth of the offset is covered.
    run(engine, 5, 0.1);
    EXPECT_GT(engine.state("ego")->y, 0.0);
    EXPECT_LT(engine.state("ego")->y, 0.5);
    // Mid-transition the entity is steering: the heading has left the axis.
    EXPECT_GT(engine.state("ego")->heading, 0.0);
    EXPECT_EQ(state_of(engine), scena::runtime::ElementState::Running);

    // Half way through the 3 s span the cubic sits at half the offset.
    run(engine, 10, 0.1);
    EXPECT_NEAR(engine.state("ego")->y, 1.5, 1e-6);

    run(engine, 25, 0.1); // 4 s: comfortably past the span
    EXPECT_EQ(state_of(engine), scena::runtime::ElementState::Complete);
    // Arrived exactly on the target offset, straightened back onto the axis.
    EXPECT_NEAR(engine.state("ego")->y, 3.0, 1e-9);
    EXPECT_DOUBLE_EQ(engine.state("ego")->heading, 0.0);
    // The longitudinal domain was never touched: 4 s at 10 m/s.
    EXPECT_NEAR(engine.state("ego")->x, 40.0, 1e-9);
    EXPECT_NEAR(engine.state("ego")->speed, 10.0, kTol);
}

TEST(LaneOffsetTest, MissingMaxLateralAccIsInstantaneous) {
    // "Missing value is interpreted as 'inf'" (Class `LaneOffsetActionDynamics`).
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event(
        "lateral", 0.0,
        std::make_shared<LaneOffsetAction>("ego", AbsoluteTargetLaneOffset{-1.25},
                                           /*continuous=*/false, DynamicsShape::Sinusoidal)));
    Engine engine;
    ASSERT_EQ(engine.init(make_lateral_scenario(plain_entity("ego"), plain_entity("lead"), 20.0,
                                                3.5, 10.0, 10.0, std::move(events))),
              Status::Ok);
    ASSERT_EQ(engine.step(0.1), Status::Ok);
    EXPECT_EQ(state_of(engine), scena::runtime::ElementState::Complete);
    // A negative offset is to the right of the axis.
    EXPECT_NEAR(engine.state("ego")->y, -1.25, 1e-12);
    EXPECT_DOUBLE_EQ(engine.state("ego")->heading, 0.0);
}

TEST(LaneOffsetTest, AStepShapeIsInstantaneousWhateverTheAcceleration) {
    // §7.4.1.4: with a step shape "the displacement required to achieve the
    // desired lane offset is performed instantaneously - not over time".
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event(
        "lateral", 0.0,
        std::make_shared<LaneOffsetAction>("ego", AbsoluteTargetLaneOffset{2.0},
                                           /*continuous=*/false, DynamicsShape::Step, 0.5)));
    Engine engine;
    ASSERT_EQ(engine.init(make_lateral_scenario(plain_entity("ego"), plain_entity("lead"), 20.0,
                                                3.5, 10.0, 10.0, std::move(events))),
              Status::Ok);
    ASSERT_EQ(engine.step(0.1), Status::Ok);
    EXPECT_EQ(state_of(engine), scena::runtime::ElementState::Complete);
    EXPECT_NEAR(engine.state("ego")->y, 2.0, 1e-12);
}

TEST(LaneOffsetTest, RelativeTargetIsMeasuredFromTheReferenceLanePosition) {
    // Flat world: the reference sits on its own lane centre, so a relative
    // offset of 0 puts the actor on the reference's lateral position.
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event(
        "lateral", 0.0,
        std::make_shared<LaneOffsetAction>("ego", RelativeTargetLaneOffset{"lead", 0.0},
                                           /*continuous=*/false, DynamicsShape::Linear, 1.0)));
    Engine engine;
    ASSERT_EQ(engine.init(make_lateral_scenario(plain_entity("ego"), plain_entity("lead"), 20.0,
                                                3.5, 10.0, 10.0, std::move(events))),
              Status::Ok);
    run(engine, 100, 0.1);
    EXPECT_EQ(state_of(engine), scena::runtime::ElementState::Complete);
    EXPECT_NEAR(engine.state("ego")->y, 3.5, 1e-9);
}

TEST(LaneOffsetTest, ContinuousNeverEndsAndRecorrectsWhenTheReferenceMoves) {
    // §7.5.3 / Annex A Table 10: continuous == true has "no regular ending".
    // After arrival each re-poll re-measures, so a reference that jumps sideways
    // is tracked (Running, not Ongoing — an Ongoing action is never re-polled).
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event(
        "lateral", 0.0,
        std::make_shared<LaneOffsetAction>("ego", RelativeTargetLaneOffset{"lead", 0.0},
                                           /*continuous=*/true, DynamicsShape::Linear, 1.0)));
    events.push_back(timed_event(
        "shove", 20.0, std::make_shared<TeleportAction>("lead", WorldPosition{400.0, 7.0, 0.0})));
    Engine engine;
    ASSERT_EQ(engine.init(make_lateral_scenario(plain_entity("ego"), plain_entity("lead"), 20.0,
                                                3.5, 10.0, 10.0, std::move(events))),
              Status::Ok);
    run(engine, 100, 0.1); // 10 s: arrived at the reference's lane
    EXPECT_NEAR(engine.state("ego")->y, 3.5, 1e-9);
    EXPECT_EQ(state_of(engine), scena::runtime::ElementState::Running);

    run(engine, 200, 0.1); // through the teleport at t = 20 s and well past it
    EXPECT_NEAR(engine.state("ego")->y, 7.0, 1e-9);
    // Still no regular ending.
    EXPECT_EQ(state_of(engine), scena::runtime::ElementState::Running);
}

TEST(LaneOffsetTest, ATeleportOfTheActorCarriesTheOffsetAcross) {
    // ADR-0016: the axis is re-anchored through the entity's new position at the
    // offset it had reached, so an in-flight transition carries on instead of
    // dragging the entity back across the old axis.
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event(
        "lateral", 0.0,
        std::make_shared<LaneOffsetAction>("ego", AbsoluteTargetLaneOffset{4.0},
                                           /*continuous=*/false, DynamicsShape::Linear, 1.0)));
    events.push_back(timed_event(
        "jump", 1.0, std::make_shared<TeleportAction>("ego", WorldPosition{500.0, 100.0, 0.0})));
    Engine engine;
    ASSERT_EQ(engine.init(make_lateral_scenario(plain_entity("ego"), plain_entity("lead"), 20.0,
                                                3.5, 10.0, 10.0, std::move(events))),
              Status::Ok);
    // Linear over 4 m at 1 m/s^2 spans 2*sqrt(4/1) = 4 s, so the teleport at
    // t = 1 s lands mid-transition. Find the step it fires on and note how much
    // offset had been applied by then.
    double applied_before_jump = 0.0;
    double y_after_jump = 0.0;
    for (int i = 0; i < 20; ++i) {
        const double before = engine.state("ego")->y;
        ASSERT_EQ(engine.step(0.1), Status::Ok);
        if (engine.state("ego")->y > 50.0) {
            applied_before_jump = before;
            y_after_jump = engine.state("ego")->y;
            break;
        }
    }
    ASSERT_GT(applied_before_jump, 0.0);
    // The entity carried on from the new position rather than snapping back
    // across the old axis.
    EXPECT_GT(y_after_jump, 100.0);
    EXPECT_LT(y_after_jump, 101.0);

    run(engine, 100, 0.1);
    EXPECT_EQ(state_of(engine), scena::runtime::ElementState::Complete);
    // What had been applied before the jump stays applied; only the remainder
    // is added to the teleport target.
    EXPECT_NEAR(engine.state("ego")->y, 100.0 + (4.0 - applied_before_jump), 1e-9);
}

TEST(LaneOffsetTest, AnArrivedContinuousOffsetSurvivesATeleportExactly) {
    // Once a continuous offset has arrived, re-anchoring means a teleport moves
    // the entity without disturbing the offset it holds: it must not drift back
    // out by another 4 m, nor be dragged back to the old axis.
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event(
        "lateral", 0.0,
        std::make_shared<LaneOffsetAction>("ego", AbsoluteTargetLaneOffset{4.0},
                                           /*continuous=*/true, DynamicsShape::Linear, 1.0)));
    events.push_back(timed_event(
        "jump", 10.0, std::make_shared<TeleportAction>("ego", WorldPosition{500.0, 100.0, 0.0})));
    Engine engine;
    ASSERT_EQ(engine.init(make_lateral_scenario(plain_entity("ego"), plain_entity("lead"), 20.0,
                                                3.5, 10.0, 10.0, std::move(events))),
              Status::Ok);
    run(engine, 60, 0.1); // 6 s: past the 4 s span, holding 4 m
    EXPECT_NEAR(engine.state("ego")->y, 4.0, 1e-9);

    run(engine, 100, 0.1); // through the teleport and 6 s beyond it
    EXPECT_NEAR(engine.state("ego")->y, 100.0, 1e-9);
    EXPECT_EQ(state_of(engine), scena::runtime::ElementState::Running);
}

TEST(LaneOffsetTest, AnInitPhaseOffsetIsAppliedInstantaneously) {
    // §8.5: init actions take effect before simulation time starts, so the
    // offset is a single perpendicular translate with no heading blending.
    Scenario scenario =
        make_lateral_scenario(plain_entity("ego"), plain_entity("lead"), 20.0, 3.5, 10.0, 10.0, {});
    scenario.init_actions.push_back(std::make_shared<LaneOffsetAction>(
        "ego", AbsoluteTargetLaneOffset{1.75}, /*continuous=*/true, DynamicsShape::Cubic, 2.0));
    Engine engine;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    EXPECT_NEAR(engine.state("ego")->y, 1.75, kTol);
    EXPECT_DOUBLE_EQ(engine.state("ego")->heading, 0.0);
    // The axis did not survive into the run: the ego integrates straight.
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_NEAR(engine.state("ego")->y, 1.75, kTol);
    EXPECT_NEAR(engine.state("ego")->x, 10.0, kTol);
}

TEST(LaneOffsetTest, ADeletedReferenceStopsTheAction) {
    // §7.5.2.2: the reference entity is a prerequisite.
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event(
        "lateral", 0.0,
        std::make_shared<LaneOffsetAction>("ego", RelativeTargetLaneOffset{"lead", 0.0},
                                           /*continuous=*/true, DynamicsShape::Linear, 1.0)));
    events.push_back(
        timed_event("remove", 1.0, std::make_shared<scena::ir::DeleteEntityAction>("lead")));
    Engine engine;
    ASSERT_EQ(engine.init(make_lateral_scenario(plain_entity("ego"), plain_entity("lead"), 20.0,
                                                3.5, 10.0, 10.0, std::move(events))),
              Status::Ok);
    run(engine, 20, 0.1);
    EXPECT_EQ(state_of(engine), scena::runtime::ElementState::Complete);
    EXPECT_TRUE(has_warning(engine, Status::UnknownEntity));
}

// --- Lateral supersession (§7.5.1, Annex A Table 10) -----------------------

TEST(LateralSupersessionTest, ANewLateralActionRetiresTheRunningOneAndKeepsTheAxis) {
    // A lateral action superseding another continues on the same reference
    // line, so the second target is measured from the same axis rather than
    // from wherever the first one had got to.
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event(
        "lateral", 0.0,
        std::make_shared<LaneOffsetAction>("ego", AbsoluteTargetLaneOffset{3.5},
                                           /*continuous=*/false, DynamicsShape::Linear, 0.5)));
    events.push_back(timed_event(
        "second", 1.0,
        std::make_shared<LaneOffsetAction>("ego", AbsoluteTargetLaneOffset{1.0},
                                           /*continuous=*/false, DynamicsShape::Linear, 0.5)));
    Engine engine;
    ASSERT_EQ(engine.init(make_lateral_scenario(plain_entity("ego"), plain_entity("lead"), 20.0,
                                                3.5, 10.0, 10.0, std::move(events))),
              Status::Ok);
    run(engine, 20, 0.1);
    // The superseded action completes on its next re-poll (§7.5.2.1 override).
    EXPECT_EQ(state_of(engine), scena::runtime::ElementState::Complete);

    run(engine, 200, 0.1);
    EXPECT_EQ(state_of(engine, "story/act/group/maneuver/second"),
              scena::runtime::ElementState::Complete);
    // 1.0 measured from the original axis, not from the offset already reached.
    EXPECT_NEAR(engine.state("ego")->y, 1.0, 1e-9);
}

TEST(LateralSupersessionTest, ATrajectoryAndALateralActionStopEachOther) {
    // §7.5.2.1: "an instance of FollowTrajectoryAction overrides an instance of
    // LaneChangeAction" — and every lateral action does the same in reverse,
    // since both assign a lateral control strategy (Table 10).
    scena::ir::Trajectory path;
    path.name = "shift";
    path.vertices.push_back(
        scena::ir::TrajectoryVertex{WorldPosition{0.0, 0.0, 0.0}, std::nullopt});
    path.vertices.push_back(
        scena::ir::TrajectoryVertex{WorldPosition{200.0, -6.0, 0.0}, std::nullopt});

    std::vector<scena::ir::Event> events;
    events.push_back(timed_event(
        "lateral", 0.0,
        std::make_shared<LaneOffsetAction>("ego", AbsoluteTargetLaneOffset{3.5},
                                           /*continuous=*/true, DynamicsShape::Linear, 0.5)));
    events.push_back(
        timed_event("path", 1.0, std::make_shared<scena::ir::FollowTrajectoryAction>("ego", path)));
    Engine engine;
    ASSERT_EQ(engine.init(make_lateral_scenario(plain_entity("ego"), plain_entity("lead"), 20.0,
                                                3.5, 10.0, 10.0, std::move(events))),
              Status::Ok);
    run(engine, 20, 0.1);
    // The continuous lane offset would never end on its own; the trajectory
    // ended it.
    EXPECT_EQ(state_of(engine), scena::runtime::ElementState::Complete);
    EXPECT_EQ(state_of(engine, "story/act/group/maneuver/path"),
              scena::runtime::ElementState::Running);
}

TEST(LateralSupersessionTest, ALateralActionStopsARunningTrajectory) {
    scena::ir::Trajectory path;
    path.name = "long";
    path.vertices.push_back(
        scena::ir::TrajectoryVertex{WorldPosition{0.0, 0.0, 0.0}, std::nullopt});
    path.vertices.push_back(
        scena::ir::TrajectoryVertex{WorldPosition{1000.0, 0.0, 0.0}, std::nullopt});

    std::vector<scena::ir::Event> events;
    events.push_back(timed_event("lateral", 0.0,
                                 std::make_shared<scena::ir::FollowTrajectoryAction>("ego", path)));
    events.push_back(timed_event(
        "offset", 1.0,
        std::make_shared<LaneOffsetAction>("ego", AbsoluteTargetLaneOffset{2.0},
                                           /*continuous=*/false, DynamicsShape::Linear, 1.0)));
    Engine engine;
    ASSERT_EQ(engine.init(make_lateral_scenario(plain_entity("ego"), plain_entity("lead"), 20.0,
                                                3.5, 10.0, 10.0, std::move(events))),
              Status::Ok);
    run(engine, 20, 0.1);
    EXPECT_EQ(state_of(engine), scena::runtime::ElementState::Complete);
    run(engine, 100, 0.1);
    EXPECT_EQ(state_of(engine, "story/act/group/maneuver/offset"),
              scena::runtime::ElementState::Complete);
    EXPECT_NEAR(engine.state("ego")->y, 2.0, 1e-9);
}

TEST(LateralSupersessionTest, TheLongitudinalAndLateralDomainsAreIndependent) {
    // Table 10: a SpeedAction assigns a longitudinal control strategy only, so
    // it never conflicts with a lane offset, and vice versa. Both run.
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event(
        "lateral", 0.0,
        std::make_shared<LaneOffsetAction>("ego", AbsoluteTargetLaneOffset{3.0},
                                           /*continuous=*/false, DynamicsShape::Cubic, 2.0)));
    events.push_back(
        timed_event("brake", 1.0,
                    std::make_shared<SpeedAction>(
                        "ego", 4.0,
                        TransitionDynamics{DynamicsShape::Linear, DynamicsDimension::Time, 2.0,
                                           scena::ir::FollowingMode::Position})));
    Engine engine;
    ASSERT_EQ(engine.init(make_lateral_scenario(plain_entity("ego"), plain_entity("lead"), 20.0,
                                                3.5, 10.0, 10.0, std::move(events))),
              Status::Ok);
    run(engine, 50, 0.1);
    // The speed ramp ran to completion and the lane offset was not disturbed.
    EXPECT_NEAR(engine.state("ego")->speed, 4.0, 1e-9);
    EXPECT_EQ(state_of(engine), scena::runtime::ElementState::Complete);
    EXPECT_EQ(state_of(engine, "story/act/group/maneuver/brake"),
              scena::runtime::ElementState::Complete);
    EXPECT_NEAR(engine.state("ego")->y, 3.0, 1e-9);
}

TEST(LateralSupersessionTest, DeactivatingTheLateralDomainStopsAndSuppresses) {
    // ADR-0014: releasing a domain retires its owner and suppresses later
    // actions that need it. The entity keeps the heading it was blended to.
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event(
        "lateral", 0.0,
        std::make_shared<LaneOffsetAction>("ego", AbsoluteTargetLaneOffset{3.5},
                                           /*continuous=*/true, DynamicsShape::Linear, 0.5)));
    events.push_back(timed_event("off", 1.0,
                                 std::make_shared<scena::ir::ActivateControllerAction>(
                                     "ego", /*lateral=*/false, /*longitudinal=*/std::nullopt)));
    events.push_back(timed_event(
        "again", 2.0,
        std::make_shared<LaneOffsetAction>("ego", AbsoluteTargetLaneOffset{1.0},
                                           /*continuous=*/false, DynamicsShape::Linear, 0.5)));
    Engine engine;
    ASSERT_EQ(engine.init(make_lateral_scenario(plain_entity("ego"), plain_entity("lead"), 20.0,
                                                3.5, 10.0, 10.0, std::move(events))),
              Status::Ok);
    run(engine, 20, 0.1);
    EXPECT_EQ(state_of(engine), scena::runtime::ElementState::Complete);
    const double held_y = engine.state("ego")->y;

    run(engine, 20, 0.1);
    // The later lateral action is skipped, not run: the domain is deactivated.
    EXPECT_EQ(state_of(engine, "story/act/group/maneuver/again"),
              scena::runtime::ElementState::Complete);
    EXPECT_TRUE(has_warning(engine, Status::InvalidControlMode));
    // The released entity kept drifting on the heading it had, so it is off the
    // held offset but nowhere near the abandoned target.
    EXPECT_GT(engine.state("ego")->y, held_y);
    EXPECT_LT(engine.state("ego")->y, 3.5);
}

// --- Validation (C1) -------------------------------------------------------

TEST(LaneChangeValidationTest, UnknownReferenceEntityIsSemanticError) {
    Engine engine;
    EXPECT_EQ(engine.init(make_validation_scenario(std::make_shared<LaneChangeAction>(
                  "ego", RelativeTargetLane{"ghost", 1}, TransitionDynamics{}))),
              Status::SemanticError);
}

TEST(LaneChangeValidationTest, NonFiniteDynamicsAndOffsetAreRejected) {
    Engine negative;
    EXPECT_EQ(negative.init(make_validation_scenario(std::make_shared<LaneChangeAction>(
                  "ego", RelativeTargetLane{"lead", 1},
                  TransitionDynamics{DynamicsShape::Linear, DynamicsDimension::Time, -1.0,
                                     scena::ir::FollowingMode::Position}))),
              Status::ValidationError);

    Engine nan_offset;
    EXPECT_EQ(nan_offset.init(make_validation_scenario(std::make_shared<LaneChangeAction>(
                  "ego", RelativeTargetLane{"lead", 1}, TransitionDynamics{}, std::nan("")))),
              Status::ValidationError);
}

TEST(LaneChangeValidationTest, StepShapeRequiresAZeroDynamicsValue) {
    // §TransitionDynamics: with a step shape "value must be 0".
    Engine engine;
    EXPECT_EQ(engine.init(make_validation_scenario(std::make_shared<LaneChangeAction>(
                  "ego", RelativeTargetLane{"lead", 1},
                  TransitionDynamics{DynamicsShape::Step, DynamicsDimension::Time, 2.0,
                                     scena::ir::FollowingMode::Position}))),
              Status::ValidationError);
}

TEST(LaneChangeValidationTest, EmptyAbsoluteLaneIdIsRejected) {
    Engine engine;
    EXPECT_EQ(engine.init(make_validation_scenario(std::make_shared<LaneChangeAction>(
                  "ego", AbsoluteTargetLane{""}, TransitionDynamics{}))),
              Status::ValidationError);
}

TEST(LaneOffsetValidationTest, UnknownReferenceAndNonFiniteTargetAreRejected) {
    Engine unknown;
    EXPECT_EQ(unknown.init(make_validation_scenario(std::make_shared<LaneOffsetAction>(
                  "ego", RelativeTargetLaneOffset{"ghost", 1.0}, false, DynamicsShape::Linear))),
              Status::SemanticError);

    Engine nan_target;
    EXPECT_EQ(nan_target.init(make_validation_scenario(std::make_shared<LaneOffsetAction>(
                  "ego", AbsoluteTargetLaneOffset{std::nan("")}, false, DynamicsShape::Linear))),
              Status::ValidationError);
}

TEST(LaneOffsetValidationTest, NonPositiveMaxLateralAccIsRejected) {
    // A zero limit permits no lateral motion, so the target could never be
    // reached; an absent one means 'inf' and is fine.
    Engine zero;
    EXPECT_EQ(zero.init(make_validation_scenario(std::make_shared<LaneOffsetAction>(
                  "ego", AbsoluteTargetLaneOffset{1.0}, false, DynamicsShape::Cubic, 0.0))),
              Status::ValidationError);

    Engine absent;
    EXPECT_EQ(absent.init(make_validation_scenario(std::make_shared<LaneOffsetAction>(
                  "ego", AbsoluteTargetLaneOffset{1.0}, false, DynamicsShape::Cubic))),
              Status::Ok);
}

TEST(LateralDistanceValidationTest, NegativeDistanceCitesTheNotNegativeRule) {
    Engine engine;
    EXPECT_EQ(engine.init(make_validation_scenario(
                  std::make_shared<LateralDistanceAction>("ego", "lead", -1.0, false, false))),
              Status::ValidationError);
    ASSERT_FALSE(engine.diagnostics().empty());
    EXPECT_EQ(engine.diagnostics().front().rule_id,
              "asam.net:xosc:1.1.0:data_type.distances_are_not_negative");
}

TEST(LateralDistanceValidationTest, UnknownReferenceAndBadConstraintsAreRejected) {
    Engine unknown;
    EXPECT_EQ(unknown.init(make_validation_scenario(
                  std::make_shared<LateralDistanceAction>("ego", "ghost", 1.0, false, false))),
              Status::SemanticError);

    DynamicConstraints constraints;
    constraints.max_speed = -1.0;
    Engine bad;
    EXPECT_EQ(bad.init(make_validation_scenario(std::make_shared<LateralDistanceAction>(
                  "ego", "lead", 1.0, false, false, CoordinateSystem::Entity,
                  LateralDisplacement::Any, constraints))),
              Status::ValidationError);
}

TEST(LateralDistanceValidationTest, LaneCoordinateSystemWarnsAsUndefined) {
    // §6.4.8.2.2: "The lateral distance is undefined" in the lane coordinate
    // system. The scenario still loads; the action is warned about at init.
    Engine engine;
    ASSERT_EQ(engine.init(make_validation_scenario(std::make_shared<LateralDistanceAction>(
                  "ego", "lead", 1.0, false, true, CoordinateSystem::Lane))),
              Status::Ok);
    EXPECT_TRUE(has_warning(engine, Status::UnsupportedFeature));
}

TEST(LateralDistanceValidationTest, RoadCoordinateSystemWarns) {
    Engine engine;
    ASSERT_EQ(engine.init(make_validation_scenario(std::make_shared<LateralDistanceAction>(
                  "ego", "lead", 1.0, false, true, CoordinateSystem::Road))),
              Status::Ok);
    EXPECT_TRUE(has_warning(engine, Status::UnsupportedFeature));
}

} // namespace
