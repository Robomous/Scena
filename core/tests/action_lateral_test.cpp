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
