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
// The p5-s5 action IR surface: construction, accessors, defaults, and the
// stable ASAM kind() names of the routing, distance-keeping, controller and
// visibility actions. Runtime behavior lives in action_distance_test.cpp,
// action_routing_test.cpp and action_controller_visibility_test.cpp.

#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "scena/entity_visibility.h"
#include "scena/ir/action.h"
#include "scena/ir/controller.h"
#include "scena/ir/coordinate_system.h"
#include "scena/ir/route.h"
#include "scena/ir/rule.h"
#include "scena/ir/trajectory.h"

namespace {

using scena::EntityVisibility;
using scena::ir::AbsoluteTargetLane;
using scena::ir::AbsoluteTargetLaneOffset;
using scena::ir::AcquirePositionAction;
using scena::ir::Action;
using scena::ir::ActivateControllerAction;
using scena::ir::AssignControllerAction;
using scena::ir::AssignRouteAction;
using scena::ir::Controller;
using scena::ir::ControllerType;
using scena::ir::controls_lateral;
using scena::ir::controls_longitudinal;
using scena::ir::CoordinateSystem;
using scena::ir::DynamicConstraints;
using scena::ir::DynamicsDimension;
using scena::ir::DynamicsShape;
using scena::ir::FollowingMode;
using scena::ir::FollowTrajectoryAction;
using scena::ir::GlobalAction;
using scena::ir::LaneChangeAction;
using scena::ir::LaneOffsetAction;
using scena::ir::LateralDisplacement;
using scena::ir::LateralDistanceAction;
using scena::ir::LongitudinalDisplacement;
using scena::ir::LongitudinalDistanceAction;
using scena::ir::ModifyOperator;
using scena::ir::ParameterModifyAction;
using scena::ir::ParameterSetAction;
using scena::ir::Property;
using scena::ir::ReferenceContext;
using scena::ir::RelativeTargetLane;
using scena::ir::RelativeTargetLaneOffset;
using scena::ir::Route;
using scena::ir::RouteStrategy;
using scena::ir::Timing;
using scena::ir::Trajectory;
using scena::ir::TrajectoryVertex;
using scena::ir::TransitionDynamics;
using scena::ir::VariableModifyAction;
using scena::ir::VariableSetAction;
using scena::ir::VisibilityAction;
using scena::ir::Waypoint;
using scena::ir::WorldPosition;

constexpr double kTol = 1e-12;

// --- Value types ----------------------------------------------------------

TEST(RouteIrTest, WaypointDefaultsToShortestStrategy) {
    const Waypoint waypoint{WorldPosition{1.0, 2.0, 3.0}, RouteStrategy::Shortest};
    EXPECT_DOUBLE_EQ(waypoint.position.x, 1.0);
    EXPECT_EQ(waypoint.strategy, RouteStrategy::Shortest);
    EXPECT_EQ(Waypoint{}.strategy, RouteStrategy::Shortest);
}

TEST(RouteIrTest, RouteKeepsWaypointsInAuthoredOrder) {
    Route route;
    route.name = "detour";
    route.closed = true;
    route.waypoints.push_back(Waypoint{WorldPosition{0.0, 0.0, 0.0}, RouteStrategy::Fastest});
    route.waypoints.push_back(
        Waypoint{WorldPosition{10.0, 0.0, 0.0}, RouteStrategy::LeastIntersections});
    route.waypoints.push_back(Waypoint{WorldPosition{20.0, 5.0, 0.0}, RouteStrategy::Random});
    ASSERT_EQ(route.waypoints.size(), 3U);
    EXPECT_EQ(route.name, "detour");
    EXPECT_TRUE(route.closed);
    EXPECT_EQ(route.waypoints[0].strategy, RouteStrategy::Fastest);
    EXPECT_EQ(route.waypoints[1].strategy, RouteStrategy::LeastIntersections);
    EXPECT_EQ(route.waypoints[2].strategy, RouteStrategy::Random);
    EXPECT_DOUBLE_EQ(route.waypoints[2].position.y, 5.0);
}

TEST(TrajectoryIrTest, VertexTimeIsOptionalAndTimingDefaultsAreIdentity) {
    const TrajectoryVertex untimed{WorldPosition{1.0, 0.0, 0.0}, std::nullopt};
    const TrajectoryVertex timed{WorldPosition{2.0, 0.0, 0.0}, 1.5};
    EXPECT_FALSE(untimed.time.has_value());
    ASSERT_TRUE(timed.time.has_value());
    EXPECT_DOUBLE_EQ(*timed.time, 1.5);

    const Timing timing;
    EXPECT_EQ(timing.domain, ReferenceContext::Absolute);
    EXPECT_DOUBLE_EQ(timing.scale, 1.0); // 1.0 means no scaling (§Timing)
    EXPECT_DOUBLE_EQ(timing.offset, 0.0);
}

TEST(ControllerIrTest, DefaultsToMovementAndKeepsPropertyOrder) {
    Controller controller;
    controller.name = "driver";
    EXPECT_EQ(controller.type, ControllerType::Movement);
    controller.properties.push_back(Property{"aggressiveness", "0.7"});
    controller.properties.push_back(Property{"model", "idm"});
    ASSERT_EQ(controller.properties.size(), 2U);
    EXPECT_EQ(controller.properties[0].name, "aggressiveness");
    EXPECT_EQ(controller.properties[1].name, "model");
}

TEST(ControllerIrTest, DomainMembershipMatchesControllerType) {
    // §ControllerType: movement covers both movement domains, all covers
    // everything, the appearance domains cover neither.
    EXPECT_TRUE(controls_lateral(ControllerType::Lateral));
    EXPECT_FALSE(controls_longitudinal(ControllerType::Lateral));
    EXPECT_TRUE(controls_longitudinal(ControllerType::Longitudinal));
    EXPECT_FALSE(controls_lateral(ControllerType::Longitudinal));
    EXPECT_TRUE(controls_lateral(ControllerType::Movement));
    EXPECT_TRUE(controls_longitudinal(ControllerType::Movement));
    EXPECT_TRUE(controls_lateral(ControllerType::All));
    EXPECT_TRUE(controls_longitudinal(ControllerType::All));
    for (const ControllerType type :
         {ControllerType::Lighting, ControllerType::Animation, ControllerType::Appearance}) {
        EXPECT_FALSE(controls_lateral(type));
        EXPECT_FALSE(controls_longitudinal(type));
    }
}

TEST(EntityVisibilityTest, DefaultsToVisibleEverywhere) {
    // §VisibilityAction: "The default for entities is that they are visible
    // everywhere."
    const EntityVisibility visibility;
    EXPECT_TRUE(visibility.graphics);
    EXPECT_TRUE(visibility.sensors);
    EXPECT_TRUE(visibility.traffic);
}

// --- Actions --------------------------------------------------------------

TEST(LongitudinalDistanceActionIrTest, DistanceTargetCarriesItsParameters) {
    const LongitudinalDistanceAction action("ego", "lead", 12.0, std::nullopt,
                                            /*freespace=*/true, /*continuous=*/false);
    EXPECT_EQ(action.kind(), "LongitudinalDistanceAction");
    EXPECT_EQ(action.entity_id(), "ego");
    EXPECT_EQ(action.entity_ref(), "lead");
    ASSERT_TRUE(action.distance().has_value());
    EXPECT_DOUBLE_EQ(*action.distance(), 12.0);
    EXPECT_FALSE(action.time_gap().has_value());
    EXPECT_TRUE(action.freespace());
    EXPECT_FALSE(action.continuous());
    // §CoordinateSystem: "If not provided the value is interpreted as entity";
    // §LongitudinalDisplacement: "Where omitted, trailingReferencedEntity".
    EXPECT_EQ(action.coordinate_system(), CoordinateSystem::Entity);
    EXPECT_EQ(action.displacement(), LongitudinalDisplacement::TrailingReferencedEntity);
    EXPECT_FALSE(action.constraints().has_value());
}

TEST(LongitudinalDistanceActionIrTest, TimeGapTargetWithConstraintsAndDisplacement) {
    DynamicConstraints constraints;
    constraints.max_acceleration = 2.5;
    constraints.max_deceleration = 4.0;
    constraints.max_speed = 30.0;
    const LongitudinalDistanceAction action(
        "ego", "lead", std::nullopt, 1.8, /*freespace=*/false, /*continuous=*/true,
        CoordinateSystem::World, LongitudinalDisplacement::LeadingReferencedEntity, constraints);
    ASSERT_TRUE(action.time_gap().has_value());
    EXPECT_NEAR(*action.time_gap(), 1.8, kTol);
    EXPECT_FALSE(action.distance().has_value());
    EXPECT_TRUE(action.continuous());
    EXPECT_EQ(action.coordinate_system(), CoordinateSystem::World);
    EXPECT_EQ(action.displacement(), LongitudinalDisplacement::LeadingReferencedEntity);
    ASSERT_TRUE(action.constraints().has_value());
    ASSERT_TRUE(action.constraints()->max_acceleration.has_value());
    EXPECT_DOUBLE_EQ(*action.constraints()->max_acceleration, 2.5);
    // A missing constraint means 'inf' (§DynamicConstraints), never zero.
    EXPECT_FALSE(action.constraints()->max_acceleration_rate.has_value());
}

// --- Lateral actions (p2-s3, §7.4.1.4) -------------------------------------

TEST(LaneChangeActionIrTest, RelativeTargetCarriesItsParameters) {
    const TransitionDynamics dynamics{DynamicsShape::Sinusoidal, DynamicsDimension::Time, 3.0,
                                      FollowingMode::Position};
    const LaneChangeAction action("ego", RelativeTargetLane{"lead", -1}, dynamics, 0.25);
    EXPECT_EQ(action.kind(), "LaneChangeAction");
    EXPECT_EQ(action.entity_id(), "ego");
    ASSERT_TRUE(action.is_relative());
    ASSERT_TRUE(action.relative_target().has_value());
    EXPECT_EQ(action.relative_target()->entity_ref, "lead");
    EXPECT_EQ(action.relative_target()->value, -1);
    EXPECT_FALSE(action.absolute_target().has_value());
    EXPECT_EQ(action.dynamics().shape, DynamicsShape::Sinusoidal);
    EXPECT_DOUBLE_EQ(action.dynamics().value, 3.0);
    EXPECT_DOUBLE_EQ(action.target_lane_offset(), 0.25);
}

TEST(LaneChangeActionIrTest, AbsoluteTargetAndDefaultOffset) {
    // "Missing value is interpreted as 0" (Class `LaneChangeAction`).
    const LaneChangeAction action("ego", AbsoluteTargetLane{"-2"}, TransitionDynamics{});
    EXPECT_FALSE(action.is_relative());
    ASSERT_TRUE(action.absolute_target().has_value());
    EXPECT_EQ(action.absolute_target()->value, "-2");
    EXPECT_FALSE(action.relative_target().has_value());
    EXPECT_DOUBLE_EQ(action.target_lane_offset(), 0.0);
}

TEST(LaneOffsetActionIrTest, AbsoluteAndRelativeTargets) {
    const LaneOffsetAction absolute("ego", AbsoluteTargetLaneOffset{1.5}, /*continuous=*/true,
                                    DynamicsShape::Cubic, 2.0);
    EXPECT_EQ(absolute.kind(), "LaneOffsetAction");
    EXPECT_FALSE(absolute.is_relative());
    ASSERT_TRUE(absolute.absolute_target().has_value());
    EXPECT_DOUBLE_EQ(absolute.absolute_target()->value, 1.5);
    EXPECT_TRUE(absolute.continuous());
    EXPECT_EQ(absolute.shape(), DynamicsShape::Cubic);
    ASSERT_TRUE(absolute.max_lateral_acc().has_value());
    EXPECT_DOUBLE_EQ(*absolute.max_lateral_acc(), 2.0);

    const LaneOffsetAction relative("ego", RelativeTargetLaneOffset{"lead", -0.75},
                                    /*continuous=*/false, DynamicsShape::Sinusoidal);
    EXPECT_TRUE(relative.is_relative());
    ASSERT_TRUE(relative.relative_target().has_value());
    EXPECT_EQ(relative.relative_target()->entity_ref, "lead");
    EXPECT_DOUBLE_EQ(relative.relative_target()->value, -0.75);
    // "Missing value is interpreted as 'inf'" (Class `LaneOffsetActionDynamics`).
    EXPECT_FALSE(relative.max_lateral_acc().has_value());
}

TEST(LateralDistanceActionIrTest, CarriesItsParametersAndDefaults) {
    const LateralDistanceAction action("ego", "lead", 2.0, /*freespace=*/false,
                                       /*continuous=*/true);
    EXPECT_EQ(action.kind(), "LateralDistanceAction");
    EXPECT_EQ(action.entity_ref(), "lead");
    EXPECT_DOUBLE_EQ(action.distance(), 2.0);
    EXPECT_FALSE(action.freespace());
    EXPECT_TRUE(action.continuous());
    // "If not provided the value is interpreted as entity"; "Where omitted,
    // 'any' is assumed" (Class `LateralDistanceAction`).
    EXPECT_EQ(action.coordinate_system(), CoordinateSystem::Entity);
    EXPECT_EQ(action.displacement(), LateralDisplacement::Any);
    // "Without this limiting parameters lateral distance is kept rigid."
    EXPECT_FALSE(action.constraints().has_value());
}

TEST(LateralDistanceActionIrTest, DisplacementAndConstraintsAreCarried) {
    DynamicConstraints constraints;
    constraints.max_acceleration = 1.5;
    constraints.max_speed = 2.0;
    const LateralDistanceAction action("ego", "lead", 3.5, /*freespace=*/true,
                                       /*continuous=*/false, CoordinateSystem::World,
                                       LateralDisplacement::LeftToReferencedEntity, constraints);
    EXPECT_TRUE(action.freespace());
    EXPECT_EQ(action.coordinate_system(), CoordinateSystem::World);
    EXPECT_EQ(action.displacement(), LateralDisplacement::LeftToReferencedEntity);
    ASSERT_TRUE(action.constraints().has_value());
    ASSERT_TRUE(action.constraints()->max_speed.has_value());
    EXPECT_DOUBLE_EQ(*action.constraints()->max_speed, 2.0);
}

TEST(RoutingActionIrTest, AssignRouteCarriesTheRoute) {
    Route route;
    route.name = "r1";
    route.waypoints.push_back(Waypoint{WorldPosition{0.0, 0.0, 0.0}, RouteStrategy::Shortest});
    route.waypoints.push_back(Waypoint{WorldPosition{50.0, 0.0, 0.0}, RouteStrategy::Fastest});
    const AssignRouteAction action("ego", route);
    EXPECT_EQ(action.kind(), "AssignRouteAction");
    EXPECT_EQ(action.entity_id(), "ego");
    ASSERT_EQ(action.route().waypoints.size(), 2U);
    EXPECT_EQ(action.route().name, "r1");
    EXPECT_DOUBLE_EQ(action.route().waypoints[1].position.x, 50.0);
}

TEST(RoutingActionIrTest, AcquirePositionCarriesTheTarget) {
    const AcquirePositionAction action("ego", WorldPosition{100.0, -5.0, 0.5});
    EXPECT_EQ(action.kind(), "AcquirePositionAction");
    const auto& world = std::get<scena::ir::WorldPosition>(action.position());
    EXPECT_DOUBLE_EQ(world.x, 100.0);
    EXPECT_DOUBLE_EQ(world.y, -5.0);
    EXPECT_DOUBLE_EQ(world.z, 0.5);
}

TEST(RoutingActionIrTest, FollowTrajectoryDefaultsToPositionModeWithoutTiming) {
    Trajectory trajectory;
    trajectory.name = "t1";
    trajectory.vertices().push_back(TrajectoryVertex{WorldPosition{0.0, 0.0, 0.0}, std::nullopt});
    trajectory.vertices().push_back(TrajectoryVertex{WorldPosition{10.0, 0.0, 0.0}, std::nullopt});
    const FollowTrajectoryAction action("ego", trajectory);
    EXPECT_EQ(action.kind(), "FollowTrajectoryAction");
    EXPECT_EQ(action.following_mode(), FollowingMode::Position);
    EXPECT_FALSE(action.time_reference().has_value()); // §TimeReference "None"
    EXPECT_DOUBLE_EQ(action.initial_distance_offset(), 0.0);
    ASSERT_EQ(action.trajectory().vertices().size(), 2U);
    EXPECT_FALSE(action.trajectory().closed);
}

TEST(RoutingActionIrTest, FollowTrajectoryCarriesTimingAndOffset) {
    Trajectory trajectory;
    trajectory.vertices().push_back(TrajectoryVertex{WorldPosition{0.0, 0.0, 0.0}, 0.0});
    trajectory.vertices().push_back(TrajectoryVertex{WorldPosition{10.0, 0.0, 0.0}, 2.0});
    const Timing timing{ReferenceContext::Relative, 2.0, 0.5};
    const FollowTrajectoryAction action("ego", trajectory, FollowingMode::Follow, timing, 2.5);
    ASSERT_TRUE(action.time_reference().has_value());
    EXPECT_EQ(action.time_reference()->domain, ReferenceContext::Relative);
    EXPECT_DOUBLE_EQ(action.time_reference()->scale, 2.0);
    EXPECT_DOUBLE_EQ(action.time_reference()->offset, 0.5);
    EXPECT_EQ(action.following_mode(), FollowingMode::Follow);
    EXPECT_DOUBLE_EQ(action.initial_distance_offset(), 2.5);
}

TEST(TrajectoryShapeIrTest, DefaultTrajectoryHoldsAnEmptyPolyline) {
    const Trajectory trajectory;
    ASSERT_TRUE(std::holds_alternative<scena::ir::Polyline>(trajectory.shape));
    EXPECT_TRUE(trajectory.vertices().empty());
}

TEST(TrajectoryShapeIrTest, ClothoidCarriesCurvatureAndLength) {
    scena::ir::Clothoid clothoid;
    clothoid.start = WorldPosition{1.0, 2.0, 0.0};
    clothoid.curvature = 0.05;
    clothoid.curvature_prime = 0.001;
    clothoid.length = 40.0;
    const Trajectory trajectory{"spiral", false, clothoid};
    ASSERT_TRUE(std::holds_alternative<scena::ir::Clothoid>(trajectory.shape));
    const auto& shape = std::get<scena::ir::Clothoid>(trajectory.shape);
    EXPECT_DOUBLE_EQ(shape.curvature, 0.05);
    EXPECT_DOUBLE_EQ(shape.curvature_prime, 0.001);
    EXPECT_DOUBLE_EQ(shape.length, 40.0);
    EXPECT_DOUBLE_EQ(shape.start.x, 1.0);
    EXPECT_FALSE(shape.start_time.has_value());
}

TEST(TrajectoryShapeIrTest, NurbsCarriesOrderControlPointsAndKnots) {
    scena::ir::Nurbs nurbs;
    nurbs.order = 3;
    nurbs.control_points.push_back(
        scena::ir::ControlPoint{WorldPosition{0.0, 0.0, 0.0}, std::nullopt, 1.0});
    nurbs.control_points.push_back(
        scena::ir::ControlPoint{WorldPosition{1.0, 1.0, 0.0}, std::nullopt, 0.5});
    nurbs.control_points.push_back(
        scena::ir::ControlPoint{WorldPosition{2.0, 0.0, 0.0}, std::nullopt, 1.0});
    nurbs.knots = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};
    const Trajectory trajectory{"curve", false, nurbs};
    ASSERT_TRUE(std::holds_alternative<scena::ir::Nurbs>(trajectory.shape));
    const auto& shape = std::get<scena::ir::Nurbs>(trajectory.shape);
    EXPECT_EQ(shape.order, 3U);
    ASSERT_EQ(shape.control_points.size(), 3U);
    EXPECT_DOUBLE_EQ(shape.control_points[1].weight, 0.5);
    // knots.size() == control_points.size() + order.
    EXPECT_EQ(shape.knots.size(), shape.control_points.size() + shape.order);
}

TEST(ControllerActionIrTest, AssignControllerCarriesActivationFlags) {
    Controller controller;
    controller.name = "driver";
    controller.type = ControllerType::Movement;
    controller.properties.push_back(Property{"k", "v"});
    const AssignControllerAction action("ego", controller, /*activate_lateral=*/std::nullopt,
                                        /*activate_longitudinal=*/true);
    EXPECT_EQ(action.kind(), "AssignControllerAction");
    EXPECT_EQ(action.controller().name, "driver");
    ASSERT_EQ(action.controller().properties.size(), 1U);
    // "If not specified: No change for controlling the dimension is applied."
    EXPECT_FALSE(action.activate_lateral().has_value());
    ASSERT_TRUE(action.activate_longitudinal().has_value());
    EXPECT_TRUE(*action.activate_longitudinal());
}

TEST(ControllerActionIrTest, ActivateControllerCarriesTriStateDomains) {
    const ActivateControllerAction action("ego", /*lateral=*/false,
                                          /*longitudinal=*/std::nullopt);
    EXPECT_EQ(action.kind(), "ActivateControllerAction");
    ASSERT_TRUE(action.lateral().has_value());
    EXPECT_FALSE(*action.lateral());
    EXPECT_FALSE(action.longitudinal().has_value());
}

TEST(VisibilityActionIrTest, CarriesAllThreeFlags) {
    const VisibilityAction action("ego", /*graphics=*/true, /*sensors=*/false, /*traffic=*/false);
    EXPECT_EQ(action.kind(), "VisibilityAction");
    EXPECT_EQ(action.entity_id(), "ego");
    EXPECT_TRUE(action.graphics());
    EXPECT_FALSE(action.sensors());
    EXPECT_FALSE(action.traffic());
}

TEST(ActionIrTest, EveryNewKindIsTheStableAsamElementName) {
    // kind() feeds runtime diagnostics and must stay the ASAM element name.
    const std::vector<std::pair<std::string, std::string>> expected = {
        {LongitudinalDistanceAction("e", "r", 1.0, std::nullopt, false, false).kind().data(),
         "LongitudinalDistanceAction"},
        {AssignRouteAction("e", Route{}).kind().data(), "AssignRouteAction"},
        {AcquirePositionAction("e", WorldPosition{}).kind().data(), "AcquirePositionAction"},
        {FollowTrajectoryAction("e", Trajectory{}).kind().data(), "FollowTrajectoryAction"},
        {AssignControllerAction("e", Controller{}).kind().data(), "AssignControllerAction"},
        {ActivateControllerAction("e", std::nullopt, std::nullopt).kind().data(),
         "ActivateControllerAction"},
        {VisibilityAction("e", true, true, true).kind().data(), "VisibilityAction"},
        {LaneChangeAction("e", RelativeTargetLane{"r", 1}, TransitionDynamics{}).kind().data(),
         "LaneChangeAction"},
        {LaneOffsetAction("e", AbsoluteTargetLaneOffset{}, false, DynamicsShape::Step)
             .kind()
             .data(),
         "LaneOffsetAction"},
        {LateralDistanceAction("e", "r", 0.0, false, false).kind().data(), "LateralDistanceAction"},
    };
    for (const auto& [actual, name] : expected) {
        EXPECT_EQ(actual, name);
    }
}

// --- Global actions (p5-s6, §7.4.2) ---------------------------------------

TEST(GlobalActionIrTest, GlobalActionsCarryNoEntityId) {
    // The actor-less base finalizes entity_id() to a shared empty string, so a
    // call site that takes a reference stays valid; dispatch must branch on the
    // GlobalAction type, never on the emptiness.
    const VariableSetAction set("v", "1");
    const VariableModifyAction modify("v", ModifyOperator::Add, 2.0);
    const ParameterSetAction parameter_set("p", "x");
    const ParameterModifyAction parameter_modify("p", ModifyOperator::Multiply, 3.0);
    for (const Action* action :
         {static_cast<const Action*>(&set), static_cast<const Action*>(&modify),
          static_cast<const Action*>(&parameter_set),
          static_cast<const Action*>(&parameter_modify)}) {
        EXPECT_TRUE(action->entity_id().empty());
        EXPECT_NE(dynamic_cast<const GlobalAction*>(action), nullptr);
    }
    // A private action is not a global one — the branch that matters.
    const VisibilityAction visibility("ego", true, true, true);
    EXPECT_EQ(dynamic_cast<const GlobalAction*>(static_cast<const Action*>(&visibility)), nullptr);
}

TEST(GlobalActionIrTest, VariableAndParameterActionAccessors) {
    const VariableSetAction set("trigger", "true");
    EXPECT_EQ(set.variable_ref(), "trigger");
    EXPECT_EQ(set.value(), "true");

    const VariableModifyAction modify("counter", ModifyOperator::Multiply, 2.5);
    EXPECT_EQ(modify.variable_ref(), "counter");
    EXPECT_EQ(modify.op(), ModifyOperator::Multiply);
    EXPECT_DOUBLE_EQ(modify.value(), 2.5);

    const ParameterSetAction parameter_set("speedLimit", "30");
    EXPECT_EQ(parameter_set.parameter_ref(), "speedLimit");
    EXPECT_EQ(parameter_set.value(), "30");

    const ParameterModifyAction parameter_modify("speedLimit", ModifyOperator::Add, -4.0);
    EXPECT_EQ(parameter_modify.parameter_ref(), "speedLimit");
    EXPECT_EQ(parameter_modify.op(), ModifyOperator::Add);
    EXPECT_DOUBLE_EQ(parameter_modify.value(), -4.0);
}

TEST(GlobalActionIrTest, GlobalKindsAreTheStableAsamElementNames) {
    EXPECT_EQ(VariableSetAction("v", "1").kind(), "VariableSetAction");
    EXPECT_EQ(VariableModifyAction("v", ModifyOperator::Add, 1.0).kind(), "VariableModifyAction");
    EXPECT_EQ(ParameterSetAction("p", "1").kind(), "ParameterSetAction");
    EXPECT_EQ(ParameterModifyAction("p", ModifyOperator::Add, 1.0).kind(), "ParameterModifyAction");
}

// --- format_scalar (the write side of the variable store) ------------------

TEST(FormatScalarTest, RoundTripsEveryFiniteDoubleBitForBit) {
    // The round-trip is what lets a VariableModifyAction write its result back
    // into the string-valued store without drift.
    const double values[] = {0.0,
                             -0.0,
                             1.0,
                             -1.0,
                             0.1,
                             1.0 / 3.0,
                             1e-300,
                             1e300,
                             123456789.123456789,
                             std::numeric_limits<double>::min(),
                             std::numeric_limits<double>::max(),
                             std::numeric_limits<double>::denorm_min()};
    for (const double value : values) {
        const std::string text = scena::ir::format_scalar(value);
        const std::optional<double> parsed = scena::ir::parse_scalar(text);
        ASSERT_TRUE(parsed.has_value()) << text;
        EXPECT_EQ(*parsed, value) << text;
    }
}

TEST(FormatScalarTest, EmitsTheShortestRoundTripForm) {
    // Pinned exactly: these strings are what a scenario's variable store holds
    // after a modify action, and a diagnostic or a trace may quote them.
    EXPECT_EQ(scena::ir::format_scalar(0.1 + 0.2), "0.30000000000000004");
    EXPECT_EQ(scena::ir::format_scalar(1.0), "1");
    EXPECT_EQ(scena::ir::format_scalar(-0.0), "-0");
    EXPECT_EQ(scena::ir::format_scalar(2.5), "2.5");
    EXPECT_EQ(scena::ir::format_scalar(100.0), "100");
    // Locale-independent: never a comma, never std::to_string's six-digit pad.
    EXPECT_EQ(scena::ir::format_scalar(1.5), "1.5");
}

TEST(FormatScalarTest, NonFiniteValuesRoundTripThroughParseScalar) {
    const double infinity = std::numeric_limits<double>::infinity();
    EXPECT_EQ(scena::ir::format_scalar(infinity), "inf");
    EXPECT_EQ(scena::ir::format_scalar(-infinity), "-inf");
    const std::optional<double> parsed =
        scena::ir::parse_scalar(scena::ir::format_scalar(infinity));
    ASSERT_TRUE(parsed.has_value());
    EXPECT_EQ(*parsed, infinity);
    // NaN formats and parses, but never compares equal (IEEE ordering).
    const std::optional<double> nan_parsed =
        scena::ir::parse_scalar(scena::ir::format_scalar(std::numeric_limits<double>::quiet_NaN()));
    ASSERT_TRUE(nan_parsed.has_value());
    EXPECT_TRUE(std::isnan(*nan_parsed));
}

} // namespace
