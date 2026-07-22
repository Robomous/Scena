// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
//
// The p5-s5 action IR surface: construction, accessors, defaults, and the
// stable ASAM kind() names of the routing, distance-keeping, controller and
// visibility actions. Runtime behavior lives in action_distance_test.cpp,
// action_routing_test.cpp and action_controller_visibility_test.cpp.

#include <optional>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "scena/entity_visibility.h"
#include "scena/ir/action.h"
#include "scena/ir/controller.h"
#include "scena/ir/coordinate_system.h"
#include "scena/ir/route.h"
#include "scena/ir/trajectory.h"

namespace {

using scena::EntityVisibility;
using scena::ir::AcquirePositionAction;
using scena::ir::ActivateControllerAction;
using scena::ir::AssignControllerAction;
using scena::ir::AssignRouteAction;
using scena::ir::Controller;
using scena::ir::ControllerType;
using scena::ir::controls_lateral;
using scena::ir::controls_longitudinal;
using scena::ir::CoordinateSystem;
using scena::ir::DynamicConstraints;
using scena::ir::FollowingMode;
using scena::ir::FollowTrajectoryAction;
using scena::ir::LongitudinalDisplacement;
using scena::ir::LongitudinalDistanceAction;
using scena::ir::Property;
using scena::ir::ReferenceContext;
using scena::ir::Route;
using scena::ir::RouteStrategy;
using scena::ir::Timing;
using scena::ir::Trajectory;
using scena::ir::TrajectoryVertex;
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
    EXPECT_DOUBLE_EQ(action.position().x, 100.0);
    EXPECT_DOUBLE_EQ(action.position().y, -5.0);
    EXPECT_DOUBLE_EQ(action.position().z, 0.5);
}

TEST(RoutingActionIrTest, FollowTrajectoryDefaultsToPositionModeWithoutTiming) {
    Trajectory trajectory;
    trajectory.name = "t1";
    trajectory.vertices.push_back(TrajectoryVertex{WorldPosition{0.0, 0.0, 0.0}, std::nullopt});
    trajectory.vertices.push_back(TrajectoryVertex{WorldPosition{10.0, 0.0, 0.0}, std::nullopt});
    const FollowTrajectoryAction action("ego", trajectory);
    EXPECT_EQ(action.kind(), "FollowTrajectoryAction");
    EXPECT_EQ(action.following_mode(), FollowingMode::Position);
    EXPECT_FALSE(action.time_reference().has_value()); // §TimeReference "None"
    EXPECT_DOUBLE_EQ(action.initial_distance_offset(), 0.0);
    ASSERT_EQ(action.trajectory().vertices.size(), 2U);
    EXPECT_FALSE(action.trajectory().closed);
}

TEST(RoutingActionIrTest, FollowTrajectoryCarriesTimingAndOffset) {
    Trajectory trajectory;
    trajectory.vertices.push_back(TrajectoryVertex{WorldPosition{0.0, 0.0, 0.0}, 0.0});
    trajectory.vertices.push_back(TrajectoryVertex{WorldPosition{10.0, 0.0, 0.0}, 2.0});
    const Timing timing{ReferenceContext::Relative, 2.0, 0.5};
    const FollowTrajectoryAction action("ego", trajectory, FollowingMode::Follow, timing, 2.5);
    ASSERT_TRUE(action.time_reference().has_value());
    EXPECT_EQ(action.time_reference()->domain, ReferenceContext::Relative);
    EXPECT_DOUBLE_EQ(action.time_reference()->scale, 2.0);
    EXPECT_DOUBLE_EQ(action.time_reference()->offset, 0.5);
    EXPECT_EQ(action.following_mode(), FollowingMode::Follow);
    EXPECT_DOUBLE_EQ(action.initial_distance_offset(), 2.5);
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
    };
    for (const auto& [actual, name] : expected) {
        EXPECT_EQ(actual, name);
    }
}

} // namespace
