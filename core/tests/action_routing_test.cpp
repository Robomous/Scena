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
// The routing actions at the engine level (p5-s5): AssignRouteAction and
// AcquirePositionAction install routing state and end immediately (Annex A
// Table 10), while FollowTrajectoryAction moves the entity along a polyline —
// time-free (the entity's own speed sets the pace) or timed (the vertex times
// do, and the action owns the longitudinal domain).

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
#include "scena/ir/dynamics.h"
#include "scena/ir/entity.h"
#include "scena/ir/position.h"
#include "scena/ir/route.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/trajectory.h"
#include "scena/ir/trigger.h"

namespace {

using scena::Engine;
using scena::Status;
using scena::ir::AcquirePositionAction;
using scena::ir::AssignRouteAction;
using scena::ir::ControlMode;
using scena::ir::DynamicsDimension;
using scena::ir::DynamicsShape;
using scena::ir::Entity;
using scena::ir::FollowingMode;
using scena::ir::FollowTrajectoryAction;
using scena::ir::ReferenceContext;
using scena::ir::Route;
using scena::ir::RouteStrategy;
using scena::ir::SimulationTimeCondition;
using scena::ir::SpeedAction;
using scena::ir::TeleportAction;
using scena::ir::Timing;
using scena::ir::Trajectory;
using scena::ir::TrajectoryVertex;
using scena::ir::TransitionDynamics;
using scena::ir::VisibilityAction;
using scena::ir::Waypoint;
using scena::ir::WorldPosition;

constexpr double kTol = 1e-9;
constexpr double kHalfPi = 1.57079632679489661;

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

/// One engine-controlled entity "ego" at the origin with `speed`, plus a
/// maneuver holding `events` in document order.
scena::ir::Scenario make_scenario(double speed, std::vector<scena::ir::Event> events) {
    scena::ir::Scenario scenario;
    scenario.name = "routing";
    Entity ego;
    ego.id = "ego";
    ego.name = "ego";
    ego.control_mode = ControlMode::EngineControlled;
    scenario.entities.push_back(std::move(ego));
    scenario.init_actions.push_back(
        std::make_shared<TeleportAction>("ego", WorldPosition{0.0, 0.0, 0.0}));
    scenario.init_actions.push_back(std::make_shared<SpeedAction>("ego", speed));

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

std::string path_of(const std::string& event_name) {
    return "story/act/group/maneuver/" + event_name;
}

/// A two-segment polyline: (0,0) -> (100,0) -> (100,100), so the arc length is
/// 200 m and the headings are 0 and pi/2.
Trajectory make_corner_trajectory(std::optional<double> t0 = std::nullopt,
                                  std::optional<double> t1 = std::nullopt,
                                  std::optional<double> t2 = std::nullopt) {
    Trajectory trajectory;
    trajectory.name = "corner";
    trajectory.vertices.push_back(TrajectoryVertex{WorldPosition{0.0, 0.0, 0.0}, t0});
    trajectory.vertices.push_back(TrajectoryVertex{WorldPosition{100.0, 0.0, 0.0}, t1});
    trajectory.vertices.push_back(TrajectoryVertex{WorldPosition{100.0, 100.0, 0.0}, t2});
    return trajectory;
}

// --- AssignRouteAction / AcquirePositionAction ----------------------------

TEST(RoutingActionTest, AssignRouteCompletesImmediatelyAndStoresTheRoute) {
    Route route;
    route.name = "r1";
    route.waypoints.push_back(Waypoint{WorldPosition{10.0, 0.0, 0.0}, RouteStrategy::Fastest});
    route.waypoints.push_back(Waypoint{WorldPosition{90.0, 20.0, 0.0}, RouteStrategy::Shortest});
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event("assign", 1.0, std::make_shared<AssignRouteAction>("ego", route)));
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(10.0, std::move(events))), Status::Ok);
    EXPECT_EQ(engine.route_of("ego"), nullptr); // nothing assigned yet

    ASSERT_EQ(engine.step(1.0), Status::Ok);
    // Table 10: "immediately (does not consume simulation time)".
    EXPECT_EQ(*engine.storyboard_element_state(path_of("assign")),
              scena::runtime::ElementState::Complete);
    const Route* assigned = engine.route_of("ego");
    ASSERT_NE(assigned, nullptr);
    EXPECT_EQ(assigned->name, "r1");
    ASSERT_EQ(assigned->waypoints.size(), 2U);
    EXPECT_DOUBLE_EQ(assigned->waypoints[0].position.x, 10.0);
    EXPECT_EQ(assigned->waypoints[0].strategy, RouteStrategy::Fastest);
    EXPECT_DOUBLE_EQ(assigned->waypoints[1].position.y, 20.0);
}

TEST(RoutingActionTest, ALaterRouteOverwritesTheEarlierOne) {
    // §6.8.2: an assigned route stays "until another action overwrites them".
    Route first;
    first.name = "first";
    first.waypoints.push_back(Waypoint{WorldPosition{1.0, 0.0, 0.0}, RouteStrategy::Shortest});
    first.waypoints.push_back(Waypoint{WorldPosition{2.0, 0.0, 0.0}, RouteStrategy::Shortest});
    Route second = first;
    second.name = "second";
    second.waypoints.push_back(Waypoint{WorldPosition{3.0, 0.0, 0.0}, RouteStrategy::Random});

    std::vector<scena::ir::Event> events;
    events.push_back(timed_event("first", 1.0, std::make_shared<AssignRouteAction>("ego", first)));
    events.push_back(
        timed_event("second", 2.0, std::make_shared<AssignRouteAction>("ego", second)));
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(0.0, std::move(events))), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    ASSERT_NE(engine.route_of("ego"), nullptr);
    EXPECT_EQ(engine.route_of("ego")->name, "first");
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    ASSERT_NE(engine.route_of("ego"), nullptr);
    EXPECT_EQ(engine.route_of("ego")->name, "second");
    EXPECT_EQ(engine.route_of("ego")->waypoints.size(), 3U);
}

TEST(RoutingActionTest, AcquirePositionBuildsTheImplicitTwoWaypointRoute) {
    // §7.4.1.4: "a route with two waypoints is created: current position as
    // first and specified position as last waypoint", with the entity aiming
    // for the shortest distance.
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event(
        "acquire", 3.0,
        std::make_shared<AcquirePositionAction>("ego", WorldPosition{500.0, -25.0, 1.0})));
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(10.0, std::move(events))), Status::Ok);
    for (int i = 0; i < 3; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    EXPECT_EQ(*engine.storyboard_element_state(path_of("acquire")),
              scena::runtime::ElementState::Complete);
    const Route* route = engine.route_of("ego");
    ASSERT_NE(route, nullptr);
    ASSERT_EQ(route->waypoints.size(), 2U);
    // The first waypoint is the position at apply time: 3 steps at 10 m/s, and
    // the storyboard runs before the integrate phase, so 20 m.
    EXPECT_NEAR(route->waypoints[0].position.x, 20.0, kTol);
    EXPECT_EQ(route->waypoints[0].strategy, RouteStrategy::Shortest);
    EXPECT_DOUBLE_EQ(route->waypoints[1].position.x, 500.0);
    EXPECT_DOUBLE_EQ(route->waypoints[1].position.y, -25.0);
    EXPECT_EQ(route->waypoints[1].strategy, RouteStrategy::Shortest);
}

TEST(RoutingActionTest, AcquireResolvesARelativePositionIntoTheLastWaypoint) {
    // AcquirePosition now accepts any §6.3.8 Position: a RelativeObjectPosition
    // 100 m ahead of ego (heading 0 => +X) resolves into the route's last
    // waypoint through the PositionResolver (ADR-0017).
    scena::ir::RelativeObjectPosition ahead;
    ahead.entity_ref = "ego";
    ahead.dx = 100.0;
    std::vector<scena::ir::Event> events;
    events.push_back(
        timed_event("acquire", 3.0, std::make_shared<AcquirePositionAction>("ego", ahead)));
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(10.0, std::move(events))), Status::Ok);
    for (int i = 0; i < 3; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    const Route* route = engine.route_of("ego");
    ASSERT_NE(route, nullptr);
    ASSERT_EQ(route->waypoints.size(), 2U);
    // ego is at x = 20 m at apply time (storyboard runs before integrate); the
    // target is 100 m ahead along its +X heading.
    EXPECT_NEAR(route->waypoints[1].position.x, 120.0, kTol);
    EXPECT_NEAR(route->waypoints[1].position.y, 0.0, kTol);
}

TEST(RoutingActionTest, RouteAssignmentLeavesARunningSpeedRampAlone) {
    // §7.4.1.4: AssignRouteAction "does not override any action that controls
    // either lateral or longitudinal domain".
    Route route;
    route.waypoints.push_back(Waypoint{WorldPosition{0.0, 0.0, 0.0}, RouteStrategy::Shortest});
    route.waypoints.push_back(Waypoint{WorldPosition{9.0, 0.0, 0.0}, RouteStrategy::Shortest});
    std::vector<scena::ir::Event> events;
    events.push_back(
        timed_event("ramp", 0.0,
                    std::make_shared<SpeedAction>(
                        "ego", 20.0,
                        TransitionDynamics{DynamicsShape::Linear, DynamicsDimension::Time, 10.0})));
    events.push_back(timed_event("assign", 2.0, std::make_shared<AssignRouteAction>("ego", route)));
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(10.0, std::move(events))), Status::Ok);
    for (int i = 0; i < 5; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    // The ramp is still running and still on schedule: 10 -> 20 over 10 s.
    EXPECT_EQ(*engine.storyboard_element_state(path_of("ramp")),
              scena::runtime::ElementState::Running);
    EXPECT_NEAR(engine.state("ego")->speed, 15.0, kTol);
    EXPECT_NE(engine.route_of("ego"), nullptr);
}

TEST(RoutingActionValidationTest, RouteNeedsTwoFiniteWaypoints) {
    Route single;
    single.waypoints.push_back(Waypoint{WorldPosition{1.0, 0.0, 0.0}, RouteStrategy::Shortest});
    std::vector<scena::ir::Event> events;
    events.push_back(
        timed_event("assign", 0.0, std::make_shared<AssignRouteAction>("ego", single)));
    Engine engine;
    EXPECT_EQ(engine.init(make_scenario(0.0, std::move(events))), Status::ValidationError);

    Route infinite;
    infinite.waypoints.push_back(Waypoint{WorldPosition{0.0, 0.0, 0.0}, RouteStrategy::Shortest});
    infinite.waypoints.push_back(Waypoint{
        WorldPosition{std::numeric_limits<double>::infinity(), 0.0, 0.0}, RouteStrategy::Shortest});
    std::vector<scena::ir::Event> bad_events;
    bad_events.push_back(
        timed_event("assign", 0.0, std::make_shared<AssignRouteAction>("ego", infinite)));
    Engine bad;
    EXPECT_EQ(bad.init(make_scenario(0.0, std::move(bad_events))), Status::ValidationError);
}

TEST(RoutingActionValidationTest, AcquirePositionRejectsANonFiniteTarget) {
    std::vector<scena::ir::Event> events;
    events.push_back(
        timed_event("acquire", 0.0,
                    std::make_shared<AcquirePositionAction>(
                        "ego", WorldPosition{0.0, std::numeric_limits<double>::quiet_NaN(), 0.0})));
    Engine engine;
    EXPECT_EQ(engine.init(make_scenario(0.0, std::move(events))), Status::ValidationError);
}

TEST(RoutingActionValidationTest, RoutingActionsNeedAKnownActor) {
    Route route;
    route.waypoints.push_back(Waypoint{WorldPosition{0.0, 0.0, 0.0}, RouteStrategy::Shortest});
    route.waypoints.push_back(Waypoint{WorldPosition{5.0, 0.0, 0.0}, RouteStrategy::Shortest});
    std::vector<scena::ir::Event> events;
    events.push_back(
        timed_event("assign", 0.0, std::make_shared<AssignRouteAction>("ghost", route)));
    Engine engine;
    EXPECT_EQ(engine.init(make_scenario(0.0, std::move(events))), Status::SemanticError);
}

// --- FollowTrajectoryAction, timeReference = none -------------------------

TEST(FollowTrajectoryTest, TimeFreeModeTeleportsToTheStartAndFollowsTheShape) {
    // §6.9.1: with no time reference the entity teleports to the beginning of
    // the trajectory; from there its own speed sets the pace (Table 10 assigns
    // no longitudinal strategy).
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event(
        "follow", 1.0, std::make_shared<FollowTrajectoryAction>("ego", make_corner_trajectory())));
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(10.0, std::move(events))), Status::Ok);

    ASSERT_EQ(engine.step(1.0), Status::Ok); // fires: teleport to (0, 0)
    EXPECT_NEAR(engine.state("ego")->x, 0.0, kTol);
    EXPECT_NEAR(engine.state("ego")->y, 0.0, kTol);
    EXPECT_NEAR(engine.state("ego")->heading, 0.0, kTol);

    for (int i = 0; i < 5; ++i) { // 50 m along the first segment
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    EXPECT_NEAR(engine.state("ego")->x, 50.0, kTol);
    EXPECT_NEAR(engine.state("ego")->y, 0.0, kTol);
    EXPECT_NEAR(engine.state("ego")->heading, 0.0, kTol);
    EXPECT_NEAR(engine.state("ego")->speed, 10.0, kTol); // untouched by the follower

    for (int i = 0; i < 7; ++i) { // past the corner: 120 m of arc
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    EXPECT_NEAR(engine.state("ego")->x, 100.0, kTol);
    EXPECT_NEAR(engine.state("ego")->y, 20.0, kTol);
    // The second segment runs along +y: heading pi/2, via det_atan2.
    EXPECT_NEAR(engine.state("ego")->heading, kHalfPi, 1e-12);
    EXPECT_EQ(*engine.storyboard_element_state(path_of("follow")),
              scena::runtime::ElementState::Running);
}

TEST(FollowTrajectoryTest, EndsExactlyOnTheFinalVertex) {
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event(
        "follow", 0.0, std::make_shared<FollowTrajectoryAction>("ego", make_corner_trajectory())));
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(10.0, std::move(events))), Status::Ok);
    for (int i = 0; i < 20; ++i) { // 200 m of arc at 10 m/s
        SCOPED_TRACE(i);
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    // Table 10: ends "by reaching the end of the trajectory", snapped exactly.
    EXPECT_EQ(engine.state("ego")->x, 100.0);
    EXPECT_EQ(engine.state("ego")->y, 100.0);
    EXPECT_EQ(*engine.storyboard_element_state(path_of("follow")),
              scena::runtime::ElementState::Complete);

    // Released: the straight-line integrator has the entity back.
    const double y_before = engine.state("ego")->y;
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_NEAR(engine.state("ego")->y, y_before + 10.0, kTol);
}

TEST(FollowTrajectoryTest, InitialDistanceOffsetTruncatesTheTrajectory) {
    // §initialDistanceOffset: "the resulting new trajectory starts at that
    // distance offset".
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event("follow", 0.0,
                                 std::make_shared<FollowTrajectoryAction>(
                                     "ego", make_corner_trajectory(), FollowingMode::Position,
                                     std::nullopt, /*initial_distance_offset=*/60.0)));
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(10.0, std::move(events))), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    // Placed 60 m along, then advanced 10 m by this step's re-poll.
    EXPECT_NEAR(engine.state("ego")->x, 70.0, kTol);
    for (int i = 0; i < 13; ++i) { // 60 + 140 = the full 200 m of arc
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    EXPECT_EQ(*engine.storyboard_element_state(path_of("follow")),
              scena::runtime::ElementState::Complete);
    EXPECT_EQ(engine.state("ego")->y, 100.0);
}

TEST(FollowTrajectoryTest, TimeFreeModeAdvancesWithTheCurrentSpeed) {
    // A SpeedAction fired mid-trajectory changes the rate of advance: the
    // time-free mode leaves the longitudinal domain to whoever owns it.
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event(
        "follow", 0.0, std::make_shared<FollowTrajectoryAction>("ego", make_corner_trajectory())));
    events.push_back(timed_event("slow", 3.0, std::make_shared<SpeedAction>("ego", 2.0)));
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(10.0, std::move(events))), Status::Ok);
    for (int i = 0; i < 3; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    EXPECT_NEAR(engine.state("ego")->x, 30.0, kTol);
    ASSERT_EQ(engine.step(1.0), Status::Ok); // the SpeedAction lands first
    EXPECT_NEAR(engine.state("ego")->speed, 2.0, kTol);
    EXPECT_NEAR(engine.state("ego")->x, 32.0, kTol);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_NEAR(engine.state("ego")->x, 34.0, kTol);
}

// --- FollowTrajectoryAction, timeReference = timing -----------------------

TEST(FollowTrajectoryTest, AbsoluteTimingDrivesThePositionAndSpeed) {
    // Vertices at t = 0, 10, 20 over 100 m segments ⇒ 10 m/s along each.
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event(
        "follow", 0.0,
        std::make_shared<FollowTrajectoryAction>("ego", make_corner_trajectory(0.0, 10.0, 20.0),
                                                 FollowingMode::Position,
                                                 Timing{ReferenceContext::Absolute, 1.0, 0.0})));
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(0.0, std::move(events))), Status::Ok);
    for (int i = 0; i < 5; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    EXPECT_NEAR(engine.state("ego")->x, 50.0, kTol);
    EXPECT_NEAR(engine.state("ego")->y, 0.0, kTol);
    EXPECT_NEAR(engine.state("ego")->speed, 10.0, kTol); // the action drives it
    for (int i = 0; i < 10; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    EXPECT_NEAR(engine.state("ego")->x, 100.0, kTol);
    EXPECT_NEAR(engine.state("ego")->y, 50.0, kTol);
    EXPECT_NEAR(engine.state("ego")->heading, kHalfPi, 1e-12);
    for (int i = 0; i < 5; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    EXPECT_EQ(engine.state("ego")->y, 100.0);
    EXPECT_EQ(*engine.storyboard_element_state(path_of("follow")),
              scena::runtime::ElementState::Complete);
}

TEST(FollowTrajectoryTest, RelativeTimingWithScaleAndOffsetShiftsTheSchedule) {
    // domain=relative measures from the action's start (t = 2), scale 2 doubles
    // every vertex time and offset 1 shifts them: t' = t*2 + 1 + 2, so the
    // vertices land at t = 3, 23, 43.
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event(
        "follow", 2.0,
        std::make_shared<FollowTrajectoryAction>("ego", make_corner_trajectory(0.0, 10.0, 20.0),
                                                 FollowingMode::Position,
                                                 Timing{ReferenceContext::Relative, 2.0, 1.0})));
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(0.0, std::move(events))), Status::Ok);
    for (int i = 0; i < 2; ++i) { // t = 2: before the shifted start at t = 3
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    EXPECT_NEAR(engine.state("ego")->x, 0.0, kTol);
    for (int i = 0; i < 11; ++i) { // t = 13, i.e. 10 s into a 20 s first leg
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    EXPECT_NEAR(engine.state("ego")->x, 50.0, kTol);
    EXPECT_NEAR(engine.state("ego")->speed, 5.0, kTol); // 100 m over 20 s
}

TEST(FollowTrajectoryTest, AFutureTimeReferenceLetsTheEntityKeepMovingFirst) {
    // §6.9.2: the action starts, "the entity keeps moving as before until t1.
    // At t = t1, the entity teleports to the start of the trajectory".
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event(
        "follow", 1.0,
        std::make_shared<FollowTrajectoryAction>("ego", make_corner_trajectory(5.0, 15.0, 25.0),
                                                 FollowingMode::Position,
                                                 Timing{ReferenceContext::Absolute, 1.0, 0.0})));
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(10.0, std::move(events))), Status::Ok);
    for (int i = 0; i < 4; ++i) { // t = 4, still before t1 = 5
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    EXPECT_NEAR(engine.state("ego")->x, 40.0, kTol); // kept moving as before
    ASSERT_EQ(engine.step(1.0), Status::Ok);         // t = 5: teleport to the start
    EXPECT_NEAR(engine.state("ego")->x, 0.0, kTol);
    EXPECT_NEAR(engine.state("ego")->y, 0.0, kTol);
}

TEST(FollowTrajectoryTest, APastTimeReferenceJoinsAtTheInterpolatedPoint) {
    // §6.9.3: started after t1, "the entity behaves as if the trajectory starts
    // at the position interpolated between the most recent time reference and
    // the earliest future time reference".
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event(
        "follow", 4.0,
        std::make_shared<FollowTrajectoryAction>("ego", make_corner_trajectory(0.0, 10.0, 20.0),
                                                 FollowingMode::Position,
                                                 Timing{ReferenceContext::Absolute, 1.0, 0.0})));
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(10.0, std::move(events))), Status::Ok);
    for (int i = 0; i < 4; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    // At t = 4 the trajectory is 40 % along its first segment.
    EXPECT_NEAR(engine.state("ego")->x, 40.0, kTol);
    EXPECT_NEAR(engine.state("ego")->y, 0.0, kTol);
    EXPECT_NEAR(engine.state("ego")->speed, 10.0, kTol);
}

TEST(FollowTrajectoryTest, TimedTrajectorySupersedesARunningSpeedRampButTimeFreeDoesNot) {
    std::vector<scena::ir::Event> events;
    events.push_back(
        timed_event("ramp", 0.0,
                    std::make_shared<SpeedAction>(
                        "ego", 30.0,
                        TransitionDynamics{DynamicsShape::Linear, DynamicsDimension::Time, 20.0})));
    events.push_back(timed_event(
        "free", 2.0, std::make_shared<FollowTrajectoryAction>("ego", make_corner_trajectory())));
    events.push_back(timed_event(
        "timed", 5.0,
        std::make_shared<FollowTrajectoryAction>("ego", make_corner_trajectory(5.0, 45.0, 85.0),
                                                 FollowingMode::Position,
                                                 Timing{ReferenceContext::Absolute, 1.0, 0.0})));
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario(10.0, std::move(events))), Status::Ok);

    for (int i = 0; i < 4; ++i) { // t = 4: the time-free follower is running
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    // The ramp still owns the longitudinal domain — a time-free trajectory
    // takes only the lateral one (Table 10).
    EXPECT_EQ(*engine.storyboard_element_state(path_of("ramp")),
              scena::runtime::ElementState::Running);
    EXPECT_NEAR(engine.state("ego")->speed, 14.0, kTol);

    ASSERT_EQ(engine.step(1.0), Status::Ok); // t = 5: the timed trajectory takes over
    // Both the superseded follower and the superseded ramp report Complete on
    // their next re-poll (the p5-s4 retirement path).
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(*engine.storyboard_element_state(path_of("free")),
              scena::runtime::ElementState::Complete);
    EXPECT_EQ(*engine.storyboard_element_state(path_of("ramp")),
              scena::runtime::ElementState::Complete);
    // 100 m over 40 s of timed first segment.
    EXPECT_NEAR(engine.state("ego")->speed, 2.5, kTol);
}

// --- Validation -----------------------------------------------------------

scena::ir::Scenario make_trajectory_scenario(std::shared_ptr<scena::ir::Action> action) {
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event("follow", 0.0, std::move(action)));
    return make_scenario(10.0, std::move(events));
}

TEST(FollowTrajectoryValidationTest, NeedsAtLeastTwoVertices) {
    Trajectory single;
    single.vertices.push_back(TrajectoryVertex{WorldPosition{0.0, 0.0, 0.0}, std::nullopt});
    Engine engine;
    EXPECT_EQ(engine.init(make_trajectory_scenario(
                  std::make_shared<FollowTrajectoryAction>("ego", single))),
              Status::ValidationError);
}

TEST(FollowTrajectoryValidationTest, TimingRequiresATimeOnEveryVertex) {
    // per rule asam.net:xosc:1.0.0:routing.trajectory_timing_exists_if_requested
    Engine engine;
    EXPECT_EQ(engine.init(make_trajectory_scenario(std::make_shared<FollowTrajectoryAction>(
                  "ego", make_corner_trajectory(0.0, std::nullopt, 20.0), FollowingMode::Position,
                  Timing{ReferenceContext::Absolute, 1.0, 0.0}))),
              Status::ValidationError);
    ASSERT_FALSE(engine.diagnostics().empty());
    EXPECT_EQ(engine.diagnostics().front().rule_id,
              "asam.net:xosc:1.0.0:routing.trajectory_timing_exists_if_requested");
}

TEST(FollowTrajectoryValidationTest, TimesMustIncreaseAndScaleMustBePositive) {
    Engine decreasing;
    EXPECT_EQ(decreasing.init(make_trajectory_scenario(std::make_shared<FollowTrajectoryAction>(
                  "ego", make_corner_trajectory(0.0, 20.0, 10.0), FollowingMode::Position,
                  Timing{ReferenceContext::Absolute, 1.0, 0.0}))),
              Status::ValidationError);
    Engine zero_scale;
    EXPECT_EQ(zero_scale.init(make_trajectory_scenario(std::make_shared<FollowTrajectoryAction>(
                  "ego", make_corner_trajectory(0.0, 10.0, 20.0), FollowingMode::Position,
                  Timing{ReferenceContext::Absolute, 0.0, 0.0}))),
              Status::ValidationError);
}

TEST(FollowTrajectoryValidationTest, OffsetMustLieWithinTheArcLength) {
    // per rule
    // asam.net:xosc:1.1.0:routing.offset_should_be_less_than_trajectory_length
    Engine engine;
    EXPECT_EQ(engine.init(make_trajectory_scenario(std::make_shared<FollowTrajectoryAction>(
                  "ego", make_corner_trajectory(), FollowingMode::Position, std::nullopt, 250.0))),
              Status::ValidationError);
    ASSERT_FALSE(engine.diagnostics().empty());
    EXPECT_EQ(engine.diagnostics().front().rule_id,
              "asam.net:xosc:1.1.0:routing.offset_should_be_less_than_trajectory_length");

    Engine negative;
    EXPECT_EQ(negative.init(make_trajectory_scenario(std::make_shared<FollowTrajectoryAction>(
                  "ego", make_corner_trajectory(), FollowingMode::Position, std::nullopt, -1.0))),
              Status::ValidationError);
}

TEST(FollowTrajectoryValidationTest, ClosedAndFollowModeAreAcceptedWithAWarning) {
    Trajectory closed = make_corner_trajectory();
    closed.closed = true;
    Engine engine;
    ASSERT_EQ(engine.init(make_trajectory_scenario(
                  std::make_shared<FollowTrajectoryAction>("ego", closed, FollowingMode::Follow))),
              Status::Ok);
    int warnings = 0;
    for (const scena::Diagnostic& diagnostic : engine.diagnostics()) {
        if (diagnostic.severity == scena::Severity::Warning &&
            diagnostic.code == Status::UnsupportedFeature) {
            ++warnings;
        }
    }
    // One for the closed loop (p2-s5), one for follow-as-position (ADR-0011).
    EXPECT_EQ(warnings, 2);
    // Still runs: the open path is followed.
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_NEAR(engine.state("ego")->x, 10.0, kTol);
}

} // namespace
