#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Robomous
# SPDX-License-Identifier: Apache-2.0
"""Routing, trajectories, controllers and visibility (p5-s5).

The private actions of ASAM OpenSCENARIO XML 1.4.0 §RoutingAction and
§ControllerAction that install state rather than motion, plus the one that
moves an entity along a path:

- AssignRouteAction (§6.8.2) stores a route of waypoints and completes
  immediately (Annex A Table 10); it is readable through Engine.route_of;
- AcquirePositionAction builds the implicit two-waypoint route of §7.4.1.4
  from the entity's current position to the target;
- FollowTrajectoryAction moves the ego along a polyline. Without a time
  reference the entity teleports to the start (§6.9.1) and its own speed sets
  the pace; the heading follows each segment;
- AssignControllerAction hands a controller model to the host (Scena has none
  of its own) and VisibilityAction toggles detectability.
"""

import scena as scn


def _timed_event(name: str, at_time: float, action) -> "scn.Event":
    event = scn.Event(name, start_trigger=scn.make_trigger(scn.SimulationTimeCondition(at_time)))
    event.add_action(action)
    return event


def build_scenario() -> "scn.Scenario":
    scenario = scn.Scenario("routing")
    scenario.add_entity(scn.Entity("ego", "ego", scn.ControlMode.EngineControlled))
    scenario.add_init_action(scn.TeleportAction("ego", scn.WorldPosition(0.0, 0.0, 0.0)))
    scenario.add_init_action(scn.SpeedAction("ego", 10.0))

    # A route the host can read back; Scena stores it, road-based following
    # arrives with the road network.
    route = scn.Route(
        name="through-town",
        waypoints=[
            scn.Waypoint(scn.WorldPosition(0.0, 0.0, 0.0), scn.RouteStrategy.Shortest),
            scn.Waypoint(scn.WorldPosition(200.0, 50.0, 0.0), scn.RouteStrategy.Fastest),
            scn.Waypoint(scn.WorldPosition(400.0, 50.0, 0.0), scn.RouteStrategy.LeastIntersections),
        ],
    )

    # An L-shaped path: 100 m east, then 100 m north.
    trajectory = scn.Trajectory(
        name="corner",
        vertices=[
            scn.TrajectoryVertex(scn.WorldPosition(0.0, 0.0, 0.0)),
            scn.TrajectoryVertex(scn.WorldPosition(100.0, 0.0, 0.0)),
            scn.TrajectoryVertex(scn.WorldPosition(100.0, 100.0, 0.0)),
        ],
    )

    controller = scn.Controller(
        name="driver",
        type=scn.ControllerType.Movement,
        properties=[scn.Property("model", "idm"), scn.Property("aggressiveness", "0.7")],
    )

    maneuver = scn.Maneuver("maneuver")
    maneuver.add_event(_timed_event("assign-route", 1.0, scn.AssignRouteAction("ego", route)))
    maneuver.add_event(
        _timed_event("assign-controller", 1.0, scn.AssignControllerAction("ego", controller))
    )
    maneuver.add_event(
        _timed_event("follow", 2.0, scn.FollowTrajectoryAction("ego", trajectory))
    )
    maneuver.add_event(
        _timed_event(
            "go-dark",
            5.0,
            scn.VisibilityAction("ego", graphics=True, sensors=False, traffic=False),
        )
    )
    maneuver.add_event(
        _timed_event(
            "acquire",
            25.0,
            scn.AcquirePositionAction("ego", scn.WorldPosition(600.0, 100.0, 0.0)),
        )
    )

    group = scn.ManeuverGroup("group")
    group.add_actor("ego")
    group.add_maneuver(maneuver)
    act = scn.Act("act")
    act.add_group(group)
    story = scn.Story("story")
    story.add_act(act)
    scenario.add_story(story)
    return scenario


def main() -> None:
    engine = scn.Engine()
    status = engine.init(build_scenario())
    assert status == scn.Status.Ok, status

    print(f"{'t [s]':>6} {'x':>8} {'y':>8} {'heading':>9}")
    for step in range(300):
        assert engine.step(0.1) == scn.Status.Ok
        if step % 40 == 0:
            state = engine.state("ego")
            print(f"{engine.time:6.1f} {state.x:8.2f} {state.y:8.2f} {state.heading:9.4f}")

    route = engine.route_of("ego")
    assert route is not None
    # The AcquirePositionAction replaced the authored route with the implicit
    # two-waypoint one (§7.4.1.4).
    assert len(route.waypoints) == 2
    print(
        f"\nroute now runs from ({route.waypoints[0].position.x:.1f}, "
        f"{route.waypoints[0].position.y:.1f}) to ({route.waypoints[1].position.x:.1f}, "
        f"{route.waypoints[1].position.y:.1f})"
    )

    controller = engine.assigned_controller_of("ego")
    assert controller is not None
    properties = ", ".join(f"{p.name}={p.value}" for p in controller.properties)
    print(f"controller '{controller.name}' ({controller.type}) with {properties}")

    visibility = engine.visibility_of("ego")
    assert (visibility.graphics, visibility.sensors, visibility.traffic) == (True, False, False)
    print(f"visibility: {visibility}")

    # The trajectory ran to its end and the entity is past the corner.
    assert (
        engine.storyboard_element_state("story/act/group/maneuver/follow")
        == scn.ElementState.Complete
    )
    print("trajectory completed at its final vertex")


if __name__ == "__main__":
    main()
