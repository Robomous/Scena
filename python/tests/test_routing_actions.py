# SPDX-FileCopyrightText: 2026 Robomous
# SPDX-License-Identifier: Apache-2.0
"""Routing, controller and visibility actions through the Python bindings
(p5-s5): AssignRouteAction, AcquirePositionAction, FollowTrajectoryAction,
AssignControllerAction, ActivateControllerAction and VisibilityAction."""

import pytest

import scena as scn


def _scenario(events, *, speed=10.0):
    """One engine-controlled 'ego' at the origin cruising at `speed`, plus one
    maneuver holding `events`."""
    scenario = scn.Scenario("routing")
    scenario.add_entity(scn.Entity("ego", "ego", scn.ControlMode.EngineControlled))
    scenario.add_init_action(scn.TeleportAction("ego", scn.WorldPosition(0.0, 0.0, 0.0)))
    scenario.add_init_action(scn.SpeedAction("ego", speed))

    maneuver = scn.Maneuver("maneuver")
    for event in events:
        maneuver.add_event(event)
    group = scn.ManeuverGroup("group")
    group.add_maneuver(maneuver)
    act = scn.Act("act")
    act.add_group(group)
    story = scn.Story("story")
    story.add_act(act)
    scenario.add_story(story)
    return scenario


def _timed_event(name, at_time, action):
    event = scn.Event(name, start_trigger=scn.make_trigger(scn.SimulationTimeCondition(at_time)))
    event.add_action(action)
    return event


def _corner_trajectory(times=(None, None, None)):
    """(0,0) -> (100,0) -> (100,100): 200 m of arc, headings 0 and pi/2."""
    return scn.Trajectory(
        name="corner",
        vertices=[
            scn.TrajectoryVertex(scn.WorldPosition(0.0, 0.0, 0.0), times[0]),
            scn.TrajectoryVertex(scn.WorldPosition(100.0, 0.0, 0.0), times[1]),
            scn.TrajectoryVertex(scn.WorldPosition(100.0, 100.0, 0.0), times[2]),
        ],
    )


def test_route_round_trip() -> None:
    route = scn.Route(name="r1", closed=True)
    route.add_waypoint(scn.Waypoint(scn.WorldPosition(1.0, 2.0, 3.0), scn.RouteStrategy.Fastest))
    route.add_waypoint(scn.Waypoint(scn.WorldPosition(4.0, 5.0, 6.0)))
    action = scn.AssignRouteAction("ego", route)
    assert action.kind == "AssignRouteAction"
    assert action.route.name == "r1"
    assert action.route.closed is True
    assert len(action.route.waypoints) == 2
    assert action.route.waypoints[0].strategy == scn.RouteStrategy.Fastest
    # The default strategy is shortest.
    assert action.route.waypoints[1].strategy == scn.RouteStrategy.Shortest


def test_assign_route_completes_immediately_and_is_queryable() -> None:
    route = scn.Route(
        name="r1",
        waypoints=[
            scn.Waypoint(scn.WorldPosition(0.0, 0.0, 0.0)),
            scn.Waypoint(scn.WorldPosition(80.0, 0.0, 0.0), scn.RouteStrategy.LeastIntersections),
        ],
    )
    engine = scn.Engine()
    assert engine.init(_scenario([_timed_event("assign", 1.0, scn.AssignRouteAction("ego", route))])) == scn.Status.Ok
    assert engine.route_of("ego") is None

    assert engine.step(1.0) == scn.Status.Ok
    # Annex A Table 10: the action does not consume simulation time.
    assert (
        engine.storyboard_element_state("story/act/group/maneuver/assign")
        == scn.ElementState.Complete
    )
    assigned = engine.route_of("ego")
    assert assigned is not None
    assert assigned.name == "r1"
    assert assigned.waypoints[1].strategy == scn.RouteStrategy.LeastIntersections


def test_acquire_position_builds_the_implicit_route() -> None:
    engine = scn.Engine()
    events = [
        _timed_event(
            "acquire", 2.0, scn.AcquirePositionAction("ego", scn.WorldPosition(400.0, 0.0, 0.0))
        )
    ]
    assert engine.init(_scenario(events)) == scn.Status.Ok
    for _ in range(2):
        assert engine.step(1.0) == scn.Status.Ok
    route = engine.route_of("ego")
    assert route is not None
    assert len(route.waypoints) == 2
    # §7.4.1.4: the current position becomes the first waypoint.
    assert route.waypoints[0].position.x == pytest.approx(10.0)
    assert route.waypoints[1].position.x == pytest.approx(400.0)


def test_time_free_trajectory_teleports_to_the_start_and_follows_it() -> None:
    engine = scn.Engine()
    events = [
        _timed_event("follow", 1.0, scn.FollowTrajectoryAction("ego", _corner_trajectory()))
    ]
    assert engine.init(_scenario(events)) == scn.Status.Ok
    assert engine.step(1.0) == scn.Status.Ok  # §6.9.1: teleport to the start
    assert engine.state("ego").x == pytest.approx(0.0)
    for _ in range(12):
        assert engine.step(1.0) == scn.Status.Ok
    # 120 m of arc: 20 m up the second segment, heading pi/2.
    assert engine.state("ego").x == pytest.approx(100.0)
    assert engine.state("ego").y == pytest.approx(20.0)
    assert engine.state("ego").heading == pytest.approx(1.5707963267948966, abs=1e-12)


def test_timed_trajectory_drives_position_and_speed() -> None:
    engine = scn.Engine()
    action = scn.FollowTrajectoryAction(
        "ego",
        _corner_trajectory((0.0, 10.0, 20.0)),
        time_reference=scn.Timing(scn.ReferenceContext.Absolute, 1.0, 0.0),
    )
    assert engine.init(_scenario([_timed_event("follow", 0.0, action)], speed=0.0)) == scn.Status.Ok
    for _ in range(5):
        assert engine.step(1.0) == scn.Status.Ok
    assert engine.state("ego").x == pytest.approx(50.0)
    assert engine.state("ego").speed == pytest.approx(10.0)  # 100 m over 10 s


def test_trajectory_timing_requires_a_time_on_every_vertex() -> None:
    engine = scn.Engine()
    action = scn.FollowTrajectoryAction(
        "ego",
        _corner_trajectory((0.0, None, 20.0)),
        time_reference=scn.Timing(),
    )
    assert engine.init(_scenario([_timed_event("follow", 0.0, action)])) == scn.Status.ValidationError
    rules = [d.rule_id for d in engine.diagnostics()]
    assert "asam.net:xosc:1.0.0:routing.trajectory_timing_exists_if_requested" in rules


def test_controller_assignment_and_activation() -> None:
    controller = scn.Controller(
        name="driver",
        type=scn.ControllerType.Movement,
        properties=[scn.Property("model", "idm")],
    )
    engine = scn.Engine()
    events = [
        _timed_event("assign", 1.0, scn.AssignControllerAction("ego", controller)),
        _timed_event(
            "off", 2.0, scn.ActivateControllerAction("ego", longitudinal=False)
        ),
        _timed_event("speed", 3.0, scn.SpeedAction("ego", 25.0)),
    ]
    assert engine.init(_scenario(events)) == scn.Status.Ok
    assert engine.assigned_controller_of("ego") is None
    assert engine.controller_activation_of("ego").longitudinal is True

    assert engine.step(1.0) == scn.Status.Ok
    assigned = engine.assigned_controller_of("ego")
    assert assigned is not None
    assert assigned.name == "driver"
    assert assigned.properties[0].name == "model"

    assert engine.step(1.0) == scn.Status.Ok
    activation = engine.controller_activation_of("ego")
    assert activation.longitudinal is False
    assert activation.lateral is True  # unset flags leave a domain alone

    assert engine.step(1.0) == scn.Status.Ok
    # The SpeedAction was skipped: the domain is deactivated (§7.5.2.2).
    assert engine.state("ego").speed == pytest.approx(10.0)


def test_activating_a_domain_outside_the_controller_type_is_rejected() -> None:
    # per rule asam.net:xosc:1.2.0:scenario_logic.controller_activation
    controller = scn.Controller(name="lights", type=scn.ControllerType.Lighting)
    engine = scn.Engine()
    events = [
        _timed_event(
            "assign", 0.0, scn.AssignControllerAction("ego", controller, activate_lateral=True)
        )
    ]
    assert engine.init(_scenario(events)) == scn.Status.ValidationError


def test_visibility_defaults_and_action() -> None:
    engine = scn.Engine()
    events = [
        _timed_event(
            "hide", 1.0, scn.VisibilityAction("ego", graphics=False, sensors=True, traffic=False)
        )
    ]
    assert engine.init(_scenario(events)) == scn.Status.Ok
    # §VisibilityAction: entities are visible everywhere by default.
    before = engine.visibility_of("ego")
    assert (before.graphics, before.sensors, before.traffic) == (True, True, True)

    assert engine.step(1.0) == scn.Status.Ok
    after = engine.visibility_of("ego")
    assert (after.graphics, after.sensors, after.traffic) == (False, True, False)
    assert engine.visibility_of("ghost") is None
