# Copyright 2026 Robomous
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Clothoid and NURBS trajectory shapes through the Python bindings (p2-s5):
IR construction round-trips and following an engine-controlled entity along
the shape."""

import math

import pytest

import scena as scn


def _scenario(events, *, speed=10.0):
    scenario = scn.Scenario("trajectory")
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


def test_clothoid_shape_round_trip() -> None:
    clothoid = scn.Clothoid(
        start=scn.WorldPosition(1.0, 2.0, 0.0, 0.1),
        curvature=0.05,
        curvature_prime=0.001,
        length=40.0,
    )
    trajectory = scn.Trajectory("spiral", False, clothoid)
    shape = trajectory.shape
    assert isinstance(shape, scn.Clothoid)
    assert shape.curvature == pytest.approx(0.05)
    assert shape.curvature_prime == pytest.approx(0.001)
    assert shape.length == pytest.approx(40.0)
    assert shape.start.h == pytest.approx(0.1)
    assert shape.start_time is None


def test_nurbs_shape_round_trip() -> None:
    nurbs = scn.Nurbs(
        order=3,
        control_points=[
            scn.ControlPoint(scn.WorldPosition(20.0, 0.0, 0.0), weight=1.0),
            scn.ControlPoint(scn.WorldPosition(20.0, 20.0, 0.0), weight=1.0 / math.sqrt(2.0)),
            scn.ControlPoint(scn.WorldPosition(0.0, 20.0, 0.0), weight=1.0),
        ],
        knots=[0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
    )
    trajectory = scn.Trajectory("circle", False, nurbs)
    shape = trajectory.shape
    assert isinstance(shape, scn.Nurbs)
    assert shape.order == 3
    assert len(shape.control_points) == 3
    assert shape.control_points[1].weight == pytest.approx(1.0 / math.sqrt(2.0))
    # knots.size() == control_points.size() + order.
    assert len(shape.knots) == len(shape.control_points) + shape.order


def test_follow_clothoid_arc_stays_on_the_circle() -> None:
    radius = 20.0
    clothoid = scn.Clothoid(
        start=scn.WorldPosition(0.0, 0.0, 0.0),
        curvature=1.0 / radius,
        curvature_prime=0.0,
        length=radius * math.pi / 2.0,
    )
    action = scn.FollowTrajectoryAction("ego", scn.Trajectory("arc", False, clothoid))
    engine = scn.Engine()
    assert engine.init(_scenario([_timed_event("follow", 1.0, action)])) == scn.Status.Ok

    assert engine.step(1.0) == scn.Status.Ok  # teleport to the origin
    assert engine.step(1.0) == scn.Status.Ok  # 10 m along the arc
    state = engine.state("ego")
    # centre of curvature at (0, R): every point is a radius R from it.
    assert math.hypot(state.x - 0.0, state.y - radius) == pytest.approx(radius, abs=1e-9)


def test_follow_nurbs_circle_stays_on_the_circle() -> None:
    radius = 20.0
    weight = 1.0 / math.sqrt(2.0)
    nurbs = scn.Nurbs(
        order=3,
        control_points=[
            scn.ControlPoint(scn.WorldPosition(radius, 0.0, 0.0), weight=1.0),
            scn.ControlPoint(scn.WorldPosition(radius, radius, 0.0), weight=weight),
            scn.ControlPoint(scn.WorldPosition(0.0, radius, 0.0), weight=1.0),
        ],
        knots=[0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
    )
    action = scn.FollowTrajectoryAction("ego", scn.Trajectory("circle", False, nurbs))
    engine = scn.Engine()
    assert engine.init(_scenario([_timed_event("follow", 1.0, action)])) == scn.Status.Ok

    assert engine.step(1.0) == scn.Status.Ok  # teleport to (R, 0)
    assert engine.step(1.0) == scn.Status.Ok
    state = engine.state("ego")
    assert math.hypot(state.x, state.y) == pytest.approx(radius, abs=1e-9)
