#!/usr/bin/env python3

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

"""Clothoid and NURBS trajectory following (p2-s5).

The non-polyline shapes of ASAM OpenSCENARIO XML 1.4.0 §Trajectory:

- a Clothoid (§Clothoid) is an Euler spiral whose curvature changes linearly
  with arc length; with curvature_prime == 0 it is a circular arc, as used
  here (a quarter circle of radius 20 m);
- a Nurbs (§Nurbs) is a rational B-spline; the exact quarter circle below is a
  quadratic NURBS with the middle weight cos(45 deg) = 1/sqrt(2).

The engine evaluates both by arc length with analytic fidelity (risk R3) and
follows them in position mode: the entity's own speed sets the pace, and every
sampled point lies exactly on the circle.
"""

import math

import scena as scn


def _timed_event(name: str, at_time: float, action) -> "scn.Event":
    event = scn.Event(name, start_trigger=scn.make_trigger(scn.SimulationTimeCondition(at_time)))
    event.add_action(action)
    return event


def build_scenario() -> "scn.Scenario":
    radius = 20.0
    scenario = scn.Scenario("trajectory-shapes")
    scenario.add_entity(scn.Entity("ego", "ego", scn.ControlMode.EngineControlled))
    scenario.add_init_action(scn.TeleportAction("ego", scn.WorldPosition(0.0, 0.0, 0.0)))
    scenario.add_init_action(scn.SpeedAction("ego", 10.0))

    # A quarter circle as a constant-curvature clothoid arc from the origin.
    clothoid = scn.Clothoid(
        start=scn.WorldPosition(0.0, 0.0, 0.0),
        curvature=1.0 / radius,
        curvature_prime=0.0,
        length=radius * math.pi / 2.0,
    )

    # The same quarter circle as an exact rational quadratic NURBS (printed for
    # reference; the ego follows the clothoid here).
    nurbs = scn.Nurbs(
        order=3,
        control_points=[
            scn.ControlPoint(scn.WorldPosition(radius, 0.0, 0.0), weight=1.0),
            scn.ControlPoint(scn.WorldPosition(radius, radius, 0.0), weight=1.0 / math.sqrt(2.0)),
            scn.ControlPoint(scn.WorldPosition(0.0, radius, 0.0), weight=1.0),
        ],
        knots=[0.0, 0.0, 0.0, 1.0, 1.0, 1.0],
    )
    print(f"NURBS quarter circle: order {nurbs.order}, {len(nurbs.control_points)} control points")

    maneuver = scn.Maneuver("maneuver")
    maneuver.add_event(
        _timed_event("follow", 1.0, scn.FollowTrajectoryAction("ego", scn.Trajectory("arc", False, clothoid)))
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

    print(f"{'t [s]':>6} {'x':>8} {'y':>8} {'radius':>8} {'heading':>9}")
    for _ in range(40):
        assert engine.step(0.25) == scn.Status.Ok
        state = engine.state("ego")
        # Distance from the centre of curvature (0, 20) stays at 20 m.
        radius = math.hypot(state.x - 0.0, state.y - 20.0)
        print(f"{engine.time:6.2f} {state.x:8.3f} {state.y:8.3f} {radius:8.4f} {state.heading:9.4f}")


if __name__ == "__main__":
    main()
