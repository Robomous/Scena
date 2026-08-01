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

"""followingMode=follow, jerk limits and an entity-relative speed profile.

Two engine-controlled vehicles (ASAM OpenSCENARIO XML 1.4.0 §SpeedAction /
§SpeedProfileAction, ADR-0024):

- `lead` settles at 12 m/s and holds it.
- `ego` first runs a follow-mode SpeedAction. The author asked for a Linear
  ramp over 1 s; follow mode realises it with a shape whose acceleration is
  zero at both ends and stretches it until the peak acceleration and the peak
  jerk both fit inside the vehicle's Performance envelope. With
  maxAcceleration = 2 m/s^2 the acceleration bound wins: T = 1.5 * 12 / 2 = 9 s.
- `ego` then runs a speed profile whose entries are deltas on `lead`'s speed,
  with DynamicConstraints that override the Performance envelope: an entry of
  -4 m/s targets 8 m/s.

Position mode, by contrast, holds the authored shape exactly and clamps only
acceleration — see longitudinal.py.
"""

import scena as scn

_ACCEL_LIMITED_SPAN = 9.0  # 1.5 * 12 / 2, the Cubic peak-gradient bound


def build_scenario() -> "scn.Scenario":
    scenario = scn.Scenario("follow-mode")

    ego_vehicle = scn.Vehicle()
    ego_vehicle.performance = scn.Performance(
        max_speed=40.0,
        max_acceleration=2.0,
        max_deceleration=2.0,
        max_acceleration_rate=4.0,
        max_deceleration_rate=4.0,
    )
    scenario.add_entity(
        scn.Entity("ego", "ego vehicle", scn.ControlMode.EngineControlled, object=ego_vehicle)
    )
    scenario.add_entity(scn.Entity("lead", "lead vehicle", scn.ControlMode.EngineControlled))
    # The lead holds a constant speed for the whole run.
    scenario.add_init_action(scn.SpeedAction("lead", target_speed=12.0))

    def timed_event(name: str, at_time: float, action) -> "scn.Event":
        event = scn.Event(
            name, start_trigger=scn.make_trigger(scn.SimulationTimeCondition(at_time=at_time))
        )
        event.add_action(action)
        return event

    follow = scn.TransitionDynamics(
        shape=scn.DynamicsShape.Linear,
        dimension=scn.DynamicsDimension.Time,
        value=1.0,
        following_mode=scn.FollowingMode.Follow,
    )

    maneuver = scn.Maneuver("maneuver")
    maneuver.add_event(timed_event("ramp", 0.0, scn.SpeedAction("ego", 12.0, follow)))
    maneuver.add_event(
        timed_event(
            "relative_profile",
            12.0,
            scn.SpeedProfileAction(
                "ego",
                [scn.SpeedProfileEntry(-4.0, 4.0)],
                scn.FollowingMode.Position,
                "lead",
                scn.DynamicConstraints(max_acceleration=6.0, max_deceleration=6.0),
            ),
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
    assert engine.init(build_scenario()) == scn.Status.Ok

    # A quarter of the way through the stretched ramp the smooth shape is well
    # behind where a linear one would be: 12 * (3/16 - 2/64) = 1.875 m/s.
    for _ in range(45):  # t = 2.25 s = 9 / 4
        assert engine.step(0.05) == scn.Status.Ok
    quarter = engine.state("ego").speed
    assert abs(quarter - 1.875) < 1e-9, quarter
    print(f"follow-mode ramp is at {quarter:.3f} m/s a quarter of the way in")

    # The ramp finishes at t = 9 s, stretched from the authored 1 s.
    for _ in range(135):  # t = 9 s
        assert engine.step(0.05) == scn.Status.Ok
    assert engine.state("ego").speed == 12.0, engine.state("ego").speed
    print(f"follow-mode ramp reached {engine.state('ego').speed:.1f} m/s in "
          f"{_ACCEL_LIMITED_SPAN:.0f} s (authored 1 s)")

    # Relative profile at t = 12 s: 12 -> lead - 4 = 8 m/s over 4 s.
    for _ in range(160):  # t = 17 s
        assert engine.step(0.05) == scn.Status.Ok
    assert engine.state("ego").speed == 8.0, engine.state("ego").speed
    print(f"entity-relative profile settled at {engine.state('ego').speed:.1f} m/s "
          f"(lead {engine.state('lead').speed:.1f} m/s minus 4)")

    # Every target stayed inside maxSpeed, so no advisory diagnostics.
    assert engine.diagnostics() == [], engine.diagnostics()
    engine.close()
    print("follow mode: jerk-aware ramp + entity-relative constrained profile: OK")


if __name__ == "__main__":
    main()
