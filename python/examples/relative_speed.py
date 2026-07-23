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

"""Relative speed targets and a world-frame teleport (p5-s4).

Two engine-controlled vehicles demonstrate the private actions ASAM
OpenSCENARIO XML 1.4.0 §RelativeTargetSpeed / §TeleportAction:

- a lead vehicle cruises at a constant 12 m/s;
- the ego vehicle is teleported to its start position, then at t=1 s ramps to a
  one-shot relative target (lead + 3 = 15 m/s over 2 s);
- at t=5 s the ego switches to a *continuous* relative target that holds a
  factor of the lead's speed (0.5 x 12 = 6 m/s) and never ends on its own — the
  later action supersedes the earlier ramp.
"""

import scena as scn


def build_scenario() -> "scn.Scenario":
    scenario = scn.Scenario("relative-speed")
    scenario.add_entity(scn.Entity("lead", "lead", scn.ControlMode.EngineControlled))
    scenario.add_entity(scn.Entity("ego", "ego", scn.ControlMode.EngineControlled))

    # The lead cruises; the ego starts 40 m back after a teleport.
    scenario.add_init_action(scn.SpeedAction("lead", 12.0))
    scenario.add_init_action(scn.TeleportAction("ego", scn.WorldPosition(-40.0, 0.0, 0.0)))

    def timed_event(name: str, at_time: float, action) -> "scn.Event":
        event = scn.Event(
            name, start_trigger=scn.make_trigger(scn.SimulationTimeCondition(at_time=at_time))
        )
        event.add_action(action)
        return event

    ramp = scn.TransitionDynamics(
        shape=scn.DynamicsShape.Linear, dimension=scn.DynamicsDimension.Time, value=2.0
    )
    catch_up = scn.RelativeTargetSpeed(
        "lead", 3.0, value_type=scn.SpeedTargetValueType.Delta, continuous=False
    )
    hold_half = scn.RelativeTargetSpeed(
        "lead", 0.5, value_type=scn.SpeedTargetValueType.Factor, continuous=True
    )

    maneuver = scn.Maneuver("maneuver")
    maneuver.add_event(timed_event("catch-up", 1.0, scn.SpeedAction("ego", catch_up, ramp)))
    maneuver.add_event(
        timed_event("hold-half", 5.0, scn.SpeedAction("ego", hold_half, scn.TransitionDynamics()))
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

    # The init teleport placed the ego 40 m back.
    assert engine.state("ego").x == -40.0, engine.state("ego").x
    print(f"ego teleported to x = {engine.state('ego').x:.1f} m")

    # Cruise to t=1, then the one-shot relative ramp reaches lead + 3 = 15 m/s.
    for _ in range(80):  # 4 s
        assert engine.step(0.05) == scn.Status.Ok
    assert engine.state("ego").speed == 15.0, engine.state("ego").speed
    print(f"catch-up ramp reached {engine.state('ego').speed:.1f} m/s (lead + 3)")

    # At t=5 the continuous factor target supersedes it and holds 0.5 x 12 = 6.
    for _ in range(40):  # to t = 6 s
        assert engine.step(0.05) == scn.Status.Ok
    assert engine.state("ego").speed == 6.0, engine.state("ego").speed
    print(f"continuous target holds {engine.state('ego').speed:.1f} m/s (0.5 x lead)")

    assert engine.diagnostics() == [], engine.diagnostics()
    engine.close()
    print("relative speed targets + teleport: OK")


if __name__ == "__main__":
    main()
