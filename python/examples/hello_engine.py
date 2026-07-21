#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Build a storyboard in code, step it at 100 Hz, and verify the speed action fires.

One engine-controlled entity starts at rest. The storyboard is the full ASAM
OpenSCENARIO hierarchy (Story > Act > ManeuverGroup > Maneuver > Event): an
event triggered by a SimulationTimeCondition at t = 2.0 s sets the entity's
speed to 10 m/s. The host loop steps 5 simulated seconds at 100 Hz, watches
the event's lifecycle state, and asserts the speed change happened.
"""

import scena as scn


def build_scenario() -> "scn.Scenario":
    scenario = scn.Scenario("hello-engine")
    scenario.add_entity(scn.Entity("ego", "ego vehicle", scn.ControlMode.EngineControlled))

    event = scn.Event("speed-up", start_trigger=scn.SimulationTimeCondition(at_time=2.0))
    event.add_action(scn.SpeedAction("ego", target_speed=10.0))

    maneuver = scn.Maneuver("maneuver")
    maneuver.add_event(event)
    group = scn.ManeuverGroup("group")
    group.add_actor("ego")
    group.add_maneuver(maneuver)
    act = scn.Act("act")  # no start trigger: starts with the storyboard
    act.add_group(group)
    story = scn.Story("story")
    story.add_act(act)
    scenario.add_story(story)
    return scenario


def main() -> None:
    print(f"Scena {scn.version()}")

    engine = scn.Engine()
    status = engine.init(build_scenario())
    assert status == scn.Status.Ok, status

    event_path = "story/act/group/maneuver/speed-up"
    assert engine.storyboard_element_state(event_path) == scn.ElementState.Standby

    dt = 0.01  # 100 Hz
    steps = 500  # 5 simulated seconds
    speed_before_trigger = None

    for i in range(steps):
        status = engine.step(dt)
        assert status == scn.Status.Ok, status
        if i == 150:  # t = 1.51 s, before the trigger
            speed_before_trigger = engine.state("ego").speed
        if (i + 1) % 100 == 0:
            state = engine.state("ego")
            print(f"t={engine.time:5.2f}s  x={state.x:7.3f}m  speed={state.speed:5.2f}m/s")

    final = engine.state("ego")
    assert speed_before_trigger == 0.0, f"moved before trigger: {speed_before_trigger}"
    assert final.speed == 10.0, f"speed action did not fire: {final.speed}"
    assert final.x > 0.0, "entity did not move after the speed action"
    assert engine.storyboard_element_state(event_path) == scn.ElementState.Complete
    assert engine.storyboard_element_state("") == scn.ElementState.Running  # never self-completes

    engine.close()
    print("speed action fired at t=2.0s: OK")


if __name__ == "__main__":
    main()
