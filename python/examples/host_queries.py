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

"""What a host asks the engine between steps.

The queries a simulator integration actually needs, all of which are equally
reachable from C (see docs/user-guide/c-api.md) — that parity is the point of
p6-s1:

- enumerate the entities without knowing their names up front
  (`entity_ids()`, in ascending id order);
- read each one's state and whether it is still in the scenario;
- watch a storyboard element's lifecycle by path;
- publish a traffic signal's state from the host side, the same write a
  TrafficSignalStateAction performs.
"""

import scena as scn


def build_scenario() -> "scn.Scenario":
    scenario = scn.Scenario("host-queries")
    # Declared out of alphabetical order on purpose: enumeration sorts.
    for name in ("ego", "alpha"):
        scenario.add_entity(scn.Entity(name, name, scn.ControlMode.EngineControlled))

    ramp = scn.TransitionDynamics(
        shape=scn.DynamicsShape.Linear, dimension=scn.DynamicsDimension.Time, value=2.0
    )
    event = scn.Event(
        "event", start_trigger=scn.make_trigger(scn.SimulationTimeCondition(at_time=1.0))
    )
    event.add_action(scn.SpeedAction("ego", 10.0, ramp))
    maneuver = scn.Maneuver("maneuver")
    maneuver.add_event(event)
    group = scn.ManeuverGroup("group")
    group.add_actor("ego")
    group.add_maneuver(maneuver)
    act = scn.Act("act")
    act.add_group(group)
    story = scn.Story("story")
    story.add_act(act)
    scenario.add_story(story)
    return scenario


EVENT = "story/act/group/maneuver/event"


def main() -> None:
    engine = scn.Engine()
    assert engine.entity_ids() == [], "nothing is declared before init"
    assert engine.init(build_scenario()) == scn.Status.Ok

    # Ascending id order, whatever order the scenario declared them in — the
    # same order the engine iterates, which is what makes a walk reproducible.
    ids = engine.entity_ids()
    assert ids == ["alpha", "ego"], ids
    print(f"entities: {', '.join(ids)}")

    # The event waits for its trigger, so it is in standbyState at t = 0.
    assert engine.storyboard_element_state(EVENT) == scn.ElementState.Standby

    # A host that owns the real signals publishes their state into the scenario.
    assert engine.set_traffic_signal_state("signal_17", "green") == scn.Status.Ok
    assert engine.traffic_signal_state("signal_17") == "green"
    print(f"signal_17 is {engine.traffic_signal_state('signal_17')}")

    for _ in range(100):  # 5 s at 20 Hz
        assert engine.step(0.05) == scn.Status.Ok

    # Walk every entity the way a host loop would, without naming any of them.
    for entity_id in engine.entity_ids():
        state = engine.state(entity_id)
        active = engine.entity_active(entity_id)
        print(
            f"{entity_id}: active={active} "
            f"x={state.x:.2f} y={state.y:.2f} speed={state.speed:.2f}"
        )

    assert engine.state("ego").speed == 10.0
    assert engine.state("alpha").speed == 0.0  # no action ever touched it
    assert engine.storyboard_element_state(EVENT) == scn.ElementState.Complete

    engine.close()
    print("host queries: enumeration, element state, signal publication: OK")


if __name__ == "__main__":
    main()
