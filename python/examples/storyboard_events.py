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

"""Watch the storyboard instead of polling it.

`on_element_transition` reports every storyboard element that transitioned in
the step's evaluation, so a host does not have to ask
`storyboard_element_state()` about each element it cares about.

The order is part of the deterministic run: document order, depth first,
**parents before their children**, reported after the evaluation completes and
before entity motion is integrated. A transition is a one-evaluation pulse — a
host that steps once sees each exactly once.
"""

import scena as scn


class EventLog(scn.SimulatorGateway):
    def __init__(self) -> None:
        super().__init__()
        self.entries: list[tuple[float, str, str, str]] = []
        self.now = 0.0

    def on_step_begin(self, dt: float) -> None:
        self.now += dt

    def on_element_transition(self, path, state, transition) -> None:
        self.entries.append((self.now, path or "<storyboard>", state.name, transition.name))


def build_scenario() -> "scn.Scenario":
    scenario = scn.Scenario("storyboard-events")
    scenario.add_entity(scn.Entity("ego", "ego", scn.ControlMode.EngineControlled))

    def timed(name: str, at: float, target: float, seconds: float) -> "scn.Event":
        dynamics = scn.TransitionDynamics(
            shape=scn.DynamicsShape.Linear,
            dimension=scn.DynamicsDimension.Time,
            value=seconds,
        )
        event = scn.Event(name, start_trigger=scn.make_trigger(scn.SimulationTimeCondition(at)))
        event.add_action(scn.SpeedAction("ego", target, dynamics))
        return event

    maneuver = scn.Maneuver("maneuver")
    maneuver.add_event(timed("accelerate", 1.0, 15.0, 2.0))
    maneuver.add_event(timed("brake", 5.0, 0.0, 2.0))
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
    log = EventLog()
    engine = scn.Engine()
    engine.set_gateway(log)
    assert engine.init(build_scenario()) == scn.Status.Ok

    for _ in range(100):  # 10 s at 10 Hz
        assert engine.step(0.1) == scn.Status.Ok

    for when, path, state, transition in log.entries:
        print(f"t={when:5.1f}  {transition:<5}  {path} -> {state}")

    paths = [entry[1] for entry in log.entries]
    assert "story/act/group/maneuver/accelerate" in paths
    assert "story/act/group/maneuver/brake" in paths
    # A pulse, not a level: each element is reported once per transition, not
    # once per step for as long as it is in that state.
    starts = [e for e in log.entries if e[3] == "Start" and e[1].endswith("accelerate")]
    assert len(starts) == 1, starts

    engine.close()
    print(f"storyboard events: {len(log.entries)} transition(s) observed: OK")


if __name__ == "__main__":
    main()
