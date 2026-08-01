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

"""Co-simulation: the host drives one entity, Scena drives the other.

The gateway seam. `ego` is engine-controlled — the scenario moves it. `npc` is
host-controlled — this program moves it, and Scena only observes. Both directions
flow through one `SimulatorGateway` subclass:

- `poll_state` is called once per step for every host-controlled entity, before
  the storyboard is evaluated. Return an `EntityState` to update the entity, or
  `None` to leave it alone.
- `publish_state` is called once per step for every engine-controlled entity,
  after its motion has been integrated.
- `on_step_begin` / `on_step_end` bracket the whole exchange, for a host that
  wants to open and commit a write in one place.

Every method is optional: implement only the ones you need.

**Do not call back into the engine from a callback.** They run synchronously
inside `step()`, and a reentrant call would observe a half-built step.
"""

import scena as scn


class HostSimulator(scn.SimulatorGateway):
    """A stand-in for a real simulator: `npc` drives straight at a fixed speed."""

    def __init__(self) -> None:
        super().__init__()
        self.npc_x = 100.0
        self.npc_speed = 8.0
        self.published: dict[str, float] = {}
        self.open_frames = 0

    def on_step_begin(self, dt: float) -> None:
        # Where a real host would open its write transaction.
        self.open_frames += 1
        self.dt = dt

    def poll_state(self, entity_id: str):
        if entity_id != "npc":
            return None  # nothing to say about this one
        self.npc_x += self.npc_speed * self.dt
        state = scn.EntityState()
        state.x = self.npc_x
        state.speed = self.npc_speed
        return state

    def publish_state(self, entity_id: str, state: "scn.EntityState") -> None:
        self.published[entity_id] = state.x

    def on_step_end(self, dt: float) -> None:
        # ... and commit it.
        self.open_frames -= 1


def build_scenario() -> "scn.Scenario":
    scenario = scn.Scenario("host-controlled")
    scenario.add_entity(scn.Entity("ego", "ego", scn.ControlMode.EngineControlled))
    scenario.add_entity(scn.Entity("npc", "npc", scn.ControlMode.HostControlled))

    ramp = scn.TransitionDynamics(
        shape=scn.DynamicsShape.Linear, dimension=scn.DynamicsDimension.Time, value=2.0
    )
    event = scn.Event("go", start_trigger=scn.make_trigger(scn.SimulationTimeCondition(0.0)))
    event.add_action(scn.SpeedAction("ego", 12.0, ramp))
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


def main() -> None:
    host = HostSimulator()
    engine = scn.Engine()
    engine.set_gateway(host)
    assert engine.init(build_scenario()) == scn.Status.Ok

    for _ in range(100):  # 10 s at 10 Hz
        assert engine.step(0.1) == scn.Status.Ok

    # The engine moved ego and never touched npc's state ...
    ego = engine.state("ego")
    npc = engine.state("npc")
    print(f"ego  (engine-controlled): x={ego.x:.2f} speed={ego.speed:.2f}")
    print(f"npc  (host-controlled):   x={npc.x:.2f} speed={npc.speed:.2f}")
    assert ego.speed == 12.0
    assert npc.speed == 8.0
    assert abs(npc.x - host.npc_x) < 1e-9, "the engine reports exactly what the host said"

    # ... and it published ego, not npc: per-entity control ownership.
    assert "ego" in host.published
    assert "npc" not in host.published
    assert host.open_frames == 0, "every opened frame was committed"

    engine.close()
    print("host-controlled co-simulation: OK")


if __name__ == "__main__":
    main()
