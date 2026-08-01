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

"""The gateway from Python: callbacks, state injection and their order (p6-s3)."""

import pytest

import scena as scn


class Recorder(scn.SimulatorGateway):
    """Records the whole callback stream, so the order itself is assertable."""

    def __init__(self, injected=None):
        super().__init__()
        self.log: list[str] = []
        self.injected = injected

    def on_step_begin(self, dt):
        self.log.append(f"begin:{dt}")

    def on_step_end(self, dt):
        self.log.append(f"end:{dt}")

    def publish_state(self, entity_id, state):
        self.log.append(f"publish:{entity_id}")

    def poll_state(self, entity_id):
        self.log.append(f"poll:{entity_id}")
        return self.injected

    def on_element_transition(self, path, state, transition):
        self.log.append(f"transition:{path or '<storyboard>'}:{transition.name}")


def _scenario(mode=scn.ControlMode.EngineControlled):
    scenario = scn.Scenario("callbacks")
    scenario.add_entity(scn.Entity("ego", "ego", mode))
    dynamics = scn.TransitionDynamics(
        shape=scn.DynamicsShape.Linear, dimension=scn.DynamicsDimension.Time, value=2.0
    )
    event = scn.Event("event", start_trigger=scn.make_trigger(scn.SimulationTimeCondition(1.0)))
    event.add_action(scn.SpeedAction("ego", 10.0, dynamics))
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


def test_every_step_is_bracketed() -> None:
    gateway = Recorder()
    engine = scn.Engine()
    engine.set_gateway(gateway)
    assert engine.init(_scenario()) == scn.Status.Ok
    gateway.log.clear()

    assert engine.step(1.0) == scn.Status.Ok
    assert gateway.log[0] == "begin:1.0"
    assert gateway.log[-1] == "end:1.0"


def test_a_rejected_step_opens_no_bracket() -> None:
    gateway = Recorder()
    engine = scn.Engine()
    engine.set_gateway(gateway)
    assert engine.init(_scenario()) == scn.Status.Ok
    gateway.log.clear()
    assert engine.step(-1.0) == scn.Status.InvalidArgument
    assert gateway.log == []


def test_publish_and_poll_follow_control_ownership() -> None:
    engine_side = Recorder()
    engine = scn.Engine()
    engine.set_gateway(engine_side)
    assert engine.init(_scenario(scn.ControlMode.EngineControlled)) == scn.Status.Ok
    engine_side.log.clear()
    assert engine.step(0.5) == scn.Status.Ok
    assert "publish:ego" in engine_side.log
    assert "poll:ego" not in engine_side.log

    injected = scn.EntityState()
    injected.x = 3.5
    injected.speed = 2.25
    host_side = Recorder(injected=injected)
    host_engine = scn.Engine()
    host_engine.set_gateway(host_side)
    assert host_engine.init(_scenario(scn.ControlMode.HostControlled)) == scn.Status.Ok
    host_side.log.clear()
    assert host_engine.step(0.5) == scn.Status.Ok
    assert "poll:ego" in host_side.log
    assert "publish:ego" not in host_side.log
    assert host_engine.state("ego").x == pytest.approx(3.5)
    assert host_engine.state("ego").speed == pytest.approx(2.25)


def test_returning_none_from_poll_leaves_the_entity_alone() -> None:
    gateway = Recorder(injected=None)
    engine = scn.Engine()
    engine.set_gateway(gateway)
    assert engine.init(_scenario(scn.ControlMode.HostControlled)) == scn.Status.Ok
    assert engine.step(0.5) == scn.Status.Ok
    assert engine.state("ego").x == pytest.approx(0.0)


def test_transitions_are_reported_in_document_order_once_each() -> None:
    gateway = Recorder()
    engine = scn.Engine()
    engine.set_gateway(gateway)
    # init's evaluation at t = 0 starts everything without a start trigger.
    assert engine.init(_scenario()) == scn.Status.Ok
    assert [entry for entry in gateway.log if entry.startswith("transition:")] == [
        "transition:<storyboard>:Start",
        "transition:story:Start",
        "transition:story/act:Start",
        "transition:story/act/group:Start",
        "transition:story/act/group/maneuver:Start",
    ]

    gateway.log.clear()
    assert engine.step(1.0) == scn.Status.Ok
    assert [entry for entry in gateway.log if entry.startswith("transition:")] == [
        "transition:story/act/group/maneuver/event:Start",
    ]

    # A one-evaluation pulse: nothing is re-reported on a quiet step.
    gateway.log.clear()
    assert engine.step(0.5) == scn.Status.Ok
    assert [entry for entry in gateway.log if entry.startswith("transition:")] == []


def test_every_hook_is_optional() -> None:
    class Minimal(scn.SimulatorGateway):
        """Implements nothing — the engine must not require any of them."""

    engine = scn.Engine()
    engine.set_gateway(Minimal())
    assert engine.init(_scenario()) == scn.Status.Ok
    for _ in range(5):
        assert engine.step(0.5) == scn.Status.Ok
    assert engine.state("ego").speed > 0.0


def test_a_gateway_can_be_attached_and_detached() -> None:
    gateway = Recorder()
    engine = scn.Engine()
    assert engine.gateway is None
    assert engine.init(_scenario()) == scn.Status.Ok
    assert engine.step(0.5) == scn.Status.Ok
    assert gateway.log == []

    engine.set_gateway(gateway)
    assert engine.gateway is not None
    assert engine.step(0.5) == scn.Status.Ok
    assert gateway.log != []

    gateway.log.clear()
    engine.set_gateway(None)
    assert engine.gateway is None
    assert engine.step(0.5) == scn.Status.Ok
    assert gateway.log == []


def test_the_callback_stream_is_identical_across_repeats() -> None:
    def run():
        gateway = Recorder()
        engine = scn.Engine()
        engine.set_gateway(gateway)
        assert engine.init(_scenario()) == scn.Status.Ok
        for _ in range(30):
            assert engine.step(0.1) == scn.Status.Ok
        return list(gateway.log)

    assert run() == run()
