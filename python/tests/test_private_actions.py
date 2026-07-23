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

"""Private actions through the Python bindings (p5-s4): SpeedAction relative
targets (§RelativeTargetSpeed) and TeleportAction (§TeleportAction)."""

import pytest

import scena as scn


def _two_entity_scenario(events, *, lead_speed=8.0):
    """A 'lead' and an 'ego' engine-controlled vehicle, the lead given an initial
    speed, plus one maneuver holding `events`."""
    scenario = scn.Scenario("relative")
    scenario.add_entity(scn.Entity("lead", "lead", scn.ControlMode.EngineControlled))
    scenario.add_entity(scn.Entity("ego", "ego", scn.ControlMode.EngineControlled))
    scenario.add_init_action(scn.SpeedAction("lead", target_speed=lead_speed))

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


def test_relative_target_round_trip() -> None:
    target = scn.RelativeTargetSpeed(
        "lead", 5.0, value_type=scn.SpeedTargetValueType.Delta, continuous=True
    )
    action = scn.SpeedAction("ego", target, scn.TransitionDynamics())
    assert action.is_relative
    assert action.kind == "SpeedAction"
    assert action.relative_target.entity_ref == "lead"
    assert action.relative_target.value == pytest.approx(5.0)
    assert action.relative_target.value_type == scn.SpeedTargetValueType.Delta
    assert action.relative_target.continuous is True
    # An absolute action is not relative and carries no relative target.
    assert scn.SpeedAction("ego", 10.0).is_relative is False
    assert scn.SpeedAction("ego", 10.0).relative_target is None


def test_relative_delta_resolves_against_reference() -> None:
    target = scn.RelativeTargetSpeed("lead", 4.0, value_type=scn.SpeedTargetValueType.Delta)
    engine = scn.Engine()
    action = scn.SpeedAction("ego", target, scn.TransitionDynamics())  # Step
    assert engine.init(_two_entity_scenario([_timed_event("go", 0.0, action)])) == scn.Status.Ok
    # lead cruises at 8, ego jumps to 8 + 4 = 12 m/s.
    assert engine.state("ego").speed == 12.0


def test_continuous_relative_holds_factor_of_reference() -> None:
    target = scn.RelativeTargetSpeed(
        "lead", 1.5, value_type=scn.SpeedTargetValueType.Factor, continuous=True
    )
    engine = scn.Engine()
    action = scn.SpeedAction("ego", target, scn.TransitionDynamics())
    assert engine.init(_two_entity_scenario([_timed_event("hold", 0.0, action)])) == scn.Status.Ok
    for _ in range(3):
        assert engine.step(1.0) == scn.Status.Ok
        assert engine.state("ego").speed == 12.0  # 1.5 x 8, held every step


def test_continuous_with_timed_transition_is_rejected() -> None:
    target = scn.RelativeTargetSpeed("lead", 3.0, continuous=True)
    td = scn.TransitionDynamics(
        shape=scn.DynamicsShape.Linear, dimension=scn.DynamicsDimension.Time, value=2.0
    )
    engine = scn.Engine()
    action = scn.SpeedAction("ego", target, td)
    assert (
        engine.init(_two_entity_scenario([_timed_event("bad", 0.0, action)]))
        == scn.Status.ValidationError
    )


def test_teleport_round_trip() -> None:
    action = scn.TeleportAction("ego", scn.WorldPosition(7.0, -2.0, 1.0))
    assert action.kind == "TeleportAction"
    assert action.entity_id == "ego"
    assert action.position.x == pytest.approx(7.0)
    assert action.position.y == pytest.approx(-2.0)
    assert action.position.z == pytest.approx(1.0)


def test_init_teleport_sets_world_position() -> None:
    scenario = scn.Scenario("teleport")
    scenario.add_entity(scn.Entity("ego", "ego", scn.ControlMode.EngineControlled))
    scenario.add_init_action(scn.TeleportAction("ego", scn.WorldPosition(15.0, 4.0, 0.0)))
    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    state = engine.state("ego")
    assert state.x == 15.0
    assert state.y == 4.0
    assert state.z == 0.0
