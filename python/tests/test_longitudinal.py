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

"""Longitudinal dynamics (SpeedAction transition dynamics + SpeedProfileAction)
through the Python bindings (p2-s2)."""

import pytest

import scena as scn


def _scenario_with_action(action, *, initial_speed=0.0, perf=None):
    scenario = scn.Scenario("longitudinal")
    obj = None
    if perf is not None:
        vehicle = scn.Vehicle()
        vehicle.performance = perf
        obj = vehicle
    scenario.add_entity(scn.Entity("ego", "ego", scn.ControlMode.EngineControlled, object=obj))
    if initial_speed:
        scenario.add_init_action(scn.SpeedAction("ego", target_speed=initial_speed))

    event = scn.Event("event", start_trigger=scn.make_trigger(scn.SimulationTimeCondition(0.0)))
    event.add_action(action)
    maneuver = scn.Maneuver("maneuver")
    maneuver.add_event(event)
    group = scn.ManeuverGroup("group")
    group.add_maneuver(maneuver)
    act = scn.Act("act")
    act.add_group(group)
    story = scn.Story("story")
    story.add_act(act)
    scenario.add_story(story)
    return scenario


def test_transition_dynamics_round_trip() -> None:
    td = scn.TransitionDynamics(
        shape=scn.DynamicsShape.Cubic, dimension=scn.DynamicsDimension.Rate, value=2.5
    )
    action = scn.SpeedAction("ego", target_speed=8.0, dynamics=td)
    assert action.dynamics.shape == scn.DynamicsShape.Cubic
    assert action.dynamics.dimension == scn.DynamicsDimension.Rate
    assert action.dynamics.value == pytest.approx(2.5)
    assert action.dynamics.following_mode == scn.FollowingMode.Position
    # The 2-arg form is a Step transition.
    assert scn.SpeedAction("ego", 3.0).dynamics.shape == scn.DynamicsShape.Step


def test_linear_speed_ramp_drives_across_steps() -> None:
    td = scn.TransitionDynamics(
        shape=scn.DynamicsShape.Linear, dimension=scn.DynamicsDimension.Time, value=4.0
    )
    engine = scn.Engine()
    assert engine.init(_scenario_with_action(scn.SpeedAction("ego", 8.0, td))) == scn.Status.Ok

    assert engine.step(2.0) == scn.Status.Ok
    assert engine.state("ego").speed == pytest.approx(4.0)  # halfway
    assert engine.step(2.0) == scn.Status.Ok
    assert engine.state("ego").speed == pytest.approx(8.0)  # target reached


def test_performance_acceleration_clamp_extends_ramp() -> None:
    perf = scn.Performance(max_speed=60.0, max_acceleration=2.0, max_deceleration=2.0)
    td = scn.TransitionDynamics(
        shape=scn.DynamicsShape.Linear, dimension=scn.DynamicsDimension.Time, value=1.0
    )
    engine = scn.Engine()
    scenario = _scenario_with_action(scn.SpeedAction("ego", 10.0, td), perf=perf)
    assert engine.init(scenario) == scn.Status.Ok

    assert engine.step(1.0) == scn.Status.Ok
    assert engine.state("ego").speed == pytest.approx(2.0)  # clamped slope, not 10
    for _ in range(4):
        assert engine.step(1.0) == scn.Status.Ok
    assert engine.state("ego").speed == pytest.approx(10.0)  # reached at 5 s


def test_speed_profile_follows_targets() -> None:
    profile = scn.SpeedProfileAction(
        "ego",
        [scn.SpeedProfileEntry(10.0, 2.0), scn.SpeedProfileEntry(4.0, 2.0)],
    )
    engine = scn.Engine()
    assert engine.init(_scenario_with_action(profile)) == scn.Status.Ok

    assert engine.step(2.0) == scn.Status.Ok
    assert engine.state("ego").speed == pytest.approx(10.0)
    assert engine.step(2.0) == scn.Status.Ok
    assert engine.state("ego").speed == pytest.approx(4.0)


def test_empty_profile_is_rejected() -> None:
    engine = scn.Engine()
    assert engine.init(_scenario_with_action(scn.SpeedProfileAction("ego", []))) == (
        scn.Status.ValidationError
    )
