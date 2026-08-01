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

"""followingMode=follow, DynamicConstraints and entity-relative speed profiles
through the Python bindings (#62, ADR-0024)."""

import math

import pytest

import scena as scn


def _scenario(action, *, entities=(("ego", None),), init_actions=()):
    scenario = scn.Scenario("follow-mode")
    for name, perf in entities:
        obj = None
        if perf is not None:
            vehicle = scn.Vehicle()
            vehicle.performance = perf
            obj = vehicle
        scenario.add_entity(
            scn.Entity(name, name, scn.ControlMode.EngineControlled, object=obj)
        )
    for init_action in init_actions:
        scenario.add_init_action(init_action)

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


def test_follow_mode_uses_a_smooth_shape_and_the_acceleration_envelope() -> None:
    # 0 -> 10 m/s authored over 2 s, but maxAcceleration is 2 m/s^2 and the
    # realised shape is Cubic (peak gradient 1.5): T = 1.5 * 10 / 2 = 7.5 s.
    perf = scn.Performance(max_speed=60.0, max_acceleration=2.0, max_deceleration=2.0)
    td = scn.TransitionDynamics(
        shape=scn.DynamicsShape.Linear,
        dimension=scn.DynamicsDimension.Time,
        value=2.0,
        following_mode=scn.FollowingMode.Follow,
    )
    engine = scn.Engine()
    assert engine.init(_scenario(scn.SpeedAction("ego", 10.0, td), entities=(("ego", perf),))) == (
        scn.Status.Ok
    )
    assert engine.step(7.5 * 0.25) == scn.Status.Ok
    # Cubic at p = 0.25: 3(1/16) - 2(1/64) = 0.15625, not the Linear 0.25.
    assert engine.state("ego").speed == pytest.approx(1.5625)
    assert engine.step(7.5 * 0.75) == scn.Status.Ok
    assert engine.state("ego").speed == pytest.approx(10.0)


def test_follow_mode_honours_the_jerk_rate_limit() -> None:
    # Acceleration effectively unbounded, jerk limited to 3 m/s^3:
    # T = sqrt(6 * 10 / 3) = sqrt(20).
    perf = scn.Performance(
        max_speed=60.0,
        max_acceleration=100.0,
        max_deceleration=100.0,
        max_acceleration_rate=3.0,
        max_deceleration_rate=3.0,
    )
    td = scn.TransitionDynamics(
        shape=scn.DynamicsShape.Linear,
        dimension=scn.DynamicsDimension.Time,
        value=1.0,
        following_mode=scn.FollowingMode.Follow,
    )
    engine = scn.Engine()
    assert engine.init(_scenario(scn.SpeedAction("ego", 10.0, td), entities=(("ego", perf),))) == (
        scn.Status.Ok
    )
    span = math.sqrt(20.0)
    assert engine.step(span * 0.5) == scn.Status.Ok
    assert engine.state("ego").speed == pytest.approx(5.0)
    assert engine.step(span * 0.5) == scn.Status.Ok
    assert engine.state("ego").speed == pytest.approx(10.0)


def test_speed_profile_constraints_take_precedence_over_performance() -> None:
    # A looser DynamicConstraints wins over a tighter Performance envelope
    # (§SpeedProfileAction: "has precedence over any Performance settings").
    perf = scn.Performance(max_speed=60.0, max_acceleration=1.0, max_deceleration=1.0)
    profile = scn.SpeedProfileAction(
        "ego",
        [scn.SpeedProfileEntry(10.0, None)],
        scn.FollowingMode.Position,
        "",
        scn.DynamicConstraints(max_acceleration=5.0),
    )
    assert profile.constraints is not None
    assert profile.constraints.max_acceleration == pytest.approx(5.0)
    assert not profile.is_relative

    engine = scn.Engine()
    assert engine.init(_scenario(profile, entities=(("ego", perf),))) == scn.Status.Ok
    assert engine.step(1.0) == scn.Status.Ok
    assert engine.state("ego").speed == pytest.approx(5.0)  # 10 / 5, not 10 / 1
    assert engine.step(1.0) == scn.Status.Ok
    assert engine.state("ego").speed == pytest.approx(10.0)


def test_entity_relative_speed_profile_entries_are_deltas() -> None:
    profile = scn.SpeedProfileAction(
        "ego",
        [scn.SpeedProfileEntry(-2.0, 2.0)],
        scn.FollowingMode.Position,
        "lead",
    )
    assert profile.is_relative
    assert profile.entity_ref == "lead"

    engine = scn.Engine()
    scenario = _scenario(
        profile,
        entities=(("ego", None), ("lead", None)),
        init_actions=(scn.SpeedAction("lead", target_speed=12.0),),
    )
    assert engine.init(scenario) == scn.Status.Ok
    assert engine.step(1.0) == scn.Status.Ok
    assert engine.state("ego").speed == pytest.approx(5.0)  # halfway to 12 - 2
    assert engine.step(1.0) == scn.Status.Ok
    assert engine.state("ego").speed == pytest.approx(10.0)


def test_unknown_profile_reference_entity_is_rejected() -> None:
    engine = scn.Engine()
    profile = scn.SpeedProfileAction(
        "ego", [scn.SpeedProfileEntry(1.0, 1.0)], scn.FollowingMode.Position, "ghost"
    )
    assert engine.init(_scenario(profile)) == scn.Status.SemanticError


def test_max_speed_advice_cites_its_rule_id() -> None:
    perf = scn.Performance(max_speed=20.0, max_acceleration=2.0, max_deceleration=2.0)
    td = scn.TransitionDynamics(
        shape=scn.DynamicsShape.Linear,
        dimension=scn.DynamicsDimension.Time,
        value=2.0,
        following_mode=scn.FollowingMode.Follow,
    )
    engine = scn.Engine()
    assert engine.init(_scenario(scn.SpeedAction("ego", 30.0, td), entities=(("ego", perf),))) == (
        scn.Status.Ok  # advisory only
    )
    rules = {diagnostic.rule_id for diagnostic in engine.diagnostics()}
    assert "asam.net:xosc:1.2.0:scenario_logic.targetspeed_maxspeed_general" in rules
