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

"""Global actions through the Python bindings (p5-s6, ASAM OpenSCENARIO XML
1.4.0 §7.4.2 / §7.4.3): variable and deprecated parameter actions over the
runtime named-value stores, the add/delete entity lifecycle, the environment
store and its time-of-day clock, and CustomCommandAction.

Traffic signals live in test_traffic_signals.py."""

import pytest

import scena as scn


def _scenario(events, *, entities=(("ego", scn.ControlMode.EngineControlled),), init=()):
    """A scenario with `entities` and one maneuver holding `events`."""
    scenario = scn.Scenario("global-actions")
    for entity_id, mode in entities:
        scenario.add_entity(scn.Entity(entity_id, entity_id, mode))
    for action in init:
        scenario.add_init_action(action)

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


def _gated_event(name, condition, action):
    event = scn.Event(name, start_trigger=scn.make_trigger(condition))
    event.add_action(action)
    return event


# --- Variables (§6.12) -----------------------------------------------------


def test_variable_set_action_updates_runtime_store() -> None:
    scenario = _scenario([_timed_event("set", 1.0, scn.VariableSetAction("trigger", "true"))])
    scenario.declare_variable("trigger", "false")

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    assert engine.variable("trigger") == "false"
    assert engine.step(1.0) == scn.Status.Ok
    assert engine.variable("trigger") == "true"


def test_variable_modify_add_is_exact() -> None:
    action = scn.VariableModifyAction("counter", scn.ModifyOperator.Add, 0.2)
    scenario = _scenario([_timed_event("add", 1.0, action)])
    scenario.declare_variable("counter", "0.1")

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    assert engine.step(1.0) == scn.Status.Ok
    # The store holds the exact double, not a rounded rendering of it.
    assert engine.variable("counter") == "0.30000000000000004"
    assert float(engine.variable("counter")) == 0.1 + 0.2


def test_variable_modify_multiply_is_exact() -> None:
    action = scn.VariableModifyAction("gain", scn.ModifyOperator.Multiply, 4.0)
    scenario = _scenario([_timed_event("scale", 1.0, action)])
    scenario.declare_variable("gain", "2.5")

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    assert engine.step(1.0) == scn.Status.Ok
    assert engine.variable("gain") == "10"


def test_variable_modify_on_non_numeric_warns_and_no_ops() -> None:
    action = scn.VariableModifyAction("mode", scn.ModifyOperator.Multiply, 2.0)
    scenario = _scenario([_timed_event("modify", 1.0, action)])
    scenario.declare_variable("mode", "cruise")

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    assert engine.step(1.0) == scn.Status.Ok
    # Rule data_type.variable_modification_or_comparison_possible (C.2.6): a
    # modify action acts on numeric types only.
    assert engine.variable("mode") == "cruise"
    assert any("non-numeric" in d.message for d in engine.diagnostics())


def test_variable_action_visible_to_variable_condition() -> None:
    scenario = _scenario(
        [
            _timed_event("arm", 1.0, scn.VariableSetAction("armed", "true")),
            _gated_event(
                "gated",
                scn.VariableCondition("armed", scn.Rule.EqualTo, "true"),
                scn.VariableSetAction("armed", "consumed"),
            ),
        ]
    )
    scenario.declare_variable("armed", "false")

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    assert engine.step(1.0) == scn.Status.Ok
    assert engine.step(1.0) == scn.Status.Ok
    # The §6.12 loop closed inside the scenario, with no host in the middle.
    assert engine.variable("armed") == "consumed"


def test_variable_action_unknown_ref_fails_init() -> None:
    scenario = _scenario([_timed_event("set", 1.0, scn.VariableSetAction("missing", "1"))])

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.SemanticError
    assert any(
        d.rule_id == "asam.net:xosc:1.2.0:reference_control.resolvable_variable_reference"
        for d in engine.diagnostics()
    )


# --- Deprecated parameter actions ------------------------------------------


def test_parameter_set_action_overlays_declaration_and_deprecates() -> None:
    scenario = _scenario(
        [
            _timed_event("raise", 1.0, scn.ParameterSetAction("speedLimit", "50")),
            _gated_event(
                "gated",
                scn.ParameterCondition("speedLimit", scn.Rule.EqualTo, "50"),
                scn.VariableSetAction("seen", "yes"),
            ),
        ]
    )
    scenario.set_parameter("speedLimit", "30")
    scenario.declare_variable("seen", "no")

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    assert engine.step(1.0) == scn.Status.Ok
    assert engine.step(1.0) == scn.Status.Ok
    # A 1.0/1.1 file's ParameterCondition observes the overlay the deprecated
    # action wrote, while §9.1 immutability stands for 1.2+ content.
    assert engine.variable("seen") == "yes"
    assert any(d.code == scn.Status.DeprecatedFeature for d in engine.diagnostics())


def test_parameter_modify_action_chains_on_the_overlay() -> None:
    scenario = _scenario(
        [
            _timed_event(
                "widen", 1.0, scn.ParameterModifyAction("gap", scn.ModifyOperator.Add, 4.0)
            ),
            _timed_event(
                "double",
                2.0,
                scn.ParameterModifyAction("gap", scn.ModifyOperator.Multiply, 2.0),
            ),
            _gated_event(
                "gated",
                scn.ParameterCondition("gap", scn.Rule.EqualTo, "24"),
                scn.VariableSetAction("seen", "yes"),
            ),
        ]
    )
    scenario.set_parameter("gap", "8")  # (8 + 4) * 2 = 24
    scenario.declare_variable("seen", "no")

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    for _ in range(4):
        assert engine.step(1.0) == scn.Status.Ok
    assert engine.variable("seen") == "yes"


# --- Entity lifecycle (§EntityAction) --------------------------------------


def test_delete_then_add_respawns_the_entity() -> None:
    scenario = _scenario(
        [
            _timed_event("remove", 1.0, scn.DeleteEntityAction("ego")),
            _timed_event(
                "restore", 2.0, scn.AddEntityAction("ego", scn.WorldPosition(-30.0, 4.0, 0.0))
            ),
        ],
        init=[scn.SpeedAction("ego", 12.0)],
    )

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    assert engine.entity_active("ego") is True
    assert engine.state("ego") is not None

    assert engine.step(1.0) == scn.Status.Ok
    assert engine.entity_active("ego") is False
    # A deleted entity reports nothing at all: the host sees what the
    # by-entity conditions see.
    assert engine.state("ego") is None
    assert engine.visibility_of("ego") is None
    assert engine.controller_activation_of("ego") is None
    # An id the scenario never declared is distinct from a deleted one.
    assert engine.entity_active("nobody") is None

    assert engine.step(1.0) == scn.Status.Ok
    assert engine.entity_active("ego") is True
    state = engine.state("ego")
    assert state is not None
    assert state.x == pytest.approx(-30.0)
    assert state.y == pytest.approx(4.0)
    # A re-added entity starts from rest; the delete released its speed action.
    assert state.speed == pytest.approx(0.0)


def test_adding_an_active_entity_is_a_no_op() -> None:
    scenario = _scenario(
        [_timed_event("add", 1.0, scn.AddEntityAction("ego", scn.WorldPosition(500.0, 0.0, 0.0)))],
        init=[scn.SpeedAction("ego", 10.0)],
    )

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    assert engine.step(1.0) == scn.Status.Ok
    # §EntityAction: "Adding an already active entity will have no effect."
    assert engine.state("ego").x == pytest.approx(10.0)
    assert engine.state("ego").speed == pytest.approx(10.0)


def test_report_state_on_a_deleted_entity_is_rejected() -> None:
    scenario = _scenario(
        [_timed_event("remove", 1.0, scn.DeleteEntityAction("host"))],
        entities=(("host", scn.ControlMode.HostControlled),),
    )

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    assert engine.report_state("host", scn.EntityState()) == scn.Status.Ok
    assert engine.step(1.0) == scn.Status.Ok
    assert engine.report_state("host", scn.EntityState()) == scn.Status.UnknownEntity


def test_entity_action_unknown_ref_fails_init() -> None:
    scenario = _scenario([_timed_event("add", 1.0, scn.DeleteEntityAction("ghost"))])
    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.SemanticError
    assert any("undeclared entity 'ghost'" in d.message for d in engine.diagnostics())


# --- EnvironmentAction (§Environment) --------------------------------------


def test_environment_action_merges_partial_updates() -> None:
    first = scn.Environment()
    first.name = "dusk"
    weather = scn.Weather()
    weather.fog = scn.Fog(120.0)
    weather.temperature = 288.0
    first.weather = weather
    first.road_condition = scn.RoadCondition(0.8)

    second = scn.Environment()
    rain = scn.Weather()
    rain.precipitation = scn.Precipitation(scn.PrecipitationType.Rain, 4.5)
    second.weather = rain

    scenario = _scenario(
        [
            _timed_event("dusk", 1.0, scn.EnvironmentAction(first)),
            _timed_event("rain", 2.0, scn.EnvironmentAction(second)),
        ]
    )

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    assert engine.environment.weather is None  # nothing set yet
    assert engine.step(1.0) == scn.Status.Ok
    assert engine.environment.weather.fog.visual_range == pytest.approx(120.0)

    assert engine.step(1.0) == scn.Status.Ok
    merged = engine.environment
    # §Weather: "If one of the conditions is missing it means that it doesn't
    # change" — the fog and the road condition survive the second update.
    assert merged.name == "dusk"
    assert merged.weather.fog.visual_range == pytest.approx(120.0)
    assert merged.weather.temperature == pytest.approx(288.0)
    assert merged.weather.precipitation.type == scn.PrecipitationType.Rain
    assert merged.weather.precipitation.intensity == pytest.approx(4.5)
    assert merged.weather.wind is None
    assert merged.weather.sun is None
    assert merged.road_condition.friction_scale_factor == pytest.approx(0.8)


@pytest.mark.parametrize("animation", [True, False])
def test_time_of_day_animated_and_frozen(animation: bool) -> None:
    environment = scn.Environment()
    environment.time_of_day = scn.TimeOfDay(animation, scn.DateTime(2026, 7, 22, 12, 0, 0, 0, 0))
    scenario = _scenario([], init=[scn.EnvironmentAction(environment)])

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    anchor = engine.date_time
    assert anchor is not None
    for _ in range(5):
        assert engine.step(1.0) == scn.Status.Ok
    if animation:
        assert engine.date_time == pytest.approx(anchor + 5.0)
    else:
        # §TimeOfDay animation="false": bit-identical, never moves.
        assert engine.date_time == anchor


def test_frozen_time_of_day_holds_a_time_of_day_condition_false() -> None:
    environment = scn.Environment()
    environment.time_of_day = scn.TimeOfDay(False, scn.DateTime(2026, 7, 22, 11, 59, 58, 0, 0))
    scenario = _scenario(
        [
            _gated_event(
                "noon",
                scn.TimeOfDayCondition(
                    scn.DateTime(2026, 7, 22, 12, 0, 0, 0, 0), scn.Rule.GreaterOrEqual
                ),
                scn.VariableSetAction("seen", "yes"),
            )
        ],
        init=[scn.EnvironmentAction(environment)],
    )
    scenario.declare_variable("seen", "no")

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    for _ in range(20):
        assert engine.step(1.0) == scn.Status.Ok
    assert engine.variable("seen") == "no"


def test_environment_invalid_date_time_fails_init() -> None:
    environment = scn.Environment()
    # February 30 does not exist in any year.
    environment.time_of_day = scn.TimeOfDay(True, scn.DateTime(2026, 2, 30, 12, 0, 0, 0, 0))
    scenario = _scenario([_timed_event("bad", 1.0, scn.EnvironmentAction(environment))])

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.ValidationError
    assert any(
        d.rule_id == "asam.net:xosc:1.0.0:data_type.time_format" for d in engine.diagnostics()
    )


def test_environment_out_of_range_weather_fails_init() -> None:
    environment = scn.Environment()
    weather = scn.Weather()
    weather.temperature = 100.0  # below the §Weather range [170..340] K
    environment.weather = weather
    scenario = _scenario([_timed_event("cold", 1.0, scn.EnvironmentAction(environment))])

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.ValidationError


# --- CustomCommandAction (§7.4.3) ------------------------------------------


def test_custom_command_action_without_a_gateway_is_a_silent_no_op() -> None:
    scenario = _scenario(
        [_timed_event("command", 1.0, scn.CustomCommandAction("script", "run.py --fast"))]
    )

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    assert engine.step(1.0) == scn.Status.Ok
    # §7.4.3 makes executability depend on the host recognizing the action, so
    # no host means no effect — and no diagnostic.
    assert engine.diagnostics() == []
    assert (
        engine.storyboard_element_state("story/act/group/maneuver/command")
        == scn.ElementState.Complete
    )


def test_custom_command_action_accessors() -> None:
    action = scn.CustomCommandAction("script", "run.py")
    assert action.type == "script"
    assert action.content == "run.py"


# --- The actor-less base ----------------------------------------------------


def test_global_actions_carry_no_entity_id() -> None:
    actions = [
        scn.VariableSetAction("v", "1"),
        scn.VariableModifyAction("v", scn.ModifyOperator.Add, 1.0),
        scn.ParameterSetAction("p", "1"),
        scn.ParameterModifyAction("p", scn.ModifyOperator.Multiply, 2.0),
        scn.AddEntityAction("e", scn.WorldPosition()),
        scn.DeleteEntityAction("e"),
        scn.EnvironmentAction(scn.Environment()),
        scn.TrafficSignalStateAction("s", "on"),
        scn.TrafficSignalControllerAction("c", "p"),
        scn.CustomCommandAction("t", "c"),
    ]
    for action in actions:
        assert isinstance(action, scn.GlobalAction)
        assert isinstance(action, scn.Action)
        assert action.entity_id == ""
    # kind() is the stable ASAM element name, used in runtime diagnostics.
    assert [action.kind for action in actions] == [
        "VariableSetAction",
        "VariableModifyAction",
        "ParameterSetAction",
        "ParameterModifyAction",
        "AddEntityAction",
        "DeleteEntityAction",
        "EnvironmentAction",
        "TrafficSignalStateAction",
        "TrafficSignalControllerAction",
        "CustomCommandAction",
    ]
    # A private action is not a global one — the branch that matters.
    assert not isinstance(scn.SpeedAction("ego", 10.0), scn.GlobalAction)
