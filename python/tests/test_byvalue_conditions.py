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

"""By-value conditions through the Python bindings (ASAM OpenSCENARIO XML 1.4.0)."""

import scena as scn


def _scenario(condition, target_speed=10.0, edge=scn.ConditionEdge.NoEdge):
    """A one-entity scenario whose only event fires on `condition`."""
    scenario = scn.Scenario("byvalue")
    scenario.add_entity(scn.Entity("ego", "ego", scn.ControlMode.EngineControlled))
    event = scn.Event("event", start_trigger=scn.make_trigger(condition, edge=edge))
    event.add_action(scn.SpeedAction("ego", target_speed=target_speed))
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


def _fired(engine):
    state = engine.state("ego")
    return state is not None and state.speed != 0.0


def test_rule_enum_exposes_all_six_operators():
    assert {r.name for r in scn.Rule} == {
        "EqualTo",
        "GreaterThan",
        "LessThan",
        "GreaterOrEqual",
        "LessOrEqual",
        "NotEqualTo",
    }


def test_datetime_construction_validity_and_epoch():
    epoch = scn.DateTime()
    assert epoch.valid()
    assert epoch.to_epoch_seconds() == 0.0

    y2k = scn.DateTime(year=2000, month=1, day=1)
    assert y2k.to_epoch_seconds() == 946684800.0
    # A +01:00 offset is one hour ahead of UTC.
    cet = scn.DateTime(year=1970, month=1, day=1, hour=1, utc_offset_minutes=60)
    assert cet.to_epoch_seconds() == 0.0
    assert not scn.DateTime(year=2001, month=2, day=29).valid()  # not a leap year
    assert "DateTime(year=2000" in repr(y2k)


def test_simulation_time_condition_gains_rule():
    condition = scn.SimulationTimeCondition(at_time=2.5, rule=scn.Rule.GreaterThan)
    assert condition.value == 2.5
    assert condition.at_time == 2.5
    assert condition.rule == scn.Rule.GreaterThan
    # Default rule preserves the historical greater-or-equal behavior.
    assert scn.SimulationTimeCondition(at_time=1.0).rule == scn.Rule.GreaterOrEqual


def test_condition_classes_expose_their_fields():
    parameter = scn.ParameterCondition("p", scn.Rule.EqualTo, "5")
    assert (parameter.parameter_ref, parameter.rule, parameter.value) == ("p", scn.Rule.EqualTo, "5")

    variable = scn.VariableCondition("v", scn.Rule.LessThan, "9")
    assert (variable.variable_ref, variable.rule, variable.value) == ("v", scn.Rule.LessThan, "9")

    user = scn.UserDefinedValueCondition("u", scn.Rule.NotEqualTo, "x")
    assert (user.name, user.rule, user.value) == ("u", scn.Rule.NotEqualTo, "x")

    time_of_day = scn.TimeOfDayCondition(scn.DateTime(year=2000), scn.Rule.GreaterOrEqual)
    assert time_of_day.rule == scn.Rule.GreaterOrEqual
    assert time_of_day.date_time.year == 2000

    element = scn.StoryboardElementStateCondition(
        scn.StoryboardElementType.Event, "other", scn.StoryboardElementState.CompleteState
    )
    assert element.element_type == scn.StoryboardElementType.Event
    assert element.element_ref == "other"
    assert element.state == scn.StoryboardElementState.CompleteState


def test_parameter_condition_matches_declared_value():
    scenario = _scenario(scn.ParameterCondition("speedLimit", scn.Rule.EqualTo, "30"))
    scenario.set_parameter("speedLimit", "30")
    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    assert _fired(engine)  # matches at t = 0


def test_dangling_parameter_ref_fails_init():
    engine = scn.Engine()
    assert engine.init(_scenario(scn.ParameterCondition("missing", scn.Rule.EqualTo, "1"))) == (
        scn.Status.SemanticError
    )


def test_variable_host_interface():
    scenario = _scenario(
        scn.VariableCondition("gate", scn.Rule.EqualTo, "open"), edge=scn.ConditionEdge.Rising
    )
    scenario.declare_variable("gate", "closed")
    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    assert engine.variable("gate") == "closed"
    assert engine.step(0.1) == scn.Status.Ok
    assert not _fired(engine)

    assert engine.set_variable("gate", "open") == scn.Status.Ok
    assert engine.step(0.1) == scn.Status.Ok  # rising edge fires
    assert _fired(engine)

    assert engine.set_variable("undeclared", "1") == scn.Status.UnknownName
    assert engine.variable("undeclared") is None


def test_user_defined_value_host_interface():
    engine = scn.Engine()
    assert engine.set_user_defined_value("sensor", "2") == scn.Status.Ok  # staged before init
    assert engine.init(_scenario(scn.UserDefinedValueCondition("sensor", scn.Rule.GreaterThan, "3")))
    assert engine.user_defined_value("sensor") == "2"
    assert not _fired(engine)  # 2 > 3 is false

    assert engine.set_user_defined_value("sensor", "5") == scn.Status.Ok
    assert engine.step(0.1) == scn.Status.Ok
    assert _fired(engine)  # 5 > 3 holds


def test_time_of_day_advances_with_simulation_time():
    engine = scn.Engine()
    assert engine.date_time is None  # no anchor yet
    assert engine.set_date_time(scn.DateTime(year=2000, month=1, day=1, hour=12)) == scn.Status.Ok
    reference = scn.DateTime(year=2000, month=1, day=1, hour=12, minute=0, second=2)
    assert engine.init(_scenario(scn.TimeOfDayCondition(reference, scn.Rule.GreaterOrEqual)))
    assert not _fired(engine)  # t = 0, before the reference
    for _ in range(30):
        assert engine.step(0.1) == scn.Status.Ok
    assert _fired(engine)  # simulated clock passed the reference
    assert engine.date_time >= reference.to_epoch_seconds()

    # An out-of-range date-time is rejected.
    assert engine.set_date_time(scn.DateTime(year=2001, month=2, day=29)) == scn.Status.InvalidArgument


def test_storyboard_element_state_condition_observes_completion():
    scenario = scn.Scenario("element")
    scenario.add_entity(scn.Entity("ego", "ego", scn.ControlMode.EngineControlled))
    source = scn.Event("source", start_trigger=scn.make_trigger(scn.SimulationTimeCondition(at_time=1.0)))
    source.add_action(scn.SpeedAction("ego", target_speed=5.0))
    observer = scn.Event(
        "observer",
        start_trigger=scn.make_trigger(
            scn.StoryboardElementStateCondition(
                scn.StoryboardElementType.Event, "source", scn.StoryboardElementState.CompleteState
            )
        ),
    )
    observer.add_action(scn.SpeedAction("ego", target_speed=9.0))
    maneuver = scn.Maneuver("maneuver")
    maneuver.add_event(source)
    maneuver.add_event(observer)
    group = scn.ManeuverGroup("group")
    group.add_maneuver(maneuver)
    act = scn.Act("act")
    act.add_group(group)
    story = scn.Story("story")
    story.add_act(act)
    scenario.add_story(story)

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    assert engine.step(1.0) == scn.Status.Ok  # source completes, observer sees it same evaluation
    assert engine.storyboard_element_state("story/act/group/maneuver/observer") == (
        scn.ElementState.Complete
    )


def test_dangling_storyboard_element_ref_fails_init():
    engine = scn.Engine()
    condition = scn.StoryboardElementStateCondition(
        scn.StoryboardElementType.Event, "ghost", scn.StoryboardElementState.CompleteState
    )
    assert engine.init(_scenario(condition)) == scn.Status.SemanticError
