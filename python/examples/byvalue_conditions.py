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

"""Drive the ASAM OpenSCENARIO by-value conditions from the host side.

One engine-controlled entity is driven by four events, each guarded by a
different by-value condition (ASAM OpenSCENARIO XML 1.4.0 ByValueCondition
group):

* ParameterCondition       - a load-time constant compared to a literal (§9.1).
* VariableCondition         - a runtime variable the host flips mid-run (§6.12),
                              behind a rising edge.
* UserDefinedValueCondition - an external value the host injects.
* TimeOfDayCondition        - the simulated wall clock, anchored by the host and
                              advancing with simulation time.

The host loop declares the parameter and the variable, seeds a time-of-day
anchor, steps the engine, and injects the variable / user value at chosen
steps. Each event sets a distinct speed, so the run asserts every by-value
path fired by watching the elements reach completeState.
"""

import scena as scn

EVENTS = ("on-parameter", "on-variable", "on-user-value", "on-time")


def _event(name: str, condition, target_speed: float, edge=scn.ConditionEdge.NoEdge):
    event = scn.Event(name, start_trigger=scn.make_trigger(condition, edge=edge))
    event.add_action(scn.SpeedAction("ego", target_speed=target_speed))
    return event


def build_scenario() -> "scn.Scenario":
    scenario = scn.Scenario("byvalue-conditions")
    scenario.add_entity(scn.Entity("ego", "ego vehicle", scn.ControlMode.EngineControlled))
    # Load-time declarations: the parameter is immutable, the variable seeds a
    # runtime store the host can change.
    scenario.set_parameter("speedLimit", "30")
    scenario.declare_variable("gate", "closed")

    maneuver = scn.Maneuver("maneuver")
    maneuver.add_event(
        _event(
            "on-parameter",
            scn.ParameterCondition("speedLimit", scn.Rule.EqualTo, "30"),
            5.0,
        )
    )
    maneuver.add_event(
        _event(
            "on-variable",
            scn.VariableCondition("gate", scn.Rule.EqualTo, "open"),
            15.0,
            edge=scn.ConditionEdge.Rising,
        )
    )
    maneuver.add_event(
        _event(
            "on-user-value",
            scn.UserDefinedValueCondition("sensor", scn.Rule.GreaterThan, "10"),
            25.0,
        )
    )
    reference = scn.DateTime(year=2000, month=1, day=1, hour=12, minute=0, second=2)
    maneuver.add_event(
        _event(
            "on-time",
            scn.TimeOfDayCondition(reference, scn.Rule.GreaterOrEqual),
            35.0,
        )
    )

    group = scn.ManeuverGroup("group")
    group.add_maneuver(maneuver)
    act = scn.Act("act")
    act.add_group(group)
    story = scn.Story("story")
    story.add_act(act)
    scenario.add_story(story)
    return scenario


def path(event: str) -> str:
    return f"story/act/group/maneuver/{event}"


def main() -> None:
    engine = scn.Engine()
    # Anchor the simulated clock to noon UTC at t = 0 before init, so the
    # TimeOfDayCondition has a reference from the very first evaluation.
    assert engine.set_date_time(scn.DateTime(year=2000, month=1, day=1, hour=12)) == scn.Status.Ok
    assert engine.init(build_scenario()) == scn.Status.Ok

    # The parameter matches at t = 0, so its event has already completed.
    assert engine.storyboard_element_state(path("on-parameter")) == scn.ElementState.Complete
    assert engine.state("ego").speed == 5.0

    for step in range(40):
        if step == 5:
            # Flip the variable: the rising-edge VariableCondition fires next step.
            assert engine.set_variable("gate", "open") == scn.Status.Ok
            assert engine.variable("gate") == "open"
        if step == 10:
            # Inject an external value above the threshold.
            assert engine.set_user_defined_value("sensor", "12") == scn.Status.Ok
            assert engine.user_defined_value("sensor") == "12"
        assert engine.step(0.1) == scn.Status.Ok

    # Every by-value path fired and completed.
    for event in EVENTS:
        assert engine.storyboard_element_state(path(event)) == scn.ElementState.Complete, event

    # The simulated clock advanced with simulation time past the reference.
    assert engine.date_time is not None
    assert engine.date_time >= scn.DateTime(year=2000, month=1, day=1, hour=12, minute=0, second=2).to_epoch_seconds()

    # An undeclared variable is rejected, not silently accepted.
    assert engine.set_variable("undeclared", "1") == scn.Status.UnknownName

    print("by-value conditions: all four events fired;", f"final ego speed {engine.state('ego').speed} m/s")


if __name__ == "__main__":
    main()
