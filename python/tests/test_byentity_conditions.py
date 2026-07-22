# SPDX-License-Identifier: Apache-2.0
"""By-entity conditions through the Python bindings (ASAM OpenSCENARIO XML 1.4.0 §7.6.5.1)."""

import pytest

import scena as scn


def _scenario(condition, observed=("ego",)):
    """A scenario whose event fires on `condition`, driving an engine 'probe'."""
    scenario = scn.Scenario("byentity")
    for entity_id in observed:
        scenario.add_entity(scn.Entity(entity_id, entity_id, scn.ControlMode.HostControlled))
    scenario.add_entity(scn.Entity("probe", "probe", scn.ControlMode.EngineControlled))

    event = scn.Event("event", start_trigger=scn.make_trigger(condition))
    event.add_action(scn.SpeedAction("probe", target_speed=99.0))
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


def _probe_fired(engine):
    state = engine.state("probe")
    return state is not None and state.speed != 0.0


def _ego(rule=scn.TriggeringEntitiesRule.Any):
    return scn.TriggeringEntities(["ego"], rule=rule)


def test_enums_and_value_types_round_trip():
    assert {r.name for r in scn.TriggeringEntitiesRule} == {"Any", "All"}
    assert {d.name for d in scn.DirectionalDimension} == {"Longitudinal", "Lateral", "Vertical"}

    triggering = scn.TriggeringEntities(["a", "b"], rule=scn.TriggeringEntitiesRule.All)
    assert triggering.rule == scn.TriggeringEntitiesRule.All
    assert triggering.entity_refs == ["a", "b"]
    # Default rule is Any.
    assert scn.TriggeringEntities(["a"]).rule == scn.TriggeringEntitiesRule.Any

    position = scn.WorldPosition(x=1.0, y=2.0, z=3.0)
    assert (position.x, position.y, position.z) == (1.0, 2.0, 3.0)


def test_condition_classes_expose_their_fields():
    speed = scn.SpeedCondition(_ego(), 5.0, scn.Rule.GreaterThan, scn.DirectionalDimension.Longitudinal)
    assert speed.value == 5.0
    assert speed.rule == scn.Rule.GreaterThan
    assert speed.direction == scn.DirectionalDimension.Longitudinal
    assert speed.triggering_entities.entity_refs == ["ego"]
    # direction is optional.
    assert scn.SpeedCondition(_ego(), 1.0, scn.Rule.LessThan).direction is None

    relative = scn.RelativeSpeedCondition(_ego(), "lead", -2.0, scn.Rule.LessThan)
    assert relative.entity_ref == "lead"
    assert relative.value == -2.0

    accel = scn.AccelerationCondition(_ego(), 1.5, scn.Rule.GreaterOrEqual)
    assert accel.value == 1.5

    assert scn.StandStillCondition(_ego(), 2.0).duration == 2.0
    assert scn.TraveledDistanceCondition(_ego(), 10.0).value == 10.0

    reach = scn.ReachPositionCondition(_ego(), scn.WorldPosition(1.0, 2.0, 0.0), 3.0)
    assert reach.tolerance == 3.0
    assert reach.position.x == 1.0


def test_speed_condition_flips_probe():
    engine = scn.Engine()
    assert engine.init(_scenario(scn.SpeedCondition(_ego(), 5.0, scn.Rule.GreaterOrEqual)))
    assert not _probe_fired(engine)  # ego at rest

    fast = scn.EntityState(speed=6.0)
    assert engine.report_state("ego", fast) == scn.Status.Ok
    assert engine.step(0.1) == scn.Status.Ok
    assert _probe_fired(engine)


@pytest.mark.parametrize(
    "rule,host_fast,auto_fast,expected",
    [
        (scn.TriggeringEntitiesRule.Any, True, False, True),
        (scn.TriggeringEntitiesRule.Any, False, False, False),
        (scn.TriggeringEntitiesRule.All, True, False, False),
        (scn.TriggeringEntitiesRule.All, True, True, True),
    ],
)
def test_any_all_reduction(rule, host_fast, auto_fast, expected):
    triggering = scn.TriggeringEntities(["host", "auto"], rule=rule)
    scenario = scn.Scenario("anyall")
    scenario.add_entity(scn.Entity("host", "host", scn.ControlMode.HostControlled))
    scenario.add_entity(scn.Entity("auto", "auto", scn.ControlMode.HostControlled))
    scenario.add_entity(scn.Entity("probe", "probe", scn.ControlMode.EngineControlled))
    event = scn.Event(
        "event",
        start_trigger=scn.make_trigger(scn.SpeedCondition(triggering, 5.0, scn.Rule.GreaterOrEqual)),
    )
    event.add_action(scn.SpeedAction("probe", target_speed=99.0))
    maneuver = scn.Maneuver("maneuver")
    maneuver.add_event(event)
    group = scn.ManeuverGroup("group")
    group.add_maneuver(maneuver)
    act = scn.Act("act")
    act.add_group(group)
    story = scn.Story("story")
    story.add_act(act)
    scenario.add_story(story)

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    if host_fast:
        engine.report_state("host", scn.EntityState(speed=6.0))
    if auto_fast:
        engine.report_state("auto", scn.EntityState(speed=6.0))
    assert engine.step(0.1) == scn.Status.Ok
    assert _probe_fired(engine) == expected


def test_dangling_triggering_entity_fails_init():
    engine = scn.Engine()
    condition = scn.SpeedCondition(
        scn.TriggeringEntities(["ghost"]), 1.0, scn.Rule.GreaterThan
    )
    assert engine.init(_scenario(condition)) == scn.Status.SemanticError


def test_reach_position_deprecation_warns_but_init_succeeds():
    engine = scn.Engine()
    condition = scn.ReachPositionCondition(_ego(), scn.WorldPosition(0.0, 0.0, 0.0), 1.0)
    assert engine.init(_scenario(condition)) == scn.Status.Ok
    warnings = [
        d
        for d in engine.diagnostics()
        if d.severity == scn.Severity.Warning and d.code == scn.Status.DeprecatedFeature
    ]
    assert len(warnings) == 1
