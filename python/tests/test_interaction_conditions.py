# SPDX-FileCopyrightText: 2026 Robomous
# SPDX-License-Identifier: Apache-2.0
"""Interaction conditions through the Python bindings (ASAM OpenSCENARIO XML 1.4.0 §6.4)."""

import pytest

import scena as scn


def _ego(rule=scn.TriggeringEntitiesRule.Any):
    return scn.TriggeringEntities(["ego"], rule=rule)


def _box(length=4.0, width=2.0):
    return scn.BoundingBox(length=length, width=width)


def _scenario(condition, entities):
    """A scenario firing an engine 'probe' on `condition`. `entities` is a list
    of (id, ControlMode, bounding_box | None)."""
    scenario = scn.Scenario("interaction")
    for entity_id, mode, box in entities:
        obj = scn.MiscObject(bounding_box=box) if box is not None else None
        scenario.add_entity(scn.Entity(entity_id, entity_id, mode, object=obj))
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


def test_enums_round_trip():
    assert {c.name for c in scn.CoordinateSystem} == {
        "Entity",
        "Lane",
        "Road",
        "Trajectory",
        "World",
    }
    assert {d.name for d in scn.RelativeDistanceType} == {
        "Longitudinal",
        "Lateral",
        "CartesianDistance",
        "EuclidianDistance",
    }
    assert {r.name for r in scn.RoutingAlgorithm} == {
        "AssignedRoute",
        "Fastest",
        "LeastIntersections",
        "Shortest",
        "Undefined",
    }


def test_bounding_box_fields():
    box = scn.BoundingBox(center_x=1.5, center_z=0.9, length=4.0, width=2.0, height=1.8)
    assert box.center_x == 1.5
    assert box.center_y == 0.0
    assert box.length == 4.0
    assert box.width == 2.0
    # Entity derives its bounding box from the concrete object it carries.
    entity = scn.Entity(
        "ego", "ego", scn.ControlMode.HostControlled, object=scn.MiscObject(bounding_box=box)
    )
    assert entity.bounding_box is not None
    assert entity.bounding_box.length == 4.0
    assert scn.Entity("plain", "plain").bounding_box is None


def test_relative_lane_range_uses_from_lane_to_lane():
    lane_range = scn.RelativeLaneRange(from_lane=-1, to_lane=2)
    assert lane_range.from_lane == -1
    assert lane_range.to_lane == 2
    # Both default to None (-inf / +inf).
    default = scn.RelativeLaneRange()
    assert default.from_lane is None
    assert default.to_lane is None


def test_condition_classes_expose_fields():
    distance = scn.DistanceCondition(
        _ego(),
        scn.WorldPosition(3.0, 4.0, 0.0),
        5.0,
        True,
        scn.Rule.LessOrEqual,
        coordinate_system=scn.CoordinateSystem.World,
        relative_distance_type=scn.RelativeDistanceType.Longitudinal,
    )
    assert distance.value == 5.0
    assert distance.freespace is True
    assert distance.position.x == 3.0
    assert distance.coordinate_system == scn.CoordinateSystem.World
    # Optionals default to None.
    assert scn.DistanceCondition(_ego(), scn.WorldPosition(), 1.0, False, scn.Rule.LessThan).along_route is None

    rel = scn.RelativeDistanceCondition(
        _ego(), "lead", 2.0, True, scn.RelativeDistanceType.EuclidianDistance, scn.Rule.LessThan
    )
    assert rel.entity_ref == "lead"
    assert rel.relative_distance_type == scn.RelativeDistanceType.EuclidianDistance

    thw = scn.TimeHeadwayCondition(_ego(), "lead", 3.0, False, scn.Rule.LessOrEqual)
    assert thw.entity_ref == "lead"
    assert thw.value == 3.0

    clearance = scn.RelativeClearanceCondition(
        _ego(), True, False, distance_forward=10.0, entity_refs=["lead"]
    )
    assert clearance.free_space is True
    assert clearance.distance_forward == 10.0
    assert clearance.entity_refs == ["lead"]


def test_ttc_requires_exactly_one_target():
    entity_target = scn.TimeToCollisionCondition(_ego(), 2.0, False, scn.Rule.LessThan, entity_ref="lead")
    assert entity_target.entity_ref == "lead"
    assert entity_target.position is None

    position_target = scn.TimeToCollisionCondition(
        _ego(), 2.0, False, scn.Rule.LessThan, position=scn.WorldPosition(20.0, 0.0, 0.0)
    )
    assert position_target.position.x == 20.0
    assert position_target.entity_ref is None

    with pytest.raises(ValueError):
        scn.TimeToCollisionCondition(_ego(), 2.0, False, scn.Rule.LessThan)  # neither
    with pytest.raises(ValueError):
        scn.TimeToCollisionCondition(
            _ego(), 2.0, False, scn.Rule.LessThan, entity_ref="lead",
            position=scn.WorldPosition(),
        )  # both


def test_collision_fires_when_boxes_overlap():
    # Both boxed entities start at the origin, so their boxes overlap and the
    # collision condition holds at t = 0.
    condition = scn.CollisionCondition(_ego(), "lead")
    scenario = _scenario(
        condition,
        [
            ("ego", scn.ControlMode.HostControlled, _box()),
            ("lead", scn.ControlMode.HostControlled, _box()),
        ],
    )
    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    assert _probe_fired(engine)


def test_collision_without_geometry_is_false():
    # lead has no bounding box ⇒ nothing to intersect ⇒ never a collision, even
    # coincident with ego.
    condition = scn.CollisionCondition(_ego(), "lead")
    scenario = _scenario(
        condition,
        [
            ("ego", scn.ControlMode.HostControlled, _box()),
            ("lead", scn.ControlMode.HostControlled, None),
        ],
    )
    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    assert not _probe_fired(engine)
    assert engine.step(0.1) == scn.Status.Ok
    assert not _probe_fired(engine)


def test_distance_condition_fires():
    # ego reaches within 3 m of a fixed position (reference-point euclidean).
    condition = scn.DistanceCondition(
        _ego(), scn.WorldPosition(10.0, 0.0, 0.0), 3.0, False, scn.Rule.LessOrEqual
    )
    scenario = _scenario(condition, [("ego", scn.ControlMode.HostControlled, None)])
    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    assert not _probe_fired(engine)

    engine.report_state("ego", scn.EntityState(x=8.0))  # distance 2 <= 3
    assert engine.step(0.1) == scn.Status.Ok
    assert _probe_fired(engine)


def test_negative_distance_value_fails_init():
    condition = scn.DistanceCondition(_ego(), scn.WorldPosition(), -1.0, False, scn.Rule.LessThan)
    engine = scn.Engine()
    assert (
        engine.init(_scenario(condition, [("ego", scn.ControlMode.HostControlled, None)]))
        == scn.Status.ValidationError
    )


def test_road_coordinate_system_warns():
    condition = scn.DistanceCondition(
        _ego(), scn.WorldPosition(), 1.0, False, scn.Rule.LessThan,
        coordinate_system=scn.CoordinateSystem.Road,
    )
    engine = scn.Engine()
    assert engine.init(_scenario(condition, [("ego", scn.ControlMode.HostControlled, None)])) == scn.Status.Ok
    warnings = [
        d
        for d in engine.diagnostics()
        if d.severity == scn.Severity.Warning and d.code == scn.Status.UnsupportedFeature
    ]
    assert len(warnings) == 1
