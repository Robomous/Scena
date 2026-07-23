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

"""LongitudinalDistanceAction through the Python bindings (p5-s5):
distance and timeGap targets, freespace gaps, DynamicConstraints and the
never-ending continuous mode (§LongitudinalDistanceAction)."""

import pytest

import scena as scn


def _following_scenario(events, *, lead_x=100.0, speed=10.0, boxed=False):
    """A 'lead' ahead of an 'ego' on the +x axis, both engine-controlled and
    cruising at `speed`, plus one maneuver holding `events`."""
    scenario = scn.Scenario("distance-keeping")
    for entity_id in ("lead", "ego"):
        if boxed:
            vehicle = scn.Vehicle(
                category=scn.VehicleCategory.Car,
                bounding_box=scn.BoundingBox(0.0, 0.0, 0.75, 4.0, 2.0, 1.5),
                performance=scn.Performance(60.0, 5.0, 5.0),
            )
            scenario.add_entity(
                scn.Entity(entity_id, entity_id, scn.ControlMode.EngineControlled, object=vehicle)
            )
        else:
            scenario.add_entity(
                scn.Entity(entity_id, entity_id, scn.ControlMode.EngineControlled)
            )
    scenario.add_init_action(scn.TeleportAction("lead", scn.WorldPosition(lead_x, 0.0, 0.0)))
    scenario.add_init_action(scn.SpeedAction("lead", speed))
    scenario.add_init_action(scn.SpeedAction("ego", speed))

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


def _gap(engine):
    return engine.state("lead").x - engine.state("ego").x


def test_distance_action_round_trip() -> None:
    constraints = scn.DynamicConstraints(max_acceleration=2.0, max_speed=30.0)
    action = scn.LongitudinalDistanceAction(
        "ego",
        "lead",
        distance=25.0,
        freespace=True,
        continuous=True,
        displacement=scn.LongitudinalDisplacement.LeadingReferencedEntity,
        constraints=constraints,
    )
    assert action.kind == "LongitudinalDistanceAction"
    assert action.entity_id == "ego"
    assert action.entity_ref == "lead"
    assert action.distance == pytest.approx(25.0)
    assert action.time_gap is None
    assert action.freespace is True
    assert action.continuous is True
    # §CoordinateSystem: omitted means "entity".
    assert action.coordinate_system == scn.CoordinateSystem.Entity
    assert action.displacement == scn.LongitudinalDisplacement.LeadingReferencedEntity
    assert action.constraints.max_acceleration == pytest.approx(2.0)
    # A missing constraint means unlimited, so it reads back as None.
    assert action.constraints.max_deceleration is None


def test_distance_target_is_reached_and_ends_the_action() -> None:
    engine = scn.Engine()
    events = [
        _timed_event(
            "keep",
            0.0,
            scn.LongitudinalDistanceAction("ego", "lead", distance=50.0),
        )
    ]
    assert engine.init(_following_scenario(events)) == scn.Status.Ok
    assert engine.step(1.0) == scn.Status.Ok
    # v_cmd = v_lead + (gap - target)/dt = 10 + 50/1.
    assert engine.state("ego").speed == pytest.approx(60.0)
    assert _gap(engine) == pytest.approx(50.0)
    assert engine.step(1.0) == scn.Status.Ok
    assert (
        engine.storyboard_element_state("story/act/group/maneuver/keep")
        == scn.ElementState.Complete
    )


def test_continuous_time_gap_never_completes() -> None:
    engine = scn.Engine()
    events = [
        _timed_event(
            "keep",
            0.0,
            scn.LongitudinalDistanceAction("ego", "lead", time_gap=2.0, continuous=True),
        )
    ]
    assert engine.init(_following_scenario(events, lead_x=50.0)) == scn.Status.Ok
    for _ in range(10):
        assert engine.step(0.5) == scn.Status.Ok
        # §7.5.3: a continuous action has no regular ending.
        assert (
            engine.storyboard_element_state("story/act/group/maneuver/keep")
            == scn.ElementState.Running
        )


def test_freespace_gap_is_measured_between_the_boxes() -> None:
    engine = scn.Engine()
    events = [
        _timed_event(
            "keep",
            0.0,
            scn.LongitudinalDistanceAction(
                "ego", "lead", distance=10.0, freespace=True, continuous=True
            ),
        )
    ]
    assert engine.init(_following_scenario(events, lead_x=60.0, boxed=True)) == scn.Status.Ok
    for _ in range(60):
        assert engine.step(0.5) == scn.Status.Ok
    # Both boxes are 4 m long and centred, so a 10 m bumper-to-bumper gap is a
    # 14 m reference-point gap.
    assert _gap(engine) == pytest.approx(14.0, abs=1e-3)


@pytest.mark.parametrize(
    "kwargs",
    [
        {"distance": 10.0, "time_gap": 2.0},  # mutually exclusive
        {},  # neither given
        {"distance": -5.0},  # negative distance
    ],
)
def test_invalid_targets_are_rejected(kwargs) -> None:
    engine = scn.Engine()
    events = [_timed_event("keep", 0.0, scn.LongitudinalDistanceAction("ego", "lead", **kwargs))]
    assert engine.init(_following_scenario(events)) == scn.Status.ValidationError
    assert any(d.severity == scn.Severity.Error for d in engine.diagnostics())


def test_unknown_reference_entity_is_a_semantic_error() -> None:
    engine = scn.Engine()
    events = [_timed_event("keep", 0.0, scn.LongitudinalDistanceAction("ego", "ghost", distance=5.0))]
    assert engine.init(_following_scenario(events)) == scn.Status.SemanticError
