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

"""The lateral actions through the Python bindings (p2-s3): LaneChangeAction,
LaneOffsetAction and LateralDistanceAction over the flat-world lateral model
(§7.4.1.4, ADR-0016), plus the default lane width knob."""

import pytest

import scena as scn

LANE_WIDTH = 3.5
EVENT = "story/act/group/maneuver/lateral"


def _lateral_scenario(events, *, other_y=LANE_WIDTH, speed=10.0, boxed=False):
    """An 'ego' at the origin and a 'lead' 20 m ahead at `other_y`, both
    engine-controlled and cruising at `speed`, plus one maneuver of `events`."""
    scenario = scn.Scenario("lateral")
    for entity_id in ("ego", "lead"):
        if boxed:
            vehicle = scn.Vehicle(
                category=scn.VehicleCategory.Car,
                bounding_box=scn.BoundingBox(0.0, 0.0, 0.75, 5.0, 2.0, 1.5),
            )
            scenario.add_entity(
                scn.Entity(entity_id, entity_id, scn.ControlMode.EngineControlled, object=vehicle)
            )
        else:
            scenario.add_entity(scn.Entity(entity_id, entity_id, scn.ControlMode.EngineControlled))
    scenario.add_init_action(scn.TeleportAction("ego", scn.WorldPosition(0.0, 0.0, 0.0)))
    scenario.add_init_action(scn.TeleportAction("lead", scn.WorldPosition(20.0, other_y, 0.0)))
    scenario.add_init_action(scn.SpeedAction("ego", speed))
    scenario.add_init_action(scn.SpeedAction("lead", speed))

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


def _run(engine, count, dt=0.1):
    for _ in range(count):
        assert engine.step(dt) == scn.Status.Ok


# --- IR round trips --------------------------------------------------------


def test_lane_change_action_round_trip() -> None:
    dynamics = scn.TransitionDynamics(
        shape=scn.DynamicsShape.Sinusoidal, dimension=scn.DynamicsDimension.Time, value=4.0
    )
    relative = scn.LaneChangeAction(
        "ego", scn.RelativeTargetLane("lead", -1), dynamics, target_lane_offset=0.4
    )
    assert relative.kind == "LaneChangeAction"
    assert relative.entity_id == "ego"
    assert relative.is_relative is True
    assert relative.relative_target.entity_ref == "lead"
    assert relative.relative_target.value == -1
    assert relative.absolute_target is None
    assert relative.dynamics.shape == scn.DynamicsShape.Sinusoidal
    assert relative.target_lane_offset == pytest.approx(0.4)

    absolute = scn.LaneChangeAction("ego", scn.AbsoluteTargetLane("-2"), dynamics)
    assert absolute.is_relative is False
    assert absolute.absolute_target.value == "-2"
    assert absolute.relative_target is None
    # "Missing value is interpreted as 0" (Class `LaneChangeAction`).
    assert absolute.target_lane_offset == pytest.approx(0.0)


def test_lane_offset_action_round_trip() -> None:
    absolute = scn.LaneOffsetAction(
        "ego",
        scn.AbsoluteTargetLaneOffset(1.5),
        continuous=True,
        shape=scn.DynamicsShape.Cubic,
        max_lateral_acc=2.0,
    )
    assert absolute.kind == "LaneOffsetAction"
    assert absolute.is_relative is False
    assert absolute.absolute_target.value == pytest.approx(1.5)
    assert absolute.continuous is True
    assert absolute.shape == scn.DynamicsShape.Cubic
    assert absolute.max_lateral_acc == pytest.approx(2.0)

    relative = scn.LaneOffsetAction(
        "ego", scn.RelativeTargetLaneOffset("lead", -0.75), shape=scn.DynamicsShape.Linear
    )
    assert relative.is_relative is True
    assert relative.relative_target.entity_ref == "lead"
    assert relative.relative_target.value == pytest.approx(-0.75)
    # "Missing value is interpreted as 'inf'" — None on the Python side.
    assert relative.max_lateral_acc is None
    assert relative.continuous is False


def test_lateral_distance_action_round_trip() -> None:
    action = scn.LateralDistanceAction(
        "ego",
        "lead",
        distance=2.0,
        freespace=True,
        continuous=True,
        displacement=scn.LateralDisplacement.LeftToReferencedEntity,
        constraints=scn.DynamicConstraints(max_speed=0.5),
    )
    assert action.kind == "LateralDistanceAction"
    assert action.entity_ref == "lead"
    assert action.distance == pytest.approx(2.0)
    assert action.freespace is True
    assert action.continuous is True
    # Omitted defaults: entity coordinate system, `any` displacement.
    assert action.coordinate_system == scn.CoordinateSystem.Entity
    assert action.displacement == scn.LateralDisplacement.LeftToReferencedEntity
    assert action.constraints.max_speed == pytest.approx(0.5)
    assert scn.LateralDistanceAction("ego", "lead").displacement == scn.LateralDisplacement.Any


# --- Behavior --------------------------------------------------------------


def test_cut_in_covers_one_lane_width() -> None:
    dynamics = scn.TransitionDynamics(
        shape=scn.DynamicsShape.Sinusoidal, dimension=scn.DynamicsDimension.Time, value=4.0
    )
    engine = scn.Engine()
    scenario = _lateral_scenario(
        [
            _timed_event(
                "lateral",
                0.0,
                scn.LaneChangeAction("ego", scn.RelativeTargetLane("ego", -1), dynamics),
            )
        ]
    )
    assert engine.init(scenario) == scn.Status.Ok
    _run(engine, 50)
    assert engine.storyboard_element_state(EVENT) == scn.ElementState.Complete
    # A negative lane count goes right (§7.4.1.4, ISO 8855).
    assert engine.state("ego").y == pytest.approx(-LANE_WIDTH, abs=1e-9)
    # A sinusoidal transition ends with zero lateral rate, so the heading is
    # back on the axis.
    assert engine.state("ego").heading == pytest.approx(0.0, abs=1e-12)


@pytest.mark.parametrize(
    "shape",
    [scn.DynamicsShape.Linear, scn.DynamicsShape.Cubic, scn.DynamicsShape.Sinusoidal],
)
def test_lane_offset_reaches_its_target_for_every_smooth_shape(shape) -> None:
    engine = scn.Engine()
    scenario = _lateral_scenario(
        [
            _timed_event(
                "lateral",
                0.0,
                scn.LaneOffsetAction(
                    "ego", scn.AbsoluteTargetLaneOffset(2.0), shape=shape, max_lateral_acc=1.0
                ),
            )
        ]
    )
    assert engine.init(scenario) == scn.Status.Ok
    _run(engine, 200)
    assert engine.storyboard_element_state(EVENT) == scn.ElementState.Complete
    assert engine.state("ego").y == pytest.approx(2.0, abs=1e-9)
    assert engine.state("ego").heading == pytest.approx(0.0, abs=1e-12)


def test_a_step_shaped_offset_is_instantaneous() -> None:
    engine = scn.Engine()
    scenario = _lateral_scenario(
        [
            _timed_event(
                "lateral",
                0.0,
                scn.LaneOffsetAction(
                    "ego", scn.AbsoluteTargetLaneOffset(-1.0), shape=scn.DynamicsShape.Step
                ),
            )
        ]
    )
    assert engine.init(scenario) == scn.Status.Ok
    _run(engine, 1)
    assert engine.storyboard_element_state(EVENT) == scn.ElementState.Complete
    assert engine.state("ego").y == pytest.approx(-1.0, abs=1e-12)


def test_a_continuous_offset_event_never_ends() -> None:
    # §7.5.3 / Annex A Table 10: "no regular ending".
    engine = scn.Engine()
    scenario = _lateral_scenario(
        [
            _timed_event(
                "lateral",
                0.0,
                scn.LaneOffsetAction(
                    "ego",
                    scn.RelativeTargetLaneOffset("lead", 0.0),
                    continuous=True,
                    shape=scn.DynamicsShape.Linear,
                    max_lateral_acc=1.0,
                ),
            )
        ]
    )
    assert engine.init(scenario) == scn.Status.Ok
    _run(engine, 200)
    assert engine.state("ego").y == pytest.approx(LANE_WIDTH, abs=1e-9)
    assert engine.storyboard_element_state(EVENT) == scn.ElementState.Running


def test_lateral_keeping_converges_and_ends() -> None:
    engine = scn.Engine()
    scenario = _lateral_scenario(
        [
            _timed_event(
                "lateral",
                0.0,
                scn.LateralDistanceAction(
                    "ego",
                    "lead",
                    distance=1.0,
                    displacement=scn.LateralDisplacement.RightToReferencedEntity,
                    constraints=scn.DynamicConstraints(
                        max_acceleration=1.0, max_deceleration=1.0, max_speed=1.0
                    ),
                ),
            )
        ]
    )
    assert engine.init(scenario) == scn.Status.Ok
    _run(engine, 200)
    assert engine.storyboard_element_state(EVENT) == scn.ElementState.Complete
    assert engine.state("ego").y == pytest.approx(LANE_WIDTH - 1.0, abs=1e-6)


def test_freespace_lateral_keeping_measures_flank_to_flank() -> None:
    engine = scn.Engine()
    scenario = _lateral_scenario(
        [
            _timed_event(
                "lateral",
                0.0,
                scn.LateralDistanceAction("ego", "lead", distance=1.0, freespace=True),
            )
        ],
        other_y=6.0,
        boxed=True,
    )
    assert engine.init(scenario) == scn.Status.Ok
    _run(engine, 60, 0.5)
    assert engine.storyboard_element_state(EVENT) == scn.ElementState.Complete
    # Both boxes are 2 m wide, so a 1 m freespace gap is 3 m between reference
    # points.
    assert engine.state("ego").y == pytest.approx(3.0, abs=1e-6)


# --- The flat-world lane width knob ---------------------------------------


def test_default_lane_width_round_trips_and_is_validated() -> None:
    engine = scn.Engine()
    assert engine.default_lane_width == pytest.approx(3.5)
    assert engine.set_default_lane_width(0.0) == scn.Status.InvalidArgument
    assert engine.set_default_lane_width(-1.0) == scn.Status.InvalidArgument
    assert engine.set_default_lane_width(float("nan")) == scn.Status.InvalidArgument
    assert engine.default_lane_width == pytest.approx(3.5)
    assert engine.set_default_lane_width(2.5) == scn.Status.Ok
    assert engine.default_lane_width == pytest.approx(2.5)


def test_the_configured_lane_width_drives_the_lane_change() -> None:
    dynamics = scn.TransitionDynamics(
        shape=scn.DynamicsShape.Cubic, dimension=scn.DynamicsDimension.Time, value=4.0
    )
    engine = scn.Engine()
    assert engine.set_default_lane_width(2.5) == scn.Status.Ok
    scenario = _lateral_scenario(
        [
            _timed_event(
                "lateral",
                0.0,
                scn.LaneChangeAction("ego", scn.RelativeTargetLane("ego", 2), dynamics),
            )
        ]
    )
    assert engine.init(scenario) == scn.Status.Ok
    # The setting survives init, like the time-of-day anchor.
    assert engine.default_lane_width == pytest.approx(2.5)
    _run(engine, 50)
    assert engine.state("ego").y == pytest.approx(5.0, abs=1e-9)
    # close() forgets it.
    assert engine.close() == scn.Status.Ok
    assert engine.default_lane_width == pytest.approx(3.5)


def test_an_absolute_lane_target_needs_a_road_network() -> None:
    dynamics = scn.TransitionDynamics(
        shape=scn.DynamicsShape.Sinusoidal, dimension=scn.DynamicsDimension.Time, value=4.0
    )
    engine = scn.Engine()
    scenario = _lateral_scenario(
        [
            _timed_event(
                "lateral", 0.0, scn.LaneChangeAction("ego", scn.AbsoluteTargetLane("-2"), dynamics)
            )
        ]
    )
    assert engine.init(scenario) == scn.Status.Ok
    _run(engine, 10)
    assert engine.storyboard_element_state(EVENT) == scn.ElementState.Complete
    assert engine.state("ego").y == pytest.approx(0.0)
    assert any(
        diagnostic.code == scn.Status.UnsupportedFeature for diagnostic in engine.diagnostics()
    )


def test_a_zero_max_lateral_acc_is_rejected_at_init() -> None:
    engine = scn.Engine()
    scenario = _lateral_scenario(
        [
            _timed_event(
                "lateral",
                0.0,
                scn.LaneOffsetAction(
                    "ego",
                    scn.AbsoluteTargetLaneOffset(1.0),
                    shape=scn.DynamicsShape.Cubic,
                    max_lateral_acc=0.0,
                ),
            )
        ]
    )
    assert engine.init(scenario) == scn.Status.ValidationError
