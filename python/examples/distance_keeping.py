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

"""Traffic-jam approach with distance keeping (p5-s5).

A miniature of the GS-4 golden scenario (docs/roadmap/golden-scenarios.md),
built on ASAM OpenSCENARIO XML 1.4.0 §LongitudinalDistanceAction:

- a lead vehicle cruises at 25 m/s and then brakes to a crawl (the jam);
- the ego brakes when the freespace time to collision drops below 6 s;
- a continuous LongitudinalDistanceAction then holds an 8 m bumper-to-bumper
  gap — with `continuous=True` it never ends by itself (§7.5.3), so the ego
  keeps tracking the lead when the jam dissolves;
- at t = 40 s the lead accelerates away and the ego follows it back up.

Both vehicles carry a bounding box (freespace gaps need geometry) and a
Performance envelope, which the distance controller clamps to.
"""

import scena as scn

GAP = 8.0  # target bumper-to-bumper gap [m]
CAR_LENGTH = 5.0


def _vehicle(entity_id: str, max_deceleration: float) -> "scn.Entity":
    vehicle = scn.Vehicle(
        category=scn.VehicleCategory.Car,
        bounding_box=scn.BoundingBox(0.0, 0.0, 0.75, CAR_LENGTH, 2.0, 1.5),
        performance=scn.Performance(40.0, 3.0, max_deceleration),
    )
    return scn.Entity(entity_id, entity_id, scn.ControlMode.EngineControlled, object=vehicle)


def _timed_event(name: str, at_time: float, action) -> "scn.Event":
    event = scn.Event(name, start_trigger=scn.make_trigger(scn.SimulationTimeCondition(at_time)))
    event.add_action(action)
    return event


def build_scenario() -> "scn.Scenario":
    scenario = scn.Scenario("distance-keeping")
    scenario.add_entity(_vehicle("lead", 6.0))
    scenario.add_entity(_vehicle("ego", 8.0))
    scenario.add_init_action(scn.TeleportAction("lead", scn.WorldPosition(150.0, 0.0, 0.0)))
    scenario.add_init_action(scn.TeleportAction("ego", scn.WorldPosition(0.0, 0.0, 0.0)))
    scenario.add_init_action(scn.SpeedAction("lead", 25.0))
    scenario.add_init_action(scn.SpeedAction("ego", 25.0))

    linear = scn.TransitionDynamics(
        shape=scn.DynamicsShape.Linear, dimension=scn.DynamicsDimension.Time, value=8.0
    )
    maneuver = scn.Maneuver("maneuver")
    maneuver.add_event(_timed_event("jam-forms", 2.0, scn.SpeedAction("lead", 0.5, linear)))

    # The ego brakes when the freespace time to collision drops below 6 s.
    brake = scn.Event(
        "ego-brakes",
        start_trigger=scn.make_trigger(
            scn.TimeToCollisionCondition(
                scn.TriggeringEntities(["ego"]),
                6.0,
                True,
                scn.Rule.LessThan,
                entity_ref="lead",
                coordinate_system=scn.CoordinateSystem.Entity,
                relative_distance_type=scn.RelativeDistanceType.Longitudinal,
            )
        ),
    )
    brake.add_action(
        scn.SpeedAction(
            "ego",
            2.0,
            scn.TransitionDynamics(
                shape=scn.DynamicsShape.Linear, dimension=scn.DynamicsDimension.Time, value=5.0
            ),
        )
    )
    maneuver.add_event(brake)

    # Once the braking event ends, distance keeping takes over.
    keep = scn.Event(
        "ego-keeps-gap",
        start_trigger=scn.make_trigger(
            scn.StoryboardElementStateCondition(
                scn.StoryboardElementType.Event,
                "ego-brakes",
                scn.StoryboardElementState.CompleteState,
            )
        ),
    )
    keep.add_action(
        scn.LongitudinalDistanceAction(
            "ego", "lead", distance=GAP, freespace=True, continuous=True
        )
    )
    maneuver.add_event(keep)
    maneuver.add_event(_timed_event("jam-dissolves", 40.0, scn.SpeedAction("lead", 20.0, linear)))

    group = scn.ManeuverGroup("group")
    group.add_actor("ego")
    group.add_maneuver(maneuver)
    act = scn.Act("act")
    act.add_group(group)
    story = scn.Story("story")
    story.add_act(act)
    scenario.add_story(story)
    return scenario


def main() -> None:
    engine = scn.Engine()
    status = engine.init(build_scenario())
    assert status == scn.Status.Ok, status

    def freespace_gap() -> float:
        return engine.state("lead").x - engine.state("ego").x - CAR_LENGTH

    minimum_gap = freespace_gap()
    print(f"{'t [s]':>6} {'ego v':>8} {'lead v':>8} {'gap [m]':>9}")
    for step in range(600):
        assert engine.step(0.1) == scn.Status.Ok
        minimum_gap = min(minimum_gap, freespace_gap())
        assert freespace_gap() > 0.0, "the vehicles collided"
        if step % 60 == 0:
            print(
                f"{engine.time:6.1f} {engine.state('ego').speed:8.2f} "
                f"{engine.state('lead').speed:8.2f} {freespace_gap():9.2f}"
            )

    print(f"\nminimum freespace gap: {minimum_gap:.2f} m")
    # The continuous action never ends on its own (§7.5.3).
    assert (
        engine.storyboard_element_state("story/act/group/maneuver/ego-keeps-gap")
        == scn.ElementState.Running
    )
    # After the jam dissolves the ego is back up to speed, holding the gap.
    assert abs(engine.state("ego").speed - engine.state("lead").speed) < 0.5
    assert abs(freespace_gap() - GAP) < 0.5
    print("distance keeping held the gap through the jam and its dissolution")


if __name__ == "__main__":
    main()
