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

"""Cut-in with lateral dynamics (p2-s3).

A miniature of the GS-2 golden scenario (docs/roadmap/golden-scenarios.md),
built on the ASAM OpenSCENARIO XML 1.4.0 lateral actions (§7.4.1.4):

- the ego cruises straight in its lane;
- a faster cutter overtakes in the lane to the ego's left;
- once the cutter is close enough ahead, a LaneChangeAction moves it one lane
  to the right — into the ego's lane — over a sinusoidal 3 s transition;
- the cutter then brakes, and the ego holds a lateral distance to it with a
  continuous LateralDistanceAction, which never ends by itself (§7.5.3).

There is no road network here, so "one lane over" is the engine's flat-world
default lane width (Engine.set_default_lane_width); real lane identity needs a
road backend. Positive lateral values are to the reference entity's left
(§6.3.4, ISO 8855), so the cutter's move to the right is -1 lane.
"""

import scena as scn

LANE_WIDTH = 3.5
EVENT = "story/act/group/maneuver/{}"


def _vehicle(entity_id: str) -> "scn.Entity":
    vehicle = scn.Vehicle(
        category=scn.VehicleCategory.Car,
        bounding_box=scn.BoundingBox(0.0, 0.0, 0.75, 5.0, 2.0, 1.5),
        performance=scn.Performance(60.0, 4.0, 8.0),
    )
    return scn.Entity(entity_id, entity_id, scn.ControlMode.EngineControlled, object=vehicle)


def _timed_event(name: str, at_time: float, action) -> "scn.Event":
    event = scn.Event(name, start_trigger=scn.make_trigger(scn.SimulationTimeCondition(at_time)))
    event.add_action(action)
    return event


def build_scenario() -> "scn.Scenario":
    scenario = scn.Scenario("cut-in")
    scenario.add_entity(_vehicle("ego"))
    scenario.add_entity(_vehicle("cutter"))
    # The cutter starts behind the ego, one lane to its left, and closes.
    scenario.add_init_action(scn.TeleportAction("ego", scn.WorldPosition(0.0, 0.0, 0.0)))
    scenario.add_init_action(
        scn.TeleportAction("cutter", scn.WorldPosition(-40.0, LANE_WIDTH, 0.0))
    )
    scenario.add_init_action(scn.SpeedAction("ego", 25.0))
    scenario.add_init_action(scn.SpeedAction("cutter", 33.0))

    maneuver = scn.Maneuver("maneuver")

    # Starting 40 m back and 8 m/s faster, the cutter is about 16 m ahead of
    # the ego by t = 7 s; that is when it cuts in.
    maneuver.add_event(
        _timed_event(
            "cut-in",
            7.0,
            scn.LaneChangeAction(
                "cutter",
                scn.RelativeTargetLane("cutter", -1),  # one lane to its own right
                scn.TransitionDynamics(
                    shape=scn.DynamicsShape.Sinusoidal,
                    dimension=scn.DynamicsDimension.Time,
                    value=3.0,
                ),
            ),
        )
    )

    # Having cut in, the cutter brakes hard.
    maneuver.add_event(
        _timed_event(
            "cutter-brakes",
            12.0,
            scn.SpeedAction(
                "cutter",
                14.0,
                scn.TransitionDynamics(
                    shape=scn.DynamicsShape.Linear,
                    dimension=scn.DynamicsDimension.Time,
                    value=4.0,
                ),
            ),
        )
    )

    # The ego edges away sideways and keeps that clearance for the rest of the
    # run: continuous, so it never ends on its own.
    maneuver.add_event(
        _timed_event(
            "ego-edges-away",
            14.0,
            scn.LateralDistanceAction(
                "ego",
                "cutter",
                distance=1.2,
                freespace=True,
                continuous=True,
                displacement=scn.LateralDisplacement.RightToReferencedEntity,
                constraints=scn.DynamicConstraints(
                    max_acceleration=1.5, max_deceleration=1.5, max_speed=0.8
                ),
            ),
        )
    )

    group = scn.ManeuverGroup("group")
    group.add_actor("cutter")
    group.add_maneuver(maneuver)
    act = scn.Act("act")
    act.add_group(group)
    story = scn.Story("story")
    story.add_act(act)
    scenario.add_story(story)
    return scenario


def main() -> None:
    engine = scn.Engine()
    # Flat world: this is what "one lane over" means without a road network.
    assert engine.set_default_lane_width(LANE_WIDTH) == scn.Status.Ok
    assert engine.default_lane_width == LANE_WIDTH
    status = engine.init(build_scenario())
    assert status == scn.Status.Ok, status

    print(f"{'t [s]':>6} {'cutter y':>9} {'heading':>9} {'ego y':>7} {'gap y':>7}")
    max_heading = 0.0
    for step in range(300):
        assert engine.step(0.1) == scn.Status.Ok
        cutter = engine.state("cutter")
        max_heading = max(max_heading, abs(cutter.heading))
        if step % 30 == 0:
            ego = engine.state("ego")
            print(
                f"{engine.time:6.1f} {cutter.y:9.2f} {cutter.heading:9.3f} "
                f"{ego.y:7.2f} {ego.y - cutter.y:7.2f}"
            )

    cutter = engine.state("cutter")
    ego = engine.state("ego")
    # The cutter ended one lane to the right of where it started, pointing
    # straight again: a sinusoidal transition ends with zero lateral rate.
    assert abs(cutter.y - 0.0) < 1e-6, cutter.y
    assert abs(cutter.heading) < 1e-9, cutter.heading
    # It really did steer on the way over.
    assert max_heading > 0.05, max_heading
    # The ego is holding its lateral clearance to the right of the cutter, and
    # a continuous action never ends on its own (§7.5.3).
    assert ego.y < cutter.y, (ego.y, cutter.y)
    assert engine.storyboard_element_state(EVENT.format("ego-edges-away")) == (
        scn.ElementState.Running
    )
    assert engine.storyboard_element_state(EVENT.format("cut-in")) == scn.ElementState.Complete
    print(f"\npeak cut-in heading: {max_heading:.3f} rad")
    print("the cutter changed lane and the ego held its lateral clearance")


if __name__ == "__main__":
    main()
