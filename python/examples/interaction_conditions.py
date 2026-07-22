#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Drive the ASAM OpenSCENARIO interaction conditions from the host side.

An ego vehicle closes on a stationary lead vehicle. Three interaction
conditions (ASAM OpenSCENARIO XML 1.4.0 §6.4), each firing a distinct
engine-controlled probe, watch the encounter:

* DistanceCondition        - the ego reaches within 5 m of a fixed point.
* TimeToCollisionCondition - the predicted time to collision drops below 5 s.
* CollisionCondition       - the ego's bounding box touches the lead's.

Both vehicles carry a bounding box (a minimal forward-pull from the full
entity model), so the collision and freespace metrics have geometry to work
with. The host reports entity states each step and the run asserts every
probe fires.
"""

import scena as scn

PROBES = ("p_distance", "p_ttc", "p_collision")


def _add_event(scenario, label, condition, probe):
    event = scn.Event(label, start_trigger=scn.make_trigger(condition))
    event.add_action(scn.SpeedAction(probe, target_speed=99.0))
    maneuver = scn.Maneuver(f"{label}-maneuver")
    maneuver.add_event(event)
    group = scn.ManeuverGroup(f"{label}-group")
    group.add_maneuver(maneuver)
    act = scn.Act(f"{label}-act")
    act.add_group(group)
    story = scn.Story(f"{label}-story")
    story.add_act(act)
    scenario.add_story(story)


def build_scenario() -> "scn.Scenario":
    scenario = scn.Scenario("interaction-conditions")
    # A 4 m x 2 m box for each vehicle (length x width).
    box = scn.BoundingBox(length=4.0, width=2.0)
    for entity_id in ("ego", "lead"):
        scenario.add_entity(
            scn.Entity(
                entity_id,
                entity_id,
                scn.ControlMode.HostControlled,
                object=scn.MiscObject(bounding_box=box),
            )
        )
    for probe in PROBES:
        scenario.add_entity(scn.Entity(probe, probe, scn.ControlMode.EngineControlled))

    ego = scn.TriggeringEntities(["ego"])
    _add_event(
        scenario,
        "distance",
        scn.DistanceCondition(ego, scn.WorldPosition(20.0, 0.0, 0.0), 5.0, False, scn.Rule.LessOrEqual),
        "p_distance",
    )
    _add_event(
        scenario,
        "ttc",
        scn.TimeToCollisionCondition(ego, 5.0, False, scn.Rule.LessOrEqual, entity_ref="lead"),
        "p_ttc",
    )
    _add_event(scenario, "collision", scn.CollisionCondition(ego, "lead"), "p_collision")
    return scenario


def _fired(engine, probe):
    state = engine.state(probe)
    return state is not None and state.speed != 0.0


def main() -> None:
    engine = scn.Engine()
    assert engine.init(build_scenario()) == scn.Status.Ok

    # ego closes on the lead parked at x = 20 m, at 5 m/s.
    for step in range(30):
        x = min(0.7 * step, 18.0)
        engine.report_state("ego", scn.EntityState(x=x, speed=5.0))
        engine.report_state("lead", scn.EntityState(x=20.0, speed=0.0))
        assert engine.step(0.5) == scn.Status.Ok

    for probe in PROBES:
        assert _fired(engine, probe), probe

    print("interaction conditions: distance, time-to-collision and collision all fired")


if __name__ == "__main__":
    main()
