#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Drive the ASAM OpenSCENARIO by-entity conditions from the host side.

Three entities are observed by four by-entity conditions (ASAM OpenSCENARIO
XML 1.4.0 §7.6.5.1), each firing a distinct engine-controlled probe:

* SpeedCondition            - the ego's speed crosses a threshold.
* RelativeSpeedCondition    - the ego is faster than a lead vehicle.
* StandStillCondition       - a parked vehicle has stood still long enough.
* TraveledDistanceCondition - the ego has covered a given path length.

The host reports entity states each step (kinematics the engine differences
into acceleration, an odometer and a standstill timer), and the run asserts
every by-entity path fires by watching its probe.
"""

import scena as scn

PROBES = ("p_speed", "p_relative", "p_standstill", "p_distance")


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
    scenario = scn.Scenario("byentity-conditions")
    for entity_id in ("ego", "lead", "parked"):
        scenario.add_entity(scn.Entity(entity_id, entity_id, scn.ControlMode.HostControlled))
    for probe in PROBES:
        scenario.add_entity(scn.Entity(probe, probe, scn.ControlMode.EngineControlled))

    ego = scn.TriggeringEntities(["ego"])
    _add_event(scenario, "speed", scn.SpeedCondition(ego, 10.0, scn.Rule.GreaterOrEqual), "p_speed")
    _add_event(
        scenario,
        "relative",
        scn.RelativeSpeedCondition(ego, "lead", 2.0, scn.Rule.GreaterOrEqual),
        "p_relative",
    )
    _add_event(
        scenario,
        "standstill",
        scn.StandStillCondition(scn.TriggeringEntities(["parked"]), 1.0),
        "p_standstill",
    )
    _add_event(
        scenario,
        "distance",
        scn.TraveledDistanceCondition(ego, 5.0),
        "p_distance",
    )
    return scenario


def _fired(engine, probe):
    state = engine.state(probe)
    return state is not None and state.speed != 0.0


def main() -> None:
    engine = scn.Engine()
    assert engine.init(build_scenario()) == scn.Status.Ok

    # lead cruises at 8 m/s; parked stays at rest; ego accelerates and moves.
    x = 0.0
    for step in range(20):
        engine.report_state("lead", scn.EntityState(x=100.0, speed=8.0))
        engine.report_state("parked", scn.EntityState(x=-5.0, speed=0.0))
        speed = 4.0 + 0.6 * step  # crosses 10 m/s partway through
        x += speed * 0.5
        engine.report_state("ego", scn.EntityState(x=x, speed=speed))
        assert engine.step(0.5) == scn.Status.Ok

    for probe in PROBES:
        assert _fired(engine, probe), probe

    # A dangling triggering entity would have been rejected at init; the host
    # interface never silently drops a reference.
    print("by-entity conditions: all four probes fired")


if __name__ == "__main__":
    main()
