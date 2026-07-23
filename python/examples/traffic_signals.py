# SPDX-FileCopyrightText: 2026 Robomous
# SPDX-License-Identifier: Apache-2.0
"""A signalized intersection: controllers, phases, and the signal conditions.

This is the shape of the §11.12 worked example ("Signalized T-intersection"),
and the GS-11 golden scenario built on it:

  * two chained TrafficSignalControllers run the straight and turning
    approaches, the second offset by a delay (§6.11.3) so the two cycles
    interlock — a progressive signal system;
  * the ego reaches the stop line and a TrafficSignalControllerAction jumps
    the cycle to its clearing phase;
  * a rising-edge TrafficSignalControllerCondition releases the cross vehicle
    once the straight signal turns green;
  * a TrafficSignalStateAction simulates a bulb failure, forcing one signal
    into a state no phase produces — it stands until the cycle's next phase
    transition reclaims the signal;
  * a TrafficSignalCondition notices the failure.

Run with:  python python/examples/traffic_signals.py
"""

import scena as scn


def vehicle(entity_id):
    entity = scn.Entity(entity_id, entity_id, scn.ControlMode.EngineControlled)
    body = scn.Vehicle()
    body.bounding_box = scn.BoundingBox(0.0, 0.0, 0.75, 4.6, 1.9, 1.5)
    body.performance = scn.Performance(45.0, 3.5, 7.0)
    entity.object = body
    return entity


def phase(name, duration, signal, state):
    return scn.Phase(name, duration, [scn.TrafficSignalState(signal, state)])


def timed_event(name, at_time, action):
    event = scn.Event(name, start_trigger=scn.make_trigger(scn.SimulationTimeCondition(at_time)))
    event.add_action(action)
    return event


def gated_event(name, condition, action, edge=scn.ConditionEdge.Rising):
    event = scn.Event(name, start_trigger=scn.make_trigger(condition, edge))
    event.add_action(action)
    return event


def build_scenario():
    scenario = scn.Scenario("signalized-intersection")
    scenario.add_entity(vehicle("ego"))
    scenario.add_entity(vehicle("cross"))

    # The straight approach runs stop → caution → go; the turning approach runs
    # the complementary cycle, 12.5 s behind it (§6.11.3). Their durations sum
    # to the same 50 s, which is what makes a delay meaningful.
    scenario.add_traffic_signal_controller(
        scn.TrafficSignalController(
            "straight",
            phases=[
                phase("stop", 30.0, "signal-straight", "red"),
                phase("caution", 2.5, "signal-straight", "red;amber"),
                phase("go", 17.5, "signal-straight", "green"),
            ],
        )
    )
    scenario.add_traffic_signal_controller(
        scn.TrafficSignalController(
            "turning",
            delay=12.5,
            reference="straight",
            phases=[
                phase("go", 17.5, "signal-turning", "green"),
                phase("amber", 2.5, "signal-turning", "amber"),
                phase("stop", 30.0, "signal-turning", "red"),
            ],
        )
    )

    scenario.add_init_action(scn.TeleportAction("ego", scn.WorldPosition(0.0, 0.0, 0.0)))
    scenario.add_init_action(scn.TeleportAction("cross", scn.WorldPosition(130.0, -60.0, 0.0)))
    scenario.add_init_action(scn.SpeedAction("ego", 13.5))

    maneuver = scn.Maneuver("maneuver")
    # t1: the ego reaches the stop line and requests the phase change.
    maneuver.add_event(
        gated_event(
            "at-stop-line",
            scn.TraveledDistanceCondition(
                scn.TriggeringEntities(["ego"], scn.TriggeringEntitiesRule.Any), 120.0
            ),
            scn.TrafficSignalControllerAction("straight", "caution"),
            edge=scn.ConditionEdge.NoEdge,
        )
    )
    # t2: the straight signal is green, so the cross traffic moves off.
    maneuver.add_event(
        gated_event(
            "cross-releases",
            scn.TrafficSignalControllerCondition("straight", "go"),
            scn.SpeedAction(
                "cross",
                11.0,
                scn.TransitionDynamics(
                    scn.DynamicsShape.Sinusoidal, scn.DynamicsDimension.Time, 6.0
                ),
            ),
        )
    )
    # t3: a bulb failure forces a signal into a state no phase produces.
    maneuver.add_event(
        timed_event("bulb-failure", 14.0, scn.TrafficSignalStateAction("signal-turning", "red;green"))
    )
    # …which a TrafficSignalCondition then notices.
    maneuver.add_event(
        gated_event(
            "failure-noticed",
            scn.TrafficSignalCondition("signal-turning", "red;green"),
            scn.SpeedAction(
                "ego", 6.0, scn.TransitionDynamics(scn.DynamicsShape.Linear,
                                                   scn.DynamicsDimension.Time, 4.0)
            ),
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


def main():
    engine = scn.Engine()
    status = engine.init(build_scenario())
    if status != scn.Status.Ok:
        raise SystemExit(f"init failed: {status}")

    header = (
        f"{'t':>5}  {'straight':>10} {'signal':>10}  "
        f"{'turning':>10} {'signal':>10}  {'ego v':>6} {'cross v':>7}"
    )
    print(header)
    print("-" * len(header))
    for step in range(1, 176):  # 35 s at 5 Hz
        if engine.step(0.2) != scn.Status.Ok:
            raise SystemExit("step failed")
        if step % 10:
            continue
        # A controller with a delay still running has no phase at all, and a
        # signal nothing has written yet has no state — both print as "-".
        straight_phase = engine.traffic_signal_controller_phase("straight") or "-"
        straight_state = engine.traffic_signal_state("signal-straight") or "-"
        turning_phase = engine.traffic_signal_controller_phase("turning") or "-"
        turning_state = engine.traffic_signal_state("signal-turning") or "-"
        print(
            f"{engine.time:5.1f}  {straight_phase:>10} {straight_state:>10}  "
            f"{turning_phase:>10} {turning_state:>10}  "
            f"{engine.state('ego').speed:6.2f} {engine.state('cross').speed:7.2f}"
        )

    print()
    print("The forced 'red;green' state held until the turning cycle's next")
    print("phase transition reclaimed the signal — actions win over the clock,")
    print("but only until the clock moves on.")


if __name__ == "__main__":
    main()
