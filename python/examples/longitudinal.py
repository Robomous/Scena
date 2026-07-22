#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Drive an entity's speed with transition dynamics and a speed profile.

One engine-controlled Vehicle carries a Performance envelope. Three events
demonstrate the p2-s2 longitudinal model (ASAM OpenSCENARIO XML 1.4.0
§SpeedAction / §SpeedProfileAction):

- a Cubic SpeedAction ramps 0 -> 20 m/s over 4 s (smooth, zero acceleration at
  the endpoints);
- a Linear SpeedAction then demands 20 -> 40 m/s in 1 s, which exceeds the
  vehicle's 3 m/s^2 limit, so the default controller stretches it to stay
  within the Performance envelope;
- a SpeedProfileAction winds back down through 10 then 0 m/s.

The host steps the engine and asserts the speed reaches each target.
"""

import scena as scn


def build_scenario() -> "scn.Scenario":
    scenario = scn.Scenario("longitudinal")

    vehicle = scn.Vehicle()
    vehicle.performance = scn.Performance(
        max_speed=45.0, max_acceleration=8.0, max_deceleration=5.0
    )
    scenario.add_entity(
        scn.Entity("ego", "ego vehicle", scn.ControlMode.EngineControlled, object=vehicle)
    )

    def timed_event(name: str, at_time: float, action) -> "scn.Event":
        event = scn.Event(
            name, start_trigger=scn.make_trigger(scn.SimulationTimeCondition(at_time=at_time))
        )
        event.add_action(action)
        return event

    cubic = scn.TransitionDynamics(
        shape=scn.DynamicsShape.Cubic, dimension=scn.DynamicsDimension.Time, value=4.0
    )
    steep = scn.TransitionDynamics(
        shape=scn.DynamicsShape.Linear, dimension=scn.DynamicsDimension.Time, value=1.0
    )

    maneuver = scn.Maneuver("maneuver")
    maneuver.add_event(timed_event("ramp-up", 0.0, scn.SpeedAction("ego", 20.0, cubic)))
    maneuver.add_event(timed_event("push", 6.0, scn.SpeedAction("ego", 40.0, steep)))
    maneuver.add_event(
        timed_event(
            "wind-down",
            20.0,
            scn.SpeedProfileAction(
                "ego", [scn.SpeedProfileEntry(10.0, 4.0), scn.SpeedProfileEntry(0.0, 4.0)]
            ),
        )
    )

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
    assert engine.init(build_scenario()) == scn.Status.Ok

    # Cubic ramp completes by t = 4 s.
    for _ in range(120):  # 6 s
        assert engine.step(0.05) == scn.Status.Ok
    assert engine.state("ego").speed == 20.0, engine.state("ego").speed
    print(f"cubic ramp reached {engine.state('ego').speed:.1f} m/s")

    # The steep 20 -> 40 in 1 s demand needs 20 m/s^2; the 8 m/s^2 envelope
    # stretches it to 2.5 s. Run well past that.
    for _ in range(200):  # 10 s -> t = 16 s
        assert engine.step(0.05) == scn.Status.Ok
    assert engine.state("ego").speed == 40.0, engine.state("ego").speed
    print(f"clamped push reached {engine.state('ego').speed:.1f} m/s")

    # Wind-down profile at t = 20 s: 40 -> 10 (4 s) -> 0 (4 s), done by t = 28 s.
    for _ in range(260):  # 13 s -> t = 29 s
        assert engine.step(0.05) == scn.Status.Ok
    assert engine.state("ego").speed == 0.0, engine.state("ego").speed
    print(f"profile wound down to {engine.state('ego').speed:.1f} m/s")

    # The run applied only actions the engine implements, so no warnings.
    assert engine.diagnostics() == [], engine.diagnostics()
    engine.close()
    print("longitudinal dynamics: cubic ramp, performance-clamped push, speed profile: OK")


if __name__ == "__main__":
    main()
