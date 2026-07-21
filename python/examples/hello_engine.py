#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Build a scenario in code, step it at 100 Hz, and verify the speed action fires.

One engine-controlled entity starts at rest; a SpeedAction triggered by a
SimulationTimeCondition at t = 2.0 s sets its speed to 10 m/s. The host loop
steps 5 simulated seconds at 100 Hz and asserts the speed change happened.
"""

import scena as scn


def main() -> None:
    print(f"Scena {scn.version()}")

    scenario = scn.Scenario("hello-engine")
    scenario.add_entity(scn.Entity("ego", "ego vehicle", scn.ControlMode.EngineControlled))
    scenario.add_entry(
        scn.SimulationTimeCondition(at_time=2.0),
        scn.SpeedAction("ego", target_speed=10.0),
    )

    engine = scn.Engine()
    status = engine.init(scenario)
    assert status == scn.Status.Ok, status

    dt = 0.01  # 100 Hz
    steps = 500  # 5 simulated seconds
    speed_before_trigger = None

    for i in range(steps):
        status = engine.step(dt)
        assert status == scn.Status.Ok, status
        if i == 150:  # t = 1.51 s, before the trigger
            speed_before_trigger = engine.state("ego").speed
        if (i + 1) % 100 == 0:
            state = engine.state("ego")
            print(f"t={engine.time:5.2f}s  x={state.x:7.3f}m  speed={state.speed:5.2f}m/s")

    final = engine.state("ego")
    assert speed_before_trigger == 0.0, f"moved before trigger: {speed_before_trigger}"
    assert final.speed == 10.0, f"speed action did not fire: {final.speed}"
    assert final.x > 0.0, "entity did not move after the speed action"

    engine.close()
    print("speed action fired at t=2.0s: OK")


if __name__ == "__main__":
    main()
