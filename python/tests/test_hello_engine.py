# SPDX-License-Identifier: MIT
"""Pytest suite covering the hello_engine flow through the Python bindings."""

import scena as scn


def build_scenario() -> "scn.Scenario":
    scenario = scn.Scenario("hello-engine")
    scenario.add_entity(scn.Entity("ego", "ego vehicle", scn.ControlMode.EngineControlled))
    scenario.add_entry(
        scn.SimulationTimeCondition(at_time=2.0),
        scn.SpeedAction("ego", target_speed=10.0),
    )
    return scenario


def test_version() -> None:
    assert scn.version() == scn.__version__
    major, minor, patch = scn.version().split(".")
    assert all(part.isdigit() for part in (major, minor, patch))


def test_hello_engine_flow() -> None:
    engine = scn.Engine()
    assert engine.init(build_scenario()) == scn.Status.Ok

    dt = 0.01  # 100 Hz
    speed_before_trigger = None
    for i in range(500):  # 5 simulated seconds
        assert engine.step(dt) == scn.Status.Ok
        if i == 150:  # t = 1.51 s, before the trigger
            speed_before_trigger = engine.state("ego").speed

    final = engine.state("ego")
    assert speed_before_trigger == 0.0
    assert final.speed == 10.0
    assert final.x > 0.0
    assert engine.close() == scn.Status.Ok


def test_error_codes() -> None:
    engine = scn.Engine()
    assert engine.step(0.01) == scn.Status.NotInitialized
    assert engine.init(build_scenario()) == scn.Status.Ok
    assert engine.init(build_scenario()) == scn.Status.AlreadyInitialized
    assert engine.state("missing") is None
    assert engine.report_state("ego", scn.EntityState()) == scn.Status.InvalidControlMode


def test_host_controlled_round_trip() -> None:
    scenario = scn.Scenario("host-controlled")
    scenario.add_entity(scn.Entity("npc", "host vehicle", scn.ControlMode.HostControlled))

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok

    reported = scn.EntityState(x=1.0, y=2.0, z=3.0, heading=0.5, speed=7.0)
    assert engine.report_state("npc", reported) == scn.Status.Ok
    assert engine.step(0.01) == scn.Status.Ok

    state = engine.state("npc")
    assert (state.x, state.y, state.z) == (1.0, 2.0, 3.0)
    assert state.heading == 0.5
    assert state.speed == 7.0
