# SPDX-License-Identifier: MIT
"""Pytest suite covering the hello_engine flow through the Python bindings."""

import kinema as knm


def build_scenario() -> "knm.Scenario":
    scenario = knm.Scenario("hello-engine")
    scenario.add_entity(knm.Entity("ego", "ego vehicle", knm.ControlMode.EngineControlled))
    scenario.add_entry(
        knm.SimulationTimeCondition(at_time=2.0),
        knm.SpeedAction("ego", target_speed=10.0),
    )
    return scenario


def test_version() -> None:
    assert knm.version() == knm.__version__
    major, minor, patch = knm.version().split(".")
    assert all(part.isdigit() for part in (major, minor, patch))


def test_hello_engine_flow() -> None:
    engine = knm.Engine()
    assert engine.init(build_scenario()) == knm.Status.Ok

    dt = 0.01  # 100 Hz
    speed_before_trigger = None
    for i in range(500):  # 5 simulated seconds
        assert engine.step(dt) == knm.Status.Ok
        if i == 150:  # t = 1.51 s, before the trigger
            speed_before_trigger = engine.state("ego").speed

    final = engine.state("ego")
    assert speed_before_trigger == 0.0
    assert final.speed == 10.0
    assert final.x > 0.0
    assert engine.close() == knm.Status.Ok


def test_error_codes() -> None:
    engine = knm.Engine()
    assert engine.step(0.01) == knm.Status.NotInitialized
    assert engine.init(build_scenario()) == knm.Status.Ok
    assert engine.init(build_scenario()) == knm.Status.AlreadyInitialized
    assert engine.state("missing") is None
    assert engine.report_state("ego", knm.EntityState()) == knm.Status.InvalidControlMode


def test_host_controlled_round_trip() -> None:
    scenario = knm.Scenario("host-controlled")
    scenario.add_entity(knm.Entity("npc", "host vehicle", knm.ControlMode.HostControlled))

    engine = knm.Engine()
    assert engine.init(scenario) == knm.Status.Ok

    reported = knm.EntityState(x=1.0, y=2.0, z=3.0, heading=0.5, speed=7.0)
    assert engine.report_state("npc", reported) == knm.Status.Ok
    assert engine.step(0.01) == knm.Status.Ok

    state = engine.state("npc")
    assert (state.x, state.y, state.z) == (1.0, 2.0, 3.0)
    assert state.heading == 0.5
    assert state.speed == 7.0
