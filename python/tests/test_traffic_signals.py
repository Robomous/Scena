# SPDX-FileCopyrightText: 2026 Robomous
# SPDX-License-Identifier: Apache-2.0
"""Traffic signals through the Python bindings (p5-s6, ASAM OpenSCENARIO XML
1.4.0 §6.11): the controller cycle clock, the chained-controller delay of
§6.11.3, the two infrastructure actions and the two signal conditions."""

import pytest

import scena as scn


def _controller(name, signal, phases, *, delay=None, reference=None):
    """A controller driving one signal, phase i writing state `<phase name>`."""
    return scn.TrafficSignalController(
        name,
        delay=delay,
        reference=reference,
        phases=[
            scn.Phase(phase_name, duration, [scn.TrafficSignalState(signal, phase_name)])
            for phase_name, duration in phases
        ],
    )


def _scenario(events, controllers=()):
    scenario = scn.Scenario("traffic-signals")
    scenario.add_entity(scn.Entity("ego", "ego", scn.ControlMode.EngineControlled))
    for controller in controllers:
        scenario.add_traffic_signal_controller(controller)

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


def _gated_event(name, condition, action, *, edge=scn.ConditionEdge.NoEdge):
    event = scn.Event(name, start_trigger=scn.make_trigger(condition, edge))
    event.add_action(action)
    return event


# --- The cycle clock (§6.11.4) ---------------------------------------------


def test_first_phase_applies_at_storyboard_start() -> None:
    scenario = _scenario([], [_controller("group1", "s1", [("stop", 20.0), ("go", 15.0)])])
    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    # §6.11.4: "The first Phase starts with the execution of the storyboard."
    assert engine.traffic_signal_state("s1") == "stop"
    assert engine.traffic_signal_controller_phase("group1") == "stop"
    # Nothing has written this id, and no such controller exists.
    assert engine.traffic_signal_state("s9") is None
    assert engine.traffic_signal_controller_phase("group9") is None


def test_cycle_repeats_after_the_total_duration() -> None:
    scenario = _scenario([], [_controller("group1", "s1", [("stop", 4.0), ("go", 6.0)])])
    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok

    transcript = []
    for _ in range(50):  # 25 s: two and a half 10 s cycles
        assert engine.step(0.5) == scn.Status.Ok
        transcript.append(engine.traffic_signal_state("s1"))
    assert transcript[6] == "stop"  # t = 3.5
    assert transcript[7] == "go"  # t = 4.0, the boundary belongs to the next phase
    assert transcript[18] == "go"  # t = 9.5
    assert transcript[19] == "stop"  # t = 10.0, the cycle restarts
    assert transcript[27] == "go"  # t = 14.0


def test_phase_is_independent_of_the_step_pattern() -> None:
    # The phase is derived from simulation time with fmod, not accumulated, so
    # two engines reaching the same instant differently agree. Both step
    # sequences sum to exactly 20 s (4.0 and 0.125 are exact binary fractions).
    def run(dt, count):
        engine = scn.Engine()
        controller = _controller("main", "s1", [("stop", 7.0), ("caution", 2.5), ("go", 9.5)])
        assert engine.init(_scenario([], [controller])) == scn.Status.Ok
        for _ in range(count):
            assert engine.step(dt) == scn.Status.Ok
        return engine

    coarse = run(4.0, 5)
    fine = run(0.125, 160)
    assert coarse.time == fine.time
    assert coarse.traffic_signal_state("s1") == fine.traffic_signal_state("s1")
    assert coarse.traffic_signal_controller_phase(
        "main"
    ) == fine.traffic_signal_controller_phase("main")


def test_delayed_controller_starts_after_its_reference() -> None:
    # §6.11.3: a chained controller's first phase starts `delay` seconds after
    # the reference's does; until then it has no phase at all.
    scenario = _scenario(
        [],
        [
            _controller("first", "s1", [("a", 20.0), ("b", 20.0)]),
            _controller("second", "s2", [("a", 20.0), ("b", 20.0)], delay=4.0, reference="first"),
        ],
    )
    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    assert engine.traffic_signal_controller_phase("first") == "a"
    assert engine.traffic_signal_controller_phase("second") is None
    assert engine.traffic_signal_state("s2") is None

    for _ in range(8):  # t = 4
        assert engine.step(0.5) == scn.Status.Ok
    assert engine.traffic_signal_controller_phase("second") == "a"
    assert engine.traffic_signal_state("s2") == "a"


# --- Load-time validation ---------------------------------------------------


def test_delay_without_reference_fails_init() -> None:
    controller = _controller("group1", "s1", [("a", 5.0)], delay=3.0)
    engine = scn.Engine()
    assert engine.init(_scenario([], [controller])) == scn.Status.ValidationError
    assert any("delay requires a reference" in d.message for d in engine.diagnostics())


def test_unknown_reference_fails_init_c713() -> None:
    controller = _controller("group1", "s1", [("a", 5.0)], delay=3.0, reference="nobody")
    engine = scn.Engine()
    assert engine.init(_scenario([], [controller])) == scn.Status.SemanticError
    assert any(
        d.rule_id
        == "asam.net:xosc:1.0.0:reference_control.traffic_signal_controller_references"
        for d in engine.diagnostics()
    )


def test_negative_phase_duration_fails_init_c23() -> None:
    controller = _controller("group1", "s1", [("a", 5.0), ("b", -1.0)])
    engine = scn.Engine()
    assert engine.init(_scenario([], [controller])) == scn.Status.ValidationError
    assert any(
        d.rule_id == "asam.net:xosc:1.0.0:data_type.phase_duration_positive"
        for d in engine.diagnostics()
    )


def test_zero_total_duration_pins_the_first_phase() -> None:
    controller = _controller("group1", "s1", [("stop", 0.0), ("go", 0.0)])
    engine = scn.Engine()
    assert engine.init(_scenario([], [controller])) == scn.Status.Ok
    assert any("zero total duration" in d.message for d in engine.diagnostics())
    for _ in range(5):
        assert engine.step(1.0) == scn.Status.Ok
        assert engine.traffic_signal_state("s1") == "stop"


# --- The infrastructure actions (§7.4.2) -----------------------------------


def test_controller_action_restarts_the_cycle_at_a_named_phase() -> None:
    controller = _controller(
        "group1", "s1", [("stop", 20.0), ("caution", 4.0), ("go", 16.0)]
    )
    scenario = _scenario(
        [_timed_event("jump", 5.0, scn.TrafficSignalControllerAction("group1", "caution"))],
        [controller],
    )
    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    assert engine.traffic_signal_state("s1") == "stop"

    for _ in range(10):  # t = 5
        assert engine.step(0.5) == scn.Status.Ok
    assert engine.traffic_signal_controller_phase("group1") == "caution"

    for _ in range(8):  # t = 9 = 5 + the 4 s caution phase
        assert engine.step(0.5) == scn.Status.Ok
    assert engine.traffic_signal_state("s1") == "go"


def test_controller_action_unknown_phase_fails_init_c711() -> None:
    controller = _controller("group1", "s1", [("go", 5.0)])
    scenario = _scenario(
        [_timed_event("jump", 1.0, scn.TrafficSignalControllerAction("group1", "nope"))],
        [controller],
    )
    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.SemanticError
    assert any(
        d.rule_id
        == "asam.net:xosc:1.0.0:reference_control.traffic_signal_controller_action_references"
        for d in engine.diagnostics()
    )


def test_state_action_overrides_until_the_next_phase_transition() -> None:
    # The §11.12 traffic-light-failure shape.
    controller = _controller("group1", "s1", [("stop", 10.0), ("go", 10.0)])
    scenario = _scenario(
        [_timed_event("break", 4.0, scn.TrafficSignalStateAction("s1", "red;green"))],
        [controller],
    )
    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok

    for _ in range(8):  # t = 4
        assert engine.step(0.5) == scn.Status.Ok
    assert engine.traffic_signal_state("s1") == "red;green"
    for _ in range(11):  # t = 9.5, still broken
        assert engine.step(0.5) == scn.Status.Ok
    assert engine.traffic_signal_state("s1") == "red;green"
    assert engine.traffic_signal_controller_phase("group1") == "stop"

    assert engine.step(0.5) == scn.Status.Ok  # t = 10: the phase transition wins
    assert engine.traffic_signal_state("s1") == "go"


# --- The signal conditions (§7.6.5.2) --------------------------------------


def test_signal_conditions_gate_events() -> None:
    controller = _controller("group1", "s1", [("stop", 5.0), ("go", 5.0)])
    scenario = _scenario(
        [
            _gated_event(
                "on-signal",
                scn.TrafficSignalCondition("s1", "go"),
                scn.VariableSetAction("signal_seen", "yes"),
            ),
            _gated_event(
                "on-phase",
                scn.TrafficSignalControllerCondition("group1", "go"),
                scn.VariableSetAction("phase_seen", "yes"),
            ),
        ],
        [controller],
    )
    scenario.declare_variable("signal_seen", "no")
    scenario.declare_variable("phase_seen", "no")

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    for _ in range(8):  # t = 4, still "stop"
        assert engine.step(0.5) == scn.Status.Ok
    assert engine.variable("signal_seen") == "no"
    assert engine.variable("phase_seen") == "no"

    for _ in range(4):  # past t = 5
        assert engine.step(0.5) == scn.Status.Ok
    assert engine.variable("signal_seen") == "yes"
    assert engine.variable("phase_seen") == "yes"


def test_controller_condition_unknown_phase_fails_init_c712() -> None:
    controller = _controller("group1", "s1", [("go", 5.0)])
    scenario = _scenario(
        [
            _gated_event(
                "gated",
                scn.TrafficSignalControllerCondition("group1", "amber"),
                scn.VariableSetAction("seen", "yes"),
            )
        ],
        [controller],
    )
    scenario.declare_variable("seen", "no")

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.SemanticError
    assert any(
        d.rule_id
        == "asam.net:xosc:1.0.0:reference_control.traffic_signal_controller_condition_references"
        for d in engine.diagnostics()
    )


def test_unknown_signal_condition_is_false_and_warns() -> None:
    scenario = _scenario(
        [
            _gated_event(
                "gated",
                scn.TrafficSignalCondition("ghost", "go"),
                scn.VariableSetAction("seen", "yes"),
            )
        ]
    )
    scenario.declare_variable("seen", "no")

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    for _ in range(5):
        assert engine.step(1.0) == scn.Status.Ok
    assert engine.variable("seen") == "no"
    # Warn-once: a signal id is a road-network reference (rule C.7.10) Scena
    # cannot resolve until p3-s4.
    warnings = [d for d in engine.diagnostics() if "traffic signal 'ghost'" in d.message]
    assert len(warnings) == 1


def test_rising_edge_turns_a_level_condition_into_reaches() -> None:
    controller = _controller("group1", "s1", [("stop", 5.0), ("go", 5.0)])
    scenario = _scenario(
        [
            _gated_event(
                "counted",
                scn.TrafficSignalCondition("s1", "go"),
                scn.VariableModifyAction("count", scn.ModifyOperator.Add, 1.0),
                edge=scn.ConditionEdge.Rising,
            )
        ],
        [controller],
    )
    scenario.declare_variable("count", "0")

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    for _ in range(60):  # 30 s: three "go" phases
        assert engine.step(0.5) == scn.Status.Ok
    # The event has a default maximumExecutionCount of 1, so a rising edge
    # fires it exactly once even though the level holds repeatedly.
    assert engine.variable("count") == "1"


# --- IR value types ---------------------------------------------------------


def test_traffic_signal_value_types_round_trip() -> None:
    state = scn.TrafficSignalState("signal1", "off;off;on")
    assert state.traffic_signal_id == "signal1"
    assert state.state == "off;off;on"

    phase = scn.Phase("go", 20.0, [state])
    assert phase.name == "go"
    assert phase.duration == pytest.approx(20.0)
    assert len(phase.signal_states) == 1

    controller = scn.TrafficSignalController("g1", delay=4.5, reference="g0", phases=[phase])
    assert controller.name == "g1"
    assert controller.delay == pytest.approx(4.5)
    assert controller.reference == "g0"
    assert len(controller.phases) == 1
    # An unchained controller has neither.
    plain = scn.TrafficSignalController("g0")
    assert plain.delay is None
    assert plain.reference is None
    assert plain.phases == []
