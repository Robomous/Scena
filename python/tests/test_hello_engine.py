# SPDX-License-Identifier: Apache-2.0
"""Pytest suite covering the hello_engine flow through the Python bindings."""

import scena as scn


def build_scenario(at_time: float = 2.0) -> "scn.Scenario":
    scenario = scn.Scenario("hello-engine")
    scenario.add_entity(scn.Entity("ego", "ego vehicle", scn.ControlMode.EngineControlled))

    event = scn.Event(
        "speed-up", start_trigger=scn.make_trigger(scn.SimulationTimeCondition(at_time=at_time))
    )
    event.add_action(scn.SpeedAction("ego", target_speed=10.0))
    maneuver = scn.Maneuver("maneuver")
    maneuver.add_event(event)
    group = scn.ManeuverGroup("group")
    group.add_actor("ego")
    group.add_maneuver(maneuver)
    act = scn.Act("act")
    act.add_group(group)
    story = scn.Story("story")
    story.add_act(act)
    scenario.add_story(story)
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


def test_storyboard_element_states() -> None:
    engine = scn.Engine()
    assert engine.init(build_scenario()) == scn.Status.Ok

    path = "story/act/group/maneuver/speed-up"
    assert engine.storyboard_element_state(path) == scn.ElementState.Standby
    assert engine.storyboard_element_state("") == scn.ElementState.Running
    assert engine.storyboard_element_state("story/none") is None

    assert engine.step(3.0) == scn.Status.Ok
    assert engine.storyboard_element_state(path) == scn.ElementState.Complete
    assert engine.storyboard_element_transition(path) == scn.TransitionKind.End
    # A storyboard without a stop trigger never completes (§8.4.7).
    assert engine.storyboard_element_state("") == scn.ElementState.Running


def test_init_actions_and_stop_trigger() -> None:
    scenario = build_scenario(at_time=5.0)
    scenario.add_init_action(scn.SpeedAction("ego", target_speed=3.0))
    scenario.set_stop_trigger(scn.make_trigger(scn.SimulationTimeCondition(at_time=2.0)))

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    # Init actions apply before simulation time starts.
    assert engine.state("ego").speed == 3.0
    assert engine.state("ego").x == 0.0

    for _ in range(600):  # 6 simulated seconds
        assert engine.step(0.01) == scn.Status.Ok

    # The stop trigger completed the storyboard before the t=5 event fired.
    assert engine.storyboard_element_state("") == scn.ElementState.Complete
    assert engine.storyboard_element_transition("") == scn.TransitionKind.Stop
    assert engine.state("ego").speed == 3.0


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


def _single_event_scenario(trigger: "scn.Trigger", act_stop: "scn.Trigger | None" = None):
    """One ego entity and one event behind `trigger`, in a minimal hierarchy."""
    scenario = scn.Scenario("triggers")
    scenario.add_entity(scn.Entity("ego", "ego vehicle", scn.ControlMode.EngineControlled))
    event = scn.Event("event", start_trigger=trigger)
    event.add_action(scn.SpeedAction("ego", target_speed=10.0))
    maneuver = scn.Maneuver("maneuver")
    maneuver.add_event(event)
    group = scn.ManeuverGroup("group")
    group.add_maneuver(maneuver)
    act = scn.Act("act", stop_trigger=act_stop)
    act.add_group(group)
    story = scn.Story("story")
    story.add_act(act)
    scenario.add_story(story)
    return scenario


def test_condition_groups_are_or_of_ands() -> None:
    # (t >= 5) OR (t >= 1 AND t >= 2): the second group fires first (§7.6.1).
    late = scn.ConditionGroup()
    late.add_condition(scn.TriggerCondition(scn.SimulationTimeCondition(at_time=5.0)))
    both = scn.ConditionGroup()
    both.add_condition(scn.TriggerCondition(scn.SimulationTimeCondition(at_time=1.0)))
    both.add_condition(scn.TriggerCondition(scn.SimulationTimeCondition(at_time=2.0)))
    trigger = scn.Trigger()
    trigger.add_group(late)
    trigger.add_group(both)

    engine = scn.Engine()
    assert engine.init(_single_event_scenario(trigger)) == scn.Status.Ok
    path = "story/act/group/maneuver/event"

    assert engine.step(1.5) == scn.Status.Ok
    assert engine.storyboard_element_state(path) == scn.ElementState.Standby
    assert engine.step(0.5) == scn.Status.Ok  # t = 2.0
    assert engine.storyboard_element_state(path) == scn.ElementState.Complete
    assert engine.state("ego").speed == 10.0


def test_rising_edge_with_delay() -> None:
    # A rising edge delayed by 0.5 s fires one evaluation, half a second
    # after the expression rises (§7.6.2.1, §7.6.3).
    trigger = scn.make_trigger(
        scn.SimulationTimeCondition(at_time=1.0), edge=scn.ConditionEdge.Rising, delay=0.5
    )
    engine = scn.Engine()
    assert engine.init(_single_event_scenario(trigger)) == scn.Status.Ok
    path = "story/act/group/maneuver/event"

    for _ in range(140):  # up to t = 1.4 s
        assert engine.step(0.01) == scn.Status.Ok
    assert engine.storyboard_element_state(path) == scn.ElementState.Standby

    for _ in range(20):  # past t = 1.5 s
        assert engine.step(0.01) == scn.Status.Ok
    assert engine.storyboard_element_state(path) == scn.ElementState.Complete


def test_constant_true_never_raises_a_rising_edge() -> None:
    # §7.6.4: the first check of an edge condition is false, so an
    # expression that already holds never produces a rise.
    trigger = scn.make_trigger(
        scn.SimulationTimeCondition(at_time=0.0), edge=scn.ConditionEdge.Rising
    )
    engine = scn.Engine()
    assert engine.init(_single_event_scenario(trigger)) == scn.Status.Ok

    for _ in range(300):
        assert engine.step(0.01) == scn.Status.Ok
    assert (
        engine.storyboard_element_state("story/act/group/maneuver/event")
        == scn.ElementState.Standby
    )


def test_negative_delay_is_rejected() -> None:
    # per rule asam.net:xosc:1.0.0:data_type.condition_delay_not_negative
    trigger = scn.make_trigger(scn.SimulationTimeCondition(at_time=1.0), delay=-0.5)
    engine = scn.Engine()
    assert engine.init(_single_event_scenario(trigger)) == scn.Status.ValidationError
    assert not engine.initialized


def test_empty_trigger_is_always_false() -> None:
    # §7.6.1: an empty trigger never fires, so the event stays in standby.
    engine = scn.Engine()
    assert engine.init(_single_event_scenario(scn.Trigger())) == scn.Status.Ok
    for _ in range(300):
        assert engine.step(0.01) == scn.Status.Ok
    assert (
        engine.storyboard_element_state("story/act/group/maneuver/event")
        == scn.ElementState.Standby
    )


def test_act_stop_trigger_stops_its_subtree() -> None:
    # §7.6.1.2: the act and its descendants complete with a stopTransition,
    # while the storyboard — which has no stop trigger — keeps running.
    engine = scn.Engine()
    scenario = _single_event_scenario(
        scn.make_trigger(scn.SimulationTimeCondition(at_time=3.0)),
        act_stop=scn.make_trigger(scn.SimulationTimeCondition(at_time=1.0)),
    )
    assert engine.init(scenario) == scn.Status.Ok

    for _ in range(400):  # 4 simulated seconds
        assert engine.step(0.01) == scn.Status.Ok

    assert engine.storyboard_element_transition("story/act") == scn.TransitionKind.Stop
    assert (
        engine.storyboard_element_transition("story/act/group/maneuver/event")
        == scn.TransitionKind.Stop
    )
    assert engine.storyboard_element_state("") == scn.ElementState.Running
    assert engine.state("ego").speed == 0.0  # the t = 3.0 event never fired
