# SPDX-License-Identifier: Apache-2.0
"""Event priority and execution counts through the Python bindings.

ASAM OpenSCENARIO XML 1.4.0 §7.3.2, §8.2, §8.3.3.2 and §8.4.2.
"""

import pytest

import scena as scn

EVENT_PATH = "story/act/group/maneuver/event"


def build_scenario(**event_kwargs: object) -> "scn.Scenario":
    """One story/act/group/maneuver chain around a single timed event."""
    scenario = scn.Scenario("event-lifecycle")
    scenario.add_entity(scn.Entity("ego", "ego vehicle", scn.ControlMode.EngineControlled))

    event = scn.Event(
        "event",
        start_trigger=scn.make_trigger(scn.SimulationTimeCondition(at_time=1.0)),
        **event_kwargs,
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


def test_event_priority_enum_is_exported() -> None:
    assert "EventPriority" in scn.__all__
    assert {"Override", "Parallel", "Skip"} <= set(scn.EventPriority.__members__)


def test_transition_kind_has_skip() -> None:
    # §8.2: skipTransition is specific to Event.
    assert "Skip" in scn.TransitionKind.__members__


def test_event_defaults_are_parallel_and_single_execution() -> None:
    event = scn.Event("event")
    assert event.priority == scn.EventPriority.Parallel
    assert event.maximum_execution_count == 1


@pytest.mark.parametrize(
    "priority",
    [scn.EventPriority.Override, scn.EventPriority.Parallel, scn.EventPriority.Skip],
)
def test_event_priority_round_trips(priority: "scn.EventPriority") -> None:
    event = scn.Event("event", priority=priority)
    assert event.priority == priority
    event.priority = scn.EventPriority.Parallel
    assert event.priority == scn.EventPriority.Parallel


def test_event_re_executes_up_to_maximum_count() -> None:
    # §8.3.3.2: the executions are performed sequentially — the event ends,
    # re-arms to standby, and starts again on the next evaluation.
    engine = scn.Engine()
    assert engine.init(build_scenario(maximum_execution_count=3)) == scn.Status.Ok
    assert engine.storyboard_element_state(EVENT_PATH) == scn.ElementState.Standby

    assert engine.step(1.0) == scn.Status.Ok
    assert engine.storyboard_element_state(EVENT_PATH) == scn.ElementState.Standby
    assert engine.storyboard_element_transition(EVENT_PATH) == scn.TransitionKind.End
    assert engine.state("ego").speed == 10.0

    assert engine.step(0.01) == scn.Status.Ok
    assert engine.storyboard_element_state(EVENT_PATH) == scn.ElementState.Standby

    assert engine.step(0.01) == scn.Status.Ok
    assert engine.storyboard_element_state(EVENT_PATH) == scn.ElementState.Complete
    assert engine.storyboard_element_transition(EVENT_PATH) == scn.TransitionKind.End


def test_single_execution_event_does_not_repeat() -> None:
    engine = scn.Engine()
    assert engine.init(build_scenario()) == scn.Status.Ok
    assert engine.step(1.0) == scn.Status.Ok
    assert engine.storyboard_element_state(EVENT_PATH) == scn.ElementState.Complete


def test_zero_maximum_execution_count_completes_with_skip_transition() -> None:
    # §8.4.2.1 read with a budget of zero: the event is exhausted in standby
    # and completes with a skipTransition without ever executing.
    engine = scn.Engine()
    assert engine.init(build_scenario(maximum_execution_count=0)) == scn.Status.Ok
    assert engine.storyboard_element_state(EVENT_PATH) == scn.ElementState.Complete
    assert engine.storyboard_element_transition(EVENT_PATH) == scn.TransitionKind.Skip

    assert engine.step(1.0) == scn.Status.Ok
    assert engine.state("ego").speed == 0.0


def test_negative_maximum_execution_count_is_rejected() -> None:
    engine = scn.Engine()
    assert engine.init(build_scenario(maximum_execution_count=-1)) == scn.Status.ValidationError
    assert not engine.initialized


def test_priority_literals_are_accepted_by_the_engine() -> None:
    # No action the engine can apply is ongoing (§7.4.1.2), so no event is
    # ever in runningState when a sibling starts: every literal resolves to a
    # plain start here. The point is that all three branches are walked.
    for priority in (
        scn.EventPriority.Override,
        scn.EventPriority.Parallel,
        scn.EventPriority.Skip,
    ):
        engine = scn.Engine()
        assert engine.init(build_scenario(priority=priority)) == scn.Status.Ok
        assert engine.step(1.0) == scn.Status.Ok
        assert engine.state("ego").speed == 10.0
        assert engine.storyboard_element_transition(EVENT_PATH) == scn.TransitionKind.End
