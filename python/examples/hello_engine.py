#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2026 Robomous
# SPDX-License-Identifier: Apache-2.0
"""Build a storyboard in code, step it at 100 Hz, and verify the speed action fires.

One engine-controlled entity starts at rest. The storyboard is the full ASAM
OpenSCENARIO hierarchy (Story > Act > ManeuverGroup > Maneuver > Event): an
event triggered by a SimulationTimeCondition at t = 2.0 s sets the entity's
speed to 10 m/s. A second event demonstrates the full trigger model: an OR of
two condition groups, one carrying a rising edge with a 0.5 s delay. A third
event carries a maximumExecutionCount of 3 and so executes on three
consecutive evaluations before completing. The act
stops itself at t = 4.5 s. The host loop steps 5 simulated seconds at 100 Hz,
watches the elements' lifecycle states, and asserts the speed changes happened.
"""

import scena as scn


def build_scenario() -> "scn.Scenario":
    scenario = scn.Scenario("hello-engine")
    scenario.add_entity(scn.Entity("ego", "ego vehicle", scn.ControlMode.EngineControlled))

    event = scn.Event(
        "speed-up",
        start_trigger=scn.make_trigger(scn.SimulationTimeCondition(at_time=2.0)),
    )
    event.add_action(scn.SpeedAction("ego", target_speed=10.0))

    # A second event behind a hand-built trigger: the OR of two condition
    # groups, one of which ANDs a rising edge with a plain condition
    # (ASAM OpenSCENARIO XML 1.4.0 §7.6.1). The rise of "t >= 3.0" is a
    # one-evaluation-wide pulse, delayed by 0.5 s, so this fires at t = 3.5.
    edge_group = scn.ConditionGroup()
    edge_group.add_condition(
        scn.TriggerCondition(
            scn.SimulationTimeCondition(at_time=3.0),
            edge=scn.ConditionEdge.Rising,
            delay=0.5,
            name="ramp-start",
        )
    )
    late_group = scn.ConditionGroup()
    late_group.add_condition(scn.TriggerCondition(scn.SimulationTimeCondition(at_time=30.0)))
    trigger = scn.Trigger()
    trigger.add_group(edge_group)
    trigger.add_group(late_group)

    # Priority is resolved in the scope of the Maneuver (§8.4.2.2). No action
    # the engine can apply is ongoing yet, so no event is ever in runningState
    # when a sibling starts and this override resolves to a plain start; the
    # literal still round-trips through the IR and the scheduler.
    slow_down = scn.Event(
        "slow-down", start_trigger=trigger, priority=scn.EventPriority.Override
    )
    slow_down.add_action(scn.SpeedAction("ego", target_speed=4.0))

    # Sequential re-execution (§8.3.3.2): the event ends, re-arms to standby
    # and starts again on each of the next two evaluations, then completes —
    # executions are its startTransitions plus its skipTransitions (§8.4.2.1).
    nudge = scn.Event(
        "nudge",
        start_trigger=scn.make_trigger(scn.SimulationTimeCondition(at_time=1.6)),
        maximum_execution_count=3,
    )
    nudge.add_action(scn.SpeedAction("ego", target_speed=2.0))

    # Never fires: the act's stop trigger completes the whole subtree first.
    stopped = scn.Event(
        "stop-me", start_trigger=scn.make_trigger(scn.SimulationTimeCondition(at_time=10.0))
    )
    stopped.add_action(scn.SpeedAction("ego", target_speed=25.0))

    maneuver = scn.Maneuver("maneuver")
    maneuver.add_event(nudge)
    maneuver.add_event(event)
    maneuver.add_event(slow_down)
    maneuver.add_event(stopped)
    group = scn.ManeuverGroup("group")
    group.add_actor("ego")
    group.add_maneuver(maneuver)
    act = scn.Act("act")  # no start trigger: starts with the storyboard
    # The act stops itself at t = 4.5 s, taking its whole subtree with it
    # (§7.6.1.2); the storyboard keeps running.
    act.set_stop_trigger(scn.make_trigger(scn.SimulationTimeCondition(at_time=4.5)))
    act.add_group(group)
    story = scn.Story("story")
    story.add_act(act)
    scenario.add_story(story)
    return scenario


def main() -> None:
    print(f"Scena {scn.version()}")

    engine = scn.Engine()
    status = engine.init(build_scenario())
    assert status == scn.Status.Ok, status

    # A valid scenario produces no diagnostics; a defective one would list
    # every finding here (severity, code, element path, and — for standards
    # violations — the ASAM checker rule id).
    diagnostics = engine.diagnostics()
    assert diagnostics == [], diagnostics
    print(f"init: {len(diagnostics)} diagnostic(s)")

    event_path = "story/act/group/maneuver/speed-up"
    assert engine.storyboard_element_state(event_path) == scn.ElementState.Standby

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
    assert final.speed == 4.0, f"delayed rising edge did not fire: {final.speed}"
    assert final.x > 0.0, "entity did not move after the speed action"
    assert engine.storyboard_element_state(event_path) == scn.ElementState.Complete
    # "nudge" spent all three of its executions and ended regularly.
    nudge_path = "story/act/group/maneuver/nudge"
    assert engine.storyboard_element_state(nudge_path) == scn.ElementState.Complete
    assert engine.storyboard_element_transition(nudge_path) == scn.TransitionKind.End
    # The act was stopped at t = 4.5 s, so "stop-me" never fired...
    assert engine.storyboard_element_transition(
        "story/act/group/maneuver/stop-me"
    ) == scn.TransitionKind.Stop
    assert engine.storyboard_element_transition("story/act") == scn.TransitionKind.Stop
    # ...but the storyboard has no stop trigger, so it never self-completes.
    assert engine.storyboard_element_state("") == scn.ElementState.Running

    # Every action the engine applied was a SpeedAction it implements, so the
    # run recorded no runtime warnings.
    run_diagnostics = engine.diagnostics()
    assert run_diagnostics == [], run_diagnostics
    print(f"run: {len(run_diagnostics)} diagnostic(s)")

    engine.close()
    print(
        "nudge ran 3x from t=1.6s, speed action at t=2.0s, "
        "delayed rising edge at t=3.5s: OK"
    )


if __name__ == "__main__":
    main()
