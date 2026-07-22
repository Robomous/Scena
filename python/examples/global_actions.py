# Copyright 2026 Robomous
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Global actions: variables, the entity lifecycle, environment and commands.

Builds one scenario that exercises the actor-less §7.4.2 / §7.4.3 actions:

  * a VariableAction pair writing the runtime variable store a
    VariableCondition then reads (§6.12) — a feedback loop closed entirely
    inside the scenario, with no host in the middle;
  * a DeleteEntityAction taking a vehicle out of the scenario and an
    AddEntityAction bringing it back somewhere else (§EntityAction);
  * an EnvironmentAction merging weather and anchoring the simulated clock
    (§Environment);
  * a CustomCommandAction handed to the host through the gateway (§7.4.3).

Run with:  python python/examples/global_actions.py
"""

import scena as scn


def timed_event(name, at_time, action):
    """An event that fires `action` when simulation time reaches `at_time`."""
    event = scn.Event(name, start_trigger=scn.make_trigger(scn.SimulationTimeCondition(at_time)))
    event.add_action(action)
    return event


def gated_event(name, condition, action):
    event = scn.Event(name, start_trigger=scn.make_trigger(condition))
    event.add_action(action)
    return event


def build_scenario():
    scenario = scn.Scenario("global-actions-demo")
    scenario.add_entity(scn.Entity("ego", "ego", scn.ControlMode.EngineControlled))
    scenario.add_entity(scn.Entity("hazard", "hazard", scn.ControlMode.EngineControlled))

    # §6.12: variables are declared with an initialization value and change at
    # runtime; parameters (§9.1) are immutable, which is why the deprecated
    # ParameterAction pair exists only for 1.0/1.1 content.
    scenario.declare_variable("hazard_cleared", "false")
    scenario.declare_variable("clearances", "0")

    scenario.add_init_action(scn.SpeedAction("ego", 15.0))
    scenario.add_init_action(scn.TeleportAction("hazard", scn.WorldPosition(200.0, 0.0, 0.0)))

    # The environment: fog rolling in, and a simulated clock that advances with
    # simulation time. An absent member "doesn't change" (§Environment), so this
    # update leaves everything it does not mention alone.
    environment = scn.Environment()
    environment.name = "fog-bank"
    weather = scn.Weather()
    weather.fog = scn.Fog(80.0)
    weather.precipitation = scn.Precipitation(scn.PrecipitationType.Rain, 2.5)
    environment.weather = weather
    environment.road_condition = scn.RoadCondition(0.65)
    environment.time_of_day = scn.TimeOfDay(True, scn.DateTime(2026, 7, 22, 7, 45, 0, 0, 0))

    maneuver = scn.Maneuver("maneuver")
    maneuver.add_event(timed_event("weather-turns", 2.0, scn.EnvironmentAction(environment)))
    # The hazard is removed from the scenario, then re-added off to the side.
    maneuver.add_event(timed_event("hazard-removed", 4.0, scn.DeleteEntityAction("hazard")))
    maneuver.add_event(
        timed_event("hazard-cleared-flag", 4.0, scn.VariableSetAction("hazard_cleared", "true"))
    )
    maneuver.add_event(
        timed_event(
            "hazard-returns", 8.0, scn.AddEntityAction("hazard", scn.WorldPosition(400.0, 6.0, 0.0))
        )
    )
    # A VariableCondition reading what a VariableAction wrote, and a
    # VariableModifyAction counting the crossings.
    maneuver.add_event(
        gated_event(
            "count-clearance",
            scn.VariableCondition("hazard_cleared", scn.Rule.EqualTo, "true"),
            scn.VariableModifyAction("clearances", scn.ModifyOperator.Add, 1.0),
        )
    )
    maneuver.add_event(
        timed_event(
            "tell-the-host", 5.0, scn.CustomCommandAction("log", "hazard cleared, resuming")
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

    print(f"{'t':>5}  {'hazard':>8}  {'fog':>6}  {'active':>8}  {'time of day':>12}")
    for step in range(1, 51):  # 10 s at 5 Hz
        if engine.step(0.2) != scn.Status.Ok:
            raise SystemExit("step failed")
        if step % 5:
            continue
        # A deleted entity reports nothing at all — the host sees exactly what
        # the by-entity conditions see.
        active = engine.entity_active("hazard")
        state = engine.state("hazard")
        where = f"x={state.x:6.1f}" if state is not None else "  gone  "
        fog = engine.environment.weather
        visual_range = f"{fog.fog.visual_range:5.0f}m" if fog and fog.fog else "    --"
        instant = engine.date_time
        clock = f"{instant:.1f}" if instant is not None else "unset"
        print(
            f"{engine.time:5.1f}  {where:>8}  {visual_range:>6}  "
            f"{str(active):>8}  {clock:>12}"
        )

    print()
    print(f"hazard_cleared = {engine.variable('hazard_cleared')}")
    print(f"clearances     = {engine.variable('clearances')}")
    print(f"environment    = {engine.environment.name}")
    print(f"friction       = {engine.environment.road_condition.friction_scale_factor}")
    # The CustomCommandAction fired, but with no gateway attached there is
    # nobody to hand it to — a documented no-op, not a diagnostic (§7.4.3).
    print(f"diagnostics    = {len(engine.diagnostics())}")
    for diagnostic in engine.diagnostics():
        print(f"  [{diagnostic.severity}] {diagnostic.message}")


if __name__ == "__main__":
    main()
