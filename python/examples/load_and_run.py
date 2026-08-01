#!/usr/bin/env python3

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

"""Load an OpenSCENARIO XML scenario and run it.

The shortest useful Scena program: parse a `.xosc`, hand the scenario to the
engine, step it, read the result. `load_string_with_diagnostics` is the form to
prefer — a document that loads with `Status.Ok` may still have told you
something worth reading.
"""

import scena as scn

SCENARIO = """<?xml version="1.0"?>
<OpenSCENARIO>
  <FileHeader revMajor="1" revMinor="3" date="2026-08-01T00:00:00"
              description="load and run" author="scena"/>
  <Entities>
    <ScenarioObject name="ego"><Vehicle name="ego_v" vehicleCategory="car">
      <BoundingBox><Center x="0" y="0" z="0"/>
        <Dimensions width="2" length="4" height="1.5"/></BoundingBox>
      <Performance maxSpeed="60" maxAcceleration="5" maxDeceleration="9"/>
      <Axles><RearAxle maxSteering="0" wheelDiameter="0.6" trackWidth="1.7"
                       positionX="0" positionZ="0.3"/></Axles>
    </Vehicle></ScenarioObject>
  </Entities>
  <Storyboard>
    <Init><Actions><Private entityRef="ego"><PrivateAction><TeleportAction>
      <Position><WorldPosition x="0" y="0" z="0" h="0"/></Position>
    </TeleportAction></PrivateAction></Private></Actions></Init>
    <Story name="story"><Act name="act">
      <ManeuverGroup name="group" maximumExecutionCount="1">
        <Actors selectTriggeringEntities="false"><EntityRef entityRef="ego"/></Actors>
        <Maneuver name="maneuver"><Event name="accelerate" priority="parallel">
          <Action name="go"><PrivateAction><LongitudinalAction><SpeedAction>
            <SpeedActionDynamics dynamicsShape="linear" dynamicsDimension="time" value="4"/>
            <SpeedActionTarget><AbsoluteTargetSpeed value="20"/></SpeedActionTarget>
          </SpeedAction></LongitudinalAction></PrivateAction></Action>
          <StartTrigger><ConditionGroup><Condition name="at1" delay="0" conditionEdge="none">
            <ByValueCondition><SimulationTimeCondition value="1" rule="greaterThan"/></ByValueCondition>
          </Condition></ConditionGroup></StartTrigger>
        </Event></Maneuver>
      </ManeuverGroup>
    </Act></Story>
  </Storyboard>
</OpenSCENARIO>
"""


def main() -> None:
    status, scenario, diagnostics = scn.load_string_with_diagnostics(SCENARIO)
    assert status == scn.Status.Ok, status
    # A clean load can still carry warnings — deprecated spellings, constructs
    # outside the implemented subset. Read them even when the status is Ok.
    for diagnostic in diagnostics:
        print(f"  {diagnostic.severity.name}: {diagnostic.path}: {diagnostic.message}")
    print(f"loaded '{scenario.name}' with {len(diagnostics)} finding(s)")

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok

    # 10 s at 50 Hz. The host owns the clock; nothing moves without step().
    for _ in range(500):
        assert engine.step(0.02) == scn.Status.Ok

    state = engine.state("ego")
    print(f"ego: x={state.x:.2f} m, speed={state.speed:.2f} m/s at t={engine.time:.1f} s")
    assert state.speed == 20.0, state.speed
    assert engine.storyboard_element_state("story/act/group/maneuver/accelerate") == (
        scn.ElementState.Complete
    )

    engine.close()
    print("load and run: OK")


if __name__ == "__main__":
    main()
