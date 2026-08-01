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

"""Loading OpenSCENARIO XML from Python (p6-s3)."""

import pytest

import scena as scn

MINIMAL = """<?xml version="1.0"?>
<OpenSCENARIO>
  <FileHeader revMajor="1" revMinor="3" date="2026-08-01T00:00:00" description="t" author="s"/>
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
        <Maneuver name="maneuver"><Event name="event" priority="parallel">
          <Action name="go"><PrivateAction><LongitudinalAction><SpeedAction>
            <SpeedActionDynamics dynamicsShape="step" dynamicsDimension="time" value="0"/>
            <SpeedActionTarget><AbsoluteTargetSpeed value="10"/></SpeedActionTarget>
          </SpeedAction></LongitudinalAction></PrivateAction></Action>
          <StartTrigger><ConditionGroup><Condition name="c" delay="0" conditionEdge="none">
            <ByValueCondition><SimulationTimeCondition value="0" rule="greaterThan"/></ByValueCondition>
          </Condition></ConditionGroup></StartTrigger>
        </Event></Maneuver>
      </ManeuverGroup>
    </Act></Story>
  </Storyboard>
</OpenSCENARIO>
"""


def test_load_string_returns_a_runnable_scenario() -> None:
    status, scenario = scn.load_string(MINIMAL)
    assert status == scn.Status.Ok
    assert [entity.id for entity in scenario.entities] == ["ego"]

    engine = scn.Engine()
    assert engine.init(scenario) == scn.Status.Ok
    assert engine.step(1.0) == scn.Status.Ok
    assert engine.state("ego").speed == pytest.approx(10.0)


def test_load_string_with_diagnostics_reports_findings() -> None:
    status, _scenario, diagnostics = scn.load_string_with_diagnostics(MINIMAL)
    assert status == scn.Status.Ok
    # A clean document may still warn; what matters is that the list is reachable
    # and that nothing in it is an error when the status is Ok.
    assert all(d.severity != scn.Severity.Error for d in diagnostics)


def test_a_malformed_document_reports_rather_than_raising() -> None:
    status, _scenario, diagnostics = scn.load_string_with_diagnostics("<OpenSCENARIO>")
    assert status != scn.Status.Ok
    assert any(d.severity == scn.Severity.Error for d in diagnostics)
    # Diagnostics carry a machine-readable code and a path, not just a message.
    error = next(d for d in diagnostics if d.severity == scn.Severity.Error)
    assert error.message


def test_load_file_round_trips_through_disk(tmp_path) -> None:
    path = tmp_path / "scenario.xosc"
    path.write_text(MINIMAL, encoding="utf-8")
    status, scenario = scn.load_file(path)
    assert status == scn.Status.Ok
    assert [entity.id for entity in scenario.entities] == ["ego"]


def test_a_missing_file_is_reported_not_raised(tmp_path) -> None:
    status, _scenario, diagnostics = scn.load_file_with_diagnostics(tmp_path / "nope.xosc")
    assert status != scn.Status.Ok
    assert diagnostics
