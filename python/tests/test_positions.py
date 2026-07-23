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

"""Position resolution through the Python bindings (p2-s4): the §6.3.8 variants
teleport an entity to a resolved world pose, and orientation composes per
§Orientation."""

import math

import pytest

import scena as scn


def _scenario(entities, init_actions):
    scenario = scn.Scenario("positions")
    for entity in entities:
        scenario.add_entity(scn.Entity(entity, entity, scn.ControlMode.EngineControlled))
    for action in init_actions:
        scenario.add_init_action(action)
    return scenario


def test_world_position_writes_full_pose():
    engine = scn.Engine()
    scenario = _scenario(
        ["ego"],
        [scn.TeleportAction("ego", scn.WorldPosition(1.0, 2.0, 3.0, h=0.5, p=0.1, r=-0.2))],
    )
    assert engine.init(scenario) == scn.Status.Ok
    state = engine.state("ego")
    assert state.x == 1.0
    assert state.heading == 0.5
    assert state.pitch == 0.1
    assert state.roll == -0.2


def test_relative_world_position_uses_world_axes():
    engine = scn.Engine()
    scenario = _scenario(
        ["lead", "ego"],
        [
            scn.TeleportAction("lead", scn.WorldPosition(10.0, 20.0, 0.0)),
            scn.TeleportAction("ego", scn.RelativeWorldPosition("lead", dx=5.0, dy=-3.0)),
        ],
    )
    assert engine.init(scenario) == scn.Status.Ok
    state = engine.state("ego")
    assert state.x == pytest.approx(15.0)
    assert state.y == pytest.approx(17.0)


def test_relative_object_position_rotates_into_entity_frame():
    engine = scn.Engine()
    scenario = _scenario(
        ["lead", "ego"],
        [
            # lead faces +90 deg; "2 m ahead" is +2 in world Y.
            scn.TeleportAction("lead", scn.WorldPosition(5.0, 5.0, 0.0, h=math.pi / 2.0)),
            scn.TeleportAction("ego", scn.RelativeObjectPosition("lead", dx=2.0)),
        ],
    )
    assert engine.init(scenario) == scn.Status.Ok
    state = engine.state("ego")
    assert state.x == pytest.approx(5.0, abs=1e-9)
    assert state.y == pytest.approx(7.0, abs=1e-9)


def test_absolute_orientation_ignores_reference_entity():
    engine = scn.Engine()
    scenario = _scenario(
        ["lead", "ego"],
        [
            scn.TeleportAction("lead", scn.WorldPosition(0.0, 0.0, 0.0, h=1.0)),
            scn.TeleportAction(
                "ego",
                scn.RelativeWorldPosition(
                    "lead",
                    orientation=scn.Orientation(h=0.3, type=scn.ReferenceContext.Absolute),
                ),
            ),
        ],
    )
    assert engine.init(scenario) == scn.Status.Ok
    assert engine.state("ego").heading == pytest.approx(0.3)


def test_unsupported_variant_is_reported_and_no_op():
    engine = scn.Engine()
    scenario = _scenario(
        ["ego"],
        [scn.TeleportAction("ego", scn.GeoPosition(latitude_deg=48.1, longitude_deg=11.5))],
    )
    assert engine.init(scenario) == scn.Status.Ok
    # The teleport is a no-op: ego stays at the origin.
    assert engine.state("ego").x == 0.0
    # A rule-cited diagnostic explains why.
    codes = [d.code for d in engine.diagnostics()]
    assert scn.Status.UnsupportedFeature in codes
    rules = [d.rule_id for d in engine.diagnostics() if d.rule_id]
    assert "asam.net:xosc:1.1.0:positioning.geodetic_datum_defined" in rules
