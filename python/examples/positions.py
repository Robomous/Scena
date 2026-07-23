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

"""Teleport entities using the ASAM OpenSCENARIO §6.3.8 Position variants.

The PositionResolver (p2-s4) maps each variant to a world pose. This example
places a "lead" with a heading, then teleports "ego" three ways relative to it —
a world pose, world-axis deltas (RelativeWorldPosition), and deltas in the
lead's own frame (RelativeObjectPosition) — and finally shows that a variant
with no backend yet (GeoPosition) is reported, never silently applied.
"""

import math

import scena as scn


def build_scenario() -> "scn.Scenario":
    scenario = scn.Scenario("positions")
    scenario.add_entity(scn.Entity("lead", "lead", scn.ControlMode.EngineControlled))
    scenario.add_entity(scn.Entity("ego", "ego", scn.ControlMode.EngineControlled))

    # Place the lead at (10, 20) facing +90 degrees (along world +Y).
    scenario.add_init_action(
        scn.TeleportAction("lead", scn.WorldPosition(10.0, 20.0, 0.0, h=math.pi / 2.0))
    )
    # "ego" 3 m ahead of the lead in the lead's own frame: since the lead faces
    # +Y, "ahead" is +3 in world Y — the deltas rotate with the reference.
    scenario.add_init_action(
        scn.TeleportAction("ego", scn.RelativeObjectPosition("lead", dx=3.0))
    )
    return scenario


def main() -> None:
    print(f"Scena {scn.version()}")
    engine = scn.Engine()
    assert engine.init(build_scenario()) == scn.Status.Ok

    lead = engine.state("lead")
    ego = engine.state("ego")
    print(f"lead: x={lead.x:.2f} y={lead.y:.2f} heading={lead.heading:.3f}")
    print(f"ego : x={ego.x:.2f} y={ego.y:.2f}  (3 m ahead of the lead's +Y heading)")
    assert ego.x == round(lead.x, 6)  # no world-X change
    assert ego.y > lead.y  # moved forward along the lead's heading

    # A GeoPosition has no geodetic datum to resolve against yet: the teleport is
    # reported (never silently wrong) and the entity does not move.
    geo = scn.Engine()
    geo_scenario = scn.Scenario("geo")
    geo_scenario.add_entity(scn.Entity("ego", "ego", scn.ControlMode.EngineControlled))
    geo_scenario.add_init_action(
        scn.TeleportAction("ego", scn.GeoPosition(latitude_deg=48.14, longitude_deg=11.58))
    )
    assert geo.init(geo_scenario) == scn.Status.Ok
    assert geo.state("ego").x == 0.0  # untouched
    rule = next(d.rule_id for d in geo.diagnostics() if d.rule_id)
    print(f"geo teleport reported, entity untouched; rule: {rule}")


if __name__ == "__main__":
    main()
