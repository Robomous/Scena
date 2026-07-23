# Positions and control ownership

This chapter covers how a **position** becomes a world pose, and who owns an
entity's state. It follows ASAM OpenSCENARIO XML 1.4.0 §6.3 (coordinate
systems), §6.3.8 (the ten position variants), §Orientation, and §5 (the
corrected ≥1.3 calculations, applied uniformly to all input versions).

A position is a target for `TeleportAction`, `AcquirePositionAction`,
`AddEntityAction`, and (in later sprints) the position conditions. The engine
resolves it once, through a single **PositionResolver**, into a world pose:
`x/y/z` plus a heading/pitch/roll orientation.

## The ten variants

`Position` is one of ten variants (§6.3.8), in the standard's order. Scena
resolves the **self-contained** ones today; the rest need a backend that has not
landed yet and are **reported, never silently applied**:

| Variant | Resolves | Notes |
|---|---|---|
| `WorldPosition` | ✅ now | Absolute `x/y/z` + world-frame `h/p/r`. |
| `RelativeWorldPosition` | ✅ now | `dx/dy/dz` along the **world** axes from a reference entity — not rotated. |
| `RelativeObjectPosition` | ✅ now | `dx/dy/dz` in the reference entity's **local** frame — rotated by its heading. |
| `RoadPosition`, `RelativeRoadPosition` | ⏳ p3-s4 | Need a road network (`IRoadQuery`). |
| `LanePosition`, `RelativeLanePosition` | ⏳ p3-s4 | Need a road network. |
| `RoutePosition` | ⏳ p3-s4 | Needs a resolved route. |
| `TrajectoryPosition` | ⏳ p2-s5 | Needs trajectory shapes. |
| `GeoPosition` | ⏳ post | Needs a geodetic datum; reports `asam.net:xosc:1.1.0:positioning.geodetic_datum_defined`. |

When a variant cannot be resolved, the action that used it is a **no-op** and a
diagnostic is recorded (an `UnsupportedFeature` warning, with the ASAM rule id
where the standard defines one). Nothing is guessed.

### World versus local deltas

The distinction between the two relative-to-entity variants is the single most
important rule:

- `RelativeWorldPosition` adds its deltas along the **world** axes. `dx=5` is
  always "5 m east", whichever way the reference entity faces.
- `RelativeObjectPosition` expresses its deltas in the reference entity's **own**
  frame. For an entity facing +90°, `dx=5` ("5 m ahead") lands 5 m along world
  +Y. The rotation goes through the deterministic `det_sincos`, so the result is
  bit-identical on every platform.

## Orientation

Every variant except `WorldPosition` may carry an `Orientation` (`h/p/r` plus a
`ReferenceContext`). `WorldPosition`'s own `h/p/r` are inherently world-frame.

- **Missing** orientation → the pose copies the reference frame's orientation.
- **`Absolute`** → the angles are taken directly in the world frame; the
  reference orientation is ignored.
- **`Relative`** → the angles are an additive (counter-clockwise) shift on top of
  the reference orientation.

Rotations apply in the order Z (heading), then Y (pitch), then X (roll). The
straight-line runtime keeps pitch and roll at 0, so today only heading varies for
engine-driven motion; a host may report any full pose for its own entities.

```python
import math, scena as scn

engine = scn.Engine()
s = scn.Scenario("positions")
s.add_entity(scn.Entity("lead", "lead", scn.ControlMode.EngineControlled))
s.add_entity(scn.Entity("ego", "ego", scn.ControlMode.EngineControlled))
# Place the lead facing +90°, then put ego 3 m ahead of it in the lead's frame.
s.add_init_action(scn.TeleportAction("lead", scn.WorldPosition(10, 20, 0, h=math.pi / 2)))
s.add_init_action(scn.TeleportAction("ego", scn.RelativeObjectPosition("lead", dx=3.0)))
engine.init(s)
print(engine.state("ego").x, engine.state("ego").y)  # 10.0 23.0
```

See `python/examples/positions.py` for the full example, including the reported
GeoPosition.

## Control ownership

Every entity is either **engine-controlled** (the default — the engine
integrates its motion) or **host-controlled** (the host simulator reports its
state each step). This is declared per entity in the scenario.

The engine drives only the entities it owns. A `TeleportAction` — or any
engine-driven action — aimed at a host-controlled entity is a **mode violation**:
it is reported (`InvalidControlMode`) and the entity's state is left untouched.
The host is the sole authority for its entities:

- push a full state between steps with `report_state(id, state)` (rejected with
  `InvalidControlMode` for an engine-controlled entity, `UnknownEntity` for an
  unknown one), or
- answer the gateway's `poll_state` each step.

A host-controlled entity is **never integrated** — a reported speed does not
advance its position — but scenario **conditions still observe** its reported
state. The in-step order is fixed (see [Determinism](determinism.md) and
ADR-0003): the clock advances, host states are polled, the storyboard is
evaluated, engine entities are integrated, and engine states are published.

## See also

- [Motion](motion.md) — `TeleportAction` alongside the speed and lateral actions.
- [The entity model](entities.md) — the full h/p/r `EntityState` pose and control
  mode.
- ADR-0017 (position resolution and control ownership), ADR-0003 (the simulator
  gateway).
