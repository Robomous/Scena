# ADR-0017: Position resolution and control ownership

- **Status:** accepted
- **Date:** 2026-07-23

## Context

Sprint p2-s4 (#18, "Position resolution, teleport & host round-trip") has two
goals: **one resolver for all position types**, and **airtight control-ownership
semantics**.

Until this sprint the IR had exactly one position type — `ir::WorldPosition`, a
flat `{x, y, z}` — reused as a stand-in wherever a position target was needed
(`TeleportAction`, `AcquirePositionAction`, `AddEntityAction`, the position
conditions, route waypoints, trajectory vertices). There was no orientation, no
`PositionResolver`, and nine of the ten ASAM OpenSCENARIO §6.3.8 position
variants were missing. `TeleportAction` wrote only `x/y/z` and, on a
host-controlled entity, its write was silently overwritten by the next poll.

The standard (§6.3.8) defines ten position variants. §5 states that the
**corrected (≥1.3)** position/orientation calculations "are not necessary to
implement differently" for older versions — the corrected math applies uniformly
to all input versions. Some variants are self-contained; the rest need a road
network, a route, a trajectory, or a geodetic datum that Scena does not yet
consume (P3/p2-s5/post-v0.0.1).

## Decision

### 1. `ir::Position` and `ir::Orientation`

Model the ten §6.3.8 variants as a `std::variant` `ir::Position`
(`core/include/scena/ir/position.h`), in the spec's `xsd:choice` order.
`WorldPosition` gains **append-only** `h/p/r` orientation fields (so `{x, y, z}`
initializers still compile). Add `ir::Orientation` (h/p/r + a `ReferenceContext`)
and promote `ReferenceContext` to this header as the single canonical definition
(`trajectory.h` includes it).

`ir::Position` replaces `WorldPosition` in the three position-**target actions**
only — `TeleportAction`, `AcquirePositionAction`, `AddEntityAction`. A
`WorldPosition` converts to `Position` implicitly, so existing call sites and the
C ABI/bindings keep compiling. The conditions, route waypoints and trajectory
vertices keep their `WorldPosition` field and their existing deferral notes;
generalizing those is deferred to when the resolver is wired into each path.

### 2. `runtime::PositionResolver`

A **standalone** resolver (`core/include/scena/runtime/position_resolver.h`)
maps any `ir::Position` to a world `Pose` (position + heading/pitch/roll) or a
`PositionResolution` carrying a non-Ok `Status`, a deterministic message, and —
for GeoPosition — the ASAM rule id. It depends only on a pose-lookup callback
(entity id → current `EntityState`), not on the engine, so every variant and
every orientation-composition case is unit-testable without a scenario
(`position_test.cpp`).

Coverage this sprint:
- **Self-contained, resolve fully:** `WorldPosition` (taken directly);
  `RelativeWorldPosition` (world-axis deltas from the reference entity — *not*
  rotated); `RelativeObjectPosition` (deltas in the reference entity's local
  frame — rotated by its heading via `det_sincos`, so bit-identical, ADR-0006).
- **Road family** (`RoadPosition`, `RelativeRoadPosition`, `LanePosition`,
  `RelativeLanePosition`, `RoutePosition`): report `UnsupportedFeature`. They
  need a road network *and* the s-axis tangent at the target for relative
  orientation — neither exists until the road backend (p3-s4).
- **`TrajectoryPosition`:** `UnsupportedFeature` until trajectory shapes land
  (p2-s5).
- **`GeoPosition`:** `UnsupportedFeature` with rule
  `asam.net:xosc:1.1.0:positioning.geodetic_datum_defined` — it needs the road
  network's geodetic datum (post-v0.0.1).

Every variant either resolves or is reported — **none is silently wrong**, the
sprint's hard exit criterion.

Orientation composes per §Orientation: a missing orientation copies the
reference frame's; an `Absolute` one replaces it in the world frame (the
reference orientation is ignored); a `Relative` one is an additive
counter-clockwise shift on top of it.

**Yaw-only rotation.** The straight-line runtime keeps `pitch`/`roll` at 0
(`entity_state.h`), so `RelativeObjectPosition`'s local-frame rotation is a yaw
about +Z — exact for every state the runtime produces. The full Z→Y→X frame
rotation lands when the runtime carries a non-zero pitch/roll.

### 3. Control ownership is airtight

The engine drives only the entities it owns. A `TeleportAction` — or any
engine-driven private action — aimed at a **HostControlled** entity is a **mode
violation**: it is reported (`InvalidControlMode`, warn-once) and leaves the
entity's state untouched. The host is the sole authority for its entities'
state, via `report_state()` or the gateway poll; scenario conditions still
observe that reported state (the observation facet is fed for both control modes,
ADR-0008). The in-step order is re-verified against ADR-0003: poll host states →
evaluate storyboard → integrate engine entities → publish engine states.

### 4. C ABI and Python

The C ABI is append-only: `scn_engine_add_teleport_action` is unchanged; two new
builders (`_oriented`, `_relative`) reach the self-contained variants. The
road/lane/route/geo/trajectory variants stay kernel-only in C (exposing them
would only surface `UnsupportedFeature`). Python binds `Orientation`, the two
relative variants and `GeoPosition` (to demonstrate the diagnostic), extends
`WorldPosition` with h/p/r, and takes `ir::Position` in the three actions.

## Consequences

- One resolver is the single place §6.3.8 positions become world poses; the XML
  frontend (P4) and the DSL frontend (P7) both lower into `ir::Position` and get
  it for free.
- Road/lane/route/geo/trajectory teleports **parse and validate** but are
  reported at apply time. p3-s4 wires the road backend (and the s-axis tangent
  query) into the resolver's road-family branches; p2-s5 wires TrajectoryPosition.
- Control ownership is now enforced, not merely documented: a scenario cannot
  move an entity the host owns.
- Extending `WorldPosition` and taking `ir::Position` in the actions is
  source-compatible for every existing caller (implicit conversion) and
  append-only for the mirrored C-ABI struct.

## Alternatives considered

- **Resolve inside the engine, keyed off entity records.** Rejected: a standalone
  resolver is unit-testable per variant without booting a scenario, and keeps the
  §6.3.8 math in one auditable place.
- **Generalize every `WorldPosition` site now** (conditions, routes,
  trajectories). Rejected: it balloons the diff into p5-s2/s3 and p2-s5 territory
  without serving an exit criterion. The actions are the resolver's natural first
  consumers; the rest follow when their paths need non-world targets.
- **Apply a host-entity teleport and let the next poll overwrite it.** Rejected:
  a momentary engine-driven move of a host-owned entity reads as "silently
  wrong" and violates ADR-0003's ownership split. Reporting the violation is the
  airtight semantics the sprint calls for.
- **Full Z→Y→X frame rotation for RelativeObjectPosition now.** Deferred: the
  runtime never produces a non-zero pitch/roll, so yaw-only is exact for every
  reachable state; the extra math would be untestable dead code today.
