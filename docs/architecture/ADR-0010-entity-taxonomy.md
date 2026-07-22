# ADR-0010: Entity taxonomy — variant EntityObject, folded geometry, full pose

- **Status:** accepted
- **Date:** 2026-07-22

## Context

Through F0 and the P1/P5 sprints, a scenario entity was a flat record —
`{id, name, control_mode}` — with an optional `BoundingBox` added as a minimal
forward-pull in p5-s3 (ADR-0009) so the interaction conditions had geometry to
measure. p2-s1 (#15) delivers the real OpenSCENARIO entity model
(ASAM OpenSCENARIO XML 1.4.0 §7.2.2): a `ScenarioObject` is a name plus one
`EntityObject` — a **Vehicle**, **Pedestrian**, or **MiscObject** — each
carrying its own bounding box, categories, and (for a vehicle) performance
limits, axles, and properties. This ADR records how that taxonomy attaches to
the IR, how it crosses the C ABI and Python, and what is deliberately deferred.

p2-s1 sits on the critical path to the action chain: the longitudinal dynamics
sprint (p2-s2, #16) needs `Performance` limits, and Private actions I (p5-s4,
#32) needs both. Getting the data model right here unblocks that chain.

Three constraints frame the decisions:

- **Scena targets XML 1.0–1.3; the local spec copy is 1.4.0.** The category
  enumerations diverge (1.4 added and deprecated literals). The targeted
  version wins, handled explicitly.
- **The runtime consumes almost none of this yet.** Only the bounding box
  (freespace math) and, later, `Performance` (p2-s2) are read; everything else
  is faithful data carried for frontends and hosts.
- **The C-ABI structs are frozen, append-only.** Growing `scn_entity_state`
  and adding transparent structs must not reorder existing layout.

## Decision

### Variant `EntityObject`, with geometry folded in

`ir::Entity` becomes the `ScenarioObject`:
`{id, name, control_mode, std::optional<EntityObject>}` where
`EntityObject = std::variant<Vehicle, Pedestrian, MiscObject>`. Each concrete
object owns its `BoundingBox` (and, for `Vehicle`, `Performance`/`Axles`). The
standalone `Entity::bounding_box` member from p5-s3 is **removed** — geometry
lives in one place, inside the object.

Three free helpers read across the variant so callers never `std::visit`
themselves: `object_type_of(entity)` → `optional<ObjectType>`,
`bounding_box_of(entity)` → `optional<BoundingBox>` (the single geometry source
the runtime freespace path now reads), and `performance_of(entity)` →
`const Performance*` (nullptr for non-vehicles).

`object` is **optional**: an entity with no object is a valid *unclassified
participant* (identity + control mode only), preserving the F0 minimal entity
so existing engine/fixture entities compile unchanged.

*Rejected:* three parallel `std::optional<Vehicle>/<Pedestrian>/<MiscObject>`
members plus a kept `Entity::bounding_box`. It keeps geometry in two places
(box on the entity *and* inside each object) and admits nonsensical states (a
vehicle and a pedestrian at once). The variant makes "exactly one object"
unrepresentable-otherwise.

### Category enums carry the full 1.4.0 set

`VehicleCategory`/`PedestrianCategory`/`MiscObjectCategory`/`Role` enumerate the
complete 1.4.0 literals, so any 1.0–1.3 document (a subset) always maps. Post-1.3
additions and 1.3→1.4 deprecations (e.g. `Motorbike`→`Motorcycle`,
`Truck`→`HeavyTruck`/`Semitractor`, `Role::Fire`→`FireBrigade`) are annotated per
enumerator. Enumerator order follows the spec document order and is mirrored 1:1
by the C-ABI enums (compile-time `static_assert`s pin the correspondence).
`external` (`ExternalObjectReference`) is out of scope — Scena executes only the
three inline object types.

### Full-pose `EntityState`

`EntityState` gains `pitch` and `roll` (radians), completing the Z-up h/p/r
pose (§Orientation). They are **appended after `speed`** so existing positional
`EntityState{x, y, z, heading, speed}` initializers keep compiling. The
straight-line integrator leaves `z`/`pitch`/`roll` untouched (ground-plane
model); a host may report any pose. Freespace stays yaw-only (the 2D OBB kernel
is planar). The mirrored `scn_entity_state` gains the two fields as a transparent
append.

### Validation

`Engine::init` rejects a `Vehicle`'s `Performance` with any negative, NaN, or
non-finite `maxSpeed`/`maxAcceleration`/`maxDeceleration` or rate limit
(ranges `[0..inf[`, §Performance) as a `ValidationError`. A zero
`maxDeceleration` is spec-permitted (degenerate; the p2-s2 controller handles
it) and is not rejected. §Performance defines no `asam.net` checker rule id, so
the diagnostic cites the section only — no invented id. The bounding-box check
carries over from p5-s3.

### C ABI and Python

Typed builders `scn_engine_add_vehicle/_pedestrian/_misc_object` (bounding box
required; performance required for a vehicle; category range-checked as
unsigned) plus read-back accessors
`scn_engine_entity_object_type/_bounding_box/_performance` reading the authored
scenario (valid before and after init). Transparent, append-only
`scn_bounding_box`/`scn_performance` structs; an optional performance rate is
signalled by a negative sentinel. Python binds the three concrete types, the
enums, and the value types; `Entity` takes an optional `object` and exposes
`object_type`/`bounding_box`/`performance` as read-only derived views. The
spec's `none` role/category is exposed as `NONE` in Python (the keyword `None`
cannot be an attribute name). No new `Status` enumerator.

## Consequences

- The interaction conditions and the C-ABI freespace path now read geometry via
  `bounding_box_of`; the p5-s3 sites that set the old member were migrated.
- p2-s2 (#16) can build the longitudinal controller directly on `Performance`;
  p5-s4 (#32) is unblocked once #16 lands.
- **Deferred:** `EntitySelection` / `ByType` / `ByObjectType` selection
  homogeneity → p4-s4; wiring `CollisionCondition`'s ByObjectType target (the
  p5-s3 deferral) stays a condition follow-up; `Trailer`/hitch, `model3d`, and
  catalog `parameterDeclarations` are out of the v0.0.1 entity scope; XML
  lowering of the entity model → P4.
- Growing `scn_entity_state` is an ABI addition (release-note material): a host
  compiled against the older 5-field struct must be recompiled.

## Alternatives considered

- **Keep `Entity::bounding_box`, add typed detail alongside** — rejected as
  above (duplicated geometry, representable nonsense).
- **A single flattened struct with an `ObjectType` tag and all fields inline** —
  rejected: pedestrians and misc objects would carry meaningless
  performance/axle fields, and the tag could disagree with which fields are set.
- **Defer the pose extension** — rejected: the issue calls for it, hosts already
  need to report pitch/roll, and appending now avoids a second ABI break later.
