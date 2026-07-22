# ADR-0009: Interaction metrics — bounding-box forward-pull and planar freespace geometry

- **Status:** accepted
- **Date:** 2026-07-22

## Context

The interaction conditions (ASAM OpenSCENARIO XML 1.4.0 §7.6.5.1, distance
definitions §6.4) measure between two entities, or an entity and a position:
**Distance, RelativeDistance, TimeHeadway, TimeToCollision, Collision,
EndOfRoad, Offroad, RelativeClearance**. p5-s1 built the `EvaluationContext`
seam (ADR-0007); p5-s2 added the entity-kinematics facet and
**pre-authorized two-entity metrics calling it twice** (ADR-0008). This ADR
records how the interaction subset is measured while two dependencies are
still unmerged.

Three constraints frame the decisions:

- **No geometry exists anywhere.** The full entity taxonomy (p2-s1, #15) is
  unlanded, but freespace distance and collision need a bounding box.
- **No road network exists.** `IRoadQuery` (p3-s4, #23) is an unimplemented
  interface, so road/lane/trajectory coordinate systems and the
  road-topology predicates cannot be evaluated.
- **`EntityState` is a scalar model** (`{x, y, z, heading, speed}`): yaw only,
  speed longitudinal along heading, no velocity vector.

## Decision

### Minimal bounding-box forward-pull

A minimal `ir::BoundingBox` (center offset in the entity frame plus
length/width/height) lands as an optional field on `ir::Entity`, threaded
through `EntityKinematics` so the conditions read it via the existing facet.
It is copied once into the engine's entity record at init and is immutable at
runtime. p2-s1 later adds categories, `Performance`, and axles on top of this
same box. Init validation rejects NaN/negative dimensions and a NaN center; a
zero-size box is a valid degenerate point.

### 2D ground-plane OBB kernel

`scena/runtime/obb2.h` reduces an entity to a heading-rotated rectangle
(`Obb2`). Because `EntityState` carries yaw only, the spec box projects
*exactly* to that rectangle; `z`/`height` are stored but never enter the
planar math. The caller performs the **only** trigonometry — once, through
`det_sincos` — and every kernel routine is pure multiply/add/compare plus
`sqrt`/`fabs`/`fmin`/`fmax` (never `std::hypot`), fixed operand order, so
outputs are bit-identical across platforms and raw distances can be
hex-pinned. Touching boxes count as intersecting (freespace distance 0,
§6.4.7.2, strict `>` separation test).

### Distance semantics matrix (§6.4)

`measure_distance` resolves an effective coordinate system and
relative-distance type, then:

| Effective CS | RDT | `freespace=false` (reference point) | `freespace=true` |
|---|---|---|---|
| Entity / World | Euclidian (default) | 3D `sqrt(dx²+dy²+dz²)` (§6.4.3, CS-independent) | box↔box / point↔box distance |
| Entity | Longitudinal / Lateral | `\|Δp·û\|` on the triggering body axis via `det_sincos` (§6.4.4) | axis-interval gap on the same axis |
| World | Longitudinal / Lateral | `\|Δx\|` / `\|Δy\|` on the world axes | axis-interval gap on world X/Y |
| any | CartesianDistance | = Euclidian + DeprecatedFeature warning | same |
| Lane / Road / Trajectory | any | **deterministic false** + init UnsupportedFeature warning | same |

### Road-dependent scope lands as classes, evaluates false

All eight classes land with validation and Python bindings. **EndOfRoad,
Offroad, RelativeClearance, and any road/lane/trajectory coordinate system**
evaluate to a deterministic false (ADR-0007 absent-facet rule) with an
init-time **UnsupportedFeature warning** citing spec sections
(§7.6.5.1/§6.4.5 — no `asam.net:` rule id exists for a road prerequisite, so
none is invented, p5-s1 precedent). Real road math is deferred to p3-s4
(noted on #23). XML lowering is deferred to P4 (standing precedent).

### Spec-silent / structural calls

- **Collision target = EntityRef only.** `ir::Entity` has no category, so a
  `ByObjectType` target would be permanently false today and would silently
  change behavior when the p2-s1 taxonomy lands. Deferred to p2-s1 (noted on
  #15).
- **TimeToCollision closing speed.** Planar velocity is `speed·(cos_h, sin_h)`
  per entity (`det_sincos`; a position target is stationary). Euclidian: the
  line-of-sight rate `(r·(vA−vB))/d_ref`, z relative velocity ≡ 0.
  Longitudinal/Lateral: the component reducing the signed axis separation, the
  axis frozen at the evaluation instant (no yaw-rate term). Closing speed ≤ 0
  ⇒ false (spec: negative ⇒ false; zero folded in — collision unreachable,
  avoids 0-division). Coincident reference points ⇒ false. Freespace TTC uses
  the freespace distance ÷ the reference-point closing speed (the spec ties
  the relative speed to the coordinate system, not to the gap).
- **TimeHeadway** divides the distance by the **triggering** entity's raw
  signed speed only (reference assumed leading); `s_trig ≤ 0` ⇒ false
  (a stopped/reversing follower never covers the gap; spec silent).
- **Freespace with an absent bounding box** ⇒ per-entity false at runtime with
  no diagnostic (geometry is optional until p2-s1; a warning would be noise).
- **alongRoute mapping** (Distance/TimeHeadway/TimeToCollision only;
  RelativeDistance has none): stored `optional<bool>`; authored ⇒
  DeprecatedFeature warning. If neither coordinateSystem nor
  relativeDistanceType is authored and `alongRoute == true`, the effective CS
  becomes Road ⇒ deferred false + UnsupportedFeature warning. Otherwise
  defaults apply (spec: alongRoute ignored iff CS or RDT is set).
- **cartesianDistance** (deprecated literal) ⇒ DeprecatedFeature warning,
  treated as euclidianDistance.

### No status / ABI churn

Every diagnostic reuses an existing `Status`
(`ValidationError`/`SemanticError`/`UnsupportedFeature`/`DeprecatedFeature`),
so there is **no C ABI or Python `Status` addition**. The conditions are
reachable from C++/Python only; no C-ABI condition builders (standing design).
Negative/NaN *distances* cite the
`asam.net:xosc:1.1.0:data_type.distances_are_not_negative` rule id; the
*time* values (headway, TTC) cite the section only, since the standard names
no rule for their range.

## Consequences

The interaction metrics are pure instantaneous functions of two
`entity_kinematics` reads plus the immutable box — no new engine step phase, no
derived state, no pair caching. When a velocity vector and the p2-s1 taxonomy
land, the scalar-velocity closing-speed model and the minimal box are replaced
behind the same condition API. When `IRoadQuery` lands (p3-s4), the
road/lane/trajectory rows of the matrix and the three road-topology predicates
gain real implementations, replacing the deterministic-false stubs; the
UnsupportedFeature warnings disappear at that point.
