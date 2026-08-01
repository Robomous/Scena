# ADR-0019: Road-based execution semantics (p3-s4 subset)

## Status

Accepted (p3-s4, #23).

## Context

p3-s1..s3 delivered the frozen `IRoadQuery` v1 and an OpenDRIVE backend.
p3-s4 wires roads into the execution path: the road-family Position
variants, road-coordinate distance measurement for the P5 conditions and
the distance-keeping actions, lane-change target resolution (already
road-backed since p2-s3), and the road-network predicates
(EndOfRoadCondition, OffroadCondition). The standard leaves several
evaluation methods open; this ADR fixes the v0.0.1 subset so every
behavior is deterministic and every limit is reported, never silent.

## Decision

- **Flat world.** Elevation is scoped out for v0.0.1 (P3 scope): roads live
  in the z = 0 plane, `to_world_position` returns z = 0, and a point off
  the plane (beyond numeric noise) is off-road. This keeps the two
  conversions exact mutual inverses. A point past the road's longitudinal
  extent is likewise off-road: the backend rejects projections whose
  planar distance exceeds the lateral offset by more than the projection
  tolerance.
- **Orientation base.** The orientation base of every road-family position
  is the road s-axis tangent at the target (`road_heading`), turned by pi
  on route spans traversed against the s-axis; pitch/roll stay 0 (the
  standard assigns them to the road surface — the z = 0 plane here).
- **Same-road distances.** Road/lane-CS distance measurements answer only
  when both endpoints project onto the same road; cross-road measurements
  need route context and stay deferred (reported through the conditions'
  deterministic-false / the actions' completes-immediately paths).
  Euclidean distance is CS-independent (§6.4.3) and measures regardless.
- **Freespace in road coordinates.** Bounding-box corners are linearized
  into the road tangent frame at the entity's reference point (exact on
  straight roads, first-order on curved ones); the gap is the s-/t-interval
  separation with the same sign conventions as the entity/world-CS gaps.
- **Road-predicate clocks.** The engine accumulates per-entity
  `end_of_road_seconds` / `offroad_seconds` each step (the
  standstill_seconds pattern): at-end means the bounding-box front reaches
  the road boundary in the entity's own driving direction (tangent-aligned
  forward, opposed backward) within the projection tolerance (1e-6 m);
  off-road means the reference point projects onto no road. The conditions
  compare the clocks against their `duration`.
- **Subset deferrals, all reported:** `dsLane` on RelativeLanePosition
  (distance along the lane centre line), deltas that leave the reference
  entity's road (no continuation onto linked roads), the trajectory
  coordinate system for distances, RelativeClearanceCondition, and
  catalog route references (P4).

## Consequences

- The P5 deferrals from p5-s2/p5-s3 (road distance metrics, road-network
  predicates, lane-relative resolution) are paid off within the declared
  subset; conditions and actions degrade exactly as before when no road
  network is attached.
- `ir::RoutePosition` now carries its §RoutePosition data model (route +
  in-route coordinate). It is not exposed through the C ABI or the Python
  bindings (only WorldPosition is), so the embedding surface is unchanged.
- The `EvaluationContext` gains a road-network facet (default nullptr),
  and `EntityKinematics` two clock fields — both appended, source
  compatible.
