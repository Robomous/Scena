# ADR-0008: Entity-kinematics observation

- **Status:** accepted
- **Date:** 2026-07-21

## Context

The by-entity conditions (ASAM OpenSCENARIO XML 1.4.0 §7.6.5.1) observe
entities' kinematics: speed, relative speed, acceleration, stand-still
duration, traveled distance, and reached position. p5-s1 built the
`EvaluationContext` seam and ADR-0007 **pre-authorized an entity-kinematics
facet** as a new default-absent virtual; this ADR records how that facet is
shaped and fed.

Two constraints frame the decisions:

- **`EntityState` is a scalar model** (`{x, y, z, heading, speed}`): speed is
  longitudinal along `heading`, with no velocity vector, no acceleration, and
  no history. The measurements must be defined from that and generalize when a
  velocity vector lands.
- **The two hard dependencies are unmerged.** The PositionResolver (p2-s4,
  #18) and road-coordinate plumbing (p3-s4, #23) do not exist yet, so
  road-relative measurement and the full Position variants cannot be wired.

## Decision

### The facet

One coarse virtual on `EvaluationContext`:

```cpp
struct EntityKinematics {
    EntityState state;                    // snapshot the evaluation observes
    std::optional<double> acceleration;   // absent until two samples
    double traveled_distance = 0.0;       // cumulative path length since init
    double standstill_seconds = 0.0;      // contiguous time at speed == 0.0
};
virtual std::optional<EntityKinematics> entity_kinematics(std::string_view id) const;
```

One method, not four: a single `Scheduler::BoundContext` passthrough, a
two-entity metric (p5-s3) calls it twice, and a test overrides one method.
Absent⇒false at two grains: an outer `nullopt` (unknown entity or a
facet-less context) makes the per-entity expression false; a nested-absent
`acceleration` makes only `AccelerationCondition` false while `SpeedCondition`
on the same entity still works.

### Derived-observation update (engine phase 2b)

Derived quantities are refreshed once per `step(dt)`, in a new sub-phase 2b
between the host poll and storyboard evaluation, iterating entities in
`std::map` order for both control modes. At evaluation *k* they are pure
functions of the observed snapshots (S₀ seeded at init) and the dt sequence:

- `acceleration_k = (speed_k − speed_{k−1}) / dt_k`; absent until two samples.
- `traveled_distance` += Euclidean displacement between consecutive observed
  samples — **path length**, not straight-line displacement.
- `standstill_seconds` += dt while `speed_k == 0.0`, else resets to 0.

Deliberate consequences:

- **Init seeding.** After the init actions, `prev_sample = state`. So an
  init-action speed is the baseline (no phantom acceleration/distance at
  t = 0), acceleration is absent at t = 0, and a stationary entity's
  stand-still holds a `duration = 0` immediately.
- **dt == 0 freeze.** Phase 2b is skipped: no accumulator update and
  `prev_sample` is not overwritten, avoiding a `0/0` NaN (which would poison
  `notEqualTo` into true). The live `state` is still visible.
- **report_state sampling.** Out-of-band reports write only `state`; the
  accumulators observe it at the next step's 2b. Between two steps only the
  last report is sampled — net displacement, deterministic.
- The existing one-step observation lag for engine-controlled entities
  (integration is phase 4, after 2b) is intentional and unchanged; sub-phase
  2b is an addition to the documented step order, not a reordering.

### Spec-silent calls

- **StandStill = exact `speed == 0.0`** (true for `-0.0`). The spec gives no
  threshold, so none is invented.
- **TraveledDistance = path length from init** for all entities, regardless of
  when a condition arms (an arm-time start would make results depend on arming
  order).
- **ReachPosition = 2D horizontal circle.** The spec calls `tolerance` the
  "radius of tolerance circle", so `z` is ignored; distance is
  `sqrt(dx²+dy²)` (never `std::hypot`, which is not IEEE-exact-mandated).

### Scalar-model projections

- Total speed/acceleration is `|value|`; directional longitudinal is the
  signed value, lateral/vertical are exactly `0.0`.
- RelativeSpeed without a direction is the spec's signed difference of total
  speeds `|s_trig| − |s_ref|`; with a direction the relative velocity is
  projected in the **triggering** entity's frame via `det_sincos` (the only
  trigonometry — mandatory for bit-identity; headings beyond
  `kDetTrigMaxAbsInput` yield NaN ⇒ condition false).

### Reduction and status

- The any/all reduction happens **inside** `ByEntityCondition::evaluate`
  (no short-circuit), before the scheduler's edge/delay machinery — exactly
  §7.6.5.1.
- **`Status::DeprecatedFeature`** is appended (ABI addition, synced to the C
  ABI and Python) for the ReachPosition deprecation warning; p5-s6 reuses it.

### Deferred

- `EntitySelection` references in `TriggeringEntities` (no selection IR yet,
  p4-s4).
- Road-coordinate measurement and ReachPosition against the full Position
  variants — a follow-up when #18/#23 land (noted on issue #23).
- XML lowering (P4).

## Consequences

Adding the p5-s3 two-entity metrics (distance, time-headway, TTC) needs no new
facet — they call `entity_kinematics` twice. A velocity vector, when it lands,
replaces the scalar projections behind the same condition API. The step-order
documentation on `Engine::step` gains sub-phase 2b as its record of the
observation contract.
