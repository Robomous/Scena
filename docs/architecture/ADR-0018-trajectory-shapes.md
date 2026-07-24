# ADR-0018: Trajectory shapes — clothoid & NURBS numerics

- **Status:** accepted
- **Date:** 2026-07-23

## Context

Sprint p2-s5 (#19, "Trajectory following: polyline, clothoid, NURBS") is the
last P2 sprint and it retires **risk R3 — trajectory numerical fidelity**
(clothoid Fresnel evaluation, NURBS basis stability, arc-length
parameterisation).

Before this sprint `ir::Trajectory` modelled only the **polyline** shape (a bare
vertex vector) and `FollowTrajectoryAction` was wired for it (p5-s5, ADR-0014):
both `timeReference` modes, `initialDistanceOffset`, the §6.9.1–§6.9.3
start-state cases, exact completion at the final vertex. The `Clothoid`,
`ClothoidSpline` and `NURBS` shapes of §6.9, and the resolution of
`TrajectoryPosition` (deferred from p2-s4), were outstanding.

The standard (§6.9) defines the trajectory `Shape` as an `xsd:choice` of
`Polyline | Clothoid | ClothoidSpline | Nurbs`. A clothoid is an Euler spiral
whose curvature changes linearly with arc length (`curvaturePrime = 0` ⇒ a
circular arc, both zero ⇒ a line). A NURBS is a rational B-spline of a given
`order` over a control-point and knot vector, able to express conics exactly.

## Decision

### 1. `Shape` variant in the IR
`ir::Trajectory` carries `std::variant<Polyline, Clothoid, Nurbs>`. A
backward-compatible `Trajectory{name, closed, vertices}` constructor and a
`vertices()` accessor keep the many polyline construction sites concise.
`ClothoidSpline`, the 1.4 `Motion` element and the polyline `Interpolation`
element are **out of scope** for v0.0.1 (a single-segment clothoid covers the
need; the coverage matrix records this).

### 2. A standalone arc-length evaluator
`runtime::TrajectoryEvaluator` maps arc length → world pose, mirroring
`PositionResolver`: no engine dependency, so the numerics are unit-tested
against analytic ground truth (circles, straights). It is the single source of
trajectory geometry — the follower and `TrajectoryPosition` both go through it.

- **Polyline** — exact per-segment linear interpolation; heading is the segment
  tangent via `det_atan2`.
- **Clothoid** — the straight-line and circular-arc degenerate cases are closed
  form (bit-exact to the analytic reference). A general spiral integrates the
  Fresnel-type integrand `cos/sin(theta(u))` with a **fixed-step composite
  Simpson quadrature** built on `det_sincos`: a cumulative table at construction
  (node spacing `kClothoidStep = 1 cm`) plus a residual panel per query, so cost
  is O(1) per step and the accumulated error stays well under 1e-9 m for the
  curvatures and lengths of realistic drivable spirals. Because `det_sincos` is
  an ordinary identifier (not a bare `sin(`/`cos(` token) the evaluator lives in
  a normal runtime file — **no change to `detmath.cpp`** was needed.
- **NURBS** — the homogeneous control points are evaluated by the **rational de
  Boor recursion** (IEEE operations only, so bit-identical by construction, no
  detmath) and divided back to the point. Tangents come from the exact rational
  derivative `C' = (A'w − A w') / w²`. Arc length is a fixed-resolution
  cumulative table with a linear `s → u` inversion (error O(1/N²)); the evaluated
  point is always exactly on the curve regardless of the table.

### 3. Following through the evaluator
For clothoid/NURBS the follower stores the evaluator and steps it
(`drive_trajectory_eval`); the **polyline keeps its own sampled path** so its
determinism traces do not churn. Time-free mode advances by the entity's own
speed (§6.9.1 teleport to the start); timing mode maps the clock **linearly**
across the shape's start/end times (clothoid `start/stopTime`; NURBS first/last
control-point times) to an arc length, owning the longitudinal domain. Init
validation constructs the evaluator to surface shape diagnostics, checks
`initialDistanceOffset` against the true arc length, and requires start/end
times when timing is requested.

### 4. TrajectoryPosition resolution
`TrajectoryPosition` gains a (forward-declared) `shared_ptr<Trajectory>` to
break the `position.h` ↔ `trajectory.h` include cycle. The resolver evaluates
the trajectory at arc length `s` and steps the lateral offset `t` along the
tangent left-normal (`det_sincos`). Road-projected trajectory lateral distance
(§6.4.6) still awaits the road backend (p3-s4).

### 5. `followingMode = follow` stays accepted-as-position
A true steering controller (steer toward the path within performance limits,
§6.9.1–§6.9.3 `follow`) is a large, separate concern; GS-7 and the R3 goal need
only position mode. `follow` remains accepted-and-executed-as-`position` with an
UnsupportedFeature warning, the ADR-0011/ADR-0014 precedent, and a follow-up
issue tracks the steering controller.

## Consequences

- GS-7 (the trajectory-fidelity slalom) runs in its **programmatic form** as a
  determinism anchor (`make_clothoid_nurbs_scenario`), bit-identical across two
  engines and hex-pinned for the 3-OS matrix — **risk R3 is retired**. The
  XML-parsed form still awaits P4, as with GS-4.
- Analytic-reference tests hit the evaluator directly: a quarter circle as a
  constant-curvature clothoid and as a quadratic NURBS is on the circle within
  1e-9, the general spiral matches an independent 200k-panel libm reference to
  1e-8, and samples are hex-pinned.
- The C ABI gains two append-only builders
  (`scn_engine_add_follow_{clothoid,nurbs}_trajectory_action`) and the Python
  bindings gain the `Polyline`/`Clothoid`/`ControlPoint`/`Nurbs` types; existing
  signatures are unchanged.

## Alternatives considered

- **Add Fresnel `C(t)/S(t)` primitives to `detmath`.** More general and O(1) per
  query, but it needs a vendored rational/Taylor approximation with its own
  coefficient generator. The deterministic quadrature on `det_sincos` is simpler
  to verify and bound, hits the same 1e-9 fidelity, and keeps `detmath` focused
  on the trig primitives. Rejected for v0.0.1.
- **Pre-sample clothoid/NURBS into a dense polyline** and reuse the existing
  follower verbatim. Simpler wiring, but it fails the 1e-9 analytic check at
  arbitrary arc lengths and blurs the curvature-continuity checkpoints. Rejected
  in favour of the exact evaluator.
