# Golden scenarios — v0.0.1 acceptance suite

Golden scenarios are the end-to-end acceptance layer of the v0.0.1 release
gate. Each one is a complete scenario exercising several pillars at once,
executed headlessly through `scena-run` (pillar P6) and validated against a
committed reference trace. They complement — never replace — the per-sprint
unit and conformance tests.

## Ground rules

- **Fixtures are ours.** Every scenario file, catalog, and OpenDRIVE map in
  this suite is authored from the ASAM specification texts (or adapted from
  the spec's own examples) — never taken from any other project's corpus
  (ADR-0002). Maps are small hand-written `.xodr` files.
- **Planned layout** (created by sprint p6-s4, extended by later sprints):
  - `tests/golden/scenarios/` — `.xosc` and `.osc` files, GS-numbered.
  - `tests/golden/catalogs/`, `tests/golden/maps/` — shared fixtures.
  - `tests/golden/traces/<platform>/` — committed reference traces.
  - `scripts/golden.py` — runs a scenario via `scena-run`, compares traces.
- **Two validation modes:**
  1. **Determinism (bit-exact):** re-running a scenario with the same step
     sequence must reproduce the committed reference trace **bit-identically**
     on every platform. This is the primary pass criterion; any diff is a
     release blocker.
  2. **Semantic assertions (tolerance):** each scenario declares checkpoints
     (e.g. "lateral offset of `cutin_vehicle` returns to lane center within
     ±0.05 m by t=12 s") verified by `scripts/golden.py` with per-checkpoint
     tolerances. These catch "deterministically wrong" behavior.
- **External cross-validation** against an established external reference
  player for OpenSCENARIO XML runs **as an external process only**, driven by
  the maintainer outside CI: the maintainer runs the same `.xosc` in that
  player, exports its trace, and compares with
  `scripts/golden.py --compare-external <trace> --tol <spec>`. The player is
  never a dependency, is never invoked by committed code, and is never named
  in committed files (results are recorded generically as "external
  reference player"). Divergences are triaged against the specification
  text, which is the only arbiter.
- **CI vs maintainer:** CI runs every golden scenario on all three platforms
  and enforces bit-identity + semantic assertions on each merge after p6-s4.
  The release gate additionally requires the maintainer to **hand-execute**
  the full suite on macOS, Linux, and Windows and record the results in
  `docs/roadmap/golden-scenarios-results.md` (template below).

## Standard execution profile

Unless a scenario says otherwise: fixed `dt = 0.01 s`, duration as declared,
trace = one row per step per entity with `t, entity, x, y, z, heading,
speed` (CSV; JSON available via `--trace-format json`), full IEEE-754 double
round-trip formatting so bit-identity is checkable from the text file.

Hand-execution procedure (identical on macOS/Linux/Windows, from the build
tree):

```sh
scena-run tests/golden/scenarios/<GS-file> \
  --dt 0.01 --duration <T> --trace out/<gs>.csv
python scripts/golden.py verify <gs> out/<gs>.csv   # bit-compare + checkpoints
```

## The scenarios

Feature keys reference the coverage matrices
(`docs/roadmap/coverage/osc-xml-coverage.md`,
`docs/roadmap/coverage/osc-dsl-coverage.md`).

### GS-1 — Cruise baseline

Single vehicle on a straight two-lane road; init teleport + init speed;
a `SimulationTimeCondition` at t=5 s triggers an absolute `SpeedAction` with
linear transition dynamics.

- **Exercises:** P1 (storyboard lifecycle, by-value condition), P2
  (longitudinal dynamics), P4 (loader, init actions), P6 (`scena-run`).
- **Pass:** bit-identical trace; speed reaches target within dynamics
  duration ±1 step; heading constant.
- **Role:** smoke test and determinism anchor; first scenario ported to
  every new platform.
- **Status:** the scenario body runs via the C++ API as of p5-s4 —
  `action_longitudinal_test` (functional: target reached, heading constant,
  forward motion) and `determinism_test` (bit-identity anchor across a
  non-uniform step sequence). Full `scena-run` execution with a committed
  reference trace is p6-s4.

### GS-2 — Cut-in

Ego cruises in the right lane; an adjacent vehicle ahead-left triggers on
`RelativeDistanceCondition` (longitudinal gap to ego below threshold) and
performs a `LaneChangeAction` (relative target lane, sinusoidal-shape
dynamics) into ego's lane, then an absolute `SpeedAction` slows it.

- **Exercises:** P1 (trigger rising edge), P2 (lateral dynamics), P3 (lane
  identity for relative lane target), P4, P5 (by-entity conditions, lateral
  action), P6.
- **Pass:** bit-identical trace; cutting vehicle's lane id changes exactly
  once; lateral offset settles within ±0.05 m of lane center by declared
  checkpoint; longitudinal gap at cut-in start within declared window.

### GS-3 — Overtake

Ego overtakes a slower lead: lane change left (triggered by
`TimeHeadwayCondition`), acceleration phase, lane change right back
(triggered by `RelativeDistanceCondition` freespace clearance), sequenced
via `StoryboardElementStateCondition` on the preceding events.

- **Exercises:** P1 (event sequencing via storyboard-state conditions,
  `maximumExecutionCount` defaults), P2, P5 (headway + freespace distance),
  P3 (lane queries), P4, P6.
- **Pass:** bit-identical trace; the three maneuver phases occur in order;
  ego ends in its original lane with target speed.

### GS-4 — Traffic-jam approach

Lead vehicle decelerates to near standstill; ego approaches, a
`TimeToCollisionCondition` arms a strong `SpeedAction` deceleration, then a
`LongitudinalDistanceAction` (distance keeping with freespace=true) holds
the gap; jam dissolves at a timed trigger and ego resumes.

- **Exercises:** P1 (event priority/overwrite between the braking and
  distance-keeping events), P2 (distance-keeping controller, performance
  limits), P5 (TTC, standstill, distance keeping), P4, P6.
- **Pass:** bit-identical trace; minimum gap never below declared floor; no
  collision (freespace distance > 0 throughout).

### GS-5 — Pedestrian crossing

A pedestrian follows a polyline trajectory across the road at a crosswalk;
ego brakes on `TimeToCollisionCondition` against the pedestrian and resumes
after a `ReachPositionCondition` confirms the pedestrian cleared the ego
lane corridor.

- **Exercises:** P2 (pedestrian entity, trajectory following, polyline), P5
  (TTC vs non-vehicle, reach position), P1, P4, P6.
- **Pass:** bit-identical trace; ego speed < 0.5 m/s while the pedestrian
  is inside the corridor checkpoint window; pedestrian path matches the
  polyline within interpolation tolerance.

### GS-6 — Emergency brake with standstill hold

Lead vehicle performs a step deceleration to a stop
(`AbsoluteSpeed` 0 with limited dynamics); ego reacts via TTC trigger;
both must reach and hold `StandStillCondition`; a delayed trigger
(`ConditionDelay` on the standstill condition) restarts the lead.

- **Exercises:** P1 (condition delay semantics, falling/rising edges), P2
  (deceleration limits), P5 (standstill, relative speed), P4, P6.
- **Pass:** bit-identical trace; both entities report speed == 0.0 exactly
  during the hold window; restart happens exactly `delay` seconds after the
  standstill edge.

### GS-7 — Trajectory fidelity slalom

An engine-controlled vehicle follows a committed trajectory that chains a
polyline segment, a clothoid segment, and a NURBS segment (position-timing
via `TimeReference`), on a straight road.

- **Exercises:** P2 (all three trajectory shapes, following mode =
  position), P5 (`FollowTrajectoryAction`), P4 (trajectory parsing), P6.
- **Pass:** bit-identical trace; sampled positions at declared parameter
  values match analytically computed curve points within 1e-9 m on the same
  platform; curvature continuity checkpoints at segment joins.
- **Role:** retires the trajectory numerical-fidelity risk (R3).

### GS-8 — Route through a junction

On a hand-authored four-way junction map, ego receives an `AssignRouteAction`
(waypoints across the junction) plus an `AcquirePositionAction` target; a
crossing vehicle yields via a `ReachPositionCondition` trigger.

- **Exercises:** P3 (routes, junction connectivity, road/lane ↔ world
  conversions), P5 (routing actions, reach position), P2, P4, P6.
- **Pass:** bit-identical trace; ego's road-id sequence equals the declared
  route; both vehicles clear the junction without freespace violation.

### GS-9 — Catalogs, parameters, and expressions

A parameterized cut-in variant: vehicle definitions from a catalog with
`ParameterAssignments`, trigger distance and target speed given as
expressions over declared parameters; loaded twice with different parameter
overrides in one maintainer session.

- **Exercises:** P4 (catalogs, parameter declarations, expression
  evaluation, structured diagnostics on a deliberately unresolvable
  reference variant), P1, P5, P6.
- **Pass:** bit-identical traces for both parameterizations against their
  own references; the two traces differ from each other exactly at the
  parameterized quantities; the malformed variant fails to load with the
  expected rule-ID-bearing diagnostic and nonzero exit code.

### GS-10 — Host-controlled ego round-trip

Ego is **host-controlled**: `scena-run --replay ego=<csv>` feeds a committed
ego state trace through `report_state()` each step. A scenario vehicle
reacts to the replayed ego (cut-out triggered by relative distance), while
conditions observe the host entity.

- **Exercises:** P6 (gateway/report-state round-trip in the CLI), P1/P5
  (conditions over host-controlled entities), P2, ADR-0003 ownership rules.
- **Pass:** bit-identical trace including the echoed ego states (host
  replay is part of the determinism equation); reacting vehicle's trigger
  fires at the declared step.

### GS-11 — Signalized intersection

A `TrafficSignalController` cycles a junction's signal phases via
`TrafficSignalControllerAction`/`TrafficSignalStateAction`; ego proceeds
only when a `TrafficSignalCondition` reports its approach signal green;
a cross vehicle flows during the orthogonal phase.

- **Exercises:** P5 (infrastructure actions, signal conditions), P1 (timed
  phase logic through the storyboard), P3 (junction map), P4, P6.
- **Pass:** bit-identical trace; ego standstill while red; ego crosses only
  during green window; phase timings match declared program.

### GS-12 — DSL cut-in (parity pair with GS-2)

The GS-2 scenario expressed in OpenSCENARIO DSL 2.x as a concrete scenario
(`osc.vehicle` actors, `do serial`/`parallel` composition, movement actions
with speed/position/lane modifiers), compiled through the DSL frontend to
the same Scenario IR.

- **Exercises:** P7 (parse, type-check against the standard library), P8
  (lowering, modifiers, composition), plus everything GS-2 touches.
- **Pass:** DSL trace is **bit-identical to the GS-2 XML trace** — the
  two-frontends-one-runtime claim, verified literally.
- **Role:** retires the execution-parity risk (R5); the release headline
  demo.

### GS-13 — DSL composition showcase

A DSL-only scenario: `do serial` of a `parallel` phase (two vehicles adjust
speeds concurrently) and a `one_of` phase executed with the documented
deterministic selection input (host-selected alternative, default first);
an `event`/`wait` dependency gates the final phase; `until` bounds a
sub-composition.

- **Exercises:** P7 (full syntax incl. events), P8 (serial/parallel/one_of
  semantics per DSL §7.6, modifier timing), P6.
- **Pass:** bit-identical trace per selected alternative (run twice with
  alternative 0 and 1, each against its own reference); phase boundaries at
  declared times; `wait` releases exactly on the emitted event.

### GS-14 — Determinism soak

A dense 10-minute scenario combining GS-2/GS-4/GS-5 patterns: six entities
(one host-controlled from replay), lane changes, distance keeping, a
trajectory follower, signals, catalogs and parameters. This is the workload
for the release gate's 24-hour ASan soak (looped execution) and the
cross-platform bit-identity check.

- **Exercises:** every pillar except P7/P8; long-horizon floating-point
  stability; scheduler behavior at scale.
- **Pass:** bit-identical trace across macOS/Linux/Windows for the full
  duration; zero sanitizer findings over the 24 h loop; memory high-water
  mark flat between iterations (leak guard).
- **Role:** retires the cross-platform determinism risk (R1) at release
  scale.

## Coverage of pillars

| Scenario | P1 | P2 | P3 | P4 | P5 | P6 | P7 | P8 |
|----------|----|----|----|----|----|----|----|----|
| GS-1  | ● | ● |   | ● | ● | ● |   |   |
| GS-2  | ● | ● | ● | ● | ● | ● |   |   |
| GS-3  | ● | ● | ● | ● | ● | ● |   |   |
| GS-4  | ● | ● |   | ● | ● | ● |   |   |
| GS-5  | ● | ● |   | ● | ● | ● |   |   |
| GS-6  | ● | ● |   | ● | ● | ● |   |   |
| GS-7  | ● | ● |   | ● | ● | ● |   |   |
| GS-8  | ● | ● | ● | ● | ● | ● |   |   |
| GS-9  | ● | ● |   | ● | ● | ● |   |   |
| GS-10 | ● | ● |   | ● | ● | ● |   |   |
| GS-11 | ● | ● | ● | ● | ● | ● |   |   |
| GS-12 | ● | ● | ● |   | ● | ● | ● | ● |
| GS-13 | ● | ● |   |   | ● | ● | ● | ● |
| GS-14 | ● | ● | ● | ● | ● | ● |   |   |

Every pillar is exercised by at least three scenarios (P3 by four; P7/P8 by
the two DSL scenarios plus the P7 `scena-check` gate on the full standard
library).

## Results template (`docs/roadmap/golden-scenarios-results.md`)

Created at release-gate time by the maintainer; one row per scenario per
platform: date, commit, platform/OS version, bit-identity (pass/fail),
semantic checkpoints (pass/fail), external cross-check (done/not-applicable,
max deviation), notes. All rows must be **pass** on all three platforms for
the gate.
