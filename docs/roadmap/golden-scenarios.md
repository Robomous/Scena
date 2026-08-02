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
- **Layout** (live as of p6-s4; extended by later sprints):
  - `tests/golden/scenarios/` — `.xosc` and `.osc` files, GS-numbered.
  - `tests/golden/catalogs/`, `tests/golden/maps/` — shared fixtures. The maps
    are hand-authored `.xodr` files; a scenario names its own in
    `RoadNetwork/LogicFile`, resolved relative to the scenario file.
  - `tests/golden/traces/reference/` — the committed reference traces.
    **One shared set, not one per platform**: determinism is bit-identical
    across platforms, so a per-platform reference would be a place for a
    divergence to hide. Comparing every platform against the same bytes makes
    the golden CI job a cross-platform determinism check too.
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
- **CI vs maintainer:** the `golden` CI job runs every golden scenario on all
  three platforms and enforces bit-identity + semantic assertions on each
  merge (live as of p6-s4).
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
python scripts/golden.py check-all          # the whole suite, or:
python scripts/golden.py run gs1            # one scenario -> build/golden-out/gs1.csv
python scripts/golden.py verify gs1 build/golden-out/gs1.csv
```

`scripts/golden.py list` prints the suite with what each scenario exercises.
Durations, step sizes and checkpoints live in the script, so the command line
above is the same for every scenario. See
[`docs/user-guide/scena-run.md`](../user-guide/scena-run.md) for the CLI
itself.

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
- **Status:** **live** as of p6-s4 —
  `tests/golden/scenarios/gs1-cruise-baseline.xosc`, run by the `golden` CI job
  on all three platforms against a committed reference trace, with four
  checkpoints (cruise speed before the trigger, target reached after the ramp,
  heading and lateral position constant). Also covered at unit level by
  `action_longitudinal_test` and `determinism_test`.

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
- **Status:** runs via the C++ API as of **p2-s3**
  (`make_gs2_scenario` in `core/tests/determinism_test.cpp`, with a
  bit-identity test and a hex-pinned final trace; the Python flavour is
  `python/examples/lane_change.py`). **Live in the golden suite** as of p6-s4 —
  `gs2-cut-in.xosc`, with checkpoints on the lateral offset before and after the
  lane change and on the post-cut-in speed. Lane spacing is the flat-world
  default lane width, so the "lane id changes exactly once" criterion still
  awaits a road backend in the fixture (#23).

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
- **Status:** **live** —  `gs3-overtake.xosc` on `maps/overtake.xodr` (two
  same-direction driving lanes), with eight checkpoints: the outer-lane start,
  the inner-lane centre after phase 1 (a value only a road backend can supply),
  the overtaking speed after phase 2, and the return to the outer lane after
  phase 3, plus the lead holding its own lane and speed throughout.

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
- **Status (p5-s5):** runs end to end through the C++ API
  (`core/tests/action_distance_test.cpp`, `GS4TrafficJamApproachHoldsTheGap`)
  with the functional pass criteria asserted, and is a bit-identity anchor in
  `determinism_test.cpp`. A Python flavour lives in
  `python/examples/distance_keeping.py`. The XML form waits for the frontend
  (P4).

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
- **Status (p2-s5):** runs in its **programmatic form** and is a determinism
  anchor (`determinism_test.cpp`, `make_clothoid_nurbs_scenario`): an
  engine-controlled entity follows a general clothoid then a NURBS, bit-identical
  across two engines and hex-pinned for the 3-OS matrix — retiring R3. The
  analytic-fidelity checks (points on the circle within 1e-9) live in
  `trajectory_test.cpp`. The full **XML-parsed** form still awaits the XML
  frontend (P4), as with GS-4.

### GS-8 — Route through a junction

On a hand-authored four-way junction map, ego receives an `AssignRouteAction`
(waypoints across the junction) plus an `AcquirePositionAction` target; a
crossing vehicle yields via a `ReachPositionCondition` trigger.

- **Exercises:** P3 (routes, junction connectivity, road/lane ↔ world
  conversions), P5 (routing actions, reach position), P2, P4, P6.
- **Pass:** bit-identical trace; ego's road-id sequence equals the declared
  route; both vehicles clear the junction without freespace violation.
- **Status:** **live** — `gs8-junction-route.xosc` on `maps/junction4.xodr`, a
  hand-authored four-way junction where a west-east corridor
  (roads 1 → junction 50 → 4) crosses a south-north one (5 → 50 → 6). Ego takes
  the corridor under an `AssignRouteAction` plus an `AcquirePositionAction`
  target; the crossing vehicle holds at the stop line and pulls away only once
  a `DistanceCondition` says ego has cleared the junction box, so the two never
  occupy it together. Eight checkpoints cover ego's lane through the junction
  and the crosser's yield-then-go.

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

### GS-12 — DSL cruise, with an XML twin (parity pair) — **landed**

`gs12-dsl-cruise.osc`: a concrete DSL scenario — one `vehicle` actor, a
`do serial` of two `drive()` phases with concrete durations, speed modifiers
anchored at `start` and at `end` — compiled through the DSL frontend into the
same Scenario IR the XML frontend produces. `gs12-xml-cruise.xosc` says the same
thing in the other language.

- **Exercises:** P7 (parse, type-check against the standard library), P8
  (entry point, actor lowering, serial composition, durations, `at` anchoring).
- **Pass:** the two traces are **equal byte for byte**
  (`golden.py compare-pair gs12`, part of `check-all` on all three platforms) —
  the two-frontends-one-runtime claim, verified literally.
- **Role:** retires the execution-parity risk (R5); the release headline demo.

> **Why the pair is GS-1's shape and not GS-2's.** The plan was a DSL twin of
> the GS-2 cut-in. A byte-identical pair needs both files to denote the *same IR
> actions*, and GS-2's lane change does not survive that: §8.9's `lateral`
> modifier lowers to a `LaneOffsetAction` while GS-2's XML uses a
> `LaneChangeAction`, whose transition time is derived from `maxLateralAcc`
> rather than from a duration (ADR-0016). The two are different actions with
> different timing laws, so a pair built on them would compare two things that
> were never claimed to be equal. GS-1's longitudinal shape is expressible
> identically in both languages, so it is what the parity claim is made on;
> GS-13 covers the DSL-only constructs that have no XML counterpart at all.

### GS-13 — DSL composition showcase — **landed**

`gs13-dsl-alternatives.osc`: DSL-only constructs, with no XML counterpart. A
`do serial` whose first phase is a `parallel` of two vehicles adjusting speed
concurrently — one of them placed by a `position(ahead_of:)` modifier — followed
by a `one_of` phase run with the documented deterministic selection input
(`--select`, default the first alternative). The chosen alternative uses a
relative-speed modifier (`slower_than`), the other an absolute target.

- **Exercises:** P7 (full syntax), P8 (serial/parallel/one_of per §7.6.2.1,
  modifier timing, relative targets), P6 (the CLI's selection input).
- **Pass:** bit-identical trace for the default alternative; phase boundaries at
  the declared times; `--select brake` changes what runs and nothing else.
- **Not covered, and deliberately:** `event`/`wait`/`until` gating. §7.6.2.5's
  events are abstract control objects with no runtime carrier in v0.0.1, so they
  are reported rather than executed (ADR-0031); a golden scenario asserting them
  would be asserting a diagnostic, not a behaviour.

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
