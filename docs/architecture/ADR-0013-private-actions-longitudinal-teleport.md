# ADR-0013: Private actions I — relative speed targets and world-frame teleport

- **Status:** accepted
- **Date:** 2026-07-22

## Context

Sprint p5-s4 (#32, "Private actions I: longitudinal, lateral, teleport") sits on
the critical path (p2-s2 → **p5-s4** → p5-s5). Its issue lists the full private
motion catalog — `SpeedAction` with relative targets, `SpeedProfileAction`, the
three lateral actions (`LaneChangeAction`, `LaneOffsetAction`,
`LateralDistanceAction`), `TeleportAction` "over all position types", and XML
frontend lowering.

Most of that catalog reaches into machinery that does not exist yet. The roadmap
gates p5-s4 only on p2-s2 (longitudinal dynamics, done) and p5-s3 (interaction
conditions, done) — deliberately **not** on the sprints the lateral and
full-position work needs, all of which are open:

- **p2-s3** (#17) — lateral dynamics & lane-change shapes: the lateral machinery
  `LaneChangeAction`/`LaneOffsetAction`/`LateralDistanceAction` build on. The
  engine today integrates position straight-line from `speed × heading` only;
  there is no 2D lateral state to drive.
- **p2-s4** (#18) — the `PositionResolver` and the ten §6.3.8 Position variants;
  it also owns the `TeleportAction` **runtime** over those variants. Only
  `WorldPosition` exists in the IR today.
- **p3-s4** (#23) — road/lane/route position resolution against `IRoadQuery`.
- The XML frontend is a stub until P4 (standing precedent from p5-s1/s2/s3).

ADR-0011 explicitly handed p5-s4 the relative-target and multi-action
longitudinal-conflict work; ADR-0005 deliberately deferred the full §7.5 conflict
machinery, which has its own issue (#51).

## Decision

p5-s4 lands the subset that is buildable now and honest — the longitudinal
relative targets and a world-frame teleport — as a kernel + C ABI + Python sprint
(no XML frontend), and defers the rest with explicit pointers.

### Relative speed targets (§RelativeTargetSpeed)

`SpeedAction` gains a relative target alongside its absolute one (mirroring the
`SpeedActionTarget` union): `RelativeTargetSpeed{entity_ref, value, value_type ∈
{delta, factor}, continuous}`. The existing absolute constructors and
`target_speed()` are unchanged (additive), preserving the ~30 call sites, the C
ABI, and the bindings.

- **Non-continuous:** the target is resolved **once**, against the reference
  entity's speed when the action starts (delta ⇒ `ref + value`, factor ⇒
  `ref × value`), then reached through the same default longitudinal controller
  as an absolute target — Step ⇒ instantaneous, otherwise a ramp under the
  Performance clamp.
- **Continuous (§7.5.3):** installs per-entity continuous tracking that
  re-matches the reference **every step** and reports `ActionOutcome::Running`
  indefinitely — it never completes on its own. This extends the meaning of
  `Running` (previously "governed by transition dynamics until it reaches its
  target") to "a re-polled multi-step action; a continuous one simply never
  reports Complete". Per the standard it must not be combined with a time- or
  distance-dimensioned transition; a positive time/distance is a validation
  error.

The reference speed is read from the entity table (`std::map`, sorted iteration)
at application time — deterministic given the fixed storyboard/scheduler order. A
one-step lag versus the reference is possible when the reference's own action is
applied later in the same step; it is reproducible and acceptable. Fixtures place
the reference's maneuver first so the common case reads the current-step speed.

### Minimal single-domain conflict resolution (§7.5.1)

A continuous target that never completes forces the question the finite case
could dodge: what happens when a second longitudinal action lands on an entity
already driven by one. The minimal rule: **the newest longitudinal action
supersedes**. The superseded action is *retired* — on its next re-poll it reports
`Complete` (releasing its event per §8.4.2) instead of reinstalling itself and
fighting the current owner. Retirement is tracked per-entity by action identity
only (never iterated for a result), so it does not reach the determinism
contract. The full §7.5 catalog — cross-domain conflicts, §7.5.4 bulk/actor
semantics, the completion-reason taxonomy — remains **#51**.

### World-frame teleport (§TeleportAction)

`TeleportAction` carries a `WorldPosition` and applies as a step (instantaneous)
action: it writes the entity's world position and completes in the evaluation it
fires, both as an init action and mid-run. Orientation is part of the full
Position and is not modeled yet, so heading/pitch/roll are left unchanged. A
host-controlled entity's reported state overwrites the teleport on the next poll;
the formal host round-trip is p2-s4.

The action holds a `WorldPosition` directly rather than an `ir::Position` variant:
p2-s4 owns the position taxonomy and the resolver, and building a variant here
would encroach on it. `TeleportAction` generalizes to `ir::Position` when the
resolver lands.

### Deferrals (scope out)

- **Lateral actions** (`LaneChangeAction`, `LaneOffsetAction`,
  `LateralDistanceAction`) → a **p5-s4 follow-up** gated on p2-s3 (lateral
  machinery) + p3-s4 (road/lane). There is no lateral runtime to hook onto, and
  an action that silently no-ops would be more misleading than a road-dependent
  condition that correctly evaluates false (the p5-s2/s3 pattern applies to
  *evaluation*, not to a motion the scenario expects to happen). The coverage
  rows record the deferral. The maintainer seeds the follow-up issue.
- **Non-world Teleport targets** + the `PositionResolver` → p2-s4/p3-s4.
- **Full §7.5 conflict / bulk-actor semantics** → #51.
- **entityRef-relative `SpeedProfileAction`** + `followingMode=follow`
  jerk/`DynamicConstraints` → #62. p5-s4 implements relative targets for
  `SpeedAction` only, partially addressing #62.
- **XML lowering** → P4.

### Diagnostics

A relative target with an unresolvable reference entity is a `SemanticError`
(the `RelativeSpeedCondition` precedent); a continuous target combined with a
time/distance transition, and a non-finite teleport position, are
`ValidationError`s. No new `Status`/ABI enumerator was needed.

## Consequences

- The longitudinal action surface is now complete for v0.0.1 bar follow-mode
  jerk: absolute and relative targets, one-shot and continuous, with defined
  supersession. p5-s5's distance-keeping actions build on the same `Running`
  re-poll and retirement mechanism.
- `EntityRecord` grows `continuous_speed` and `retired_longitudinal`; both are
  identity/value state that never reaches results, preserving bit-identity
  (a GS-1 determinism anchor guards it).
- GS-1 (Cruise baseline) runs end-to-end via the C++ API — init teleport + init
  speed + a timed linear `SpeedAction` — as a functional test and a determinism
  anchor. GS-2 (Cut-in) stays deferred: its `LaneChangeAction` needs the lateral
  follow-up and lane identity (p3-s4).
- The C ABI gained `scn_speed_target_value_type` and two builders
  (`scn_engine_add_relative_speed_action`, `scn_engine_add_teleport_action`),
  appended; Python gained `SpeedTargetValueType`, `RelativeTargetSpeed`, the
  `SpeedAction` relative constructor, and `TeleportAction`.
- When p2-s3/p3-s4 land, the lateral follow-up implements the three lateral
  actions and GS-2; when p2-s4 lands, `TeleportAction` generalizes to
  `ir::Position`. Neither disturbs the longitudinal or teleport-world semantics
  fixed here.
