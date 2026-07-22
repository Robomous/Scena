# ADR-0011: Longitudinal dynamics — multi-step actions and the default controller

- **Status:** accepted
- **Date:** 2026-07-22

## Context

Through F0 and the P1/P5 sprints a `SpeedAction` was instantaneous: `Engine::apply`
set the target speed in the evaluation the event fired and reported
`ActionOutcome::Complete`. p2-s2 (#16) delivers the real longitudinal model of
ASAM OpenSCENARIO XML 1.4.0 §7.4.1.2 — a speed **transition** governed by
`TransitionDynamics` (a shape over a time, distance, or rate), a default
controller that clamps to the entity's `Performance` envelope, and a
`SpeedProfileAction` follower — and, with it, the runtime's first **actions that
span multiple steps**. It sits on the critical path to Private actions I (p5-s4,
#32), whose `SpeedAction`/`SpeedProfileAction` semantics this sprint provides.

Two structural facts framed the work:

- **The scheduler fired each action exactly once**, at `enter_running`, and never
  re-polled a running event's actions; a running event could only leave
  `runningState` through a stopTransition. `ActionOutcome` had two values —
  `Complete` and `Ongoing` (never-ending, §7.5.3) — and ADR-0005 explicitly left
  the "missing middle" (transition-driven lifetime) for "an additional
  enumerator with p2-s2".
- **`SpeedAction(entity, target)` is used in ~30 call sites**, the C ABI, and the
  bindings, all expecting instantaneous behavior.

## Decision

### Multi-step action lifetime — `ActionOutcome::Running` + scheduler re-poll

Add a third outcome `ActionOutcome::Running`: the action is in progress under
transition dynamics and will end on its own. The scheduler records, per event,
the actions that reported `Running` at their fire and **re-polls them through the
same fire callback every step** (`progress_event`, driven from
`update_maneuver`); the event ends **regularly** (endTransition, §8.4.2) once the
last one reports `Complete`. `Ongoing` is unchanged and is never re-polled — a
never-ending action still needs a stopTransition. An event is **not** re-polled
on the evaluation it fired the action (its startTransition is this evaluation), so
a transition never reaches its goal in the applying evaluation.

This reuses the existing `FireCallback` signature unchanged, exactly as ADR-0005
predicted ("nothing about the interface has to change").

### The engine owns the transition; the controller is pure kinematics

`TransitionDynamics` and the shape/dimension/following-mode enums live in the IR
(`ir/dynamics.h`). The math lives in `runtime/longitudinal.{h,cpp}` as a pure,
deterministic evaluator plus a `LongitudinalController` (a sequence of segments —
one per SpeedAction, one per SpeedProfileEntry). The engine stores an optional
controller **per entity** on its `EntityRecord`, together with the owning
action pointer (compared by identity only, never for ordering, so it never
reaches results — the determinism contract). Each re-poll advances the controller
by the current step; on completion the record releases it.

An entity has **at most one** active longitudinal controller. A later
longitudinal action supersedes any previous one (it takes ownership on its first
fire). Two *concurrent* longitudinal actions on one entity across different
events — the action-conflict case ADR-0005 deferred — are **not** resolved here;
the behavior is defined (deterministic, bounded) but intentionally simplistic, to
be addressed when the action-conflict registry lands.

### The math (clean-room, from the spec)

Normalized progress `p ∈ [0,1]`, `Δ = to − from`:

- **linear** `from + Δ·p`; **cubic** `from + Δ·(3p²−2p³)` (smoothstep, zero
  gradient at both ends); **sinusoidal** `from + Δ·(1−cosine(π·p))/2` (zero
  gradient at both ends) — routed through `det_cos`, so the only transcendental
  is bit-identical across platforms; **step** instantaneous.
- **time** dimension: `p = elapsed / value`. **rate** dimension: `value` is the
  peak gradient of the shape, so `duration = factor · |Δ| / value` with
  `factor` = 1 (linear), 1.5 (cubic), π/2 (sinusoidal) — for linear this is the
  plain "constant rate = slope"; the shape factor generalizes it to the smooth
  shapes. **distance** dimension: `p = travelled / value`, progress measured in
  metres the entity covers (explicit accumulation).
- A completed transition **snaps** the speed exactly to `to`, so a sinusoidal
  endpoint is never left a `det_cos` ulp short.

### Performance clamp (position-mode + hard clamp)

`followingMode=position` yields the exact spec shapes. On top of that, Scena
applies a **hard Performance clamp** to every transition (the maintainer's
chosen "clamped by performance limits" deliverable), even in position mode —
an intentional, documented divergence from strict position-mode adherence:

- the target is clamped to `maxSpeed`;
- for time/rate dimensions the **duration is extended** so the transition's peak
  acceleration stays within `maxAcceleration` (speeding up) or `maxDeceleration`
  (slowing down). Extending the duration keeps the shape exact and still reaches
  the target — the transition simply takes longer, which is the achievable
  motion.

The clamp applies only to a Vehicle carrying `Performance`; unclassified
participants and non-vehicles are unclamped.

### Position integration — the point-mass model

Speed is evaluated **analytically at t′** during the storyboard tick (phase 3);
the existing explicit position integrator (`x += speed·cos(heading)·dt`) then
runs in phase 4 with that speed. The documented 5-phase step order is preserved.
Position stays a **point-mass** model: no vehicle body dynamics, no lateral
coupling, z/pitch/roll untouched. This is recorded as a simplification in the
motion user-guide chapter.

### `SpeedAction` restructuring, C ABI, Python

`SpeedAction` gains a `TransitionDynamics` and a dynamics-bearing constructor;
the 2-arg constructor is retained as a **Step** transition, so every existing
call site, the C ABI, and the bindings keep compiling and behaving exactly as
before. The C ABI grows **append-only** — enums `scn_dynamics_shape` /
`_dimension` / `scn_following_mode`, transparent structs `scn_transition_dynamics`
/ `scn_speed_profile_entry`, and builders `scn_engine_add_speed_action_dyn` /
`scn_engine_add_speed_profile_action`. `ActionOutcome::Running` is a **runtime**
enum, not a `Status`, so no `scn_status` value is added.

## Alternatives rejected

- **A second scheduler callback for progress.** Cleaner separation, but a public
  signature change the shared `FireCallback` did not need; identity-keyed
  ownership in the engine distinguishes first-fire from re-poll without it.
- **Per-step acceleration clamping** (clamp the achieved `Δspeed/dt` each step).
  Deterministic but it distorts the authored shape and complicates the completion
  test; extending the duration up front keeps the shape exact and the endpoint
  honoured.
- **Trapezoidal / implicit position integration** for exact ramp displacement.
  Rejected to avoid perturbing the determinism goldens; the closed-form contract
  is on *speed* (exact), and position is explicitly a point-mass approximation.

## Consequences

- The runtime can now express any action whose end is governed by transition
  dynamics; p5-s4's lateral and other private actions build on the same
  `Running` re-poll mechanism.
- Determinism holds: the sinusoidal path is `det_cos`, everything else is IEEE
  arithmetic; the `determinism_test` dynamics fixture (cubic + sinusoidal ramps)
  is bit-identical across runs.
- **Deferred:** relative speed targets (`RelativeTargetSpeed`); `followingMode=
  follow` jerk-shaping and `DynamicConstraints` (`maxAccelerationRate` /
  `maxDecelerationRate`); the `entityRef`-relative speed profile; multi-action
  longitudinal conflict resolution; and XML lowering (P4/p5-s4). A follow-up
  issue tracks the relative-target and follow-mode work.
