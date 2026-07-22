# ADR-0014: Private actions II — routing, distance keeping and controllers

- **Status:** accepted
- **Date:** 2026-07-22

## Context

Sprint p5-s5 (#33, "Private actions II: routing, distance keeping &
controllers") continues the critical path (p5-s4 → **p5-s5** → p5-s6). Its issue
covers seven private actions: `LongitudinalDistanceAction`,
`AssignRouteAction`, `AcquirePositionAction`, `FollowTrajectoryAction`,
`AssignControllerAction`, `ActivateControllerAction` and `VisibilityAction`.

The issue declares dependencies on p5-s4 (merged) and p2-s5 (trajectory shapes,
open). p2-s3 (lateral dynamics), p2-s4 (positions) and all of P3 (roads) are
open too: the tree has no route, trajectory, controller or visibility
scaffolding at all, and no lateral machinery to build a steering controller on.

Annex A Table 10 resolves most of that tension. Of the seven actions, only
`FollowTrajectoryAction` and `LongitudinalDistanceAction` assign a control
strategy — the other five "complete immediately (does not consume simulation
time)" because they install state rather than motion. Those five are therefore
buildable today without lateral dynamics or a road network, and the two that do
move an entity are buildable in a well-defined subset.

## Decision

p5-s5 lands all seven actions as a kernel + C ABI + Python sprint (no XML
frontend, the standing precedent since p5-s1), in the subset that is honest
today, and defers the rest with explicit pointers.

### Scope decisions

1. **`FollowTrajectoryAction` — polyline subset now.** The `Trajectory` IR
   models the `Polyline` shape only; `followingMode=position` is implemented and
   `follow` is accepted-as-position with an `UnsupportedFeature` warning (the
   ADR-0011 precedent). The full `timeReference` semantics ship, including the
   §6.9.1–§6.9.3 start-state cases. Clothoid/ClothoidSpline/NURBS numerics
   (risk R3), the `Motion` element, `Interpolation`, trajectory catalogs and
   true follow-mode steering → **p2-s5 (#19)**. A `closed` trajectory is stored,
   warned about, and followed as an open path — the loop needs the same
   machinery as the other shapes.
2. **Routing actions — assignment now, roads later.** `Route`, `Waypoint` and
   `RouteStrategy` are modeled over world-frame waypoints; both actions install
   the route on the entity and return Complete immediately per Table 10. The
   strategy is **stored, never interpreted** — path selection needs a road
   network, and interpreting `random` would put a random generator in the
   runtime, which the determinism contract forbids. Waypoint-on-road
   prerequisite checks, `RouteStrategy` interpretation, route-following motion
   and the GS-8 junction map → **p3-s4 (#23)**.
3. **Gateway — defaulted virtuals.** `ISimulatorGateway` gains
   `on_controller_assigned` and `on_visibility_changed` as non-pure virtuals
   with no-op bodies, so hosts written against the previous interface keep
   compiling. This is an amendment to ADR-0003 (see below).

### The distance-keeping control law

`LongitudinalDistanceAction` is a longitudinal-domain owner: it installs no
finite `LongitudinalController` but is re-polled every step, the same shape as
p5-s4's continuous relative speed target. Per re-poll:

1. Measure the signed longitudinal separation `g` from the actor to the
   reference along the effective coordinate system's longitudinal axis —
   reference-point or, with `freespace`, the bumper-to-bumper gap between the
   two oriented boxes, carrying the sign of the reference-point separation. Both
   go through the shared `runtime::distance_measure` kernels the interaction
   conditions use, so an action and a condition can never disagree about a gap.
2. Resolve the target `G`: `distance`, or `timeGap × the actor's current speed`
   (the same headway arithmetic `TimeHeadwayCondition` uses). The desired signed
   separation is `+G` for `trailingReferencedEntity`, `-G` for
   `leadingReferencedEntity`, and `±G` following the current side for `any`.
3. Command `v = v_ref + approach`, where `v_ref` is the reference's speed
   projected on the same axis and `approach` closes the error `e = g - desired`.

The obvious choice for `approach` is the deadbeat term `e/dt`, which lands the
gap exactly on target in one step. Alone it is wrong: as soon as a
`DynamicConstraints` or `Performance` limit keeps the command from being
applied, the controller saturates its acceleration, keeps demanding more, and
overshoots — in testing, straight through the lead vehicle. The approach rate is
therefore additionally bounded by `sqrt(2·a·|e|)`, the relative speed the actor
could still brake off within the error it has left (`a` is the deceleration
limit when closing a gap, the acceleration limit when opening one). Near the
target the deadbeat term is the smaller of the two and wins, which is what
preserves the exact landing.

The resulting command is clamped by `min(DynamicConstraints, Performance)` per
step, floored at zero (the scalar-speed model has no reverse) and capped by the
maximum speed. A non-continuous action completes when `|e| ≤ 1e-9 m` — a band
that only has to absorb the rounding of the position integration, because the
law lands exactly in the point-mass model. A continuous one returns Running
forever (§7.5.3) and ends only through supersession or a stopTransition.

The reference's speed is read from the entity table at command time, carrying
the same one-step-lag caveat as p5-s4's `resolve_relative_speed`; the ordering
is fixed by the storyboard walk, so it is deterministic.

Both deferrals report rather than pretend: a road-based coordinate system
(p3-s4) and a freespace gap without bounding boxes both emit an
`UnsupportedFeature` warning and end the action — the missing-prerequisite path
of §7.5.2.2.

### The trajectory follower

The follower resolves everything once at install (segment lengths, cumulative
arc length, per-segment headings, effective vertex times) so a step only
interpolates. Which domains it owns follows Table 10:

- **`timeReference=none`** owns the lateral domain only. The entity teleports to
  the start of the trajectory (§6.9.1) and advances by `speed × dt`, so whatever
  owns the longitudinal domain still sets the pace.
- **`timeReference=timing`** owns the longitudinal domain too and goes through
  `supersede_longitudinal`. Effective vertex times are `t × scale + offset`,
  measured from simulation time zero (absolute) or the action's start
  (relative). Before the first time reference the entity keeps moving as before
  and is teleported at `t1` (§6.9.2); starting after it joins at the
  time-interpolated point (§6.9.3). Position, heading and speed all come from
  the timed segments.

`initialDistanceOffset` truncates the trajectory at that arc length, which also
implements the spec's "the time that would be taken to reach this point is
deducted from all calculated waypoint time values".

Segment headings reach `EntityState.heading`, so they are inside the
bit-identity contract and cannot come from libm. `det_atan2` joins
`det_sin`/`det_cos` in `scena/runtime/detmath.h` on the ADR-0006 terms:
exact-rational Taylor coefficients generated by `scripts/gen_detmath_coeffs.py`,
two exact reduction identities, IEEE quadrant and signed-zero behavior for
finite arguments, and a quiet NaN for non-finite ones.

**The step-order contract is unchanged.** When a follower has placed an entity
during the storyboard evaluation, the integrate phase of that same step skips
it: only the *source* of phase 4 changes for that entity, not the order of the
phases documented in `engine.h`. That would have been an ADR-level change; this
is not one.

### Controller activation semantics

Scena implements no controller models — an assigned controller is name, type and
ordered properties handed to the host. But Scena's engine **is** the default
controller (docs/user-guide/motion.md), so `ActivateControllerAction` toggles
the engine's own control of a domain:

- **Deactivating longitudinal** retires the action currently owning the domain
  (it completes on its next re-poll, the §7.5.2.1 override path) and the entity
  holds its current speed. **Deactivating lateral** stops an active trajectory
  follower the same way.
- A longitudinal action fired while that domain is inactive is reported and
  skipped, the missing-prerequisite analog of §7.5.2.2; a trajectory needs the
  lateral domain, and a timed one both. The event still completes, so a
  scenario cannot hang on a deactivated domain.
- An unset flag means "no change", and re-activation resumes normal dispatch.

Activating a domain the controller's `controllerType` does not define is a
`ValidationError` citing
`asam.net:xosc:1.2.0:scenario_logic.controller_activation`. Deactivation is
never constrained by that rule. `ActivateControllerAction` itself is not checked
against an assigned controller: the engine's default controller covers both
movement domains, so activation is always in-domain.

### ADR-0003 amendment: gateway hand-offs

ADR-0003 fixed the gateway as the engine's only route to the outside world, with
three pure virtuals. p5-s5 adds two **defaulted** virtuals — `on_controller_
assigned` and `on_visibility_changed` — called synchronously at the action's
apply point, a fixed instant in the step, so the sequence of calls is part of
the deterministic run. Defaulting them keeps existing hosts source-compatible;
the layering rule is untouched, since both hand over IR/value types the core
already owns and neither lets the host call back into the engine.

## Consequences

- The private-action surface is now complete for v0.0.1 except the lateral
  actions (p2-s3 follow-up), `SynchronizeAction` and
  `OverrideControllerValueAction` (both Post). `EntityRecord` gains a route, a
  trajectory follower, an assigned controller, per-domain activation and
  visibility; `retired_longitudinal` is renamed `retired_actions` because
  trajectories retire through it too.
- **GS-4 (Traffic-jam approach) runs end to end** through the C++ API and is a
  determinism anchor. **GS-7 does not**: it chains polyline, clothoid and NURBS
  segments, so it stays with p2-s5. **GS-8 does not**: it needs a junction map,
  so it stays with p3-s4. The issue's exit criterion is therefore met partially,
  and `golden-scenarios.md` says so.
- The C ABI gained five enums, six structs, seven builders and six queries, all
  appended; Python gained the matching types plus `route_of`,
  `assigned_controller_of`, `controller_activation_of` and `visibility_of`.
- Determinism gained `det_atan2` (hex-pinned over a 25-pair probe set and in the
  cross-platform trace), a GS-4 anchor and a timed-trajectory anchor.
- What p5-s5 deliberately did not do: interpret a `RouteStrategy`, follow a
  route, steer in follow mode, close a trajectory loop, clamp jerk, or model
  per-sensor visibility. Each is warned about at load or documented in the
  coverage matrix, never silently ignored.
