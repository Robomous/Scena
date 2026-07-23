# ADR-0016: Lateral kinematics and flat-world lane resolution

- **Status:** accepted
- **Date:** 2026-07-23

## Context

Sprint p2-s3 (#17, "Lateral dynamics & lane-change shapes") builds the lateral
motion machinery and lands the three `LateralAction` types that P5 descoped for
want of it: `LaneChangeAction`, `LaneOffsetAction` and `LateralDistanceAction`
(ADR-0013).

Before this sprint the engine had **no lateral state at all**. `EntityState`
carries a scalar `speed` along a `heading`, and the integrate phase moved every
engine-controlled entity in a straight line. The only code that wrote a
sideways displacement was the trajectory follower (ADR-0014), which bypasses the
integrator entirely by writing positions outright.

Two problems have to be solved together:

1. **A lateral motion model.** Something has to turn "move 3.5 m to the left
   over 4 seconds" into per-step positions and a plausible heading, without
   inventing vehicle dynamics the project has explicitly deferred.
2. **Target-lane resolution.** A `LaneChangeAction` names a *lane*, and lanes
   live in the road network — which Scena cannot reach. p2-s3's declared
   dependency p3-s1 (the frozen `IRoadQuery` v1, #20) has not been executed.

## Decision

### 1. The lateral axis

When a lateral action first needs one, the engine captures a straight **lateral
axis** from the entity's current pose: an origin at the entity, a heading equal
to the entity's, and the `det_sincos` of that heading cached once. The entity
starts at **offset zero** on it.

While an axis is live, the integrate phase composes two legs — an advance of
`speed·dt` along the axis and the offset increment the storyboard phase
commanded across it — and blends the heading from them:

```
x += ds·cos(a) − do·sin(a)
y += ds·sin(a) + do·cos(a)
heading = a + det_atan2(do, ds)
```

The axis is the **flat-world stand-in for a lane centre line**. Capturing the
entity at offset zero encodes the assumption that an actor sits on its lane
centre when a lateral action starts; a real in-lane offset needs a road backend
(#23).

**An entity with no live axis takes the previous straight-line branch,
untouched.** That guard is what makes the model additive: every pre-existing
hex-pinned trace (GS-1, GS-4, trajectory, OBB, `det_atan2`) is bit-for-bit
unchanged, which the test suite verifies rather than assumes.

#### Heading blending and its edge cases

The blend is not a smoothing heuristic — it is the angle the two legs subtend,
so the entity points where it is actually going. Three consequences are
documented as *properties*, not approximations:

- **`do == 0`** ⇒ `atan2(0, ds) == 0` ⇒ the heading is the axis heading. This
  also covers `dt == 0`, with no special case.
- **`ds == 0`, `do ≠ 0`** ⇒ a ±π/2 crab angle. An entity moving purely sideways
  really is pointing across its path.
- **`Linear` kinks** at both endpoints, because its offset rate jumps from zero
  and back. `Cubic` and `Sinusoidal` end with zero lateral rate and straighten
  out on their own. This is a property of the authored shape, and smoothing it
  would misrepresent what the scenario asked for.

#### Dissolution, release and teleport

- **Natural completion**: the action reaches its goal during the storyboard
  phase, but the axis must *not* dissolve there — the integrate phase of that
  same step still has an increment to apply. A pending flag defers it: the
  increment lands, the heading snaps to the axis, and the axis dissolves.
  Continuing straight through the offset reached is exactly what the composed
  model would have produced from there on.
- **Release** (controller deactivation, entity deletion): the axis dissolves
  immediately and the entity keeps the heading it was last blended to — the
  lateral counterpart of a released longitudinal domain holding its speed.
- **Teleport**: a live axis is **re-anchored** through the new position at the
  offset already reached, so an in-flight transition continues from where the
  entity landed instead of dragging it back across the old axis.
- **Init phase** (§8.5): an init action's offset is applied as a single
  perpendicular translate in a settle pass, with no heading blending, and the
  axis is dissolved before the run starts.

### 2. Duration from `maxLateralAcc`

A `LaneOffsetAction` authors no duration: `LaneOffsetActionDynamics` gives only
a shape and a `maxLateralAcc`. The duration is derived so that the peak lateral
acceleration of the shaped ramp equals that limit. With `o(t) = Δ·g(t/T)` the
lateral acceleration is `Δ·g''(p)/T²`, so equating its peak to `a` gives

```
T = √(k·|Δ| / a),   k = peak |g''| of the unit shape
```

| Shape | `k` | Derivation |
|---|---|---|
| `Sinusoidal` | π²/2 | `g = (1 − cos πp)/2`, `g'' = (π²/2)·cos πp`, peak at the endpoints |
| `Cubic` | 6 | `g = 3p² − 2p³`, `g'' = 6 − 12p`, peak at the endpoints |
| `Linear` | 4 | see below |
| `Step` | — | instantaneous, `T = 0` |

`Linear` has zero curvature in its interior and unbounded kinks at its ends, so
curvature yields nothing. The substitute is the **minimum-time rest-to-rest
bound** for an acceleration-limited move of the same displacement: accelerate at
`a` for half the time, decelerate for the other half, giving `Δ = a·T²/4`. It is
the unique acceleration-limited lower bound for that displacement, and it sits
continuously below the smooth shapes — `4 < π²/2 < 6` — so the three shapes
order sensibly.

`T = 0` also for `Δ = 0` and for an absent `maxLateralAcc`, which "is
interpreted as 'inf'". A `maxLateralAcc` of exactly `0` is **rejected at init**
even though the standard's stated range is `[0..inf[`: it permits no lateral
motion at all, so the target could never be reached and the action would never
end. Rejecting it is preferable to modelling an infinite transition.

### 3. Reusing the value-transition sequencer

The offset ramps run on `runtime::LongitudinalController` verbatim. An offset
ramp and a speed ramp are the same `TransitionDynamics` shape math over a
different quantity, and sharing the code keeps them bit-identical. Only its
documentation changed, from "speed controller" to "value-transition sequencer".

A distance-dimensioned lane change feeds it `speed · last_dt_` as the step
distance — the same explicit scheme `drive_longitudinal` and the position
integrator use. An entity at a standstill therefore never finishes one, exactly
as a distance-dimensioned `SpeedAction` does not.

### 4. Flat-world lanes, and a forward-pulled `IRoadQuery`

p3-s1's frozen `IRoadQuery` v1 does not exist, so p2-s3 forward-pulls the
minimum it needs, following the p5-s3 bounding-box and p5-s5 gateway
precedents. Three queries are appended to `IRoadQuery` as **defaulted** virtuals
returning `false`:

```cpp
bool lane_width(road_id, lane_id, s, out_width) const;
bool lane_center_offset(road_id, lane_id, s, out_t) const;
bool relative_lane(road_id, lane_id, count, out_lane_id) const;
```

Defaulted, not pure: an existing host implementation keeps compiling, and p3-s1
retains full freedom over the shape of the pure-virtual core. "Unsupported" and
"no such lane" are deliberately the same answer, because the caller does the
same thing either way.

Resolution order for a **relative** target: map actor and reference to lane
positions, step `count` lanes from the reference's lane, and take the difference
of the two lane centre lines. Any query that declines degrades to the flat-world
model — `count` **default lane widths** from the reference entity's lateral
position — rather than stopping the action. The default is
`kDefaultLaneWidth = 3.5 m`, settable per engine with `set_default_lane_width`;
it is a modeling default, not a value the standard prescribes, since ASAM
OpenSCENARIO leaves lanes to the road network.

An **absolute** lane id has no flat-world reading at all. With a backend it is
parsed with `std::from_chars` (never `std::stoi` — the locale rule) and
resolved; without one the action warns `UnsupportedFeature` and completes.

**Deliberately no init-time warning for absolute lane targets**, in contrast to
the road-coordinate-system deferrals, which *are* warned about at init. The
difference is real: a road coordinate system cannot be honoured by any host
Scena can talk to today, while an absolute lane target *can* be, by a host that
implements the forward-pulled queries. Only the runtime knows which it has, so
the runtime warning is the authoritative one.

### 5. `continuous` ⇒ `Running`, not `Ongoing`

A continuous `LaneOffsetAction` reports `ActionOutcome::Running`. `Ongoing` is
never re-polled, so the offset could not be re-enforced after the reference
entity moved or a teleport displaced the actor. `Running` is re-polled every
step and matches the continuous `RelativeTargetSpeed` (ADR-0013) and continuous
distance-keeping (ADR-0014) precedents.

After arrival each re-poll re-measures and installs a fresh transition only when
the target has moved by more than the distance-keeping epsilon. Mid-transition
the target stays where it was when the ramp was built, so a moving reference
does not restart the authored shape every step.

### 6. Lateral distance keeping

The controller is the ADR-0014 deadbeat-plus-glide law moved across to the
lateral rate. With no `DynamicConstraints` every limit is infinite and it
degenerates to closing the error in one step — which is exactly what the
standard means by keeping the distance "rigid". There is no forward/backward
asymmetry to model (moving left and moving right are the same manoeuvre), so
the glide bound is always the deceleration limit, while the rate slews up by
`maxAcceleration` and down by `maxDeceleration`.

The `Performance` envelope is **not** consulted: its limits are longitudinal
(§Performance), while this action's `DynamicConstraints` are explicitly
"limiting values for lateral acceleration, lateral deceleration and lateral
speed".

Two measurement decisions exist purely because the heading blend would otherwise
close an unintended feedback loop:

- **The entity-CS lateral axis comes from the *axis* heading, not the
  instantaneous yaw.** Sideways motion blends the heading away from the
  direction of travel; measuring in that transient frame made the controller
  chase its own manoeuvre and oscillate instead of settling. The two headings
  agree whenever the entity is not mid-manoeuvre, which is the only time the
  distinction could be observed.
- **`any` picks its side from the reference-point separation** even when the
  target is a freespace gap. A yawed vehicle presents a wider flank, so a
  freespace gap can cross zero mid-manoeuvre; taking the side from it flipped
  the target and drove the actor across the reference. For the same reason
  overlapping flanks clamp to zero clearance rather than going negative, so the
  measured gap's sign always follows the side the actor is on.

The **lane** coordinate system is warned about on its own terms, not as a
deferral: §6.4.8.2.2 states the lane-CS lateral distance *is undefined*. No road
backend could rescue it.

### 7. Supersession (§7.5.1, Annex A Table 10)

The lateral domain has one owner per entity. `supersede_lateral` retires
whichever action holds it — the offset slot or a trajectory follower, both of
which "assign a lateral control strategy" — and the retired action reports
`Complete` on its next re-poll.

The axis is **kept** across a lateral-to-lateral handover, so an absolute lane
offset keeps meaning the same thing: an entity 1.2 m off its lane asked to hold
1.0 m should move back 0.2 m, not out another 1.0 m. A `FollowTrajectoryAction`
dissolves it instead, because the follower writes positions outright
(§7.5.2.1: "an instance of `FollowTrajectoryAction` overrides an instance of
`LaneChangeAction`"). The reverse holds too.

The two movement domains are independent: a `SpeedAction` never supersedes a
lane change, and vice versa.

### 8. Same-step ordering

A distance-dimensioned lateral ramp reads `record.state.speed`, which a
`SpeedAction` earlier in document order may already have updated in the same
step. That is deterministic — document order is fixed and the storyboard walk is
ordered — and it is the same coupling `drive_longitudinal` already has with the
position integrator. It is noted here so it is a decision rather than an
accident.

## Consequences

- Lane changes, lane offsets and lateral distance keeping run without a road
  network, which unblocks GS-2 and the lateral half of the coverage matrix.
- Two new deterministic trig sites: one `det_sincos` per axis capture (cached)
  and one `det_atan2` per composed step. The duration derivation adds none —
  `std::sqrt` is IEEE-exact.
- Lateral displacement accumulates as a telescoping sum, so an entity that
  "arrives at 3.5 m" lands a few ulps off it. The residue is deterministic and
  hex-pinned by the GS-2 anchor.
- Three new public `IRoadQuery` signatures that p3-s1 must live with. They are
  kept minimal and defaulted so the v1 freeze can still reshape the pure-virtual
  core.
- Real lane identity — which lane an entity is *in*, and lane geometry that
  varies along a road — remains with p3-s4 (#23).

## Alternatives considered

- **A steering/bicycle model instead of an offset ramp.** It would produce the
  heading for free, but it needs a controller, a wheelbase and a tyre model —
  all deferred — and it would make the exact endpoint arrival the standard
  requires ("by reaching the lateral centerline offset on the target lane") a
  tuning problem rather than a guarantee.
- **Lateral state on `EntityState`.** Rejected: `EntityState` is ABI, and the
  lateral offset is a private detail of an in-flight action, not something a
  host reports or consumes. It lives on `EntityRecord`.
- **Dissolving the axis on lateral-to-lateral supersession.** Simpler, but it
  would silently re-base absolute lane offsets on the entity's current position,
  changing what a scenario means depending on what ran before it.
- **Blocking p2-s3 on p3-s1.** The maintainer-approved route was the minimal
  forward-pull; blocking would have stalled the lateral half of P2 behind an
  unexecuted sprint whose interface this work informs.
- **Accepting `maxLateralAcc = 0` per the stated range.** It would need an
  infinite-duration transition — a never-ending non-continuous action — for no
  expressible scenario intent.

## References

- ASAM OpenSCENARIO XML 1.4.0 §6.3.2–§6.3.4 (coordinate systems, t-axis and
  vehicle y-axis point left), §6.4.8.2 (lateral distance per coordinate system;
  §6.4.8.2.2 lane CS undefined), §7.4.1.2 (motion control actions), §7.4.1.4
  (types of private action), §7.5.1 (conflicting actions), §7.5.2 (completing an
  action, prerequisites), §7.5.3 (continuous actions), §8.5 (init phase),
  Annex A Table 10; classes `LaneChangeAction`, `LaneChangeTarget`,
  `AbsoluteTargetLane`, `RelativeTargetLane`, `LaneOffsetAction`,
  `LaneOffsetActionDynamics`, `LaneOffsetTarget`, `LateralDistanceAction`,
  `DynamicConstraints`, enumeration `LateralDisplacement`.
- [ADR-0011](ADR-0011-longitudinal-dynamics.md) — the transition-dynamics model
  and the Rate-as-peak-gradient generalization this reuses.
- [ADR-0013](ADR-0013-private-actions-longitudinal-teleport.md) — where the
  three lateral actions were descoped from, and the continuous ⇒ Running
  precedent.
- [ADR-0014](ADR-0014-private-actions-routing-distance-controllers.md) — the
  deadbeat-plus-glide distance law, the defaulted-virtual gateway precedent, and
  the trajectory follower this shares the lateral domain with.
- [Motion](../user-guide/motion.md) — the user-facing description.
