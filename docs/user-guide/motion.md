# Motion — longitudinal and lateral dynamics, and teleport

This chapter covers how an engine-controlled entity moves: how its **speed**
changes over time — the `SpeedAction` transition model (absolute and relative
targets), the default controller and its `Performance` clamp, and the
`SpeedProfileAction` follower — how it moves **sideways** (`LaneChangeAction`,
`LaneOffsetAction`, `LateralDistanceAction`), and the instantaneous
**`TeleportAction`**. It follows ASAM OpenSCENARIO XML 1.4.0 §7.4.1.2
(SpeedAction / TransitionDynamics), §7.4.1.4 (the lateral actions),
§SpeedProfileAction, §RelativeTargetSpeed and §TeleportAction.

Road-relative positioning arrives in a later sprint: lateral motion here is
modelled in a **flat world**, without a road network. See
[Lateral motion](#lateral-motion).

## The model

An engine-controlled entity integrates straight-line kinematics each step:
position advances from **speed** along **heading**. A `SpeedAction` no longer
sets the speed instantaneously — it drives a **transition** from the current
speed to a target, shaped over a time, a distance, or a rate. While the
transition is in progress its event stays running; it ends regularly when the
target is reached.

### Transition dynamics

A `TransitionDynamics` has a **shape**, a **dimension**, and a **value**:

| Shape | Speed as a function of progress `p ∈ [0,1]` | Notes |
|---|---|---|
| `Linear` | `from + Δ·p` | constant rate |
| `Cubic` | `from + Δ·(3p² − 2p³)` | smoothstep; zero acceleration at both ends |
| `Sinusoidal` | `from + Δ·(1 − cos(π·p))/2` | zero acceleration at both ends |
| `Step` | jumps to `target` | instantaneous; consumes no time (value must be 0) |

where `Δ = target − current`.

| Dimension | Meaning of `value` | Progress |
|---|---|---|
| `Time` | duration [s] | `p = elapsed / value` |
| `Rate` | peak rate of change [Δ/s] | duration = `factor · |Δ| / value` |
| `Distance` | distance to acquire the target [m] | `p = travelled / value` |

For the `Rate` dimension, `value` is read as the **peak** gradient of the shape.
For `Linear` that is just the slope (`duration = |Δ| / rate`, the plain reading
of "constant rate"); the shape factor generalizes it to the smooth shapes —
1.5 for `Cubic`, π/2 for `Sinusoidal`.

The 2-argument `SpeedAction(entity, target)` is shorthand for a `Step`
transition — the historical instantaneous behavior.

### The default controller and the Performance clamp

A Vehicle may carry a `Performance` envelope (max speed, max acceleration, max
deceleration). The default longitudinal controller clamps every transition to
it:

- the target is capped at `maxSpeed`;
- for `Time` and `Rate` transitions the **duration is extended** if the authored
  transition would exceed `maxAcceleration` (speeding up) or `maxDeceleration`
  (slowing down). The shape is kept exact and the target is still reached — the
  transition just takes longer.

So a `SpeedAction` demanding 0 → 10 m/s in 1 s on a vehicle limited to 2 m/s²
takes 5 s, not 1 s, and its event completes when the (stretched) transition
finishes. A non-vehicle, or a vehicle without `Performance`, is not clamped.

> The clamp is applied even in `followingMode=position`. This is an intentional
> simplification (see ADR-0011): Scena favours achievable motion over strict
> adherence to a physically impossible shape. Strict position-mode adherence and
> `followingMode=follow` jerk-shaping are deferred.

### Speed profiles

A `SpeedProfileAction` is a series of `SpeedProfileEntry{speed, time?}` targets.
In `followingMode=position` the controller interpolates **linearly** between
successive targets, starting from the entity's current speed. The first entry's
`time` is a delta from the start of the action, each later entry a delta from the
previous. An entry with **no** time is reached as fast as the `Performance`
envelope allows (or instantaneously without one).

### Relative speed targets

A `SpeedAction` target may instead be **relative to another entity**
(`RelativeTargetSpeed`): a reference entity, a `value`, and a `value_type` —
`Delta` (`target = reference + value`, in m/s) or `Factor`
(`target = reference × value`, unitless). The `continuous` flag selects one of
two behaviors (§7.5.3):

- **`continuous = false`** — the target is resolved **once**, against the
  reference's speed when the action starts, then reached through the action's
  transition dynamics exactly like an absolute target.
- **`continuous = true`** — a controller keeps matching the reference **every
  step**; the action never ends on its own (only a superseding longitudinal
  action or a stop ends it). It must **not** be combined with a time- or
  distance-dimensioned transition — use `Step` (the default).

The reference's speed is read in the deterministic per-step order, so relative
targets are bit-identical across runs.

### Overlapping longitudinal actions

Each entity has at most one active longitudinal action. When a newer one lands —
an absolute ramp, a relative target, a profile, or a continuous target — it
**supersedes** the running one, which is retired and ends cleanly. This is the
minimal single-domain rule (§7.5.1); the full conflict/priority catalog is a
later sprint.

## Distance keeping

A `LongitudinalDistanceAction` makes an entity hold a **distance** [m] or a
**time gap** [s] to a reference entity — the two are mutually exclusive. It
belongs to the same longitudinal domain as `SpeedAction`, so it supersedes and
is superseded by the actions above.

- `freespace` chooses how the gap is measured: between the closest
  bounding-box points (needs geometry on both entities) or between reference
  points. It is the same measurement the interaction conditions use, so an
  action and a `DistanceCondition` never disagree.
- `displacement` chooses which side of the reference the actor holds:
  `trailingReferencedEntity` (behind, the default), `leadingReferencedEntity`
  (ahead), or `any` (keep whichever side it is on).
- `continuous=false` ends the action when the gap reaches its target;
  `continuous=true` keeps holding it and never ends on its own.
- `DynamicConstraints` (max acceleration, deceleration, speed) limit the
  controller; whichever is tighter, they or the entity's `Performance`
  envelope, wins. A missing constraint means unlimited.

The controller matches the reference's speed — which holds the gap where it is
— plus a term that closes the remaining error, bounded by the speed the entity
could still brake off within the error it has left. That bound is what keeps an
acceleration-limited approach from overshooting; close to the target the error
term takes over and the gap lands exactly. The entity never reverses: it stops
instead. A time gap is read against the actor's own speed, so a slower follower
targets a shorter gap.

A road-based `coordinateSystem`, or a freespace gap on an entity without a
bounding box, cannot be measured yet: the action reports an
`UnsupportedFeature` warning and ends rather than guessing (the
missing-prerequisite rule of §7.5.2.2).

## Lateral motion

The three `LateralAction` types move an entity **sideways**. Positive is always
the reference entity's **+y axis, which points left** (§6.3.4, ISO 8855); the
road and lane t-axes of §6.3.2 and §6.3.3 point left too, so the two agree.

### The lateral axis

Scena has no road network yet, so it stands one in: when a lateral action
starts, the engine captures a straight **lateral axis** through the entity along
its current heading, and the action displaces the entity **across** that axis by
a shaped offset ramp. Each step the entity advances `speed·dt` along the axis
and by the commanded offset increment across it, and its heading is the axis
heading turned by the angle those two legs subtend:

```
x += ds·cos(a) − do·sin(a)
y += ds·sin(a) + do·cos(a)
heading = a + atan2(do, ds)
```

This is **heading blending**: a lane change looks like steering because the
entity really does point where it is going. Three edge cases fall out of the
geometry and are deliberate, not artefacts:

- **No lateral step** leaves the heading on the axis. This also covers a
  zero-length step.
- **A lateral step with no forward travel** gives a ±π/2 crab angle. It is
  geometrically truthful: the entity is moving purely sideways.
- **A `Linear` shape kinks** at both endpoints, because its offset *rate* jumps
  from zero and back. Choose `Cubic` or `Sinusoidal` for a smooth entry and
  exit — those end with zero lateral rate, so the entity straightens out on its
  own.

When an action reaches its goal the heading snaps back to the axis and the axis
**dissolves**: the entity carries on straight through the offset it reached,
which is exactly what the composed model would have produced anyway. An entity
with no live axis integrates the plain straight-line model, unchanged.

A `TeleportAction` on an entity mid-manoeuvre **re-anchors** the axis through
the new position at the offset already reached, so the transition continues from
where the entity landed rather than dragging it back across the old axis.

### Lane changes without a road

A `LaneChangeAction` moves an entity to a **target lane**, given either
relative to an entity's current lane (a signed lane count, positive to its left,
the road centre lane not counted) or as an absolute **lane id**.

Without a road network there are no lanes, so "one lane over" means one
**default lane width** across the axis — `3.5 m`, configurable per engine with
`set_default_lane_width`. It is a modeling default, not a value the standard
prescribes: ASAM OpenSCENARIO leaves lanes to the road network, which is
external to it.

When the host provides an `IRoadQuery` that answers the lane queries
(`lane_width`, `lane_center_offset`, `relative_lane`), real lane geometry wins:
the actor's and the reference's lane positions are mapped, the target lane is
found `count` lanes over, and the difference of the two lane centre lines is the
offset. A backend that declines any one query falls back to the flat-world model
rather than failing.

An **absolute lane id** has no flat-world reading at all — it names an element of
the road network. Without a backend that resolves it the action reports an
`UnsupportedFeature` warning and completes without moving the entity sideways.
Unlike the road-coordinate-system deferrals this is *not* warned about at init:
a host with a backend can resolve absolute lanes, so only the runtime knows.

The transition follows the action's `TransitionDynamics`: `Time` and `Rate` over
seconds, `Distance` over **metres travelled** (so a slower actor takes longer but
changes lane over the same ground — and one at a standstill never finishes), and
`Step` as an instantaneous translate. `targetLaneOffset` is the offset to reach
on the target lane, where the action ends; the previous offset is not carried.

The target is resolved **once, at install**. §7.5.2.2 lists only "Target lane
exists" as this action's prerequisite, so losing the reference entity afterwards
leaves the committed target standing — unlike a `LateralDistanceAction`.

### Lane offsets and `maxLateralAcc`

A `LaneOffsetAction` moves an entity to a lateral **offset** — absolute (from
its own lane centre line) or relative to another entity's lane position — and
keeps it there if `continuous=true`.

It authors no duration. `LaneOffsetActionDynamics` gives only a shape and a
`maxLateralAcc`, so the duration is what makes the peak lateral acceleration of
the shaped ramp equal that limit. With `o(t) = Δ·g(t/T)` the lateral
acceleration is `Δ·g''(p)/T²`, and equating its peak to `a` gives:

| Shape | Peak `\|g''\|` | Duration |
|---|---|---|
| `Sinusoidal` | π²/2 | `T = π·√(Δ/2a)` |
| `Cubic` | 6 | `T = √(6Δ/a)` |
| `Linear` | (see below) | `T = 2·√(Δ/a)` |
| `Step` | — | `T = 0` |

`Linear` has zero curvature in its interior and unbounded kinks at its ends, so
curvature alone yields nothing. The documented substitute is the **minimum-time
rest-to-rest bound** for an acceleration-limited move of the same displacement —
accelerate at `a` for half the time, decelerate for the other half, giving
`Δ = a·T²/4`. It is the unique acceleration-limited lower bound and sits
continuously below the smooth shapes: `4 < π²/2 < 6`.

A **missing** `maxLateralAcc` means `inf` (the standard's reading), which makes
the transition instantaneous, as does a `Step` shape. A `maxLateralAcc` of
exactly `0` is rejected at init: it would permit no lateral motion at all, so
the target could never be reached.

With `continuous=true` the action never ends (§7.5.3). After arriving it
re-measures every step and installs a fresh transition whenever the target has
moved — which is how a relative offset tracks a reference that changes lane, and
how the offset survives a teleport.

### Lateral distance keeping

A `LateralDistanceAction` holds a lateral **distance** to a reference entity.
`freespace`, `continuous` and `DynamicConstraints` mean what they do for
longitudinal distance keeping; `displacement` chooses the side —
`leftToReferencedEntity`, `rightToReferencedEntity`, or `any` (keep whichever
side the actor is on).

The law is the same deadbeat-plus-glide controller, moved across to the lateral
rate: close the whole error this step, bounded by what the entity could still
shed within the error it has left. With **no** `DynamicConstraints` every limit
is infinite and the command closes the error outright — which is precisely what
the standard means by keeping the distance **rigid**. The `Performance` envelope
is deliberately *not* consulted here: its limits are longitudinal, while this
action's constraints are explicitly lateral acceleration, deceleration and
speed.

Two measurement details matter, both because the heading blend would otherwise
feed back into the error the controller is closing:

- In the **entity** coordinate system the lateral axis comes from the entity's
  *axis* heading, not its instantaneous yaw. The two agree whenever the entity
  is not mid-manoeuvre.
- `any` picks its side from the **reference-point** separation even for a
  freespace target, because a yawed vehicle presents a wider flank and a
  freespace gap could otherwise cross zero mid-manoeuvre and flip the side.

The **lane** coordinate system is accepted but cannot be honoured: §6.4.8.2.2
states outright that the lane-CS lateral distance is *undefined*. Scena warns at
init and again when the action runs, then completes it. Road and trajectory
coordinate systems, and a freespace gap without geometry, behave the same way
(§7.5.2.2).

### Overlapping lateral actions

The lateral domain has one owner per entity, like the longitudinal one. A new
lateral action **supersedes** the running one, and a `FollowTrajectoryAction`
supersedes either — all of them "assign a lateral control strategy" (Annex A
Table 10), and §7.5.2.1 names the trajectory-over-lane-change case explicitly.
It works in reverse too: a lateral action stops a running trajectory.

A lateral action superseding another **keeps the axis**, so an absolute lane
offset keeps meaning the same thing across the handover. A trajectory dissolves
it, because the follower writes positions outright.

The two domains are independent: a `SpeedAction` never supersedes a lane change,
and a lane change never disturbs a speed ramp.

## Teleport

A `TeleportAction` moves an entity to a target **position** instantaneously and
completes in the evaluation it fires — as an init action (to place an entity at
the start) or mid-run. Scena resolves the **world-frame** target
(`WorldPosition`, x/y/z in metres) today; the other §6.3.8 position variants
(lane, road, relative, …) and orientation arrive with the position resolver in a
later sprint, so a teleport currently leaves heading, pitch and roll unchanged.

## Simplifications

| Area | v0.0.1 model | Deferred |
|---|---|---|
| Position | Point-mass: explicit `x += speed·cos(heading)·dt`; z/pitch/roll untouched | Vehicle body dynamics; slope/3D integration |
| Speed target | Absolute + relative (`RelativeTargetSpeed`: delta/factor, one-shot + continuous) | — |
| Following mode | `position` shapes + hard Performance clamp | `follow` jerk-shaping, `DynamicConstraints` (accel/decel-rate) |
| Speed profile | Position-mode linear interpolation | `entityRef`-relative profile |
| Overlap | One active longitudinal action per entity; a later one supersedes and retires the earlier | Full cross-domain / bulk-actor §7.5 conflict resolution |
| Distance keeping | Distance and time-gap targets, freespace or reference-point gaps, `DynamicConstraints` + Performance clamping, one-shot and continuous | Jerk (rate) clamping; road-based coordinate systems |
| Lateral model | Kinematic: an offset ramp across a straight axis, heading blended from the two legs | Tyre dynamics, slip, yaw rate, steering actuation |
| Lanes | Flat world: a configurable default lane width, or a host `IRoadQuery` answering the lane queries | Real lane identity and geometry, absolute lane ids without a backend |
| Lateral start offset | The actor is assumed to sit on its lane centre when a lateral action starts | Real in-lane offset (needs a road backend) |
| Heading blending | Endpoint kinks on `Linear`; ±π/2 crab angle at zero speed | — (both are geometric properties, not approximations) |
| Lateral distance | Entity and world coordinate systems | Lane CS (undefined per §6.4.8.2.2), road and trajectory CS |
| Teleport | World-frame (`WorldPosition`) target; orientation untouched | Other position variants + orientation (position resolver) |
| Distance dimension | Explicit progress from accumulated travel | — |
| XML lowering | Built in code (IR / C ABI / Python) | XML frontend (P4/p5-s4) |

## Determinism

The transition math is closed-form and routes its only transcendental (the
`Sinusoidal` shape) through `det_cos`, so speeds are bit-identical across
platforms. A completed transition snaps exactly to its target rather than
leaving a last-ulp residue.

Lateral motion adds exactly two deterministic trig sites: one `det_sincos` when
the axis is captured (cached for every later step), and one `det_atan2` per step
for the heading blend. The `maxLateralAcc` duration derivation adds no
transcendental at all — `std::sqrt` is IEEE-exact. An entity with no live axis
takes the untouched straight-line branch, which is why every pre-lateral pinned
trace is bit-for-bit unchanged.

One caveat worth knowing: the lateral displacement accumulates as a telescoping
sum of per-step increments, so an entity that "arrives at 3.5 m" lands a few
ulps off it. That residue is deterministic and part of the pinned contract — it
is the same residue on every platform.

See [Determinism](determinism.md).

## Driving it from code

**C++** — build a `SpeedAction` with dynamics, or a `SpeedProfileAction`:

```cpp
using namespace scena::ir;
event.actions.push_back(std::make_shared<SpeedAction>(
    "ego", 20.0, TransitionDynamics{DynamicsShape::Cubic, DynamicsDimension::Time, 4.0}));
event.actions.push_back(std::make_shared<SpeedProfileAction>(
    "ego", std::vector<SpeedProfileEntry>{{10.0, 4.0}, {0.0, 4.0}}));
// Continuously hold the lead's speed + 3 m/s (never ends on its own):
event.actions.push_back(std::make_shared<SpeedAction>(
    "ego", RelativeTargetSpeed{"lead", 3.0, SpeedTargetValueType::Delta, /*continuous=*/true},
    TransitionDynamics{}));
init_actions.push_back(std::make_shared<TeleportAction>("ego", WorldPosition{0.0, 0.0, 0.0}));
```

Distance keeping and the routing actions have their own chapter:
[Routing, trajectories and controllers](routing.md).

**C ABI** — `scn_engine_add_speed_action_dyn`,
`scn_engine_add_speed_profile_action` (a negative entry `time` means
"unspecified"), `scn_engine_add_relative_speed_action`, and
`scn_engine_add_teleport_action`:

```c
scn_transition_dynamics d = {SCN_DYNAMICS_SHAPE_CUBIC, SCN_DYNAMICS_DIMENSION_TIME, 4.0,
                             SCN_FOLLOWING_MODE_POSITION};
scn_engine_add_speed_action_dyn(engine, "ego", 20.0, &d, 0.0, SCN_PRIORITY_PARALLEL, 1);

scn_transition_dynamics step = {SCN_DYNAMICS_SHAPE_STEP, SCN_DYNAMICS_DIMENSION_TIME, 0.0,
                                SCN_FOLLOWING_MODE_POSITION};
scn_engine_add_relative_speed_action(engine, "ego", "lead", 3.0, SCN_SPEED_TARGET_DELTA,
                                     /*continuous=*/1, &step, 0.0, SCN_PRIORITY_PARALLEL, 1);
scn_engine_add_teleport_action(engine, "ego", 0.0, 0.0, 0.0, 0.0, SCN_PRIORITY_PARALLEL, 1);

/* Hold 25 m behind the lead, bumper to bumper, for as long as the scenario runs.
   A negative distance/timeGap means "not used"; exactly one must be given. */
scn_dynamic_constraints limits = {2.0, -1.0, 3.0, -1.0, 40.0};
scn_engine_add_longitudinal_distance_action(engine, "ego", "lead", /*distance=*/25.0,
                                            /*time_gap=*/-1.0, /*freespace=*/1,
                                            /*continuous=*/1, SCN_COORDINATE_SYSTEM_ENTITY,
                                            SCN_LONGITUDINAL_DISPLACEMENT_TRAILING, &limits,
                                            0.0, SCN_PRIORITY_PARALLEL, 1);
```

**Python** — see [`python/examples/longitudinal.py`](../../python/examples/longitudinal.py)
and [`python/examples/relative_speed.py`](../../python/examples/relative_speed.py), and
[`python/examples/distance_keeping.py`](../../python/examples/distance_keeping.py)
for the traffic-jam approach:

```python
td = scn.TransitionDynamics(shape=scn.DynamicsShape.Cubic,
                            dimension=scn.DynamicsDimension.Time, value=4.0)
event.add_action(scn.SpeedAction("ego", 20.0, td))

target = scn.RelativeTargetSpeed("lead", 3.0, value_type=scn.SpeedTargetValueType.Delta,
                                 continuous=True)
event.add_action(scn.SpeedAction("ego", target, scn.TransitionDynamics()))
scenario.add_init_action(scn.TeleportAction("ego", scn.WorldPosition(0.0, 0.0, 0.0)))

event.add_action(scn.LongitudinalDistanceAction("ego", "lead", distance=25.0,
                                                freespace=True, continuous=True))
```

### Lateral actions

**C++**:

```cpp
using namespace scena::ir;
// One lane to the actor's own right, sinusoidal over 3 s:
event.actions.push_back(std::make_shared<LaneChangeAction>(
    "cutter", RelativeTargetLane{"cutter", -1},
    TransitionDynamics{DynamicsShape::Sinusoidal, DynamicsDimension::Time, 3.0}));
// Hold 1 m of lane offset, re-enforced for the rest of the run:
event.actions.push_back(std::make_shared<LaneOffsetAction>(
    "ego", AbsoluteTargetLaneOffset{1.0}, /*continuous=*/true, DynamicsShape::Cubic,
    /*max_lateral_acc=*/1.5));
// Keep 1.2 m of freespace clearance to the cutter's right:
event.actions.push_back(std::make_shared<LateralDistanceAction>(
    "ego", "cutter", 1.2, /*freespace=*/true, /*continuous=*/true,
    CoordinateSystem::Entity, LateralDisplacement::RightToReferencedEntity));
engine.set_default_lane_width(3.5); // what "one lane over" means without a road
```

**C ABI** — `scn_engine_add_relative_lane_change_action`,
`scn_engine_add_absolute_lane_change_action`,
`scn_engine_add_lane_offset_action`, `scn_engine_add_lateral_distance_action`
and `scn_engine_set_default_lane_width`:

```c
scn_transition_dynamics sine = {SCN_DYNAMICS_SHAPE_SINUSOIDAL, SCN_DYNAMICS_DIMENSION_TIME,
                                3.0, SCN_FOLLOWING_MODE_POSITION};
scn_engine_add_relative_lane_change_action(engine, "cutter", "cutter", /*lane_delta=*/-1,
                                           /*target_lane_offset=*/0.0, &sine, 7.0,
                                           SCN_PRIORITY_PARALLEL, 1);

/* A NULL reference means an absolute offset; a negative maxLateralAcc means
   "unset", which the standard reads as 'inf' (an instantaneous transition). */
scn_engine_add_lane_offset_action(engine, "ego", NULL, 1.0, /*continuous=*/1,
                                  SCN_DYNAMICS_SHAPE_CUBIC, /*max_lateral_acc=*/1.5, 0.0,
                                  SCN_PRIORITY_PARALLEL, 1);

/* NULL constraints is what the standard calls keeping the distance rigid. */
scn_engine_add_lateral_distance_action(engine, "ego", "cutter", /*distance=*/1.2,
                                       /*freespace=*/1, /*continuous=*/1,
                                       SCN_COORDINATE_SYSTEM_ENTITY,
                                       SCN_LATERAL_DISPLACEMENT_RIGHT, NULL, 14.0,
                                       SCN_PRIORITY_PARALLEL, 1);
scn_engine_set_default_lane_width(engine, 3.5);
```

**Python** — see [`python/examples/lane_change.py`](../../python/examples/lane_change.py)
for the full cut-in:

```python
sine = scn.TransitionDynamics(shape=scn.DynamicsShape.Sinusoidal,
                              dimension=scn.DynamicsDimension.Time, value=3.0)
event.add_action(scn.LaneChangeAction("cutter", scn.RelativeTargetLane("cutter", -1), sine))

event.add_action(scn.LaneOffsetAction("ego", scn.AbsoluteTargetLaneOffset(1.0),
                                      continuous=True, shape=scn.DynamicsShape.Cubic,
                                      max_lateral_acc=1.5))

event.add_action(scn.LateralDistanceAction(
    "ego", "cutter", distance=1.2, freespace=True, continuous=True,
    displacement=scn.LateralDisplacement.RightToReferencedEntity))

engine.set_default_lane_width(3.5)
assert engine.default_lane_width == 3.5
```
