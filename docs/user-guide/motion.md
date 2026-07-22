# Motion — longitudinal dynamics and teleport

This chapter covers how an engine-controlled entity's **speed** changes over
time — the `SpeedAction` transition model (absolute and relative targets), the
default controller and its `Performance` clamp, and the `SpeedProfileAction`
follower — and the instantaneous **`TeleportAction`**. It follows ASAM
OpenSCENARIO XML 1.4.0 §7.4.1.2 (SpeedAction / TransitionDynamics),
§SpeedProfileAction, §RelativeTargetSpeed and §TeleportAction.

Lateral motion (lane changes, offsets) and road-relative positioning arrive in
later sprints; this chapter is the longitudinal half plus world-frame teleport.

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
| Teleport | World-frame (`WorldPosition`) target; orientation untouched | Other position variants + orientation (position resolver) |
| Distance dimension | Explicit progress from accumulated travel | — |
| XML lowering | Built in code (IR / C ABI / Python) | XML frontend (P4/p5-s4) |

## Determinism

The transition math is closed-form and routes its only transcendental (the
`Sinusoidal` shape) through `det_cos`, so speeds are bit-identical across
platforms. A completed transition snaps exactly to its target rather than
leaving a last-ulp residue. See [Determinism](determinism.md).

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
```

**Python** — see [`python/examples/longitudinal.py`](../../python/examples/longitudinal.py)
and [`python/examples/relative_speed.py`](../../python/examples/relative_speed.py):

```python
td = scn.TransitionDynamics(shape=scn.DynamicsShape.Cubic,
                            dimension=scn.DynamicsDimension.Time, value=4.0)
event.add_action(scn.SpeedAction("ego", 20.0, td))

target = scn.RelativeTargetSpeed("lead", 3.0, value_type=scn.SpeedTargetValueType.Delta,
                                 continuous=True)
event.add_action(scn.SpeedAction("ego", target, scn.TransitionDynamics()))
scenario.add_init_action(scn.TeleportAction("ego", scn.WorldPosition(0.0, 0.0, 0.0)))
```
