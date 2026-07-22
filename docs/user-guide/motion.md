# Motion — longitudinal dynamics

This chapter covers how an engine-controlled entity's **speed** changes over
time: the `SpeedAction` transition model, the default controller and its
`Performance` clamp, and the `SpeedProfileAction` follower. It follows ASAM
OpenSCENARIO XML 1.4.0 §7.4.1.2 (SpeedAction / TransitionDynamics) and
§SpeedProfileAction.

Lateral motion (lane changes, offsets) and road-relative positioning arrive in
later sprints; this chapter is the longitudinal half.

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

## Simplifications

| Area | v0.0.1 model | Deferred |
|---|---|---|
| Position | Point-mass: explicit `x += speed·cos(heading)·dt`; z/pitch/roll untouched | Vehicle body dynamics; slope/3D integration |
| Speed target | Absolute speed | Relative-to-entity target (`RelativeTargetSpeed`) |
| Following mode | `position` shapes + hard Performance clamp | `follow` jerk-shaping, `DynamicConstraints` (accel/decel-rate) |
| Speed profile | Position-mode linear interpolation | `entityRef`-relative profile |
| Overlap | One active longitudinal action per entity; a later one supersedes | Concurrent-action conflict resolution |
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
```

**C ABI** — `scn_engine_add_speed_action_dyn` and
`scn_engine_add_speed_profile_action` (a negative entry `time` means
"unspecified"):

```c
scn_transition_dynamics d = {SCN_DYNAMICS_SHAPE_CUBIC, SCN_DYNAMICS_DIMENSION_TIME, 4.0,
                             SCN_FOLLOWING_MODE_POSITION};
scn_engine_add_speed_action_dyn(engine, "ego", 20.0, &d, 0.0, SCN_PRIORITY_PARALLEL, 1);
```

**Python** — see [`python/examples/longitudinal.py`](../../python/examples/longitudinal.py):

```python
td = scn.TransitionDynamics(shape=scn.DynamicsShape.Cubic,
                            dimension=scn.DynamicsDimension.Time, value=4.0)
event.add_action(scn.SpeedAction("ego", 20.0, td))
```
