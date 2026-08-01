# ADR-0024: `followingMode=follow` — jerk-aware transitions and dynamic constraints

- **Status:** accepted
- **Date:** 2026-08-01

## Context

ADR-0011 gave Scena its longitudinal model: a `TransitionDynamics` shape driven
over a time, distance or rate, and a default controller that stretches the
transition until its peak acceleration fits the actor's `Performance` envelope.
It implemented exactly one of the two `FollowingMode` literals. `follow` was
accepted and executed as `position`, and the jerk limits — `maxAccelerationRate`
and `maxDecelerationRate`, on `Performance` and on `DynamicConstraints` — were
parsed, stored and never read. `SpeedProfileAction` had neither its `entityRef`
nor its `DynamicConstraints`. Issue #62 tracked all of it.

Three pieces of normative text settle most of the design:

- **§FollowingMode.** `follow` means "follow the lateral and/or longitudinal
  target value as good as possible by observing the dynamic constraints of the
  entity (e.g. for a driver model)"; `position` means strict adherence to the
  shape. The authored shape is a *request* under `follow` and a *contract* under
  `position`.
- **§SpeedProfileAction / `followingMode`.** `position` is "strictly linear
  interpolation between speed target values"; `follow` applies "jerk (change
  rate of acceleration/deceleration) and other optional constraints of the
  Performance class of a Vehicle entity resulting in a smoother speed profile
  curve", and — decisively — **"for mode=follow the acceleration is zero at the
  start and end of the profile"**.
- **§SpeedProfileAction / `dynamicConstraints`.** "These settings has precedence
  over any Performance settings (applies to vehicles only)."

The open question was what "as good as possible" means for a deterministic
executor. A real driver model would run a control loop whose output depends on
its gains, and §7.6 of the DSL spec notwithstanding, no simulator's loop is
reproducible on another. Scena's contract is bit-identical results everywhere,
so `follow` has to be a **closed-form** construction, documented rather than
tuned.

## Decision

### Follow mode is a shape substitution plus a duration stretch

A `follow` transition is realised as a shape whose gradient is zero at both
endpoints, over the shortest duration that satisfies every effective limit and
is not shorter than the authored one.

- **`follow_shape`.** `Sinusoidal` is kept; `Linear`, `Step` and `Cubic` are all
  realised as `Cubic`. §SpeedProfileAction's "acceleration is zero at the start
  and end" excludes `Linear` and `Step` by construction — both step their
  gradient at the endpoints — and `Cubic` (smoothstep, `3p² - 2p³`) is the
  lowest-order polynomial that satisfies the requirement. It is what a linear
  ramp becomes when its corners are rounded. `Sinusoidal` already qualifies and
  is therefore left exactly as authored, so an author who chose it still gets
  it.
- **`constrained_duration`.** For a value change `Δ` realised with shape `g`
  over duration `T`, the peak first derivative is
  `shape_peak_gradient_factor(g)·|Δ|/T` and the peak second derivative is
  `shape_peak_jerk_factor(g)·|Δ|/T²`. Requiring both to stay inside the
  acceleration limit `a` and the jerk limit `j` gives
  `T ≥ max(authored, factor_g'·|Δ|/a, sqrt(factor_g''·|Δ|/j))`.
  `shape_peak_jerk_factor` is 6 for `Cubic` and π²/2 for `Sinusoidal`, and
  **infinity for `Linear` and `Step`**: no finite duration bounds the jerk of a
  shape that steps its gradient, so no jerk limit can be honoured by stretching
  one. `follow_shape` is what keeps that infinity out of the arithmetic;
  `constrained_duration` skips a non-finite factor rather than diverging, which
  is reachable only defensively.

Constraints may only **slow a transition down**. The authored dynamics are a
lower bound on the duration, never an upper one: a limit looser than the author
asked for changes nothing.

`position` mode is untouched — the authored shape is held exactly and only the
acceleration clamp of ADR-0011 applies. That is the whole observable difference,
and it is the one the standard describes.

### The Distance dimension keeps its position-mode reading

A distance-dimensioned transition measures progress in metres travelled, not
seconds, so an acceleration or jerk bound expressed per second has no duration
to stretch. Follow mode therefore substitutes the shape but leaves the distance
span as authored — the same choice ADR-0011 made for the acceleration clamp.

### `DynamicConstraints` on a `SpeedProfileAction` override, they do not merge

`overriding_limit` implements §SpeedProfileAction's precedence literally: a
present, positive constraint value wins outright, **including when it is looser
than the `Performance` envelope**, and only an absent or non-positive one falls
back to the envelope.

The distance-keeping actions keep `effective_limit`'s tighter-of-both reading.
Their §DynamicConstraints text states no precedence — it only says what the
distance controller "is allowed to use" — so nothing licenses letting an action
raise a vehicle's physical envelope there. Two helpers with two names, each
citing the clause it implements, beats one helper that is right in one place.

### An entity-relative speed profile is resolved once, at install

With `entityRef` set, each entry's `speed` is a delta on the referenced entity's
speed. Scena reads that reference **once, when the action starts**, and holds
the resulting absolute targets for the rest of the profile.

The alternative — re-reading the reference each step — would turn the action
into a tracking controller. But a `SpeedProfileAction` is a series of authored
targets *over time*; §SpeedProfileEntry's `time` is a delta from the previous
entry, and a moving reference would make those times describe a curve nobody
authored. `RelativeTargetSpeed` is the construct that offers continuous
tracking (§7.5.3), and it does so behind an explicit `continuous` flag. The
profile action has no such flag, so it gets the one-shot reading, matching a
non-continuous `RelativeTargetSpeed`.

The reference entity is a §7.5.2.2 prerequisite: unknown at init is a semantic
error, and gone at apply time stops the action, exactly as for a relative
`SpeedAction`.

### The maxSpeed rules are advisory, and stay advisory

`targetspeed_maxspeed_general` and `targetspeed_maxspeed_speedprofileaction` are
both phrased with "should", and both are scoped to `followingMode=follow`. Scena
warns with the rule id and clamps the target to `maxSpeed` anyway — the same
clamp position mode has always applied. Rejecting the scenario would fail a file
the standard permits.

`targetspeed_maxspeed_speedprofileaction` additionally scopes itself to "no
`entityRef` is specified", and Scena honours that: with an `entityRef` the entry
is a delta, and comparing a delta against a maximum speed is a category error.

## Consequences

- `follow` and `position` now produce genuinely different trajectories, and a
  `follow` transition is reproducible across platforms: the arithmetic is a
  polynomial, a square root and a division, all IEEE-exact, plus `det_cos` for
  the `Sinusoidal` shape.
- `ir::SpeedProfileAction` gains `entity_ref` and `constraints`. The original
  three-argument constructor is unchanged, so no existing call site moves; the
  C ABI adds `scn_engine_add_speed_profile_action_ex` alongside the original
  entry point rather than changing it.
- `ir::DynamicConstraints` moved earlier in `action.h` so the profile action can
  hold one by value. No field changed.
- `shape_peak_jerk_factor` and `lateral.h`'s `shape_peak_curvature_factor` agree
  on `Cubic` and `Sinusoidal` and deliberately differ on `Linear` and `Step`.
  The lateral one stands in as a minimum-time rest-to-rest bound for a lane
  offset that has no authored duration at all (ADR-0016), where an infinity
  would be useless; here the infinity carries the meaning. Both say so.
- Still deferred to #51: two longitudinal actions from different events
  competing for one entity. Follow mode changes nothing about who wins — the
  later action still supersedes — only how the winner's transition is shaped.

## Alternatives considered

**Run a jerk-limited control loop.** Physically the most faithful, and the way a
driver model would do it. Rejected: a loop's output depends on its gains and its
integration scheme, which puts it outside a contract that promises bit-identical
results on every platform. A closed-form stretch is reproducible and
explainable, and the standard asks for "as good as possible", not for a
particular controller.

**Report `follow` as unsupported.** Honest, and the ADR-0011 status quo in
spirit. Rejected because the standard describes the intended behavior precisely
enough to implement — "acceleration is zero at the start and end", constraints
observed — and a scenario that asks for a smoother curve is not asking for
anything Scena cannot deliver.

**Keep the authored shape and only stretch the duration.** Would leave a
`Linear` follow-mode ramp with unbounded jerk, so the jerk limits would remain
dead for exactly the shape most scenarios use, and it contradicts the
"acceleration is zero at the start and end" clause outright.

**One `shape_peak_curvature_factor` shared with the lateral module.** Tempting,
and it would remove two duplicated constants. Rejected: the two callers need
different answers for `Linear` and `Step`, and a single function that returns a
pragmatic bound in one place and a truth in the other is the kind of shared code
that gets misread later.
