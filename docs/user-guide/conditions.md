# Conditions

A trigger [condition](triggers.md) is a logical expression evaluated each
step. Scena implements two families from ASAM OpenSCENARIO XML §7.6.5: the
**by-value** conditions (§7.6.5.2, comparing a runtime value to a reference)
and the **by-entity** conditions (§7.6.5.1, observing one or more entities'
kinematics or position). This chapter covers both, and the host interface
they read.

> The kernel exposes these conditions and their host interface; lowering them
> from OpenSCENARIO XML is a P4 frontend concern and lands separately.

## By-value conditions

The **by-value** conditions compare a runtime value against a reference: the
simulation time, a named parameter or variable, an external value the host
injects, the simulated time of day, or the live state of another storyboard
element.

### The condition catalog

| Condition | Compares | Reads from |
|---|---|---|
| `SimulationTimeCondition` | simulation time to a value `[s]` | the clock (§8.4.7) |
| `ParameterCondition` | a parameter's value to a literal | scenario parameters (§9.1, immutable at runtime) |
| `VariableCondition` | a variable's value to a literal | the runtime variable store (§6.12), ≥1.2 |
| `UserDefinedValueCondition` | an external named value to a literal | host-injected values |
| `TimeOfDayCondition` | the simulated date-time to a reference | the time-of-day anchor |
| `StoryboardElementStateCondition` | an element's state/transition | the storyboard tree (§8.1–8.2) |

All comparisons go through one shared `Rule` operator.

## The Rule comparator

Every by-value condition carries a `rule` (ASAM `Rule` enumeration). The six
operators and how Scena applies them:

| Rule | Meaning |
|---|---|
| `equalTo` | `value == reference` |
| `notEqualTo` | `value != reference` |
| `greaterThan` | `value > reference` |
| `lessThan` | `value < reference` |
| `greaterOrEqual` | `value >= reference` |
| `lessOrEqual` | `value <= reference` |

The left operand is always the **stored/actual** value, the right operand the
condition's reference literal.

- **Numeric comparison is exact IEEE-754**, with no tolerance — the standard
  defines none, and an epsilon would tie the result to operand magnitude and
  break bit-identity. `NaN` follows IEEE ordering: `equalTo` and every
  ordering test are false, `notEqualTo` is true.
- **String operands** (Parameter/Variable/UserDefinedValue values are
  strings) are converted to scalars when both sides parse unambiguously
  (locale-independently, e.g. `5`, `16.667`, `+5`; never `1,5` or `1.5x`).
  When they do, the comparison is numeric. Otherwise only `equalTo` /
  `notEqualTo` apply, byte for byte; **ordering rules on non-scalar operands
  are always false** (per the ParameterCondition / VariableCondition clause).
  Scena warns at init when it can see this statically. There is no boolean
  coercion yet: `"true"` and `"1"` are distinct strings (typed declarations
  and coercion arrive with p4-s3).

## SimulationTimeCondition

Simulation time starts when the storyboard enters runningState (§8.4.7), so
`t = 0` is the init evaluation the engine issues during `init()`. A
`SimulationTimeCondition` with the default `greaterOrEqual` rule therefore
fires at exactly `t = value` and every step after — the classic
fire-at-value edge — and other rules behave as the table above.

## The host-value interface

Parameters, variables, user-defined values and the time-of-day anchor are the
seam between the host and the by-value conditions. Values read through a
read-only evaluation context (see
[ADR-0007](../architecture/ADR-0007-condition-evaluation-context.md)); a
condition that reads a value the context cannot supply evaluates to a
deterministic `false`.

| Value kind | Declared | Mutable at runtime | Cleared by `close()` |
|---|---|---|---|
| Parameter (§9.1) | scenario builder | no (constant over a run) | n/a |
| Variable (§6.12) | scenario builder | yes, via the host | yes |
| User-defined value | not declared | yes, any name | yes |
| Time of day | — (host-anchored) | advances with sim time | yes |

- **Parameters** are load-time constants: a `ParameterCondition`'s result is
  fixed for the whole run. A `parameterRef` the scenario does not declare is
  rejected at `init()` with `SemanticError`.
- **Variables** are seeded into the runtime store from their declarations at
  `init()` and changed with `set_variable`. Setting an **undeclared** name
  returns the `UnknownName` status and changes nothing; a dangling
  `variableRef` fails `init()` citing
  `asam.net:xosc:1.2.0:reference_control.resolvable_variable_reference`.
- **User-defined values** are external, so any name is accepted. A value may
  be staged **before** `init()` (visible at the `t = 0` evaluation) and
  persists across `init()`. Reading a name the host never set is a
  deterministic `false` and the engine warns **once** per name.

### C++

```cpp
scena::Engine engine;
engine.set_user_defined_value("sensor", "12"); // may be staged pre-init
engine.init(std::move(scenario));
engine.set_variable("gate", "open");           // UnknownName if undeclared
auto v = engine.variable("gate");              // std::optional<std::string>
```

### C

```c
scn_engine_set_parameter(e, "speedLimit", "30");   /* builder, next init */
scn_engine_declare_variable(e, "gate", "closed");  /* builder, next init */
scn_engine_init(e);
scn_engine_set_variable(e, "gate", "open");        /* SCN_ERROR_UNKNOWN_NAME if undeclared */
const char* value = NULL;
scn_engine_get_variable(e, "gate", &value);        /* borrowed until the next get/mutate */
```

The C getters return a **borrowed** string valid until the next
`scn_engine_get_*` or any mutating call on the same engine; an
undeclared/unset name returns `SCN_ERROR_UNKNOWN_NAME` and leaves the
out-parameter untouched.

### Python

```python
scenario.set_parameter("speedLimit", "30")
scenario.declare_variable("gate", "closed")
engine.init(scenario)
assert engine.set_variable("gate", "open") == scn.Status.Ok
assert engine.set_variable("ghost", "1") == scn.Status.UnknownName
engine.set_user_defined_value("sensor", "12")
```

## Time of day

`TimeOfDayCondition` compares the **simulated** instant to a reference
date-time. The host anchors the clock with `set_date_time`: the given
date-time is taken to hold at the current simulation instant and advances
one-for-one with simulation time thereafter (`anchor_epoch + (t − anchor_sim)`).
No wall clock is ever read.

- The comparison is over epoch seconds, so the reference's ISO-8601 UTC offset
  is folded in: `13:00+01:00` equals `12:00Z`.
- `DateTime` canonicalizes with civil-day integer math (C++20 `std::chrono`),
  so leap years are honored and an out-of-range value (Feb 30, hour 24, …) is
  rejected — by the setter with `InvalidArgument`, at init with
  `ValidationError`.
- Until an anchor is set the condition is a deterministic `false` and the
  engine warns once. The same anchor will later be fed by the
  EnvironmentAction time-of-day clock (p5-s6).

## Storyboard element state

`StoryboardElementStateCondition` holds when a referenced element is in a
runtime **state** or performs a **transition** at this discrete time (§8.1–8.2):

- **States** — `standbyState`, `runningState`, `completeState` — are level
  triggered: they hold for as long as the element is in that state.
- **Transitions** — `startTransition`, `endTransition`, `stopTransition`,
  `skipTransition` — are **one-evaluation pulses**: they hold only in the
  evaluation the transition occurs. `skipTransition` is Event-only; on any
  other element type it can never occur (Scena warns and evaluates false).

### The transition window and document order

Because the storyboard is walked once per step in document order, a
transition is observable only within that single evaluation, and only by
elements evaluated *after* the one that transitions:

- An **earlier**-evaluated condition does not see a transition a **later**
  element performs in the same evaluation.
- Because **stop is processed before start** at every level (see
  [Triggers → stop before start](triggers.md#evaluation-order-stop-before-start)),
  a `stopTransition` *is* visible to a later element's start trigger in the
  same evaluation.

A delay shifts the whole pulse: a delayed transition literal replays the
one-evaluation pulse `Δt` later, exactly as a delayed edge does.

### Name resolution

`storyboardElementRef` is a **nameRef**, not a path. A globally unique name is
used bare; a name shared across scopes is disambiguated with `::` prefixes of
directly enclosing element names, most-enclosing last
(`first::leg` selects the `leg` under maneuver `first`). Resolution must be
**unique**: a reference that matches zero elements (dangling, or an element of
a different type) or several (ambiguous) fails `init()` with `SemanticError`
citing `asam.net:xosc:1.0.0:reference_control.resolvable_storyboard_element_ref`.
Per-action nodes do not exist yet, so an `action`-typed reference warns and
evaluates false (p5-s4).

## Determinism

Every by-value condition is a pure function of the evaluation context, and the
host interface introduces no nondeterminism: two engines fed the same scenario
and the same host-setter sequence produce bit-identical entity states and an
identical, ordered diagnostic stream. The warn-once messages name the offending
value only — never a floating-point or date value, which would be
locale-sensitive. See [Determinism](determinism.md) and
[ADR-0007](../architecture/ADR-0007-condition-evaluation-context.md).

## By-entity conditions

The **by-entity** conditions (§7.6.5.1, `ByEntityCondition`) observe one or
more entities' kinematics or position. Every one names a set of
**triggering entities** and reduces a per-entity logical expression over them.

### Triggering entities: any / all

A by-entity condition carries a `TriggeringEntities` set — one or more entity
references and a rule. The per-entity expression is evaluated **independently
for every entity**, then reduced:

| Rule | Holds when |
|---|---|
| `any` | at least one triggering entity satisfies the expression |
| `all` | every triggering entity satisfies it |

The reduction happens **inside** the condition, before the trigger's
edge/delay machinery (§7.6.5.1). Every entity is evaluated — there is no
short-circuit — so the outcome never depends on evaluation order. An entity
the engine cannot observe contributes `false` for itself only, so `any` can
still hold through another reference (the ADR-0007 absent⇒false contract at
its finest grain). Dangling references are rejected at `init()` with
`SemanticError`.

### The condition catalog

| Condition | Holds when |
|---|---|
| `SpeedCondition` | the entity's speed compares to a value under a `Rule` |
| `RelativeSpeedCondition` | the entity's speed relative to a reference entity compares to a value |
| `AccelerationCondition` | the entity's acceleration compares to a value |
| `StandStillCondition` | the entity has stood still for `duration` seconds (`>=`) |
| `TraveledDistanceCondition` | the entity has traveled `value` meters of path (`>=`) |
| `ReachPositionCondition` | the entity is within `tolerance` of a position (2D) |

Speed, RelativeSpeed and Acceleration take a `Rule`; the other three have no
rule and use an exact `>=` / `<=`, so a `0` threshold holds immediately.

### The scalar-velocity model

Scena's `EntityState` carries a scalar `speed` along the entity's heading — no
full velocity vector yet. The measurements follow from that (they generalize
when a velocity vector lands):

- **Total speed / acceleration** (no `direction`) is the magnitude, `|value|`.
- **Directional** (`DirectionalDimension`, ISO 8855 body axes): `longitudinal`
  is the signed value; `lateral` and `vertical` are exactly `0.0`.
- **RelativeSpeed** without direction is the spec's
  `speed_rel = speed(triggering) − speed(reference)` — a signed difference of
  total speeds, not the magnitude of the relative velocity. With a direction,
  the relative velocity is projected in the **triggering** entity's frame,
  through the deterministic `det_sincos` (the only trigonometry here).
- **ReachPosition** uses the 2D horizontal distance `sqrt(dx²+dy²)` — the spec
  calls `tolerance` the "radius of tolerance circle", so `z` is ignored.
  It is deprecated (1.2, superseded by `DistanceCondition`): using it emits a
  `DeprecatedFeature` warning, and the condition still evaluates.

### How the engine observes entities

The derived quantities are refreshed once per `step(dt)`, between the host
poll and the storyboard evaluation, from the snapshots the conditions
actually observe:

- **Acceleration** is a finite difference `(speed − prev_speed)/dt`; until two
  samples exist it is **absent**, so an AccelerationCondition is false for
  every rule (including `lessThan` / `notEqualTo`).
- **Traveled distance** is the cumulative world-frame path length since init
  (Euclidean displacement per step) — path length, not straight-line
  displacement.
- **Standstill** accumulates `dt` while speed is **exactly** `0.0` and resets
  on any motion. The spec is silent on a threshold and Scena invents none.

A `dt == 0` step is skipped entirely (no `0/0`, the previous sample is kept).
The baseline is seeded at `init()` after the init actions, so an init-action
speed is the reference — there is no phantom acceleration or distance at
`t = 0`. Host- and engine-controlled entities are treated identically; the
existing one-step observation lag for engine-controlled motion (integration
runs later in the step) is unchanged. This is the same for both control modes
and is documented on `Engine::step` and in
[ADR-0008](../architecture/ADR-0008-entity-kinematics-observation.md).

### Interaction metrics (distance, headway, collision)

A second group of by-entity conditions measures **between** two entities, or an
entity and a position (§6.4):

| Condition | Holds when |
|---|---|
| `DistanceCondition` | the distance to a position compares to a value |
| `RelativeDistanceCondition` | the distance to a reference entity compares to a value |
| `TimeHeadwayCondition` | distance ÷ the entity's own speed compares to a value |
| `TimeToCollisionCondition` | distance ÷ closing speed compares to a value |
| `CollisionCondition` | the entity's bounding box intersects a reference entity's |
| `EndOfRoadCondition` / `OffroadCondition` | the entity is at the road end / off-road for a `duration` |
| `RelativeClearanceCondition` | the surrounding lanes are clear of other entities |

The distance is selected by two attributes: `coordinateSystem` (`entity`
default, or `world`) and `relativeDistanceType` (`euclidianDistance` default,
`longitudinal`, or `lateral`). Euclidean is the coordinate-system-independent
straight line (3D for reference points, §6.4.3); longitudinal/lateral project
onto the effective axis (§6.4.4). `freespace` switches from the entity
**origin** to the closest **bounding-box** points (§6.4.7): give an entity a
`BoundingBox` and the freespace metrics and `CollisionCondition` use its
2D footprint (a heading-rotated rectangle; touching boxes count as a
collision).

Key rules Scena implements:

- **TimeHeadway** divides by the *triggering* entity's speed only (the
  reference is assumed leading); a stopped or reversing follower is false.
- **TimeToCollision** divides by the closing speed (no acceleration); if the
  entities are moving apart, the closing speed is ≤ 0 and the condition is
  false. The target is a reference entity **xor** a position.
- A missing `BoundingBox` makes the freespace metrics and `CollisionCondition`
  false for that entity (geometry is optional for now).

> **Road-dependent modes evaluate to a deterministic false until p3-s4.**
> `EndOfRoad`, `Offroad`, `RelativeClearance`, and any `road` / `lane` /
> `trajectory` coordinate system need a road network (`IRoadQuery`), which is
> not wired yet. These land as full conditions — they parse, validate, and
> bind — but evaluate to false and emit an `UnsupportedFeature` warning at
> `init()` citing the spec section. `CollisionCondition` matches an entity
> reference only; a by-object-type target arrives with the entity taxonomy
> (p2-s1).

### Building by-entity conditions in Python

```python
import scena as scn

ego = scn.TriggeringEntities(["ego"])                       # rule defaults to Any
faster_than_lead = scn.RelativeSpeedCondition(ego, "lead", 2.0, scn.Rule.GreaterOrEqual)
parked = scn.StandStillCondition(scn.TriggeringEntities(["parked"]), 1.0)
reached = scn.ReachPositionCondition(ego, scn.WorldPosition(10.0, 0.0, 0.0), 0.5)

# Interaction metrics: give the entities a bounding box for freespace/collision.
boxed = scn.Entity("ego", "ego", scn.ControlMode.HostControlled,
                   bounding_box=scn.BoundingBox(length=4.0, width=2.0))
near = scn.DistanceCondition(ego, scn.WorldPosition(20.0, 0.0, 0.0), 5.0,
                             False, scn.Rule.LessOrEqual)      # freespace=False
ttc = scn.TimeToCollisionCondition(ego, 3.0, False, scn.Rule.LessOrEqual, entity_ref="lead")
crash = scn.CollisionCondition(ego, "lead")
```

Complete examples live in
[`python/examples/byentity_conditions.py`](../../python/examples/byentity_conditions.py)
and
[`python/examples/interaction_conditions.py`](../../python/examples/interaction_conditions.py).

> **Deferred (P4 / p3-s4 / p2-s1):** the by-entity conditions are cartesian-only
> in the kernel. Road-coordinate measurement, the road-topology predicates, and
> `ReachPosition`/`Distance` against the full Position variants arrive when the
> PositionResolver (#18) and road plumbing (#23) land; the full bounding-box
> taxonomy and by-object-type collisions arrive with p2-s1 (#15); XML lowering
> arrives with the P4 frontend. See
> [ADR-0009](../architecture/ADR-0009-interaction-metrics.md).
