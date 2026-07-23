# Global and infrastructure actions

This chapter covers the actions that have no actor: the ASAM OpenSCENARIO XML
1.4.0 §7.4.2 `GlobalAction` family — `VariableAction`, the deprecated
`ParameterAction`, `EntityAction`, `EnvironmentAction` and
`InfrastructureAction` (traffic signals) — plus the §7.4.3
`UserDefinedAction` / `CustomCommandAction`.

Every action here **completes in the evaluation it fires** (Annex A Tables 11
and 12). None of them assigns a control strategy, so none consumes simulation
time; what they do instead is write engine-level state that stays until
something replaces it.

The two signal conditions that read that state — `TrafficSignalCondition` and
`TrafficSignalControllerCondition` — are documented in
[Conditions](conditions.md#traffic-signals).

## Actions without an actor

A private action names the entity it drives. A global action names nothing:
"the `GlobalAction` type is used to set or modify non-entity-related
quantities" (§7.4.2). In the IR these derive from `ir::GlobalAction`, whose
`entity_id()` is finalized to the empty string so existing call sites keep a
valid reference:

```cpp
class GlobalAction : public Action {
public:
    [[nodiscard]] const std::string& entity_id() const final;  // always ""
};
```

The engine branches on the **type**, never on the empty string — an entity id
is scenario content and must not carry meaning by being empty. In Python the
same distinction is an `isinstance` check:

```python
isinstance(scn.VariableSetAction("v", "1"), scn.GlobalAction)  # True
isinstance(scn.SpeedAction("ego", 10.0), scn.GlobalAction)     # False
```

Global actions are legal both in the init phase (§8.5) and in storyboard
events, and both routes reach the same engine code.

## Variables

Variables (§6.12) are the one named-value namespace that changes during a run.
They are declared with an initialization value, seeded into the runtime store
at `init()`, and written either by the host (`Engine::set_variable`) or by the
scenario itself:

| Action | Effect |
|---|---|
| `VariableSetAction` | `variables[ref] = value` |
| `VariableModifyAction` | `variables[ref] = ref <add\|multiplyByValue> value` |

A `VariableCondition` reads the same store, so a scenario can close a feedback
loop with no host in the middle:

```python
scenario.declare_variable("armed", "false")
maneuver.add_event(timed_event("arm", 5.0, scn.VariableSetAction("armed", "true")))
maneuver.add_event(gated_event(
    "act-on-it",
    scn.VariableCondition("armed", scn.Rule.EqualTo, "true"),
    scn.SpeedAction("ego", 0.0),
))
```

### Exact arithmetic

`VariableModifyAction` applies **one fixed IEEE expression** per operator and
writes the result back through `ir::format_scalar`, the shortest decimal that
round-trips bit for bit. There is no rounding at the store boundary:

```python
scenario.declare_variable("counter", "0.1")
# … VariableModifyAction("counter", ModifyOperator.Add, 0.2) …
engine.variable("counter")          # "0.30000000000000004"
float(engine.variable("counter")) == 0.1 + 0.2   # True
```

Rule `data_type.variable_modification_or_comparison_possible` (Annex C.2.6)
restricts modification to numeric types. Scena has no typed declarations yet
(they arrive with p4-s3), so scalar-convertibility stands in for the type
check: a non-numeric current value reports the rule **once** as a Warning and
leaves the variable untouched, rather than ending the run.

An undeclared `variableRef` is rejected at `init()` with
`Status::SemanticError`, citing rule
`reference_control.resolvable_variable_reference`.

### The deprecated parameter actions

`ParameterSetAction` and `ParameterModifyAction` were deprecated with 1.2 in
favour of the variable actions, because parameters became immutable at runtime
(§9.1). A 1.0 or 1.1 file may still contain them, and Scena executes them: they
write a **runtime overlay** that the `ParameterCondition` lookup consults ahead
of the declared value.

That keeps both promises at once — a legacy file's parameter action is visible
to the condition that reads it, and §9.1 immutability stands for 1.2+ content,
which cannot contain these actions at all. The overlay is per-run state and is
cleared by `init()` and `close()`. Each name warns once with
`Status::DeprecatedFeature`.

## The entity lifecycle

`AddEntityAction` and `DeleteEntityAction` take a declared entity in and out of
the running scenario. Both target an entity by `entityRef`, which resolves
against the **declaration** in the Entities section, not against a live
instance — §7.4.2 is explicit that "entities to be added or deleted must be
defined in the Entities section", and that "an entity can only exist in one
copy. Adding an already active entity will have no effect, neither will
deleting an already inactive entity."

Scena models this as an `active` flag on the entity record, never as an erase
or insert. The entity map's structure does not change at runtime, so iteration
order and per-record bookkeeping — the two things the determinism contract
rests on — are unaffected.

### What a delete clears

A deleted entity is skipped by the host-state poll, the observation refresh,
the motion integrator and the publish. It also loses every piece of runtime
state, so a later add restarts from a clean slate:

| Cleared | Kept |
|---|---|
| longitudinal ownership, continuous targets, retired actions | control mode |
| trajectory follower | bounding box |
| route, assigned controller, per-domain activation, visibility | performance limits |
| acceleration, odometer, standstill timer | the declaration itself |

An `AddEntityAction` installs a fresh state at its world position and seeds the
observation baseline from it, exactly as `init()` does after the init actions —
so the first step after a respawn reports no phantom acceleration and no
teleport-sized odometer jump.

### What the host and the conditions see

They see the same thing, which is the point:

```python
engine.entity_active("hazard")   # True → False after the delete
engine.state("hazard")           # None while inactive
engine.visibility_of("hazard")   # None while inactive
engine.report_state("hazard", s) # Status.UnknownEntity while inactive
```

`entity_active` distinguishes the two ways an id can fail: `None` means the
scenario never declared it, `False` means a delete removed it.

Every by-entity condition over an inactive entity is a deterministic false —
the entity-kinematics facet is simply absent, the ADR-0007 grain.

### Running actions stop

§7.5.2.2 makes "the actor is a valid entity" a general prerequisite of every
private action, and §7.5.2.1 names the disappearing-reference case explicitly.
So a delete stops whatever was driving the entity, and also stops a
`LongitudinalDistanceAction` or a relative `SpeedAction` whose **reference**
entity disappeared.

One documented deviation: the standard has the action itself take a
`stopTransition`. Scena has no per-action observable transitions, so the stop
surfaces as the **owning event** ending one evaluation later, when the
scheduler re-polls the action. The observable end state is the same; only the
granularity differs (ADR-0015).

## Environment

`EnvironmentAction` merges weather, road condition and time of day into the
engine's environment store. §Environment gives the merge its semantics — "if
one of the conditions is missing it means that it doesn't change" — so an
action updates only the members it carries, at both levels:

```python
first = scn.Environment()
weather = scn.Weather()
weather.fog = scn.Fog(120.0)
weather.temperature = 288.0
first.weather = weather

second = scn.Environment()
rain = scn.Weather()
rain.precipitation = scn.Precipitation(scn.PrecipitationType.Rain, 4.5)
second.weather = rain
# After both actions: fog and temperature survive, precipitation is added.
```

Scena couples the environment to **no physics and no rendering** — the engine
has neither. Weather, sun, wind and friction are state a host reads back
through `Engine::environment` and acts on itself. That is a deliberate
simplification, recorded in the coverage matrix.

### The time-of-day clock

`TimeOfDay` is the one member with runtime meaning: it re-anchors the same
simulated clock `Engine::set_date_time` drives, and its `animation` flag
decides whether that clock runs.

| `animation` | Simulated instant |
|---|---|
| `true` | anchor + elapsed simulation time (advances one-for-one) |
| `false` | the anchor, forever |

A frozen clock is bit-identical every step, so a `TimeOfDayCondition` with a
reference in the future never becomes true. The condition itself is unchanged
by any of this — it reads whatever instant the context reports.

The host setter always means an *advancing* clock, so calling
`set_date_time` un-freezes a pinned anchor.

## Traffic signals

A `TrafficSignalController` (§6.11.2) applies a cycle of `Phase`s to one or
more signals. Each phase has a name (unique within its controller), a duration,
and the observable states it writes:

```python
scenario.add_traffic_signal_controller(scn.TrafficSignalController(
    "straight",
    phases=[
        scn.Phase("stop",    30.0, [scn.TrafficSignalState("signal-1", "red")]),
        scn.Phase("caution",  2.5, [scn.TrafficSignalState("signal-1", "red;amber")]),
        scn.Phase("go",      17.5, [scn.TrafficSignalState("signal-1", "green")]),
    ],
))
```

Signal ids and state strings are **opaque**: an id names an element of the road
network file, and "interpretation and notation of `state` are specific to the
simulation engine used" (§6.11.4). Scena stores and compares them byte for
byte and never reads meaning into either.

### The cycle clock

The cycle duration is the sum of the phase durations, and "the first `Phase`
repeats after the last `Phase` has ended" (§6.11.4). Scena derives the current
phase **arithmetically**, not by accumulating across steps:

```
local = fmod(t - anchor, total)   →  a linear scan of the cumulative offsets
```

`fmod` is exact in IEEE, so the phase at time `t` depends only on `t`. Two
engines that reach the same instant through different step sequences are in the
same phase — a host is free to change its step rate mid-run.

Phase intervals are **half-open**: at exactly the cumulative duration of phase
*i*, the cycle has already moved into phase *i+1*. (This is the one place the
lookup differs from trajectory segments, which own both endpoints.) A
zero-duration phase therefore occupies no time at all.

A cycle whose durations are all zero has no length to advance through: the
first phase is held for the whole run, and the load warns.

### Chained controllers

A controller may declare a `delay` to a `reference` controller, so that "its
first phase virtually starts *delaytime* seconds after the start of the
reference's first phase" (§6.11.3) — a progressive signal system. Delays
compose along the chain.

Before its start offset elapses a controller has **no phase**: it drives
nothing, `traffic_signal_controller_phase` returns nothing, and a condition
naming its first phase is a deterministic false.

`delay` requires `reference`. An unresolvable reference (rule
`reference_control.traffic_signal_controller_references`, C.7.13), a cycle in
the chain, a duplicate controller or phase name, and a negative phase duration
(rule `data_type.phase_duration_positive`, C.2.3) are all rejected at `init()`.

### Actions win, until the clock moves on

The controllers tick at the **top** of the storyboard-evaluation phase and
write **on transition only** — a phase writes its states once, when the cycle
enters it. Two consequences, and together they reproduce the §11.12 worked
example exactly:

* `TrafficSignalStateAction` fires after the tick, so a forced state overrides
  the cycle's;
* because the controller does not rewrite the signal every step, that forced
  state **stands** until the controlling cycle's next phase transition
  reclaims the signal.

That is what makes a simulated bulb failure work: force `"red;green"` onto a
signal and it stays broken for the rest of the phase.

`TrafficSignalControllerAction` restarts a cycle at a named phase: it
re-anchors the controller so the current instant sits exactly at that phase's
start, and ticks that one controller immediately — so conditions evaluated
later in the same storyboard walk agree with it. The cycle continues from there
in declared order.

Init ordering follows the same precedence: controllers seed their first phases
*before* the init actions run, so an init-phase state action wins over the seed.

## Custom commands

`CustomCommandAction` (§7.4.3) hands a `type` and a `content` to the host
through the gateway. Both are opaque — they are "defined as a contract between
the simulation environment provider and the author of a scenario" — so the
engine passes them verbatim and interprets neither:

```cpp
virtual void on_custom_command(const std::string& type, const std::string& content) {}
```

The callback is defaulted, like the other p5-s5 hand-offs, so a gateway written
against the older interface keeps compiling. Without a gateway the action is a
**silent no-op**, and deliberately emits no diagnostic: §7.4.3 makes
executability depend on "the ability of the specific simulation environment
recognizing these actions", so a host that does not implement it is the
documented contract, not a defect.

As with every gateway callback, the host must not call back into the engine
from it; reactions feed back through the sanctioned setters between steps.

## Driving it from code

### C++

```cpp
scena::ir::Scenario scenario;
scenario.variables["armed"] = "false";
scenario.traffic_signal_controllers.push_back(controller);
scenario.init_actions.push_back(
    std::make_shared<scena::ir::EnvironmentAction>(environment));

scena::Engine engine;
engine.init(std::move(scenario));
engine.traffic_signal_state("signal-1");             // std::optional<std::string>
engine.traffic_signal_controller_phase("straight");  // std::optional<std::string>
engine.entity_active("hazard");                      // std::optional<bool>
const scena::ir::Environment& env = engine.environment();
```

### C

The C ABI ships the builders and the small getters; the trailing
`at_time` / `priority` / `maximum_execution_count` follow the `_ex` convention
of the private-action builders.

```c
scn_engine_add_variable_set_action(engine, "armed", "true", 5.0, SCN_PRIORITY_PARALLEL, 1);
scn_engine_add_delete_entity_action(engine, "hazard", 4.0, SCN_PRIORITY_PARALLEL, 1);

const scn_traffic_signal_state states[] = {{"signal-1", "red"}};
const scn_signal_phase phases[] = {{"stop", 30.0, states, 1}};
scn_engine_declare_traffic_signal_controller(engine, "straight", -1.0, NULL, phases, 1);

/* Zero-initialize: every has_* flag off means "nothing changes". */
scn_environment environment = {0};
environment.has_weather = 1;
environment.has_fog = 1;
environment.fog_visual_range = 120.0;
scn_engine_add_environment_action(engine, &environment, 1.0, SCN_PRIORITY_PARALLEL, 1);

const char* state = NULL;
scn_engine_traffic_signal_state(engine, "signal-1", &state);  /* borrowed */
int active = 0;
scn_engine_entity_active(engine, "hazard", &active);
```

Borrowed strings follow the `scn_engine_get_variable` lifetime rules: valid
until the next such call or any mutating call on the same engine.

Full environment read-back over the C ABI is deferred to the p6-s1 C-API
expansion; C++ and Python have it today.

### Python

```python
engine.variable("armed")
engine.entity_active("hazard")
engine.traffic_signal_state("signal-1")
engine.traffic_signal_controller_phase("straight")
engine.environment.weather.fog.visual_range
engine.date_time
```

Runnable examples: [`global_actions.py`](../../python/examples/global_actions.py)
and [`traffic_signals.py`](../../python/examples/traffic_signals.py).

## Not yet modeled

* **XML lowering.** The kernel executes all of this; loading it from a `.xosc`
  file arrives with the XML frontend in P4 (the standing precedent since
  p5-s1).
* **`TrafficAction`** — sources, sinks, swarms and traffic areas — is out of
  scope for this sprint.
* **1.4-only constructs** are outside the targeted 1.0–1.3 versions:
  `SetMonitorAction`, the `Phase` `semantics` attribute
  (`TrafficSignalSemantics`), and `TrafficSignalGroupState`.
* **Road-network validation of signal ids.** Rules C.7.10, C.7.14 and C.7.15
  check that a signal id names a real element of the road network file, which
  Scena cannot reach until p3-s4. Ids stay free-form; an id nothing writes
  warns once at evaluation and reads as false.
* **Environment coupling.** Weather, sun, fog, wind and friction are stored,
  not applied — no visibility model, no tyre model. `RoadCondition.wetness`
  (1.2) and the free-form `Properties` are not modeled at all.
* **Catalog references.** `EnvironmentAction` accepts an inline `Environment`;
  the `CatalogReference` form arrives with catalogs (P4).
