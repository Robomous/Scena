# ADR-0015: Global and infrastructure actions

- **Status:** accepted
- **Date:** 2026-07-22

## Context

Sprint p5-s6 (#34, "Global & infrastructure actions") is the last sprint of the
P5 pillar and completes the declared action/condition catalog. It covers the
ASAM OpenSCENARIO XML 1.4.0 §7.4.2 `GlobalAction` family — `VariableAction`,
the deprecated `ParameterAction`, `EntityAction`, `EnvironmentAction` and
`InfrastructureAction` — the §7.4.3 `CustomCommandAction`, and the two
traffic-signal conditions of §7.6.5.2.

Two things make this sprint different from its predecessors.

First, **none of these actions has an actor.** Every one of the ten private
actions landed in p5-s4 and p5-s5 names the entity it drives, and
`ir::Action::entity_id()` is pure — the engine's validation, dispatch and
diagnostics are all keyed on it. A global action has nothing to name.

Second, **almost none of them consumes simulation time.** Annex A Tables 11 and
12 mark every action in this sprint as ending "immediately (does not consume
simulation time)". What they do instead is write engine-level state that
outlives the evaluation: a variable, an entity's presence, the environment, a
signal's observable state. The interesting design questions are therefore about
*storage and precedence*, not about control laws — the opposite of p5-s5.

Much of the substrate already existed. `ParameterCondition`,
`VariableCondition` and `TimeOfDayCondition` landed in p5-s1 together with the
runtime variable store and the time-of-day anchor; the defaulted-virtual
gateway pattern landed in p5-s5.

## Decision

p5-s6 lands the whole family as a kernel + C ABI + Python sprint (no XML
frontend — the standing precedent since p5-s1), with the following design
decisions.

### D1 — An actor-less base, not an empty string

`ir::GlobalAction` is an intermediate abstract base that finalizes
`entity_id()` to a shared static empty string. Existing call sites keep a valid
reference and nothing in the engine had to become optional.

Dispatch and validation branch on a `dynamic_cast<const ir::GlobalAction*>`,
**never** on the emptiness of the id. An entity id is scenario content; giving
"" a second meaning would make a malformed scenario indistinguishable from a
global action. The unknown-kind diagnostic keys its path on the action kind
(`actions/TrafficSourceAction`) when there is no entity to name, and keeps the
entity-keyed form otherwise.

When §7.5.4 per-actor expansion arrives with the frontend, a `GlobalAction`
lowers **once**, never per actor.

### D2 — The deprecated parameter actions get an overlay, not a rewrite

`ParameterSetAction` and `ParameterModifyAction` were deprecated with 1.2
because §9.1 made parameters immutable at runtime. Three options existed:
reject them, silently map them onto the variable store, or execute them.

We execute them against a **runtime overlay** — a separate map consulted ahead
of `scenario_.parameters` when a `ParameterCondition` resolves a name. This is
the only option that keeps both promises: a 1.0/1.1 file's parameter action is
visible to the condition that reads it (which is the whole point of the
action), and §9.1 immutability stays literally true for 1.2+ content, which
cannot contain these actions at all. Mapping onto the variable store instead
would have conflated two namespaces the standard keeps separate — a variable
and a parameter may share a name.

Each name warns once with `Status::DeprecatedFeature`. The overlay is per-run
state, cleared by `init()` and `close()`.

### D3 — The entity lifecycle is a flag, not an erase/insert

`EntityAction` "adds or deletes" entities, and the obvious implementation is to
erase from and insert into the engine's entity map. We do not.

The entity table is a `std::map` whose *iteration order* is load-bearing for
the determinism contract, and every record carries accumulated bookkeeping
(the observation baseline, the odometer, the standstill timer, domain
ownership). An erase/insert would churn both for no gain: the entity is
declared in the Entities section either way, so its record can simply carry an
`active` flag. The map's structure never changes at runtime.

A delete clears all runtime motion, assignment and derived-observation state
and keeps the declared immutables (control mode, bounding box, performance), so
a later add restarts from a clean slate. An add installs a fresh state at the
action's world position and seeds the observation baseline from it exactly as
`init()` does — otherwise the first step after a respawn would report a phantom
acceleration and a teleport-sized odometer jump.

An inactive entity is skipped by the poll, the observation refresh, the
integrator and the publish; reports nothing through any host query; rejects
`report_state` with `UnknownEntity`; and is absent from the entity-kinematics
facet, so every by-entity condition over it is a deterministic false at the
ADR-0007 grain. `Engine::entity_active` distinguishes "never declared"
(`nullopt`) from "deleted" (`false`).

**Deviation from §7.5.2.2 (documented).** The standard has a private action
whose actor disappears take a `stopTransition` *at the action level*. Scena has
no per-action observable transitions — `StoryboardElementType::Action` is
itself deferred — so `record_for` returns null for an inactive entity, every
private-action branch maps that to `Complete`, and the stop surfaces as the
**owning event** ending on the next evaluation. The observable end state is the
same; only the granularity differs. The same treatment applies to *reference*
entities in distance keeping and relative speed targets, which §7.5.2.1 names
explicitly ("if the referenced entity of an instance of a
LongitudinalDistanceAction disappears").

### D4 — An arithmetic phase clock, and actions-win precedence

A traffic signal controller's phase could be advanced incrementally — subtract
`dt` from the remaining phase time each step and roll over. We derive it
arithmetically instead:

```
local = fmod(t - anchor, total)   →  linear scan of the cumulative offsets
```

`fmod` is exact in IEEE and the cumulative offsets are summed once in declared
order, so the phase at time `t` is a pure function of `t`. An incremental
clock would accumulate rounding and make the phase depend on the host's step
*pattern* — two runs at different rates reaching t = 20 s could disagree, which
the determinism contract forbids in spirit even where it is not directly
tested. It is tested here: a 5 × 4 s run and a 160 × 0.125 s run must be in the
same phase.

Phase intervals are **half-open** — at exactly the cumulative duration of phase
*i* the cycle is already in phase *i+1*. This is deliberately unlike the
trajectory segment lookup, where a vertex belongs to both adjacent segments: a
phase owns its start and not its end (§6.11.4), and the half-open rule also
gives a zero-duration phase the only sensible reading, that it occupies no
time.

**Precedence: write-on-transition, and actions win.** Controllers tick at the
top of the storyboard-evaluation phase and write a phase's signal states only
when the cycle *enters* that phase. A `TrafficSignalStateAction` therefore
lands after the tick and overrides it, and — because the controller does not
rewrite the signal every step — the forced state stands until the next phase
transition reclaims the signal. That is exactly the §11.12 worked example: a
traffic light forced into a broken state stays broken.

`TrafficSignalControllerAction` re-anchors the cycle at the named phase and
ticks that one controller immediately, so conditions evaluated later in the
same storyboard walk agree with it.

Init follows the same precedence: controllers seed their first phases *before*
the init actions run, so an init-phase state action wins over the seed.

**Delayed start ⇒ no phase.** Before its §6.11.3 start offset elapses a
controller drives nothing and has no observable phase, so a condition naming
its first phase is a deterministic false rather than a guess at what it "would"
be showing.

**Zero-length cycle ⇒ pin the first phase.** A cycle whose durations sum to
zero has no length to advance through. Rather than `fmod(x, 0)` (a NaN), the
first phase is held for the run and the load warns.

### D5 — Environment is stored, and only the clock is live

`EnvironmentAction` merges member-wise, following §Environment / §Weather
literally: "if one of the conditions is missing it means that it doesn't
change". Both levels merge independently, so a weather update that mentions
only precipitation leaves an earlier fog in place.

Everything in `Environment` is **stored and never applied**: Scena has no
renderer, no sensor model and no tyre model, so weather, sun, wind and friction
are state the host reads back and acts on itself. Storing them without acting
on them is honest; pretending to model them would not be.

The one exception is `TimeOfDay`, which re-anchors the same simulated clock
`Engine::set_date_time` drives and additionally decides whether that clock
*advances*: `animation="false"` freezes the simulated instant at the anchor.
`current_date_time_seconds` picks between the two with one fixed IEEE
expression, and `TimeOfDayCondition` needed no change at all — it reads
whatever instant the context reports. The host setter keeps meaning an
advancing clock, so it un-freezes a pinned anchor.

### D6 — The signal conditions are level predicates

Both signal conditions are worded as "reaches" in the standard. Both are
implemented as **level** predicates — true for as long as the state or phase
matches — with `conditionEdge rising` supplying "reaches", exactly as for every
other level condition in the p5-s1 catalog.

The storyboard already owns edge semantics, and it owns them for every
condition uniformly. Building a second notion of "reaches" inside two
conditions would give two places to disagree about what a discrete evaluation
means, for no expressive gain.

Signal states compare byte for byte and carry no `Rule`: "interpretation and
notation of `state` are specific to the simulation engine used" (§6.11.4).

### D7 — `on_custom_command`, and a no-op host as the contract

`CustomCommandAction`'s `type` and `content` are "defined as a contract between
the simulation environment provider and the author of a scenario", so the
engine hands them to the gateway verbatim and interprets neither.

`ISimulatorGateway::on_custom_command` is a **defaulted** virtual, like
`on_controller_assigned` and `on_visibility_changed` — an amendment to
ADR-0003, on the same terms as ADR-0014's: hosts written against the older
interface keep compiling.

Without a gateway the action is a silent no-op and emits **no diagnostic**.
§7.4.3 makes executability depend on "the ability of the specific simulation
environment recognizing these actions", so a host that does not implement a
command is the documented contract, not a defect. Warning about it would make
every gateway-less run noisy about something the standard permits.

### Validation and rule citation

Load-time validation cites the Annex C rule id wherever the standard names
one, and the section number where it does not:

| Check | Rule |
|---|---|
| undeclared `variableRef` | `reference_control.resolvable_variable_reference` |
| non-numeric modify target | `data_type.variable_modification_or_comparison_possible` (C.2.6) |
| invalid `TimeOfDay` date-time | `data_type.time_format` (C.2.4) |
| negative phase duration | `data_type.phase_duration_positive` (C.2.3) |
| controller `reference` unresolvable / cyclic | `reference_control.traffic_signal_controller_references` (C.7.13) |
| controller-action refs | `..._controller_action_references` (C.7.11) |
| controller-condition refs | `..._controller_condition_references` (C.7.12) |
| undeclared `parameterRef`, undeclared entity ref | none defined — cites §9.1 / §7.4.2 |

Rules C.7.10, C.7.14 and C.7.15 check signal ids against the **road network
file**, which Scena cannot reach until p3-s4. Signal ids stay free-form; an id
nothing writes warns once at evaluation and reads as false.

## Consequences

**Positive.**

- The declared v0.0.1 action and condition catalog is complete at the kernel
  level; P5 closes.
- The determinism contract extends cleanly to the new observables: the GS-11
  anchor compares signal state and phase *strings* alongside bit-identical
  doubles, and the entity-lifecycle anchor proves that churning one entity's
  presence leaves the others untouched.
- The signal clock is step-pattern independent, which is a stronger property
  than "two identical runs agree" and matters for a host that changes rate.
- `format_scalar` gives the string-valued variable store an exact write side,
  so a chain of modify actions does not drift.

**Negative / accepted.**

- The §7.5.2.2 stop is one evaluation coarser than the standard's. Per-action
  observable transitions would close the gap; they are deferred with
  `StoryboardElementType::Action` itself.
- The environment is inert. A host that wants weather to affect anything must
  implement that itself.
- `Weather.fractional_cloud_cover` is carried as an okta count rather than the
  §FractionalCloudCover enumeration; the two map one-for-one, and the frontend
  will lower the literals.
- The C ABI ships builders and small getters only; full environment read-back
  waits for the p6-s1 expansion.

## Alternatives considered

- **Reject the deprecated parameter actions.** Cleanest, and wrong: valid 1.0
  and 1.1 files contain them, and the coverage matrix commits to loading
  deprecated-but-valid content rather than refusing it.
- **Erase/insert for the entity lifecycle.** Matches the standard's wording
  more literally, at the cost of churning the exact structure the determinism
  contract depends on, for no behavioral gain.
- **Incremental phase advance.** Simpler to read, and it makes the signal
  phase a function of the host's step pattern rather than of simulation time.
- **Edge-triggered signal conditions.** Would match the "reaches" wording
  without a `conditionEdge`, at the cost of a second, inconsistent notion of
  edge semantics inside the condition layer.

## References

- ASAM OpenSCENARIO XML 1.4.0 §6.11 (traffic signals), §6.12 (variables),
  §7.4.2 (global actions), §7.4.3 (user-defined actions), §7.5.2 (completing an
  action), §8.5 (init phase), §9.1 (parameters), §11.12 (signalized
  T-intersection), Annex A Tables 11–12, Annex C rules C.2.3, C.2.4, C.2.6,
  C.7.9–C.7.15.
- [ADR-0003](ADR-0003-simulator-gateway.md) — amended again, for
  `on_custom_command`.
- [ADR-0007](ADR-0007-condition-evaluation-context.md) — the absent-facet grain
  the two new facets follow.
- [ADR-0014](ADR-0014-private-actions-routing-distance-controllers.md) — the
  defaulted-virtual gateway precedent.
- [Global and infrastructure actions](../user-guide/global-actions.md) — the
  user-facing description.
