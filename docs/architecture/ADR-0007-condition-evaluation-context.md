# ADR-0007: Condition evaluation context

- **Status:** accepted
- **Date:** 2026-07-21

## Context

Through P1, a trigger condition was a pure function of one number — the
simulation time (`Condition::evaluate(double)`). The by-value condition set
(p5-s1) breaks that: a `ParameterCondition` reads a named parameter, a
`VariableCondition` a mutable variable, a `UserDefinedValueCondition` an
external value the host injects, a `TimeOfDayCondition` a simulated wall
clock, and a `StoryboardElementStateCondition` the live state of another
storyboard element. These are five different runtime facets, some of them
mutable during a run, one of them (element state) internal to the scheduler.

Conditions must stay **pure and deterministic** — the whole edge/delay
history machinery and the bit-identity contract depend on `evaluate` having
no side effects and being a function of its inputs alone. So the runtime
state a condition may read has to arrive through a single, read-only seam
rather than by giving conditions pointers into the mutable engine.

## Decision

Introduce `ir::EvaluationContext`, an abstract read-only interface that a
condition is evaluated against. `Condition::evaluate` becomes
`evaluate(const ir::EvaluationContext&)`.

- **Simulation time** is always available (`simulation_time()`); the other
  facets are optional virtuals that default to "absent". A condition that
  reads a facet the context does not provide evaluates to a deterministic
  `false`, never an error.
- **Named values** (`named_value(kind, name)`) span three namespaces —
  `Parameter`, `Variable`, `UserDefinedValue` — returned as strings, matching
  the conditions' XSD (they compare stringly). Parameters are immutable at
  runtime (§9.1); variables are seeded from their declarations at init and
  mutable (§6.12); user-defined values are external and may be staged before
  init. A `Status::UnknownName` distinguishes host misuse (setting an
  undeclared variable) from scenario defects.
- **Time of day** (`date_time_seconds()`) is a simulated instant in epoch
  seconds, anchored by the host and advancing one-for-one with simulation
  time: `anchor_epoch + (t − anchor_sim)`. No wall clock is ever read.
- **Storyboard element state** is answered by the scheduler, not the engine.
  `Scheduler::step` wraps the host's context in a `BoundContext` that forwards
  every other facet unchanged but answers `storyboard_element_state` from its
  own bound tree. References are pre-resolved at `bind()` to a unique element
  (or none); resolution and the engine's validation share one nameRef matcher
  (`runtime/element_ref`) so "init accepted the reference" and "the scheduler
  can resolve it" never disagree.

### One-evaluation transition window

Level states (standby/running/complete) hold whenever the element is in that
state. Transitions (start/end/stop/skip) are **one-evaluation pulses**: the
scheduler stamps every transition with an evaluation counter (the init
evaluation is 1), and a transition literal holds only when the stamp equals
the current evaluation. A single-pass, document-order walk gives the
consequence that an earlier-evaluated condition does not observe a transition
a later element performs in the same evaluation; because stop is processed
before start at every level, a stopTransition *is* visible to a later element's
start trigger.

## Consequences

- The engine builds a per-step context view over its live stores and hands it
  to the scheduler; the scheduler alone knows how to answer element-state
  queries. The two never share mutable state — the context is a read-only
  view with a diagnostics side-channel for the deduplicated warn-once messages
  (unset user value, unset time of day).
- Adding a future facet (entity kinematics for the by-entity conditions,
  p5-s2/s3) is a new virtual on `EvaluationContext` with an "absent" default —
  existing conditions and contexts are unaffected.
- `Status::UnknownName` is an ABI addition, appended to the status enum and
  mirrored across the C ABI and the Python bindings.
