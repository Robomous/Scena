<!--
SPDX-FileCopyrightText: 2026 Robomous
SPDX-License-Identifier: Apache-2.0
-->

# ADR-0031 — Lowering the DSL composition operators

- **Status:** Accepted
- **Date:** 2026-08-02
- **Sprint:** p8-s2 (#45)
- **Supersedes:** nothing. Builds on ADR-0030 (lowering to the IR) and on the
  storyboard model P1 pinned (ADR-0025, ADR-0026).

## Context

ADR-0030 gave the DSL frontend a storyboard: one Story, one Act, one
ManeuverGroup per phase. It handled the two easy cases — `serial` chained on the
predecessor's completion, `parallel` left the triggers absent — and reported
everything else.

§7.6.2.1 asks for more than that. A composition can nest; it can carry a
`duration`; `one_of` picks one of several alternatives; `wait elapsed(d)`
introduces a phase in which nothing is specified. None of these is a runtime
concept: the storyboard machinery the XML frontend has been driving since P1
already does everything they need. What was missing is the *arithmetic* that
turns a composition into start conditions.

Three questions had no obvious answer:

1. **How does a `duration` become anything the IR can hold?** A storyboard
   element has no duration field, and — probed against the engine — neither of
   the obvious tricks works. A stop trigger only cuts a phase short: an Act
   completes as soon as its groups do, so a step-shaped action ends the phase
   immediately whatever the stop trigger says. And a start trigger delayed on
   the predecessor's `runningState` never fires, because the predecessor is no
   longer running by the time the delayed lookup happens.
2. **Which alternative does `one_of` run?** §7.6.2.1.3 says at least one must
   hold and says nothing about which.
3. **What ends a parallel composition?** Its members end at different times,
   and the phase after it has to wait for all of them.

## Decision

### 1. Composition decides *when* a phase starts, and nothing else

A composition operator contributes exactly one thing to the IR: the start
trigger on a phase's Event. No new runtime concept, no new IR node, no change to
the scheduler. That is what makes the two frontends share one runtime rather
than two that resemble each other.

### 2. A concrete duration is arithmetic, done at load time

The storyboard starts at t = 0 and every duration that lowers is a constant, so
a phase's absolute start time is the sum of the durations before it. That sum is
computable while lowering, exact, and needs no feedback from the run — which is
also why it is inside the determinism contract rather than at odds with it.

Lowering therefore tracks, for each point in the `do` directive, what load time
knows about it:

- an **absolute time**, when every preceding phase had a concrete duration; and
- a set of **groups that must have completed**, for the phases whose end only
  the runtime knows.

A trigger ANDs whichever of the two is known — a `SimulationTimeCondition` plus
one `StoryboardElementStateCondition` per group, in a single ConditionGroup.
A phase whose start is t = 0 with nothing to wait for gets *no* trigger, because
`t >= 0` is a tautology and §7.6.1.1 already says a trigger-less element starts
with its parent.

**A range duration is not a value.** §7.6.2.4 lets `duration` be `[10s..30s]`,
which constrains accepted traces rather than fixing a time; choosing a value
from it needs a solver, and that is post-v0.0.1 (ADR-0004). It is reported and
the phase falls back to completion chaining.

### 3. A parallel join is an AND, which the trigger model already has

A `ConditionGroup` is a conjunction, so "every member has finished" is one group
with one condition per member. Members that end at a known time collapse into a
single `SimulationTimeCondition` at the latest of them; members that end when
their actions do each contribute their group's completion. Mixing the two is
therefore free, and the common case — every member has a duration — reduces to
one arithmetic comparison.

§7.6.2.1.4's default overlap is `start`: members begin together and may end
apart, which is exactly what absent start triggers already give. The other seven
overlap kinds and the `start_to_start`/`end_to_end` offsets are reported and not
realised in v0.0.1.

### 4. The `one_of` alternative is an input, defaulting to the first

§7.6.2.1.3's latitude is real, and an executor has to pick. Picking at random
would put a hidden input into the run, which is the one thing the determinism
contract exists to prevent — and the engine has no seed machinery to make such a
choice reproducible even in principle.

So the choice is an explicit input: `LowerOptions::alternative`, which
`scena-run --select` feeds, naming an alternative by its label. Empty means the
first alternative in declaration order. An alternative that is not there is an
error listing the ones that are.

This is a *lowering-time* decision, not a runtime one: the alternatives that were
not chosen do not reach the IR at all. That keeps the engine free of a concept
the XML side has no counterpart for.

### 5. `wait elapsed(d)` lowers to nothing but the clock

§7.6.2.4.2 introduces a phase of the given length in which nothing is specified.
Nothing is what it produces: no group, no event, no action — it only advances
the running offset the next phase starts from. An IR element that did nothing
for `d` seconds would be a fiction, and the clock is already running.

`wait` for an *event* is a different construct, and §7.6.2.5's events have no
runtime carrier in v0.0.1; it is reported, along with `emit`, `on` and `until`.

## Consequences

- A DSL scenario's phases are sequenced by construction. `scena-run` runs one
  and the trace shows each phase taking over at the second the scenario named.
- Composition is a pure frontend concern. Adding an overlap kind, or a future
  operator, changes lowering and nothing below it.
- The scenario's own end becomes the storyboard's stop trigger when it is a
  constant, so a run's element states finish rather than sit at running.
- **A pre-existing lexer defect surfaced and was fixed here.** §7.2.2.6.7 spells
  the range constructor `'[' expression '..' expression ']'`, and
  §7.2.1.5.2's `float-literal ::= digit* '.' digit+` makes the leading digits
  optional — so `[2..4]` is a race between the `..` operator and the float `.4`,
  and the float was winning. The lexer emitted no `..` token at all and the
  parser looked for `...`, a spelling the standard does not have. The two agreed
  with each other, so every range in the test suite was written the wrong way and
  nothing noticed. `duration: [10s..30s]` is what made it visible.

## Alternatives considered

**Give a storyboard element a duration field.** Rejected: it is a DSL concept
with no OpenSCENARIO XML counterpart, so it would be runtime machinery only one
frontend can reach — the opposite of the two-frontends-one-runtime rule.

**Bound a phase with a stop trigger on the Act.** Probed and rejected: a stop
trigger can only cut a phase short. An Act completes as soon as its groups do,
so a step-shaped action ends the phase immediately, and the duration would be
silently ignored for exactly the scenarios that are easiest to write.

**Delay the start trigger on the predecessor's `runningState`.** Probed and
rejected: the predecessor is no longer running when the delayed lookup happens,
so the trigger never fires at all.

**Pick a `one_of` alternative at random from a seeded generator.** Rejected: it
makes the run depend on a seed the scenario does not state, and the engine has
no seed machinery — introducing one for a frontend concept would put randomness
inside the determinism contract for the first time.
