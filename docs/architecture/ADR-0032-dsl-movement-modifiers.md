<!--
SPDX-FileCopyrightText: 2026 Robomous
SPDX-License-Identifier: Apache-2.0
-->

# ADR-0032 — Lowering the §8.9 movement modifiers

- **Status:** Accepted
- **Date:** 2026-08-02
- **Sprint:** p8-s3 (#46)
- **Supersedes:** nothing. Builds on ADR-0030 (lowering to the IR) and ADR-0031
  (the composition operators).

## Context

The DSL's generic actions — `drive()`, `move()`, `walk()` — carry no target of
their own. ADR-0030 lowered them to nothing and said why: they exist to be
*shaped* by §8.9's modifiers, and until those lower there is nothing to shape.
The standard's own style guide is explicit that this is the idiomatic form
("prefer using generic actions", §9.1.8), so nearly every scenario in the
specification is `drive()` plus modifiers.

A modifier is an equality constraint on the invoked behavior's parameters
(§7.3.12.4), and §8.9 gives seventeen of them plus four parameters common to
all. The question is what each one *denotes* in a runtime that was built for
OpenSCENARIO XML.

## Decision

### 1. Every modifier lands on an action the runtime already has

No modifier machinery is added to the runtime. Each one lowers to an IR action
P2 or P5 already implements — a `SpeedAction`, a `TeleportAction`, a
`LaneChangeAction`, a `LaneOffsetAction`, a `LongitudinalDistanceAction`,
a `LateralDistanceAction` — or it is reported. That is the same rule ADR-0031
applied to composition, and it is what keeps one runtime under two frontends
rather than two runtimes that resemble each other.

| §8.9 modifier | IR action |
|---|---|
| `speed` (absolute) | `SpeedAction` |
| `speed` (`faster_than`/`slower_than`/`same_as`, `factor`) | `SpeedAction` with a `RelativeTargetSpeed` |
| `change_speed` | `SpeedAction` relative to the actor's own speed |
| `position` at the start | `TeleportAction` to a `RelativeObjectPosition` |
| `position` over the phase | `LongitudinalDistanceAction` |
| `lateral` (with `side_of`) | `LateralDistanceAction` |
| `lateral` (without) | `LaneOffsetAction` |
| `lane` (number) | `LaneChangeAction` to an absolute lane |
| `lane` (`side_of` + `side`) | `LaneChangeAction` to a relative lane |
| `change_lane` | `LaneChangeAction` relative to the actor |
| `keep_lane` | continuous `LaneOffsetAction` at zero |

### 2. The `at` anchor decides *how*, and the phase duration is what it needs

§8.9.1.1.1's anchor takes `start`, `end` or `all`, and it is optional.

- **absent or `all`** — the value holds for the invocation. Realised by setting
  it when the phase begins, which is a Step transition.
- **`start`** — the same thing, said explicitly.
- **`end`** — the value must be *reached* by the end of the phase, so it becomes
  a transition spread over the phase's length.

That last case is why p8-s2's durations are load-bearing here: without a
concrete duration there is no interval to spread the change over. Rather than
invent one, lowering reports and sets the value at once — a fabricated duration
would put a number in the trace the scenario never stated.

`position` is the modifier where the anchor changes the *kind* of action, not
just its shape: at the start it is a placement (`TeleportAction`), over the
phase it is a gap to reach and hold (`LongitudinalDistanceAction`). Both
readings are in §8.9.2; the anchor is what chooses between them.

### 3. "Keep doing what you are doing" lowers to nothing

`keep_speed` (§8.9.6) and `keep_position` (§8.9.3) constrain the actor not to
change. The runtime already holds an entity's speed and its relative position
between actions, so the faithful lowering is *no action*: one that set the
current value would be a no-op that nonetheless appears in the trace and
competes for the same action domain (§7.5).

`keep_lane` (§8.9.16) is different and does produce an action, because holding a
lane is active work — a continuous `LaneOffsetAction` at zero is exactly the
runtime's way of saying it.

### 4. What is reported, and why

- **`acceleration` (§8.9.7)** shapes the acceleration of a movement the phase is
  already performing. There is no acceleration-target action in the IR — the
  coverage matrix defers §8.8's for the same reason — and on its own the
  modifier states a rate with nothing to apply it to.
- **`along` and `along_trajectory` (§8.9.11–.12)** need a concrete route or
  trajectory *value*. The DSL has no struct constructor (§7.2.2.6.7), so one can
  only come from §8.12.2's `map.create_route(...)`, and the standard itself says
  map methods may be external implementations (§7.3.7.4) — post-v0.0.1.
- **`distance` (§8.9.13)** bounds a phase by distance travelled rather than by
  time, and ADR-0031 sequences phases by time.
- **`yaw`, `orientation`, `physical_movement`, `avoid_collisions`** were already
  Post in the coverage matrix and stay there.

Each is reported by name with its section, never silently dropped.

## Consequences

- The idiomatic DSL scenario — `drive()` with modifiers — now produces a
  runnable IR, which is what GS-12 and GS-13 will be written in.
- Modifier lowering reads the phase's duration, so it sits after ADR-0031's
  time arithmetic and depends on it. That ordering is now a fact of the code.
- **A pre-existing checker gap became visible and was filed as #110.** §7.3.3
  says an overloaded enum literal "will depend on the type requirements of the
  place it is used in"; Scena reports the ambiguity before consulting them. `at`
  (§8.9.19) and `route_overlap_kind` (§8.12) both declare `start` and `end`, so
  the specification's own `at: start` is rejected while `at: at!start` is
  accepted. It blocks nothing — the qualified spelling works — but a conforming
  scenario copied out of the standard does not check, so it is tracked rather
  than absorbed.

## Alternatives considered

**Give the runtime a modifier concept.** Rejected: modifiers are a DSL surface,
and a runtime that knew about them would be machinery only one frontend could
reach.

**Default an `at: end` modifier to some duration.** Rejected: it puts a number
in the trace the scenario never stated, which is the same failure mode ADR-0030
rejects for performance limits and ADR-0031 for `one_of` selection.

**Lower `keep_speed` to a speed action holding the current value.** Rejected: it
is a no-op that would still occupy the longitudinal action domain (§7.5) and
could supersede a running action — a behaviour change dressed up as a
constraint.
