<!--
SPDX-FileCopyrightText: 2026 Robomous
SPDX-License-Identifier: Apache-2.0
-->

# ADR-0030 — Lowering a checked DSL program to the Scenario IR

- **Status:** Accepted
- **Date:** 2026-08-02
- **Sprint:** p8-s1 (#44)
- **Supersedes:** nothing. Builds on ADR-0028 (symbols and the type model),
  ADR-0029 (the bundled standard library) and ADR-0010 (the entity taxonomy).

## Context

P7 ended with a frontend that *checks* OpenSCENARIO DSL: it parses, resolves,
types and validates a program against the whole §8 standard library. Nothing
runs. P8 connects that to the runtime the XML frontend has been feeding since
P4, and the connection point is the Scenario IR — the architecture's rule that
both frontends compile into one IR and that runtime semantics live in the
runtime, never in a frontend.

Lowering therefore decides one thing only: **which DSL construct denotes which
IR construct**. Where it cannot decide, it reports.

Three questions had no obvious answer:

1. **Which scenario runs?** §7.7.2 says outright that entry-point selection "is
   defined by implementation", and that this is deliberate: "This allows
   implementations to experiment with the best ways to select this entry point."
2. **What is a participant?** The DSL has no `Entities` section. A scenario
   declares fields, some of which happen to be actors.
3. **Where do concrete values come from?** The IR wants numbers. §7.3.11
   constraints can say far more than "this field is 4.5 m", and solving them in
   general needs a constraint solver, which ADR-0004 places after v0.0.1.

## Decision

### 1. The entry point is named, and a single scenario names itself

`LowerOptions::entry_point` takes a scenario by qualified name (`demo::overtake`)
or by the name as written (`overtake`) — a file with one namespace makes the
prefix pure ceremony.

When it is empty:

- a root file declaring exactly **one** scenario uses that one;
- a root file declaring **several** is an error that lists them.

Guessing among several would make the run depend on declaration order, which is
the kind of hidden input the determinism contract exists to eliminate.
`entry_points()` returns the same list a CLI would print, in declaration order:
that is what the file *offers*, and a reader matches it against the file in
front of them rather than against an alphabetized list.

Only the **root** file's scenarios are offered. An imported file contributes
types, not entry points.

### 2. A participant is a field whose type derives from `std::physical_object`

§8.7 roots its actor hierarchy at `physical_object`, so that is the test.
Every such field of the entry scenario becomes one `ir::Entity`, in declaration
order, with the field name as both id and name.

Classification follows the same hierarchy onto ADR-0010's taxonomy:

| DSL actor | IR object |
|---|---|
| derives from `std::vehicle` | `ir::Vehicle` |
| derives from `std::person` | `ir::Pedestrian` |
| derives from `std::stationary_object` | `ir::MiscObject` |
| anything else deriving from `physical_object` | unclassified |

The last row matters. §8.7.10's `animal` is a sibling actor, not a pedestrian
category, and XML has nowhere to put it; an entity with an identity and a
control mode is all the runtime needs of it, and a wrong classification would be
worse than none.

Every lowered participant is `EngineControlled`: the DSL has no way to say
otherwise, and the host reassigns ownership through the engine API (ADR-0003).

### 3. Concrete values come from equality constraints and nowhere else

Lowering reads exactly one constraint shape: `keep(<field-path> == <constant>)`,
in either operand order, where the constant side folds without a solver. That is
what "attribute-level concrete" means (§6.3.1.2.1) and it is the only shape
whose meaning is unambiguous without search.

Two consequences worth stating:

- **Lowering never converts.** §7.3.4 folding already happened during checking,
  so a physical value arrives in its base unit. In particular lowering must not
  re-apply the standard's printed conversion factors — ADR-0029 carries them
  verbatim once, at fold time, and once is the whole point.
- **What is not fixed keeps the IR's own default.** A vehicle whose category no
  constraint fixes is whatever `ir::Vehicle` defaults to, not a guess made here.

§7.3.8.2's conditional inheritance (`inherits vehicle(vehicle_category == car)`)
fixes a value on the *type* rather than in the scenario, and is read the same
way — it is the spelling §8.7's own examples use.

### 4. §8.7 has no performance limits, and lowering does not invent any

The DSL domain model has no counterpart to XML's `Performance` element: §8.7
declares no maximum speed, acceleration or deceleration anywhere. The IR's zeros
are the faithful lowering rather than a gap, because the runtime already reads a
non-positive limit as "unconstrained" (`actor_max_speed` in `engine.cpp`). A DSL
vehicle is therefore unlimited until the scenario says otherwise, and no numbers
are fabricated.

### 5. A remnant is reported, never approximated

Anything that would need search is a diagnostic, not a silently-defaulted value
— the same stance ADR-0004 takes for the checker and the same one the XML
frontend takes for constructs it does not implement. A scenario that declares no
participant is a warning: the file is well-formed, it simply has nothing to run.

## Consequences

- `.osc` produces an `ir::Scenario` the existing engine accepts. Actions,
  `set_map_file` and `scena-run`'s `.osc` support are the rest of p8-s1.
- Lowering is inside the determinism contract, because load time is. It reads
  ordered containers only, walks fields in declaration order, and does no
  floating-point arithmetic of its own — the values it copies were folded once,
  during checking, by the same detmath-constrained path.
- A future frontend (or a future entry-point mechanism the standard may
  introduce) changes §7.7.2's rule here without touching the runtime, which is
  the point of putting the decision in the frontend.

## Alternatives considered

**Pick the first scenario when several are declared.** Rejected: it makes the
run depend on declaration order, and reordering a file would silently change
what executes.

**Give DSL vehicles a documented default performance profile.** Rejected: it
fabricates numbers the standard does not state. "Unconstrained" is what §8.7
actually says, and the runtime already spells it as zero.

**Solve constraints during lowering.** Rejected by ADR-0004: constraint solving
is post-v0.0.1, and half-solving would produce scenarios whose behaviour depends
on the solver's search order — the opposite of the determinism promise.
