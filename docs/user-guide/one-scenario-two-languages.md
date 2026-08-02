# One scenario, two languages

Scena executes ASAM OpenSCENARIO XML and ASAM OpenSCENARIO DSL. They are not
two engines that behave similarly: both frontends compile into the same
Scenario IR, and one runtime executes it. This page is where that claim is
spelled out — and where the places the two languages genuinely differ are
written down rather than glossed over.

## The claim, as a test

`tests/golden/scenarios/gs12-dsl-cruise.osc` and `gs12-xml-cruise.xosc` say the
same thing in the two languages:

```
# OpenSCENARIO DSL
do serial:
    cruise: ego.drive(duration: 5s) with:
        speed(speed: 10mps, at: at!start)
    accelerate: ego.drive(duration: 4s) with:
        speed(speed: 20mps, at: at!end)
```

```xml
<!-- OpenSCENARIO XML -->
<Event name="cruise">…step to 10 m/s…</Event>
<Event name="accelerate">…linear over 4 s to 20 m/s, from t >= 5…</Event>
```

Run both and the traces are equal **byte for byte** — not close, equal:

```sh
python scripts/golden.py compare-pair gs12
ok   gs12 pair  (gs12-dsl-cruise.osc == gs12-xml-cruise.xosc, byte for byte)
```

The pair is part of `check-all`, so CI asserts it on macOS, Linux and Windows
on every push. If a change made one frontend drift from the other, this is what
would say so.

## How the two get there

| | OpenSCENARIO XML | OpenSCENARIO DSL |
|---|---|---|
| Who runs | the `Storyboard`'s stories and acts | the entry scenario's `do` directive (§7.7.2) |
| Who is in it | `Entities`/`ScenarioObject` | fields whose type derives from `std::physical_object` |
| Concrete values | attributes | `keep(field == constant)` (§7.3.11) |
| Sequencing | start triggers you write | `serial`/`parallel`/`one_of` lowered to start triggers |
| What acts | actions in events | actions, plus §8.9 modifiers on generic actions |
| Road network | `RoadNetwork/LogicFile` | §8.5.4's `map_file` |

ADR-0030, ADR-0031 and ADR-0032 record the mapping decisions. The short version:
a DSL construct only ever decides *which IR construct it denotes*. Nothing in
the runtime knows the DSL exists.

## Where the two languages genuinely differ

These are not implementation gaps. They are places where the standards
themselves do not line up, and Scena had to choose. Each choice is recorded
here, in the coverage matrix, and in the ADR that made it.

**The DSL declares no performance limits.** §8.7's actor hierarchy has no
counterpart to XML's `Performance` element — no maximum speed, acceleration or
deceleration anywhere. A DSL vehicle is therefore unconstrained, which the IR
already spells as zero (the runtime reads a non-positive limit as "no limit").
No numbers are invented. It does mean an XML scenario and its DSL twin will
diverge if the XML one relies on a `Performance` clamp — which is why
`gs12-xml-cruise.xosc` sets its limits to zero.

**`animal` has no taxonomy counterpart.** §8.7.10 declares `animal` as a sibling
of `vehicle` and `person`. XML has nowhere to put it, so a DSL `animal` lowers
as an unclassified participant: it has an identity and a control mode, which is
all the runtime needs of it. A wrong classification would be worse than none.

**Entry-point selection is implementation-defined.** §7.7.2 says so outright. A
file with one scenario runs it; a file with several is an error listing them,
because guessing would make the run depend on declaration order. `--entry`
names one.

**`one_of` has no XML counterpart at all.** §7.6.2.1.3 says at least one
alternative must hold and says nothing about which. Scena makes the choice an
*input* (`--select`, defaulting to the first alternative) rather than a random
draw, because a seed the scenario does not state is exactly the hidden input the
determinism contract exists to rule out. Alternatives that are not chosen never
reach the IR.

**Three §8.8/§8.9 names collide in the standard.** `change_speed`, `keep_speed`
and `change_lane` are declared both as actions on an actor (§8.8) and as
modifiers for the same actor (§8.9). A qualified behavior name identifies
exactly one declaration, so the language cannot hold both; the three modifiers
take §7.3.12.3's *unassociated* form, the only spelling that exists and does not
collide.

**Two more §8.15 contradictions**, resolved by asking which surface the rest of
the chapter corroborates: §8.15.4.2.1 prints `extend traffic_light:` for a
*group*'s `state_equal` while its heading, prose and Table 319 all say group —
the printed code loses. Table 337 names `set_group_bulb_state`'s first parameter
`traffic_light` while its description says "group" — the parameter table wins,
because that is what a conforming scenario is written against.

**An overloaded enum literal is not yet resolved by context** — a known gap,
tracked as issue #110. §7.3.3 says such a literal "will depend on the type
requirements of the place it is used in"; Scena reports the ambiguity first. In
practice this means `at: start` is rejected and `at: at!start` is accepted,
because §8.9.19's `at` and §8.12's `route_overlap_kind` both declare `start`.
Nothing is blocked, but a scenario copied out of the specification may need the
literal qualified.

## What the DSL does not execute yet

Checked but not executed in v0.0.1, always with a structured diagnostic naming
the section: logical-scenario *selection* (ranges, distributions), constraints
that would need a solver, coverage collection, `emit`/`wait @event`/`on`/`until`
(§7.6.2.5's events have no runtime carrier), `along`/`along_trajectory` (a
concrete route comes only from §8.12.2's map methods, which the standard says
may be external), and the non-default parallel overlaps. The DSL coverage matrix
lists every one with its reason.

Nothing in that list is approximated. Where a value cannot be made concrete
without search, Scena reports it — reporting beats approximating when
determinism is the promise (ADR-0004).
