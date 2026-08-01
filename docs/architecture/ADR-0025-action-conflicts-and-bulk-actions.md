# ADR-0025: Action conflicts, override by event, and bulk actions (§7.5)

- **Status:** accepted
- **Date:** 2026-08-01

## Context

ADR-0005 deferred §7.5's runtime rules because, at the time, every action Scena
executed was instantaneous: there was nothing to conflict with. p2-s2 and the
P5 sprints changed that, and p5-s4 landed a *minimal* resolution — one
longitudinal slot and one lateral slot per entity, a later action retiring the
earlier one. Issue #51 collects what that left out:

- **Which actions conflict at all.** The engine decided by type: anything in the
  longitudinal `dynamic_cast` branch took the longitudinal slot. §7.4.1.2 says
  otherwise — the classification is settings-dependent, and a Step-shaped
  action assigns no control strategy.
- **Override by event (§7.5.2.1).** A stopped event marked its element Complete
  and left the engine still driving the entity from an action that no longer
  existed as far as the storyboard was concerned.
- **Bulk actions (§7.5.4, §8.3.3.3).** `ManeuverGroup::actors` was validated and
  then ignored by the runtime. The XML frontend already emits one action per
  actor, so *completion* was right by construction; *override* was not.

## Decision

### `ir::ActionDomain` and `control_domains`

A new header, `scena/ir/action_domain.h`, declares the §7.4.1.1 domain set —
Longitudinal, Lateral, Lighting, Animation — as a bitmask, plus
`control_domains(const Action&)`, which is Annex A Table 10 written down once.
Lighting and Animation are declared although no v0.0.1 action claims them: the
conflict rule should be stated against the standard's domain set, not against
whichever subset happens to exist.

It takes an `Action`, not a kind string, because the classification depends on
the action's settings. `conflicts(a, b)` is set intersection — §7.5.1's
"competing for control of the same domain in the same resource".

The engine now asks `control_domains` before superseding. That is a
**behavior change**, and the interesting case is the one the standard spells
out:

> LaneChangeActions, SpeedAction, LaneOffsetAction may be used to set a state,
> if used with the step dynamic option. In this particular use case, these
> actions do not assign a control strategy as the changes are enacted
> instantaneously. — §7.4.1.2

So a Step-shaped `SpeedAction` fired while a ramp is running **no longer
overrides the ramp**. It writes the speed and completes; the ramp, which was
never in conflict with it, resumes from its own schedule on the next step and
overwrites the value. Two tests that asserted the old behavior were rewritten
against the new rule, and one of them now uses a shaped action — the actual
conflict case — to keep supersession covered.

Writing a speed that a control strategy erases one step later is almost
certainly not what an author meant, so the engine **reports it** the first time
it happens on an entity, citing §7.4.1.2. The standard's own guidance is that
"it is up to the implementation of the simulator to warn the users about
potential unintended behavior, for example ... assigning conflicting control
strategies simultaneously" (§7.5.2.2). Rejecting the scenario would be wrong —
the file is valid, and a Step action in Init (where nothing is running) is the
ordinary, intended use.

### Override by event: the scheduler names the actions, the host releases them

`Scheduler::step` takes an optional `StopCallback`. `stop_cascade` calls it for
every action still in an event's running set, in document order, **before**
marking the element Complete, then clears that set.

The split follows the existing layering: the scheduler owns storyboard element
state and knows nothing about entities; only the engine knows that a given
action was holding a longitudinal controller or a trajectory follower.
`Engine::stop_action` releases exactly the domains that action owned — a
stopped event must not disturb a strategy some other action assigned — and
leaves the entity in the state the release found it: speed held, heading held.

The existing `step` overloads keep working; they pass a no-op. A caller with no
per-action state has nothing to release.

This also closes §7.5.3's loose end from the other direction: a never-ending
action has no regular ending, but "all action ends described in §7.5.2.1 are
applicable", and a stopTransition is now one of them in practice, not only on
paper.

### Bulk actions: one `ir::Action` per actor, sharing a group id

`ir::Action` gains `bulk_group()` — a `std::size_t`, 0 meaning "stands alone".
The XML frontend already lowers a private action once per actor of its
ManeuverGroup (§8.3.3.3, "applies the PrivateAction in parallel to all given
ScenarioObject instances"); it now stamps those instances with a shared id
drawn from a per-document counter, so the ids are a function of document order
alone and two loads of the same bytes produce the same IR.

One instance per actor is what makes the rest fall out for free:

- each instance carries its own `entity_id()`, so nothing in the runtime needs
  an actor parameter, and the whole existing per-entity ownership machinery
  applies unchanged;
- **completion** needs no new code at all. The instances share an Event, and an
  Event already ends only when all of its actions have (§8.3.3.1) — which is
  exactly the join §8.3.3.3 describes. §7.5.4's "only complete if all instances
  have completed" was already true.

The group id is load-bearing for exactly one rule, §7.5.4's:

> If any of the corresponding instances of Entity sparks a conflict with a newly
> started action, then the running action is overridden. All its instances of
> Entity are supposed to fall back to default behavior simultaneously.

`Engine::retire_bulk_siblings` walks the entity table — a `std::map`, so in
entity-id order on every platform — and retires every owner sharing the group.
Retiring a sibling is itself a supersession, and superseding propagates to the
group, so a single `retiring_bulk_group_` guard breaks the recursion.

Siblings are *retired*, not released outright: each is still Running inside its
own event, and its next re-poll is what turns that into a Complete. That is the
same mechanism ADR-0013 introduced, reused rather than duplicated.

## Consequences

- A Step-shaped motion action is now inert with respect to a running control
  strategy. This is the one place where §7.5 made Scena less aggressive rather
  than more.
- `Scheduler::step` has a third overload. `scheduler.h` is a core header, not
  the C ABI, so no stability commitment is touched; the C API and the bindings
  need no change, and none was made.
- `ir::Action` grows one `std::size_t`. It is set while a scenario is built and
  never during execution, so the "IR is immutable once the engine holds it"
  invariant stands.
- A host that builds scenarios through the C++ or Python API and wants bulk
  semantics sets the group id itself; the XML frontend does it automatically.
  A group of one is legal and behaves identically to no group.
- Still open: §7.5.1's "conflicts of actions of different types ... need to be
  identified in a case-by-case basis" beyond the two motion domains. Lighting
  and Animation have no runtime, so there is nothing to arbitrate yet — the
  enum is ready for them.

## Alternatives considered

**Keep type-based conflict classification.** No behavior change, no tests to
rewrite. Rejected: it makes a Step `SpeedAction` override a running ramp, which
§7.4.1.2 explicitly says it does not do, and it leaves the classification
scattered across `dynamic_cast` branches where nobody can check it against
Table 10.

**Reject a Step action that lands on a busy domain.** Rejected: the file is
valid and the situation is legal. §7.5.2.2 asks implementations to *warn* about
unintended behavior, which is what Scena does.

**Give the scheduler an actor parameter and expand actors in the kernel.**
Closer to reading §8.3.3.3 literally as "one action, many actors". Rejected: it
would change `FireCallback` for every caller, thread an actor through every
`apply` path, and duplicate the per-entity bookkeeping that already exists —
all to express something one action object per actor expresses for free.

**Track bulk membership in a set on the engine instead of an id on the action.**
Rejected: the set would have to be invalidated when an event re-arms, and there
is no signal for that. An id on the action is immutable and needs no lifetime
rule.
