# ADR-0005: Action lifetime, event priority and the p1-s3 scope line

- **Status:** accepted
- **Date:** 2026-07-21

## Context

ASAM OpenSCENARIO XML defines event priority entirely in terms of *running*
events: `override` "terminates any running Event in the same scope
(Maneuver)" (§8.4.2.2), and `skip` "will not be ran if there is any other
event in the same scope (maneuver) in the running state" (`Priority` class
reference).

Until p1-s3 the runtime completed every event in the evaluation that started
it, because every action it could apply finished instantly. No event was ever
in `runningState` when a sibling was evaluated, so all three priority
literals were unreachable. Implementing §8.4.2.2 therefore requires a way for
an event to persist across evaluations — that is, some notion of how long an
action lasts.

The obvious full answer, a control-strategy and action-instance model with
conflict resolution per §7.5, has no subject yet: §7.4.1.2 states that a
`SpeedAction` used with the step dynamic option "does not assign a control
strategy as the changes are enacted instantaneously", and that is exactly the
only action Scena has. Building the conflict machinery now would produce code
no shipped action can reach, designed for a consumer (transition dynamics)
that does not exist until P2.

## Decision

### A two-valued action outcome, reported by the applier

The scheduler's fire callback returns a `runtime::ActionOutcome`:

- `Complete` — the action reached its goal in the evaluation it was applied
  in. This is the §7.4.1.2 step-dynamic case and covers every action in the
  Scenario IR today.
- `Ongoing` — the action cannot end by itself; only a stopTransition ends it.
  This is the §7.5.3 never-ending case.

An event ends regularly in its starting evaluation when every action reported
`Complete`, and otherwise stays in `runningState` (§8.4.2). Every action is
still fired, whatever an earlier one reported.

These are the two cases the standard describes completely. The missing middle
— actions whose end is governed by `TransitionDynamics` — is deliberately
absent and arrives as an **additional enumerator** with p2-s2/p5-s4. Nothing
about the interface has to change to accommodate it.

The classification lives in the applier, not in `ir::Action`. `Engine::apply`
reports `Complete` for `SpeedAction`, honestly and per spec.

### Alternatives rejected

- **A virtual on `ir::Action`** (`instantaneous()`, or an `ActionKind`).
  `ir::Action` is public IR and is bound to Python; p2-s2 and p5-s4 replace a
  static boolean with a dynamics-driven lifetime, so this is public API churn
  that gets redone. It is also incomplete on its own: it says an event should
  keep running but never says when it stops, so it needs a completion channel
  anyway.
- **A completion-polling sink or registry** keyed by action. Actions are held
  by `shared_ptr` and are shared between events, and re-execution applies the
  same object several times, so any sound version needs (event instance,
  action instance) identity. That identity model *is* the p5-s4 design; it
  would be built here against test doubles only and redesigned there.
- **A richer callback carrying event identity.** Nothing in p1-s3 consumes
  it; speculative surface.
- **Events end at the end of the evaluation that started them.** Zero API
  change, but `override` would then only ever alter an element state after
  the victim's actions had already fired, and would remain a no-op across
  steps. It also invents a lifetime rule the standard does not have.

### Scope line: §7.5 and ManeuverGroup re-execution are deferred

p1-s3 delivers the **event** lifecycle. Deferred, with nothing left unused:

- **§7.5 action conflict resolution, control-strategy domains, continuous
  actions and bulk actions over resolved `ManeuverGroup` actors → p5-s4**,
  whose issue already scopes "event-priority interaction with running motion
  actions verified (conflict rules §7.5)". Reason: §7.5.1 defines a conflict
  as competing "for control of the same domain in the same resource", and no
  shipped action assigns a control strategy (§7.4.1.2). A domain taxonomy and
  conflict registry would be unreachable until p2-s2.
- **`ManeuverGroup::maximumExecutionCount` (§8.4.4) → p4-s2**, the sprint
  that must load the required XML attribute. Reason: §8.4.4 re-arms the group
  onto the start trigger inherited from the enclosing Act, which is not
  re-evaluated while the act runs. Any behaviour therefore requires an
  invented restart rule — "a running act re-evaluates its start trigger", or
  "a group restarts within the same evaluation, resetting its subtree" — both
  with determinism consequences and neither derivable from the text. Shipping
  a field with no behaviour, or an invented behaviour, is worse than shipping
  the decision explicitly.

## Spec ambiguities resolved here

Later sprints must not silently re-decide these.

| # | Question | Resolution | Basis |
|---|---|---|---|
| a | §8.4.2.2 describes `skip` as unconditional, §7.3.2 as "does not leave standbyState" | Neither: the `Priority` class reference makes it conditional on a running sibling. With none, the event starts normally; with one, it performs a skipTransition | `Priority` class reference, §8.4.2.2 |
| b | `skipTransition` has two distinct meanings (§8.2 priority skip, §8.4.2.1 exhausted count) | Both are implemented and both emit `TransitionKind::Skip`; they are distinguished by the state the element ends up in, not by the transition | §8.2, §8.4.2.1 |
| c | No ordering rule for events of one Maneuver triggering at the same discrete time | Single pass in document order. Of two events triggering together where the later is `override`, the later wins; a `skip` placed before an event that starts in the same evaluation is not skipped | Scena determinism contract |
| d | Does `override` stop *standby* siblings? | No — running only. §7.3.2's "all other events" is prose; stopping standby siblings would also permanently kill re-executable events, contradicting §8.3.3.2 | §8.4.2.2, `Priority` class reference |
| e | Does `override` stop the triggering event itself? | No. "all events … as the starting event" excludes it, and the case cannot arise: a running event has no reachable start trigger | §8.4.2.2, §7.3.2 |
| f | Does priority apply to an event without its own start trigger? | Yes. Such an event inherits the Act's start trigger (§8.4.2), so it is a triggered event and goes through the same path | §8.4.2, §8.3 |
| g | `maximumExecutionCount = 0` (schema-valid; the `[1..inf[` range is documentation-only and carries no rule id) | Accepted. §8.4.2.1's standby rule already gives it a coherent reading: the event is exhausted before it starts and completes with a skipTransition, never executing. Negative counts are rejected at `Engine::init` | §8.4.2.1 |
| h | Re-execution of an event without its own start trigger | It re-arms to standbyState and waits on the Act-inherited trigger, which has already fired and is not re-evaluated while the act runs, so it executes once whatever its budget says. No same-evaluation restart loop is invented | §8.4.2, §8.4.2.1 |

Re-arming does **not** reset condition history — only `bind()` rebuilds it —
so an event with a rising-edge trigger re-executes on the next rise rather
than immediately.

## Consequences

- `Scheduler::FireCallback` changes shape. It is an internal runtime
  interface: the C ABI and the Python bindings do not expose it, and both
  gained only additive symbols.
- Through `Engine`, no action is ever `Ongoing`, so `override` and `skip`
  currently resolve to plain starts and are exercised by GoogleTest appliers
  that report `Ongoing`. This is a capability boundary, not dead code: every
  caller of the public `runtime::Scheduler` API can reach the branch, which
  is how the scheduler's own suites drive it. It closes when p2-s2 lands
  actions with duration.
- Priority resolution never replaces a trigger evaluation. A start trigger is
  evaluated exactly once per evaluation while its element is in standbyState,
  and a `skip` event's condition histories advance exactly as any other's —
  otherwise edge and delay histories would fall out of lockstep and the
  bit-identical reproducibility contract would break.
- The unit of scheduling for events becomes the Maneuver, which is the scope
  §7.3.3 defines priority over and the only element that can see all the
  siblings a starting event interacts with.

## References

- ASAM OpenSCENARIO XML §7.3.2, §7.3.3, §7.4.1.1–§7.4.1.2, §7.5, §8.2,
  §8.3.3.2, §8.4.2, §8.4.4, and the `Priority` class reference.
- [ADR-0003](ADR-0003-simulator-gateway.md) — the step API and the boundary
  the host is allowed to influence.
- `docs/roadmap/coverage/osc-xml-coverage.md` — the deferral rows.
