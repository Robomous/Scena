# ADR-0026: ManeuverGroup re-execution (§8.4.4)

- **Status:** accepted
- **Date:** 2026-08-01

## Context

`maximumExecutionCount` is a **required** attribute on `ManeuverGroup`, and
§8.3.3.2 covers it in the same sentence as the `Event` one. p1-s3 implemented
the Event counter and deferred the group's (ADR-0005, issue #52), because
§8.4.4's rule does not close on its own:

> If the number of executions is smaller than `maximumExecutionCount`, the
> `ManeuverGroup` transfers from `runningState` into `standbyState` and waits
> until the start trigger is executed. The start trigger is inherited from the
> enclosing `Act`.

A `ManeuverGroup` hosts no start trigger of its own (§7.6.1.1) and the Act's is
not re-evaluated while the Act runs. Under the Event re-arm rule, a re-armed
group would wait for a trigger that can never fire again — the field would be
dead data. Making it live requires choosing a restart rule, and both candidates
carry determinism consequences.

Until now the XML frontend read the attribute and reported that it was not
honoured, so a scenario asking for four executions got one and was told so.

## Decision

### The inherited start trigger is "the Act is running"

A group in `standbyState` starts on the next evaluation in which its Act is
still running. The scheduler only recurses into a running parent, so reaching a
standby group during the walk *is* the evidence that the inherited trigger has
fired; there is nothing further to evaluate.

This is the reading that keeps §8.4.4's own sentence true — the group waits in
standbyState and starts when its (inherited) start trigger is satisfied —
without inventing a rule that re-evaluates a running Act's trigger, which would
change Act semantics for every scenario, not just the ones using this field.

### One execution per evaluation

A group that re-arms is stamped with the current evaluation, and a standby group
is skipped for the rest of that evaluation. Two guards enforce it:

- `update()` stops advancing an element whose own start left it not-running, so
  an element cannot be entered and then advanced again in the same pass;
- a standby group whose last transition is stamped with the current evaluation
  does not start, which covers the case where the *parent's* `enter_running`
  already ran the group before the walk reached it.

Without the bound, a group whose maneuvers complete instantly would burn its
entire budget inside one evaluation, and the number of executions per step would
depend on where in the tree the group sits — a scenario's timing would follow
its nesting rather than its content. With it, `maximumExecutionCount = N` costs
at most N evaluations, deterministically.

### A new execution resets the subtree

Re-arming resets every descendant to `standbyState` with:

- its execution tally cleared, so an `Event` with `maximumExecutionCount = 1`
  fires once *per group execution*, not once per run. Otherwise the group's
  second execution would find an exhausted subtree and complete instantly having
  done nothing, which makes the count meaningless;
- its trigger **condition histories** cleared — the edge state and the delay
  samples. An execution must not depend on the previous one: a rising edge in
  execution 2 that was decided by a sample taken during execution 1 would make
  the executions non-independent and the behaviour hard to reason about;
- its running-action bookkeeping cleared.

The alternative — carrying histories over — was rejected because it makes a
group's Nth execution depend on the whole prior history of the run, which is
exactly the property that makes a repeated element useful to author against.

**The consequence is documented rather than worked around:** a rising-edge
condition that has already become true does not fire again in a later execution.
After the reset the condition has no previous sample, §7.6.4 makes that first
check false, and a constantly-true expression never produces a rising edge. A
group whose events are gated on rising edges therefore runs once and then waits.
That is the correct composition of two rules Scena already implements, and a
test pins it so it cannot drift silently.

### Counting, zero and stop

- Executions count **startTransitions only**. A group has no priority and
  therefore no `skipTransition` (§8.4.4), unlike an Event whose tally counts
  both (§8.4.2.1).
- `maximumExecutionCount = 0` is schema-valid and means the group never runs: no
  startTransition is ever performed, and with no skipTransition available it
  simply completes. Its subtree stays in `standbyState`.
- A negative count is rejected at `Engine::init`, the same reading the Event
  counter gets: the XSD type is `unsignedInt`, so a negative budget has no
  meaning at all.
- A stop trigger completes the group "regardless of the number of execution
  counts left" — `stop_cascade` already spends the budget before completing, so
  this needed no new code.

## Consequences

- `ir::ManeuverGroup` gains `maximum_execution_count` (default 1), validated at
  init and bound in Python. The XML frontend lowers the attribute instead of
  reporting it; its deferral test flipped to an assertion.
- `Scheduler::update` no longer advances an element in the same pass that
  started it. For every element other than a re-arming group this is a no-op —
  such an element is either Running (and was advanced anyway) or Complete (and
  returned already).
- Default behaviour is unchanged: with the default count of 1 a group completes
  exactly as before, which is what keeps the rest of the storyboard suite
  meaningful.
- No C ABI change. The C API builds storyboards through
  `scn_engine_add_*_action`, which synthesizes a single-execution group; a host
  that needs a repeated group uses the C++ or Python API.

## Alternatives considered

**Re-evaluate a running Act's start trigger.** The other candidate in #52. It
would let a group genuinely "wait for the start trigger". Rejected: it changes
what an Act's start trigger means for every scenario — a trigger that fires
twice would restart groups that had nothing to do with the feature — and §8.4.5
describes the Act trigger as the thing that takes the Act *into* runningState,
once.

**Restart within the same evaluation.** Also listed in #52. It makes a group of
instantaneous maneuvers spend its budget immediately, which reads naturally as
"run it N times". Rejected: the number of executions per step would then depend
on tree position, and an unbounded same-evaluation loop is exactly the kind of
thing the determinism contract exists to prevent.

**Keep descendant execution counts across executions.** Simpler: no reset at
all. Rejected: an `Event` with the default count of 1 would fire only in the
group's first execution, so a repeated group would repeat nothing.
