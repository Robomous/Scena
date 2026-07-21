# Triggers

Triggers decide *when* a storyboard element starts and *when* it is forced
to stop. Scena implements the model of ASAM OpenSCENARIO XML §7.6 as
written; this chapter is the operational summary, including the places
where the standard leaves a choice and Scena had to make one.

## Composition: OR of ANDs

A `Trigger` is an association of condition groups, and a `ConditionGroup`
is an association of conditions (§7.6.1):

```
T(t_d) = OR over groups ( AND over the group's conditions ( C(t_d) ) )
```

A group holds at least one condition (`ConditionGroup` class reference:
1..\*). An empty group would be a vacuously true conjunction, so
`Engine::init()` rejects it with `InvalidArgument` rather than inventing a
meaning for it.

A trigger with **no groups at all** is legal and **always false** (§7.6.1
keeps empty triggers only for backward compatibility). That is not the
same as having no trigger:

| Trigger field | Meaning |
|---|---|
| absent (`std::nullopt` / `None`) | No trigger. A start-trigger-less element starts with its parent (§7.6.1.1); a stop-trigger-less element is only stopped through an ancestor. |
| present but empty | Always false. The element never starts, respectively is never stopped by its own trigger. |

Conditions are evaluated **without short-circuiting**: every condition of
every reachable trigger is evaluated exactly once per evaluation, whatever
the group's outcome. Edges and delays depend on a condition's own history,
so skipping one would make the result depend on the order the conditions
happen to be written in.

## Hosting and inheritance

| Trigger | Hosted by | Inherited by |
|---|---|---|
| Start (§7.6.1.1) | `Act`, `Event` | Elements without one start with their parent. `Story` never has one. |
| Stop (§7.6.1.2) | `Storyboard`, `Act` | **All** descendants, even ones that host their own. |

A firing stop trigger moves the element and its whole subtree to
completeState through a `stopTransition`, out of standbyState as well as
runningState, and clears the remaining execution counts of the stopped
elements. A running element terminates immediately.

## Condition edges

Each condition carries an edge modifier over its logical expression `LE`
(§7.6.2), evaluated at the current discrete time `t_d` and the previous
evaluation `t_{d-1}`:

| Edge | Formula | Section |
|---|---|---|
| `none` | `LE(t_d)` | §7.6.2.4 |
| `rising` | `LE(t_d) AND NOT LE(t_{d-1})` | §7.6.2.1 |
| `falling` | `NOT LE(t_d) AND LE(t_{d-1})` | §7.6.2.2 |
| `risingOrFalling` | rising OR falling | §7.6.2.3 |

Corner cases (§7.6.4), all implemented and tested:

- The **first check** of a condition defined with an edge is always
  `false`. A constantly true expression therefore never produces a rising
  edge.
- That first check happens when the enclosing element enters
  standbyState — for an event, when its act starts running. An expression
  that was already true before the act started shows no rise.
- An edge is a **one-evaluation-wide pulse**. In an AND group, the other
  conditions must hold at that very evaluation.

## Condition delays

A delay `Δt` (seconds, `Δt >= 0`) shifts the condition into the past:
`C_D(t_d) = C(t_{d-Δ})` (§7.6.3). Two decisions Scena had to make:

- **The delay applies after edge detection.** §7.6.3 phrases the delay
  over the logical expression, while the `Condition.delay` class reference
  states the delay elapses once the edge condition is verified. The class
  reference is the more specific statement and wins: the pipeline is
  *expression → edge → delay*, so a delayed rising edge is a pulse shifted
  by `Δt`, not widened.
- **The lookup is sample-and-hold.** The host chooses the step times, so
  there is usually no evaluation at exactly `t_d - Δt`. Scena takes the
  most recent evaluation at or before that target. With no such
  evaluation the condition is `false`, which covers the §7.6.4 rule that a
  delayed condition is false while `t_d < Δt`. Time comparisons are exact,
  with no tolerance — an epsilon would make results depend on the
  magnitude of the times and break bit-identical reproducibility.

`Δt` must be non-negative: `Engine::init()` returns `InvalidArgument` for
a negative or NaN delay, per rule
`asam.net:xosc:1.0.0:data_type.condition_delay_not_negative`.

If the host steps twice at the same simulation time (`dt = 0`), each call
is a new discrete evaluation `t_d`, so the second sees the first as
`t_{d-1}` — and the delayed lookup reads the most recent sample at that
time, i.e. the later one.

## Evaluation order: stop before start

Within one evaluation, stop triggers are checked before start triggers, at
every level: the storyboard's before the storyboard walk, an act's before
its own start trigger. If an act's stop and start triggers both hold at
the same discrete time, the act is **stopped, not started**. This makes
the outcome of a same-step collision defined rather than incidental.

Otherwise the walk visits elements in document order, and the actions of
events that start in the same evaluation fire in document order — delayed
conditions included.

## Building triggers in Python

```python
import scena as scn

# The common case: one logical expression, optional edge and delay.
trigger = scn.make_trigger(
    scn.SimulationTimeCondition(at_time=3.0),
    edge=scn.ConditionEdge.Rising,
    delay=0.5,
)

# The general case: OR of groups, AND within a group.
group = scn.ConditionGroup()
group.add_condition(scn.TriggerCondition(scn.SimulationTimeCondition(at_time=1.0)))
group.add_condition(
    scn.TriggerCondition(
        scn.SimulationTimeCondition(at_time=2.0),
        edge=scn.ConditionEdge.Rising,
        delay=0.25,
        name="ramp-start",
    )
)
composed = scn.Trigger()
composed.add_group(group)

event = scn.Event("slow-down", start_trigger=composed)

act = scn.Act("act")                          # no start trigger: starts with the storyboard
act.set_stop_trigger(scn.make_trigger(scn.SimulationTimeCondition(at_time=4.5)))
```

`scn.ConditionEdge.NoEdge` is the `none` edge (`None` is a Python
keyword). A complete runnable example lives in
[`python/examples/hello_engine.py`](../../python/examples/hello_engine.py).

## Triggers and the event lifecycle

A start trigger is evaluated exactly once per evaluation, and only while its
element is in standbyState. A running or complete element has no reachable
start trigger — §7.3.2: "start triggers only make sense for events in
standbyState" — so its condition histories are frozen while it runs, and an
event never has two simultaneous instantiations.

Two consequences for events that execute more than once
(see [Execution counts](storyboard.md#execution-counts)):

- **Re-arming does not reset condition history.** Only binding the
  storyboard rebuilds it. An event with a rising-edge trigger therefore
  re-executes on the next *rise*, not on the next evaluation at which the
  expression is still true; an event with an edgeless condition re-executes
  on the very next evaluation.
- **Resolving event priority never replaces a trigger evaluation.** A `skip`
  event stands by, its trigger is evaluated normally and its edge and delay
  histories advance in lockstep with every other element's. Only the
  transition that follows the evaluation differs.
