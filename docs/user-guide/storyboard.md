# The storyboard model

Scena executes the ASAM OpenSCENARIO storyboard hierarchy:

```
Storyboard ─ Story (0..*) ─ Act (1..*) ─ ManeuverGroup (1..*)
                                          └─ Maneuver (0..*) ─ Event (1..*) ─ Action (1..*)
```

(nesting and cardinalities per ASAM OpenSCENARIO XML 1.4.0 §8.3.2). In this
phase the Scenario IR is built in code — through C++, the C API, or the
Python bindings; the XML frontend will produce the same IR from `.xosc`
files.

## Element lifecycle

Every storyboard element moves through the states of XML §8.1, observable
through `Engine::storyboard_element_state()` (Python:
`Engine.storyboard_element_state(path)`; paths are element names joined
with `/`, e.g. `"story/act/group/maneuver/event"`, and the empty path
addresses the storyboard itself):

- **standby** — waiting for a start trigger,
- **running** — executing,
- **complete** — finished regularly or stopped.

Transitions (§8.2) are observable through
`storyboard_element_transition()`: `Start`, `End` (regular end), `Stop`
(stopped by a stop trigger or by an overriding event) and `Skip`. `Skip` is
specific to Events and the standard gives it two meanings, both of which
Scena emits: an event that could not start because of its `skip` priority
(§8.2), and a standby event whose executions reached its
`maximumExecutionCount` (§8.4.2.1). They are told apart by the state the
element ends up in. Semantics implemented per §8.3–§8.4:

- **Child start rule** (§8.3): when a parent enters running, children with
  a start trigger go to standby, all others start immediately. Only Acts
  and Events carry start triggers (§7.6.1.1).
- **Events** fire their actions on start and end regularly once every
  action completed (§8.4.1–8.4.2). Every action Scena can apply today
  reaches its goal in the evaluation it was applied in — §7.4.1.2's step
  dynamics — so events complete in the same evaluation.
- **Completion propagates upward**: a Maneuver completes when all its
  Events completed, a ManeuverGroup when all Maneuvers (an empty group
  completes instantly, §8.4.4), an Act when all groups, a Story when all
  Acts (§8.4.3–8.4.6).
- **The storyboard never completes on its own** (§8.4.7): only its stop
  trigger completes it (stopping everything still executing); without one
  it runs until the host stops stepping. Hosts can use the storyboard
  state to decide when to stop the simulation.
- Sibling elements execute **in parallel** (§8.3.3.1); the engine walks
  them in document order, which keeps runs deterministic.

Element names address the state query, so the engine requires them to be
non-empty and unique among siblings at `init()`.

## Init phase

`Scenario.init_actions` run during `Engine::init()`, before simulation
time starts (§8.5): they consume no time and set up initial entity states.
The storyboard then enters running and is evaluated once at t = 0, so
start conditions that already hold (e.g. a simulation-time condition at
0 s) fire before the first host step.

## Triggers

Start and stop triggers are the full §7.6 model — an OR of condition
groups, each an AND of conditions with edge and delay modifiers. See
[Triggers](triggers.md) for composition, hosting and inheritance rules,
the edge formulas and the delay lookup.

A start trigger is evaluated exactly once per evaluation and only while its
element is in standbyState — a running or complete element has no reachable
start trigger, so its condition histories are frozen (§7.3.2: "start
triggers only make sense for events in standbyState").

## Event priority

Every event carries a priority that governs how it interacts with the other
events **of its own Maneuver** — that is the scope, not the maneuver group:
"A maneuver groups events creating a scope where the events can interact
with each other using the event priority rules" (§7.3.3).

| Priority | Behaviour | Reference |
|---|---|---|
| `parallel` (default) | Starts regardless of the states of the other events of the Maneuver. | §8.4.2.2 |
| `override` | Stops every event of the Maneuver currently in **running** state — each with a stopTransition and with its remaining executions cleared — and then starts. Standby siblings are untouched. | §8.4.2.2 |
| `skip` | Does not start while another event of the Maneuver is running: it performs a skipTransition instead, which counts as an execution. With no running sibling it starts normally. | `Priority` class reference |

Two readings worth stating, because the prose of §7.3.2 is looser than the
normative text: `override` targets running events only (§7.3.2's "all other
events in the scope" is prose), and `skip` is conditional rather than an
unconditional refusal to leave standby.

An overriding event and a stop trigger take the same code path, so a
stopped event and an overridden one are indistinguishable — which is what
§8.4.2 requires ("The Event transfers to completeState with stopTransition
under two conditions").

The deprecated pre-1.3 literal `overwrite` carries a description that is
word-for-word identical to `override`, so it is a purely lexical synonym.
The IR has no separate value for it; the XML frontend will map it at load
time.

Priority also applies to an event without its own start trigger, because
such an event inherits the enclosing Act's start trigger (§8.4.2) and is
therefore a triggered event like any other.

### Ordering when several events trigger together

The standard gives no rule for two events of one Maneuver whose start
triggers hold at the same discrete time. Scena resolves them in a single
pass in **document order**, so every decision is a pure function of the time
and of the decisions taken by strictly earlier siblings. Two consequences:

- of two events triggering together where the later one is `override`, the
  later one wins — it stops the one that just started;
- a `skip` event placed *before* an event that starts in the same
  evaluation is not skipped, because nothing was running yet when its turn
  came.

Resolving priority never replaces a trigger evaluation: a `skip` event
stands by, its trigger is evaluated normally and its edge and delay
histories advance exactly as any other element's. Only the transition that
follows the evaluation differs.

## Execution counts

`Event.maximum_execution_count` is the budget of §8.3.3.2; the executions
are performed sequentially, never concurrently — "there shall not be
multiple instantiations of the same event running simultaneously" (§7.3.2).
An event's executions are the sum of its startTransitions **and** its
skipTransitions (§8.4.2.1).

- Leaving runningState with an endTransition returns the event to
  **standby** while executions remain, and completes it once they are
  exhausted.
- A standby event whose executions already reached the maximum completes
  with a **skipTransition**.
- Re-arming does **not** reset condition history — only binding the
  storyboard rebuilds it — so an event with a rising-edge trigger
  re-executes on the next rise rather than immediately.
- A stop trigger, and an overriding sibling, complete the event
  "regardless of the number of executions left" (§8.4.2.2): the remaining
  budget is cleared.
- `maximum_execution_count = 0` is schema-valid and means the event never
  executes: it is exhausted before it starts and completes with a
  skipTransition. A negative count is rejected at `init()`.
- An event **without its own start trigger** executes once whatever its
  budget says: it re-arms to standby and waits on the Act-inherited start
  trigger, which has already fired and is not re-evaluated while the act
  runs.

## Action lifetime and conflicts

The scheduler asks the applier what happened to each action: it either
completed in the evaluation it was applied in, is running under transition
dynamics and will end on its own, or is ongoing and can be ended only by a
stopTransition (§7.5.3). An event stays in runningState while any of its
actions is running or ongoing, and ends regularly when the last one completes
(§8.3.3.1).

### Which actions conflict

An action conflicts with another only when both "compete for control of the
same domain in the same resource" (§7.5.1) — the same entity, and an
overlapping set of control domains. `control_domains()` writes the §7.4.1.1
domains and Annex A Table 10 down in one place:

| Action | Domains |
|---|---|
| `SpeedAction`, `SpeedProfileAction`, `LongitudinalDistanceAction` | longitudinal |
| `LaneChangeAction`, `LaneOffsetAction`, `LateralDistanceAction` | lateral |
| `FollowTrajectoryAction` with `timeReference = timing` | lateral + longitudinal |
| `FollowTrajectoryAction` without a timing | lateral |
| everything else (`TeleportAction`, routing, controller, visibility, global) | none |

The classification depends on the action's **settings**, not only its type.
§7.4.1.2: a `SpeedAction`, `LaneChangeAction` or `LaneOffsetAction` "used with
the step dynamic option ... does not assign a control strategy as the changes
are enacted instantaneously". A Step-shaped action therefore holds no domain: it
writes its value and completes, and it does **not** override a control strategy
already running on that entity — which resumes from its own schedule on the next
step. Scena reports that situation when it arises mid-run, because writing a
value something else immediately overwrites is rarely the authored intent. In
`Init`, where nothing else is running, a Step action is the ordinary case and
nothing is reported.

When two actions do conflict, the newly triggered one wins and the running one
"is overridden ... equivalent to issuing a stop trigger to that action"
(§7.5.1): it moves to completeState with a stopTransition on its next re-poll,
so its event can end.

### Stopping an event stops its actions

A stopped event "is forced to stop all actions in its scope" (§7.5.2.1). The
engine releases whatever control strategy each of them had assigned and leaves
the entity as the release found it — holding its speed, holding its heading. A
never-ending action is ended this way too: it has no regular ending, but every
other ending in §7.5.2.1 applies to it (§7.5.3).

### Bulk actions

When a `ManeuverGroup`'s actors resolve to several entities, its private actions
are applied "in parallel to all given ScenarioObject instances" (§8.3.3.3).
Scena represents that as one action instance per actor, sharing a bulk group id.
Two consequences follow:

- the event ends only when **every** instance has ended — that is just the
  Event's own all-actions join, and it is what §7.5.4 asks for;
- a conflict on **any** one actor overrides the whole action: every instance is
  retired and all of its entities fall back to default behavior together
  (§7.5.4).

`ManeuverGroup`'s own `maximumExecutionCount` (§8.4.4) remains deferred. See
[ADR-0005](../architecture/ADR-0005-action-lifetime-and-event-priority.md) for
the event lifetime and priority reasoning, and
[ADR-0025](../architecture/ADR-0025-action-conflicts-and-bulk-actions.md) for
the conflict, override-by-event and bulk rules.

## Example

See [`python/examples/hello_engine.py`](../../python/examples/hello_engine.py)
for a complete host loop that builds a storyboard, steps it at 100 Hz, and
observes an event's lifecycle.
