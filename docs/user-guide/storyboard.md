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
(stopped by a stop trigger). Semantics implemented per §8.3–§8.4:

- **Child start rule** (§8.3): when a parent enters running, children with
  a start trigger go to standby, all others start immediately. Only Acts
  and Events carry start triggers (§7.6.1.1).
- **Events** fire their actions on start; actions are instantaneous in
  this phase, so events complete in the same evaluation (§8.4.1–8.4.2).
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

## Triggers in this phase

A start/stop trigger is currently a single condition object; the full
trigger model (condition groups, edges, delays — §7.6) replaces it in the
next runtime sprint without changing the hierarchy. Event priorities and
`maximumExecutionCount` also arrive in a later sprint; every event
currently executes once.

## Example

See [`python/examples/hello_engine.py`](../../python/examples/hello_engine.py)
for a complete host loop that builds a storyboard, steps it at 100 Hz, and
observes an event's lifecycle.
