# Scena user guide

The user guide grows sprint by sprint along the
[roadmap](../roadmap/roadmap.md). Current chapters:

- [The entity model](entities.md) — ScenarioObject and the Vehicle /
  Pedestrian / MiscObject taxonomy, bounding boxes, performance limits, the
  full h/p/r pose, and how to build and query entities from C++/C/Python.
- [Motion](motion.md) — longitudinal dynamics: SpeedAction transition
  dynamics (shapes × dimensions), the default controller and its Performance
  clamp, speed profiles, distance keeping, the point-mass simplifications, and
  how to drive speed from C++/C/Python.
- [Positions and control ownership](positions.md) — the ten §6.3.8 position
  variants and the one resolver that maps them to a world pose, orientation
  composition, and engine- vs host-controlled entities (the report_state
  round-trip and mode violations).
- [Routing, trajectories and controllers](routing.md) — routes and waypoints,
  polyline trajectories with and without a time reference, controller
  assignment and per-domain activation, and entity visibility, plus which of
  those actions consume simulation time.
- [Global and infrastructure actions](global-actions.md) — the actor-less
  actions: variables and the deprecated parameter actions, the add/delete
  entity lifecycle, the environment store and its time-of-day clock, traffic
  signal controllers and their phase clock, and custom commands through the
  gateway.
- [The storyboard model](storyboard.md) — hierarchy, element lifecycle,
  init phase, event priority and execution counts, and how to observe
  element states.
- [Triggers](triggers.md) — condition groups, edges, delays, stop-trigger
  inheritance, and the stop-before-start evaluation order.
- [Conditions](conditions.md) — the by-value conditions (Rule comparator,
  simulation time, parameter/variable/user-defined value, time of day,
  storyboard element state) and the by-entity conditions (triggering-entities
  any/all, speed/relative-speed/acceleration, stand still, traveled distance,
  reach position), the traffic-signal conditions, the scalar-velocity model, the observation-sampling
  contract, and the host interface (C++/C/Python).
- [Error handling](error-handling.md) — status codes, structured
  diagnostics, the severity/status invariant, the path grammar, and the
  C-ABI borrowed-string lifetime.
- [Determinism](determinism.md) — the bit-identity contract, how the engine
  guarantees it, deterministic transcendentals (detmath), what hosts must
  uphold, and the cross-platform replay/trace harness.

Also see:

- [README](../../README.md) — mission, architecture, quickstart.
- [`python/examples/hello_engine.py`](../../python/examples/hello_engine.py) —
  minimal embedding loop through the Python bindings.
- [Architecture decision records](../architecture/) — API contract and design
  rationale.
