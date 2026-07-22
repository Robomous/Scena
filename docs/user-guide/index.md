# Scena user guide

The user guide grows sprint by sprint along the
[roadmap](../roadmap/roadmap.md). Current chapters:

- [The storyboard model](storyboard.md) — hierarchy, element lifecycle,
  init phase, event priority and execution counts, and how to observe
  element states.
- [Triggers](triggers.md) — condition groups, edges, delays, stop-trigger
  inheritance, and the stop-before-start evaluation order.
- [By-value conditions](conditions.md) — the Rule comparator, the
  simulation-time / parameter / variable / user-defined-value / time-of-day /
  storyboard-element-state conditions, the host-value interface (C++/C/Python),
  and the one-evaluation transition window.
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
