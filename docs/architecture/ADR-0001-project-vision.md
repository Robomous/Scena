# ADR-0001: Project vision — two frontends, one runtime

- **Status:** accepted
- **Date:** 2026-07-21

## Context

ASAM OpenSCENARIO exists in two generations: the XML format (1.0–1.3), widely
supported across the simulation ecosystem, and the DSL (2.x), a new
domain-specific language for which no native C++ execution engine currently
exists. Simulation platforms need an embeddable engine that executes both,
behaves identically regardless of the input format, and integrates into an
existing simulation loop without imposing its own runtime model.

## Decision

Scena is a **library-first** scenario execution engine:

1. **Embeddable C++20 core.** The engine ships as a library with a stable C
   ABI (`capi/`) and Python bindings (`python/`). Any player or CLI tool is a
   thin consumer of that API, never part of the core.
2. **Two frontends, one runtime.** Both OpenSCENARIO XML and OpenSCENARIO DSL
   compile into a common intermediate representation, the **Scenario IR**
   (`core/include/scena/ir/`). A single runtime executes the IR. Frontends
   depend on the core; the core knows nothing about frontends; frontends never
   depend on each other.
3. **The simulator owns the clock.** The engine exposes a step-based API
   (`init → step(dt) → query/report state → close`), spawns no threads, and
   imposes no main loop. Control ownership is per entity: engine-controlled
   (default behavior) or host-controlled (the host simulator reports states).
   See ADR-0003.
4. **Standards correctness.** The ASAM OpenSCENARIO specifications are the
   sole normative reference; behavior is implemented against them and cited by
   section number. See ADR-0002 for the clean-room policy.

Scena is a standalone project. Other tools may consume it as an ordinary
dependency; Scena never depends on them.

## Phase plan

| Phase | Scope |
|-------|-------|
| **F0** | Project scaffold + kernel skeleton: minimal Scenario IR, deterministic clock and scheduler, step-based engine with per-entity control ownership, C API and Python binding skeletons, CI |
| **F1** | XML frontend + runtime core: real `.xosc` parsing, the OpenSCENARIO condition/action catalog, trajectories |
| **F2** | DSL frontend: parser + type checker for the finalized 2.x standard libraries |
| **F3** | DSL concrete-scenario execution over the shared runtime |
| **F4** | Constraint solving / abstract scenarios (feasibility-gated) |

Each phase ends at an explicit gate (see `docs/roadmap/roadmap.md`); work
beyond the active gate does not start without a maintainer decision.

## Consequences

- The Scenario IR is the central contract of the codebase. Both frontends and
  the runtime evolve against it, so IR changes carry the highest review bar.
- Executing the DSL through the same IR as XML forces early IR design toward
  the concepts shared by both standards rather than XML-specific structure.
- Library-first means every capability must be reachable through the C ABI and
  Python bindings; features that only work in some bundled tool are rejected
  by design.
- The F0 storyboard (a flat condition/action list) is a deliberate
  simplification and will be replaced by the full OpenSCENARIO storyboard
  hierarchy in F1 without changing the two-frontends-one-runtime shape.
