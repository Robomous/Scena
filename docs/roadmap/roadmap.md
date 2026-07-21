# Kinema roadmap

Phased development with explicit gates. A phase is done only when every gate
criterion is verified; work beyond the active phase does not start without a
maintainer decision. The maintainer alone handles releases, tags, and
packaging.

## F0 — Scaffold + kernel skeleton *(current)*

Scope: repository scaffold, build system, minimal-but-real kernel: Scenario IR
(entities, simplified storyboard, `SpeedAction`, `SimulationTimeCondition`),
deterministic clock and scheduler, step-based engine with per-entity control
ownership, gateway and road-query interfaces, C API and Python binding
skeletons, docs and CI.

Gate:
- Full repository layout in place; all stubs compile.
- `cmake -B build && cmake --build build && ctest` green locally with zero
  warnings; all F0 test suites (version, clock, scheduler, engine, C API,
  determinism, Python) implemented and passing.
- Python example runs and its assertion passes.
- CI matrix (macOS/Linux/Windows + sanitizers + format check) green.
- ADRs 0001–0003, README, CONTRIBUTING, THIRD_PARTY_LICENSES written.
- No third-party product names anywhere in the tree.

## F1 — XML frontend + runtime core

Scope: real ASAM OpenSCENARIO XML (1.0–1.3) parsing into the IR; the full
storyboard hierarchy; the condition and action catalogs; trajectories;
parameter handling.

Gate: representative `.xosc` scenarios load and execute per the
specification; conformance-oriented test suite; frontend diagnostics with
source locations.

## F2 — DSL frontend: parser + type checker

Scope: ASAM OpenSCENARIO DSL (2.x) lexer, parser, and type checker for the
finalized standard libraries; DSL diagnostics.

Gate: the DSL standard library type-checks; representative DSL files parse
with precise diagnostics; no execution yet.

## F3 — DSL concrete-scenario execution

Scope: compile checked DSL concrete scenarios into the shared Scenario IR and
execute them on the same runtime as XML.

Gate: equivalent XML and DSL scenarios produce equivalent runs on the shared
runtime.

## F4 — Constraint solving / abstract scenarios *(feasibility-gated)*

Scope: constraint solving for DSL abstract scenarios (parameter ranges,
distributions, coverage-driven concretization). Starts only after an explicit
feasibility decision.

Gate: defined when the phase is opened.
