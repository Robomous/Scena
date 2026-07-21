# ADR-0004: Road to v0.0.1 — single-release roadmap

- **Status:** accepted
- **Date:** 2026-07-21

## Context

F0 (scaffold + kernel skeleton) is complete: the repository builds on
macOS/Linux/Windows with warnings-as-errors, the deterministic step engine
executes a simplified flat storyboard, and the C ABI and Python bindings
prove the embedding shape end to end. What follows F0 is the actual product:
a runtime faithful to the ASAM OpenSCENARIO storyboard semantics, two
frontends (XML 1.0–1.3 and DSL 2.x concrete scenarios) compiling to one
Scenario IR, a road interface with an ASAM OpenDRIVE reference backend, and
a stable embedding surface.

Two release strategies were considered:

1. **Incremental releases** (v0.0.1 after the XML frontend, v0.0.2 after the
   DSL, …), shipping partial standard coverage early.
2. **A single release** (v0.0.1) published only when both frontends execute
   on the shared runtime and a formal gate passes.

## Decision

### Single release: v0.0.1

There is exactly **one release on this roadmap: v0.0.1**. It is published
only when all eight pillars of `docs/roadmap/roadmap.md` are complete and
the release gate defined there passes. No intermediate releases, no tags, no
version bumps before that; development builds are unversioned. Scena's value
proposition is standards correctness and determinism — a first public
release with a partially implemented storyboard or a frontend that silently
drops constructs would undermine exactly the property the project exists to
provide.

### Pillar → sprint → PR → gate

Work between F0 and v0.0.1 is organized as eight pillars (P1–P8), each
decomposed into sprints. Every sprint closes via a single focused PR merged
to `main` with CI green. Every behavioral promise in the roadmap maps to an
automated test or an explicit maintainer-executed check; a promise without a
verification mechanism does not enter the roadmap. The pillar structure
refines the F-phase plan of ADR-0001: P1–P6 realize F1 (plus the embedding
surface), P7 realizes F2, P8 realizes F3. The F-gates remain the
coarse-grained sequence; the pillars are the executable plan.

### Maintainer-exclusive publication

The roadmap tooling produces documents and reviewable scripts only. Git
tags, GitHub releases, milestones, and issues are created exclusively by the
maintainer, who also hand-executes the golden scenarios on all three
platforms and gives explicit written approval before tagging v0.0.1 and
claiming the `scena` package name on PyPI.

### Out of scope for v0.0.1

The following are explicitly **post-v0.0.1** (feasibility-gated where
noted). Each exclusion keeps the declared coverage honestly and completely
implementable:

- **DSL constraint solving and abstract/logical scenario concretization**
  (F4, feasibility-gated). v0.0.1 executes concrete DSL scenarios only;
  `keep()` constraints beyond concrete value assignment, ranges,
  distributions, and coverage-driven generation are parsed and type-checked
  but not solved.
- **OpenSCENARIO XML elements outside the declared coverage matrix**
  (`docs/roadmap/coverage/osc-xml-coverage.md`) — notably traffic swarms and
  stochastic distributions, environment/weather visuals, and elements
  introduced in XML 1.4.
- **A visualization or player UI.** `scena-run` emits state traces;
  rendering belongs to host tools.
- **Concrete simulator adapters.** The gateway interfaces are the product;
  adapters live outside this repository.
- **OpenSCENARIO XML 1.4 as a target.** The local reference copy is 1.4.0;
  the supported input range remains 1.0–1.3.

## Consequences

- The first tag carries real meaning: both frontends, one runtime, declared
  coverage fully tested. Marketing and packaging effort happens once.
- The roadmap must burn down risk early (runtime semantics, determinism,
  road backend) because nothing ships until the end; the pillar ordering in
  `roadmap.md` exists precisely to front-load those risks.
- Coverage matrices become normative project documents: a feature is either
  in the matrix with tests, or it is out of scope. Frontends must emit
  structured warnings for out-of-coverage constructs rather than silently
  dropping them.
- The single-release model concentrates the release gate: a 24-hour
  sanitizer soak, hand-executed golden scenarios on three platforms, and
  synchronized documentation are all blocking, which is acceptable because
  they run exactly once.
