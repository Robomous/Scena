# OpenSCENARIO DSL frontend (reserved)

This directory is reserved for the ASAM OpenSCENARIO DSL (2.x) frontend,
scheduled for a later phase (see `docs/roadmap/roadmap.md`).

Like the XML frontend, the DSL frontend will compile scenarios into the shared
Scenario IR (`core/include/kinema/ir/`); the runtime stays frontend-agnostic.
Planned scope:

- **F2** — parser and type checker for the finalized OpenSCENARIO DSL 2.x
  standard libraries.
- **F3** — concrete-scenario execution over the shared runtime.
- **F4** — constraint solving for abstract scenarios (feasibility-gated).

No code lives here yet.
