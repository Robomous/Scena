<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [P8] OpenSCENARIO DSL Execution — tracking

Tracking issue for pillar P8. See `docs/roadmap/roadmap.md`.

## Objective
Lower checked concrete DSL scenarios to the shared Scenario IR and execute them on the same runtime as XML, with composition operators (serial/parallel/one_of) and concrete-value movement modifiers, demonstrating execution parity via XML/DSL scenario pairs with matching trajectories (DSL §7.6 semantics, §8.8 actions, §8.9 movement modifiers). The DSL spec defines trace-acceptance rather than operational semantics (§7.6.1) — Scena's deterministic executor is one conformant operational choice, documented per section. Constraint solving and abstract-scenario generation are explicitly out of scope for v0.0.1 (ADR-0004).

## Sprints
- [ ] p8-s1 — Lowering concrete scenarios to the IR (#)
- [ ] p8-s2 — Composition operators: serial, parallel, one_of (#)
- [ ] p8-s3 — Movement modifiers (#)
- [ ] p8-s4 — XML↔DSL parity & DSL golden scenarios (#)

## Pillar exit criteria
- [ ] GS-12/GS-13 green (GS-12 bit-identical to its XML twin)
- [ ] Every DSL-execution matrix row In-v0.0.1 implemented with tests
- [ ] `scena-run` accepts `.osc` concrete scenarios
