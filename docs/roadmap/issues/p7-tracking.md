<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [P7] OpenSCENARIO DSL Frontend — tracking

Tracking issue for pillar P7. See `docs/roadmap/roadmap.md`.

## Objective
A grammar-based parser (ANTLR4 C++ runtime, BSD), AST, symbol resolution and type system for ASAM OpenSCENARIO DSL 2.x, able to load and check the finalized standard library and concrete scenario files with actionable diagnostics — shipping `scena-check` for DSL files (DSL §7; coverage: `docs/roadmap/coverage/osc-dsl-coverage.md`). Constraint (`keep`) and coverage constructs are parsed and checked but not solved. Out of scope: external-method binding to host code (diagnosed as unsupported when invoked), constraint solving (F4), any execution (P8).

## Sprints
- [ ] p7-s1 — DSL grammar & lexer (ANTLR4) (#)
- [ ] p7-s2 — Parser & AST (#)
- [ ] p7-s3 — Symbols & type system (#)
- [ ] p7-s4 — Expressions, constraints & coverage (check-only) (#)
- [ ] p7-s5 — Standard library checking & scena-check (#)

## Pillar exit criteria
- [ ] The DSL standard library type-checks with zero errors
- [ ] Spec-annex-derived example files parse and check with precise diagnostics (section-number citations — the DSL spec defines no rule IDs)
- [ ] `scena-check` exits nonzero with actionable messages on seeded error corpora
