<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p7-s4] Expressions, constraints & coverage (check-only)

**Pillar:** P7 — OpenSCENARIO DSL Frontend · **Roadmap:** `docs/roadmap/roadmap.md` § p7-s4

## Goal
DSL §7.4 expressions everywhere they appear; §7.5 constructs checked, not solved.

## Deliverables
- Frontend: expression typing + constant-context evaluation — arithmetic with units and the §7.4.2.3.1 conversion rules, logical short-circuit, relational incl. `in`, ternary, `is()`/`as()`, list and range operators.
- `keep([hard|default])` parsing + type checks + **concrete-value satisfiability only** (a concrete assignment violating a keep is an error; anything requiring search → `UnsupportedFeature` per ADR-0004).
- `sample()` fields; coverage (`cover`/`record`/cross) parsed + checked with an explicit not-collected diagnostic note.

## Tests
- `dsl_expression_test.cpp`.
- `dsl_constraint_test.cpp` (concrete-violation red fixtures, solver-required unsupported-diagnostic fixtures).

## Docs
- Coverage matrix rows updated with spec citations (`docs/roadmap/coverage/`).

## Exit criteria
- [ ] `dsl_expression_test` and `dsl_constraint_test` suites green
- [ ] Unsupported paths diagnose, never crash
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p7-s3
