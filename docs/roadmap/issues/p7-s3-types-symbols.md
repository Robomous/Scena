<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p7-s3] Symbols & type system

**Pillar:** P7 — OpenSCENARIO DSL Frontend · **Roadmap:** `docs/roadmap/roadmap.md` § p7-s3

## Goal
Name resolution and the DSL §7.3 type rules.

## Deliverables
- Frontend: symbol tables (deterministic ordered storage); namespace/scoping and `::` resolution (§7.7.4); declare-anywhere resolution order (§7.3.15).
- Single inheritance + conditional inheritance (`is()`/`as()` latent subtypes, §7.3.8); extension composition (§7.3.9).
- Physical types with SI-exponent dimension checking and unit conversion factors/offsets (§7.3.4); enum (`enum!member`) typing.
- Actor/scenario/action signature checking; modifier applicability (§7.3.12); `it` binding rules.
- Diagnostics with §-citations.

## Tests
- `dsl_types_test.cpp` (unit-conversion matrix, extension and conditional-inheritance cases, negative fixtures per rule).

## Docs
- Coverage matrix rows updated with spec citations (`docs/roadmap/coverage/`).

## Exit criteria
- [ ] Type-rule matrix green
- [ ] A standard-library subset loads
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p7-s2
