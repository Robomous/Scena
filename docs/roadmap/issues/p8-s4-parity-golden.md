<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p8-s4] XML↔DSL parity & DSL golden scenarios

**Pillar:** P8 — OpenSCENARIO DSL Execution · **Roadmap:** `docs/roadmap/roadmap.md` § p8-s4

## Goal
Prove two-frontends-one-runtime, literally.

## Deliverables
- GS-12 (DSL twin of GS-2) and GS-13 fixtures + reference traces.
- Parity harness (`scripts/golden.py compare-pair`) asserting bit-identical traces for declared pairs.
- DSL golden CI job.
- User-guide "one scenario, two languages" page.

## Tests
- Golden CI extended (GS-12 pair equality, GS-13 alternatives).
- `dsl_parity_test.cpp` for IR-level equivalence.

## Docs
- User-guide "one scenario, two languages" page.

## Exit criteria
- [ ] GS-12 bit-identical pair green on all platforms
- [ ] GS-13 green
- [ ] Pillar exit review done
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p8-s3, p6-s4
