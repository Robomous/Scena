<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p8-s3] Movement modifiers

**Pillar:** P8 — OpenSCENARIO DSL Execution · **Roadmap:** `docs/roadmap/roadmap.md` § p8-s3

## Goal
The declared DSL §8.9 modifier set with concrete values.

## Deliverables
- Frontend + kernel: modifier application per §7.3.12.4 (argument = equality constraint) compiled onto the P2/P5 motion machinery.
- In-scope set per matrix: `position`/`keep_position`; `speed`/`change_speed`/`keep_speed`; `acceleration`; `lane`/`change_lane` (side required for determinism — unspecified side is diagnosed, not randomized)/`keep_lane`; `lateral`; `along`/`along_trajectory`; `distance`.
- `at` phase anchoring (start/end/all, §8.9.19).
- No parallel modifier machinery in the runtime — modifiers lower to existing IR actions/constraints wherever expressible, with any kernel addition documented and spec-cited.

## Tests
- `dsl_modifier_test.cpp` (each modifier → expected IR/trace, combination fixtures, conflict diagnostics).

## Docs
- Coverage matrix rows updated with spec citations (`docs/roadmap/coverage/`).

## Exit criteria
- [ ] Modifier matrix green
- [ ] Coverage matrix rows flipped
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p8-s2
