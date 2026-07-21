<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p8-s1] Lowering concrete scenarios to the IR

**Pillar:** P8 — OpenSCENARIO DSL Execution · **Roadmap:** `docs/roadmap/roadmap.md` § p8-s1

## Goal
`.osc` in, Scenario IR out.

## Deliverables
- Frontend (`frontends/dsl/lowering/`): scenario entry point selection (implementation-defined per DSL §7.7.2 — top-level scenario by qualified name, CLI/API parameter).
- Concrete parameter binding (equality/single-value `keep`s resolved at init per §7.3.11; anything else diagnosed).
- Actor → IR entity mapping (vehicle/person physical fields → p2-s1 taxonomy).
- §8.8 movement-action subset (`move`, `drive`, `walk`, assign/change/keep speed and position family, `follow_lane`, `change_lane`, gap/headway keeping, `follow_path`/`follow_trajectory` per matrix) → IR actions.
- `set_map_file` → road backend wiring; diagnostics for non-concrete remnants (§-citations); `scena-run` accepts `.osc`.

## Tests
- `dsl_lowering_test.cpp` (IR snapshots for self-authored spec-derived concrete scenarios; non-concrete red fixtures).

## Docs
- Coverage matrix rows updated with spec citations (`docs/roadmap/coverage/`).

## Exit criteria
- [ ] IR snapshot suite green
- [ ] A trivial DSL scenario runs end-to-end
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p7-s5, p5-s6
