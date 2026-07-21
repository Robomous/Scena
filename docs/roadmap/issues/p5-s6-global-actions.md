<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p5-s6] Global & infrastructure actions

**Pillar:** P5 — Actions & Conditions Semantics · **Roadmap:** `docs/roadmap/roadmap.md` § p5-s6

## Goal
Complete the declared catalog.

## Deliverables
- Kernel: VariableSetAction/VariableModifyAction (add/multiply rules, numeric-only per rule citation); deprecated ParameterSetAction/ParameterModifyAction lowered onto the same runtime store for 1.0/1.1 files.
- Kernel: AddEntityAction/DeleteEntityAction with deterministic entity-table updates.
- Kernel: EnvironmentAction (environment state store incl. TimeOfDay clock feeding TimeOfDayCondition; no physics coupling, documented).
- Kernel: TrafficSignalStateAction + TrafficSignalControllerAction (phase model per XML §6.11) with TrafficSignalCondition + TrafficSignalControllerCondition.
- Kernel: CustomCommandAction as a gateway callback (no-op without a host).
- Frontend: lowering for each.

## Tests
- `action_global_test.cpp`.
- `traffic_signal_test.cpp` (phase timing determinism).
- Entity add/delete determinism fixture.

## Docs
- Coverage matrix rows updated with spec citations (`docs/roadmap/coverage/`).

## Exit criteria
- [ ] Every In-v0.0.1 matrix row implemented + tested
- [ ] Matrix cross-checked by the docs consistency grep in CI
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p5-s5
