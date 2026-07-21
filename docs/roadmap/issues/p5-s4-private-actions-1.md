<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p5-s4] Private actions I: longitudinal, lateral, teleport

**Pillar:** P5 — Actions & Conditions Semantics · **Roadmap:** `docs/roadmap/roadmap.md` § p5-s4

## Goal
The core motion actions end-to-end (XML → IR → runtime → trace).

## Deliverables
- Kernel: SpeedAction (absolute/relative targets; `continuous=true` relative tracking never ends per XML §7.5.3; step shape = instantaneous; replaces the F0 placeholder); SpeedProfileAction.
- Kernel: LaneChangeAction (absolute/relative targets, offset carryover); LaneOffsetAction (continuous variant); LateralDistanceAction.
- Kernel: TeleportAction over all position types.
- Frontend: lowering for each; event-priority interaction with running motion actions verified (conflict rules §7.5).

## Tests
- `action_longitudinal_test.cpp`, `action_lateral_test.cpp`.
- Overwrite-during-transition fixtures.
- GS-2's core sequence as an engine test.

## Docs
- Coverage matrix rows updated with spec citations (`docs/roadmap/coverage/`).

## Exit criteria
- [ ] `action_longitudinal_test` and `action_lateral_test` suites green
- [ ] GS-1/GS-2 scenario bodies run via the C++ API with stable traces
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p5-s3, p2-s2
