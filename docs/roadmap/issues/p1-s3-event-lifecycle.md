<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p1-s3] Event lifecycle, priority & overwrite rules

**Pillar:** P1 — Runtime Core & Determinism · **Roadmap:** `docs/roadmap/roadmap.md` § p1-s3

## Goal
Correct concurrent-event behavior inside a maneuver.

## Deliverables
- Kernel: event priority per XML §7.3.2/§8.4.2.2 — literals `override` (stops all running events in the same Maneuver), `skip` (event never leaves standby; the skipTransition counts as an execution), `parallel`.
- Kernel: deprecated literal `overwrite` accepted at load time as a synonym for `override` (pre-1.3 spelling; the coverage matrix records the compatibility decision).
- Kernel: `maximumExecutionCount` sequential re-execution per §8.3.3.2/§8.4.2.1 (executions = startTransitions + skipTransitions; stop trigger cancels remaining counts).
- Kernel: action-conflict resolution per §7.5 (conflicting control strategy on the same domain/entity stops the older action; bulk actions over resolved ManeuverGroup actors per §7.5.4/§8.3.3.3: complete when all children complete, conflict on one entity overrides for all).

## Tests
- `event_priority_test.cpp` (override kills running action same step; skip counts executions; parallel coexists; rerun counting; bulk actor conflict propagation).
- Storyboard restart cases.

## Docs
- User-guide event semantics.
- Coverage matrix rows flip to implemented.

## Exit criteria
- [ ] Priority matrix tests green on all platforms
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p1-s2
