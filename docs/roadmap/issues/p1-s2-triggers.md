<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p1-s2] Triggers, condition groups & edge semantics

**Pillar:** P1 — Runtime Core & Determinism · **Roadmap:** `docs/roadmap/roadmap.md` § p1-s2

## Goal
Standard-faithful trigger evaluation.

## Deliverables
- Kernel: `ir::Trigger` — OR of ConditionGroups; group = AND of Conditions per XML §7.6.1; empty trigger always false.
- Kernel: condition edges none/rising/falling/risingOrFalling with the §7.6.4 corner cases (first-ever evaluation of an edge condition is false; evaluation begins on entering standby).
- Kernel: delay semantics per §7.6.3 — delayed condition evaluates the expression state of t−Δt; false while t < Δt; Δt ≥ 0 enforced with rule `asam.net:xosc:1.0.0:data_type.condition_delay_not_negative`.
- Kernel: start triggers only on Act and Event; stop triggers only on Storyboard and Act, with descendant inheritance and execution-count clearing (§7.6.1.1–7.6.1.2).

## Tests
- `trigger_test.cpp` (edge matrix × delay × group combinations, exact-step firing, empty-trigger and first-evaluation corner cases).
- Scheduler tests for delay ordering ties (deterministic tie-break documented).

## Docs
- User-guide trigger semantics section with spec citations.

## Exit criteria
- [ ] Trigger test matrix green
- [ ] Delayed-condition determinism case in `determinism_test` green
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p1-s1
