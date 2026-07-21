<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p5-s1] ByValue conditions

**Pillar:** P5 — Actions & Conditions Semantics · **Roadmap:** `docs/roadmap/roadmap.md` § p5-s1

## Goal
The by-value condition set (signal conditions follow with their actions in p5-s6).

## Deliverables
- Kernel: SimulationTime condition (time starts with the storyboard running, XML §8.4.7); Parameter, Variable, UserDefinedValue (host-provided named values), and TimeOfDay conditions.
- Kernel: StoryboardElementState condition — states **and** transitions observed, wired to the p1-s1 state machine (§7.6.5.2).
- Kernel: the Rule comparator (equal/greater/less family) as one shared component with documented numeric/string semantics.
- Frontend: XML lowering for each condition.

## Tests
- `condition_byvalue_test.cpp` (per-condition semantics × edges × delays).
- Storyboard-state observation fixtures.

## Docs
- Coverage matrix rows updated with spec citations (`docs/roadmap/coverage/`).

## Exit criteria
- [ ] `condition_byvalue_test` suite green
- [ ] Coverage matrix rows flipped with spec citations
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p1-s3, p4-s2
