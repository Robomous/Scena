<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p8-s2] Composition operators: serial, parallel, one_of

**Pillar:** P8 — OpenSCENARIO DSL Execution · **Roadmap:** `docs/roadmap/roadmap.md` § p8-s2

## Goal
DSL §7.6.2 invocation semantics on the runtime's storyboard machinery.

## Deliverables
- Kernel: composition nodes mapping onto the P1 execution model (designed for in p1-s1's lowerability review) — serial segmentation (member starts when predecessor ends, §7.6.2.1.2); parallel with default `start` overlap and all-members-complete join (§7.6.2.1.4); one_of deterministic selection input (engine API + `--select`, default first alternative, §7.6.2.1.3).
- `duration` bounds; `until` (terminates the annotated invocation exactly at the event, §7.6.2.5.4).
- `wait`/`emit`/`on` plumbed onto the trigger system (§7.6.2.5; emit is zero-time, wait is pure synchronization).
- Predefined `start`/`end`/`fail` events (§7.3.10.3) mapped to storyboard element transitions.

## Tests
- `dsl_composition_test.cpp` (phase-boundary traces per operator, nested compositions, event-gated phases).
- Determinism fixture per alternative.

## Docs
- Coverage matrix rows updated with spec citations (`docs/roadmap/coverage/`).

## Exit criteria
- [ ] Operator suite green
- [ ] GS-13's body runs via the API
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p8-s1
