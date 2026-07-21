<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p1-s1] Full storyboard IR & element state machines

**Pillar:** P1 — Runtime Core & Determinism · **Roadmap:** `docs/roadmap/roadmap.md` § p1-s1

## Goal
IR and runtime for the real storyboard hierarchy with the standard's element state machine.

## Deliverables
- Kernel: `ir/storyboard.h` — Storyboard → Story → Act → ManeuverGroup (actors) → Maneuver → Event → Action nesting per XML §7.2.1/§7.3.
- Kernel: element state machine per §8.1–8.2 (standby, running, complete; start/end/stop/skip transitions observable for storyboard-state conditions).
- Kernel: child start rule per §8.3 (children with a start trigger enter standby, others go straight to running).
- Kernel: init-action phase per §8.5 (runs before simulation time starts; non-instantaneous init actions carry into the storyboard).
- Kernel: storyboard completion only via stop trigger (§8.4.7); engine walks the hierarchy each step.
- Kernel: explicit IR lowerability review against DSL composition concepts (serial/parallel/one_of, DSL §7.3.13 — risk R5).
- Bindings: minimal builder surface in C ABI/Python kept compiling (full expansion lands in P6).

## Tests
- `storyboard_test.cpp` (state transitions, hierarchy walk order, completeness propagation child→parent, init-phase ordering).
- Determinism suite extended to hierarchical storyboards.

## Docs
- `docs/user-guide/` storyboard model section.
- `engine.h` step order comment updated (ADR-level contract unchanged).

## Exit criteria
- [ ] Named tests green on all platforms
- [ ] F0 flat-storyboard API removed
- [ ] Determinism suite green
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** F0 (done)
