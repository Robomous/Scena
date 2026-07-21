<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p2-s2] Longitudinal dynamics & default controller

**Pillar:** P2 — Entities, Motion & Control · **Roadmap:** `docs/roadmap/roadmap.md` § p2-s2

## Goal
Speed control with standard transition dynamics, clamped by performance limits.

## Deliverables
- Kernel: transition-dynamics evaluator — shapes linear, cubic, sinusoidal, step; dimensions time, distance, rate per the XML TransitionDynamics model, §7.4.1.2.
- Kernel: longitudinal integrator through `detmath`.
- Kernel: default controller applying target-speed setpoints.
- Kernel: speed profile follower (entry series with optional acceleration/jerk limits).
- Documented simplification note (point-mass longitudinal model).

## Tests
- `longitudinal_test.cpp` (each shape × dimension against closed-form references, performance clamping, zero-duration edge cases).

## Docs
- User-guide motion chapter §longitudinal, simplifications table.

## Exit criteria
- [ ] Shape/dimension matrix green on all platforms
- [ ] Determinism suite extended with a dynamics-heavy fixture
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p2-s1
