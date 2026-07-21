<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p2-s5] Trajectory following: polyline, clothoid, NURBS

**Pillar:** P2 — Entities, Motion & Control · **Roadmap:** `docs/roadmap/roadmap.md` § p2-s5

## Goal
The standard's trajectory shapes with numerical fidelity (risk R3).

## Deliverables
- Kernel: trajectory representation in IR (shape + `TimeReference`), arc-length parameterization.
- Kernel: polyline interpolation; clothoid evaluation (Fresnel via `detmath`, documented approximation with error bound); NURBS evaluation (de Boor through `detmath`).
- Kernel: following modes (follow/position) and timing (none/timing) per XML §6.9.1–6.9.5.
- ClothoidSpline and the 1.4 Motion/Interpolation additions are out (see coverage matrix).

## Tests
- `trajectory_test.cpp` (analytic circles/straights as NURBS and clothoid degenerate cases, hex-pinned samples, arc-length round-trip error bounds).
- Determinism fixture with a follower.

## Docs
- Motion chapter §trajectories incl. the coverage decision and numerical method notes.

## Exit criteria
- [ ] Analytic-reference tests within 1e-9 on all platforms
- [ ] Bit-identical traces cross-platform
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p2-s4
