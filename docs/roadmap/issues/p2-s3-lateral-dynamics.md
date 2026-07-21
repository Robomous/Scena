<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p2-s3] Lateral dynamics & lane-change shapes

**Pillar:** P2 — Entities, Motion & Control · **Roadmap:** `docs/roadmap/roadmap.md` § p2-s3

## Goal
Lateral motion machinery for lane change / lane offset / lateral distance.

## Deliverables
- Kernel: lateral offset controller and lane-change trajectory generation over the same transition-dynamics shapes (time/distance dimensions).
- Kernel: target-lane resolution against the p3-s1 interface.
- Kernel: heading blending rules documented.
- Documented simplification (kinematic lateral model, no tire dynamics).

## Tests
- `lateral_test.cpp` (shape sweeps, lateral overshoot bounds, combined longitudinal+lateral stepping determinism).

## Docs
- User-guide motion chapter §lateral.

## Exit criteria
- [ ] Named suites green
- [ ] Hex-pinned lateral trace fixture green cross-platform
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p2-s2, p3-s1 (interface for target-lane resolution)
