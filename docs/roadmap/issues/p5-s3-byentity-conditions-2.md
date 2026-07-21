<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p5-s3] ByEntity conditions II: interaction metrics

**Pillar:** P5 — Actions & Conditions Semantics · **Roadmap:** `docs/roadmap/roadmap.md` § p5-s3

## Goal
The interaction measures scenarios actually trigger on.

## Deliverables
- Kernel: Distance and RelativeDistance conditions (coordinateSystem × relativeDistanceType per XML §6.4; freespace true/false using p2-s1 bounding boxes; deprecated `alongRoute` mapped).
- Kernel: TimeHeadway; TimeToCollision (documented closed-form: distance over closing speed, no acceleration term, diverging ⇒ false).
- Kernel: EndOfRoad; Offroad (road-network-dependent, road-prerequisite rule `asam.net:xosc:1.0.0:...` cited); Collision (OBB intersection); RelativeClearance (adjacent-lane/longitudinal-window freeness).
- Freespace math through `detmath`.

## Tests
- `condition_byentity_interaction_test.cpp` (analytic gap fixtures, freespace vs reference-point, TTC edges: diverging, zero relative speed).

## Docs
- Coverage matrix rows updated with spec citations (`docs/roadmap/coverage/`).

## Exit criteria
- [ ] `condition_byentity_interaction_test` suite green cross-platform (freespace math hex-pinned)
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p5-s2
