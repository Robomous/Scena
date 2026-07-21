<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p3-s4] Road-based positions in actions & conditions

**Pillar:** P3 — Road Interface · **Roadmap:** `docs/roadmap/roadmap.md` § p3-s4

## Goal
Wire roads into the execution path.

## Deliverables
- Kernel: `PositionResolver` road/lane/route variants fully live against the backend.
- Kernel: lane-change target resolution (p2-s3) uses lane queries.
- Kernel: the distance-measurement plumbing P5 needs — longitudinal/lateral distance in road coordinates vs cartesian, per the XML §6.4 distance definitions and the `coordinateSystem`/`relativeDistanceType` attributes, with the deprecated `alongRoute` mapped onto them.

## Tests
- `road_integration_test.cpp` (fixtures on the hand-authored maps driving road-relative teleports and lane changes).
- Determinism fixture on the curve map.

## Docs
- Positions page updated with road-backed examples.

## Exit criteria
- [ ] Integration fixtures green
- [ ] GS-8's map prerequisites committed
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p3-s3
