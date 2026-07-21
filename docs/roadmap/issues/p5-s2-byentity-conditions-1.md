<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p5-s2] ByEntity conditions I: kinematics & position

**Pillar:** P5 — Actions & Conditions Semantics · **Roadmap:** `docs/roadmap/roadmap.md` § p5-s2

## Goal
Entity-observing conditions with correct triggering-entity semantics.

## Deliverables
- Kernel: `TriggeringEntities` any/all evaluation frame (XML §7.6.5.1).
- Kernel: Speed (optional direction), RelativeSpeed, Acceleration, StandStill (duration accumulation), TraveledDistance, and ReachPosition (deprecated; position + tolerance via `PositionResolver`) conditions.
- Kernel: road-coordinate vs cartesian measurement plumbing from p3-s4.

## Tests
- `condition_byentity_kinematics_test.cpp` (incl. observation of host-controlled entities).

## Docs
- Coverage matrix rows updated with spec citations (`docs/roadmap/coverage/`).

## Exit criteria
- [ ] `condition_byentity_kinematics_test` suite green
- [ ] TriggeringEntities any/all matrix covered
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p5-s1, p3-s4
