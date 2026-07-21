<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p5-s5] Private actions II: routing, distance keeping & controllers

**Pillar:** P5 — Actions & Conditions Semantics · **Roadmap:** `docs/roadmap/roadmap.md` § p5-s5

## Goal
Path-level control and the controller/visibility surface.

## Deliverables
- Kernel: AssignRouteAction (XML §6.8.2); AcquirePositionAction (implicit route); FollowTrajectoryAction (p2-s5 machinery; timeReference × followingMode matrix per §6.9).
- Kernel: LongitudinalDistanceAction (distance/timeGap modes, freespace, constraints from performance limits, `continuous` keeping).
- Kernel: ActivateControllerAction (per-domain toggling of the engine default controller; deprecated direct placement accepted); AssignControllerAction (controller metadata handed to the host via gateway); VisibilityAction (graphics/sensors/traffic flags surfaced through state/gateway).
- Frontend: lowering for each.

## Tests
- `action_routing_test.cpp`.
- `action_distance_test.cpp` (convergence bounds, freespace hold).
- Controller/visibility round-trip tests.

## Docs
- Coverage matrix rows updated with spec citations (`docs/roadmap/coverage/`).

## Exit criteria
- [ ] `action_routing_test` and `action_distance_test` suites green
- [ ] GS-4/GS-7/GS-8 scenario bodies run via the C++ API
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p5-s4, p2-s5
