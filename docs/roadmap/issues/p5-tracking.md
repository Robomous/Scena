<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [P5] Actions & Conditions Semantics — tracking

Tracking issue for pillar P5. See `docs/roadmap/roadmap.md`.

## Objective
Implement the declared action and condition catalog against the coverage matrix (`docs/roadmap/coverage/osc-xml-coverage.md`, normative) — every implemented item cross-referenced to its spec section (XML §7.4 actions, §7.5 conflicts, §7.6 conditions) and its tests. Out of scope (post-v0.0.1, reasons in the matrix): SynchronizeAction, OverrideControllerValueAction, traffic source/sink/swarm/area/stop, appearance actions, trailers, monitors, RandomRouteAction, Angle/RelativeAngle conditions, all 1.4-only items.

## Sprints
- [ ] p5-s1 — ByValue conditions (#)
- [ ] p5-s2 — ByEntity conditions I: kinematics & position (#)
- [ ] p5-s3 — ByEntity conditions II: interaction metrics (#)
- [ ] p5-s4 — Private actions I: longitudinal, lateral, teleport (#)
- [ ] p5-s5 — Private actions II: routing, distance keeping & controllers (#)
- [ ] p5-s6 — Global & infrastructure actions (#)

## Pillar exit criteria
- [ ] Every matrix row marked In-v0.0.1 has a runtime implementation, a named test, a loading path from XML, and a spec-section citation in code
- [ ] Per-family determinism fixtures green
