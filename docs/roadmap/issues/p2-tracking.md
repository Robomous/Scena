<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [P2] Entities, Motion & Control — tracking

Tracking issue for pillar P2. See `docs/roadmap/roadmap.md`.

## Objective
The standard's entity taxonomy and the motion machinery scenario actions steer: longitudinal/lateral dynamics with documented simplifications, default engine controllers, host-controlled round-trip, position resolution, and trajectory following (XML §6 concepts + §7.2.2 entities, trajectory model §6.9).

## Sprints
- [ ] p2-s1 — Entity taxonomy, bounding boxes & performance (#)
- [ ] p2-s2 — Longitudinal dynamics & default controller (#)
- [ ] p2-s3 — Lateral dynamics & lane-change shapes (#)
- [ ] p2-s4 — Position resolution, teleport & host round-trip (#)
- [ ] p2-s5 — Trajectory following: polyline, clothoid, NURBS (#)

## Pillar exit criteria
- [ ] All P2 sprints merged
- [ ] Motion suites green including hex-pinned dynamics tests through `detmath`
- [ ] GS-1 executable end-to-end through the C++ API (pre-CLI)
