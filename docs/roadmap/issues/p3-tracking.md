<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [P3] Road Interface — tracking

Tracking issue for pillar P3. See `docs/roadmap/roadmap.md`.

## Objective
Finalize `IRoadQuery` (lane-relative positioning, s/t coordinates, lane queries, road/lane ↔ world conversions, routes) and provide a reference backend implementing it over ASAM OpenDRIVE input, so road/lane positions work in actions and conditions (ADR-0003 boundary; OpenDRIVE s/t conventions).

## Sprints
- [ ] p3-s1 — IRoadQuery v1: the frozen road interface (#)
- [ ] p3-s2 — OpenDRIVE backend: reference-line geometry (#)
- [ ] p3-s3 — Lanes, lane sections & routes (#)
- [ ] p3-s4 — Road-based positions in actions & conditions (#)

## Pillar exit criteria
- [ ] All P3 sprints merged
- [ ] Conversion round-trip and route tests green
- [ ] Hand-authored map fixtures (straight, curve, junction) committed
- [ ] The runtime consumes roads only through `IRoadQuery`
