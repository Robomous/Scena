<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p4-s2] Entities, storyboard & init loading

**Pillar:** P4 — OpenSCENARIO XML Frontend · **Roadmap:** `docs/roadmap/roadmap.md` § p4-s2

## Goal
The structural spine: `.xosc` → IR.

## Deliverables
- Frontend: `ScenarioDefinition` skeleton; `Entities` (inline Vehicle/Pedestrian/MiscObject → p2-s1 IR).
- Frontend: `Storyboard` hierarchy incl. triggers/edges/delays (→ p1-s1/s2 IR); `Init` actions (global + private, §8.5 ordering); stop trigger.
- Frontend: action/condition payload lowering for everything P5 has landed so far; structured `UnsupportedFeature` warnings for the rest (matrix-driven).
- Frontend: `RoadNetwork` element (logic file reference handed to the host/road backend).

## Tests
- `xml_storyboard_test.cpp` with IR-snapshot fixtures (self-authored from spec examples).
- Round-trip invariant: load → IR dump → stable snapshot.

## Docs
- Coverage matrix status flips.

## Exit criteria
- [ ] Snapshot suite green
- [ ] The GS-1 file loads and runs through the C++ API
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p4-s1
