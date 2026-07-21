<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p2-s4] Position resolution, teleport & host round-trip

**Pillar:** P2 — Entities, Motion & Control · **Roadmap:** `docs/roadmap/roadmap.md` § p2-s4

## Goal
One resolver for all position types; airtight control-ownership semantics.

## Deliverables
- Kernel: `PositionResolver` mapping the ten IR position variants (world, relative-world, relative-object, road, relative-road, lane, relative-lane, route, trajectory, geographic per XML §6.3.8) to world poses — road-family resolution via `IRoadQuery` (p3-s1), others self-contained.
- Kernel: the corrected ≥1.3 position/orientation calculations applied uniformly to all input versions (per XML §5, which prescribes exactly that); orientation reference handling (absolute/relative).
- Kernel: `TeleportAction` runtime.
- Kernel: host-controlled entities — `report_state` round-trip formalized (engine never integrates them; conditions observe reported states; publish/poll order per ADR-0003 re-verified).
- GeoPosition resolves only when a geodetic datum is available, else a rule-cited diagnostic (`asam.net:xosc:1.1.0:positioning.geodetic`).

## Tests
- `position_test.cpp` (each variant, orientation composition).
- `control_ownership_test.cpp` (round-trip, mode violations).

## Docs
- User-guide positions page.

## Exit criteria
- [ ] All position variants resolve or return `UnsupportedFeature` diagnostics (none silently wrong)
- [ ] Suites green
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p2-s3, p3-s1 (interface for road-relative positions; world/relative positions do not wait on P3)
