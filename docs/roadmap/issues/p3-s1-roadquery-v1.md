<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p3-s1] IRoadQuery v1: the frozen road interface

**Pillar:** P3 — Road Interface · **Roadmap:** `docs/roadmap/roadmap.md` § p3-s1

## Goal
Freeze the interface P2/P5 code against (unblocks parallel work; contains risk R4).

## Deliverables
- Kernel: extended `gateway::IRoadQuery` — lane existence/width/type queries, s-range queries, lane-relative ↔ world conversions, relative-lane arithmetic (lane ±n at s), route interface (waypoints → ordered road/lane spans, position-along-route).
- Kernel: documented unsupported-reporting semantics.
- Kernel: a `FlatWorldRoadQuery` null object for road-free scenarios.

## Tests
- `road_query_contract_test.cpp` — an executable contract suite any backend must pass (runs against `FlatWorldRoadQuery` now, the OpenDRIVE backend later).

## Docs
- ADR-0003 amendment note (interface v1).
- User-guide road page.

## Exit criteria
- [ ] Interface merged with contract suite green
- [ ] Downstream sprints reference only this header
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** none in the graph (P3 entry point; unblocks p2-s3/p2-s4 and P5 road-based work)
