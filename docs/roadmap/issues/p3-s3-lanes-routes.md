<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p3-s3] Lanes, lane sections & routes

**Pillar:** P3 — Road Interface · **Roadmap:** `docs/roadmap/roadmap.md` § p3-s3

## Goal
Lane model + routing over the road graph.

## Deliverables
- Lane sections, ids, polynomial width records, lane linkage across sections/roads, junction connections.
- `IRoadQuery` implementation completed over this model incl. lane queries and relative-lane arithmetic.
- Deterministic shortest-path routing (ordered adjacency, documented tie-break) producing the p3-s1 route representation.

## Tests
- The backend passes `road_query_contract_test` fully.
- `opendrive_lanes_test.cpp`.
- `route_test.cpp` (junction map fixture, tie-break determinism).

## Docs
- Roads page extended (lane id conventions: negative right, positive left, 0 = reference line, per OpenDRIVE).

## Exit criteria
- [ ] Contract suite green against the real backend
- [ ] Route fixtures green
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p3-s2
