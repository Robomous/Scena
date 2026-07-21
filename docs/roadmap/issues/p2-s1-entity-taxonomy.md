<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p2-s1] Entity taxonomy, bounding boxes & performance

**Pillar:** P2 — Entities, Motion & Control · **Roadmap:** `docs/roadmap/roadmap.md` § p2-s1

## Goal
Real entity model in the IR.

## Deliverables
- Kernel: `ir::Vehicle`/`ir::Pedestrian`/`ir::MiscObject` — categories, bounding box center+dimensions, `Performance` limits (maxSpeed/maxAcceleration/maxDeceleration), axles as data, ordered properties map.
- Kernel: entity table keyed by stable order (`std::map` stays).
- Kernel: `EntityState` extended to a full pose (pitch/roll) with documented conventions (Z-up, radians, per the standards skill).
- Bindings: entity metadata queries (C ABI + Python) updated in the same PR.

## Tests
- `entity_model_test.cpp`.
- `capi_test`/pytest metadata round-trip.

## Docs
- User-guide entity model page.

## Exit criteria
- [ ] Suites green
- [ ] ABI/bindings/example updated in the same PR (per CONTRIBUTING rule)
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p1-s1
