<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p4-s4] Catalogs, entity selections & controllers

**Pillar:** P4 — OpenSCENARIO XML Frontend · **Roadmap:** `docs/roadmap/roadmap.md` § p4-s4

## Goal
Cross-file reuse and controller assignment.

## Deliverables
- Frontend: `CatalogLocations` directory scanning with deterministic file ordering; catalog loading and reference resolution with `ParameterAssignments` for the eight 1.0–1.3 catalog types (vehicle, controller, pedestrian, miscObject, environment, maneuver, trajectory, route — §9.4–9.6).
- Frontend: `EntitySelection` (explicit members and by-type, §7.2.2.2–7.2.2.5, homogeneity rules cited).
- Frontend: `ObjectController` assignment lowering to control-ownership + controller metadata (engine-controlled default vs host-declared, mapping documented per ADR-0003 and §6.6).

## Tests
- `xml_catalog_test.cpp` (multi-file fixtures, missing-ref diagnostics with rule IDs, parameterized catalog entries).
- `xml_selection_test.cpp`.

## Docs
- Catalogs page.
- Controller-mapping note in gateway docs.

## Exit criteria
- [ ] Catalog fixture tree loads reproducibly (filesystem enumeration order does not affect the IR)
- [ ] Suites green
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p4-s3
