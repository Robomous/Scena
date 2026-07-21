<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [P4] OpenSCENARIO XML Frontend — tracking

Tracking issue for pillar P4. See `docs/roadmap/roadmap.md`.

## Objective
A full loader for OpenSCENARIO XML 1.0–1.3 within the declared coverage matrix: document reading, storyboard/entities lowering to IR, parameters and expressions, catalogs, variables, init actions, entity selections, controller assignment, file-level validation with structured warnings, and documented version migration (XML §6–§9; coverage: `docs/roadmap/coverage/osc-xml-coverage.md`).

## Sprints
- [ ] p4-s1 — XML infrastructure: parsing, versions, diagnostics (#)
- [ ] p4-s2 — Entities, storyboard & init loading (#)
- [ ] p4-s3 — Parameters, expressions & variables (#)
- [ ] p4-s4 — Catalogs, entity selections & controllers (#)
- [ ] p4-s5 — 1.0–1.3 migration & file-level validation (#)

## Pillar exit criteria
- [ ] All P4 sprints merged
- [ ] Conformance fixture corpus (spec-derived, self-authored) loads with expected IR snapshots
- [ ] Diagnostics cite `asam.net:xosc` rule IDs where defined
- [ ] Version matrix tests green
