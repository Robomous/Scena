<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p4-s5] 1.0–1.3 migration & file-level validation

**Pillar:** P4 — OpenSCENARIO XML Frontend · **Roadmap:** `docs/roadmap/roadmap.md` § p4-s5

## Goal
Honest multi-version support and a strict validation story.

## Deliverables
- Frontend: per-version acceptance rules — 1.3's corrected position semantics applied to all versions per §5; deprecated constructs of the coverage matrix accepted with warnings (deprecated `ActivateControllerAction` placement, `ParameterSetAction`/`ParameterModifyAction`, `ReachPositionCondition`, `alongRoute`, priority `overwrite`).
- Frontend: file-level validation pass (referential integrity: entity refs, storyboard-element refs, catalog refs; cardinalities; unused-declaration warnings) with rule-ID citations.
- Frontend: `scena::xml::validate()` API (load + check without executing).

## Tests
- `xml_version_test.cpp` (the same scenario expressed per 1.0/1.1/1.2/1.3 → equivalent IR or documented divergence).
- `xml_validate_test.cpp` (each validation rule has a red fixture).

## Docs
- `docs/user-guide/xml-versions.md` migration table (the documented version-migration handling).

## Exit criteria
- [ ] Version/validation suites green
- [ ] Migration table published
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p4-s4
