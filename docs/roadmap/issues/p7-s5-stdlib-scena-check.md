<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p7-s5] Standard library checking & scena-check

**Pillar:** P7 — OpenSCENARIO DSL Frontend · **Roadmap:** `docs/roadmap/roadmap.md` § p7-s5

## Goal
The pillar's user-visible capability.

## Deliverables
- Frontend: bundled standard library (`stdtypes` + `std` namespaces authored from the DSL §8/§8.13–8.15 normative text — the spec's library *text* is normative, its files are not — clean-room, cited per section); full-library type-check as a test.
- `tools/scena-check/`: `scena-check <file.osc>` → diagnostics with §-citations, exit codes.
- Bindings: C ABI + Python `check_dsl` entry points (parse + check only).

## Tests
- `dsl_stdlib_test.cpp` (library loads with zero errors).
- `scena-check` CLI tests.
- pytest `test_dsl_check.py`.

## Docs
- DSL user-guide page (checking workflow).

## Exit criteria
- [ ] Standard library checks clean
- [ ] `scena-check` corpus green
- [ ] Bindings updated in the same PR
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p7-s4
