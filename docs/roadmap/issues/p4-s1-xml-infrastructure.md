<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p4-s1] XML infrastructure: parsing, versions, diagnostics

**Pillar:** P4 — OpenSCENARIO XML Frontend · **Roadmap:** `docs/roadmap/roadmap.md` § p4-s1

## Goal
Deterministic, locale-safe document layer.

## Deliverables
- Frontend: approved XML parser dependency pinned (`cmake/Dependencies.cmake` + `THIRD_PARTY_LICENSES.md` in the same commit).
- Frontend: binary-mode CRLF-safe reading; `std::from_chars`-only numeric conversion helpers (cross-platform rule).
- Frontend: `FileHeader` revMajor/revMinor detection — 1.0–1.3 accepted; 1.4 rejected with a diagnostic; unknown → error.
- Frontend: xpath-ish source locations on every diagnostic.
- Frontend: loader API (`xml::load(path|string) → {Scenario IR, diagnostics}`).

## Tests
- `xml_infra_test.cpp` (encodings, CRLF, the comma-decimal locale trap as a dedicated test, version matrix).

## Docs
- User-guide "loading scenarios" page.

## Exit criteria
- [ ] Infra suite green incl. locale case on all platforms
- [ ] Dependency approved and pinned
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p1-s1 (IR), p1-s4 (diagnostics). Prerequisite maintainer action: XML parser dependency license approval before this sprint (open question OQ-1).
