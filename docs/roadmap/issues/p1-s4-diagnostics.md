<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p1-s4] Structured diagnostics & error model

**Pillar:** P1 — Runtime Core & Determinism · **Roadmap:** `docs/roadmap/roadmap.md` § p1-s4

## Goal
One diagnostics system for frontends and runtime; no exceptions across any boundary.

## Deliverables
- Kernel: `scena::Diagnostic` {severity, code, message, source location (file/line/xpath-ish), optional `asam.net:` rule ID}; `DiagnosticSink` collected per load/run.
- Kernel: `Status` extended (ParseError, ValidationError, SemanticError, UnsupportedFeature).
- Kernel: engine emits runtime diagnostics (e.g. action on missing entity) instead of failing silently.
- Bindings: diagnostics readable through the C ABI (stable struct + iteration) and Python.

## Tests
- `diagnostics_test.cpp`.
- `capi_test` extension (ABI diagnostic round-trip).
- pytest `test_diagnostics.py`.

## Docs
- User-guide error-handling chapter.
- capi-and-bindings notes.

## Exit criteria
- [ ] All fallible paths return `Status` + diagnostics
- [ ] ABI check green
- [ ] Sanitizer job green
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p1-s3
