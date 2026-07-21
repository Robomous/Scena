<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p6-s4] scena-run CLI & golden harness

**Pillar:** P6 — Embedding: Gateway, C API & Python · **Roadmap:** `docs/roadmap/roadmap.md` § p6-s4

## Goal
The validation vehicle for the release gate.

## Deliverables
- `tools/scena-run/` (thin consumer of the public API only): `scena-run <file> --dt --duration --trace out.{csv,json} --replay <entity>=<csv> --select <alt>`; bit-exact double round-trip formatting; exit codes (0 ok; nonzero on load/validation errors).
- `scripts/golden.py` (run + verify per `docs/roadmap/golden-scenarios.md`).
- `tests/golden/` layout with the XML golden set GS-1…GS-11 + GS-14 fixtures and per-platform reference traces.
- CI job `golden` on all three OS runners.

## Tests
- CLI behavior tests (`tools/scena-run/tests/`).
- The golden CI job itself.

## Docs
- `scena-run` reference page.
- `golden-scenarios.md` flips from plan to live doc.

## Exit criteria
- [ ] GS-1…GS-11 green in CI on all platforms (bit-identity + checkpoints)
- [ ] Maintainer dry-runs the hand-execution procedure once on macOS
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p6-s3, p5-s6 (and P4, per the pillar dependency note — meaningful scenarios)
