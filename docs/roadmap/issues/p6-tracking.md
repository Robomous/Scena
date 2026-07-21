<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [P6] Embedding: Gateway, C API & Python — tracking

Tracking issue for pillar P6. See `docs/roadmap/roadmap.md`.

## Objective
The embedding surface at release quality: full-engine C ABI, mature gateway (state injection, callbacks, time sourcing), Python parity, and the `scena-run` headless CLI executing a scenario file and emitting a state trace (CSV/JSON) — deliberately no visualization (ADR-0001, library-first). Out of scope: GUI/visualization, concrete simulator adapters, network transports.

## Sprints
- [ ] p6-s1 — C ABI expansion to the full engine surface (#)
- [ ] p6-s2 — Gateway maturity: callbacks & state injection (#)
- [ ] p6-s3 — Python parity & examples (#)
- [ ] p6-s4 — scena-run CLI & golden harness (#)

## Pillar exit criteria
- [ ] Parity audit table (C++ ↔ C ABI ↔ Python) checked in with zero gaps
- [ ] `scena-run` executes the golden corpus
- [ ] The capi-and-bindings review checklist passes
