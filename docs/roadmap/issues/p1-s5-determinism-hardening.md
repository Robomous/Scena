<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p1-s5] Determinism hardening & replay harness

**Pillar:** P1 — Runtime Core & Determinism · **Roadmap:** `docs/roadmap/roadmap.md` § p1-s5

## Goal
Make the bit-identity promise mechanically enforced before the numeric pillars build on it (risk R1).

## Deliverables
- Kernel: `runtime/detmath.h` — the only sanctioned transcendental/rounding entry points for runtime code (platform-stable implementations, no `-ffast-math`, FMA policy fixed and documented).
- Kernel: CI-enforced guard forbidding raw `<cmath>` transcendentals in `core/src/runtime` and `core/src/ir`.
- Kernel: trace recorder (binary-exact state dump per step) in the test support library.
- CI job `determinism-cross` that runs a fixture set on all three OS runners and diffs traces bit-exactly.

## Tests
- `detmath_test.cpp` (value pinning against hex-float references).
- `determinism_test` extended to long-horizon accumulation.

## Docs
- Determinism contract page in the user guide (including what hosts must uphold, per ADR-0003).

## Exit criteria
- [ ] `determinism-cross` CI job green
- [ ] detmath guard active
- [ ] Hex-pinning tests green on macOS/Linux/Windows
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p1-s4
