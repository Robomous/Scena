# ADR-0006: Deterministic transcendental math

- **Status:** accepted
- **Date:** 2026-07-21

## Context

Scena's core promise is bit-identity: an identical scenario plus an identical
`step(dt)` sequence must produce **memcmp-equal** entity states on every
platform and ISA (ADR-0003). The kinematics integrator computes entity motion
from `sin`/`cos` of each entity's heading, and that is the first place the
promise meets a hazard the earlier phases did not:

- **libm is not bit-reproducible.** The C `sin`/`cos` are only required to be a
  few ulp accurate, not correctly rounded. Their last bits differ between
  platforms (glibc vs Apple libm vs MSVC CRT), between library versions, and
  between CPU generations. Two runners computing `sin(heading)` will not agree
  bit-for-bit.
- **FMA contraction is an ISA-level divergence.** `a * b + c` may fuse into a
  single fused-multiply-add that rounds once, or execute as two operations that
  round twice, depending on the compiler and target. GCC defaults to
  `-ffp-contract=fast`; Apple clang fuses on arm64. Our macOS runner is arm64
  while the Linux and Windows runners are x64 — a live cross-ISA hazard.

This is roadmap risk **R1**: if bit-identity is not mechanically enforced
before the numeric pillars (P2 motion, P3 road) build on it, a latent
divergence can go undetected until it is expensive to unwind.

## Decision

**Vendor a deterministic transcendental implementation and pin IEEE-strict
evaluation**, rather than relying on libm or maintaining per-platform golden
values.

### detmath

`scena::runtime::det_sin` / `det_cos` / `det_sincos`
(`core/include/scena/runtime/detmath.h`) replace all libm trig in the runtime:

- **Algorithm:** Cody & Waite argument reduction (π/2 split into three 30-bit
  chunks, so `kd * chunk` is exact for `|kd| < 2^22`, i.e. `|x| ≤ 1e6`)
  followed by fully-parenthesized Taylor kernels on `[-π/4, π/4]`. The kernel
  coefficients are **exact rationals** (`(-1)^j/(2j+1)!` and `(-1)^j/(2j)!`),
  not a Remez fit, so the implementation is clean-room by construction
  (ADR-0002) and reproducible from `scripts/gen_detmath_coeffs.py`.
- **Domain:** `|x| ≤ 1e6` radians. Out-of-domain inputs — including NaN and
  ±inf — return a quiet NaN; signed zero is preserved.
- **Accuracy is a few ulp, not correctly rounded.** Determinism is the
  contract, not last-bit precision. `detmath_test` cross-checks a `1e-12` match
  against libm so the pinned bits cannot lock in a wrong-but-consistent result.

### FMA / evaluation policy

`cmake/FloatingPoint.cmake` (`scn_set_fp_strictness`) applies `-ffp-contract=off`
(GCC/Clang) and `/fp:precise` (MSVC) to every first-party target — core, capi,
frontends, tests, the support library, and the trace runner. Each floating-point
operation rounds independently on every platform.

### Enforcement

- **Guard:** `scripts/check_detmath.sh` fails if any raw libm transcendental
  appears in `core/` runtime code (the IEEE-exact functions — `sqrt`, `fabs`,
  `floor`, … — and `detmath.cpp` itself are exempt). It runs in the `format` CI
  job.
- **Cross-platform proof:** `scena-trace-runner` records a fixed detmath +
  engine workload to a locale-immune text trace; the `determinism-trace` job
  produces one on each of the three OS runners and `determinism-cross` diffs
  them byte-for-byte.

## Consequences

- Entity motion is bit-identical across Linux/macOS/Windows and x64/arm64, and
  a regression fails CI rather than silently drifting.
- **detmath output is not stable across Scena versions.** Any change to the
  algorithm or coefficients changes the exact bits — a golden-breaking event
  that must be release-noted and re-pins `detmath_test` and any downstream
  trace goldens.
- New runtime transcendental needs (e.g. `atan2` for lane geometry in later
  pillars) must be added to detmath first; the guard blocks the libm shortcut.
- The few-ulp accuracy is more than sufficient for placeholder kinematics and
  is documented as a non-goal to exceed; correctness is verified against libm,
  determinism against the cross-platform trace.
- Hosts remain part of the determinism equation (ADR-0003): a host that flips
  the FPU into flush-to-zero / denormals-are-zero mode, or reports
  non-reproducible host-controlled states, can still break end-to-end
  bit-identity. The user-guide determinism page states what hosts must uphold.
