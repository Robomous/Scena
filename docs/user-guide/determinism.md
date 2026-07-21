# Determinism

Determinism is Scena's core promise, not a nice-to-have: it is what makes
scenario runs reproducible, testable, and comparable across machines.

## The contract

> An identical scenario plus an identical `step(dt)` sequence produces
> **memcmp-equal** entity states — on Linux, macOS, and Windows, on x64 and
> arm64 alike.

"Identical step sequence" means the same `dt` values in the same order. Given
that, every `double` in every `EntityState` is bit-for-bit the same on every
platform, run after run. This is enforced by a dedicated test suite and, as of
this chapter, by a cross-platform CI job that diffs recorded traces byte-for-byte
(ADR-0006).

## How the engine guarantees it

- **No wall clock, no randomness.** The engine never reads `chrono::now`, never
  blocks, and uses no RNG. Simulated time is pure accumulation of the `dt`
  values the host supplies.
- **Deterministic iteration order.** Entities live in a sorted `std::map`, and
  the storyboard walk follows document order. Nothing that reaches results
  iterates an unordered container or depends on pointer addresses.
- **Fixed step order.** Within one `step(dt)` the phases always run in the same
  order: clock advance → poll host-controlled states → evaluate storyboard
  conditions and fire due actions → integrate engine-controlled entities →
  publish engine-controlled states. The order is documented in `engine.h` and
  is part of the contract (ADR-0003).
- **Pinned floating-point evaluation.** Every first-party target is built with
  FMA contraction off (`-ffp-contract=off`, `/fp:precise`), so `a * b + c`
  rounds identically everywhere instead of fusing on some ISAs (ADR-0006).
- **Deterministic transcendentals (detmath).** The integrator computes trig
  through `scena::runtime::det_*`, not platform libm — see below.

## detmath — deterministic sin/cos

libm's `sin`/`cos` are only a few ulp accurate and their last bits differ
across platforms, compilers, and CPU generations, so they cannot back a
bit-identity contract. `scena/runtime/detmath.h` provides vendored replacements
whose output depends only on IEEE-754 double operations:

- **Sanctioned entry points:** `det_sin(x)`, `det_cos(x)`, `det_sincos(x)`
  (paired, one argument reduction). A CI guard (`scripts/check_detmath.sh`)
  forbids raw libm transcendentals anywhere in `core/`; all trig must route
  through these.
- **Domain:** `x` is radians with `|x| ≤ 1e6`. Outside that range — including
  NaN and ±inf — the result is a quiet NaN. Signed zero is preserved
  (`det_sin(-0.0)` is `-0.0`).
- **IEEE-exact std functions are still allowed** directly in runtime code,
  because the standard makes them bit-identical: `sqrt`, `fabs`, `abs`,
  `floor`, `ceil`, `trunc`, `round`, `fmod`, `remainder`, `fmin`, `fmax`,
  `copysign`, `ldexp`, `frexp`, and the classifiers (`isnan`, `isinf`, …).
- **Accuracy is a few ulp, not correctly rounded**, and the exact bits are
  **not stable across Scena versions** — a change to the algorithm is
  release-noted and re-pins the goldens. Determinism is the contract;
  correctness is verified separately against libm to `1e-12`.

## What hosts must uphold

Host-controlled entities and the host's calling discipline are part of the
determinism equation (ADR-0003):

- **Identical `dt` sequence on replay.** Feed the same steps in the same order.
- **Reproducible host reports.** Any state the host reports for a
  host-controlled entity must itself be bit-identical on replay; the engine
  reproduces exactly what it is given.
- **Round-to-nearest, no FTZ/DAZ.** Do not run engine steps with the FPU in
  flush-to-zero / denormals-are-zero mode or under fast-math that flips the
  MXCSR/FPCR control bits — that changes how the engine's own arithmetic
  rounds.
- **Single-threaded calls.** The engine spawns no threads and expects `step`
  and the query/report calls to be made from one thread in order.

## Not guaranteed

- **NaN payloads.** Out-of-domain detmath results are quiet NaNs, but the NaN
  payload bits are out of contract and may differ across platforms.
- **Cross-version bit-stability.** A new Scena release may change detmath bits
  or step arithmetic; goldens are pinned per version, not forever.
- **Equality with libm.** detmath is close to libm (a few ulp), never promised
  equal to it.
- **x87 / extended precision.** 80-bit x87 evaluation is not a supported target;
  Scena assumes IEEE-754 double throughout.

## Replay harness

The building blocks that make the contract mechanically checkable live in the
`scena::testsupport` library (`core/tests/support/`):

- **`scena-trace-runner <output-file>`** writes a `scena-trace v1` image of a
  fixed detmath + engine workload. Two runs on different platforms produce
  byte-identical files iff the runtime is bit-identical.
- **Trace format** (`scena-trace v1`, ASCII, LF line endings; all doubles as 16
  lowercase hex chars of their IEEE-754 bits):

  | Line | Meaning |
  |---|---|
  | `# scena-trace v1 fixture=<name>` | Header; first line. |
  | `# <text>` | Free-form note. |
  | `t <step> <time>` | Engine clock at `step`. |
  | `e <step> <id> <x> <y> <z> <heading> <speed>` | Entity state (each field hex). |
  | `e <step> <id> MISSING` | The entity was absent at that step. |
  | `m <tag> <in> <out>` | A detmath result (`out` is `nan` when NaN). |

- **CI.** `determinism-trace` runs the trace runner on all three OS runners and
  uploads each trace; `determinism-cross` diffs the macOS and Windows traces
  against the Linux one byte-for-byte and fails on any mismatch.
- **Reuse in your own tests.** Link `scena::testsupport` for the shared
  determinism fixture (`make_determinism_scenario()`), the detmath probe list
  (`detmath_probe_inputs()`), the `hex_bits()` helper, and `TraceRecorder`.
