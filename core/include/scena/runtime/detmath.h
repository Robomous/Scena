// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#pragma once

namespace scena::runtime {

/// Deterministic transcendental math.
///
/// The runtime's bit-identity contract (identical scenario + identical step
/// sequence => memcmp-equal entity states on every platform and ISA) forbids
/// the libm transcendentals: `std::sin`, `std::cos`, and friends are only
/// required to be a few ulp accurate, and their results differ between
/// platforms, compilers, and CPU generations. This header provides vendored
/// replacements whose output depends on nothing but the IEEE-754 double
/// operations the compiler is pinned to (`-ffp-contract=off` / `/fp:precise`),
/// so it is identical everywhere.
///
/// Accuracy is a few ulp, NOT correctly rounded — determinism is the contract,
/// not last-bit precision, and the exact bits may change between Scena
/// versions (such a change is release-noted and breaks trace goldens). A CI
/// guard (scripts/check_detmath.sh) forbids raw libm transcendentals anywhere
/// in core/; all trig must route through the entry points below.
///
/// Only transcendental functions are banned. The IEEE-exact std functions are
/// bit-identical across platforms by the standard and may be used directly in
/// runtime code: `sqrt`, `fabs`, `abs`, `floor`, `ceil`, `trunc`, `round`,
/// `fmod`, `remainder`, `fmin`, `fmax`, `copysign`, `ldexp`, `frexp`, and the
/// classifiers (`isnan`, `isinf`, `isfinite`, `signbit`, ...).
///
/// Domain: inputs are radians with `|x| <= kDetTrigMaxAbsInput`. Outside that
/// range — including NaN and +/-inf — the result is a quiet NaN, because
/// argument reduction loses all significance for very large magnitudes and the
/// contract cannot be honored. Signed zero is preserved: `det_sin(-0.0)` is
/// `-0.0`.

/// Largest accepted absolute input, in radians. Beyond this, Cody & Waite
/// reduction can no longer keep the reduced argument exact, so the functions
/// return a quiet NaN rather than a value that would not reproduce.
inline constexpr double kDetTrigMaxAbsInput = 1.0e6;

/// Paired sine and cosine of the same argument. Defaults describe cos/sin of
/// zero so a default-constructed value is a valid (angle 0) result.
struct SinCos {
    double sin = 0.0;
    double cos = 1.0;
};

/// Deterministic sine. `+/-0` maps to `+/-0`; NaN, +/-inf, or `|x|` above
/// kDetTrigMaxAbsInput map to a quiet NaN.
[[nodiscard]] double det_sin(double x) noexcept;

/// Deterministic cosine. `+/-0` maps to `1.0`; same out-of-domain policy as
/// det_sin.
[[nodiscard]] double det_cos(double x) noexcept;

/// Deterministic sine and cosine computed together (one argument reduction).
/// `det_sincos(x).sin` is bit-identical to `det_sin(x)`, likewise for cos.
[[nodiscard]] SinCos det_sincos(double x) noexcept;

} // namespace scena::runtime
