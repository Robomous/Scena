/*
 * Copyright 2026 Robomous
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <optional>

#include "scena/ir/dynamics.h"

namespace scena::runtime {

/// Deterministic lateral-dynamics helpers, per ASAM OpenSCENARIO XML 1.4.0
/// §7.4.1.4 and Class `LaneOffsetActionDynamics`.
///
/// The offset transitions themselves reuse the value-transition sequencer of
/// runtime/longitudinal.h: an offset ramp and a speed ramp are the same
/// §TransitionDynamics shape math over a different quantity. What is genuinely
/// lateral is here — the derivation of a transition duration from a maximum
/// lateral acceleration, which no longitudinal action needs.
///
/// Everything is pure arithmetic: no engine, gateway or wall-clock dependency,
/// no trigonometry, and std::sqrt is IEEE-exact, so results are bit-identical
/// across platforms (the determinism contract).

/// Peak absolute curvature |g''(p)| of a unit transition (span 1, delta 1) of
/// this shape, the `k` of the duration derivation below (ADR-0016):
///
/// - Sinusoidal: g(p) = (1 - cos(pi*p))/2, so g''(p) = (pi^2/2)*cos(pi*p) and
///   the peak is pi^2/2, reached at the endpoints.
/// - Cubic:      g(p) = 3p^2 - 2p^3, so g''(p) = 6 - 12p and the peak is 6,
///   likewise at the endpoints.
/// - Linear:     g''(p) = 0 in the interior and unbounded at the two kinks, so
///   curvature alone yields no duration. The documented substitute is the
///   minimum-time rest-to-rest bound for an acceleration-limited move of the
///   same displacement (accelerate at `a` for half the time, decelerate for the
///   other half): delta = a*T^2/4, i.e. k = 4. It is the unique
///   acceleration-limited lower bound and sits continuously below the smooth
///   shapes, 4 < pi^2/2 < 6.
/// - Step:       instantaneous, so 0.
[[nodiscard]] double shape_peak_curvature_factor(ir::DynamicsShape shape) noexcept;

/// Duration, in seconds, of a LaneOffsetAction transition covering `delta`
/// metres of lateral offset under the shape and the maximum lateral
/// acceleration.
///
/// A LaneOffsetAction authors no duration; Class `LaneOffsetActionDynamics`
/// gives only a shape and `maxLateralAcc`, so the duration is what makes the
/// peak lateral acceleration of the shaped offset ramp equal that limit. With
/// o(t) = delta * g(t/T) the lateral acceleration is delta * g''(p) / T^2, so
/// equating its peak to `a` gives
///
///     T = sqrt(shape_peak_curvature_factor(shape) * |delta| / a).
///
/// Returns 0 — an instantaneous transition — for a Step shape, for a zero (or
/// NaN) delta, and when `max_lateral_acc` is absent or non-finite: "Missing
/// value is interpreted as 'inf'" (Class `LaneOffsetActionDynamics`), and an
/// unbounded lateral acceleration reaches any offset in no time. A
/// non-positive limit is rejected at init (it would never reach the target), so
/// it is treated as instantaneous here rather than dividing by zero.
[[nodiscard]] double lane_offset_duration(ir::DynamicsShape shape, double delta,
                                          const std::optional<double>& max_lateral_acc) noexcept;

} // namespace scena::runtime
