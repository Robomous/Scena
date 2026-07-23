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

#include <cstddef>
#include <vector>

#include "scena/ir/dynamics.h"

namespace scena::runtime {

/// Deterministic longitudinal-dynamics evaluator and controller.
///
/// Implements the value-transition math of ASAM OpenSCENARIO XML 1.4.0
/// §TransitionDynamics / §DynamicsShape as pure kinematics: no engine, gateway
/// or wall-clock dependency, and all trig routed through det_sincos so results
/// are bit-identical across platforms (the determinism contract). The engine's
/// default longitudinal controller (p2-s2) is built on top of this.

/// Normalized transition fraction g(p) in [0, 1] for progress p, for the given
/// shape. p is clamped to [0, 1]. g(0) = 0 and g(1) = 1 for Linear, Cubic and
/// Sinusoidal; Cubic and Sinusoidal additionally have g'(0) = g'(1) = 0. Step
/// is instantaneous: g(p) = 0 for p <= 0, else 1.
///
/// - Linear:     g(p) = p.
/// - Cubic:      g(p) = 3p^2 - 2p^3 (smoothstep).
/// - Sinusoidal: g(p) = (1 - cosine of pi*p) / 2, via det_cos.
[[nodiscard]] double shape_fraction(ir::DynamicsShape shape, double p) noexcept;

/// The transition value at progress p between `from` and `to`, for the given
/// shape: `from + (to - from) * shape_fraction(shape, p)`. p is clamped to
/// [0, 1].
[[nodiscard]] double transition_value(ir::DynamicsShape shape, double from, double to,
                                      double p) noexcept;

/// Peak absolute gradient of a unit-span (span 1, delta 1) transition of this
/// shape: 1 (Linear), 1.5 (Cubic), pi/2 (Sinusoidal), 0 (Step). Converts a
/// rate or acceleration limit into a transition duration: the duration whose
/// peak gradient equals a limit L over a value change |delta| is
/// `factor * |delta| / L`.
[[nodiscard]] double shape_peak_gradient_factor(ir::DynamicsShape shape) noexcept;

/// Resolved duration, in seconds, of a Time- or Rate-dimensioned transition
/// from `from` to `to`. Returns 0 for a transition that consumes no time —
/// Step shape, zero value span (`to == from`), or a non-positive/non-finite
/// `value`. Returns NaN for the Distance dimension, whose progress is measured
/// in metres travelled rather than seconds (see LongitudinalController).
///
/// For the Rate dimension `value` is read as the peak gradient of the shape, so
/// the duration is `shape_peak_gradient_factor(shape) * |to - from| / value`
/// (for Linear this is the constant slope, the plain reading of "constant
/// rate"; the shape-dependent factor generalizes it to the smooth shapes — see
/// ADR-0011).
[[nodiscard]] double transition_duration(const ir::TransitionDynamics& td, double from,
                                         double to) noexcept;

/// A default longitudinal speed controller: drives an entity's speed through a
/// sequence of segments. A plain SpeedAction is one segment; a
/// SpeedProfileAction is one segment per profile entry.
///
/// The controller is pure kinematics. Performance clamping is folded into the
/// segment spans by the caller (as an extended, achievable duration), so the
/// controller itself never needs the Performance envelope. Time/Rate segments
/// advance by elapsed simulation time; Distance segments advance by metres
/// travelled, which the caller supplies each step.
struct LongitudinalController {
    struct Segment {
        double from = 0.0; ///< Speed at segment start [m/s].
        double to = 0.0;   ///< Target speed [m/s].
        ir::DynamicsShape shape = ir::DynamicsShape::Linear;
        bool by_distance = false; ///< false: `span` is seconds; true: metres.
        double span = 0.0;        ///< Duration [s] or distance [m]; <= 0 ⇒ instant.
    };

    std::vector<Segment> segments;
    std::size_t index = 0; ///< Index of the active segment; == segments.size() when finished.
    double elapsed = 0.0;  ///< Seconds into the active time-segment.
    double traveled = 0.0; ///< Metres into the active distance-segment.

    /// True once every segment has completed.
    [[nodiscard]] bool done() const noexcept { return index >= segments.size(); }

    /// Commanded speed at the current progress. After done(), the final target
    /// (exactly, so a Sinusoidal endpoint is not left a det_cos ulp short).
    [[nodiscard]] double speed() const noexcept;

    /// Advances by one step of `dt` seconds during which the entity travelled
    /// `step_distance` metres (used only by distance-segments), crossing
    /// segment boundaries as needed and carrying the unused remainder into the
    /// next segment. Returns the commanded speed at the end of the step; sets
    /// done() once the last segment finishes.
    double advance(double dt, double step_distance) noexcept;
};

} // namespace scena::runtime
