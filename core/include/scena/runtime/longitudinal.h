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

/// Peak absolute second derivative of a unit-span (span 1, delta 1) transition
/// of this shape — the factor that turns a jerk limit into a duration. The
/// duration whose peak jerk equals a limit J over a value change |delta| is
/// `sqrt(factor * |delta| / J)`.
///
/// Cubic: 6 (|g''| = |6 - 12p|, peak at both endpoints). Sinusoidal: pi^2/2
/// (|g''| is pi^2/2 times the absolute cosine of pi*p, peak at both endpoints,
/// and the shape itself routes through det_cos). Linear and Step are
/// infinity: both hold the gradient constant and then drop it to zero in no
/// time at all, so no finite duration bounds their second derivative, and no
/// jerk limit can be honoured by stretching them. `follow_shape` is what keeps
/// that infinity out of the arithmetic.
///
/// `lateral.h`'s `shape_peak_curvature_factor` is the same quantity for Cubic
/// and Sinusoidal, and deliberately differs for Linear and Step: there it
/// stands in as a minimum-time rest-to-rest bound for a lane offset that has no
/// authored duration at all, where "infinite" would be useless. Here the
/// infinity is the point — it says a jerk limit is unsatisfiable for that
/// shape.
[[nodiscard]] double shape_peak_jerk_factor(ir::DynamicsShape shape) noexcept;

/// The shape a transition is realised with under
/// `FollowingMode::Follow`. Sinusoidal is returned unchanged; every other shape
/// becomes Cubic.
///
/// §SpeedProfileAction is explicit that with `followingMode=follow` "the
/// acceleration is zero at the start and end of the profile", which excludes
/// the Linear and Step shapes by construction — their acceleration steps at the
/// endpoints. §FollowingMode describes `follow` as tracking the target "as good
/// as possible by observing the dynamic constraints of the entity", so the
/// authored shape is a request rather than a contract, and Scena honours it
/// with the closest shape that satisfies the endpoint requirement. Cubic is
/// that shape: it is the lowest-order polynomial with a zero gradient at both
/// ends, and it is what a Linear ramp becomes once its corners are rounded.
/// `FollowingMode::Position` keeps the authored shape exactly. See ADR-0024.
[[nodiscard]] ir::DynamicsShape follow_shape(ir::DynamicsShape shape) noexcept;

/// The shortest duration [s] in which a `delta`-sized value change of this
/// shape stays inside every supplied limit, never shorter than `authored`.
///
/// `acceleration_limit` bounds the peak first derivative [unit/s] and
/// `jerk_limit` the peak second derivative [unit/s^2]; either may be infinity
/// for "unconstrained" (§DynamicConstraints reads a missing value as 'inf').
/// A non-positive or non-finite `delta` needs no time, and an unconstrained
/// transition simply keeps its authored duration.
[[nodiscard]] double constrained_duration(ir::DynamicsShape shape, double delta,
                                          double acceleration_limit, double jerk_limit,
                                          double authored) noexcept;

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

/// A sequencer for shaped value transitions: drives a scalar through a sequence
/// of segments. A plain SpeedAction is one segment; a SpeedProfileAction is one
/// segment per profile entry.
///
/// The name is historical — it is the default longitudinal speed controller —
/// but nothing in it is longitudinal. p2-s3 reuses it verbatim as the lateral
/// offset sequencer of a LaneChangeAction or LaneOffsetAction, whose segments
/// carry metres of lateral offset rather than metres per second; an offset ramp
/// and a speed ramp are the same §TransitionDynamics shape math over a
/// different quantity, and sharing the code keeps them bit-identical.
///
/// The controller is pure kinematics. Performance clamping is folded into the
/// segment spans by the caller (as an extended, achievable duration), so the
/// controller itself never needs the Performance envelope. Time/Rate segments
/// advance by elapsed simulation time; Distance segments advance by metres
/// travelled, which the caller supplies each step.
struct LongitudinalController {
    struct Segment {
        double from = 0.0; ///< Value at segment start [m/s speed, or m offset].
        double to = 0.0;   ///< Target value, same unit as `from`.
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

    /// Commanded value at the current progress. After done(), the final target
    /// (exactly, so a Sinusoidal endpoint is not left a det_cos ulp short).
    [[nodiscard]] double speed() const noexcept;

    /// Advances by one step of `dt` seconds during which the entity travelled
    /// `step_distance` metres (used only by distance-segments), crossing
    /// segment boundaries as needed and carrying the unused remainder into the
    /// next segment. Returns the commanded value at the end of the step; sets
    /// done() once the last segment finishes.
    double advance(double dt, double step_distance) noexcept;
};

} // namespace scena::runtime
