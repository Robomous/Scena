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

#include "scena/runtime/lateral.h"

#include <cmath>

namespace scena::runtime {

namespace {

/// pi to double precision, the same literal runtime/longitudinal.cpp uses. The
/// sinusoidal curvature factor is pi^2/2; forming it from this literal by two
/// IEEE multiplies keeps it identical on every platform.
constexpr double kPi = 3.14159265358979311599796346854;

} // namespace

double shape_peak_curvature_factor(ir::DynamicsShape shape) noexcept {
    switch (shape) {
    case ir::DynamicsShape::Step:
        return 0.0;
    case ir::DynamicsShape::Linear:
        // Minimum-time rest-to-rest bound: delta = a*T^2/4 (see lateral.h).
        return 4.0;
    case ir::DynamicsShape::Cubic:
        // max |g''(p)| of 3p^2 - 2p^3 is |6 - 12p| at p = 0 and p = 1: 6.
        return 6.0;
    case ir::DynamicsShape::Sinusoidal:
        // max |g''(p)| of the (1 - cosine)/2 shape is pi^2/2, at p = 0 and
        // p = 1.
        return kPi * kPi * 0.5;
    }
    return 0.0; // unreachable; every enumerator is handled above
}

double lane_offset_duration(ir::DynamicsShape shape, double delta,
                            const std::optional<double>& max_lateral_acc) noexcept {
    if (shape == ir::DynamicsShape::Step) {
        return 0.0; // §7.4.1.4: "performed instantaneously - not over time"
    }
    const double span = std::fabs(delta);
    if (!(span > 0.0)) { // no change (or NaN) ⇒ nothing to acquire
        return 0.0;
    }
    // "Missing value is interpreted as 'inf'" — an unbounded lateral
    // acceleration reaches the offset in no time. A non-positive limit is
    // rejected at init, so treating it the same way here is defensive.
    if (!max_lateral_acc.has_value() || !std::isfinite(*max_lateral_acc) ||
        !(*max_lateral_acc > 0.0)) {
        return 0.0;
    }
    // T = sqrt(k * |delta| / a). Fixed operand order; std::sqrt is IEEE-exact.
    return std::sqrt(shape_peak_curvature_factor(shape) * span / *max_lateral_acc);
}

} // namespace scena::runtime
