// SPDX-License-Identifier: MIT
#include "scena/runtime/longitudinal.h"

#include <cmath>
#include <limits>

#include "scena/runtime/detmath.h"

namespace scena::runtime {

namespace {

/// pi to double precision. Local to keep the transcendental constant with the
/// only code that needs it; the sinusoidal shape argument pi*p stays within
/// det_cos's input domain for p in [0, 1].
constexpr double kPi = 3.14159265358979311599796346854;

double clamp01(double p) noexcept {
    if (!(p > 0.0)) { // also maps NaN to 0
        return 0.0;
    }
    if (p > 1.0) {
        return 1.0;
    }
    return p;
}

} // namespace

double shape_fraction(ir::DynamicsShape shape, double p) noexcept {
    switch (shape) {
    case ir::DynamicsShape::Step:
        // Instantaneous: at the very start the value is still the origin, and
        // any positive progress has already reached the target.
        return p > 0.0 ? 1.0 : 0.0;
    case ir::DynamicsShape::Linear:
        return clamp01(p);
    case ir::DynamicsShape::Cubic: {
        // Smoothstep 3p^2 - 2p^3; gradient zero at p = 0 and p = 1.
        const double c = clamp01(p);
        return c * c * (3.0 - 2.0 * c);
    }
    case ir::DynamicsShape::Sinusoidal: {
        // (1 - cosine of pi*p) / 2; gradient zero at p = 0 and p = 1. det_cos,
        // not libm, so the transcendental is bit-identical across platforms.
        const double c = clamp01(p);
        return (1.0 - det_cos(kPi * c)) * 0.5;
    }
    }
    return clamp01(p); // unreachable; every enumerator is handled above
}

double transition_value(ir::DynamicsShape shape, double from, double to, double p) noexcept {
    return from + (to - from) * shape_fraction(shape, p);
}

double shape_peak_gradient_factor(ir::DynamicsShape shape) noexcept {
    switch (shape) {
    case ir::DynamicsShape::Step:
        return 0.0;
    case ir::DynamicsShape::Linear:
        return 1.0;
    case ir::DynamicsShape::Cubic:
        // max |g'(p)| of 3p^2 - 2p^3 is at p = 0.5: g'(0.5) = 6(0.5) - 6(0.25) = 1.5.
        return 1.5;
    case ir::DynamicsShape::Sinusoidal:
        // Peak gradient of the (1 - cosine)/2 shape is at p = 0.5, where it
        // equals pi/2.
        return kPi * 0.5;
    }
    return 1.0; // unreachable
}

double transition_duration(const ir::TransitionDynamics& td, double from, double to) noexcept {
    if (td.shape == ir::DynamicsShape::Step) {
        return 0.0;
    }
    const double delta = std::fabs(to - from);
    if (!(delta > 0.0)) { // no change (or NaN span) ⇒ nothing to acquire
        return 0.0;
    }
    if (!(td.value > 0.0) || !std::isfinite(td.value)) { // degenerate ⇒ instantaneous
        return 0.0;
    }
    switch (td.dimension) {
    case ir::DynamicsDimension::Time:
        return td.value;
    case ir::DynamicsDimension::Rate:
        return shape_peak_gradient_factor(td.shape) * delta / td.value;
    case ir::DynamicsDimension::Distance:
        // Progress is measured in metres travelled, not seconds.
        return std::numeric_limits<double>::quiet_NaN();
    }
    return 0.0; // unreachable
}

double LongitudinalController::speed() const noexcept {
    if (segments.empty()) {
        return 0.0;
    }
    if (done()) {
        return segments.back().to;
    }
    const Segment& s = segments[index];
    if (!(s.span > 0.0)) { // instantaneous segment: already at target
        return s.to;
    }
    const double p = s.by_distance ? traveled / s.span : elapsed / s.span;
    return transition_value(s.shape, s.from, s.to, p);
}

double LongitudinalController::advance(double dt, double step_distance) noexcept {
    double dt_left = dt > 0.0 ? dt : 0.0;
    double dist_left = step_distance > 0.0 ? step_distance : 0.0;

    while (!done()) {
        Segment& s = segments[index];
        if (!(s.span > 0.0)) { // instantaneous: consume no time/distance, advance
            ++index;
            elapsed = 0.0;
            traveled = 0.0;
            continue;
        }
        if (s.by_distance) {
            const double room = s.span - traveled;
            if (dist_left >= room) {
                dist_left -= room;
                ++index;
                elapsed = 0.0;
                traveled = 0.0;
                continue;
            }
            traveled += dist_left;
            dist_left = 0.0;
            break;
        }
        const double room = s.span - elapsed;
        if (dt_left >= room) {
            dt_left -= room;
            ++index;
            elapsed = 0.0;
            traveled = 0.0;
            continue;
        }
        elapsed += dt_left;
        dt_left = 0.0;
        break;
    }
    return speed();
}

} // namespace scena::runtime
