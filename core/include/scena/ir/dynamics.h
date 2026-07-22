// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#pragma once

namespace scena::ir {

/// Shape of a value transition: the function f(x) describing how a value
/// changes between its current and target value. Per ASAM OpenSCENARIO XML
/// 1.4.0 §DynamicsShape.
enum class DynamicsShape {
    /// Linear transition: f(x) = f_0 + rate * x.
    Linear,
    /// Cubic transition with the constraint that the gradient is zero at start
    /// and end (a smoothstep). §DynamicsShape.
    Cubic,
    /// Sinusoidal transition with the constraint that the gradient is zero at
    /// start and end. §DynamicsShape.
    Sinusoidal,
    /// The target value is set instantaneously and consumes no simulation time.
    /// The associated dimension value must be 0. §DynamicsShape.
    Step,
};

/// How a target value is acquired: over a fixed time, over a fixed distance, or
/// at a fixed rate. Per ASAM OpenSCENARIO XML 1.4.0 §DynamicsDimension.
enum class DynamicsDimension {
    /// A predefined time (duration) is used to acquire the target value. [s].
    Time,
    /// A predefined distance is used to acquire the target value. [m].
    Distance,
    /// A predefined constant rate is used to acquire the target value.
    /// [delta/s].
    Rate,
};

/// Shape-following behavior of the actor. Per ASAM OpenSCENARIO XML 1.4.0
/// §FollowingMode.
///
/// Scena implements `Position` (the actor strictly adheres to the given
/// shape). `Follow` (best-effort, limited by the Performance envelope, dynamic
/// constraints and control-loop implementation, and therefore not guaranteed
/// identical across simulators) is accepted but treated as `Position` for now;
/// Scena instead applies a hard Performance clamp to every transition. See
/// ADR-0011.
enum class FollowingMode {
    Position, ///< Strict adherence to the shape. Default value if omitted.
    Follow,   ///< Best-effort follow, limited by Performance/constraints.
};

/// Dynamics of a value transition: how a value changes over time or distance,
/// using a shape. Per ASAM OpenSCENARIO XML 1.4.0 §TransitionDynamics.
///
/// `Step` is an immediate transition (a jump to the target value); in that case
/// `value` must be 0. `Linear` is a linear transition between start and target;
/// a smooth transition is given only with `Cubic` or `Sinusoidal`.
struct TransitionDynamics {
    DynamicsShape shape = DynamicsShape::Step;
    DynamicsDimension dimension = DynamicsDimension::Time;
    /// The value for a rate ([delta/s]), time ([s]) or distance ([m]) used to
    /// acquire the target value. Range: [0..inf[.
    double value = 0.0;
    FollowingMode following_mode = FollowingMode::Position;
};

} // namespace scena::ir
