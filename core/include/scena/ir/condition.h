// SPDX-License-Identifier: MIT
#pragma once

namespace scena::ir {

/// Base class for all storyboard trigger conditions in the Scenario IR.
///
/// In this phase conditions are evaluated against simulation time only. Later
/// phases extend evaluation to entity states and other runtime context, per
/// the ASAM OpenSCENARIO condition catalog.
class Condition {
public:
    virtual ~Condition() = default;

    /// Returns true when the condition holds at the given simulation time
    /// (seconds since Engine::init). Must be pure: no side effects, and the
    /// same inputs always produce the same result (determinism requirement).
    [[nodiscard]] virtual bool evaluate(double simulation_time) const = 0;
};

/// Fires once simulation time reaches `at_time` (greater-or-equal comparison),
/// modeled on the ASAM OpenSCENARIO SimulationTimeCondition.
class SimulationTimeCondition final : public Condition {
public:
    explicit SimulationTimeCondition(double at_time);

    [[nodiscard]] bool evaluate(double simulation_time) const override;

    [[nodiscard]] double at_time() const;

private:
    double at_time_;
};

} // namespace scena::ir
