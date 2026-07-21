// SPDX-License-Identifier: MIT
#pragma once

#include "scena/ir/rule.h"

namespace scena::ir {

class EvaluationContext;

/// Base class for all storyboard trigger conditions in the Scenario IR — the
/// logical-expression layer the ASAM OpenSCENARIO condition catalog subclasses
/// (§7.6.5). A condition is evaluated against an EvaluationContext, which
/// exposes the runtime facets it may read (simulation time, named values,
/// time of day, storyboard element state).
class Condition {
public:
    virtual ~Condition() = default;

    /// Returns true when the condition holds against `context`. Must be pure:
    /// no side effects, and the same inputs always produce the same result
    /// (determinism requirement). A condition that reads a facet the context
    /// does not provide returns a deterministic false.
    [[nodiscard]] virtual bool evaluate(const EvaluationContext& context) const = 0;
};

/// Compares the simulation time to `value` under `rule`, per ASAM
/// OpenSCENARIO XML 1.4.0 SimulationTimeCondition. The default rule is
/// greaterOrEqual, which is the "fires once time reaches value" behavior.
class SimulationTimeCondition final : public Condition {
public:
    explicit SimulationTimeCondition(double value, Rule rule = Rule::GreaterOrEqual);

    [[nodiscard]] bool evaluate(const EvaluationContext& context) const override;

    /// The reference time value in seconds (the `value` attribute).
    [[nodiscard]] double value() const;

    /// Backwards-compatible alias of value(): the trigger fixtures, the C API
    /// and the Python binding spell the attribute `at_time`.
    [[nodiscard]] double at_time() const;

    /// The comparison operator (the `rule` attribute).
    [[nodiscard]] Rule rule() const;

private:
    double value_;
    Rule rule_;
};

} // namespace scena::ir
