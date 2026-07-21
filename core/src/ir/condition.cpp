// SPDX-License-Identifier: MIT
#include "scena/ir/condition.h"

#include "scena/ir/evaluation_context.h"
#include "scena/ir/rule.h"

namespace scena::ir {

SimulationTimeCondition::SimulationTimeCondition(double value, Rule rule)
    : value_(value), rule_(rule) {}

bool SimulationTimeCondition::evaluate(const EvaluationContext& context) const {
    // Left operand is the simulation time, right operand the reference value:
    // greaterOrEqual reads "time >= value", the classic fire-at-value edge.
    return compare(context.simulation_time(), rule_, value_);
}

double SimulationTimeCondition::value() const {
    return value_;
}

double SimulationTimeCondition::at_time() const {
    return value_;
}

Rule SimulationTimeCondition::rule() const {
    return rule_;
}

} // namespace scena::ir
