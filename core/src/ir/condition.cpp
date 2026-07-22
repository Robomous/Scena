// SPDX-License-Identifier: MIT
#include "scena/ir/condition.h"

#include <optional>
#include <string_view>
#include <utility>

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

ParameterCondition::ParameterCondition(std::string parameter_ref, Rule rule, std::string value)
    : parameter_ref_(std::move(parameter_ref)), rule_(rule), value_(std::move(value)) {}

bool ParameterCondition::evaluate(const EvaluationContext& context) const {
    // Left operand is the stored parameter value, right operand the literal.
    const std::optional<std::string_view> stored =
        context.named_value(NamedValueKind::Parameter, parameter_ref_);
    if (!stored.has_value()) {
        return false;
    }
    return compare_values(*stored, rule_, value_);
}

const std::string& ParameterCondition::parameter_ref() const {
    return parameter_ref_;
}

Rule ParameterCondition::rule() const {
    return rule_;
}

const std::string& ParameterCondition::value() const {
    return value_;
}

VariableCondition::VariableCondition(std::string variable_ref, Rule rule, std::string value)
    : variable_ref_(std::move(variable_ref)), rule_(rule), value_(std::move(value)) {}

bool VariableCondition::evaluate(const EvaluationContext& context) const {
    const std::optional<std::string_view> stored =
        context.named_value(NamedValueKind::Variable, variable_ref_);
    if (!stored.has_value()) {
        return false;
    }
    return compare_values(*stored, rule_, value_);
}

const std::string& VariableCondition::variable_ref() const {
    return variable_ref_;
}

Rule VariableCondition::rule() const {
    return rule_;
}

const std::string& VariableCondition::value() const {
    return value_;
}

UserDefinedValueCondition::UserDefinedValueCondition(std::string name, Rule rule, std::string value)
    : name_(std::move(name)), rule_(rule), value_(std::move(value)) {}

bool UserDefinedValueCondition::evaluate(const EvaluationContext& context) const {
    const std::optional<std::string_view> stored =
        context.named_value(NamedValueKind::UserDefinedValue, name_);
    if (!stored.has_value()) {
        return false;
    }
    return compare_values(*stored, rule_, value_);
}

const std::string& UserDefinedValueCondition::name() const {
    return name_;
}

Rule UserDefinedValueCondition::rule() const {
    return rule_;
}

const std::string& UserDefinedValueCondition::value() const {
    return value_;
}

} // namespace scena::ir
