// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <string>

#include "scena/ir/date_time.h"
#include "scena/ir/evaluation_context.h"
#include "scena/ir/rule.h"

namespace scena::ir {

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

/// Compares a named global parameter's current value to a reference literal
/// under `rule`, per ASAM OpenSCENARIO XML 1.4.0 ParameterCondition.
/// Parameters are immutable at runtime (§9.1), so this condition's result is
/// constant over a run. The comparison follows the scalar-convertibility
/// clause via compare_values(). A parameterRef the scenario does not declare
/// is rejected at Engine::init, so at runtime the referenced value is always
/// present; if the context cannot supply it the condition is false.
class ParameterCondition final : public Condition {
public:
    ParameterCondition(std::string parameter_ref, Rule rule, std::string value);

    [[nodiscard]] bool evaluate(const EvaluationContext& context) const override;

    [[nodiscard]] const std::string& parameter_ref() const;
    [[nodiscard]] Rule rule() const;
    [[nodiscard]] const std::string& value() const;

private:
    std::string parameter_ref_;
    Rule rule_;
    std::string value_;
};

/// Compares a named global variable's current value to a reference literal
/// under `rule`, per VariableCondition (≥1.2). Unlike parameters, variable
/// values change during the run (§6.12), so this condition is level-triggered
/// on the live value. A variableRef with no declaration is rejected at
/// Engine::init (rule resolvable_variable_reference).
class VariableCondition final : public Condition {
public:
    VariableCondition(std::string variable_ref, Rule rule, std::string value);

    [[nodiscard]] bool evaluate(const EvaluationContext& context) const override;

    [[nodiscard]] const std::string& variable_ref() const;
    [[nodiscard]] Rule rule() const;
    [[nodiscard]] const std::string& value() const;

private:
    std::string variable_ref_;
    Rule rule_;
    std::string value_;
};

/// Compares an externally supplied named value to a reference literal under
/// `rule`, per UserDefinedValueCondition — the interface for values the host
/// sets from outside the scenario. The name is not declared in the scenario,
/// so it is not validated at init; an unset name evaluates to a deterministic
/// false and the engine warns once.
class UserDefinedValueCondition final : public Condition {
public:
    UserDefinedValueCondition(std::string name, Rule rule, std::string value);

    [[nodiscard]] bool evaluate(const EvaluationContext& context) const override;

    [[nodiscard]] const std::string& name() const;
    [[nodiscard]] Rule rule() const;
    [[nodiscard]] const std::string& value() const;

private:
    std::string name_;
    Rule rule_;
    std::string value_;
};

/// Compares the simulated date-time to a reference date-time under `rule`, per
/// TimeOfDayCondition. The simulated date-time advances with simulation time
/// from a host-set anchor; until an anchor is set the condition is a
/// deterministic false and the engine warns once. The comparison is over
/// epoch seconds, so it honors the reference's UTC offset.
class TimeOfDayCondition final : public Condition {
public:
    TimeOfDayCondition(DateTime date_time, Rule rule);

    [[nodiscard]] bool evaluate(const EvaluationContext& context) const override;

    [[nodiscard]] const DateTime& date_time() const;
    [[nodiscard]] Rule rule() const;

private:
    DateTime date_time_;
    Rule rule_;
};

/// Holds while a referenced traffic signal is in a given observable state, per
/// TrafficSignalCondition (§7.6.5.2, §6.11).
///
/// The standard phrases it as "reaches a specific observable state". Scena
/// models it as a **level** predicate — true whenever the signal's current
/// state equals the reference — exactly like every other by-value condition in
/// the catalog; "reaches" is what the trigger's `conditionEdge rising` supplies
/// on top (see docs/user-guide/conditions.md). The comparison is byte-for-byte:
/// "interpretation and notation of state are specific to the simulation engine
/// used" (§6.11.4), so the engine must not read meaning into the string.
///
/// `name` is a road-network signal id, so it cannot be validated at load time
/// (rule traffic_signal_condition_references, C.7.10, needs a road network). A
/// signal nothing has written yet is a deterministic false and warns once.
class TrafficSignalCondition final : public Condition {
public:
    TrafficSignalCondition(std::string name, std::string state);

    [[nodiscard]] bool evaluate(const EvaluationContext& context) const override;

    /// ID of the referenced signal in the road network file.
    [[nodiscard]] const std::string& name() const;

    /// The observable state to be reached.
    [[nodiscard]] const std::string& state() const;

private:
    std::string name_;
    std::string state_;
};

/// Holds while a traffic signal controller is in a given semantic phase, per
/// TrafficSignalControllerCondition (§7.6.5.2, §6.11). A level predicate on the
/// phase name, with the same edge reasoning as TrafficSignalCondition.
///
/// Both references are resolvable within the scenario and are checked at
/// Engine::init (rule traffic_signal_controller_condition_references, C.7.12).
/// A controller that has not started yet — one waiting out its §6.11.3 delay —
/// has no phase, so the condition is a deterministic false.
class TrafficSignalControllerCondition final : public Condition {
public:
    TrafficSignalControllerCondition(std::string traffic_signal_controller_ref, std::string phase);

    [[nodiscard]] bool evaluate(const EvaluationContext& context) const override;

    /// Name of the referenced TrafficSignalController.
    [[nodiscard]] const std::string& traffic_signal_controller_ref() const;

    /// Name of the phase to be reached.
    [[nodiscard]] const std::string& phase() const;

private:
    std::string traffic_signal_controller_ref_;
    std::string phase_;
};

/// Holds when the referenced storyboard element is in the given runtime state,
/// or performs the given transition at this discrete time, per
/// StoryboardElementStateCondition. `element_ref` is a nameRef resolved
/// against the storyboard hierarchy (unique or "::"-disambiguated); the
/// referenced element must exist, which is checked at Engine::init. The
/// context answers the state/transition query; an unresolved reference is a
/// deterministic false.
class StoryboardElementStateCondition final : public Condition {
public:
    StoryboardElementStateCondition(StoryboardElementType element_type, std::string element_ref,
                                    StoryboardElementState state);

    [[nodiscard]] bool evaluate(const EvaluationContext& context) const override;

    [[nodiscard]] StoryboardElementType element_type() const;
    [[nodiscard]] const std::string& element_ref() const;
    [[nodiscard]] StoryboardElementState state() const;

private:
    StoryboardElementType element_type_;
    std::string element_ref_;
    StoryboardElementState state_;
};

} // namespace scena::ir
