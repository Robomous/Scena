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

#include "parameters.h"

#include <utility>

#include "read_common.h"
#include "scena/ir/rule.h"

namespace scena::xml::detail {

namespace {

constexpr const char* kRuleParameterName =
    "asam.net:xosc:1.1.0:naming.parameter_declaration_parameter_name";
constexpr const char* kRuleReservedPrefix =
    "asam.net:xosc:1.0.0:naming.parameter_declaration_name_prefix_reserved";
constexpr const char* kRuleTypeInference =
    "asam.net:xosc:1.0.0:parameters.parameter_declaration_parameter_type_inference";
constexpr const char* kRuleTypeCasting = "asam.net:xosc:1.1.0:expressions.type_casting";

constexpr std::initializer_list<EnumEntry<ir::Rule>> kConstraintRules = {
    {"equalTo", ir::Rule::EqualTo},         {"greaterThan", ir::Rule::GreaterThan},
    {"lessThan", ir::Rule::LessThan},       {"greaterOrEqual", ir::Rule::GreaterOrEqual},
    {"lessOrEqual", ir::Rule::LessOrEqual}, {"notEqualTo", ir::Rule::NotEqualTo},
};

/// The parameter name syntax of §9.1: `[A-Za-z_][A-Za-z0-9_]*`.
bool is_valid_parameter_name(std::string_view name) {
    if (name.empty()) {
        return false;
    }
    const auto is_letter = [](char character) {
        return (character >= 'a' && character <= 'z') || (character >= 'A' && character <= 'Z') ||
               character == '_';
    };
    if (!is_letter(name.front())) {
        return false;
    }
    for (const char character : name.substr(1)) {
        if (!is_letter(character) && !(character >= '0' && character <= '9')) {
            return false;
        }
    }
    return true;
}

/// Converts an evaluated value to the declared type, applying the one
/// implicit conversion the standard allows: an integer widens to a double,
/// never the other way (rule expressions.type_casting).
std::optional<Value> convert_to(const Value& value, ValueType declared) {
    if (value.type == declared) {
        return value;
    }
    if (declared == ValueType::Double && is_integral(value.type)) {
        Value widened;
        widened.type = ValueType::Double;
        widened.number = static_cast<double>(value.integer);
        return widened;
    }
    if (is_integral(declared) && is_integral(value.type)) {
        // The three integer types share one representation; range checking
        // happens where the text is parsed.
        Value narrowed = value;
        narrowed.type = declared;
        return narrowed;
    }
    if ((declared == ValueType::String || declared == ValueType::DateTime) &&
        (value.type == ValueType::String || value.type == ValueType::DateTime)) {
        Value retyped = value;
        retyped.type = declared;
        return retyped;
    }
    return std::nullopt;
}

/// Checks a declared value against one ConstraintGroup (§9.1): the group is
/// satisfied when every ValueConstraint in it holds.
bool group_is_satisfied(ReadContext& ctx, const pugi::xml_node& group, const Value& value,
                        bool& readable) {
    bool satisfied = true;
    for (pugi::xml_node constraint : group.children("ValueConstraint")) {
        ir::Rule rule = ir::Rule::EqualTo;
        std::string reference;
        if (!read_enum(ctx, constraint, "rule", kConstraintRules, rule) ||
            !require_string(ctx, constraint, "value", reference)) {
            readable = false;
            continue;
        }
        // The same comparison the by-value conditions use: numeric when both
        // sides read as scalars, byte-wise equality otherwise.
        if (!ir::compare_values(value.to_text(), rule, reference)) {
            satisfied = false;
        }
    }
    return satisfied;
}

} // namespace

void ParameterScope::declare(std::string name, Value value) {
    frames_.back().insert_or_assign(std::move(name), std::move(value));
}

std::optional<Value> ParameterScope::find(std::string_view name) const {
    for (auto frame = frames_.rbegin(); frame != frames_.rend(); ++frame) {
        const auto found = frame->find(name);
        if (found != frame->end()) {
            return found->second;
        }
    }
    return std::nullopt;
}

const std::map<std::string, Value, std::less<>>& ParameterScope::declared() const {
    return frames_.front();
}

void ParameterScope::push() {
    frames_.emplace_back();
}

void ParameterScope::pop() {
    // The outermost frame holds the global parameters and is never popped.
    if (frames_.size() > 1) {
        frames_.pop_back();
    }
}

std::optional<std::string> resolve_attribute_text(ReadContext& ctx, const pugi::xml_node& node,
                                                  const char* attribute, std::string_view raw) {
    if (raw.empty()) {
        return std::string(raw);
    }

    if (raw.size() >= 3 && raw.front() == '$' && raw[1] == '{' && raw.back() == '}') {
        const std::string_view body = raw.substr(2, raw.size() - 3);
        ExpressionError error;
        const std::optional<Value> value = evaluate_expression(
            body, [&ctx](std::string_view name) { return ctx.parameters().find(name); },
            // The `pi` constant is a 1.4 addition; a document in the targeted
            // 1.0-1.3 range does not have it.
            ctx.version_at_least(1, 4), error);
        if (!value.has_value()) {
            ctx.report_at(node, Severity::Error, Status::ValidationError,
                          attribute_path(node, attribute),
                          std::string("expression in attribute '") + attribute +
                              "' cannot be evaluated: " + error.message,
                          error.rule_id);
            return std::nullopt;
        }
        return value->to_text();
    }

    if (raw.front() == '$') {
        const std::string_view name = raw.substr(1);
        if (!is_valid_parameter_name(name)) {
            ctx.report_at(node, Severity::Error, Status::ValidationError,
                          attribute_path(node, attribute),
                          std::string("'") + std::string(raw) + "' is not a parameter reference",
                          kRuleParameterName);
            return std::nullopt;
        }
        const std::optional<Value> value = ctx.parameters().find(name);
        if (!value.has_value()) {
            ctx.report_at(node, Severity::Error, Status::SemanticError,
                          attribute_path(node, attribute),
                          std::string("parameter '") + std::string(name) + "' is not in scope",
                          "asam.net:xosc:1.1.0:parameters.parameter_declaration_parameter_scope");
            return std::nullopt;
        }
        return value->to_text();
    }

    return std::string(raw);
}

void read_parameter_declarations(ReadContext& ctx, const pugi::xml_node& node) {
    static const char* const kConsumed[] = {"ParameterDeclaration", nullptr};
    warn_unconsumed_children(ctx, node, kConsumed);

    for (pugi::xml_node declaration : node.children("ParameterDeclaration")) {
        static const char* const kItemConsumed[] = {"ConstraintGroup", nullptr};
        warn_unconsumed_children(ctx, declaration, kItemConsumed);

        std::string name;
        std::string type_text;
        std::string value_text;
        if (!require_string(ctx, declaration, "name", name) ||
            !require_string(ctx, declaration, "parameterType", type_text) ||
            !require_string(ctx, declaration, "value", value_text)) {
            continue;
        }
        if (!is_valid_parameter_name(name)) {
            ctx.report_at(declaration, Severity::Error, Status::ValidationError,
                          attribute_path(declaration, "name"),
                          "parameter name must match [A-Za-z_][A-Za-z0-9_]*", kRuleParameterName);
            continue;
        }
        if (name.rfind("OSC", 0) == 0) {
            // "Parameter names starting with OSC are reserved for special use
            // in future versions" (§9.1) — a should, so a warning.
            ctx.report_at(declaration, Severity::Warning, Status::ValidationError,
                          attribute_path(declaration, "name"),
                          "parameter names starting with 'OSC' are reserved", kRuleReservedPrefix);
        }

        // The type itself may be given as a parameter reference (§9.1).
        const std::optional<std::string> resolved_type =
            resolve_attribute_text(ctx, declaration, "parameterType", type_text);
        if (!resolved_type.has_value()) {
            continue;
        }
        const std::optional<ValueType> type = parse_value_type(*resolved_type);
        if (!type.has_value()) {
            ctx.report_at(declaration, Severity::Error, Status::ValidationError,
                          attribute_path(declaration, "parameterType"),
                          std::string("'") + *resolved_type + "' is not a parameter type");
            continue;
        }

        // The value may reference or compute from parameters already in
        // scope — the declarations of one block are read in document order,
        // so a later one may build on an earlier one.
        const std::optional<std::string> resolved_value =
            resolve_attribute_text(ctx, declaration, "value", value_text);
        if (!resolved_value.has_value()) {
            continue;
        }
        std::optional<Value> value = parse_typed(*resolved_value, *type);
        if (!value.has_value()) {
            ctx.report_at(declaration, Severity::Error, Status::ValidationError,
                          attribute_path(declaration, "value"),
                          std::string("value '") + *resolved_value +
                              "' is not of the declared type '" + *resolved_type + "'",
                          kRuleTypeInference);
            continue;
        }
        // An expression may have produced a value of a different numeric
        // type; the one implicit conversion the standard allows is applied
        // here, and anything else is a type error.
        const std::optional<Value> converted = convert_to(*value, *type);
        if (!converted.has_value()) {
            ctx.report_at(declaration, Severity::Error, Status::ValidationError,
                          attribute_path(declaration, "value"),
                          "declared value does not convert to the declared type", kRuleTypeCasting);
            continue;
        }
        value = converted;

        // Constraint groups: the declared value must satisfy at least one
        // group (§9.1, ValueConstraintGroup is a disjunction of conjunctions).
        const pugi::xml_node first_group = declaration.child("ConstraintGroup");
        if (first_group) {
            bool any_satisfied = false;
            bool readable = true;
            for (pugi::xml_node group : declaration.children("ConstraintGroup")) {
                if (group_is_satisfied(ctx, group, *value, readable)) {
                    any_satisfied = true;
                }
            }
            if (readable && !any_satisfied) {
                ctx.report_at(declaration, Severity::Error, Status::ValidationError,
                              element_path(declaration),
                              std::string("declared value '") + value->to_text() +
                                  "' satisfies no constraint group");
                continue;
            }
        }

        ctx.parameters().declare(std::move(name), *value);
    }
}

} // namespace scena::xml::detail
