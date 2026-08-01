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

#include "expression.h"

#include <cmath>
#include <utility>

#include "scena/ir/rule.h" // ir::parse_scalar / ir::format_scalar
#include "scena/runtime/detmath.h"
#include "scena/xml/detail/attributes.h"

namespace scena::xml::detail {

namespace {

// Rule ids of annex C.11, the expression rule set.
constexpr const char* kRuleEvaluation =
    "asam.net:xosc:1.1.0:expressions.evaluation_of_expressions_possible";
constexpr const char* kRuleAllowedOperators = "asam.net:xosc:1.1.0:expressions.allowed_operators";
constexpr const char* kRuleArguments = "asam.net:xosc:1.1.0:expressions.arguments_of_operators";
constexpr const char* kRuleBooleanType = "asam.net:xosc:1.1.0:expressions.type_of_boolean";

/// π to the precision the standard prints (§9.2.2). Only reachable when the
/// document's version defines the constant.
constexpr double kPi = 3.141592653589793;

bool is_name_start(char character) {
    return (character >= 'a' && character <= 'z') || (character >= 'A' && character <= 'Z') ||
           character == '_';
}

bool is_name_char(char character) {
    return is_name_start(character) || (character >= '0' && character <= '9');
}

Value make_integer(long long value, ValueType type = ValueType::Integer) {
    Value result;
    result.type = type;
    result.integer = value;
    return result;
}

Value make_double(double value) {
    Value result;
    result.type = ValueType::Double;
    result.number = value;
    return result;
}

Value make_boolean(bool value) {
    Value result;
    result.type = ValueType::Boolean;
    result.boolean = value;
    return result;
}

/// A recursive-descent parser over the expression body.
///
/// The grammar is §9.2.1's precedence ladder, lowest binding first:
///   or  := and ( 'or' and )*
///   and := not ( 'and' not )*
///   not := 'not' not | sum
///   sum := product ( ('+' | '-') product )*
///   product := unary ( ('*' | '/' | '%') unary )*
///   unary := '-' unary | primary
///   primary := number | 'true' | 'false' | '$name' | name | call | '(' or ')'
///
/// Boolean and arithmetic operators share one ladder because the standard
/// gives them one precedence order; the type rules — not the grammar —
/// reject mixing them.
class Parser {
public:
    Parser(std::string_view text, const ParameterLookup& lookup, bool version_supports_pi,
           ExpressionError& error)
        : text_(text), lookup_(lookup), pi_available_(version_supports_pi), error_(error) {}

    std::optional<Value> parse() {
        std::optional<Value> value = parse_or();
        if (!value.has_value()) {
            return std::nullopt;
        }
        skip_spaces();
        if (at_ != text_.size()) {
            return fail("unexpected trailing text in expression", kRuleEvaluation);
        }
        return value;
    }

private:
    std::optional<Value> fail(std::string message, const char* rule_id) {
        if (error_.message.empty()) {
            error_.message = std::move(message);
            error_.rule_id = rule_id;
        }
        return std::nullopt;
    }

    void skip_spaces() {
        while (at_ < text_.size() && (text_[at_] == ' ' || text_[at_] == '\t' ||
                                      text_[at_] == '\n' || text_[at_] == '\r')) {
            ++at_;
        }
    }

    bool consume(char character) {
        skip_spaces();
        if (at_ < text_.size() && text_[at_] == character) {
            ++at_;
            return true;
        }
        return false;
    }

    /// Consumes the keyword `word` when it stands alone (not as the prefix of
    /// a longer name).
    bool consume_word(std::string_view word) {
        skip_spaces();
        if (text_.compare(at_, word.size(), word) != 0) {
            return false;
        }
        const std::size_t after = at_ + word.size();
        if (after < text_.size() && is_name_char(text_[after])) {
            return false;
        }
        at_ = after;
        return true;
    }

    [[nodiscard]] bool peek_word(std::string_view word) {
        const std::size_t saved = at_;
        const bool found = consume_word(word);
        at_ = saved;
        return found;
    }

    std::optional<Value> require_numeric(std::optional<Value> value, const char* what) {
        if (!value.has_value()) {
            return std::nullopt;
        }
        if (!is_numeric(value->type)) {
            return fail(std::string("operator '") + what + "' needs numeric operands",
                        value->type == ValueType::Boolean ? kRuleBooleanType
                                                          : kRuleAllowedOperators);
        }
        return value;
    }

    std::optional<Value> require_boolean(std::optional<Value> value, const char* what) {
        if (!value.has_value()) {
            return std::nullopt;
        }
        if (value->type != ValueType::Boolean) {
            return fail(std::string("operator '") + what + "' needs boolean operands",
                        kRuleBooleanType);
        }
        return value;
    }

    /// Rejects a non-finite result: an expression must be evaluable
    /// (rule evaluation_of_expressions_possible), and a NaN or infinity
    /// reaching the IR would silently poison everything derived from it.
    std::optional<Value> finite(Value value) {
        if (value.type == ValueType::Double && !std::isfinite(value.number)) {
            return fail("expression result is not a finite number", kRuleEvaluation);
        }
        return value;
    }

    std::optional<Value> parse_or() {
        std::optional<Value> left = parse_and();
        while (left.has_value() && peek_word("or")) {
            (void)consume_word("or");
            left = require_boolean(std::move(left), "or");
            std::optional<Value> right = require_boolean(parse_and(), "or");
            if (!left.has_value() || !right.has_value()) {
                return std::nullopt;
            }
            left = make_boolean(left->boolean || right->boolean);
        }
        return left;
    }

    std::optional<Value> parse_and() {
        std::optional<Value> left = parse_not();
        while (left.has_value() && peek_word("and")) {
            (void)consume_word("and");
            left = require_boolean(std::move(left), "and");
            std::optional<Value> right = require_boolean(parse_not(), "and");
            if (!left.has_value() || !right.has_value()) {
                return std::nullopt;
            }
            left = make_boolean(left->boolean && right->boolean);
        }
        return left;
    }

    std::optional<Value> parse_not() {
        if (consume_word("not")) {
            const std::optional<Value> value = require_boolean(parse_not(), "not");
            if (!value.has_value()) {
                return std::nullopt;
            }
            return make_boolean(!value->boolean);
        }
        return parse_sum();
    }

    std::optional<Value> parse_sum() {
        std::optional<Value> left = parse_product();
        while (left.has_value()) {
            skip_spaces();
            if (at_ >= text_.size() || (text_[at_] != '+' && text_[at_] != '-')) {
                break;
            }
            const char op = text_[at_];
            ++at_;
            left = require_numeric(std::move(left), op == '+' ? "+" : "-");
            std::optional<Value> right = require_numeric(parse_product(), op == '+' ? "+" : "-");
            if (!left.has_value() || !right.has_value()) {
                return std::nullopt;
            }
            if (is_integral(left->type) && is_integral(right->type)) {
                left = make_integer(op == '+' ? left->integer + right->integer
                                              : left->integer - right->integer);
            } else {
                left = finite(make_double(op == '+' ? left->as_double() + right->as_double()
                                                    : left->as_double() - right->as_double()));
            }
        }
        return left;
    }

    std::optional<Value> parse_product() {
        std::optional<Value> left = parse_unary();
        while (left.has_value()) {
            skip_spaces();
            if (at_ >= text_.size() ||
                (text_[at_] != '*' && text_[at_] != '/' && text_[at_] != '%')) {
                break;
            }
            const char op = text_[at_];
            ++at_;
            const char name[2] = {op, '\0'};
            left = require_numeric(std::move(left), name);
            std::optional<Value> right = require_numeric(parse_unary(), name);
            if (!left.has_value() || !right.has_value()) {
                return std::nullopt;
            }
            if (op == '/') {
                // "/" is defined for doubles only (§9.2.1), so integer
                // operands widen rather than truncating silently.
                if (right->as_double() == 0.0) {
                    return fail("division by zero", kRuleEvaluation);
                }
                left = finite(make_double(left->as_double() / right->as_double()));
                continue;
            }
            if (is_integral(left->type) && is_integral(right->type)) {
                if (op == '%' && right->integer == 0) {
                    return fail("remainder by zero", kRuleEvaluation);
                }
                left = make_integer(op == '*' ? left->integer * right->integer
                                              : left->integer % right->integer);
            } else if (op == '*') {
                left = finite(make_double(left->as_double() * right->as_double()));
            } else {
                // The standard is explicit that % is the remainder, not the
                // modulo; std::fmod is exactly that and is IEEE-exact.
                if (right->as_double() == 0.0) {
                    return fail("remainder by zero", kRuleEvaluation);
                }
                left = finite(make_double(std::fmod(left->as_double(), right->as_double())));
            }
        }
        return left;
    }

    std::optional<Value> parse_unary() {
        skip_spaces();
        if (at_ < text_.size() && text_[at_] == '-') {
            ++at_;
            const std::optional<Value> value = require_numeric(parse_unary(), "unary -");
            if (!value.has_value()) {
                return std::nullopt;
            }
            if (is_integral(value->type)) {
                return make_integer(-value->integer);
            }
            return make_double(-value->number);
        }
        return parse_primary();
    }

    /// Reads `count` comma-separated arguments of a call whose '(' has been
    /// consumed.
    bool parse_arguments(const char* name, int count, Value arguments[]) {
        for (int index = 0; index < count; ++index) {
            if (index > 0 && !consume(',')) {
                (void)fail(std::string("operator '") + name + "' needs " + std::to_string(count) +
                               " arguments",
                           kRuleArguments);
                return false;
            }
            const std::optional<Value> argument = parse_or();
            if (!argument.has_value()) {
                return false;
            }
            arguments[index] = *argument;
        }
        if (!consume(')')) {
            (void)fail(std::string("operator '") + name + "' has an unclosed argument list",
                       kRuleArguments);
            return false;
        }
        return true;
    }

    std::optional<Value> parse_call(std::string_view name) {
        if (!consume('(')) {
            // Every operator's arguments "shall be given after the operator
            // name and surrounded by parentheses" (rule arguments_of_operators).
            return fail(std::string("operator '") + std::string(name) +
                            "' needs its arguments in parentheses",
                        kRuleArguments);
        }
        const std::string operator_name(name);
        Value arguments[2];

        const int arity = (name == "pow" || name == "max" || name == "min") ? 2 : 1;
        if (!parse_arguments(operator_name.c_str(), arity, arguments)) {
            return std::nullopt;
        }
        for (int index = 0; index < arity; ++index) {
            if (!is_numeric(arguments[index].type)) {
                return fail(std::string("operator '") + operator_name + "' needs numeric arguments",
                            arguments[index].type == ValueType::Boolean ? kRuleBooleanType
                                                                        : kRuleAllowedOperators);
            }
        }
        const double first = arguments[0].as_double();

        if (name == "round") {
            // double -> int (§9.2.1). std::llround is the away-from-zero
            // rounding the standard's "round" names, and it is exact.
            return make_integer(std::llround(first));
        }
        if (name == "floor") {
            return make_integer(static_cast<long long>(std::floor(first)));
        }
        if (name == "ceil") {
            return make_integer(static_cast<long long>(std::ceil(first)));
        }
        if (name == "sqrt") {
            if (first < 0.0) {
                return fail("sqrt of a negative number", kRuleEvaluation);
            }
            return finite(make_double(std::sqrt(first)));
        }
        if (name == "sin") {
            return finite(make_double(runtime::det_sin(first)));
        }
        if (name == "cos") {
            return finite(make_double(runtime::det_cos(first)));
        }
        if (name == "tan") {
            const runtime::SinCos parts = runtime::det_sincos(first);
            if (parts.cos == 0.0) {
                return fail("tan is undefined at this argument", kRuleEvaluation);
            }
            return finite(make_double(parts.sin / parts.cos));
        }
        if (name == "asin" || name == "acos") {
            if (first < -1.0 || first > 1.0) {
                return fail(std::string("operator '") + operator_name +
                                "' needs an argument in [-1, 1]",
                            kRuleEvaluation);
            }
            // Both are derived from det_atan2 so the result is bit-identical
            // across platforms: the inverse sine of x is the deterministic
            // arc tangent of x over the complement, and the inverse cosine
            // swaps the two arguments.
            const double complement = std::sqrt(1.0 - first * first);
            return finite(make_double(name == "asin" ? runtime::det_atan2(first, complement)
                                                     : runtime::det_atan2(complement, first)));
        }
        if (name == "atan") {
            return finite(make_double(runtime::det_atan2(first, 1.0)));
        }
        if (name == "sign") {
            // "sign(x) = (x > 0) - (x < 0)" (§9.2.1), keeping the argument's
            // own type.
            const int sign = (first > 0.0) - (first < 0.0);
            return is_integral(arguments[0].type) ? make_integer(sign)
                                                  : make_double(static_cast<double>(sign));
        }
        if (name == "abs") {
            return is_integral(arguments[0].type)
                       ? make_integer(arguments[0].integer < 0 ? -arguments[0].integer
                                                               : arguments[0].integer)
                       : make_double(std::fabs(first));
        }
        if (name == "max" || name == "min") {
            const double second = arguments[1].as_double();
            const bool take_first = name == "max" ? first >= second : first <= second;
            const Value& chosen = take_first ? arguments[0] : arguments[1];
            if (is_integral(arguments[0].type) && is_integral(arguments[1].type)) {
                return make_integer(chosen.integer);
            }
            return make_double(chosen.as_double());
        }
        if (name == "pow") {
            const double exponent = arguments[1].as_double();
            if (exponent != std::floor(exponent) || std::fabs(exponent) > 1024.0) {
                // A general pow needs a platform exp/log pair, which is not
                // bit-identical across platforms and would break the
                // determinism contract before the engine ever runs.
                return fail("pow is supported for integer exponents only, so that the result is "
                            "bit-identical on every platform",
                            kRuleEvaluation);
            }
            double result = 1.0;
            const long long times = static_cast<long long>(std::fabs(exponent));
            for (long long index = 0; index < times; ++index) {
                result *= first;
            }
            if (exponent < 0.0) {
                if (result == 0.0) {
                    return fail("pow with a negative exponent of zero", kRuleEvaluation);
                }
                result = 1.0 / result;
            }
            return finite(make_double(result));
        }
        return fail(std::string("'") + operator_name + "' is not an OpenSCENARIO operator",
                    kRuleAllowedOperators);
    }

    std::optional<Value> parse_primary() {
        skip_spaces();
        if (at_ >= text_.size()) {
            return fail("expression ends unexpectedly", kRuleEvaluation);
        }
        if (text_[at_] == '(') {
            ++at_;
            const std::optional<Value> value = parse_or();
            if (!value.has_value()) {
                return std::nullopt;
            }
            if (!consume(')')) {
                return fail("unbalanced parentheses in expression", kRuleEvaluation);
            }
            return value;
        }
        if (text_[at_] == '$') {
            ++at_;
            const std::size_t begin = at_;
            while (at_ < text_.size() && is_name_char(text_[at_])) {
                ++at_;
            }
            if (begin == at_) {
                return fail("'$' is not followed by a parameter name", kRuleEvaluation);
            }
            const std::string_view name = text_.substr(begin, at_ - begin);
            const std::optional<Value> value = lookup_(name);
            if (!value.has_value()) {
                return fail(std::string("parameter '") + std::string(name) + "' is not declared",
                            "asam.net:xosc:1.1.0:parameters.parameter_declaration_parameter_scope");
            }
            if (value->type == ValueType::String || value->type == ValueType::DateTime) {
                return fail(std::string("parameter '") + std::string(name) +
                                "' is not numeric or boolean and cannot be used in an expression",
                            kRuleAllowedOperators);
            }
            return value;
        }
        if (is_name_start(text_[at_])) {
            const std::size_t begin = at_;
            while (at_ < text_.size() && is_name_char(text_[at_])) {
                ++at_;
            }
            const std::string_view name = text_.substr(begin, at_ - begin);
            if (name == "true" || name == "false") {
                return make_boolean(name == "true");
            }
            if (name == "pi") {
                if (!pi_available_) {
                    return fail("the constant 'pi' is not part of OpenSCENARIO XML 1.0-1.3",
                                kRuleAllowedOperators);
                }
                return make_double(kPi);
            }
            return parse_call(name);
        }

        // A numeric literal. Integers stay integers so integer arithmetic is
        // exact; anything with a '.', an exponent, or too many digits reads
        // as a double.
        const std::size_t begin = at_;
        while (at_ < text_.size() && ((text_[at_] >= '0' && text_[at_] <= '9') ||
                                      text_[at_] == '.' || text_[at_] == 'e' || text_[at_] == 'E' ||
                                      ((text_[at_] == '+' || text_[at_] == '-') && at_ > begin &&
                                       (text_[at_ - 1] == 'e' || text_[at_ - 1] == 'E')))) {
            ++at_;
        }
        if (begin == at_) {
            return fail("expression contains an unexpected character", kRuleEvaluation);
        }
        const std::string_view literal = text_.substr(begin, at_ - begin);
        if (const std::optional<long long> integer = parse_integer(literal)) {
            return make_integer(*integer);
        }
        if (const std::optional<double> number = parse_double(literal)) {
            return finite(make_double(*number));
        }
        return fail(std::string("'") + std::string(literal) + "' is not a number", kRuleEvaluation);
    }

    std::string_view text_;
    const ParameterLookup& lookup_;
    bool pi_available_;
    ExpressionError& error_;
    std::size_t at_ = 0;
};

} // namespace

std::string Value::to_text() const {
    switch (type) {
    case ValueType::Integer:
    case ValueType::UnsignedInt:
    case ValueType::UnsignedShort:
        return std::to_string(integer);
    case ValueType::Double:
        // The shortest text that reads back as the same double, so a value
        // that passes through an expression and back into an attribute is
        // bit-identical to the one that skipped it.
        return ir::format_scalar(number);
    case ValueType::Boolean:
        return boolean ? "true" : "false";
    case ValueType::String:
    case ValueType::DateTime:
        break;
    }
    return text;
}

std::optional<ValueType> parse_value_type(std::string_view text) {
    if (text == "integer" || text == "int") {
        // 1.2 renamed "integer" to "int"; both name the same type.
        return ValueType::Integer;
    }
    if (text == "unsignedInt") {
        return ValueType::UnsignedInt;
    }
    if (text == "unsignedShort") {
        return ValueType::UnsignedShort;
    }
    if (text == "double") {
        return ValueType::Double;
    }
    if (text == "boolean") {
        return ValueType::Boolean;
    }
    if (text == "string") {
        return ValueType::String;
    }
    if (text == "dateTime") {
        return ValueType::DateTime;
    }
    return std::nullopt;
}

std::optional<Value> parse_typed(std::string_view text, ValueType type) {
    Value value;
    value.type = type;
    switch (type) {
    case ValueType::Integer:
    case ValueType::UnsignedInt:
    case ValueType::UnsignedShort: {
        const std::optional<long long> integer = parse_integer(text);
        if (!integer.has_value()) {
            return std::nullopt;
        }
        if (type != ValueType::Integer && *integer < 0) {
            return std::nullopt; // the unsigned types have no negative values
        }
        if (type == ValueType::UnsignedShort && *integer > 65535) {
            return std::nullopt;
        }
        value.integer = *integer;
        return value;
    }
    case ValueType::Double: {
        const std::optional<double> number = parse_double(text);
        if (!number.has_value()) {
            return std::nullopt;
        }
        value.number = *number;
        return value;
    }
    case ValueType::Boolean: {
        const std::optional<bool> boolean = parse_boolean(text);
        if (!boolean.has_value()) {
            return std::nullopt;
        }
        value.boolean = *boolean;
        return value;
    }
    case ValueType::String:
    case ValueType::DateTime:
        value.text = std::string(text);
        return value;
    }
    return std::nullopt;
}

std::optional<Value> evaluate_expression(std::string_view body, const ParameterLookup& lookup,
                                         bool version_supports_pi, ExpressionError& error) {
    Parser parser(body, lookup, version_supports_pi, error);
    std::optional<Value> value = parser.parse();
    if (!value.has_value() && error.message.empty()) {
        error.message = "expression could not be evaluated";
        error.rule_id = kRuleEvaluation;
    }
    return value;
}

} // namespace scena::xml::detail
