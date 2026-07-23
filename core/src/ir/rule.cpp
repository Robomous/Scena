// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#include "scena/ir/rule.h"

#include <array>
#include <charconv>
#include <string>
#include <system_error>

namespace scena::ir {

bool compare(double lhs, Rule rule, double rhs) {
    // The built-in IEEE comparisons already give the required NaN behavior:
    // every ordering test and EqualTo is false when an operand is NaN, and
    // NotEqualTo is true. No tolerance is applied — an epsilon would tie the
    // result to operand magnitude and break bit-identity.
    switch (rule) {
    case Rule::EqualTo:
        return lhs == rhs;
    case Rule::GreaterThan:
        return lhs > rhs;
    case Rule::LessThan:
        return lhs < rhs;
    case Rule::GreaterOrEqual:
        return lhs >= rhs;
    case Rule::LessOrEqual:
        return lhs <= rhs;
    case Rule::NotEqualTo:
        return lhs != rhs;
    }
    return false;
}

std::optional<double> parse_scalar(std::string_view text) {
    if (text.empty()) {
        return std::nullopt;
    }
    std::string_view token = text;
    // std::from_chars does not recognize a leading '+', but a scalar literal
    // may carry one. Strip exactly one, then reject a second sign so that
    // "+-5" and "++5" are not silently accepted.
    if (token.front() == '+') {
        token.remove_prefix(1);
        if (token.empty() || token.front() == '+' || token.front() == '-') {
            return std::nullopt;
        }
    }
    double value = 0.0;
    const char* const first = token.data();
    const char* const last = token.data() + token.size();
    const auto result = std::from_chars(first, last, value);
    // The whole token must be consumed: a trailing remainder ("1.5x") or any
    // parse error means the string is not unambiguously a scalar.
    if (result.ec != std::errc{} || result.ptr != last) {
        return std::nullopt;
    }
    return value;
}

std::string format_scalar(double value) {
    // 32 chars hold every shortest round-trip double ("-1.7976931348623157e+308"
    // is 24); the buffer is sized well past that so to_chars cannot fail with
    // value_too_large for any input.
    std::array<char, 64> buffer{};
    const std::to_chars_result result =
        std::to_chars(buffer.data(), buffer.data() + buffer.size(), value);
    if (result.ec != std::errc{}) {
        return {}; // unreachable for the buffer size above; no partial output
    }
    // The no-format overload emits the shortest representation that round-trips
    // (the Ryū / Grisu shortest form), which is what makes format_scalar the
    // exact inverse of parse_scalar.
    return std::string(buffer.data(), result.ptr);
}

bool compare_values(std::string_view lhs, Rule rule, std::string_view rhs) {
    const std::optional<double> lhs_scalar = parse_scalar(lhs);
    const std::optional<double> rhs_scalar = parse_scalar(rhs);
    if (lhs_scalar.has_value() && rhs_scalar.has_value()) {
        return compare(*lhs_scalar, rule, *rhs_scalar);
    }
    // Not unambiguously scalar on both sides: only equality and inequality
    // are defined, byte-for-byte; ordering rules are unsupported and false
    // (ParameterCondition / VariableCondition scalar-convertibility clause).
    switch (rule) {
    case Rule::EqualTo:
        return lhs == rhs;
    case Rule::NotEqualTo:
        return lhs != rhs;
    case Rule::GreaterThan:
    case Rule::LessThan:
    case Rule::GreaterOrEqual:
    case Rule::LessOrEqual:
        return false;
    }
    return false;
}

} // namespace scena::ir
