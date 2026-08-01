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

#pragma once

#include <functional>
#include <optional>
#include <string>
#include <string_view>

namespace scena::xml::detail {

/// The value types an OpenSCENARIO parameter or expression can have
/// (§ParameterType, §9.2.3). `dateTime` and `string` are carried but take
/// part in no arithmetic: "operators are exclusively supported for numerical
/// (int, unsignedInt, unsignedShort, and double) and boolean data types"
/// (rule asam.net:xosc:1.1.0:expressions.allowed_operators).
enum class ValueType {
    Integer,       ///< int (and the unsigned integer types, which widen to it).
    UnsignedInt,   ///< unsignedInt.
    UnsignedShort, ///< unsignedShort.
    Double,
    Boolean,
    String,
    DateTime,
};

/// True for the types arithmetic operators accept.
[[nodiscard]] constexpr bool is_numeric(ValueType type) noexcept {
    return type == ValueType::Integer || type == ValueType::UnsignedInt ||
           type == ValueType::UnsignedShort || type == ValueType::Double;
}

/// True for the integer types, whose arithmetic stays integral (§9.2.1: "the
/// result of such an operator has the same data type as its arguments").
[[nodiscard]] constexpr bool is_integral(ValueType type) noexcept {
    return type == ValueType::Integer || type == ValueType::UnsignedInt ||
           type == ValueType::UnsignedShort;
}

/// A typed expression value.
///
/// Integers are carried exactly in `integer` and doubles in `number`, so
/// integer arithmetic never round-trips through a double and loses a digit.
/// `text` holds the string and dateTime forms verbatim.
struct Value {
    ValueType type = ValueType::String;
    long long integer = 0;
    double number = 0.0;
    bool boolean = false;
    std::string text;

    /// The numeric value, whichever numeric type this is.
    [[nodiscard]] double as_double() const noexcept {
        return is_integral(type) ? static_cast<double>(integer) : number;
    }

    /// The value as the text an attribute reader parses: integers and
    /// doubles round-trip through the locale-independent shortest form,
    /// booleans as `true`/`false`.
    [[nodiscard]] std::string to_text() const;
};

/// Parses `text` as a value of `type`, locale-independently. Returns nullopt
/// when the text is not a value of that type.
[[nodiscard]] std::optional<Value> parse_typed(std::string_view text, ValueType type);

/// Maps a `parameterType` / `variableType` literal onto a ValueType.
[[nodiscard]] std::optional<ValueType> parse_value_type(std::string_view text);

/// Why an expression could not be evaluated: a message and the ASAM checker
/// rule it violates (empty when the standard defines none).
struct ExpressionError {
    std::string message;
    std::string rule_id;
};

/// Looks a parameter up by name (without the `$`), or returns nullopt when it
/// is not in scope.
using ParameterLookup = std::function<std::optional<Value>(std::string_view)>;

/// Evaluates an OpenSCENARIO expression body — the text between `${` and `}`
/// — per §9.2.
///
/// Supports exactly the operator whitelist of rule
/// `asam.net:xosc:1.1.0:expressions.allowed_operators` with the precedence of
/// §9.2.1, parameter references (`$name`), integer/double/boolean literals,
/// and bracketed sub-expressions. Type rules follow §9.2.3: integer
/// arithmetic stays integral, an integer widens to a double implicitly and
/// never the other way, and an arithmetic value in a boolean position (or a
/// boolean in an arithmetic one) is a type error.
///
/// **Determinism.** Expression results reach the Scenario IR, so every
/// transcendental goes through the deterministic math layer
/// (`runtime/detmath.h`) rather than libm: `sin`/`cos` directly, `tan` as
/// their quotient, `atan`/`asin`/`acos` through `det_atan2`. `sqrt` and the
/// remainder are IEEE-exact operations. `pow` is exact repeated
/// multiplication for an integer exponent and is *rejected* for a
/// non-integer one, which would need a platform `exp`/`log` pair and break
/// bit-identity across platforms.
///
/// `version_supports_pi` gates the `pi` constant, which the targeted
/// 1.0-1.3 range does not define.
[[nodiscard]] std::optional<Value> evaluate_expression(std::string_view body,
                                                       const ParameterLookup& lookup,
                                                       bool version_supports_pi,
                                                       ExpressionError& error);

} // namespace scena::xml::detail
