// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <optional>
#include <string_view>

namespace scena::ir {

/// Comparison operator shared by every by-value condition, per ASAM
/// OpenSCENARIO XML 1.4.0 Rule enumeration (§ enum Rule): the six relational
/// operators used to compare quantitative values or signals.
///
/// The kernel accepts all six regardless of target version; which literals a
/// given OpenSCENARIO XML version admits is a frontend concern (per-version
/// gating lives in the XML lowering, not here).
enum class Rule {
    EqualTo,        ///< 'Equal to' operator.
    GreaterThan,    ///< 'Greater than' operator.
    LessThan,       ///< 'Less than' operator.
    GreaterOrEqual, ///< 'Greater or equal' operator.
    LessOrEqual,    ///< 'Less or equal' operator.
    NotEqualTo,     ///< 'Not equal' operator.
};

/// Applies `rule` to two scalars, left operand first: `compare(lhs, rule, rhs)`
/// evaluates `lhs <rule> rhs`. The comparison is exact IEEE-754 with no
/// tolerance — the standard defines none, and an epsilon would make the
/// result depend on operand magnitude and break the bit-identical contract.
///
/// NaN follows IEEE ordering, which the built-in operators already give:
/// EqualTo and every ordering rule are false when either operand is NaN,
/// NotEqualTo is true. A quiet NaN therefore never satisfies an equality or
/// an ordering test.
[[nodiscard]] bool compare(double lhs, Rule rule, double rhs);

/// Parses `text` as a whole-token scalar, locale-independently via
/// std::from_chars (never std::stod, which honors the C locale and would
/// differ across platforms — the project's #1 cross-platform parsing rule).
///
/// The entire token must be consumed: partial matches like "1.5x", locale
/// forms like "1,5", and the empty string are not scalars and return
/// nullopt. A single leading '+' is accepted (std::from_chars rejects it) so
/// that "+5" reads as 5.
[[nodiscard]] std::optional<double> parse_scalar(std::string_view text);

/// Compares a stored/actual value against a reference literal under `rule`,
/// the ParameterCondition / VariableCondition / UserDefinedValueCondition
/// semantics: "Less and greater operators will only be supported if the value
/// given as string can unambiguously be converted into a scalar value".
///
/// - Both operands parse as scalars ⇒ numeric comparison via compare().
/// - Otherwise EqualTo / NotEqualTo compare the raw bytes (string equality),
///   and every ordering rule is false (unsupported on non-scalars).
///
/// `lhs` is the stored/actual value, `rhs` the condition's reference literal —
/// the orientation matters for the ordering rules. No boolean coercion is
/// performed here: "true" and "1" are distinct byte strings (typed
/// declarations and coercion arrive with the typed-value work, p4-s3).
[[nodiscard]] bool compare_values(std::string_view lhs, Rule rule, std::string_view rhs);

} // namespace scena::ir
