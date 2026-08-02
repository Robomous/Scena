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

#include <cstdint>
#include <string>
#include <vector>

#include "scena/diagnostic.h"
#include "scena/dsl/ast.h"
#include "scena/dsl/types.h"

namespace scena::dsl {

/// What is in scope for an expression (§7.4.1.1, §7.4.1.3).
struct ExpressionContext {
    /// The structured type whose members an unqualified name may reach, and
    /// what `it` refers to (§7.4.1.3: "the instance of a type in whose scope it
    /// occurs"). kInvalidType outside any type — a global parameter's default.
    TypeId self = kInvalidType;
    /// The namespace the expression was written in, for resolving type names in
    /// `is()`, `as()` and enum references.
    std::string name_space;
    /// The source file the expression was written in. A `Program` spans every
    /// file its root imported, so a line number alone does not locate a
    /// diagnostic — `scena-check` needs the file to point at.
    std::string file;
    /// The namespaces the enclosing `namespace ... use` clause makes visible.
    std::vector<std::string> uses;
    /// Names bound by the construct the expression sits in rather than by the
    /// type: a method's or an event's parameters. They are looked up before the
    /// enclosing type's fields.
    const std::vector<FieldInfo>* locals = nullptr;
    /// What `it` refers to when the construct rebinds it: inside a parameter's
    /// `with:` block it is the parameter, not the enclosing instance
    /// (§7.3.12.4's `keep(speed == 10kph)` equivalence). kInvalidType leaves
    /// §7.4.1.3's default — the enclosing instance.
    TypeId it_binding = kInvalidType;
};

/// A value produced by constant evaluation.
///
/// Only what a *constant context* can hold: literals, enum members, and lists
/// and ranges of those. §7.3.11's concrete-value resolution and §7.5's coverage
/// arguments both need exactly this much, and nothing here needs an
/// interpreter.
struct Value {
    enum class Kind : std::uint8_t { None, Bool, Int, Uint, Float, String, Enum, List, Range };

    Kind kind = Kind::None;
    bool boolean = false;
    std::int64_t integer = 0;
    std::uint64_t unsigned_integer = 0;
    /// A float, or a physical quantity already converted to its base unit
    /// (§7.3.4) so that two units of the same type compare directly.
    double number = 0.0;
    std::string text;
    /// The enum member's value, when `kind == Enum`; `type` names the enum.
    std::uint64_t enum_value = 0;
    /// The static type of the value.
    TypeId type = kInvalidType;
    /// List members, or a range's two bounds (lower first).
    std::vector<Value> items;

    [[nodiscard]] bool is_numeric() const noexcept {
        return kind == Kind::Int || kind == Kind::Uint || kind == Kind::Float;
    }
    /// The numeric value as a double, for a comparison that has already passed
    /// type checking.
    [[nodiscard]] double as_double() const noexcept;
};

/// Types an expression, per §7.4.
///
/// Takes the program by reference because typing may *create* a type: a list or
/// range constructor names a structural aggregate that no declarator had to
/// mention, and interning it is what makes `[c, c]` and a declared
/// `list of car` the same TypeId. Nothing else about the program is modified.
///
/// Returns the expression's type, or kInvalidType when it could not be typed —
/// in which case a diagnostic naming the clause was reported. A subexpression
/// that failed to type does not cause a second, derived complaint about its
/// parent: an untyped operand silently makes the parent untyped too, so one
/// mistake produces one message.
[[nodiscard]] TypeId type_of(Program& program, const Expr& expression,
                             const ExpressionContext& context, DiagnosticSink& sink);

/// Evaluates an expression in a constant context.
///
/// Returns false when the expression is not constant — it reads a field, calls
/// a method, or otherwise needs a runtime instance. That is not an error by
/// itself: it is what tells the constraint checker that a `keep` would need
/// search rather than a fixed value (ADR-0004). Nothing is reported here; the
/// caller decides.
[[nodiscard]] bool evaluate_constant(const Program& program, const Expr& expression,
                                     const ExpressionContext& context, Value& out);

/// The common type of two operands under §7.4.2.3.1's numeric conversion rules,
/// or kInvalidType when there is none. Physical types are never converted.
[[nodiscard]] TypeId common_numeric_type(const Program& program, TypeId left, TypeId right);

} // namespace scena::dsl
