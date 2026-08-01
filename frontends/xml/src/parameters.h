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

#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <pugixml.hpp>

#include "expression.h"
#include "reader_context.h"

namespace scena::xml::detail {

/// The parameter scopes in effect at one point of a document (§9.1).
///
/// "The scope of a parameter is the subtree rooted in the element where the
/// ParameterDeclaration is located", and "if there are multiple parameters
/// with the same name and overlapping scopes, only the parameter with the
/// smallest scope that subsumes the location is accessible". A stack of
/// frames models exactly that: push when entering an element that declares
/// parameters, pop when leaving, and look up from the innermost frame
/// outwards.
class ParameterScope {
public:
    /// Declares a parameter in the innermost frame, shadowing any outer
    /// declaration of the same name.
    void declare(std::string name, Value value);

    /// The value of `name`, searching innermost frame first, or nullopt when
    /// it is not in scope.
    [[nodiscard]] std::optional<Value> find(std::string_view name) const;

    /// Every declaration of the outermost (global) frame, in name order —
    /// the ones a ParameterCondition can reference at run time.
    [[nodiscard]] const std::map<std::string, Value, std::less<>>& declared() const;

    /// Enters a new frame. Every `push` is matched by a `pop` — the RAII
    /// `ParameterFrame` below does that pairing.
    void push();
    void pop();

private:
    // Ordered map, never unordered: a scope is walked when reporting and the
    // report order must not depend on a hash seed.
    std::vector<std::map<std::string, Value, std::less<>>> frames_{1};
};

/// Scoped guard that pushes a parameter frame and pops it at end of scope.
class ParameterFrame {
public:
    explicit ParameterFrame(ParameterScope& scope) : scope_(scope) { scope_.push(); }
    ~ParameterFrame() { scope_.pop(); }
    ParameterFrame(const ParameterFrame&) = delete;
    ParameterFrame& operator=(const ParameterFrame&) = delete;

private:
    ParameterScope& scope_;
};

/// Reads a `ParameterDeclarations` element into the innermost frame of
/// `ctx`'s scope (§9.1): typed values, `$`-references and `${...}`
/// expressions resolved against the declarations already in scope, name
/// syntax and reserved-prefix rules, and `ConstraintGroup` validation.
void read_parameter_declarations(ReadContext& ctx, const pugi::xml_node& node);

/// Resolves an attribute's raw text: a whole-token `$name` reference, a
/// `${...}` expression, or a literal.
///
/// Returns nullopt after reporting when a reference is undeclared or an
/// expression cannot be evaluated, so a caller distinguishes "no such
/// attribute" (its own concern) from "the attribute is broken" (already
/// reported here).
[[nodiscard]] std::optional<std::string> resolve_attribute_text(ReadContext& ctx,
                                                                const pugi::xml_node& node,
                                                                const char* attribute,
                                                                std::string_view raw);

} // namespace scena::xml::detail
