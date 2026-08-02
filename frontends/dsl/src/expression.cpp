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

//
// Expression typing and constant evaluation, per ASAM OpenSCENARIO DSL 2.2.0
// §7.4. Written from the specification text (ADR-0002).
//

#include "scena/dsl/expression.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <string_view>

namespace scena::dsl {
namespace {

/// The list member evaluation operators of §7.4.2.7.3, which bind `it` to the
/// current member inside their argument.
[[nodiscard]] bool is_member_evaluation(std::string_view name) {
    return name == "filter" || name == "first_index" || name == "count" || name == "has" ||
           name == "map";
}

/// Reports and returns kInvalidType, so a caller can `return fail(...)`.
///
/// `file` is threaded through rather than left to the caller because a
/// `Program` spans every file its root imported: a line number on its own does
/// not say which file it is a line of.
TypeId fail(DiagnosticSink& sink, std::string_view file, const SourceRange& at,
            std::string message) {
    Diagnostic diagnostic;
    diagnostic.severity = Severity::Error;
    diagnostic.code = Status::ValidationError;
    diagnostic.message = std::move(message);
    diagnostic.location.file = file;
    diagnostic.location.line = at.line;
    diagnostic.location.column = at.column;
    sink.report(std::move(diagnostic));
    return kInvalidType;
}

class Typer {
public:
    Typer(Program& program, const ExpressionContext& context, DiagnosticSink& sink)
        : program_(program), context_(context), sink_(sink) {}

    TypeId type_of(const Expr& expression, TypeId it_type);

private:
    [[nodiscard]] TypeId lookup_type(std::string_view written) const;
    [[nodiscard]] TypeId kind_id(TypeKind kind) const;
    [[nodiscard]] TypeKind kind_of(TypeId id) const {
        return id < program_.types.size() ? program_.types[id].kind : TypeKind::Struct;
    }
    [[nodiscard]] std::string spelling(TypeId id) const {
        return id < program_.types.size() ? program_.types[id].name : "?";
    }

    TypeId type_name(const Expr& expression, TypeId it_type);
    TypeId type_enum_value(const Expr& expression);
    TypeId type_unary(const Expr& expression, TypeId it_type);
    TypeId type_binary(const Expr& expression, TypeId it_type);
    TypeId type_postfix(const Expr& expression, TypeId it_type);
    TypeId type_call(const Expr& expression, TypeId it_type);
    TypeId type_list(const Expr& expression, TypeId it_type);
    TypeId type_range(const Expr& expression, TypeId it_type);
    [[nodiscard]] TypeId aggregate(TypeKind kind, TypeId element);

    Program& program_;
    const ExpressionContext& context_;
    DiagnosticSink& sink_;
};

TypeId Typer::kind_id(TypeKind kind) const {
    switch (kind) {
    case TypeKind::Bool:
        return program_.types_by_name.at("bool");
    case TypeKind::Int:
        return program_.types_by_name.at("int");
    case TypeKind::Uint:
        return program_.types_by_name.at("uint");
    case TypeKind::Float:
        return program_.types_by_name.at("float");
    case TypeKind::String:
        return program_.types_by_name.at("string");
    default:
        break;
    }
    return kInvalidType;
}

TypeId Typer::lookup_type(std::string_view written) const {
    const auto direct = program_.types_by_name.find(std::string(written));
    if (direct != program_.types_by_name.end()) {
        return direct->second;
    }
    if (written.find("::") != std::string_view::npos) {
        return kInvalidType;
    }
    const std::string local =
        (context_.name_space.empty() ? std::string("::") : context_.name_space + "::") +
        std::string(written);
    const auto in_scope = program_.types_by_name.find(local);
    if (in_scope != program_.types_by_name.end()) {
        return in_scope->second;
    }
    // §7.7.4.2: fall back to the use list, which the resolver has already
    // reduced to export lists of fully qualified names.
    TypeId found = kInvalidType;
    for (const std::string& used : context_.uses) {
        const auto name_space = program_.namespaces.find(used);
        if (name_space == program_.namespaces.end()) {
            continue;
        }
        for (const std::string& exported : name_space->second.exports) {
            const std::size_t at = exported.rfind("::");
            if (at == std::string::npos || exported.substr(at + 2) != written) {
                continue;
            }
            const auto candidate = program_.types_by_name.find(exported);
            if (candidate == program_.types_by_name.end() || candidate->second == found) {
                continue;
            }
            if (found != kInvalidType) {
                return kInvalidType; // ambiguous
            }
            found = candidate->second;
        }
    }
    return found;
}

TypeId Typer::aggregate(TypeKind kind, TypeId element) {
    if (element == kInvalidType) {
        return kInvalidType;
    }
    const std::string name =
        (kind == TypeKind::List ? "list of " : "range of ") + program_.types[element].name;
    const auto found = program_.types_by_name.find(name);
    if (found != program_.types_by_name.end()) {
        return found->second;
    }
    // A constructor may be the first place a `list of T` appears. Aggregates
    // are structural (ADR-0028), so interning it here is what makes it the same
    // type as a declared `list of T` — and, being a vector index, the id stays
    // valid while the table grows.
    TypeInfo info;
    info.kind = kind;
    info.name = name;
    info.simple_name = name;
    info.element = element;
    const auto id = static_cast<TypeId>(program_.types.size());
    program_.types.push_back(std::move(info));
    program_.types_by_name.emplace(name, id);
    return id;
}

TypeId Typer::type_name(const Expr& expression, TypeId it_type) {
    if (expression.text == "it") {
        if (it_type != kInvalidType) {
            return it_type; // inside a list member evaluation (§7.4.2.7.3)
        }
        if (context_.it_binding != kInvalidType) {
            return context_.it_binding; // a parameter's with: block
        }
        if (context_.self != kInvalidType) {
            return context_.self;
        }
        return fail(sink_, context_.file, expression.range,
                    "'it' has no instance to refer to here (§7.4.1.3)");
    }
    // A parameter of the method or event the expression belongs to, which the
    // enclosing type's fields do not shadow.
    if (context_.locals != nullptr) {
        for (const FieldInfo& local : *context_.locals) {
            if (local.name == expression.text) {
                return local.type;
            }
        }
    }
    // A field of the enclosing type (§7.4.1.1).
    if (context_.self != kInvalidType) {
        const FieldInfo* field = program_.find_field(context_.self, expression.text);
        if (field != nullptr) {
            return field->type;
        }
    }
    // A global parameter (§7.3.14).
    const std::string qualified =
        (context_.name_space.empty() ? std::string("::") : context_.name_space + "::") +
        expression.text;
    const auto global = program_.globals.find(qualified);
    if (global != program_.globals.end()) {
        return global->second.field.type;
    }
    const auto null_global = program_.globals.find("::" + expression.text);
    if (null_global != program_.globals.end()) {
        return null_global->second.field.type;
    }
    // An enumeration member used as a literal (§7.3.3). Overloaded across two
    // enumerations, it needs the `enum!member` form.
    const std::vector<TypeId> enums = program_.enums_declaring(expression.text);
    if (enums.size() == 1) {
        return enums.front();
    }
    if (enums.size() > 1) {
        std::string names;
        for (const TypeId id : enums) {
            names += (names.empty() ? "" : ", ") + program_.types[id].name;
        }
        return fail(sink_, context_.file, expression.range,
                    "'" + expression.text + "' is a member of more than one enum (" + names +
                        "); write the enum name (§7.3.3)");
    }
    // A type name, which is legal as the operand of nothing in an expression,
    // but naming it precisely beats "unknown name".
    if (lookup_type(expression.text) != kInvalidType) {
        return fail(sink_, context_.file, expression.range,
                    "'" + expression.text + "' is a type, not a value (§7.4.1.1)");
    }
    return fail(sink_, context_.file, expression.range,
                "unknown name '" + expression.text + "' (§7.4.1.1)");
}

TypeId Typer::type_enum_value(const Expr& expression) {
    const TypeId id = lookup_type(expression.type_name);
    if (id == kInvalidType) {
        return fail(sink_, context_.file, expression.range,
                    "unknown enum '" + expression.type_name + "' (§7.3.3)");
    }
    if (program_.types[id].kind != TypeKind::Enum) {
        return fail(sink_, context_.file, expression.range,
                    "'" + expression.type_name + "' is not an enum (§7.3.3)");
    }
    const TypeInfo& enumeration = program_.types[id];
    const bool known =
        std::any_of(enumeration.enum_members.begin(), enumeration.enum_members.end(),
                    [&](const EnumMemberInfo& member) { return member.name == expression.text; });
    if (!known) {
        return fail(sink_, context_.file, expression.range,
                    "'" + expression.type_name + "' has no member '" + expression.text +
                        "' (§7.3.3)");
    }
    return id;
}

TypeId Typer::type_unary(const Expr& expression, TypeId it_type) {
    const TypeId operand = type_of(*expression.operands[0], it_type);
    if (operand == kInvalidType) {
        return kInvalidType;
    }
    if (expression.text == "not") {
        if (kind_of(operand) != TypeKind::Bool) {
            return fail(sink_, context_.file, expression.range,
                        "'not' takes a bool, not " + spelling(operand) + " (§7.4.2.2)");
        }
        return operand;
    }
    // Unary minus: "the result type is the type of its operand, or int if the
    // operand is of type uint" (§7.4.2.3.1).
    if (!is_numeric(kind_of(operand))) {
        return fail(sink_, context_.file, expression.range,
                    "unary '-' takes a numeric type, not " + spelling(operand) + " (§7.4.2.3)");
    }
    return kind_of(operand) == TypeKind::Uint ? kind_id(TypeKind::Int) : operand;
}

TypeId Typer::type_binary(const Expr& expression, TypeId it_type) {
    const std::string& op = expression.text;
    const TypeId left = type_of(*expression.operands[0], it_type);
    const TypeId right = type_of(*expression.operands[1], it_type);
    if (left == kInvalidType || right == kInvalidType) {
        return kInvalidType;
    }

    if (op == "and" || op == "or" || op == "=>") {
        if (kind_of(left) != TypeKind::Bool || kind_of(right) != TypeKind::Bool) {
            return fail(sink_, context_.file, expression.range,
                        "'" + op + "' takes bool operands (§7.4.2.2)");
        }
        return left;
    }

    if (op == "in") {
        // §7.4.2.7.1 / §7.4.2.8.3: the right operand is a list or a range.
        const TypeKind container = kind_of(right);
        if (container != TypeKind::List && container != TypeKind::Range) {
            return fail(sink_, context_.file, expression.range,
                        "'in' takes a list or a range on the right (§7.4.2.4.1)");
        }
        const TypeId element = program_.types[right].element;
        if (element != kInvalidType && left != element &&
            common_numeric_type(program_, left, element) == kInvalidType &&
            !program_.is_derived_from(left, element)) {
            return fail(sink_, context_.file, expression.range,
                        "'in' compares " + spelling(left) + " against " + spelling(right) +
                            " (§7.4.2.4.1)");
        }
        return kind_id(TypeKind::Bool);
    }

    const bool is_relational =
        op == "==" || op == "!=" || op == "<" || op == "<=" || op == ">" || op == ">=";
    if (is_relational) {
        if (is_numeric(kind_of(left)) && is_numeric(kind_of(right))) {
            // §7.4.2.4.1: after conversion the two types must be identical, and
            // physical types never convert.
            if (common_numeric_type(program_, left, right) == kInvalidType) {
                return fail(sink_, context_.file, expression.range,
                            "cannot compare " + spelling(left) + " with " + spelling(right) +
                                " (§7.4.2.4.1)");
            }
            return kind_id(TypeKind::Bool);
        }
        if (op != "==" && op != "!=") {
            return fail(sink_, context_.file, expression.range,
                        "'" + op + "' applies to numeric expressions; " + spelling(left) +
                            " is not one (§7.4.2.4.2)");
        }
        // §7.4.2.4.2: "the type of the two sub-expressions must be identical or
        // one must inherit from the other".
        if (left != right && !program_.is_derived_from(left, right) &&
            !program_.is_derived_from(right, left)) {
            return fail(sink_, context_.file, expression.range,
                        "cannot compare " + spelling(left) + " with " + spelling(right) +
                            " (§7.4.2.4.2)");
        }
        return kind_id(TypeKind::Bool);
    }

    // Arithmetic (§7.4.2.3).
    if (!is_numeric(kind_of(left)) || !is_numeric(kind_of(right))) {
        return fail(sink_, context_.file, expression.range,
                    "'" + op + "' takes numeric operands (§7.4.2.3)");
    }
    if (kind_of(left) == TypeKind::Physical || kind_of(right) == TypeKind::Physical) {
        // §7.4.2.3.1: "Physical types are not converted." Addition and
        // subtraction therefore need the same physical type on both sides;
        // multiplication and division by a plain number keep the physical type,
        // and the general dimension algebra (m/s from m and s) is a p8 concern
        // — nothing in v0.0.1's concrete scenarios needs to invent a type that
        // was never declared.
        if (op == "+" || op == "-" || op == "%") {
            if (left != right) {
                return fail(sink_, context_.file, expression.range,
                            "'" + op +
                                "' needs the same physical type "
                                "on both sides; " +
                                spelling(left) + " and " + spelling(right) +
                                " differ (§7.4.2.3.1)");
            }
            return left;
        }
        if (kind_of(left) == TypeKind::Physical && kind_of(right) == TypeKind::Physical) {
            return fail(sink_, context_.file, expression.range,
                        "'" + op +
                            "' over two physical types would need a type that is not "
                            "declared; write the result's type (§7.4.2.3.1)");
        }
        return kind_of(left) == TypeKind::Physical ? left : right;
    }
    const TypeId common = common_numeric_type(program_, left, right);
    if (common == kInvalidType) {
        return fail(sink_, context_.file, expression.range,
                    "cannot apply '" + op + "' to " + spelling(left) + " and " + spelling(right) +
                        " (§7.4.2.3.1)");
    }
    return common;
}

TypeId Typer::type_postfix(const Expr& expression, TypeId it_type) {
    if (expression.kind == ExprKind::Cast || expression.kind == ExprKind::TypeTest) {
        const TypeId operand = type_of(*expression.operands[0], it_type);
        if (operand == kInvalidType) {
            return kInvalidType;
        }
        const TypeId target = lookup_type(expression.type_name);
        if (target == kInvalidType) {
            return fail(sink_, context_.file, expression.range,
                        "unknown type '" + expression.type_name + "' (§7.4.2.5)");
        }
        if (expression.kind == ExprKind::TypeTest) {
            return kind_id(TypeKind::Bool); // §7.4.2.5 yields a Boolean
        }
        // §7.4.2.6: `as` converts; between unrelated types it can never succeed.
        const bool related = program_.is_derived_from(target, operand) ||
                             program_.is_derived_from(operand, target) ||
                             (is_numeric(kind_of(operand)) && is_numeric(kind_of(target))) ||
                             kind_of(operand) == TypeKind::Enum ||
                             kind_of(target) == TypeKind::Enum;
        if (!related) {
            return fail(sink_, context_.file, expression.range,
                        "cannot convert " + spelling(operand) + " to " + spelling(target) +
                            " (§7.4.2.6)");
        }
        return target;
    }

    if (expression.kind == ExprKind::ElementAccess) {
        const TypeId container = type_of(*expression.operands[0], it_type);
        const TypeId index = type_of(*expression.operands[1], it_type);
        if (container == kInvalidType || index == kInvalidType) {
            return kInvalidType;
        }
        if (kind_of(container) != TypeKind::List) {
            return fail(sink_, context_.file, expression.range,
                        "indexing applies to a list, not " + spelling(container) + " (§7.4.2.7.2)");
        }
        const TypeKind index_kind = kind_of(index);
        if (index_kind != TypeKind::Int && index_kind != TypeKind::Uint) {
            return fail(sink_, context_.file, expression.range,
                        "a list index is an integer expression (§7.4.2.7.2)");
        }
        return program_.types[container].element;
    }

    // Field access (§7.4.2.1 / §7.4.1.1).
    const TypeId owner = type_of(*expression.operands[0], it_type);
    if (owner == kInvalidType) {
        return kInvalidType;
    }
    const FieldInfo* field = program_.find_field(owner, expression.text);
    if (field != nullptr) {
        return field->type;
    }
    if (program_.find_method(owner, expression.text) != nullptr) {
        return fail(sink_, context_.file, expression.range,
                    "'" + expression.text + "' is a method; call it (§7.4.2.1)");
    }
    return fail(sink_, context_.file, expression.range,
                spelling(owner) + " has no field '" + expression.text + "' (§7.4.1.1)");
}

TypeId Typer::type_call(const Expr& expression, TypeId it_type) {
    const Expr& callee = *expression.operands[0];
    if (callee.kind != ExprKind::FieldAccess && callee.kind != ExprKind::Name) {
        return fail(sink_, context_.file, expression.range,
                    "this expression cannot be called (§7.4.2.1)");
    }

    // Built-in list and range operators are methods on the aggregate, not on a
    // declared type (§7.4.2.7.2–.3, §7.4.2.8.4).
    if (callee.kind == ExprKind::FieldAccess) {
        const TypeId owner = type_of(*callee.operands[0], it_type);
        if (owner == kInvalidType) {
            return kInvalidType;
        }
        const TypeKind owner_kind = kind_of(owner);
        const std::string& name = callee.text;
        if (owner_kind == TypeKind::List) {
            const TypeId element = program_.types[owner].element;
            if (name == "size") {
                return kind_id(TypeKind::Uint);
            }
            if (is_member_evaluation(name)) {
                if (expression.arguments.size() != 1) {
                    return fail(sink_, context_.file, expression.range,
                                "'" + name + "' takes one expression (§7.4.2.7.3)");
                }
                // `it` is the current member inside the argument (§7.4.2.7.3).
                const TypeId argument = type_of(*expression.arguments[0].value, element);
                if (argument == kInvalidType) {
                    return kInvalidType;
                }
                if (name == "map") {
                    return aggregate(TypeKind::List, argument);
                }
                if (kind_of(argument) != TypeKind::Bool) {
                    return fail(sink_, context_.file, expression.range,
                                "'" + name + "' takes a bool expression (§7.4.2.7.3)");
                }
                if (name == "filter") {
                    return owner;
                }
                return name == "first_index" ? kind_id(TypeKind::Int)
                       : name == "count"     ? kind_id(TypeKind::Uint)
                                             : kind_id(TypeKind::Bool);
            }
        }
        if (owner_kind == TypeKind::Range && (name == "lower" || name == "upper")) {
            return program_.types[owner].element; // §7.4.2.8.4
        }
        const MethodInfo* method = program_.find_method(owner, name);
        if (method == nullptr) {
            return fail(sink_, context_.file, expression.range,
                        spelling(owner) + " has no method '" + name + "' (§7.4.2.1)");
        }
        // Copied out before typing the arguments: that can intern an aggregate
        // and grow the type table, which would dangle the pointer.
        const TypeId returns = method->return_type;
        const std::size_t arity = method->parameters.size();
        if (expression.arguments.size() > arity) {
            return fail(sink_, context_.file, expression.range,
                        "'" + name + "' takes " + std::to_string(arity) + " arguments (§7.4.2.1)");
        }
        for (const Argument& argument : expression.arguments) {
            if (type_of(*argument.value, it_type) == kInvalidType) {
                return kInvalidType;
            }
        }
        return returns;
    }

    // An unqualified call: a method of the enclosing type, or `range(a, b)`.
    if (callee.text == "range") {
        return type_range(expression, it_type);
    }
    if (context_.self == kInvalidType) {
        return fail(sink_, context_.file, expression.range,
                    "unknown method '" + callee.text + "' (§7.4.2.1)");
    }
    const MethodInfo* method = program_.find_method(context_.self, callee.text);
    if (method == nullptr) {
        return fail(sink_, context_.file, expression.range,
                    spelling(context_.self) + " has no method '" + callee.text + "' (§7.4.2.1)");
    }
    const TypeId returns = method->return_type;
    for (const Argument& argument : expression.arguments) {
        if (type_of(*argument.value, it_type) == kInvalidType) {
            return kInvalidType;
        }
    }
    return returns;
}

TypeId Typer::type_list(const Expr& expression, TypeId it_type) {
    // §7.4.2.7.4's common-type rules.
    TypeId common = kInvalidType;
    bool all_numeric = true;
    for (const ExprPtr& member : expression.operands) {
        const TypeId member_type = type_of(*member, it_type);
        if (member_type == kInvalidType) {
            return kInvalidType;
        }
        all_numeric = all_numeric && is_numeric(kind_of(member_type));
        if (common == kInvalidType) {
            common = member_type;
            continue;
        }
        if (common == member_type) {
            continue;
        }
        if (all_numeric) {
            const TypeId numeric = common_numeric_type(program_, common, member_type);
            if (numeric != kInvalidType) {
                common = numeric;
                continue;
            }
        }
        // "the inheritance hierarchy is searched for the first base type that
        // all elements have in common".
        TypeId candidate = common;
        while (candidate != kInvalidType && !program_.is_derived_from(member_type, candidate)) {
            candidate = program_.types[candidate].base;
        }
        if (candidate == kInvalidType) {
            return fail(sink_, context_.file, expression.range,
                        "a list needs a common element type; " + spelling(common) + " and " +
                            spelling(member_type) + " have none (§7.4.2.7.4)");
        }
        common = candidate;
    }
    if (common == kInvalidType) {
        return fail(sink_, context_.file, expression.range,
                    "an empty list has no element type (§7.4.2.7.4)");
    }
    return aggregate(TypeKind::List, common);
}

TypeId Typer::type_range(const Expr& expression, TypeId it_type) {
    const std::vector<Argument>& arguments = expression.arguments;
    const Expr* low = nullptr;
    const Expr* high = nullptr;
    if (expression.kind == ExprKind::RangeConstructor && expression.operands.size() == 2) {
        low = expression.operands[0].get();
        high = expression.operands[1].get();
    } else if (arguments.size() == 2) {
        low = arguments[0].value.get();
        high = arguments[1].value.get();
    } else {
        return fail(sink_, context_.file, expression.range,
                    "a range takes two bounds (§7.4.2.8.1)");
    }
    const TypeId left = type_of(*low, it_type);
    const TypeId right = type_of(*high, it_type);
    if (left == kInvalidType || right == kInvalidType) {
        return kInvalidType;
    }
    if (!is_numeric(kind_of(left)) || !is_numeric(kind_of(right))) {
        return fail(sink_, context_.file, expression.range,
                    "a range is built over a numeric type (§7.4.2.8.1)");
    }
    const TypeId common = common_numeric_type(program_, left, right);
    if (common == kInvalidType) {
        // §7.4.2.8.1: "When constructing a range of a physical type, both
        // expressions must be of that physical type, with possibly different
        // units."
        return fail(sink_, context_.file, expression.range,
                    "a range needs one type for both bounds; " + spelling(left) + " and " +
                        spelling(right) + " differ (§7.4.2.8.1)");
    }
    return aggregate(TypeKind::Range, common);
}

TypeId Typer::type_of(const Expr& expression, TypeId it_type) {
    switch (expression.kind) {
    case ExprKind::Literal:
        switch (expression.literal_type) {
        case LiteralType::Bool:
            return kind_id(TypeKind::Bool);
        case LiteralType::UnsignedInteger:
            return kind_id(TypeKind::Uint);
        case LiteralType::Integer:
            return kind_id(TypeKind::Int);
        case LiteralType::Float:
            return kind_id(TypeKind::Float);
        case LiteralType::String:
            return kind_id(TypeKind::String);
        }
        return kInvalidType;
    case ExprKind::PhysicalLiteral: {
        const auto unit = program_.units.find(expression.text);
        if (unit == program_.units.end()) {
            return fail(sink_, context_.file, expression.range,
                        "unknown unit '" + expression.text + "' (§7.3.4)");
        }
        return unit->second.physical_type;
    }
    case ExprKind::Name:
        return type_name(expression, it_type);
    case ExprKind::EnumValue:
        return type_enum_value(expression);
    case ExprKind::Unary:
        return type_unary(expression, it_type);
    case ExprKind::Binary:
        return type_binary(expression, it_type);
    case ExprKind::Ternary: {
        // §7.4.2.9: the condition is a bool and the two arms share a type.
        const TypeId condition = type_of(*expression.operands[0], it_type);
        const TypeId when_true = type_of(*expression.operands[1], it_type);
        const TypeId when_false = type_of(*expression.operands[2], it_type);
        if (condition == kInvalidType || when_true == kInvalidType || when_false == kInvalidType) {
            return kInvalidType;
        }
        if (kind_of(condition) != TypeKind::Bool) {
            return fail(sink_, context_.file, expression.range,
                        "'?' takes a bool condition (§7.4.2.9)");
        }
        if (when_true == when_false) {
            return when_true;
        }
        const TypeId common = common_numeric_type(program_, when_true, when_false);
        if (common != kInvalidType) {
            return common;
        }
        if (program_.is_derived_from(when_true, when_false)) {
            return when_false;
        }
        if (program_.is_derived_from(when_false, when_true)) {
            return when_true;
        }
        return fail(sink_, context_.file, expression.range,
                    "the arms of '?' have no common type: " + spelling(when_true) + " and " +
                        spelling(when_false) + " (§7.4.2.9)");
    }
    case ExprKind::FieldAccess:
    case ExprKind::ElementAccess:
    case ExprKind::Cast:
    case ExprKind::TypeTest:
        return type_postfix(expression, it_type);
    case ExprKind::Call:
        return type_call(expression, it_type);
    case ExprKind::ListConstructor:
        return type_list(expression, it_type);
    case ExprKind::RangeConstructor:
        return type_range(expression, it_type);
    }
    return kInvalidType;
}

// --- constant evaluation ---------------------------------------------------

class Evaluator {
public:
    Evaluator(const Program& program, const ExpressionContext& context)
        : program_(program), context_(context) {}

    bool evaluate(const Expr& expression, Value& out);

private:
    [[nodiscard]] TypeKind kind_of(TypeId id) const {
        return id < program_.types.size() ? program_.types[id].kind : TypeKind::Struct;
    }
    bool evaluate_binary(const Expr& expression, Value& out);

    const Program& program_;
    const ExpressionContext& context_;
};

bool Evaluator::evaluate_binary(const Expr& expression, Value& out) {
    Value left;
    Value right;
    if (!evaluate(*expression.operands[0], left) || !evaluate(*expression.operands[1], right)) {
        return false;
    }
    const std::string& op = expression.text;
    out = Value{};

    if (left.kind == Value::Kind::Bool && right.kind == Value::Kind::Bool) {
        out.kind = Value::Kind::Bool;
        out.type = left.type;
        if (op == "and") {
            out.boolean = left.boolean && right.boolean;
            return true;
        }
        if (op == "or") {
            out.boolean = left.boolean || right.boolean;
            return true;
        }
        if (op == "=>") {
            out.boolean = !left.boolean || right.boolean;
            return true;
        }
        if (op == "==") {
            out.boolean = left.boolean == right.boolean;
            return true;
        }
        if (op == "!=") {
            out.boolean = left.boolean != right.boolean;
            return true;
        }
        return false;
    }

    if (left.kind == Value::Kind::Enum && right.kind == Value::Kind::Enum) {
        // §7.4.2.4.2: identical for an enum means the same unsigned value.
        if (op != "==" && op != "!=") {
            return false;
        }
        out.kind = Value::Kind::Bool;
        out.boolean = (left.enum_value == right.enum_value) == (op == "==");
        return true;
    }

    if (left.kind == Value::Kind::String && right.kind == Value::Kind::String) {
        if (op != "==" && op != "!=") {
            return false;
        }
        out.kind = Value::Kind::Bool;
        out.boolean = (left.text == right.text) == (op == "==");
        return true;
    }

    if (op == "in") {
        if (right.kind == Value::Kind::Range && right.items.size() == 2 && left.is_numeric()) {
            out.kind = Value::Kind::Bool;
            out.boolean = left.as_double() >= right.items[0].as_double() &&
                          left.as_double() <= right.items[1].as_double();
            return true;
        }
        if (right.kind == Value::Kind::List) {
            out.kind = Value::Kind::Bool;
            out.boolean = false;
            for (const Value& member : right.items) {
                const bool same = left.is_numeric() && member.is_numeric()
                                      ? left.as_double() == member.as_double()
                                  : left.kind == Value::Kind::String ? left.text == member.text
                                  : left.kind == Value::Kind::Enum
                                      ? left.enum_value == member.enum_value
                                      : false;
                if (same) {
                    out.boolean = true;
                    break;
                }
            }
            return true;
        }
        return false;
    }

    if (!left.is_numeric() || !right.is_numeric()) {
        return false;
    }
    const double a = left.as_double();
    const double b = right.as_double();
    if (op == "==" || op == "!=" || op == "<" || op == "<=" || op == ">" || op == ">=") {
        out.kind = Value::Kind::Bool;
        out.boolean = op == "=="   ? a == b
                      : op == "!=" ? a != b
                      : op == "<"  ? a < b
                      : op == "<=" ? a <= b
                      : op == ">"  ? a > b
                                   : a >= b;
        return true;
    }
    // Arithmetic. Integers stay integral, so a constraint over int values is
    // decided in integer arithmetic and not in a double that has rounded.
    const bool integral = left.kind != Value::Kind::Float && right.kind != Value::Kind::Float &&
                          left.type == right.type && kind_of(left.type) != TypeKind::Physical;
    if (integral && left.kind == Value::Kind::Int) {
        out.kind = Value::Kind::Int;
        out.type = left.type;
        if (op == "+") {
            out.integer = left.integer + right.integer;
        } else if (op == "-") {
            out.integer = left.integer - right.integer;
        } else if (op == "*") {
            out.integer = left.integer * right.integer;
        } else if (op == "/" || op == "%") {
            if (right.integer == 0) {
                return false; // reported by the caller as a failed evaluation
            }
            out.integer = op == "/" ? left.integer / right.integer : left.integer % right.integer;
        } else {
            return false;
        }
        return true;
    }
    out.kind = Value::Kind::Float;
    out.type = left.kind == Value::Kind::Float ? left.type : right.type;
    if (kind_of(left.type) == TypeKind::Physical) {
        out.type = left.type;
    } else if (kind_of(right.type) == TypeKind::Physical) {
        out.type = right.type;
    }
    if (op == "+") {
        out.number = a + b;
    } else if (op == "-") {
        out.number = a - b;
    } else if (op == "*") {
        out.number = a * b;
    } else if (op == "/") {
        if (b == 0.0) {
            return false;
        }
        out.number = a / b;
    } else if (op == "%") {
        if (b == 0.0) {
            return false;
        }
        out.number = std::fmod(a, b);
    } else {
        return false;
    }
    return true;
}

bool Evaluator::evaluate(const Expr& expression, Value& out) {
    switch (expression.kind) {
    case ExprKind::Literal:
        switch (expression.literal_type) {
        case LiteralType::Bool:
            out.kind = Value::Kind::Bool;
            out.boolean = expression.boolean;
            out.type = program_.types_by_name.at("bool");
            return true;
        case LiteralType::UnsignedInteger:
            out.kind = Value::Kind::Uint;
            out.unsigned_integer = expression.unsigned_value;
            out.type = program_.types_by_name.at("uint");
            return true;
        case LiteralType::Integer:
            out.kind = Value::Kind::Int;
            out.integer = expression.signed_value;
            out.type = program_.types_by_name.at("int");
            return true;
        case LiteralType::Float:
            out.kind = Value::Kind::Float;
            out.number = expression.number;
            out.type = program_.types_by_name.at("float");
            return true;
        case LiteralType::String:
            out.kind = Value::Kind::String;
            out.text = expression.text;
            out.type = program_.types_by_name.at("string");
            return true;
        }
        return false;
    case ExprKind::PhysicalLiteral: {
        const auto unit = program_.units.find(expression.text);
        if (unit == program_.units.end()) {
            return false;
        }
        // Converted to the base unit here, so two literals of the same physical
        // type compare directly whatever units they were written in (§7.3.4).
        out.kind = Value::Kind::Float;
        out.number = to_base_units(expression.number, unit->second);
        out.type = unit->second.physical_type;
        return true;
    }
    case ExprKind::EnumValue:
    case ExprKind::Name: {
        std::string enum_name = expression.type_name;
        const std::string& member_name = expression.text;
        TypeId id = kInvalidType;
        if (!enum_name.empty()) {
            const auto found = program_.types_by_name.find(
                enum_name.find("::") != std::string::npos
                    ? enum_name
                    : (context_.name_space.empty() ? "::" : context_.name_space + "::") +
                          enum_name);
            id = found == program_.types_by_name.end() ? kInvalidType : found->second;
        } else {
            const std::vector<TypeId> enums = program_.enums_declaring(member_name);
            if (enums.size() != 1) {
                return false; // a field, or ambiguous: not a constant
            }
            id = enums.front();
        }
        if (id == kInvalidType) {
            return false;
        }
        for (const EnumMemberInfo& member : program_.types[id].enum_members) {
            if (member.name != member_name) {
                continue;
            }
            out.kind = Value::Kind::Enum;
            out.enum_value = member.value;
            out.type = id;
            return true;
        }
        return false;
    }
    case ExprKind::Unary: {
        Value operand;
        if (!evaluate(*expression.operands[0], operand)) {
            return false;
        }
        if (expression.text == "not") {
            if (operand.kind != Value::Kind::Bool) {
                return false;
            }
            out = operand;
            out.boolean = !operand.boolean;
            return true;
        }
        out = operand;
        if (operand.kind == Value::Kind::Float) {
            out.number = -operand.number;
        } else if (operand.kind == Value::Kind::Int) {
            out.integer = -operand.integer;
        } else if (operand.kind == Value::Kind::Uint) {
            out.kind = Value::Kind::Int;
            out.integer = -static_cast<std::int64_t>(operand.unsigned_integer);
            out.type = program_.types_by_name.at("int");
        } else {
            return false;
        }
        return true;
    }
    case ExprKind::Binary:
        return evaluate_binary(expression, out);
    case ExprKind::Ternary: {
        Value condition;
        if (!evaluate(*expression.operands[0], condition) || condition.kind != Value::Kind::Bool) {
            return false;
        }
        // Short-circuiting (§7.4.2.2): only the taken arm is evaluated.
        return evaluate(*expression.operands[condition.boolean ? 1 : 2], out);
    }
    case ExprKind::ListConstructor: {
        out.kind = Value::Kind::List;
        for (const ExprPtr& member : expression.operands) {
            Value value;
            if (!evaluate(*member, value)) {
                return false;
            }
            if (value.kind == Value::Kind::List) {
                // §7.4.2.7.4: a list member that is itself a list is flattened.
                for (Value& nested : value.items) {
                    out.items.push_back(std::move(nested));
                }
                continue;
            }
            out.items.push_back(std::move(value));
        }
        return true;
    }
    case ExprKind::RangeConstructor: {
        const Expr* low = nullptr;
        const Expr* high = nullptr;
        if (expression.operands.size() == 2) {
            low = expression.operands[0].get();
            high = expression.operands[1].get();
        } else if (expression.arguments.size() == 2) {
            low = expression.arguments[0].value.get();
            high = expression.arguments[1].value.get();
        } else {
            return false;
        }
        Value lower;
        Value upper;
        if (!evaluate(*low, lower) || !evaluate(*high, upper)) {
            return false;
        }
        // §7.4.2.8.1: "The expression with the lower numeric value will provide
        // the lower bound" — either order is allowed in the source.
        out.kind = Value::Kind::Range;
        if (lower.as_double() <= upper.as_double()) {
            out.items = {lower, upper};
        } else {
            out.items = {upper, lower};
        }
        out.type = lower.type;
        return true;
    }
    default:
        break;
    }
    // A field read, a method call, a cast, an index: not constant.
    return false;
}

} // namespace

double Value::as_double() const noexcept {
    switch (kind) {
    case Kind::Int:
        return static_cast<double>(integer);
    case Kind::Uint:
        return static_cast<double>(unsigned_integer);
    case Kind::Float:
        return number;
    case Kind::Enum:
        return static_cast<double>(enum_value);
    default:
        break;
    }
    return 0.0;
}

TypeId common_numeric_type(const Program& program, TypeId left, TypeId right) {
    if (left == kInvalidType || right == kInvalidType) {
        return kInvalidType;
    }
    const TypeKind left_kind = program.types[left].kind;
    const TypeKind right_kind = program.types[right].kind;
    if (!is_numeric(left_kind) || !is_numeric(right_kind)) {
        return kInvalidType;
    }
    // §7.4.2.3.1, first rule: "Physical types are not converted."
    if (left_kind == TypeKind::Physical || right_kind == TypeKind::Physical) {
        return left == right ? left : kInvalidType;
    }
    if (left_kind == TypeKind::Float || right_kind == TypeKind::Float) {
        return program.types_by_name.at("float");
    }
    if (left_kind == right_kind) {
        return left;
    }
    // One int, one uint: the uint converts to int.
    return program.types_by_name.at("int");
}

TypeId type_of(Program& program, const Expr& expression, const ExpressionContext& context,
               DiagnosticSink& sink) {
    Typer typer(program, context, sink);
    return typer.type_of(expression, kInvalidType);
}

bool evaluate_constant(const Program& program, const Expr& expression,
                       const ExpressionContext& context, Value& out) {
    Evaluator evaluator(program, context);
    out = Value{};
    return evaluator.evaluate(expression, out);
}

} // namespace scena::dsl
