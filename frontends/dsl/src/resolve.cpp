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
// Name resolution and the §7.3 type rules of ASAM OpenSCENARIO DSL 2.2.0.
// Written from the specification text (ADR-0002); every check names the clause
// it enforces.
//

#include "scena/dsl/resolve.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace scena::dsl {
namespace {

/// The null namespace spells itself with an empty prefix, so `::name`.
/// §7.7.4.1: "For the null namespace, either the `null` namespace name or the
/// empty string, both followed by two colons, are valid prefixes."
constexpr std::string_view kNullNamespace = "";

[[nodiscard]] bool is_null_namespace(std::string_view name) {
    return name.empty() || name == "null";
}

[[nodiscard]] std::string qualify(std::string_view name_space, std::string_view simple) {
    std::string qualified;
    if (!is_null_namespace(name_space)) {
        qualified.append(name_space);
    }
    qualified.append("::");
    qualified.append(simple);
    return qualified;
}

/// Splits `ns::name` into its parts. Returns false when `text` is unprefixed.
[[nodiscard]] bool split_prefix(std::string_view text, std::string& name_space,
                                std::string& simple) {
    const std::size_t at = text.rfind("::");
    if (at == std::string_view::npos) {
        return false;
    }
    name_space = std::string(text.substr(0, at));
    simple = std::string(text.substr(at + 2));
    return true;
}

/// The part of a qualified name after the last `::`.
[[nodiscard]] std::string_view simple_name_of(std::string_view qualified) {
    const std::size_t at = qualified.rfind("::");
    return at == std::string_view::npos ? qualified : qualified.substr(at + 2);
}

/// The primitive types of §7.3.2, in the fixed order they are created in. Their
/// names are keywords rather than namespaced identifiers (§7.3.2), so they are
/// never qualified and never shadowed by a user declaration.
struct Primitive {
    std::string_view name;
    TypeKind kind;
};
constexpr std::array<Primitive, 5> kPrimitives = {
    Primitive{"bool", TypeKind::Bool}, Primitive{"int", TypeKind::Int},
    Primitive{"uint", TypeKind::Uint}, Primitive{"float", TypeKind::Float},
    Primitive{"string", TypeKind::String}};

[[nodiscard]] TypeKind kind_of(StructuredKind kind) {
    switch (kind) {
    case StructuredKind::Actor:
        return TypeKind::Actor;
    case StructuredKind::Scenario:
        return TypeKind::Scenario;
    case StructuredKind::Action:
        return TypeKind::Action;
    case StructuredKind::Modifier:
        return TypeKind::Modifier;
    case StructuredKind::Struct:
    case StructuredKind::Extension:
        break;
    }
    return TypeKind::Struct;
}

[[nodiscard]] std::string_view spelling_of(TypeKind kind) {
    switch (kind) {
    case TypeKind::Bool:
        return "bool";
    case TypeKind::Int:
        return "int";
    case TypeKind::Uint:
        return "uint";
    case TypeKind::Float:
        return "float";
    case TypeKind::String:
        return "string";
    case TypeKind::Enum:
        return "an enum";
    case TypeKind::Physical:
        return "a physical type";
    case TypeKind::Struct:
        return "a struct";
    case TypeKind::Actor:
        return "an actor";
    case TypeKind::Scenario:
        return "a scenario";
    case TypeKind::Action:
        return "an action";
    case TypeKind::Modifier:
        return "a modifier";
    case TypeKind::List:
        return "a list";
    case TypeKind::Range:
        return "a range";
    }
    return "a type";
}

/// A pending declaration, kept with the namespace and use list that were active
/// where it was written — §7.7.4 scopes both to the namespace statement, and
/// resolution happens in a later pass (§7.3.15).
struct Scope {
    std::string name_space;
    std::vector<std::string> uses;
};

struct PendingUnit {
    const UnitDecl* decl = nullptr;
    Scope scope;
};

struct PendingExtension {
    const StructuredDecl* structured = nullptr;
    const EnumDecl* enumeration = nullptr;
    Scope scope;
};

struct PendingStructured {
    TypeId id = kInvalidType;
    const StructuredDecl* decl = nullptr;
    Scope scope;
};

struct PendingGlobal {
    const GlobalParameterDecl* decl = nullptr;
    Scope scope;
};

struct PendingExport {
    const ExportDecl* decl = nullptr;
    Scope scope;
};

class Resolver {
public:
    Resolver(const std::vector<const File*>& files, Program& out, DiagnosticSink& sink)
        : files_(files), out_(out), sink_(sink) {}

    Status run();

private:
    // Reporting.
    void error(const SourceRange& at, std::string message);
    void warn(const SourceRange& at, std::string message);

    // Passes.
    void declare();
    void link();
    void check_members();

    // Declaration helpers.
    TypeId add_type(TypeKind kind, const Scope& scope, std::string_view simple_name,
                    const SourceRange& range);
    [[nodiscard]] TypeId lookup_type(std::string_view written, const Scope& scope,
                                     const SourceRange& at, std::string_view what);
    [[nodiscard]] TypeId lookup_declared(std::string_view written, const Scope& scope) const;
    [[nodiscard]] SiDimension dimension_of(const std::vector<SiExponent>& exponents,
                                           const SourceRange& at);

    // Linking helpers.
    void link_unit(const PendingUnit& pending);
    void link_structured(PendingStructured& pending);
    void link_enum_values(TypeInfo& type, const std::vector<const EnumDecl*>& declarations);
    void check_inheritance_cycles();

    // Member helpers. They take a TypeId rather than a TypeInfo& on purpose:
    // resolving a declarator can create a `list of T` type, which grows
    // Program::types and invalidates any reference held across the call.
    void add_members(const PendingStructured& pending);
    void add_field(TypeId id, const PendingStructured& pending, const Field& field);
    void add_method(TypeId id, const PendingStructured& pending, const MethodDecl& method);
    void add_event(TypeId id, const PendingStructured& pending, const EventDecl& event);
    void check_modifier_application(TypeId id, const PendingStructured& pending,
                                    const ModifierApplication& application);
    void check_literal_default(const FieldInfo& info, const Expr& value);
    [[nodiscard]] bool resolve_declarator(const TypeRef& declarator, const Scope& scope,
                                          TypeId& type, TypeId& element);
    [[nodiscard]] std::vector<FieldInfo> resolve_parameters(const std::vector<Argument>& parameters,
                                                            const Scope& scope, TypeId owner);
    void reject_it(const ExprPtr& expression, std::string_view where);

    const std::vector<const File*>& files_;
    Program& out_;
    DiagnosticSink& sink_;
    bool failed_ = false;

    std::vector<PendingUnit> units_;
    std::vector<PendingExtension> extensions_;
    std::vector<PendingStructured> structured_;
    std::vector<PendingGlobal> globals_;
    std::vector<PendingExport> exports_;
    /// The enum declarations contributing to a type, in textual order.
    std::map<TypeId, std::vector<const EnumDecl*>> enum_declarations_;
};

// --- reporting -------------------------------------------------------------

void Resolver::error(const SourceRange& at, std::string message) {
    Diagnostic diagnostic;
    diagnostic.severity = Severity::Error;
    diagnostic.code = Status::ValidationError;
    diagnostic.message = std::move(message);
    diagnostic.location.line = at.line;
    diagnostic.location.column = at.column;
    sink_.report(std::move(diagnostic));
    failed_ = true;
}

void Resolver::warn(const SourceRange& at, std::string message) {
    Diagnostic diagnostic;
    diagnostic.severity = Severity::Warning;
    diagnostic.code = Status::Ok;
    diagnostic.message = std::move(message);
    diagnostic.location.line = at.line;
    diagnostic.location.column = at.column;
    sink_.report(std::move(diagnostic));
}

// --- lookup ----------------------------------------------------------------

TypeId Resolver::add_type(TypeKind kind, const Scope& scope, std::string_view simple_name,
                          const SourceRange& range) {
    const std::string qualified = qualify(scope.name_space, simple_name);
    const auto existing = out_.types_by_name.find(qualified);
    if (existing != out_.types_by_name.end()) {
        // §7.3.9: features cannot shadow previously declared features; a second
        // declaration of the same name is a redeclaration, not an extension.
        error(range, "'" + qualified + "' is already declared (§7.3.9); use 'extend' to add to it");
        return existing->second;
    }
    TypeInfo info;
    info.kind = kind;
    info.name = qualified;
    info.simple_name = std::string(simple_name);
    info.name_space =
        is_null_namespace(scope.name_space) ? std::string(kNullNamespace) : scope.name_space;
    info.range = range;
    const auto id = static_cast<TypeId>(out_.types.size());
    out_.types.push_back(std::move(info));
    out_.types_by_name.emplace(qualified, id);
    return id;
}

TypeId Resolver::lookup_declared(std::string_view written, const Scope& scope) const {
    for (const Primitive& primitive : kPrimitives) {
        if (written == primitive.name) {
            return out_.types_by_name.at(std::string(primitive.name));
        }
    }

    std::string prefix;
    std::string simple;
    if (split_prefix(written, prefix, simple)) {
        // §7.7.4.1: an explicitly qualified identifier resolves in exactly one
        // place, whatever the use list says.
        const auto it = out_.types_by_name.find(qualify(prefix, simple));
        return it == out_.types_by_name.end() ? kInvalidType : it->second;
    }

    // §7.7.4.2 rule 2: the current namespace shadows everything reachable
    // through the use list.
    const auto local = out_.types_by_name.find(qualify(scope.name_space, written));
    if (local != out_.types_by_name.end()) {
        return local->second;
    }

    TypeId found = kInvalidType;
    for (const std::string& used : scope.uses) {
        const auto name_space = out_.namespaces.find(used);
        if (name_space == out_.namespaces.end()) {
            continue;
        }
        for (const std::string& exported : name_space->second.exports) {
            if (simple_name_of(exported) != written) {
                continue;
            }
            const auto candidate = out_.types_by_name.find(exported);
            if (candidate == out_.types_by_name.end() || candidate->second == found) {
                continue;
            }
            if (found != kInvalidType) {
                // §7.7.4.2: more than one same-named identifier reachable
                // through the use list is an error. Reported by the caller,
                // which has the range; here it is enough to say "not unique".
                return kInvalidType;
            }
            found = candidate->second;
        }
    }
    return found;
}

TypeId Resolver::lookup_type(std::string_view written, const Scope& scope, const SourceRange& at,
                             std::string_view what) {
    const TypeId id = lookup_declared(written, scope);
    if (id == kInvalidType) {
        error(at, "unknown " + std::string(what) + " '" + std::string(written) + "' (§7.7.4.2)");
    }
    return id;
}

SiDimension Resolver::dimension_of(const std::vector<SiExponent>& exponents,
                                   const SourceRange& at) {
    SiDimension dimension;
    for (const SiExponent& exponent : exponents) {
        // Compared by value: kSiBaseUnits holds pointers, and a std::find over
        // them would compare addresses.
        std::size_t index = kSiBaseUnits.size();
        for (std::size_t i = 0; i < kSiBaseUnits.size(); ++i) {
            if (exponent.unit == kSiBaseUnits[i]) {
                index = i;
                break;
            }
        }
        if (index == kSiBaseUnits.size()) {
            error(exponent.range.line != 0 ? exponent.range : at,
                  "'" + exponent.unit +
                      "' is not an SI base unit; §7.3.4 defines kg, m, s, A, K, mol, cd and rad");
            continue;
        }
        dimension.exponents[index] += exponent.exponent;
    }
    return dimension;
}

// --- pass 1: declare -------------------------------------------------------

void Resolver::declare() {
    for (const Primitive& primitive : kPrimitives) {
        TypeInfo info;
        info.kind = primitive.kind;
        info.name = std::string(primitive.name);
        info.simple_name = info.name;
        out_.types_by_name.emplace(info.name, static_cast<TypeId>(out_.types.size()));
        out_.types.push_back(std::move(info));
    }

    for (const File* file : files_) {
        // §7.7.4: "Each file processing starts in the implicit null namespace".
        Scope scope;
        for (const Declaration& declaration : file->declarations) {
            switch (declaration.kind) {
            case Declaration::Kind::Import:
                // Import resolution needs a file loader; p7-s5 wires it to the
                // standard library. The reference is recorded by the parser.
                break;
            case Declaration::Kind::Namespace: {
                scope.name_space = declaration.name_space.name;
                scope.uses = declaration.name_space.uses;
                if (!is_null_namespace(scope.name_space)) {
                    if (scope.name_space.rfind("std", 0) == 0) {
                        warn(declaration.range,
                             "namespace '" + scope.name_space +
                                 "' starts with 'std', which §7.7.4 reserves for the standard");
                    }
                    out_.namespaces[scope.name_space].name = scope.name_space;
                }
                break;
            }
            case Declaration::Kind::Export:
                exports_.push_back(PendingExport{&declaration.export_decl, scope});
                break;
            case Declaration::Kind::PhysicalType: {
                const TypeId id =
                    add_type(TypeKind::Physical, scope, declaration.physical_type.name,
                             declaration.physical_type.range);
                if (id != kInvalidType) {
                    out_.types[id].dimension = dimension_of(declaration.physical_type.exponents,
                                                            declaration.physical_type.range);
                }
                break;
            }
            case Declaration::Kind::Unit:
                units_.push_back(PendingUnit{&declaration.unit, scope});
                break;
            case Declaration::Kind::Enum: {
                if (declaration.enumeration.is_extension) {
                    extensions_.push_back(
                        PendingExtension{nullptr, &declaration.enumeration, scope});
                    break;
                }
                const TypeId id = add_type(TypeKind::Enum, scope, declaration.enumeration.name,
                                           declaration.enumeration.range);
                if (id != kInvalidType) {
                    enum_declarations_[id].push_back(&declaration.enumeration);
                }
                break;
            }
            case Declaration::Kind::Structured: {
                const StructuredDecl& structured = declaration.structured;
                if (structured.kind == StructuredKind::Extension) {
                    extensions_.push_back(PendingExtension{&structured, nullptr, scope});
                    break;
                }
                const TypeId id =
                    add_type(kind_of(structured.kind), scope, structured.name, structured.range);
                if (id == kInvalidType) {
                    break;
                }
                out_.types[id].declarations.push_back(&structured);
                // A scenario, action or modifier may be written as
                // `actor.name`; the actor prefix is part of the name and names
                // the type the behavior belongs to (§7.2.2.2.4, §7.3.12.2).
                const std::size_t dot = structured.name.find('.');
                if (dot != std::string::npos) {
                    out_.types[id].actor = structured.name.substr(0, dot);
                }
                out_.types[id].modifies = structured.modifies;
                structured_.push_back(PendingStructured{id, &structured, scope});
                break;
            }
            case Declaration::Kind::GlobalParameter:
                globals_.push_back(PendingGlobal{&declaration.global_parameter, scope});
                break;
            }
        }
    }

    // Extensions attach to what they extend. §7.3.9 is explicit that extension
    // modifies the type rather than declaring a new one, so this runs after
    // every declaration has been seen — a file may extend a type declared
    // later, or in another file (§7.3.15).
    for (const PendingExtension& extension : extensions_) {
        if (extension.enumeration != nullptr) {
            const TypeId id = lookup_declared(extension.enumeration->name, extension.scope);
            if (id == kInvalidType) {
                error(extension.enumeration->range,
                      "unknown enum '" + extension.enumeration->name + "' (§7.3.9)");
                continue;
            }
            if (out_.types[id].kind != TypeKind::Enum) {
                error(extension.enumeration->range,
                      "'" + extension.enumeration->name + "' is " +
                          std::string(spelling_of(out_.types[id].kind)) + ", not an enum (§7.3.9)");
                continue;
            }
            enum_declarations_[id].push_back(extension.enumeration);
            continue;
        }
        const StructuredDecl& structured = *extension.structured;
        const TypeId id = lookup_declared(structured.name, extension.scope);
        if (id == kInvalidType) {
            error(structured.range, "unknown type '" + structured.name + "' (§7.3.9)");
            continue;
        }
        if (!structured.base.empty()) {
            error(structured.range,
                  "an extension cannot introduce inheritance (§7.3.9); declare a new type that "
                  "inherits instead");
        }
        out_.types[id].declarations.push_back(&structured);
        structured_.push_back(PendingStructured{id, &structured, extension.scope});
    }

    // Export lists last: an export may name anything declared anywhere
    // (§7.7.4), including an identifier of another namespace.
    for (const PendingExport& pending : exports_) {
        NamespaceInfo& name_space = out_.namespaces[pending.scope.name_space];
        name_space.name = pending.scope.name_space;
        for (const std::string& written : pending.decl->names) {
            const bool is_wildcard =
                written == "*" ||
                (written.size() >= 3 && written.compare(written.size() - 3, 3, "::*") == 0);
            if (is_wildcard) {
                // §7.7.4.1: a wildcard exports every identifier of the named
                // namespace, wherever it was defined.
                const std::string target = written == "*" ? pending.scope.name_space
                                                          : written.substr(0, written.size() - 3);
                for (const auto& [name, id] : out_.types_by_name) {
                    (void)id;
                    std::string prefix;
                    std::string simple;
                    if (!split_prefix(name, prefix, simple)) {
                        continue; // a primitive; never namespaced
                    }
                    if (is_null_namespace(prefix) == is_null_namespace(target) &&
                        (is_null_namespace(target) || prefix == target)) {
                        name_space.exports.push_back(name);
                    }
                }
                continue;
            }
            std::string prefix;
            std::string simple;
            name_space.exports.push_back(split_prefix(written, prefix, simple)
                                             ? qualify(prefix, simple)
                                             : qualify(pending.scope.name_space, written));
        }
    }
}

// --- pass 2: link ----------------------------------------------------------

void Resolver::link_unit(const PendingUnit& pending) {
    const UnitDecl& decl = *pending.decl;
    if (out_.units.find(decl.name) != out_.units.end()) {
        // §7.3.4: "Unit names form their own separate global namespace. All
        // unit names must be globally unique within that namespace."
        error(decl.range, "unit '" + decl.name + "' is already declared (§7.3.4)");
        return;
    }
    Unit unit;
    unit.name = decl.name;
    unit.range = decl.range;
    unit.dimension = dimension_of(decl.exponents, decl.range);
    unit.factor = decl.factor.value_or(1.0);
    unit.offset = decl.offset.value_or(0.0);
    unit.physical_type =
        lookup_type(decl.physical_type, pending.scope, decl.range, "physical type");
    if (unit.physical_type != kInvalidType) {
        const TypeInfo& type = out_.types[unit.physical_type];
        if (type.kind != TypeKind::Physical) {
            error(decl.range, "'" + decl.physical_type + "' is " +
                                  std::string(spelling_of(type.kind)) +
                                  ", not a physical type (§7.3.4)");
        } else if (type.dimension != unit.dimension) {
            // §7.3.4: "All units of a physical type have identical exponents
            // for all SI base units and the radian" — that is what makes every
            // unit of a type directly convertible.
            error(decl.range, "unit '" + decl.name + "' has dimension " +
                                  unit.dimension.to_string() + " but '" + type.simple_name +
                                  "' is " + type.dimension.to_string() + " (§7.3.4)");
        }
    }
    out_.units.emplace(unit.name, std::move(unit));
}

void Resolver::link_enum_values(TypeInfo& type, const std::vector<const EnumDecl*>& declarations) {
    // §7.3.3: implicit values succeed the last explicit *literal* value, or 0.
    // A value given by reference to another member "is ignored for the purposes
    // of deriving succeeding implicit integer values".
    std::uint64_t next = 0;
    std::map<std::string, const EnumMember*> pending_references;
    for (const EnumDecl* declaration : declarations) {
        for (const EnumMember& member : declaration->members) {
            const bool duplicate =
                std::any_of(type.enum_members.begin(), type.enum_members.end(),
                            [&](const EnumMemberInfo& e) { return e.name == member.name; });
            if (duplicate) {
                error(member.range, "enum member '" + member.name + "' of '" + type.simple_name +
                                        "' is declared twice (§7.3.3)");
                continue;
            }
            EnumMemberInfo info;
            info.name = member.name;
            info.range = member.range;
            if (member.value.has_value()) {
                if (*member.value < 0) {
                    error(member.range, "enum member values are unsigned (§7.3.3)");
                }
                info.value = static_cast<std::uint64_t>(*member.value);
                next = info.value + 1;
            } else if (!member.value_reference.empty()) {
                pending_references.emplace(member.name, &member);
                info.value = 0;
            } else {
                info.value = next;
                ++next;
            }
            type.enum_members.push_back(std::move(info));
        }
    }

    // Resolve the reference form. A reference may name a member of this or
    // another enumeration; a cycle among them is an error (§7.3.3).
    std::set<std::string> resolving;
    std::set<std::string> resolved;
    const std::function<bool(const std::string&)> resolve_member =
        [&](const std::string& name) -> bool {
        const auto pending = pending_references.find(name);
        if (pending == pending_references.end() || resolved.count(name) != 0) {
            return true;
        }
        if (!resolving.insert(name).second) {
            return false;
        }
        const std::string& reference = pending->second->value_reference;
        const std::size_t bang = reference.find('!');
        const std::string member_name =
            bang == std::string::npos ? reference : reference.substr(bang + 1);
        const std::string enum_name =
            bang == std::string::npos ? std::string() : reference.substr(0, bang);
        bool ok = true;
        if (enum_name.empty() || enum_name == type.simple_name || enum_name == type.name) {
            ok = resolve_member(member_name);
        }
        resolving.erase(name);
        if (!ok) {
            return false;
        }
        // Look the target up: same enum first, then any enum declaring it.
        const auto* target = static_cast<const EnumMemberInfo*>(nullptr);
        for (const EnumMemberInfo& candidate : type.enum_members) {
            if (candidate.name == member_name) {
                target = &candidate;
                break;
            }
        }
        if (target == nullptr) {
            for (const TypeId other : out_.enums_declaring(member_name)) {
                for (const EnumMemberInfo& candidate : out_.types[other].enum_members) {
                    if (candidate.name == member_name) {
                        target = &candidate;
                        break;
                    }
                }
                if (target != nullptr) {
                    break;
                }
            }
        }
        for (EnumMemberInfo& info : type.enum_members) {
            if (info.name != name) {
                continue;
            }
            if (target == nullptr) {
                error(pending->second->range, "unknown enum member '" + reference + "' (§7.3.3)");
            } else {
                info.value = target->value;
            }
        }
        resolved.insert(name);
        return true;
    };
    for (const auto& [name, member] : pending_references) {
        if (!resolve_member(name)) {
            error(member->range, "enum member value references form a cycle (§7.3.3)");
        }
    }
}

void Resolver::link_structured(PendingStructured& pending) {
    TypeInfo& type = out_.types[pending.id];
    const StructuredDecl& decl = *pending.decl;

    if (!type.actor.empty() && type.actor_type == kInvalidType) {
        type.actor_type = lookup_type(type.actor, pending.scope, decl.range, "actor");
        if (type.actor_type != kInvalidType &&
            out_.types[type.actor_type].kind != TypeKind::Actor) {
            error(decl.range, "'" + type.actor + "' is " +
                                  std::string(spelling_of(out_.types[type.actor_type].kind)) +
                                  ", not an actor (§7.3.12.2)");
            type.actor_type = kInvalidType;
        }
    }

    if (!type.modifies.empty() && type.modifies_type == kInvalidType) {
        type.modifies_type = lookup_type(type.modifies, pending.scope, decl.range, "behavior");
        if (type.modifies_type != kInvalidType) {
            const TypeKind kind = out_.types[type.modifies_type].kind;
            if (kind != TypeKind::Scenario && kind != TypeKind::Action) {
                // §7.3.12.2: `of <qualified-behavior-name>` names the scenario
                // or action the modifier may be applied to.
                error(decl.range, "a modifier's 'of' names a scenario or an action (§7.3.12.2); '" +
                                      type.modifies + "' is " + std::string(spelling_of(kind)));
                type.modifies_type = kInvalidType;
            }
        }
    }

    if (decl.base.empty()) {
        return;
    }
    if (type.base != kInvalidType) {
        error(decl.range, "'" + type.simple_name + "' already inherits from '" +
                              out_.types[type.base].simple_name +
                              "'; inheritance is single (§7.3.8.1)");
        return;
    }
    const TypeId base = lookup_type(decl.base, pending.scope, decl.range, "type");
    if (base == kInvalidType) {
        return;
    }
    const TypeInfo& base_info = out_.types[base];
    if (base_info.kind != type.kind) {
        error(decl.range, "'" + type.simple_name + "' is " + std::string(spelling_of(type.kind)) +
                              " and cannot inherit from '" + base_info.simple_name +
                              "', which is " + std::string(spelling_of(base_info.kind)) +
                              " (§7.3.8.1)");
        return;
    }
    // §7.3.8.1: a behavior belonging to an actor must inherit from a behavior
    // belonging to an actor; one that does not, from one that does not.
    if ((type.kind == TypeKind::Scenario || type.kind == TypeKind::Action) &&
        type.actor.empty() != base_info.actor.empty()) {
        error(decl.range,
              type.actor.empty()
                  ? "a scenario or action with no actor cannot inherit from one that has an "
                    "actor (§7.3.8.1)"
                  : "a scenario or action of an actor must inherit from one of the same or a more "
                    "general actor (§7.3.8.1)");
        return;
    }
    if (decl.constraint_field.empty() && base_info.is_conditional) {
        // §7.3.8.2.3, Rule 1: a conditional type cannot be inherited
        // unconditionally.
        error(decl.range, "'" + base_info.simple_name +
                              "' is a conditional subtype and cannot be inherited unconditionally "
                              "(§7.3.8.2.3)");
        return;
    }
    type.base = base;
    if (decl.constraint_field.empty()) {
        return;
    }
    type.is_conditional = true;
    type.constraint_field = decl.constraint_field;
    type.constraint_value = decl.constraint_value;
}

void Resolver::check_inheritance_cycles() {
    // §7.3.8.2.3: "Inheritance relationships form a tree". A cycle is broken as
    // well as reported, so every later query over the chain terminates.
    for (TypeId id = 0; id < out_.types.size(); ++id) {
        std::set<TypeId> seen{id};
        TypeId current = id;
        while (out_.types[current].base != kInvalidType) {
            const TypeId next = out_.types[current].base;
            if (!seen.insert(next).second) {
                error(out_.types[id].range,
                      "'" + out_.types[id].simple_name + "' inherits from itself (§7.3.8.1)");
                out_.types[current].base = kInvalidType;
                break;
            }
            current = next;
        }
    }
}

void Resolver::link() {
    for (const PendingUnit& unit : units_) {
        link_unit(unit);
    }
    for (const auto& [id, declarations] : enum_declarations_) {
        link_enum_values(out_.types[id], declarations);
    }
    for (PendingStructured& pending : structured_) {
        link_structured(pending);
    }
    check_inheritance_cycles();

    // The conditional-inheritance guard needs the base's fields, which are only
    // known after members are added; §7.3.8.2 checking therefore runs at the
    // end of pass 3.
}

// --- pass 3: members -------------------------------------------------------

bool Resolver::resolve_declarator(const TypeRef& declarator, const Scope& scope, TypeId& type,
                                  TypeId& element) {
    type = kInvalidType;
    element = kInvalidType;
    if (declarator.name.empty()) {
        return false;
    }
    const TypeId named = lookup_type(declarator.name, scope, declarator.range, "type");
    if (named == kInvalidType) {
        return false;
    }
    if (!declarator.is_list && !declarator.is_range) {
        type = named;
        return true;
    }
    element = named;
    const TypeKind element_kind = out_.types[named].kind;
    if (declarator.is_list && (element_kind == TypeKind::List || element_kind == TypeKind::Range)) {
        // §7.3.1: "The elements in a list can be any other type, except another
        // list type."
        error(declarator.range, "a list element cannot itself be a list (§7.3.1)");
        return false;
    }
    if (declarator.is_range && !is_numeric(element_kind)) {
        // §7.3.1: a range is "a closed interval over a numeric base type".
        error(declarator.range, "'range of' needs a numeric type (int, uint, float or a physical "
                                "type), not " +
                                    std::string(spelling_of(element_kind)) + " (§7.3.1)");
        return false;
    }
    // Aggregates are structural: one entry per element type, created on demand
    // and shared, so `list of int` is the same TypeId everywhere.
    const std::string name =
        (declarator.is_list ? "list of " : "range of ") + out_.types[named].name;
    const auto existing = out_.types_by_name.find(name);
    if (existing != out_.types_by_name.end()) {
        type = existing->second;
        return true;
    }
    TypeInfo info;
    info.kind = declarator.is_list ? TypeKind::List : TypeKind::Range;
    info.name = name;
    info.simple_name = name;
    info.element = named;
    type = static_cast<TypeId>(out_.types.size());
    out_.types.push_back(std::move(info));
    out_.types_by_name.emplace(name, type);
    return true;
}

void Resolver::check_literal_default(const FieldInfo& info, const Expr& value) {
    if (info.type == kInvalidType) {
        return;
    }
    const TypeInfo& declared = out_.types[info.type];
    if (value.kind == ExprKind::PhysicalLiteral) {
        const auto unit = out_.units.find(value.text);
        if (unit == out_.units.end()) {
            error(value.range, "unknown unit '" + value.text + "' (§7.3.4)");
            return;
        }
        if (declared.kind != TypeKind::Physical) {
            error(value.range, "'" + info.name + "' is " + std::string(spelling_of(declared.kind)) +
                                   " and takes no unit (§7.3.4)");
            return;
        }
        if (unit->second.dimension != declared.dimension) {
            error(value.range, "unit '" + value.text + "' is " +
                                   unit->second.dimension.to_string() + " but '" +
                                   declared.simple_name + "' is " + declared.dimension.to_string() +
                                   " (§7.3.4)");
        }
        return;
    }
    if (value.kind == ExprKind::EnumValue) {
        const TypeId named = out_.types_by_name.count(qualify(kNullNamespace, value.type_name)) != 0
                                 ? out_.types_by_name.at(qualify(kNullNamespace, value.type_name))
                                 : kInvalidType;
        if (named != kInvalidType && named != info.type) {
            error(value.range, "'" + info.name + "' is '" + declared.simple_name + "', not '" +
                                   value.type_name + "' (§7.3.3)");
        }
        return;
    }
    if (value.kind != ExprKind::Literal) {
        return; // general expression typing is p7-s4
    }
    if (declared.kind == TypeKind::Physical) {
        // §7.3.4: "A unit specification is a mandatory part of any physical
        // type literal."
        error(value.range, "'" + info.name +
                               "' is a physical type, so its value needs a unit "
                               "(§7.3.4)");
        return;
    }
    const bool ok = [&] {
        switch (value.literal_type) {
        case LiteralType::Bool:
            return declared.kind == TypeKind::Bool;
        case LiteralType::UnsignedInteger:
            // §7.3.2.3: int and uint convert implicitly to float.
            return declared.kind == TypeKind::Uint || declared.kind == TypeKind::Int ||
                   declared.kind == TypeKind::Float;
        case LiteralType::Integer:
            return declared.kind == TypeKind::Int || declared.kind == TypeKind::Float;
        case LiteralType::Float:
            return declared.kind == TypeKind::Float;
        case LiteralType::String:
            return declared.kind == TypeKind::String;
        }
        return true;
    }();
    if (!ok) {
        error(value.range, "'" + info.name + "' is " + std::string(spelling_of(declared.kind)) +
                               " and cannot hold this literal (§7.3.2)");
    }
}

std::vector<FieldInfo> Resolver::resolve_parameters(const std::vector<Argument>& parameters,
                                                    const Scope& scope, TypeId owner) {
    std::vector<FieldInfo> resolved;
    for (const Argument& parameter : parameters) {
        FieldInfo info;
        info.name = parameter.name;
        info.owner = owner;
        info.range = parameter.range;
        const bool duplicate =
            std::any_of(resolved.begin(), resolved.end(),
                        [&](const FieldInfo& other) { return other.name == info.name; });
        if (duplicate) {
            error(parameter.range, "parameter '" + info.name + "' is declared twice (§7.2.2.5.1)");
            continue;
        }
        // The parser records a parameter's declared type in the argument's
        // value slot as a Name expression; an argument-list-specification is
        // `name ':' type-declarator ['=' default]`.
        if (parameter.value != nullptr && parameter.value->type.has_value()) {
            TypeId element = kInvalidType;
            (void)resolve_declarator(*parameter.value->type, scope, info.type, element);
            info.element = element;
        }
        resolved.push_back(std::move(info));
    }
    return resolved;
}

void Resolver::add_field(TypeId id, const PendingStructured& pending, const Field& field) {
    TypeId field_type = kInvalidType;
    TypeId element = kInvalidType;
    (void)resolve_declarator(field.type, pending.scope, field_type, element);
    TypeInfo& type = out_.types[id];
    for (const std::string& name : field.names) {
        if (type.fields.count(name) != 0) {
            error(field.range,
                  "field '" + name + "' of '" + type.simple_name + "' is declared twice (§7.3.9)");
            continue;
        }
        if (type.base != kInvalidType && out_.find_field(type.base, name) != nullptr) {
            // §7.3.9: "Features in an extension cannot shadow previously
            // declared features"; the same holds for an inheriting type.
            error(field.range, "field '" + name + "' already exists in a supertype of '" +
                                   type.simple_name + "' and cannot be shadowed (§7.3.9)");
            continue;
        }
        FieldInfo info;
        info.name = name;
        info.type = field_type;
        info.element = element;
        info.is_variable = field.is_variable;
        info.owner = id;
        info.declaration = &field;
        info.range = field.range;
        if (field.default_value != nullptr) {
            check_literal_default(info, *field.default_value);
        }
        type.field_order.push_back(name);
        type.fields.emplace(name, std::move(info));
    }
}

void Resolver::add_method(TypeId id, const PendingStructured& pending, const MethodDecl& method) {
    const TypeId base = out_.types[id].base;
    const std::string owner_name = out_.types[id].simple_name;
    const MethodInfo* inherited =
        base == kInvalidType ? nullptr : out_.find_method(base, method.name);
    const bool is_override = method.qualifier == "only";
    if (is_override && inherited == nullptr) {
        // §7.3.7.2: `is only` overrides a method that a supertype declares.
        error(method.range, "'is only' overrides a supertype method, and '" + method.name +
                                "' does not exist in a supertype of '" + owner_name +
                                "' (§7.3.7.2)");
    }
    if (!is_override && out_.types[id].methods.count(method.name) != 0) {
        error(method.range,
              "method '" + method.name + "' of '" + owner_name + "' is declared twice (§7.3.9)");
        return;
    }
    MethodInfo info;
    info.name = method.name;
    info.implementation = method.implementation;
    info.qualifier = method.qualifier;
    info.owner = id;
    info.range = method.range;
    info.parameters = resolve_parameters(method.parameters, pending.scope, id);
    if (method.return_type.has_value()) {
        TypeId element = kInvalidType;
        (void)resolve_declarator(*method.return_type, pending.scope, info.return_type, element);
    }
    // Re-read after the declarator resolutions above, which may have grown the
    // type table.
    inherited = base == kInvalidType ? nullptr : out_.find_method(base, method.name);
    if (is_override && inherited != nullptr && inherited->return_type != info.return_type) {
        error(method.range, "the override of '" + method.name +
                                "' must keep the supertype's return type (§7.3.7.2)");
    }
    TypeInfo& type = out_.types[id];
    if (type.methods.count(method.name) == 0) {
        type.method_order.push_back(method.name);
    }
    type.methods.insert_or_assign(method.name, std::move(info));
}

void Resolver::add_event(TypeId id, const PendingStructured& pending, const EventDecl& event) {
    if (out_.types[id].events.count(event.name) != 0) {
        error(event.range, "event '" + event.name + "' of '" + out_.types[id].simple_name +
                               "' is declared twice (§7.3.10.2)");
        return;
    }
    EventInfo info;
    info.name = event.name;
    info.owner = id;
    info.range = event.range;
    info.parameters = resolve_parameters(event.parameters, pending.scope, id);
    TypeInfo& type = out_.types[id];
    type.event_order.push_back(event.name);
    type.events.emplace(event.name, std::move(info));
}

void Resolver::check_modifier_application(TypeId container, const PendingStructured& pending,
                                          const ModifierApplication& application) {
    const TypeId id = lookup_declared(application.name, pending.scope);
    if (id == kInvalidType) {
        error(application.range, "unknown modifier '" + application.name + "' (§7.3.12.4)");
        return;
    }
    const TypeInfo& modifier = out_.types[id];
    if (modifier.kind != TypeKind::Modifier) {
        error(application.range, "'" + application.name + "' is " +
                                     std::string(spelling_of(modifier.kind)) +
                                     ", not a modifier (§7.3.12.4)");
        return;
    }
    if (modifier.modifies_type != kInvalidType) {
        // §7.3.12.2: a scenario-associated modifier "can be applied only to an
        // invocation of the specified scenario or as a member of that
        // scenario".
        if (!out_.is_derived_from(container, modifier.modifies_type)) {
            error(application.range, "modifier '" + modifier.simple_name + "' belongs to '" +
                                         modifier.modifies +
                                         "' and cannot be applied here (§7.3.12.2)");
        }
    }
    for (const Argument& argument : application.arguments) {
        if (argument.name.empty()) {
            continue; // positional; §7.3.12.4 binds it by declaration order
        }
        if (out_.find_field(id, argument.name) == nullptr) {
            error(argument.range, "modifier '" + modifier.simple_name + "' has no parameter '" +
                                      argument.name + "' (§7.3.12.4)");
        }
    }
}

void Resolver::reject_it(const ExprPtr& expression, std::string_view where) {
    if (expression == nullptr) {
        return;
    }
    if (expression->kind == ExprKind::Name && expression->text == "it") {
        // §7.4.1.3: "`it` is a reference to the instance of a type in whose
        // scope it occurs" — outside a type there is no such instance.
        error(expression->range,
              "'it' has no instance to refer to " + std::string(where) + " (§7.4.1.3)");
        return;
    }
    for (const ExprPtr& operand : expression->operands) {
        reject_it(operand, where);
    }
    for (const Argument& argument : expression->arguments) {
        reject_it(argument.value, where);
    }
}

void Resolver::add_members(const PendingStructured& pending) {
    const StructuredDecl& decl = *pending.decl;
    int behaviors = 0;
    for (const Member& member : decl.members) {
        switch (member.kind) {
        case Member::Kind::Field:
            add_field(pending.id, pending, member.field);
            break;
        case Member::Kind::Method:
            add_method(pending.id, pending, member.method);
            break;
        case Member::Kind::Event:
            add_event(pending.id, pending, member.event);
            break;
        case Member::Kind::ModifierApplication:
            check_modifier_application(pending.id, pending, member.modifier);
            break;
        case Member::Kind::Behavior: {
            ++behaviors;
            const TypeKind kind = out_.types[pending.id].kind;
            if (kind != TypeKind::Scenario && kind != TypeKind::Action &&
                kind != TypeKind::Modifier) {
                // §7.3.5.1: only scenarios and actions (and compound modifiers,
                // §7.3.12.1.2) carry a behavior specification.
                error(member.range, std::string(spelling_of(kind)) +
                                        " has no behavior specification; 'do' belongs to a "
                                        "scenario, an action or a modifier (§7.3.5.1)");
            }
            break;
        }
        case Member::Kind::Constraint:
        case Member::Kind::Coverage:
        case Member::Kind::On:
            break;
        }
    }
    if (behaviors > 1) {
        // §7.3.8.1 / §7.3.9: "not more than one `do` directive can be
        // effectively present in any scenario or action".
        error(decl.range, "'" + out_.types[pending.id].simple_name +
                              "' has more than one 'do' directive (§7.3.5.1.3)");
    }
}

void Resolver::check_members() {
    for (const PendingStructured& pending : structured_) {
        add_members(pending);
    }

    // A second sweep counts `do` directives across every declaration of a type,
    // because an extension may add one to a type that already has one (§7.3.9).
    for (const auto& [name, id] : out_.types_by_name) {
        (void)name;
        const TypeInfo& type = out_.types[id];
        int behaviors = 0;
        for (const StructuredDecl* declaration : type.declarations) {
            for (const Member& member : declaration->members) {
                if (member.kind == Member::Kind::Behavior) {
                    ++behaviors;
                }
            }
        }
        if (behaviors > 1 && type.declarations.size() > 1) {
            error(type.declarations.back()->range,
                  "'" + type.simple_name +
                      "' ends up with more than one 'do' directive after extension (§7.3.9)");
        }
    }

    // Conditional-inheritance guards, now that the base's fields exist.
    for (const PendingStructured& pending : structured_) {
        const TypeInfo& type = out_.types[pending.id];
        if (!type.is_conditional || type.base == kInvalidType) {
            continue;
        }
        const FieldInfo* determinant = out_.find_field(type.base, type.constraint_field);
        if (determinant == nullptr) {
            error(pending.decl->range, "'" + out_.types[type.base].simple_name +
                                           "' has no field '" + type.constraint_field +
                                           "' (§7.3.8.2)");
            continue;
        }
        if (determinant->type == kInvalidType) {
            continue;
        }
        const TypeKind determinant_kind = out_.types[determinant->type].kind;
        if (determinant_kind != TypeKind::Bool && determinant_kind != TypeKind::Enum) {
            // §7.3.8.2: "subtypes that depend upon a value of a Boolean or
            // enumerated field in the base type".
            error(pending.decl->range,
                  "a conditional inheritance guard needs a bool or enum field; '" +
                      type.constraint_field + "' is " + std::string(spelling_of(determinant_kind)) +
                      " (§7.3.8.2)");
            continue;
        }
        const Expr* value = type.constraint_value.get();
        if (value == nullptr) {
            continue;
        }
        if (determinant_kind == TypeKind::Bool &&
            !(value->kind == ExprKind::Literal && value->literal_type == LiteralType::Bool)) {
            error(value->range, "the guard on a bool field takes 'true' or 'false' (§7.3.8.2)");
        }
        if (determinant_kind == TypeKind::Enum) {
            const TypeInfo& enumeration = out_.types[determinant->type];
            const std::string member = value->kind == ExprKind::EnumValue ? value->text
                                       : value->kind == ExprKind::Name    ? value->text
                                                                          : std::string();
            const bool known =
                std::any_of(enumeration.enum_members.begin(), enumeration.enum_members.end(),
                            [&](const EnumMemberInfo& e) { return e.name == member; });
            if (member.empty() || !known) {
                error(value->range, "'" + enumeration.simple_name + "' has no member '" + member +
                                        "' (§7.3.8.2)");
            }
        }
    }

    // Globals (§7.3.14). Their scope is the file, not a type, so `it` has
    // nothing to bind to.
    for (const PendingGlobal& pending : globals_) {
        const Field& field = pending.decl->field;
        TypeId type = kInvalidType;
        TypeId element = kInvalidType;
        (void)resolve_declarator(field.type, pending.scope, type, element);
        for (const std::string& name : field.names) {
            const std::string qualified = qualify(pending.scope.name_space, name);
            if (out_.globals.count(qualified) != 0) {
                error(field.range, "global '" + qualified + "' is already declared (§7.3.14)");
                continue;
            }
            GlobalInfo info;
            info.name = qualified;
            info.range = pending.decl->range;
            info.field.name = name;
            info.field.type = type;
            info.field.element = element;
            info.field.is_variable = field.is_variable;
            info.field.declaration = &field;
            info.field.range = field.range;
            if (field.default_value != nullptr) {
                reject_it(field.default_value, "in a global parameter");
                check_literal_default(info.field, *field.default_value);
            }
            out_.global_order.push_back(qualified);
            out_.globals.emplace(qualified, std::move(info));
        }
    }
}

Status Resolver::run() {
    declare();
    link();
    check_members();
    return failed_ ? Status::ValidationError : Status::Ok;
}

} // namespace

Status resolve(const std::vector<const File*>& files, Program& out, DiagnosticSink& sink) {
    out = Program{};
    Resolver resolver(files, out, sink);
    return resolver.run();
}

Status resolve(const File& file, Program& out, DiagnosticSink& sink) {
    const std::vector<const File*> files{&file};
    return resolve(files, out, sink);
}

std::string_view builtin_prelude() {
    // §7.3.4's basic physical types with their SI base units, plus the derived
    // types and units the movement domain is written in. Kept as source so it
    // travels through the same lexer, parser and resolver as user code.
    return R"(type mass is SI(kg: 1)
type length is SI(m: 1)
type time is SI(s: 1)
type electrical_current is SI(A: 1)
type temperature is SI(K: 1)
type amount_of_substance is SI(mol: 1)
type luminous_intensity is SI(cd: 1)
type angle is SI(rad: 1)

type speed is SI(m: 1, s: -1)
type acceleration is SI(m: 1, s: -2)
type jerk is SI(m: 1, s: -3)
type angular_rate is SI(rad: 1, s: -1)

unit kg of mass is SI(kg: 1)
unit g of mass is SI(kg: 1, factor: 0.001)
unit m of length is SI(m: 1)
unit km of length is SI(m: 1, factor: 1000.0)
unit cm of length is SI(m: 1, factor: 0.01)
unit mm of length is SI(m: 1, factor: 0.001)
unit foot of length is SI(m: 1, factor: 0.3048)
unit s of time is SI(s: 1)
unit ms of time is SI(s: 1, factor: 0.001)
unit min of time is SI(s: 1, factor: 60.0)
unit hour of time is SI(s: 1, factor: 3600.0)
unit A of electrical_current is SI(A: 1)
unit K of temperature is SI(K: 1)
unit celsius of temperature is SI(K: 1, factor: 1.0, offset: 273.15)
unit mol of amount_of_substance is SI(mol: 1)
unit cd of luminous_intensity is SI(cd: 1)
unit rad of angle is SI(rad: 1)
unit deg of angle is SI(rad: 1, factor: 0.0174532925199433)
unit mps of speed is SI(m: 1, s: -1)
unit kph of speed is SI(m: 1, s: -1, factor: 0.2777777777777778)
unit mpss of acceleration is SI(m: 1, s: -2)
unit mpsss of jerk is SI(m: 1, s: -3)
unit radps of angular_rate is SI(rad: 1, s: -1)
)";
}

} // namespace scena::dsl
