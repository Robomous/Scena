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

#include <array>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "scena/dsl/ast.h"

namespace scena::dsl {

/// Index of a type in `Program::types`. Types are referred to by index rather
/// than by pointer: the program is built in passes and the vector grows, and a
/// stable small integer is also what keeps every ordering in this frontend
/// independent of where the allocator happened to put things.
using TypeId = std::uint32_t;

/// "No type" — an unresolved declarator, or a method with no return type.
inline constexpr TypeId kInvalidType = 0xFFFFFFFFU;

/// The eight SI base units of §7.3.4, plus the radian, in the order the
/// specification lists them.
inline constexpr std::array<const char*, 8> kSiBaseUnits = {"kg", "m",   "s",  "A",
                                                            "K",  "mol", "cd", "rad"};

/// The exponent vector of a physical quantity (§7.3.4).
///
/// Two physical types are dimensionally the same exactly when their vectors
/// are equal, and every unit of a physical type must carry that type's vector —
/// "All units of a physical type have identical exponents for all SI base units
/// and the radian", which is what makes direct conversion legal.
struct SiDimension {
    std::array<std::int64_t, 8> exponents{};

    [[nodiscard]] bool operator==(const SiDimension& other) const noexcept {
        return exponents == other.exponents;
    }
    [[nodiscard]] bool operator!=(const SiDimension& other) const noexcept {
        return !(*this == other);
    }
    [[nodiscard]] bool is_dimensionless() const noexcept {
        for (const std::int64_t exponent : exponents) {
            if (exponent != 0) {
                return false;
            }
        }
        return true;
    }

    /// A stable spelling for diagnostics, e.g. `m·s^-1`. Integer exponents
    /// only, so no locale-sensitive formatting reaches a message.
    [[nodiscard]] std::string to_string() const;
};

/// A declared unit (§7.3.4). `base_value = value * factor + offset`.
struct Unit {
    std::string name;
    TypeId physical_type = kInvalidType;
    SiDimension dimension;
    double factor = 1.0;
    double offset = 0.0;
    SourceRange range;
};

/// Converts a value expressed in `unit` to the base unit of its physical type,
/// per §7.3.4's `base_unit_value = unit_value * factor + offset`.
[[nodiscard]] inline double to_base_units(double value, const Unit& unit) noexcept {
    return value * unit.factor + unit.offset;
}

/// The inverse of to_base_units. Reported rather than approximated when the
/// factor is zero — a unit with a zero factor cannot be inverted.
[[nodiscard]] inline double from_base_units(double base_value, const Unit& unit) noexcept {
    return (base_value - unit.offset) / unit.factor;
}

/// One member of an enumeration type (§7.3.3). Values are unsigned; several
/// members may share one, in which case they compare equal.
struct EnumMemberInfo {
    std::string name;
    std::uint64_t value = 0;
    SourceRange range;
};

/// A resolved field (§7.3.6).
struct FieldInfo {
    std::string name;
    TypeId type = kInvalidType;
    /// `list of T` / `range of T` wrap the element type; `type` is then the
    /// aggregate and `element` the thing inside it.
    TypeId element = kInvalidType;
    bool is_variable = false;
    /// The type that declared it — an inherited field names its supertype.
    TypeId owner = kInvalidType;
    const Field* declaration = nullptr;
    SourceRange range;
};

/// A resolved method (§7.3.7).
struct MethodInfo {
    std::string name;
    std::vector<FieldInfo> parameters;
    TypeId return_type = kInvalidType;
    /// `expression`, `undefined` or `external`.
    std::string implementation;
    /// `only` when the method overrides (§7.3.7.2), otherwise empty.
    std::string qualifier;
    TypeId owner = kInvalidType;
    SourceRange range;
};

/// A resolved event declaration (§7.3.10.2).
struct EventInfo {
    std::string name;
    std::vector<FieldInfo> parameters;
    TypeId owner = kInvalidType;
    SourceRange range;
};

/// What a type is (§7.3.1).
enum class TypeKind : std::uint8_t {
    Bool,
    Int,
    Uint,
    Float,
    String,
    Enum,
    Physical,
    Struct,
    Actor,
    Scenario,
    Action,
    Modifier,
    List,
    Range,
};

/// True for `int`, `uint`, `float` and every physical type — §7.3.1's numeric
/// types, which are what a `range of T` may be built over.
[[nodiscard]] constexpr bool is_numeric(TypeKind kind) noexcept {
    return kind == TypeKind::Int || kind == TypeKind::Uint || kind == TypeKind::Float ||
           kind == TypeKind::Physical;
}

/// True for the compound types that may inherit, be extended, and hold members
/// (§7.3.5.1).
[[nodiscard]] constexpr bool is_structured(TypeKind kind) noexcept {
    return kind == TypeKind::Struct || kind == TypeKind::Actor || kind == TypeKind::Scenario ||
           kind == TypeKind::Action || kind == TypeKind::Modifier;
}

/// A resolved type.
///
/// Members live in `std::map`s keyed by name, and a parallel `*_order` vector
/// keeps declaration order for the places where order is meaning — positional
/// arguments (§7.2.2.5.2) and enumeration value succession (§7.3.3). Iterating
/// the map is therefore deterministic, and so is iterating the order vector;
/// nothing here depends on a hash seed or an address.
struct TypeInfo {
    TypeKind kind = TypeKind::Struct;
    /// The fully qualified name, `namespace::name`; the null namespace spells
    /// itself `::name`.
    std::string name;
    /// The name as written, without the namespace prefix.
    std::string simple_name;
    std::string name_space;

    // Physical types.
    SiDimension dimension;

    // Enumerations.
    std::vector<EnumMemberInfo> enum_members;

    // Aggregates.
    TypeId element = kInvalidType;

    // Structured types.
    TypeId base = kInvalidType;
    /// `inherits X(field == value)` (§7.3.8.2).
    bool is_conditional = false;
    std::string constraint_field;
    ExprPtr constraint_value;
    /// For a scenario, action or modifier declared as `actor.name`.
    std::string actor;
    TypeId actor_type = kInvalidType;
    /// A modifier's `of qualified-behavior-name` (§7.3.12.2).
    std::string modifies;
    TypeId modifies_type = kInvalidType;

    std::map<std::string, FieldInfo> fields;
    std::vector<std::string> field_order;
    std::map<std::string, MethodInfo> methods;
    std::vector<std::string> method_order;
    std::map<std::string, EventInfo> events;
    std::vector<std::string> event_order;

    /// Every declaration that contributed members, in textual order: the
    /// declaration itself first, then its extensions (§7.3.15).
    std::vector<const StructuredDecl*> declarations;

    SourceRange range;
};

/// A resolved global parameter (§7.3.14).
struct GlobalInfo {
    std::string name;
    FieldInfo field;
    SourceRange range;
};

/// A namespace's export list (§7.7.4).
struct NamespaceInfo {
    std::string name;
    /// Fully qualified names placed on the export list, in declaration order.
    std::vector<std::string> exports;
};

/// Everything one or more `.osc` files declare, after name resolution and the
/// §7.3 type rules.
///
/// Containers are ordered (`std::map`, `std::vector`) throughout. Load-time
/// values reach the IR, so the frontend is inside the determinism contract:
/// two runs over the same sources must produce the same program and the same
/// diagnostics in the same order.
struct Program {
    std::vector<TypeInfo> types;
    /// Fully qualified name → index into `types`.
    std::map<std::string, TypeId> types_by_name;
    /// Unit names form their own global namespace (§7.3.4).
    std::map<std::string, Unit> units;
    std::map<std::string, GlobalInfo> globals;
    std::vector<std::string> global_order;
    std::map<std::string, NamespaceInfo> namespaces;

    [[nodiscard]] const TypeInfo* find(std::string_view qualified_name) const;
    [[nodiscard]] const TypeInfo* at(TypeId id) const noexcept {
        return id < types.size() ? &types[id] : nullptr;
    }
    /// The field named `name` on `type` or on any of its supertypes.
    [[nodiscard]] const FieldInfo* find_field(TypeId type, std::string_view name) const;
    /// The method named `name` on `type` or on any of its supertypes.
    [[nodiscard]] const MethodInfo* find_method(TypeId type, std::string_view name) const;
    /// True when `derived` is `base` or inherits from it, conditionally or not.
    [[nodiscard]] bool is_derived_from(TypeId derived, TypeId base) const;
    /// The enumeration types declaring a member spelled `member`, in name
    /// order. More than one means the literal is overloaded (§7.3.3).
    [[nodiscard]] std::vector<TypeId> enums_declaring(std::string_view member) const;
};

} // namespace scena::dsl
