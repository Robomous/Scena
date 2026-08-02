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

#include "scena/dsl/types.h"

#include <string>

namespace scena::dsl {

std::string SiDimension::to_string() const {
    // Integer exponents only, so no floating-point value ever reaches a
    // diagnostic message (the kernel's rule; std::to_string on a double is
    // locale-sensitive and its shortest form differs between libraries).
    std::string text;
    for (std::size_t i = 0; i < exponents.size(); ++i) {
        if (exponents[i] == 0) {
            continue;
        }
        if (!text.empty()) {
            text += "*";
        }
        text += kSiBaseUnits[i];
        if (exponents[i] != 1) {
            text += "^" + std::to_string(exponents[i]);
        }
    }
    return text.empty() ? "1" : text;
}

const TypeInfo* Program::find(std::string_view qualified_name) const {
    const auto it = types_by_name.find(std::string(qualified_name));
    return it == types_by_name.end() ? nullptr : &types[it->second];
}

const FieldInfo* Program::find_field(TypeId type, std::string_view name) const {
    const std::string key(name);
    for (TypeId current = type; current < types.size();) {
        const TypeInfo& info = types[current];
        const auto it = info.fields.find(key);
        if (it != info.fields.end()) {
            return &it->second;
        }
        if (info.base == kInvalidType || info.base == current) {
            break;
        }
        current = info.base;
    }
    return nullptr;
}

const MethodInfo* Program::find_method(TypeId type, std::string_view name) const {
    const std::string key(name);
    for (TypeId current = type; current < types.size();) {
        const TypeInfo& info = types[current];
        const auto it = info.methods.find(key);
        if (it != info.methods.end()) {
            return &it->second;
        }
        if (info.base == kInvalidType || info.base == current) {
            break;
        }
        current = info.base;
    }
    return nullptr;
}

bool Program::is_derived_from(TypeId derived, TypeId base) const {
    // Bounded by the type count: a cycle is reported during resolution, but
    // this query must terminate even on a program that has one.
    std::size_t steps = 0;
    for (TypeId current = derived; current < types.size() && steps <= types.size(); ++steps) {
        if (current == base) {
            return true;
        }
        const TypeId next = types[current].base;
        if (next == kInvalidType || next == current) {
            break;
        }
        current = next;
    }
    return false;
}

std::vector<TypeId> Program::enums_declaring(std::string_view member) const {
    std::vector<TypeId> found;
    // types_by_name is ordered, so the result is too — an "overloaded literal"
    // diagnostic names the candidates in the same order on every platform.
    for (const auto& [name, id] : types_by_name) {
        const TypeInfo& info = types[id];
        if (info.kind != TypeKind::Enum) {
            continue;
        }
        for (const EnumMemberInfo& enum_member : info.enum_members) {
            if (enum_member.name == member) {
                found.push_back(id);
                break;
            }
        }
    }
    return found;
}

} // namespace scena::dsl
