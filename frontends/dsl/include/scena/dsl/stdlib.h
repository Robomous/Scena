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

#include <string>
#include <string_view>
#include <vector>

namespace scena::dsl {

/// The module reference of the types sub-module (§7.7.5.2.2).
inline constexpr std::string_view kStandardTypesModule = "osc.standard.types";
/// The module reference of the domain sub-module (§7.7.5.2.2).
inline constexpr std::string_view kStandardDomainModule = "osc.standard.domain";
/// The module reference of the complete library (§7.7.5.2.1).
inline constexpr std::string_view kStandardAllModule = "osc.standard.all";
/// The legacy auto-use module reference (§7.7.5.2.3).
inline constexpr std::string_view kStandardLegacyModule = "osc.standard";

/// True when `module_reference` names a bundled standard-library module.
///
/// §7.7.5.1.2 reserves every structured identifier starting with `osc` for the
/// standard, so an `osc.`-prefixed reference that is not one of the four known
/// modules is an error rather than a user module.
[[nodiscard]] bool is_standard_module(std::string_view module_reference);

/// True when `module_reference` starts with the reserved `osc` identifier
/// (§7.7.5.1.2), whether or not it names a module this release knows.
[[nodiscard]] bool is_reserved_module(std::string_view module_reference);

/// The sub-modules a standard-library module reference expands to, in import
/// order. `osc.standard.types` expands to itself; `osc.standard.all` and the
/// legacy `osc.standard` expand to the types sub-module followed by the domain
/// sub-module.
///
/// Empty when the reference is not a standard-library module.
[[nodiscard]] std::vector<std::string_view> standard_submodules(std::string_view module_reference);

/// True when importing `module_reference` also places `std` and `stdtypes` on
/// the use list of the null namespace — the legacy `import osc.standard` form
/// of §7.7.5.2.3, and only that form.
[[nodiscard]] bool standard_module_auto_uses(std::string_view module_reference);

/// The DSL source of a bundled sub-module, or an empty string when
/// `module_reference` does not name one.
///
/// The library is authored from the normative §8 text of ASAM OpenSCENARIO DSL
/// 2.2.0 (§8.16 states that the `types.osc` / `domain.osc` / `standard.osc`
/// files shipped with the standard are *non*-normative and that the document
/// itself is the normative part), and it is DSL source rather than hand-built
/// `TypeInfo`s so that it travels through the same lexer, parser and resolver
/// as user code.
///
/// The returned view is stable for the lifetime of the program.
[[nodiscard]] std::string_view standard_module_source(std::string_view module_reference);

/// The namespace a bundled sub-module declares (`stdtypes` or `std`).
[[nodiscard]] std::string_view standard_module_namespace(std::string_view module_reference);

} // namespace scena::dsl
