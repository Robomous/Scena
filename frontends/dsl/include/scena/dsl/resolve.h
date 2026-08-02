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

#include "scena/diagnostic.h"
#include "scena/dsl/ast.h"
#include "scena/dsl/types.h"
#include "scena/status.h"

namespace scena::dsl {

/// Resolves names and applies the §7.3 type rules to a set of parsed files.
///
/// Three passes, because §7.3.15 puts no ordering restriction on declaration
/// and use — "there are no restrictions on the ordering of type use and type
/// declaration in terms of textual ordering", and the same for units:
///
/// 1. **Declare.** Every physical type, unit, enum, structured type and global
///    is entered into its namespace. Extensions are attached to the type they
///    extend rather than declaring a new one (§7.3.9).
/// 2. **Link.** Supertypes, conditional-inheritance guards, unit dimensions,
///    enumeration values, modifier association and the actor of a behavior are
///    resolved against the fully populated tables.
/// 3. **Members.** Fields, methods and events of every type are typed, checked
///    against what the supertype already declares, and — for a field with a
///    literal default — checked against the declared type.
///
/// General expression typing is p7-s4's job (#42); this pass types the
/// declarations, and checks a default value only when it is a literal, which is
/// where a unit or an enum name can already be got wrong.
///
/// Diagnostics cite the section they enforce. The DSL standard defines no
/// `asam.net:` rule ids, so `Diagnostic::rule_id` is always empty.
///
/// Like the parser, one call reports as much as it can: an unresolvable name
/// costs that declaration, not the file.
///
/// Returns Status::Ok when nothing was reported as an error, and
/// Status::ValidationError otherwise.
[[nodiscard]] Status resolve(const std::vector<const File*>& files, Program& out,
                             DiagnosticSink& sink);

/// Convenience overload for a single file.
[[nodiscard]] Status resolve(const File& file, Program& out, DiagnosticSink& sink);

} // namespace scena::dsl
