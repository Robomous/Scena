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
#include "scena/dsl/token.h"
#include "scena/status.h"

namespace scena::dsl {

/// Parses a token stream into an AST, per ASAM OpenSCENARIO DSL 2.2.0 §7.2.2.
///
/// Recursive descent, hand-written (ADR-0027). The grammar in §7.2.2 is written
/// "for explanatory and normative purposes, not necessarily for parser
/// implementation purposes" and says implementers "might want to eliminate left
/// recursions"; the relation, sum, term and postfix productions are left
/// recursive and are implemented here as loops, which recognises the same
/// language.
///
/// **Error recovery is part of the contract.** A parse error does not stop the
/// parse: the parser reports it, skips to the next synchronisation point — the
/// end of the current logical line, or the end of the current block — and
/// carries on. One run therefore reports many problems, which is what the
/// pillar's exit criteria ask for. The returned AST holds everything that did
/// parse, so a later pass can still say something useful about it.
///
/// Diagnostics cite the section they enforce. The DSL spec defines no
/// `asam.net:` rule ids at all, so `Diagnostic::rule_id` is always empty.
///
/// Returns Status::Ok when nothing was reported as an error, and
/// Status::ValidationError otherwise.
[[nodiscard]] Status parse(const std::vector<Token>& tokens, const std::string& file, File& out,
                           DiagnosticSink& sink);

/// Lexes and parses `source` in one call — the usual entry point.
///
/// A lexical error does not prevent parsing: the token stream is always well
/// formed, so the parse runs and both sets of diagnostics reach `sink`.
[[nodiscard]] Status parse_source(std::string_view source, const std::string& file, File& out,
                                  DiagnosticSink& sink);

/// Convenience overload for a source with no file name.
[[nodiscard]] Status parse_source(std::string_view source, File& out, DiagnosticSink& sink);

} // namespace scena::dsl
