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
#include "scena/dsl/token.h"
#include "scena/status.h"

namespace scena::dsl {

/// Lexes ASAM OpenSCENARIO DSL 2.2.0 source into a token stream, per §7.2.1.
///
/// Hand-written rather than generated: see ADR-0027. The whole file is tokenized
/// in one pass and the tokens are returned together, because the layout rules
/// (§7.2.1.2, §7.2.1.4) are not local — whether a newline ends a logical line
/// depends on bracket depth and on whether the previous physical line ended in a
/// continuation, and DEDENT tokens are only known once the next line's
/// indentation is.
///
/// `source` must outlive the returned tokens: `Token::text` borrows from it.
///
/// Errors are *reported, not thrown*, and lexing continues. §7.2 gives no rule
/// ids (the DSL spec defines none at all), so diagnostics cite section numbers.
/// A file with an unterminated string still yields the tokens before it, which
/// is what lets the parser report more than one problem per run — the pillar's
/// exit criterion asks for actionable diagnostics, not for the first one.
///
/// Returns Status::Ok when nothing was reported as an error, and
/// Status::ValidationError otherwise. The token stream is always well formed:
/// it ends with exactly one EndOfFile and every Indent is matched by a Dedent.
[[nodiscard]] Status lex(std::string_view source, const std::string& file, std::vector<Token>& out,
                         DiagnosticSink& sink);

/// Convenience overload for a source with no file name (a string from a host).
[[nodiscard]] Status lex(std::string_view source, std::vector<Token>& out, DiagnosticSink& sink);

} // namespace scena::dsl
