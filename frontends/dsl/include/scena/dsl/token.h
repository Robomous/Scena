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
#include <string_view>

namespace scena::dsl {

/// Token kinds of ASAM OpenSCENARIO DSL 2.2.0 §7.2.1.5.
///
/// One kind per lexical category rather than one per keyword: §7.2.1.5.1 says
/// keywords "are only recognized as keywords in the places identified in the
/// grammar. This means that these identifiers should be treated as normal
/// identifier tokens in all other places." So the lexer emits `Identifier` for
/// every word and records *whether* it is a reserved word (`Token::keyword`);
/// deciding that a given occurrence is a keyword is the parser's job, where the
/// grammar position is known.
enum class TokenKind : std::uint8_t {
    /// End of input. Always the last token, exactly once.
    EndOfFile,
    /// End of a logical line (§7.2.1.2). Never emitted for a blank or
    /// comment-only line, nor inside brackets, nor after a line continuation.
    Newline,
    /// Indentation increased (§7.2.1.4).
    Indent,
    /// Indentation decreased. One per level closed.
    Dedent,
    /// An identifier, possibly a reserved word — see `Token::keyword`. Covers
    /// both the plain form and the `|...|` escaped form (§7.2.1.5.1).
    Identifier,
    /// A `uint` literal: decimal or `0x`-prefixed hexadecimal.
    UnsignedInteger,
    /// An `int` literal: a leading `-` followed by digits.
    Integer,
    /// A `float` literal, including `inf` and `nan`.
    Float,
    /// A string literal, short (`'`/`"`) or long (`'''`/`"""`).
    String,
    /// A number immediately followed by a unit name, with no whitespace
    /// between (§7.2.1.5.2 physical-literal).
    PhysicalLiteral,
    /// An operator or delimiter (§7.2.1.5.3 Table 5).
    Operator,
};

[[nodiscard]] std::string_view to_string(TokenKind kind) noexcept;

/// True when `text` is one of the §7.2.1.5.1 Table 4 reserved words.
///
/// Public because the parser needs the same answer the lexer recorded, and
/// because a diagnostic that says "expected an identifier, found the keyword
/// 'scenario'" is worth more than one that does not know the difference.
[[nodiscard]] bool is_reserved_word(std::string_view text) noexcept;

/// One token, with the source range it came from.
///
/// `text` borrows from the source buffer the lexer was given, which must
/// outlive the token stream — the same contract the XML frontend's reader has
/// with pugixml. `value` is populated only where lexing already had to do the
/// work of decoding: the unescaped body of a string, the identifier inside
/// `|...|`, and the unit name of a physical literal.
struct Token {
    TokenKind kind = TokenKind::EndOfFile;
    /// The exact source spelling, borrowed. Empty for Indent/Dedent/EndOfFile.
    std::string_view text;
    /// Decoded payload: a string literal's contents with escapes resolved, an
    /// escaped identifier's contents without the delimiters, or a physical
    /// literal's unit name. Empty otherwise.
    std::string value;
    /// The numeric part of a physical literal, or of a numeric literal.
    /// Meaningful for UnsignedInteger, Integer, Float and PhysicalLiteral.
    double number = 0.0;
    /// The exact integer value, for UnsignedInteger and Integer, where a
    /// double would lose precision above 2^53.
    std::uint64_t unsigned_value = 0;
    std::int64_t signed_value = 0;
    /// True when an Identifier token spells a reserved word (§7.2.1.5.1).
    /// Always false for the `|...|` form: delimiting is exactly how a source
    /// file names something that would otherwise be a keyword.
    bool keyword = false;
    /// 1-based source position of the token's first character.
    int line = 0;
    int column = 0;
};

} // namespace scena::dsl
