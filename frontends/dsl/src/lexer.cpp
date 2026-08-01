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

#include "scena/dsl/lexer.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

namespace scena::dsl {

namespace {

/// Table 4 of §7.2.1.5.1, sorted so a lookup is a binary search.
constexpr std::array<std::string_view, 74> kReservedWords{
    "K",         "SI",         "action",
    "actor",     "and",        "as",
    "bool",      "call",       "cd",
    "cover",     "def",        "default",
    "do",        "elapsed",    "emit",
    "enum",      "event",      "every",
    "export",    "expression", "extend",
    "external",  "factor",     "fall",
    "false",     "float",      "global",
    "hard",      "if",         "import",
    "in",        "inf",        "inherits",
    "int",       "is",         "it",
    "keep",      "kg",         "list",
    "m",         "modifier",   "mol",
    "namespace", "nan",        "not",
    "null",      "of",         "offset",
    "on",        "one_of",     "only",
    "or",        "parallel",   "rad",
    "range",     "record",     "remove_default",
    "rise",      "s",          "sample",
    "scenario",  "serial",     "string",
    "struct",    "true",       "type",
    "uint",      "undefined",  "unit",
    "until",     "use",        "var",
    "wait",      "with",
};

/// The multi-character operators of Table 5, longest first: §7.2.1.5 says
/// "tokens comprise the longest possible match that forms a legal token".
constexpr std::array<std::string_view, 8> kMultiCharOperators{
    "...", "->", "=>", "==", "!=", "<=", ">=", "::",
};

/// The single-character operators and delimiters of Table 5.
constexpr std::string_view kSingleCharOperators = ".,:=@()[]?<>+-*/%!";

// A mis-sorted table would answer binary searches wrongly and silently, so the
// order is a compile-time invariant rather than a convention.
static_assert(std::is_sorted(kReservedWords.begin(), kReservedWords.end()),
              "kReservedWords must stay sorted: is_reserved_word binary-searches it");

constexpr int kTabStop = 8; ///< §7.2.1.4: "tab stops are every 8 characters".

bool is_space(char c) noexcept {
    return c == ' ' || c == '\t';
}

bool is_digit(char c) noexcept {
    return c >= '0' && c <= '9';
}

bool is_hex_digit(char c) noexcept {
    return is_digit(c) || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
}

/// §7.2.1.5.1's `id-start-char`: the Unicode letter categories plus `_`.
///
/// The letter categories are approximated by "ASCII letter, underscore, or any
/// byte >= 0x80". A byte at or above 0x80 can only be a UTF-8 continuation or
/// lead byte, and the spec mandates UTF-8 (§7.2.1.1), so accepting them lets
/// identifiers in any script through without carrying a Unicode category table.
/// The cost is that a few non-letter code points (a currency symbol, say) would
/// be accepted where the spec would reject them; the `|...|` escaped form
/// exists precisely for characters outside the identifier set, so nothing is
/// lost, and rejecting valid non-Latin identifiers would be the worse error.
bool is_id_start(unsigned char c) noexcept {
    return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c == '_' || c >= 0x80;
}

/// §7.2.1.5.1's `id-char`: `id-start-char` plus the digit and mark categories.
bool is_id_char(unsigned char c) noexcept {
    return is_id_start(c) || is_digit(static_cast<char>(c));
}

/// Accumulates tokens and diagnostics for one source buffer.
class Lexer {
public:
    Lexer(std::string_view source, std::string file, std::vector<Token>& out, DiagnosticSink& sink)
        : source_(source), file_(std::move(file)), out_(out), sink_(sink) {}

    Status run();

private:
    // --- source cursor ---------------------------------------------------

    [[nodiscard]] bool at_end() const noexcept { return pos_ >= source_.size(); }
    [[nodiscard]] char peek(std::size_t ahead = 0) const noexcept {
        return pos_ + ahead < source_.size() ? source_[pos_ + ahead] : '\0';
    }
    [[nodiscard]] bool starts_with(std::string_view text) const noexcept {
        return source_.compare(pos_, text.size(), text) == 0;
    }
    char advance() noexcept {
        const char c = source_[pos_++];
        ++column_;
        return c;
    }

    // --- reporting -------------------------------------------------------

    void report(Severity severity, Status code, int line, int column, std::string message);
    void error(int line, int column, std::string message) {
        report(Severity::Error, Status::ValidationError, line, column, std::move(message));
        failed_ = true;
    }

    // --- pieces ----------------------------------------------------------

    /// Consumes the end-of-line sequence at the cursor (CR, LF or CRLF,
    /// §7.2.1.2). Returns false when the cursor is not at one.
    bool consume_end_of_line();
    /// Handles the start of a physical line: measures indentation and emits
    /// INDENT/DEDENT, or skips the line entirely when it is blank or a comment
    /// (§7.2.1.2, §7.2.1.4). Returns false when the file ended.
    bool begin_line();
    void emit_dedents_to(int column);
    void lex_token();
    void lex_identifier();
    void lex_escaped_identifier();
    void lex_number();
    void lex_string();
    void lex_operator();
    /// Reads a unit name directly after a numeric literal, if one is there
    /// (§7.2.1.5.2 physical-literal: no intervening whitespace).
    void attach_unit(Token& token, std::size_t number_begin);

    Token& push(TokenKind kind, std::size_t begin, int line, int column);

    std::string_view source_;
    std::string file_;
    std::vector<Token>& out_;
    DiagnosticSink& sink_;

    std::size_t pos_ = 0;
    int line_ = 1;
    int column_ = 1;
    /// Open `(` and `[` depth. Inside brackets a newline joins lines
    /// implicitly and produces no NEWLINE token (§7.2.1.2).
    int bracket_depth_ = 0;
    /// The indentation stack of §7.2.1.4, in the Python 3.10 §2.1.8 sense.
    /// Starts with the zero column that never pops.
    std::vector<int> indents_{0};
    /// True once a token has been emitted on the current logical line, which
    /// is what decides whether its end deserves a NEWLINE.
    bool line_has_token_ = false;
    bool failed_ = false;
};

void Lexer::report(Severity severity, Status code, int line, int column, std::string message) {
    Diagnostic diagnostic;
    diagnostic.severity = severity;
    diagnostic.code = code;
    diagnostic.message = std::move(message);
    diagnostic.location.file = file_;
    diagnostic.location.line = line;
    diagnostic.location.column = column;
    sink_.report(std::move(diagnostic));
}

Token& Lexer::push(TokenKind kind, std::size_t begin, int line, int column) {
    Token token;
    token.kind = kind;
    token.text = source_.substr(begin, pos_ - begin);
    token.line = line;
    token.column = column;
    out_.push_back(std::move(token));
    return out_.back();
}

bool Lexer::consume_end_of_line() {
    if (peek() == '\r') {
        ++pos_;
        if (peek() == '\n') {
            ++pos_; // CRLF is one end-of-line sequence, not two (§7.2.1.2)
        }
    } else if (peek() == '\n') {
        ++pos_;
    } else {
        return false;
    }
    ++line_;
    column_ = 1;
    return true;
}

void Lexer::emit_dedents_to(int column) {
    if (column >= indents_.back()) {
        return; // an indent, or the same level — neither closes a block
    }
    while (indents_.size() > 1 && indents_.back() > column) {
        indents_.pop_back();
        push(TokenKind::Dedent, pos_, line_, column_);
    }
    if (indents_.back() != column) {
        // Python 3.10 §2.1.8: a dedent must land on a column that is already on
        // the stack. Landing between two levels is ambiguous — the file does not
        // say which block it closes — so it is an error rather than a guess.
        error(line_, column_,
              "indentation does not match any enclosing block (§7.2.1.4); it must return to a "
              "column an outer line already used");
        indents_.push_back(column); // recover: treat it as its own level
    }
}

bool Lexer::begin_line() {
    while (!at_end()) {
        const std::size_t line_start = pos_;
        int width = 0;
        while (!at_end() && (is_space(peek()) || peek() == '\f')) {
            if (peek() == '\t') {
                // §7.2.1.4: tab stops every 8 columns.
                width = ((width / kTabStop) + 1) * kTabStop;
            } else if (peek() == ' ') {
                ++width;
            }
            // A formfeed at the start of a line is ignored for indentation.
            ++pos_;
            ++column_;
        }
        if (at_end()) {
            return false;
        }
        if (peek() == '#') {
            while (!at_end() && peek() != '\n' && peek() != '\r') {
                ++pos_;
            }
            consume_end_of_line();
            continue; // a comment-only line is ignored (§7.2.1.2)
        }
        if (peek() == '\n' || peek() == '\r') {
            consume_end_of_line();
            continue; // a blank line is ignored (§7.2.1.2)
        }
        (void)line_start;
        emit_dedents_to(width);
        if (width > indents_.back()) {
            indents_.push_back(width);
            push(TokenKind::Indent, pos_, line_, column_);
        }
        return true;
    }
    return false;
}

void Lexer::lex_string() {
    const int line = line_;
    const int column = column_;
    const std::size_t begin = pos_;
    const char quote = peek();
    // §7.2.1.5.2: a long string is the quote character three times.
    const bool is_long = peek(1) == quote && peek(2) == quote;
    const std::size_t delimiter = is_long ? 3 : 1;
    for (std::size_t i = 0; i < delimiter; ++i) {
        advance();
    }

    std::string value;
    bool terminated = false;
    while (!at_end()) {
        if (peek() == '\\') {
            // string-escape-seq ::= '\' any-char. The spec assigns no meanings
            // to the escapes beyond "backslash plus any character", so the
            // familiar ones are decoded and anything else keeps the character
            // itself — which is what "any-char" says and what makes \| and \#
            // work without inventing rules.
            advance();
            if (at_end()) {
                break;
            }
            const char escaped = advance();
            switch (escaped) {
            case 'n':
                value.push_back('\n');
                break;
            case 't':
                value.push_back('\t');
                break;
            case 'r':
                value.push_back('\r');
                break;
            case '0':
                value.push_back('\0');
                break;
            default:
                value.push_back(escaped);
                break;
            }
            continue;
        }
        if (!is_long && (peek() == '\n' || peek() == '\r')) {
            break; // shortstring-char excludes end-of-line characters
        }
        if (peek() == quote) {
            if (!is_long) {
                advance();
                terminated = true;
                break;
            }
            if (peek(1) == quote && peek(2) == quote) {
                advance();
                advance();
                advance();
                terminated = true;
                break;
            }
        }
        if (peek() == '\n') {
            // A long string may span physical lines; the line counter has to
            // follow or every later diagnostic points at the wrong place.
            ++line_;
            column_ = 0;
        }
        value.push_back(advance());
    }

    if (!terminated) {
        error(line, column,
              is_long ? "unterminated long string literal (§7.2.1.5.2)"
                      : "unterminated string literal (§7.2.1.5.2); a short string may not span "
                        "physical lines");
    }
    Token& token = push(TokenKind::String, begin, line, column);
    token.value = std::move(value);
}

void Lexer::lex_escaped_identifier() {
    const int line = line_;
    const int column = column_;
    const std::size_t begin = pos_;
    advance(); // the opening '|'
    std::string value;
    bool terminated = false;
    while (!at_end()) {
        if (peek() == '|') {
            advance();
            terminated = true;
            break;
        }
        if (peek() == '\n' || peek() == '\r') {
            break;
        }
        value.push_back(advance());
    }
    if (!terminated) {
        error(line, column, "unterminated escaped identifier (§7.2.1.5.1); expected a closing '|'");
    } else if (value.empty()) {
        // The production is '|' non-vertical-line-char+ '|' — at least one.
        error(line, column, "empty escaped identifier (§7.2.1.5.1); '||' names nothing");
    }
    Token& token = push(TokenKind::Identifier, begin, line, column);
    token.value = std::move(value);
    // Never a keyword: delimiting is exactly how a file names something that
    // would otherwise be reserved.
    token.keyword = false;
}

void Lexer::attach_unit(Token& token, std::size_t number_begin) {
    // §7.2.1.5.2: "A physical type literal is created when an identifier naming
    // a valid unit is included directly after a valid float or integer literal
    // without any intervening whitespace". Whether the name *is* a valid unit is
    // a type-system question (§7.3.4, p7-s3), so the lexer records the name and
    // the checker decides.
    if (at_end() || !is_id_start(static_cast<unsigned char>(peek()))) {
        return;
    }
    const std::size_t unit_begin = pos_;
    while (!at_end() && is_id_char(static_cast<unsigned char>(peek()))) {
        advance();
    }
    token.kind = TokenKind::PhysicalLiteral;
    token.value = std::string(source_.substr(unit_begin, pos_ - unit_begin));
    token.text = source_.substr(number_begin, pos_ - number_begin);
}

void Lexer::lex_number() {
    const int line = line_;
    const int column = column_;
    const std::size_t begin = pos_;

    // Hexadecimal: '0x' hex-digit+ (§7.2.1.5.2). Only unsigned.
    if (peek() == '0' && (peek(1) == 'x' || peek(1) == 'X') && is_hex_digit(peek(2))) {
        advance();
        advance();
        const std::size_t digits_begin = pos_;
        while (!at_end() && is_hex_digit(peek())) {
            advance();
        }
        std::uint64_t value = 0;
        const char* const first = source_.data() + digits_begin;
        const char* const last = source_.data() + pos_;
        const std::from_chars_result parsed = std::from_chars(first, last, value, 16);
        Token& token = push(TokenKind::UnsignedInteger, begin, line, column);
        if (parsed.ec != std::errc()) {
            error(line, column, "hexadecimal integer literal does not fit in uint (§7.2.1.5.2)");
        }
        token.unsigned_value = value;
        token.number = static_cast<double>(value);
        attach_unit(token, begin);
        return;
    }

    const bool negative = peek() == '-';
    if (negative) {
        advance();
    }
    while (!at_end() && is_digit(peek())) {
        advance();
    }
    bool is_float = false;
    if (peek() == '.' && is_digit(peek(1))) {
        is_float = true;
        advance();
        while (!at_end() && is_digit(peek())) {
            advance();
        }
    }
    if ((peek() == 'e' || peek() == 'E') &&
        (is_digit(peek(1)) || ((peek(1) == '+' || peek(1) == '-') && is_digit(peek(2))))) {
        is_float = true;
        advance();
        if (peek() == '+' || peek() == '-') {
            advance();
        }
        while (!at_end() && is_digit(peek())) {
            advance();
        }
    }

    const std::string_view text = source_.substr(begin, pos_ - begin);
    if (is_float) {
        double value = 0.0;
        // std::from_chars, never strtod: the locale must not decide what "1.5"
        // means. This is the same rule the XML frontend follows.
        const std::from_chars_result parsed =
            std::from_chars(text.data(), text.data() + text.size(), value);
        Token& token = push(TokenKind::Float, begin, line, column);
        if (parsed.ec != std::errc() || parsed.ptr != text.data() + text.size()) {
            error(line, column, "float literal cannot be represented as a float (§7.2.1.5.2)");
        }
        token.number = value;
        attach_unit(token, begin);
        return;
    }

    if (negative) {
        std::int64_t value = 0;
        const std::from_chars_result parsed =
            std::from_chars(text.data(), text.data() + text.size(), value);
        Token& token = push(TokenKind::Integer, begin, line, column);
        if (parsed.ec != std::errc()) {
            error(line, column, "integer literal cannot be represented as an int (§7.2.1.5.2)");
        }
        token.signed_value = value;
        token.number = static_cast<double>(value);
        attach_unit(token, begin);
        return;
    }

    std::uint64_t value = 0;
    const std::from_chars_result parsed =
        std::from_chars(text.data(), text.data() + text.size(), value);
    Token& token = push(TokenKind::UnsignedInteger, begin, line, column);
    if (parsed.ec != std::errc()) {
        error(line, column, "integer literal cannot be represented as a uint (§7.2.1.5.2)");
    }
    token.unsigned_value = value;
    token.number = static_cast<double>(value);
    attach_unit(token, begin);
}

void Lexer::lex_identifier() {
    const int line = line_;
    const int column = column_;
    const std::size_t begin = pos_;
    while (!at_end() && is_id_char(static_cast<unsigned char>(peek()))) {
        advance();
    }
    const std::string_view text = source_.substr(begin, pos_ - begin);

    // `inf` and `nan` are float literals (§7.2.1.5.2's float-literal
    // production lists them), and they are also in Table 4. The literal reading
    // wins: they can only ever denote those values.
    if (text == "inf" || text == "nan") {
        Token& token = push(TokenKind::Float, begin, line, column);
        token.number = text == "inf" ? std::numeric_limits<double>::infinity()
                                     : std::numeric_limits<double>::quiet_NaN();
        return;
    }

    Token& token = push(TokenKind::Identifier, begin, line, column);
    token.keyword = is_reserved_word(text);
}

void Lexer::lex_operator() {
    const int line = line_;
    const int column = column_;
    const std::size_t begin = pos_;
    for (const std::string_view op : kMultiCharOperators) {
        if (starts_with(op)) {
            for (std::size_t i = 0; i < op.size(); ++i) {
                advance();
            }
            push(TokenKind::Operator, begin, line, column);
            return;
        }
    }
    const char c = peek();
    if (c == '(' || c == '[') {
        ++bracket_depth_;
    } else if (c == ')' || c == ']') {
        bracket_depth_ = std::max(0, bracket_depth_ - 1);
    }
    advance();
    push(TokenKind::Operator, begin, line, column);
}

void Lexer::lex_token() {
    const char c = peek();
    if (c == '"' || c == '\'') {
        lex_string();
    } else if (c == '|') {
        lex_escaped_identifier();
    } else if (is_digit(c)) {
        lex_number();
    } else if (c == '-' && is_digit(peek(1))) {
        // int-literal ::= '-' digit+ (§7.2.1.5.2). A '-' followed by a digit is
        // a negative literal only where a value is expected; after an operand it
        // is subtraction. The lexer cannot know which, so it always produces the
        // literal and the parser re-splits it — the alternative, always emitting
        // an operator, would lose the spec's distinction between int and uint
        // literals, which §7.2.1.5.2 makes load-bearing for typing.
        lex_number();
    } else if (c == '.' && is_digit(peek(1))) {
        lex_number(); // float-literal ::= digit* '.' digit+ — the digits are optional
    } else if (is_id_start(static_cast<unsigned char>(c))) {
        lex_identifier();
    } else if (kSingleCharOperators.find(c) != std::string_view::npos || starts_with("->") ||
               starts_with("=>")) {
        lex_operator();
    } else {
        error(line_, column_,
              "unexpected character in the input (§7.2.1.5); it starts no legal token");
        advance();
        return;
    }
    line_has_token_ = true;
}

Status Lexer::run() {
    while (!at_end()) {
        if (bracket_depth_ == 0 || !line_has_token_) {
            if (!begin_line()) {
                break;
            }
        }
        line_has_token_ = false;
        // Where the logical line ends, captured before the end-of-line sequence
        // is consumed so the NEWLINE token does not point at the next line.
        int newline_line = line_;
        int newline_column = column_;

        // Tokens of one logical line.
        while (!at_end()) {
            if (is_space(peek()) || peek() == '\f') {
                advance();
                continue;
            }
            if (peek() == '#') {
                while (!at_end() && peek() != '\n' && peek() != '\r') {
                    ++pos_;
                }
                continue; // §7.2.1.3: a comment ends at the physical line's end
            }
            if (peek() == '\\' && (peek(1) == '\n' || peek(1) == '\r' ||
                                   (peek(1) == '\0' && pos_ + 1 >= source_.size()))) {
                // Explicit line joining (§7.2.1.2): the logical line continues.
                advance();
                if (!consume_end_of_line()) {
                    error(line_, column_,
                          "a line continuation must be the last character on its line "
                          "(§7.2.1.2)");
                }
                continue;
            }
            if (peek() == '\n' || peek() == '\r') {
                newline_line = line_;
                newline_column = column_;
                consume_end_of_line();
                if (bracket_depth_ > 0) {
                    continue; // implicit joining inside ( ) or [ ] (§7.2.1.2)
                }
                break;
            }
            lex_token();
        }

        if (line_has_token_) {
            push(TokenKind::Newline, pos_, newline_line, newline_column);
        }
    }

    // End of input is an implicit end-of-line (§7.2.1.2), so a file whose last
    // line has no terminator still ends that logical line.
    if (line_has_token_ && (out_.empty() || out_.back().kind != TokenKind::Newline)) {
        push(TokenKind::Newline, pos_, line_, column_);
    }
    if (bracket_depth_ > 0) {
        error(line_, column_, "unclosed '(' or '[' at end of input (§7.2.1.2)");
    }
    emit_dedents_to(0);
    push(TokenKind::EndOfFile, pos_, line_, column_);
    return failed_ ? Status::ValidationError : Status::Ok;
}

} // namespace

std::string_view to_string(TokenKind kind) noexcept {
    switch (kind) {
    case TokenKind::EndOfFile:
        return "end of file";
    case TokenKind::Newline:
        return "end of line";
    case TokenKind::Indent:
        return "indent";
    case TokenKind::Dedent:
        return "dedent";
    case TokenKind::Identifier:
        return "identifier";
    case TokenKind::UnsignedInteger:
        return "uint literal";
    case TokenKind::Integer:
        return "int literal";
    case TokenKind::Float:
        return "float literal";
    case TokenKind::String:
        return "string literal";
    case TokenKind::PhysicalLiteral:
        return "physical literal";
    case TokenKind::Operator:
        return "operator";
    }
    return "token";
}

bool is_reserved_word(std::string_view text) noexcept {
    return std::binary_search(kReservedWords.begin(), kReservedWords.end(), text);
}

Status lex(std::string_view source, const std::string& file, std::vector<Token>& out,
           DiagnosticSink& sink) {
    out.clear();
    Lexer lexer(source, file, out, sink);
    return lexer.run();
}

Status lex(std::string_view source, std::vector<Token>& out, DiagnosticSink& sink) {
    return lex(source, std::string{}, out, sink);
}

} // namespace scena::dsl
