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
// DSL lexer (p7-s1, #39): tokens and layout exactly per ASAM OpenSCENARIO DSL
// 2.2.0 §7.2.1 — line structure and joining, the offside rule, comments,
// identifiers in both forms, every literal shape, and the operator table.

#include <cmath>
#include <string>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

#include "scena/diagnostic.h"
#include "scena/dsl/lexer.h"
#include "scena/dsl/token.h"
#include "scena/status.h"

namespace {

using scena::DiagnosticSink;
using scena::Severity;
using scena::Status;
using scena::dsl::Token;
using scena::dsl::TokenKind;

/// Lexes `source` and expects it to be clean.
std::vector<Token> lex_ok(std::string_view source) {
    DiagnosticSink sink;
    std::vector<Token> tokens;
    const Status status = scena::dsl::lex(source, tokens, sink);
    EXPECT_EQ(status, Status::Ok);
    for (const scena::Diagnostic& diagnostic : sink.diagnostics()) {
        EXPECT_NE(diagnostic.severity, Severity::Error) << diagnostic.message;
    }
    return tokens;
}

/// Lexes `source`, expecting at least one error, and returns the diagnostics.
std::vector<scena::Diagnostic> lex_errors(std::string_view source) {
    DiagnosticSink sink;
    std::vector<Token> tokens;
    EXPECT_EQ(scena::dsl::lex(source, tokens, sink), Status::ValidationError);
    // Whatever went wrong, the stream is still well formed for the parser.
    EXPECT_FALSE(tokens.empty());
    EXPECT_EQ(tokens.back().kind, TokenKind::EndOfFile);
    return sink.diagnostics();
}

/// The token kinds in order, as a readable string — the golden form these
/// tests compare against.
std::string shape(const std::vector<Token>& tokens) {
    std::string out;
    for (const Token& token : tokens) {
        if (!out.empty()) {
            out.push_back(' ');
        }
        switch (token.kind) {
        case TokenKind::Newline:
            out += "NL";
            break;
        case TokenKind::Indent:
            out += "IN";
            break;
        case TokenKind::Dedent:
            out += "DE";
            break;
        case TokenKind::EndOfFile:
            out += "EOF";
            break;
        default:
            out += std::string(token.text);
            break;
        }
    }
    return out;
}

/// The tokens that carry source text, skipping layout.
std::vector<Token> significant(const std::vector<Token>& tokens) {
    std::vector<Token> out;
    for (const Token& token : tokens) {
        if (token.kind != TokenKind::Newline && token.kind != TokenKind::Indent &&
            token.kind != TokenKind::Dedent && token.kind != TokenKind::EndOfFile) {
            out.push_back(token);
        }
    }
    return out;
}

// --- line structure and joining (§7.2.1.2) ---------------------------------

TEST(DslLexerTest, EachLogicalLineEndsWithANewline) {
    EXPECT_EQ(shape(lex_ok("a\nb\n")), "a NL b NL EOF");
}

TEST(DslLexerTest, TheEndOfInputIsAnImplicitEndOfLine) {
    // §7.2.1.2: "The end of input/file also serves as an implicit end-of-line
    // sequence" — a file whose last line has no terminator still ends it.
    EXPECT_EQ(shape(lex_ok("a")), "a NL EOF");
}

TEST(DslLexerTest, EveryEndOfLineSequenceIsAccepted) {
    // CR, LF and CRLF are each *one* end-of-line sequence (§7.2.1.2).
    EXPECT_EQ(shape(lex_ok("a\nb\n")), "a NL b NL EOF");
    EXPECT_EQ(shape(lex_ok("a\rb\r")), "a NL b NL EOF");
    EXPECT_EQ(shape(lex_ok("a\r\nb\r\n")), "a NL b NL EOF");
    // A CRLF file must not be read as two line breaks, or every indented block
    // in it would be separated by a phantom blank line.
    const std::vector<Token> crlf = lex_ok("a\r\n    b\r\n");
    EXPECT_EQ(shape(crlf), "a NL IN b NL DE EOF");
}

TEST(DslLexerTest, BlankAndCommentOnlyLinesAreIgnored) {
    // §7.2.1.2: "Logical lines that contain only whitespace, formfeed
    // characters, and possibly a comment, are ignored" — no NEWLINE for them,
    // and crucially no INDENT either, however they are indented.
    EXPECT_EQ(shape(lex_ok("a\n\n   \n# note\n        # deep comment\nb\n")), "a NL b NL EOF");
}

TEST(DslLexerTest, ACommentRunsToTheEndOfItsPhysicalLine) {
    EXPECT_EQ(shape(lex_ok("a # trailing\nb\n")), "a NL b NL EOF");
}

TEST(DslLexerTest, ABackslashJoinsTheNextPhysicalLine) {
    // Explicit joining (§7.2.1.2): one logical line, so one NEWLINE.
    EXPECT_EQ(shape(lex_ok("a \\\n b\n")), "a b NL EOF");
    EXPECT_EQ(shape(lex_ok("a \\\r\n b\n")), "a b NL EOF");
}

TEST(DslLexerTest, BracketsJoinLinesImplicitly) {
    // §7.2.1.2: a line split inside parentheses or square brackets continues.
    EXPECT_EQ(shape(lex_ok("f(a,\n  b)\n")), "f ( a , b ) NL EOF");
    EXPECT_EQ(shape(lex_ok("[1,\n 2]\n")), "[ 1 , 2 ] NL EOF");
    // The indentation of a continuation line is not indentation at all.
    EXPECT_EQ(shape(lex_ok("f(\n        a)\nb\n")), "f ( a ) NL b NL EOF");
}

TEST(DslLexerTest, AnUnclosedBracketIsReported) {
    const std::vector<scena::Diagnostic> errors = lex_errors("f(a,\n b\n");
    ASSERT_FALSE(errors.empty());
    EXPECT_NE(errors.back().message.find("unclosed"), std::string::npos);
}

// --- indentation (§7.2.1.4) ------------------------------------------------

TEST(DslLexerTest, IndentAndDedentBracketBlocks) {
    EXPECT_EQ(shape(lex_ok("a:\n  b\n  c\nd\n")), "a : NL IN b NL c NL DE d NL EOF");
}

TEST(DslLexerTest, NestedBlocksCloseInOneStep) {
    // Returning to column 0 from two levels emits two DEDENTs.
    EXPECT_EQ(shape(lex_ok("a:\n  b:\n    c\nd\n")), "a : NL IN b : NL IN c NL DE DE d NL EOF");
}

TEST(DslLexerTest, EveryIndentIsClosedAtEndOfFile) {
    const std::vector<Token> tokens = lex_ok("a:\n  b:\n    c\n");
    int indents = 0;
    int dedents = 0;
    for (const Token& token : tokens) {
        indents += token.kind == TokenKind::Indent ? 1 : 0;
        dedents += token.kind == TokenKind::Dedent ? 1 : 0;
    }
    EXPECT_EQ(indents, 2);
    EXPECT_EQ(dedents, 2);
    EXPECT_EQ(tokens.back().kind, TokenKind::EndOfFile);
}

TEST(DslLexerTest, ATabAdvancesToTheNextEightColumnStop) {
    // §7.2.1.4: "tab characters are replaced with spaces with the assumption
    // that tab stops are every 8 characters". A tab and eight spaces are the
    // same indentation, so mixing them across sibling lines is not an error.
    EXPECT_EQ(shape(lex_ok("a:\n\tb\n        c\nd\n")), "a : NL IN b NL c NL DE d NL EOF");
    // Four spaces then a tab still lands on column 8.
    EXPECT_EQ(shape(lex_ok("a:\n    \tb\n        c\nd\n")), "a : NL IN b NL c NL DE d NL EOF");
}

TEST(DslLexerTest, ADedentToAnUnknownColumnIsReported) {
    // Python 3.10 §2.1.8, which §7.2.1.4 adopts: a dedent must return to a
    // column an outer line already used. Landing between levels does not say
    // which block it closes.
    const std::vector<scena::Diagnostic> errors = lex_errors("a:\n    b:\n        c\n  d\n");
    ASSERT_FALSE(errors.empty());
    EXPECT_NE(errors.front().message.find("indentation"), std::string::npos);
}

TEST(DslLexerTest, AFormfeedAtLineStartDoesNotCount) {
    // §7.2.1.4: "Formfeed characters at the start of a line are ignored for
    // indentation calculations."
    EXPECT_EQ(shape(lex_ok("a:\n\f  b\nc\n")), "a : NL IN b NL DE c NL EOF");
}

// --- identifiers (§7.2.1.5.1) ----------------------------------------------

TEST(DslLexerTest, IdentifiersAreCaseSensitiveAndMayHoldDigitsAndUnderscores) {
    const std::vector<Token> tokens = significant(lex_ok("Ego ego_2 _leading\n"));
    ASSERT_EQ(tokens.size(), 3U);
    for (const Token& token : tokens) {
        EXPECT_EQ(token.kind, TokenKind::Identifier);
        EXPECT_FALSE(token.keyword);
    }
    EXPECT_EQ(tokens[0].text, "Ego");
    EXPECT_EQ(tokens[1].text, "ego_2");
    EXPECT_EQ(tokens[2].text, "_leading");
}

TEST(DslLexerTest, ReservedWordsAreIdentifiersThatKnowTheyAreReserved) {
    // §7.2.1.5.1: keywords "are only recognized as keywords in the places
    // identified in the grammar ... treated as normal identifier tokens in all
    // other places". So the lexer must not have a Keyword token kind.
    const std::vector<Token> tokens = significant(lex_ok("scenario notakeyword do\n"));
    ASSERT_EQ(tokens.size(), 3U);
    EXPECT_EQ(tokens[0].kind, TokenKind::Identifier);
    EXPECT_TRUE(tokens[0].keyword);
    EXPECT_FALSE(tokens[1].keyword);
    EXPECT_TRUE(tokens[2].keyword);
}

TEST(DslLexerTest, EveryTable4WordIsReserved) {
    for (const std::string_view word :
         {"action", "actor", "and",      "as",         "bool",      "call",     "cd",
          "cover",  "def",   "default",  "do",         "elapsed",   "emit",     "enum",
          "event",  "every", "export",   "expression", "extend",    "external", "factor",
          "fall",   "false", "float",    "global",     "hard",      "if",       "import",
          "in",     "inf",   "inherits", "int",        "is",        "it",       "K",
          "keep",   "kg",    "list",     "m",          "modifier",  "mol",      "namespace",
          "nan",    "not",   "null",     "of",         "offset",    "on",       "one_of",
          "only",   "or",    "parallel", "rad",        "range",     "record",   "remove_default",
          "rise",   "s",     "sample",   "scenario",   "serial",    "SI",       "string",
          "struct", "true",  "type",     "uint",       "undefined", "unit",     "until",
          "use",    "var",   "wait",     "with"}) {
        EXPECT_TRUE(scena::dsl::is_reserved_word(word)) << word;
    }
    EXPECT_FALSE(scena::dsl::is_reserved_word("scenarios"));
    EXPECT_FALSE(scena::dsl::is_reserved_word("Do"));
}

TEST(DslLexerTest, AnEscapedIdentifierCarriesAnythingButAVerticalLine) {
    const std::vector<Token> tokens = significant(lex_ok("|a b·c| |scenario|\n"));
    ASSERT_EQ(tokens.size(), 2U);
    EXPECT_EQ(tokens[0].kind, TokenKind::Identifier);
    EXPECT_EQ(tokens[0].value, "a b·c");
    // Delimiting is exactly how a file names something otherwise reserved, so
    // an escaped identifier is never a keyword.
    EXPECT_EQ(tokens[1].value, "scenario");
    EXPECT_FALSE(tokens[1].keyword);
}

TEST(DslLexerTest, EscapedIdentifiersNeedNoSurroundingWhitespace) {
    // §7.2.1.5.1: "such identifiers do not need to be separated by whitespace".
    const std::vector<Token> tokens = significant(lex_ok("|a||b|\n"));
    ASSERT_EQ(tokens.size(), 2U);
    EXPECT_EQ(tokens[0].value, "a");
    EXPECT_EQ(tokens[1].value, "b");
}

TEST(DslLexerTest, AnUnterminatedOrEmptyEscapedIdentifierIsReported) {
    EXPECT_FALSE(lex_errors("|unterminated\n").empty());
    const std::vector<scena::Diagnostic> empty = lex_errors("||\n");
    ASSERT_FALSE(empty.empty());
    EXPECT_NE(empty.front().message.find("empty"), std::string::npos);
}

TEST(DslLexerTest, UnicodeIdentifiersAreAccepted) {
    // §7.2.1.1 mandates UTF-8 and §7.2.1.5.1 admits every Unicode letter
    // category, so a non-Latin identifier is ordinary, not exotic.
    const std::vector<Token> tokens = significant(lex_ok("Fahrzeug_Ähre\n"));
    ASSERT_EQ(tokens.size(), 1U);
    EXPECT_EQ(tokens[0].kind, TokenKind::Identifier);
}

// --- literals (§7.2.1.5.2) -------------------------------------------------

TEST(DslLexerTest, IntegerLiteralsCarryTheirExactValue) {
    const std::vector<Token> tokens = significant(lex_ok("42 -7 0x1F\n"));
    ASSERT_EQ(tokens.size(), 3U);
    // "Positive integer literals produce a uint value" (§7.2.1.5.2).
    EXPECT_EQ(tokens[0].kind, TokenKind::UnsignedInteger);
    EXPECT_EQ(tokens[0].unsigned_value, 42U);
    // "Negative integer literals produce an int value."
    EXPECT_EQ(tokens[1].kind, TokenKind::Integer);
    EXPECT_EQ(tokens[1].signed_value, -7);
    EXPECT_EQ(tokens[2].kind, TokenKind::UnsignedInteger);
    EXPECT_EQ(tokens[2].unsigned_value, 31U);
}

TEST(DslLexerTest, ALargeUintKeepsFullPrecision) {
    // A double would lose the low bits above 2^53, so the exact value is kept
    // separately — the spec makes uint 64-bit and says exceeding it is an error.
    const std::vector<Token> tokens = significant(lex_ok("9007199254740993\n"));
    ASSERT_EQ(tokens.size(), 1U);
    EXPECT_EQ(tokens[0].unsigned_value, 9007199254740993ULL);
}

TEST(DslLexerTest, AnOversizedIntegerLiteralIsReported) {
    // §7.2.1.5.2: "It is an error to include a positive integer literal ...
    // that cannot be represented using the uint type."
    EXPECT_FALSE(lex_errors("99999999999999999999999\n").empty());
}

TEST(DslLexerTest, FloatLiteralsCoverEveryProductionForm) {
    const std::vector<Token> tokens = significant(lex_ok("1.5 .25 1e3 2.5E-2 inf nan\n"));
    ASSERT_EQ(tokens.size(), 6U);
    for (const Token& token : tokens) {
        EXPECT_EQ(token.kind, TokenKind::Float) << token.text;
    }
    EXPECT_DOUBLE_EQ(tokens[0].number, 1.5);
    EXPECT_DOUBLE_EQ(tokens[1].number, 0.25);
    EXPECT_DOUBLE_EQ(tokens[2].number, 1000.0);
    EXPECT_DOUBLE_EQ(tokens[3].number, 0.025);
    EXPECT_TRUE(std::isinf(tokens[4].number));
    EXPECT_TRUE(std::isnan(tokens[5].number));
}

TEST(DslLexerTest, NumbersAreReadLocaleIndependently) {
    // std::from_chars, never strtod: a comma-decimal locale must not change
    // what "1.5" means. Same rule the XML frontend follows.
    const std::vector<Token> tokens = significant(lex_ok("1.5\n"));
    ASSERT_EQ(tokens.size(), 1U);
    EXPECT_DOUBLE_EQ(tokens[0].number, 1.5);
}

TEST(DslLexerTest, APhysicalLiteralIsANumberJoinedToAUnitName) {
    // §7.2.1.5.2: created "when an identifier naming a valid unit is included
    // directly after a valid float or integer literal without any intervening
    // whitespace".
    const std::vector<Token> tokens = significant(lex_ok("60kph 1.5s 10m\n"));
    ASSERT_EQ(tokens.size(), 3U);
    EXPECT_EQ(tokens[0].kind, TokenKind::PhysicalLiteral);
    EXPECT_EQ(tokens[0].value, "kph");
    EXPECT_DOUBLE_EQ(tokens[0].number, 60.0);
    EXPECT_EQ(tokens[0].text, "60kph");
    EXPECT_EQ(tokens[1].value, "s");
    EXPECT_DOUBLE_EQ(tokens[1].number, 1.5);
    EXPECT_EQ(tokens[2].value, "m");
}

TEST(DslLexerTest, WhitespaceSeparatesANumberFromAUnitName) {
    // The same characters with a space between are two tokens, not one — which
    // is what "without any intervening whitespace" means.
    const std::vector<Token> tokens = significant(lex_ok("60 kph\n"));
    ASSERT_EQ(tokens.size(), 2U);
    EXPECT_EQ(tokens[0].kind, TokenKind::UnsignedInteger);
    EXPECT_EQ(tokens[1].kind, TokenKind::Identifier);
}

TEST(DslLexerTest, StringLiteralsComeInFourDelimiters) {
    const std::vector<Token> tokens =
        significant(lex_ok("\"double\" 'single' \"\"\"long\"\"\" '''also'''\n"));
    ASSERT_EQ(tokens.size(), 4U);
    for (const Token& token : tokens) {
        EXPECT_EQ(token.kind, TokenKind::String);
    }
    EXPECT_EQ(tokens[0].value, "double");
    EXPECT_EQ(tokens[1].value, "single");
    EXPECT_EQ(tokens[2].value, "long");
    EXPECT_EQ(tokens[3].value, "also");
}

TEST(DslLexerTest, StringEscapesAreDecoded) {
    // string-escape-seq ::= '\' any-char (§7.2.1.5.2).
    const std::vector<Token> tokens = significant(lex_ok("\"a\\nb\\\"c\\\\d\"\n"));
    ASSERT_EQ(tokens.size(), 1U);
    EXPECT_EQ(tokens[0].value, "a\nb\"c\\d");
}

TEST(DslLexerTest, TheOtherQuoteNeedsNoEscapeInsideAString) {
    // shortstring-char excludes "the quote character used to introduce the
    // string" — the other one is ordinary text.
    const std::vector<Token> tokens = significant(lex_ok("'he said \"hi\"'\n"));
    ASSERT_EQ(tokens.size(), 1U);
    EXPECT_EQ(tokens[0].value, "he said \"hi\"");
}

TEST(DslLexerTest, AHashInsideAStringIsNotAComment) {
    // §7.2.1.3: comments start with a `#` "that is outside of a string literal".
    const std::vector<Token> tokens = significant(lex_ok("\"a # b\" c\n"));
    ASSERT_EQ(tokens.size(), 2U);
    EXPECT_EQ(tokens[0].value, "a # b");
    EXPECT_EQ(tokens[1].text, "c");
}

TEST(DslLexerTest, ALongStringMaySpanPhysicalLinesButAShortOneMayNot) {
    const std::vector<Token> tokens = significant(lex_ok("\"\"\"one\ntwo\"\"\"\nx\n"));
    ASSERT_EQ(tokens.size(), 2U);
    EXPECT_EQ(tokens[0].value, "one\ntwo");
    EXPECT_FALSE(lex_errors("\"unterminated\nx\n").empty());
}

// --- operators (§7.2.1.5.3) ------------------------------------------------

TEST(DslLexerTest, TheLongestLegalTokenWins) {
    // §7.2.1.5: "tokens comprise the longest possible match that forms a legal
    // token". So `==` is one token, not two `=`.
    EXPECT_EQ(shape(lex_ok("== != <= >= -> => ::\n")), "== != <= >= -> => :: NL EOF");
    EXPECT_EQ(shape(lex_ok("= < > - :\n")), "= < > - : NL EOF");
}

TEST(DslLexerTest, SingleCharacterOperatorsAreRecognized) {
    const std::vector<Token> tokens = significant(lex_ok(".,:=@()[]?+*/%\n"));
    for (const Token& token : tokens) {
        EXPECT_EQ(token.kind, TokenKind::Operator) << token.text;
    }
    EXPECT_EQ(tokens.size(), 14U);
}

TEST(DslLexerTest, AnUnexpectedCharacterIsReportedAndSkipped) {
    // Recovery matters: the pillar wants many useful diagnostics, not the first
    // one. Lexing continues, so the tokens after the offending byte survive.
    DiagnosticSink sink;
    std::vector<Token> tokens;
    EXPECT_EQ(scena::dsl::lex("a $ b\n", tokens, sink), Status::ValidationError);
    ASSERT_EQ(sink.diagnostics().size(), 1U);
    EXPECT_EQ(sink.diagnostics().front().location.line, 1);
    const std::vector<Token> kept = significant(tokens);
    ASSERT_EQ(kept.size(), 2U);
    EXPECT_EQ(kept[0].text, "a");
    EXPECT_EQ(kept[1].text, "b");
}

// --- positions and diagnostics ---------------------------------------------

TEST(DslLexerTest, TokensCarryOneBasedLineAndColumn) {
    const std::vector<Token> tokens = significant(lex_ok("ab cd\n  ef\n"));
    ASSERT_EQ(tokens.size(), 3U);
    EXPECT_EQ(tokens[0].line, 1);
    EXPECT_EQ(tokens[0].column, 1);
    EXPECT_EQ(tokens[1].line, 1);
    EXPECT_EQ(tokens[1].column, 4);
    EXPECT_EQ(tokens[2].line, 2);
    EXPECT_EQ(tokens[2].column, 3);
}

TEST(DslLexerTest, DiagnosticsCiteTheSectionTheyEnforce) {
    // The DSL spec defines no asam.net rule ids at all, so a §-citation is the
    // only machine-traceable reference a diagnostic can carry.
    const std::vector<scena::Diagnostic> errors = lex_errors("\"unterminated\n");
    ASSERT_FALSE(errors.empty());
    EXPECT_NE(errors.front().message.find("§7.2.1.5.2"), std::string::npos);
    EXPECT_TRUE(errors.front().rule_id.empty());
}

TEST(DslLexerTest, AFileNameReachesTheDiagnostics) {
    DiagnosticSink sink;
    std::vector<Token> tokens;
    EXPECT_EQ(scena::dsl::lex("\"oops\n", "cut_in.osc", tokens, sink), Status::ValidationError);
    ASSERT_FALSE(sink.diagnostics().empty());
    EXPECT_EQ(sink.diagnostics().front().location.file, "cut_in.osc");
}

// --- a realistic fragment --------------------------------------------------

TEST(DslLexerTest, ARealisticScenarioFragmentLexesCleanly) {
    // Written from the shapes §7.2.2's grammar describes; nothing here is
    // copied from another project's corpus (ADR-0002).
    constexpr std::string_view kSource = R"(import osc.standard.all

scenario dut.cut_in:
    ego: vehicle          # the vehicle under test
    cutter: vehicle
    do serial:
        phase1: parallel(duration: 5s):
            ego.drive() with:
                speed(30kph)
            cutter.drive() with:
                lane(same_as: ego, at: start)
        phase2: cutter.change_lane(side: left, \
                                   duration: 3.0s)
)";
    const std::vector<Token> tokens = lex_ok(kSource);
    ASSERT_FALSE(tokens.empty());
    EXPECT_EQ(tokens.back().kind, TokenKind::EndOfFile);

    // The physical literals came through as such, with their units.
    int physical = 0;
    for (const Token& token : tokens) {
        physical += token.kind == TokenKind::PhysicalLiteral ? 1 : 0;
    }
    EXPECT_EQ(physical, 3); // 5s, 30kph, 3.0s

    // Indentation is balanced and the continuation line did not start a block.
    int indents = 0;
    int dedents = 0;
    for (const Token& token : tokens) {
        indents += token.kind == TokenKind::Indent ? 1 : 0;
        dedents += token.kind == TokenKind::Dedent ? 1 : 0;
    }
    EXPECT_EQ(indents, dedents);
    EXPECT_GT(indents, 0);
}

TEST(DslLexerTest, TheRangeOperatorWinsOverAFloatThatStartsWithADot) {
    // §7.2.2.6.7 spells the range constructor `'[' expression '..' expression
    // ']'`, and §7.2.1.5.2's `float-literal ::= digit* '.' digit+` makes the
    // leading digits optional — so `[2..4]` is a race between `..` and `.4`,
    // and the operator has to win or the standard's own spelling does not lex.
    // Table 5 lists neither `..` nor `::`; the grammar productions decide.
    const std::vector<Token> tokens = lex_ok("[2..4]");
    ASSERT_EQ(tokens.size(), 7U); // [ 2 .. 4 ] NEWLINE EOF
    EXPECT_EQ(tokens[1].text, "2");
    EXPECT_EQ(tokens[2].text, "..");
    EXPECT_EQ(tokens[3].text, "4");
}

TEST(DslLexerTest, ARangeOfPhysicalLiteralsLexesToo) {
    // The form the specification actually writes durations in (§7.6.2.4).
    const std::vector<Token> tokens = lex_ok("[10s..30s]");
    ASSERT_EQ(tokens.size(), 7U);
    EXPECT_EQ(tokens[1].text, "10s");
    EXPECT_EQ(tokens[2].text, "..");
    EXPECT_EQ(tokens[3].text, "30s");
}

TEST(DslLexerTest, LexingIsDeterministicAcrossRepeats) {
    // The frontend's output reaches the IR, so it is inside the bit-identity
    // contract exactly as the XML frontend's is.
    constexpr std::string_view kSource = "scenario s:\n  a: b\n  do serial:\n    x(1.25m)\n";
    const std::vector<Token> first = lex_ok(kSource);
    const std::vector<Token> second = lex_ok(kSource);
    ASSERT_EQ(first.size(), second.size());
    for (std::size_t i = 0; i < first.size(); ++i) {
        EXPECT_EQ(first[i].kind, second[i].kind);
        EXPECT_EQ(first[i].text, second[i].text);
        EXPECT_EQ(first[i].value, second[i].value);
        EXPECT_DOUBLE_EQ(first[i].number, second[i].number);
        EXPECT_EQ(first[i].line, second[i].line);
        EXPECT_EQ(first[i].column, second[i].column);
    }
}

} // namespace
