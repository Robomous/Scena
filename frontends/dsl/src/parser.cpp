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

#include "scena/dsl/parser.h"

#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "scena/dsl/lexer.h"

namespace scena::dsl {

namespace {

/// Thrown internally to unwind to the nearest synchronisation point. Never
/// escapes `parse` — no exception crosses a Scena API boundary (cpp-style).
struct ParseError {};

class Parser {
public:
    Parser(const std::vector<Token>& tokens, std::string file, File& out, DiagnosticSink& sink)
        : tokens_(tokens), file_(std::move(file)), out_(out), sink_(sink) {}

    Status run();

private:
    // --- token cursor ----------------------------------------------------

    [[nodiscard]] const Token& peek(std::size_t ahead = 0) const {
        const std::size_t index = pos_ + ahead;
        return index < tokens_.size() ? tokens_[index] : tokens_.back();
    }
    [[nodiscard]] bool at(TokenKind kind) const { return peek().kind == kind; }
    [[nodiscard]] bool at_end() const { return at(TokenKind::EndOfFile); }
    /// True when the cursor is on the operator `text`.
    [[nodiscard]] bool at_op(std::string_view text) const {
        return peek().kind == TokenKind::Operator && peek().text == text;
    }
    /// True when the cursor is on the reserved word `text`. Reserved words are
    /// identifiers (§7.2.1.5.1); this is the "in the places identified in the
    /// grammar" half of that rule.
    [[nodiscard]] bool at_word(std::string_view text) const {
        return peek().kind == TokenKind::Identifier && peek().keyword && peek().text == text;
    }
    const Token& advance() {
        const Token& token = peek();
        if (pos_ + 1 < tokens_.size()) {
            ++pos_;
        }
        return token;
    }
    bool accept_op(std::string_view text) {
        if (!at_op(text)) {
            return false;
        }
        advance();
        return true;
    }
    bool accept_word(std::string_view text) {
        if (!at_word(text)) {
            return false;
        }
        advance();
        return true;
    }
    const Token& expect_op(std::string_view text, std::string_view section);
    void expect_word(std::string_view text, std::string_view section);
    void expect_newline(std::string_view section);
    void expect_indent(std::string_view section);
    void expect_dedent(std::string_view section);

    [[nodiscard]] static SourceRange range_of(const Token& token) {
        return SourceRange{token.line, token.column};
    }

    // --- reporting -------------------------------------------------------

    void error(const Token& at_token, std::string message);
    [[noreturn]] void fail(const Token& at_token, std::string message);
    /// Skips to the end of the current logical line, consuming any block it
    /// opened. The statement-level synchronisation point.
    void synchronize_statement();
    /// Skips the rest of the current block, leaving the cursor after its DEDENT.
    void skip_block();

    // --- productions -----------------------------------------------------

    std::string parse_qualified_identifier(std::string_view what, std::string_view section);
    std::string parse_structured_identifier();
    TypeRef parse_type_declarator();
    std::vector<Argument> parse_argument_list(bool allow_names = true);
    std::vector<Argument> parse_argument_list_specification();

    ExprPtr parse_expression();
    ExprPtr parse_implication();
    ExprPtr parse_disjunction();
    ExprPtr parse_conjunction();
    ExprPtr parse_inversion();
    ExprPtr parse_relation();
    ExprPtr parse_sum();
    ExprPtr parse_term();
    ExprPtr parse_factor();
    ExprPtr parse_postfix();
    ExprPtr parse_primary();

    EventSpec parse_event_spec();
    Constraint parse_constraint();
    ModifierApplication parse_modifier_application(ExprPtr actor, const Token& name_token);
    DoMemberPtr parse_do_member();
    void parse_behavior_with(BehaviorWith& out);
    OnDirective parse_on_directive();

    bool parse_member(std::vector<Member>& out);
    void parse_structured_body(StructuredDecl& decl);

    void parse_declaration();
    void parse_import();
    void parse_namespace();
    void parse_export();
    void parse_physical_type();
    void parse_unit();
    void parse_enum(bool is_extension);
    void parse_structured(StructuredKind kind);
    void parse_extend();
    void parse_global_parameter();
    std::vector<SiExponent> parse_si_specifier(std::optional<double>& factor,
                                               std::optional<double>& offset);

    const std::vector<Token>& tokens_;
    std::string file_;
    File& out_;
    DiagnosticSink& sink_;
    std::size_t pos_ = 0;
    bool failed_ = false;
};

// --- reporting -------------------------------------------------------------

void Parser::error(const Token& at_token, std::string message) {
    Diagnostic diagnostic;
    diagnostic.severity = Severity::Error;
    diagnostic.code = Status::ValidationError;
    diagnostic.message = std::move(message);
    diagnostic.location.file = file_;
    diagnostic.location.line = at_token.line;
    diagnostic.location.column = at_token.column;
    sink_.report(std::move(diagnostic));
    failed_ = true;
}

void Parser::fail(const Token& at_token, std::string message) {
    error(at_token, std::move(message));
    throw ParseError{};
}

/// A readable name for what the cursor is sitting on, for "expected X, found Y".
std::string describe(const Token& token) {
    switch (token.kind) {
    case TokenKind::EndOfFile:
        return "end of file";
    case TokenKind::Newline:
        return "end of line";
    case TokenKind::Indent:
        return "an indented block";
    case TokenKind::Dedent:
        return "the end of a block";
    default:
        return "'" + std::string(token.text) + "'";
    }
}

void Parser::synchronize_statement() {
    // Skip to the end of the logical line. If the line opened a block, skip the
    // block too, so the next statement parsed is a sibling rather than a member
    // of something that failed.
    while (!at_end() && !at(TokenKind::Newline) && !at(TokenKind::Dedent)) {
        advance();
    }
    if (at(TokenKind::Newline)) {
        advance();
    }
    if (at(TokenKind::Indent)) {
        skip_block();
    }
}

void Parser::skip_block() {
    if (!at(TokenKind::Indent)) {
        return;
    }
    advance();
    int depth = 1;
    while (!at_end() && depth > 0) {
        if (at(TokenKind::Indent)) {
            ++depth;
        } else if (at(TokenKind::Dedent)) {
            --depth;
        }
        advance();
    }
}

const Token& Parser::expect_op(std::string_view text, std::string_view section) {
    if (!at_op(text)) {
        fail(peek(), "expected '" + std::string(text) + "' (" + std::string(section) + "), found " +
                         describe(peek()));
    }
    return advance();
}

void Parser::expect_word(std::string_view text, std::string_view section) {
    if (!at_word(text)) {
        fail(peek(), "expected '" + std::string(text) + "' (" + std::string(section) + "), found " +
                         describe(peek()));
    }
    advance();
}

void Parser::expect_newline(std::string_view section) {
    if (!at(TokenKind::Newline)) {
        fail(peek(), "expected the end of the line (" + std::string(section) + "), found " +
                         describe(peek()));
    }
    advance();
}

void Parser::expect_indent(std::string_view section) {
    if (!at(TokenKind::Indent)) {
        fail(peek(), "expected an indented block (" + std::string(section) + "), found " +
                         describe(peek()));
    }
    advance();
}

void Parser::expect_dedent(std::string_view section) {
    if (!at(TokenKind::Dedent)) {
        fail(peek(), "expected the end of the block (" + std::string(section) + "), found " +
                         describe(peek()));
    }
    advance();
}

// --- names and types -------------------------------------------------------

std::string Parser::parse_qualified_identifier(std::string_view what, std::string_view section) {
    // qualified-identifier ::= identifier | [namespace-name] '::' identifier
    std::string name;
    if (at_op("::")) {
        advance();
        name = "::";
    }
    if (!at(TokenKind::Identifier)) {
        fail(peek(), "expected " + std::string(what) + " (" + std::string(section) + "), found " +
                         describe(peek()));
    }
    const Token& first = advance();
    name += first.value.empty() ? std::string(first.text) : first.value;
    if (at_op("::")) {
        advance();
        if (!at(TokenKind::Identifier)) {
            fail(peek(),
                 "expected an identifier after '::' (§7.2.1.5.1), found " + describe(peek()));
        }
        const Token& second = advance();
        name += "::" + (second.value.empty() ? std::string(second.text) : second.value);
    }
    return name;
}

std::string Parser::parse_structured_identifier() {
    // structured-identifier ::= identifier | structured-identifier '.' identifier
    std::string name = parse_qualified_identifier("an identifier", "§7.2.2.1.1");
    while (at_op(".")) {
        advance();
        if (!at(TokenKind::Identifier)) {
            fail(peek(),
                 "expected an identifier after '.' (§7.2.2.1.1), found " + describe(peek()));
        }
        const Token& part = advance();
        name += "." + (part.value.empty() ? std::string(part.text) : part.value);
    }
    return name;
}

TypeRef Parser::parse_type_declarator() {
    TypeRef type;
    type.range = range_of(peek());
    if (accept_word("list")) {
        type.is_list = true;
        expect_word("of", "§7.2.2.2.3");
    }
    if (accept_word("range")) {
        type.is_range = true;
        expect_word("of", "§7.2.2.2.3");
    }
    type.name = parse_qualified_identifier("a type name", "§7.2.2.2");
    return type;
}

// --- arguments -------------------------------------------------------------

std::vector<Argument> Parser::parse_argument_list(bool allow_names) {
    // argument-list ::= positional (',' positional)* (',' named)* | named (',' named)*
    std::vector<Argument> arguments;
    if (at_op(")")) {
        return arguments;
    }
    bool seen_named = false;
    do {
        Argument argument;
        argument.range = range_of(peek());
        // A named argument is `name ':' expression`; anything else is
        // positional. Two tokens of lookahead separate them.
        if (allow_names && at(TokenKind::Identifier) && peek(1).kind == TokenKind::Operator &&
            peek(1).text == ":") {
            const Token& name = advance();
            argument.name = name.value.empty() ? std::string(name.text) : name.value;
            advance(); // ':'
            seen_named = true;
        } else if (seen_named) {
            // §7.2.2.5.2 puts every positional argument before every named one.
            error(peek(), "a positional argument cannot follow a named one (§7.2.2.5.2)");
        }
        argument.value = parse_expression();
        arguments.push_back(std::move(argument));
    } while (accept_op(","));
    return arguments;
}

std::vector<Argument> Parser::parse_argument_list_specification() {
    // argument-specification ::= argument-name ':' type-declarator ['=' default]
    std::vector<Argument> arguments;
    if (at_op(")")) {
        return arguments;
    }
    do {
        Argument argument;
        argument.range = range_of(peek());
        argument.name = parse_qualified_identifier("an argument name", "§7.2.2.5.1");
        expect_op(":", "§7.2.2.5.1");
        // The declared type is kept on the expression node so the type system
        // sees it; a specification has no value of its own until a default.
        const TypeRef type = parse_type_declarator();
        auto marker = std::make_shared<Expr>();
        marker->kind = ExprKind::Name;
        marker->range = type.range;
        marker->text = type.name;
        marker->type = type;
        argument.value = marker;
        if (accept_op("=")) {
            argument.value = parse_expression();
            // Keep the declared type alongside the default.
            auto with_type = std::make_shared<Expr>(*argument.value);
            with_type->type = type;
            argument.value = with_type;
        }
        arguments.push_back(std::move(argument));
    } while (accept_op(","));
    return arguments;
}

// --- expressions (§7.2.2.6) ------------------------------------------------

ExprPtr make_binary(std::string op, ExprPtr lhs, ExprPtr rhs, SourceRange range) {
    auto expr = std::make_shared<Expr>();
    expr->kind = ExprKind::Binary;
    expr->text = std::move(op);
    expr->range = range;
    expr->operands = {std::move(lhs), std::move(rhs)};
    return expr;
}

ExprPtr Parser::parse_expression() {
    // expression ::= implication | ternary-op-exp, and
    // ternary-op-exp ::= implication '?' expression ':' expression — so parse
    // the implication first and let a '?' decide which production it was.
    ExprPtr condition = parse_implication();
    if (!at_op("?")) {
        return condition;
    }
    const SourceRange range = range_of(peek());
    advance();
    ExprPtr then_branch = parse_expression();
    expect_op(":", "§7.2.2.6.1");
    ExprPtr else_branch = parse_expression();
    auto expr = std::make_shared<Expr>();
    expr->kind = ExprKind::Ternary;
    expr->range = range;
    expr->operands = {std::move(condition), std::move(then_branch), std::move(else_branch)};
    return expr;
}

ExprPtr Parser::parse_implication() {
    ExprPtr lhs = parse_disjunction();
    while (at_op("=>")) {
        const SourceRange range = range_of(peek());
        advance();
        lhs = make_binary("=>", std::move(lhs), parse_disjunction(), range);
    }
    return lhs;
}

ExprPtr Parser::parse_disjunction() {
    ExprPtr lhs = parse_conjunction();
    while (at_word("or")) {
        const SourceRange range = range_of(peek());
        advance();
        lhs = make_binary("or", std::move(lhs), parse_conjunction(), range);
    }
    return lhs;
}

ExprPtr Parser::parse_conjunction() {
    ExprPtr lhs = parse_inversion();
    while (at_word("and")) {
        const SourceRange range = range_of(peek());
        advance();
        lhs = make_binary("and", std::move(lhs), parse_inversion(), range);
    }
    return lhs;
}

ExprPtr Parser::parse_inversion() {
    if (at_word("not")) {
        const SourceRange range = range_of(peek());
        advance();
        auto expr = std::make_shared<Expr>();
        expr->kind = ExprKind::Unary;
        expr->text = "not";
        expr->range = range;
        expr->operands = {parse_inversion()};
        return expr;
    }
    return parse_relation();
}

ExprPtr Parser::parse_relation() {
    // relation ::= sum | relation relational-op sum — left recursion written as
    // a loop, which recognises the same language (§7.2.2's own note).
    ExprPtr lhs = parse_sum();
    while (at_op("==") || at_op("!=") || at_op("<") || at_op("<=") || at_op(">") || at_op(">=") ||
           at_word("in")) {
        const Token& op = peek();
        const SourceRange range = range_of(op);
        const std::string spelling(op.text);
        advance();
        lhs = make_binary(spelling, std::move(lhs), parse_sum(), range);
    }
    return lhs;
}

ExprPtr Parser::parse_sum() {
    ExprPtr lhs = parse_term();
    while (at_op("+") || at_op("-")) {
        const std::string spelling(peek().text);
        const SourceRange range = range_of(peek());
        advance();
        lhs = make_binary(spelling, std::move(lhs), parse_term(), range);
    }
    // A negative numeric literal directly after an operand is subtraction, not
    // a literal: `a -1` is `a - 1`. The lexer cannot tell (§7.2.1.5.2 makes
    // '-' part of int-literal), so the split happens here, where the left
    // operand is known.
    while ((peek().kind == TokenKind::Integer || peek().kind == TokenKind::PhysicalLiteral) &&
           !peek().text.empty() && peek().text.front() == '-') {
        const Token& token = peek();
        const SourceRange range = range_of(token);
        auto positive = std::make_shared<Expr>();
        positive->range = range;
        if (token.kind == TokenKind::PhysicalLiteral) {
            positive->kind = ExprKind::PhysicalLiteral;
            positive->text = token.value;
            positive->number = -token.number;
        } else {
            positive->kind = ExprKind::Literal;
            positive->literal_type = LiteralType::Integer;
            positive->signed_value = -token.signed_value;
            positive->number = -token.number;
        }
        advance();
        lhs = make_binary("-", std::move(lhs), positive, range);
    }
    return lhs;
}

ExprPtr Parser::parse_term() {
    ExprPtr lhs = parse_factor();
    while (at_op("*") || at_op("/") || at_op("%")) {
        const std::string spelling(peek().text);
        const SourceRange range = range_of(peek());
        advance();
        lhs = make_binary(spelling, std::move(lhs), parse_factor(), range);
    }
    return lhs;
}

ExprPtr Parser::parse_factor() {
    if (at_op("-")) {
        const SourceRange range = range_of(peek());
        advance();
        auto expr = std::make_shared<Expr>();
        expr->kind = ExprKind::Unary;
        expr->text = "-";
        expr->range = range;
        expr->operands = {parse_factor()};
        return expr;
    }
    return parse_postfix();
}

ExprPtr Parser::parse_postfix() {
    ExprPtr base = parse_primary();
    for (;;) {
        if (at_op(".")) {
            const SourceRange range = range_of(peek());
            advance();
            // `.as(T)` and `.is(T)` are postfix operators, not field accesses
            // (§7.2.2.6.5); `as` and `is` are reserved words, so the spelling
            // alone identifies them here.
            if (at_word("as") || at_word("is")) {
                const bool is_cast = peek().text == "as";
                advance();
                expect_op("(", "§7.2.2.6.5");
                auto expr = std::make_shared<Expr>();
                expr->kind = is_cast ? ExprKind::Cast : ExprKind::TypeTest;
                expr->range = range;
                expr->type = parse_type_declarator();
                expr->type_name = expr->type->name;
                expr->operands = {std::move(base)};
                expect_op(")", "§7.2.2.6.5");
                base = expr;
                continue;
            }
            if (!at(TokenKind::Identifier)) {
                fail(peek(),
                     "expected a field name after '.' (§7.2.2.6.5), found " + describe(peek()));
            }
            const Token& field = advance();
            auto expr = std::make_shared<Expr>();
            expr->kind = ExprKind::FieldAccess;
            expr->range = range;
            expr->text = field.value.empty() ? std::string(field.text) : field.value;
            expr->operands = {std::move(base)};
            base = expr;
            continue;
        }
        if (at_op("[")) {
            const SourceRange range = range_of(peek());
            advance();
            auto expr = std::make_shared<Expr>();
            expr->kind = ExprKind::ElementAccess;
            expr->range = range;
            expr->operands = {std::move(base), parse_expression()};
            expect_op("]", "§7.2.2.6.5");
            base = expr;
            continue;
        }
        if (at_op("(")) {
            const SourceRange range = range_of(peek());
            advance();
            auto expr = std::make_shared<Expr>();
            expr->kind = ExprKind::Call;
            expr->range = range;
            expr->operands = {std::move(base)};
            expr->arguments = parse_argument_list();
            expect_op(")", "§7.2.2.6.5");
            base = expr;
            continue;
        }
        return base;
    }
}

ExprPtr Parser::parse_primary() {
    const Token& token = peek();
    auto expr = std::make_shared<Expr>();
    expr->range = range_of(token);

    switch (token.kind) {
    case TokenKind::UnsignedInteger:
        expr->kind = ExprKind::Literal;
        expr->literal_type = LiteralType::UnsignedInteger;
        expr->unsigned_value = token.unsigned_value;
        expr->number = token.number;
        advance();
        return expr;
    case TokenKind::Integer:
        expr->kind = ExprKind::Literal;
        expr->literal_type = LiteralType::Integer;
        expr->signed_value = token.signed_value;
        expr->number = token.number;
        advance();
        return expr;
    case TokenKind::Float:
        expr->kind = ExprKind::Literal;
        expr->literal_type = LiteralType::Float;
        expr->number = token.number;
        advance();
        return expr;
    case TokenKind::String:
        expr->kind = ExprKind::Literal;
        expr->literal_type = LiteralType::String;
        expr->text = token.value;
        advance();
        return expr;
    case TokenKind::PhysicalLiteral:
        expr->kind = ExprKind::PhysicalLiteral;
        expr->text = token.value; // the unit name
        expr->number = token.number;
        advance();
        return expr;
    default:
        break;
    }

    if (at_op("(")) {
        advance();
        ExprPtr inner = parse_expression();
        expect_op(")", "§7.2.2.6.6");
        return inner;
    }

    if (at_op("[")) {
        // Either a list constructor or the `[a..b]` range form (§7.2.2.6.7).
        advance();
        ExprPtr first = parse_expression();
        if (at_op("..")) {
            advance();
            expr->kind = ExprKind::RangeConstructor;
            expr->operands = {std::move(first), parse_expression()};
            expect_op("]", "§7.2.2.6.7");
            return expr;
        }
        expr->kind = ExprKind::ListConstructor;
        expr->operands.push_back(std::move(first));
        while (accept_op(",")) {
            expr->operands.push_back(parse_expression());
        }
        expect_op("]", "§7.2.2.6.7");
        return expr;
    }

    if (at_word("range")) {
        advance();
        expect_op("(", "§7.2.2.6.7");
        expr->kind = ExprKind::RangeConstructor;
        expr->operands.push_back(parse_expression());
        expect_op(",", "§7.2.2.6.7");
        expr->operands.push_back(parse_expression());
        expect_op(")", "§7.2.2.6.7");
        return expr;
    }

    if (at_word("true") || at_word("false")) {
        expr->kind = ExprKind::Literal;
        expr->literal_type = LiteralType::Bool;
        expr->boolean = peek().text == "true";
        advance();
        return expr;
    }

    if (at_word("it")) {
        expr->kind = ExprKind::Name;
        expr->text = "it";
        advance();
        return expr;
    }

    if (at(TokenKind::Identifier)) {
        const std::string name = parse_qualified_identifier("an expression", "§7.2.2.6.6");
        // enum-value-reference ::= [enum-name '!'] enum-member-name (§7.2.2.2.2)
        if (at_op("!")) {
            advance();
            expr->kind = ExprKind::EnumValue;
            expr->type_name = name;
            expr->text = parse_qualified_identifier("an enum member name", "§7.2.2.2.2");
            return expr;
        }
        expr->kind = ExprKind::Name;
        expr->text = name;
        return expr;
    }

    fail(token, "expected an expression (§7.2.2.6.6), found " + describe(token));
}

// --- event specifications (§7.2.2.4.1) -------------------------------------

EventSpec Parser::parse_event_spec() {
    EventSpec spec;
    spec.range = range_of(peek());

    if (at_op("@")) {
        advance();
        spec.kind = EventConditionKind::Reference;
        spec.event_path = parse_structured_identifier();
        if (accept_word("as")) {
            spec.binding = parse_qualified_identifier("an event field name", "§7.2.2.4.1");
        }
        if (accept_word("if")) {
            spec.expression = parse_expression();
        }
        return spec;
    }

    // rise / fall / elapsed / every are reserved words used as call-like forms.
    struct Form {
        std::string_view word;
        EventConditionKind kind;
    };
    for (const Form& form :
         {Form{"rise", EventConditionKind::Rise}, Form{"fall", EventConditionKind::Fall},
          Form{"elapsed", EventConditionKind::Elapsed}, Form{"every", EventConditionKind::Every}}) {
        if (!at_word(form.word)) {
            continue;
        }
        advance();
        spec.kind = form.kind;
        expect_op("(", "§7.2.2.4.1");
        spec.expression = parse_expression();
        if (form.kind == EventConditionKind::Every && accept_op(",")) {
            expect_word("offset", "§7.2.2.4.1");
            expect_op(":", "§7.2.2.4.1");
            spec.offset = parse_expression();
        }
        expect_op(")", "§7.2.2.4.1");
        return spec;
    }

    spec.kind = EventConditionKind::Expression;
    spec.expression = parse_expression();
    return spec;
}

// --- constraints and modifiers ---------------------------------------------

Constraint Parser::parse_constraint() {
    Constraint constraint;
    constraint.range = range_of(peek());
    if (accept_word("remove_default")) {
        constraint.is_remove_default = true;
        expect_op("(", "§7.2.2.4.3");
        constraint.expression = parse_expression();
        expect_op(")", "§7.2.2.4.3");
        expect_newline("§7.2.2.4.3");
        return constraint;
    }
    expect_word("keep", "§7.2.2.4.3");
    expect_op("(", "§7.2.2.4.3");
    if (at_word("default") || at_word("hard")) {
        constraint.qualifier = std::string(peek().text);
        advance();
    }
    constraint.expression = parse_expression();
    expect_op(")", "§7.2.2.4.3");
    expect_newline("§7.2.2.4.3");
    return constraint;
}

ModifierApplication Parser::parse_modifier_application(ExprPtr actor, const Token& name_token) {
    ModifierApplication application;
    application.range = range_of(name_token);
    application.actor = std::move(actor);
    application.name = name_token.value.empty() ? std::string(name_token.text) : name_token.value;
    expect_op("(", "§7.2.2.4.6");
    application.arguments = parse_argument_list();
    expect_op(")", "§7.2.2.4.6");
    expect_newline("§7.2.2.4.6");
    return application;
}

// --- behavior specification (§7.2.2.4.7) -----------------------------------

void Parser::parse_behavior_with(BehaviorWith& out) {
    // behavior-with-declaration ::= 'with' ':' NEWLINE INDENT behavior-with-member+ DEDENT
    expect_op(":", "§7.2.2.4.7");
    expect_newline("§7.2.2.4.7");
    expect_indent("§7.2.2.4.7");
    while (!at(TokenKind::Dedent) && !at_end()) {
        try {
            if (at_word("until")) {
                advance();
                out.until.push_back(parse_event_spec());
                expect_newline("§7.2.2.4.7");
            } else if (at_word("keep") || at_word("remove_default")) {
                out.constraints.push_back(parse_constraint());
            } else if (at(TokenKind::Identifier)) {
                // modifier-application, with or without an actor prefix.
                ExprPtr actor;
                Token name_token = peek();
                const std::size_t save = pos_;
                ExprPtr prefix = parse_postfix();
                if (prefix->kind == ExprKind::FieldAccess) {
                    actor = prefix->operands.front();
                    pos_ = save;
                    // Re-read so the modifier's own name token is in hand.
                    while (!at_op("(") && !at(TokenKind::Newline) && !at_end()) {
                        name_token = advance();
                    }
                } else {
                    pos_ = save;
                    name_token = advance();
                }
                out.modifiers.push_back(parse_modifier_application(std::move(actor), name_token));
            } else {
                fail(peek(), "expected a constraint, a modifier application or 'until' "
                             "(§7.2.2.4.7), found " +
                                 describe(peek()));
            }
        } catch (const ParseError&) {
            synchronize_statement();
        }
    }
    expect_dedent("§7.2.2.4.7");
}

DoMemberPtr Parser::parse_do_member() {
    auto member = std::make_shared<DoMember>();
    member->range = range_of(peek());

    // do-member ::= [label-name ':'] (...)
    if (at(TokenKind::Identifier) && !peek().keyword && peek(1).kind == TokenKind::Operator &&
        peek(1).text == ":" && peek(2).kind != TokenKind::Newline) {
        // A label only when what follows is not a block opener on its own line.
        member->label = peek().value.empty() ? std::string(peek().text) : peek().value;
        advance();
        advance();
    } else if (at(TokenKind::Identifier) && peek(1).kind == TokenKind::Operator &&
               peek(1).text == ":" && peek(2).kind == TokenKind::Newline && !peek().keyword) {
        // `label: serial:` — the label is followed by a composition on the same
        // line, so a NEWLINE right after the colon means this was not a label.
        member->label = peek().value.empty() ? std::string(peek().text) : peek().value;
        advance();
        advance();
    }

    // composition ::= composition-operator ['(' args ')'] ':' NEWLINE INDENT do-member+ DEDENT
    struct Composition {
        std::string_view word;
        CompositionOperator op;
    };
    for (const Composition& form : {Composition{"serial", CompositionOperator::Serial},
                                    Composition{"one_of", CompositionOperator::OneOf},
                                    Composition{"parallel", CompositionOperator::Parallel}}) {
        if (!at_word(form.word)) {
            continue;
        }
        advance();
        member->kind = DoMemberKind::Composition;
        member->composition = form.op;
        if (accept_op("(")) {
            member->composition_arguments = parse_argument_list();
            expect_op(")", "§7.2.2.4.7");
        }
        expect_op(":", "§7.2.2.4.7");
        expect_newline("§7.2.2.4.7");
        expect_indent("§7.2.2.4.7");
        while (!at(TokenKind::Dedent) && !at_end()) {
            try {
                member->members.push_back(parse_do_member());
            } catch (const ParseError&) {
                synchronize_statement();
            }
        }
        expect_dedent("§7.2.2.4.7");
        if (at_word("with")) {
            advance();
            parse_behavior_with(member->with);
        }
        return member;
    }

    if (accept_word("wait")) {
        member->kind = DoMemberKind::Wait;
        member->event = parse_event_spec();
        expect_newline("§7.2.2.4.7");
        return member;
    }

    if (accept_word("emit")) {
        member->kind = DoMemberKind::Emit;
        member->emit_name = parse_qualified_identifier("an event name", "§7.2.2.4.7");
        if (accept_op("(")) {
            member->arguments = parse_argument_list();
            expect_op(")", "§7.2.2.4.7");
        }
        expect_newline("§7.2.2.4.7");
        return member;
    }

    if (accept_word("call")) {
        member->kind = DoMemberKind::Call;
        ExprPtr invocation = parse_postfix();
        if (invocation->kind != ExprKind::Call) {
            error(peek(), "'call' needs a method invocation (§7.2.2.4.7)");
        }
        member->actor = invocation;
        expect_newline("§7.2.2.4.7");
        return member;
    }

    // behavior-invocation ::= [actor-expression '.'] behavior-name '(' args ')'
    member->kind = DoMemberKind::Invocation;
    ExprPtr invocation = parse_postfix();
    if (invocation->kind != ExprKind::Call) {
        fail(peek(), "expected a behavior invocation, a composition, 'wait', 'emit' or 'call' "
                     "(§7.2.2.4.7), found " +
                         describe(peek()));
    }
    member->arguments = invocation->arguments;
    const ExprPtr& callee = invocation->operands.front();
    if (callee->kind == ExprKind::FieldAccess) {
        member->actor = callee->operands.front();
        member->name = callee->text;
    } else {
        member->name = callee->text;
    }
    if (at_word("with")) {
        advance();
        parse_behavior_with(member->with);
    } else {
        expect_newline("§7.2.2.4.7");
    }
    return member;
}

OnDirective Parser::parse_on_directive() {
    OnDirective directive;
    directive.range = range_of(peek());
    expect_word("on", "§7.2.2.4.7");
    directive.event = parse_event_spec();
    expect_op(":", "§7.2.2.4.7");
    expect_newline("§7.2.2.4.7");
    expect_indent("§7.2.2.4.7");
    while (!at(TokenKind::Dedent) && !at_end()) {
        try {
            if (!at_word("call") && !at_word("emit")) {
                fail(peek(), "an 'on' directive holds only 'call' and 'emit' members "
                             "(§7.2.2.4.7), found " +
                                 describe(peek()));
            }
            directive.members.push_back(parse_do_member());
        } catch (const ParseError&) {
            synchronize_statement();
        }
    }
    expect_dedent("§7.2.2.4.7");
    return directive;
}

// --- structured-type members (§7.2.2.4) ------------------------------------

bool Parser::parse_member(std::vector<Member>& out) {
    Member member;
    member.range = range_of(peek());

    if (at_word("event")) {
        advance();
        member.kind = Member::Kind::Event;
        member.event.range = member.range;
        member.event.name = parse_qualified_identifier("an event name", "§7.2.2.4.1");
        if (accept_op("(")) {
            member.event.parameters = parse_argument_list_specification();
            expect_op(")", "§7.2.2.4.1");
        }
        if (accept_word("is")) {
            member.event.spec = parse_event_spec();
        }
        expect_newline("§7.2.2.4.1");
        out.push_back(std::move(member));
        return true;
    }

    if (at_word("keep") || at_word("remove_default")) {
        member.kind = Member::Kind::Constraint;
        member.constraint = parse_constraint();
        out.push_back(std::move(member));
        return true;
    }

    if (at_word("def")) {
        advance();
        member.kind = Member::Kind::Method;
        member.method.range = member.range;
        member.method.name = parse_qualified_identifier("a method name", "§7.2.2.4.4");
        expect_op("(", "§7.2.2.4.4");
        member.method.parameters = parse_argument_list_specification();
        expect_op(")", "§7.2.2.4.4");
        if (accept_op("->")) {
            member.method.return_type = parse_type_declarator();
        }
        expect_word("is", "§7.2.2.4.4");
        if (at_word("only")) {
            member.method.qualifier = "only";
            advance();
        }
        if (accept_word("expression")) {
            member.method.implementation = "expression";
            member.method.expression = parse_expression();
        } else if (accept_word("undefined")) {
            member.method.implementation = "undefined";
        } else if (accept_word("external")) {
            member.method.implementation = "external";
            member.method.external_name = parse_structured_identifier();
            expect_op("(", "§7.2.2.4.4");
            member.method.external_arguments = parse_argument_list();
            expect_op(")", "§7.2.2.4.4");
        } else {
            fail(peek(), "a method body is 'expression', 'undefined' or 'external' "
                         "(§7.2.2.4.4), found " +
                             describe(peek()));
        }
        expect_newline("§7.2.2.4.4");
        out.push_back(std::move(member));
        return true;
    }

    if (at_word("cover") || at_word("record")) {
        member.kind = Member::Kind::Coverage;
        member.coverage.range = member.range;
        member.coverage.is_record = peek().text == "record";
        advance();
        expect_op("(", "§7.2.2.4.5");
        member.coverage.arguments = parse_argument_list();
        expect_op(")", "§7.2.2.4.5");
        expect_newline("§7.2.2.4.5");
        out.push_back(std::move(member));
        return true;
    }

    if (at_word("on")) {
        member.kind = Member::Kind::On;
        member.on = parse_on_directive();
        out.push_back(std::move(member));
        return true;
    }

    if (at_word("do")) {
        advance();
        member.kind = Member::Kind::Behavior;
        member.behavior = parse_do_member();
        out.push_back(std::move(member));
        return true;
    }

    if (at_word("var") || at(TokenKind::Identifier)) {
        // field-declaration ::= parameter-declaration | variable-declaration.
        // A modifier application also starts with an identifier, so the two are
        // told apart by what follows the name: ':' is a field, '(' or '.' a
        // modifier application (§7.2.2.4.2 vs §7.2.2.4.6).
        const bool is_variable = at_word("var");
        if (!is_variable) {
            std::size_t ahead = 0;
            while (peek(ahead).kind == TokenKind::Identifier ||
                   (peek(ahead).kind == TokenKind::Operator &&
                    (peek(ahead).text == "," || peek(ahead).text == "::"))) {
                ++ahead;
            }
            if (!(peek(ahead).kind == TokenKind::Operator && peek(ahead).text == ":")) {
                // A modifier application.
                member.kind = Member::Kind::ModifierApplication;
                ExprPtr prefix = parse_postfix();
                if (prefix->kind != ExprKind::Call) {
                    fail(peek(), "expected a field declaration or a modifier application "
                                 "(§7.2.2.4.2, §7.2.2.4.6), found " +
                                     describe(peek()));
                }
                member.modifier.range = member.range;
                member.modifier.arguments = prefix->arguments;
                const ExprPtr& callee = prefix->operands.front();
                if (callee->kind == ExprKind::FieldAccess) {
                    member.modifier.actor = callee->operands.front();
                    member.modifier.name = callee->text;
                } else {
                    member.modifier.name = callee->text;
                }
                expect_newline("§7.2.2.4.6");
                out.push_back(std::move(member));
                return true;
            }
        } else {
            advance();
        }

        member.kind = Member::Kind::Field;
        member.field.range = member.range;
        member.field.is_variable = is_variable;
        do {
            member.field.names.push_back(parse_qualified_identifier("a field name", "§7.2.2.4.2"));
        } while (accept_op(","));
        expect_op(":", "§7.2.2.4.2");
        member.field.type = parse_type_declarator();
        if (accept_op("=")) {
            if (is_variable && at_word("sample")) {
                // sample-expression ::= 'sample' '(' expression ','
                //                        event-specification [',' default] ')'
                advance();
                expect_op("(", "§7.2.2.4.2");
                member.field.is_sampled = true;
                member.field.default_value = parse_expression();
                expect_op(",", "§7.2.2.4.2");
                member.field.sample_event = parse_event_spec();
                if (accept_op(",")) {
                    // The third argument is the default; kept on the field.
                    member.field.default_value = parse_expression();
                }
                expect_op(")", "§7.2.2.4.2");
            } else {
                member.field.default_value = parse_expression();
            }
        }
        if (!is_variable && at_word("with")) {
            advance();
            // parameter-with-declaration holds constraint declarations only.
            expect_op(":", "§7.2.2.4.2");
            expect_newline("§7.2.2.4.2");
            expect_indent("§7.2.2.4.2");
            while (!at(TokenKind::Dedent) && !at_end()) {
                try {
                    member.field.constraints.push_back(parse_constraint());
                } catch (const ParseError&) {
                    synchronize_statement();
                }
            }
            expect_dedent("§7.2.2.4.2");
        } else {
            expect_newline("§7.2.2.4.2");
        }
        out.push_back(std::move(member));
        return true;
    }

    return false;
}

void Parser::parse_structured_body(StructuredDecl& decl) {
    // Every structured type ends either with ':' NEWLINE INDENT members DEDENT
    // or with a bare NEWLINE — an empty declaration is legal (§7.2.2.2.4).
    if (!accept_op(":")) {
        expect_newline("§7.2.2.2.4");
        return;
    }
    expect_newline("§7.2.2.2.4");
    expect_indent("§7.2.2.2.4");
    while (!at(TokenKind::Dedent) && !at_end()) {
        try {
            if (!parse_member(decl.members)) {
                fail(peek(), "expected a member declaration (§7.2.2.4), found " + describe(peek()));
            }
        } catch (const ParseError&) {
            synchronize_statement();
        }
    }
    expect_dedent("§7.2.2.2.4");
}

// --- declarations ----------------------------------------------------------

std::vector<SiExponent> Parser::parse_si_specifier(std::optional<double>& factor,
                                                   std::optional<double>& offset) {
    // SI-unit-specifier ::= 'SI' '(' SI-base-exponent-list [',' factor] [',' offset] ')'
    std::vector<SiExponent> exponents;
    expect_word("SI", "§7.2.2.2.1");
    expect_op("(", "§7.2.2.2.1");
    do {
        if (at_word("factor") || at_word("offset")) {
            const bool is_factor = peek().text == "factor";
            advance();
            expect_op(":", "§7.2.2.2.1");
            const Token& value = peek();
            if (value.kind != TokenKind::Float && value.kind != TokenKind::UnsignedInteger &&
                value.kind != TokenKind::Integer) {
                fail(value, "a factor or offset is a numeric literal (§7.2.2.2.1), found " +
                                describe(value));
            }
            advance();
            (is_factor ? factor : offset) = value.number;
            continue;
        }
        SiExponent exponent;
        exponent.range = range_of(peek());
        exponent.unit = parse_qualified_identifier("an SI base unit name", "§7.2.2.2.1");
        expect_op(":", "§7.2.2.2.1");
        const Token& value = peek();
        if (value.kind == TokenKind::UnsignedInteger) {
            exponent.exponent = static_cast<std::int64_t>(value.unsigned_value);
        } else if (value.kind == TokenKind::Integer) {
            exponent.exponent = value.signed_value;
        } else {
            fail(value, "an SI base exponent is an integer literal (§7.2.2.2.1), found " +
                            describe(value));
        }
        advance();
        exponents.push_back(std::move(exponent));
    } while (accept_op(","));
    expect_op(")", "§7.2.2.2.1");
    return exponents;
}

void Parser::parse_import() {
    Declaration declaration;
    declaration.kind = Declaration::Kind::Import;
    declaration.range = range_of(peek());
    declaration.import.range = declaration.range;
    advance(); // 'import'
    if (at(TokenKind::String)) {
        declaration.import.reference = peek().value;
        declaration.import.is_path = true;
        advance();
    } else {
        declaration.import.reference = parse_structured_identifier();
    }
    expect_newline("§7.2.2.1.1");
    out_.declarations.push_back(std::move(declaration));
}

void Parser::parse_namespace() {
    Declaration declaration;
    declaration.kind = Declaration::Kind::Namespace;
    declaration.range = range_of(peek());
    declaration.name_space.range = declaration.range;
    advance(); // 'namespace'
    // namespace-name ::= identifier | 'null' (the global namespace).
    if (at_word("null")) {
        declaration.name_space.name = "null";
        advance();
    } else {
        declaration.name_space.name = parse_qualified_identifier("a namespace name", "§7.2.2.1.2");
    }
    if (accept_word("use")) {
        do {
            if (at_word("null")) {
                declaration.name_space.uses.emplace_back("null");
                advance();
            } else {
                declaration.name_space.uses.push_back(
                    parse_qualified_identifier("a namespace name", "§7.2.2.1.2"));
            }
        } while (accept_op(","));
    }
    expect_newline("§7.2.2.1.2");
    out_.declarations.push_back(std::move(declaration));
}

void Parser::parse_export() {
    Declaration declaration;
    declaration.kind = Declaration::Kind::Export;
    declaration.range = range_of(peek());
    declaration.export_decl.range = declaration.range;
    advance(); // 'export'
    do {
        // export-wildcard-specification ::= [[namespace-name] '::'] '*'
        std::string name;
        if (at(TokenKind::Identifier) && peek(1).kind == TokenKind::Operator &&
            peek(1).text == "::" && peek(2).kind == TokenKind::Operator && peek(2).text == "*") {
            name = std::string(advance().text);
            advance();
            advance();
            name += "::*";
        } else if (at_op("::")) {
            advance();
            expect_op("*", "§7.2.2.1.2");
            name = "::*";
        } else if (at_op("*")) {
            advance();
            name = "*";
        } else {
            name = parse_qualified_identifier("an exported name", "§7.2.2.1.2");
        }
        declaration.export_decl.names.push_back(std::move(name));
    } while (accept_op(","));
    expect_newline("§7.2.2.1.2");
    out_.declarations.push_back(std::move(declaration));
}

void Parser::parse_physical_type() {
    Declaration declaration;
    declaration.kind = Declaration::Kind::PhysicalType;
    declaration.range = range_of(peek());
    declaration.physical_type.range = declaration.range;
    advance(); // 'type'
    declaration.physical_type.name =
        parse_qualified_identifier("a physical type name", "§7.2.2.2.1");
    expect_word("is", "§7.2.2.2.1");
    std::optional<double> factor;
    std::optional<double> offset;
    declaration.physical_type.exponents = parse_si_specifier(factor, offset);
    if (factor.has_value() || offset.has_value()) {
        // base-unit-specifier is SI-base-unit-specifier, which has neither
        // (§7.2.2.2.1) — only a unit declaration carries them.
        error(peek(), "a physical type's base unit takes no factor or offset (§7.2.2.2.1); "
                      "those belong on a unit declaration");
    }
    expect_newline("§7.2.2.2.1");
    out_.declarations.push_back(std::move(declaration));
}

void Parser::parse_unit() {
    Declaration declaration;
    declaration.kind = Declaration::Kind::Unit;
    declaration.range = range_of(peek());
    declaration.unit.range = declaration.range;
    advance(); // 'unit'
    declaration.unit.name = parse_qualified_identifier("a unit name", "§7.2.2.2.1");
    expect_word("of", "§7.2.2.2.1");
    declaration.unit.physical_type =
        parse_qualified_identifier("a physical type name", "§7.2.2.2.1");
    expect_word("is", "§7.2.2.2.1");
    declaration.unit.exponents =
        parse_si_specifier(declaration.unit.factor, declaration.unit.offset);
    expect_newline("§7.2.2.2.1");
    out_.declarations.push_back(std::move(declaration));
}

void Parser::parse_enum(bool is_extension) {
    Declaration declaration;
    declaration.kind = Declaration::Kind::Enum;
    declaration.range = range_of(peek());
    declaration.enumeration.range = declaration.range;
    declaration.enumeration.is_extension = is_extension;
    advance(); // 'enum' (or the name after 'extend')
    if (is_extension) {
        --pos_; // 'extend' already consumed the keyword; the name is here
    }
    declaration.enumeration.name = parse_qualified_identifier("an enum name", "§7.2.2.2.2");
    expect_op(":", "§7.2.2.2.2");
    expect_op("[", "§7.2.2.2.2");
    do {
        EnumMember member;
        member.range = range_of(peek());
        member.name = parse_qualified_identifier("an enum member name", "§7.2.2.2.2");
        if (accept_op("=")) {
            const Token& value = peek();
            if (value.kind == TokenKind::UnsignedInteger) {
                member.value = static_cast<std::int64_t>(value.unsigned_value);
                advance();
            } else if (value.kind == TokenKind::Integer) {
                member.value = value.signed_value;
                advance();
            } else {
                // enum-member-value ::= uint-literal | hex-uint-literal |
                // enum-value-reference (§7.2.2.2.2). A reference names another
                // member, of this or another enumeration; what it stands for is
                // a symbol-table question (§7.3.3), so it is kept as written.
                const ExprPtr reference = parse_expression();
                if (reference && reference->kind == ExprKind::EnumValue) {
                    member.value_reference = reference->type_name + "!" + reference->text;
                } else if (reference && reference->kind == ExprKind::Name) {
                    member.value_reference = reference->text;
                } else {
                    error(value, "an enum member value is an unsigned literal or a reference to "
                                 "another enum member (§7.2.2.2.2), found " +
                                     describe(value));
                }
            }
        }
        declaration.enumeration.members.push_back(std::move(member));
    } while (accept_op(","));
    expect_op("]", "§7.2.2.2.2");
    expect_newline("§7.2.2.2.2");
    out_.declarations.push_back(std::move(declaration));
}

void Parser::parse_structured(StructuredKind kind) {
    Declaration declaration;
    declaration.kind = Declaration::Kind::Structured;
    declaration.range = range_of(peek());
    declaration.structured.kind = kind;
    declaration.structured.range = declaration.range;
    advance(); // the introducing keyword

    declaration.structured.name = parse_qualified_identifier("a type name", "§7.2.2.2.4");
    // qualified-behavior-name ::= [actor-name '.'] behavior-name.
    while (at_op(".")) {
        advance();
        declaration.structured.name +=
            "." + parse_qualified_identifier("a behavior name", "§7.2.2.2.4");
    }

    if (kind == StructuredKind::Modifier && accept_word("of")) {
        declaration.structured.modifies =
            parse_qualified_identifier("a behavior name", "§7.2.2.2.5");
        while (at_op(".")) {
            advance();
            declaration.structured.modifies +=
                "." + parse_qualified_identifier("a behavior name", "§7.2.2.2.5");
        }
    }

    if (accept_word("inherits")) {
        declaration.structured.base = parse_qualified_identifier("a base type name", "§7.3.8");
        while (at_op(".")) {
            advance();
            declaration.structured.base +=
                "." + parse_qualified_identifier("a behavior name", "§7.3.8");
        }
        if (accept_op("(")) {
            // Conditional inheritance: inherits X(field == value) (§7.3.8).
            declaration.structured.constraint_field =
                parse_qualified_identifier("a field name", "§7.3.8");
            expect_op("==", "§7.3.8");
            declaration.structured.constraint_value = parse_expression();
            expect_op(")", "§7.3.8");
        }
    }

    parse_structured_body(declaration.structured);
    out_.declarations.push_back(std::move(declaration));
}

void Parser::parse_extend() {
    // type-extension ::= enum-type-extension | structured-type-extension |
    //                    primitive-type-extension (§7.2.2.2.6)
    const std::size_t save = pos_;
    advance(); // 'extend'
    const std::string name = parse_qualified_identifier("a type name", "§7.2.2.2.6");
    if (at_op(":") && peek(1).kind == TokenKind::Operator && peek(1).text == "[") {
        // An enum extension. Rewind so parse_enum reads the name itself.
        pos_ = save + 1;
        parse_enum(/*is_extension=*/true);
        return;
    }
    Declaration declaration;
    declaration.kind = Declaration::Kind::Structured;
    declaration.range = range_of(tokens_[save]);
    declaration.structured.kind = StructuredKind::Extension;
    declaration.structured.range = declaration.range;
    declaration.structured.name = name;
    parse_structured_body(declaration.structured);
    out_.declarations.push_back(std::move(declaration));
}

void Parser::parse_global_parameter() {
    Declaration declaration;
    declaration.kind = Declaration::Kind::GlobalParameter;
    declaration.range = range_of(peek());
    declaration.global_parameter.range = declaration.range;
    advance(); // 'global'
    std::vector<Member> members;
    if (!parse_member(members) || members.empty() || members.front().kind != Member::Kind::Field) {
        fail(peek(),
             "'global' introduces a parameter declaration (§7.2.2.3), found " + describe(peek()));
    }
    declaration.global_parameter.field = members.front().field;
    out_.declarations.push_back(std::move(declaration));
}

void Parser::parse_declaration() {
    if (at_word("import")) {
        parse_import();
    } else if (at_word("namespace")) {
        parse_namespace();
    } else if (at_word("export")) {
        parse_export();
    } else if (at_word("type")) {
        parse_physical_type();
    } else if (at_word("unit")) {
        parse_unit();
    } else if (at_word("enum")) {
        parse_enum(/*is_extension=*/false);
    } else if (at_word("struct")) {
        parse_structured(StructuredKind::Struct);
    } else if (at_word("actor")) {
        parse_structured(StructuredKind::Actor);
    } else if (at_word("scenario")) {
        parse_structured(StructuredKind::Scenario);
    } else if (at_word("action")) {
        parse_structured(StructuredKind::Action);
    } else if (at_word("modifier")) {
        parse_structured(StructuredKind::Modifier);
    } else if (at_word("extend")) {
        parse_extend();
    } else if (at_word("global")) {
        parse_global_parameter();
    } else {
        fail(peek(), "expected a top-level declaration — import, namespace, export, type, unit, "
                     "enum, struct, actor, scenario, action, modifier, extend or global "
                     "(§7.2.2.1), found " +
                         describe(peek()));
    }
}

Status Parser::run() {
    // Prelude statements must precede all others (§7.2.2.1.1); which one a file
    // used is checked here so a misplaced import is reported where it is,
    // rather than silently accepted.
    bool seen_main = false;
    while (!at_end()) {
        if (at(TokenKind::Newline) || at(TokenKind::Indent) || at(TokenKind::Dedent)) {
            advance();
            continue;
        }
        const bool is_import = at_word("import");
        if (is_import && seen_main) {
            error(peek(), "import statements must come before every other statement "
                          "(§7.2.2.1.1)");
        }
        seen_main = seen_main || !is_import;
        try {
            parse_declaration();
        } catch (const ParseError&) {
            synchronize_statement();
        }
    }
    return failed_ ? Status::ValidationError : Status::Ok;
}

} // namespace

Status parse(const std::vector<Token>& tokens, const std::string& file, File& out,
             DiagnosticSink& sink) {
    out.declarations.clear();
    out.path = file;
    if (tokens.empty()) {
        return Status::Ok;
    }
    Parser parser(tokens, file, out, sink);
    return parser.run();
}

Status parse_source(std::string_view source, const std::string& file, File& out,
                    DiagnosticSink& sink) {
    std::vector<Token> tokens;
    // A lexical error does not prevent parsing: the token stream is well formed
    // either way, so both sets of diagnostics reach the caller in one run.
    const Status lexed = lex(source, file, tokens, sink);
    const Status parsed = parse(tokens, file, out, sink);
    return lexed != Status::Ok ? lexed : parsed;
}

Status parse_source(std::string_view source, File& out, DiagnosticSink& sink) {
    return parse_source(source, std::string{}, out, sink);
}

} // namespace scena::dsl
