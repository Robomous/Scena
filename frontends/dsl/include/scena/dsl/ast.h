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
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace scena::dsl {

/// Where a construct came from, for diagnostics. 1-based, 0 meaning unknown.
struct SourceRange {
    int line = 0;
    int column = 0;
};

// --- expressions (§7.2.2.6) ------------------------------------------------

struct Expr;
using ExprPtr = std::shared_ptr<const Expr>;

/// Expression node kinds, following §7.2.2.6's productions.
///
/// One node type with a kind tag rather than a class per production: the tree
/// is walked far more often than it is built (the type checker, the constant
/// evaluator and the lowering all walk it), and a uniform node keeps those
/// walks to a single switch instead of three visitor hierarchies.
enum class ExprKind : std::uint8_t {
    /// A `uint`, `int`, `float`, `bool` or `string` literal (§7.2.2.6.6).
    Literal,
    /// A number with a unit name (§7.2.1.5.2). `text` is the unit.
    PhysicalLiteral,
    /// `qualified-identifier`, or `it` (§7.2.2.6.6). `text` is the name.
    Name,
    /// `[enum-name '!'] enum-member-name` (§7.2.2.2.2). `text` is the member,
    /// `type_name` the optional enum name.
    EnumValue,
    /// A unary operator: `not` or arithmetic negation. `text` is the spelling.
    Unary,
    /// A binary operator (§7.2.2.6.2–7.2.2.6.4). `text` is the spelling.
    Binary,
    /// `implication '?' expression ':' expression` (§7.2.2.6.1).
    Ternary,
    /// `postfix-exp '.' field-name` (§7.2.2.6.5). `text` is the field.
    FieldAccess,
    /// `postfix-exp '[' expression ']'` (§7.2.2.6.5).
    ElementAccess,
    /// `postfix-exp '(' [argument-list] ')'` (§7.2.2.6.5). Arguments are in
    /// `arguments`.
    Call,
    /// `postfix-exp '.' 'as' '(' type-declarator ')'` (§7.2.2.6.5).
    Cast,
    /// `postfix-exp '.' 'is' '(' type-declarator ')'` (§7.2.2.6.5).
    TypeTest,
    /// `'[' expression (',' expression)* ']'` (§7.2.2.6.7).
    ListConstructor,
    /// `range(a, b)` or `[a..b]` (§7.2.2.6.7).
    RangeConstructor,
};

/// The literal types a `Literal` node can hold.
enum class LiteralType : std::uint8_t { Bool, UnsignedInteger, Integer, Float, String };

/// One argument of a call, a behavior invocation or a modifier application
/// (§7.2.2.5.2). A positional argument has an empty `name`.
struct Argument {
    std::string name;
    ExprPtr value;
    SourceRange range;
};

/// A type declarator (§7.2.2.2), kept as written so the type system can resolve
/// it against the symbol table rather than the parser guessing.
struct TypeRef {
    /// `int`, `uint`, `float`, `bool`, `string`, or a user-defined qualified
    /// name. Empty when the declarator is purely aggregate.
    std::string name;
    /// `list of X` (§7.2.2.2.3).
    bool is_list = false;
    /// `range of X` (§7.2.2.2.3).
    bool is_range = false;
    SourceRange range;
};

struct Expr {
    ExprKind kind = ExprKind::Literal;
    SourceRange range;

    /// Literal payload, meaningful when `kind == Literal` or PhysicalLiteral.
    LiteralType literal_type = LiteralType::Float;
    double number = 0.0;
    std::uint64_t unsigned_value = 0;
    std::int64_t signed_value = 0;
    bool boolean = false;

    /// A name, an operator spelling, a field name, a unit name, or an enum
    /// member — whichever the kind calls for.
    std::string text;
    /// The enum name of an `EnumValue`, or the target type of a Cast/TypeTest.
    std::string type_name;
    /// The declarator of a Cast or TypeTest.
    std::optional<TypeRef> type;

    std::vector<ExprPtr> operands;
    std::vector<Argument> arguments;
};

// --- event specifications (§7.2.2.4.1) -------------------------------------

/// How an event specification is written.
enum class EventConditionKind : std::uint8_t {
    /// `@event-path [as name] [if condition]`.
    Reference,
    /// A plain boolean expression.
    Expression,
    /// `rise(expr)`.
    Rise,
    /// `fall(expr)`.
    Fall,
    /// `elapsed(duration)`.
    Elapsed,
    /// `every(duration [, offset: duration])`. Post-v0.0.1; parsed and checked.
    Every,
};

struct EventSpec {
    EventConditionKind kind = EventConditionKind::Expression;
    /// The event path of a Reference.
    std::string event_path;
    /// The `as` binding of a Reference (§7.2.2.4.1 event-field-decl).
    std::string binding;
    /// The condition expression, the `if` guard of a Reference, or the
    /// duration of Elapsed/Every.
    ExprPtr expression;
    /// `every`'s optional offset.
    ExprPtr offset;
    SourceRange range;
};

// --- behavior specification (§7.2.2.4.7) -----------------------------------

struct DoMember;
using DoMemberPtr = std::shared_ptr<DoMember>;

/// §7.2.2.4.7's composition operators.
enum class CompositionOperator : std::uint8_t { Serial, OneOf, Parallel };

enum class DoMemberKind : std::uint8_t {
    /// `serial:` / `parallel:` / `one_of:` with nested members.
    Composition,
    /// `[actor '.'] behavior-name '(' args ')' [with:]`.
    Invocation,
    /// `wait event-specification`.
    Wait,
    /// `emit event-name [ '(' args ')' ]`.
    Emit,
    /// `call method-invocation`.
    Call,
};

/// A `keep(...)` or `remove_default(...)` constraint (§7.2.2.4.3).
struct Constraint {
    /// `default`, `hard`, or empty for an unqualified keep.
    std::string qualifier;
    /// True for `remove_default`, in which case `expression` is the parameter
    /// reference.
    bool is_remove_default = false;
    ExprPtr expression;
    SourceRange range;
};

/// A modifier application (§7.2.2.4.6).
struct ModifierApplication {
    /// The actor expression before the `.`, if written.
    ExprPtr actor;
    std::string name;
    std::vector<Argument> arguments;
    SourceRange range;
};

/// The `with:` block of a behavior invocation (§7.2.2.4.7).
struct BehaviorWith {
    std::vector<Constraint> constraints;
    std::vector<ModifierApplication> modifiers;
    /// `until event-specification`.
    std::vector<EventSpec> until;
};

struct DoMember {
    DoMemberKind kind = DoMemberKind::Invocation;
    /// The optional `label-name ':'` prefix.
    std::string label;
    SourceRange range;

    // Composition.
    CompositionOperator composition = CompositionOperator::Serial;
    std::vector<Argument> composition_arguments;
    std::vector<DoMemberPtr> members;

    // Invocation and Call.
    ExprPtr actor;
    std::string name;
    std::vector<Argument> arguments;
    BehaviorWith with;

    // Wait and Emit.
    EventSpec event;
    std::string emit_name;
};

/// An `on event-specification:` directive (§7.2.2.4.7).
struct OnDirective {
    EventSpec event;
    /// `emit` and `call` members, kept as DoMembers because they are the same
    /// two productions.
    std::vector<DoMemberPtr> members;
    SourceRange range;
};

// --- structured-type members (§7.2.2.4) ------------------------------------

/// A field: a parameter or a `var` variable (§7.2.2.4.2).
struct Field {
    /// One declaration may name several fields: `a, b: int`.
    std::vector<std::string> names;
    TypeRef type;
    bool is_variable = false;
    ExprPtr default_value;
    /// `sample(expr, event-spec [, default])` (§7.3.10.4). Post-v0.0.1 for
    /// execution; parsed and checked.
    bool is_sampled = false;
    std::optional<EventSpec> sample_event;
    /// A parameter's `with:` block carries constraints (§7.2.2.4.2).
    std::vector<Constraint> constraints;
    SourceRange range;
};

/// An `event` declaration (§7.2.2.4.1).
struct EventDecl {
    std::string name;
    std::vector<Argument> parameters; ///< argument-list-specification
    std::optional<EventSpec> spec;    ///< the `is ...` formula
    SourceRange range;
};

/// A `def` method (§7.2.2.4.4).
struct MethodDecl {
    std::string name;
    std::vector<Argument> parameters;
    std::optional<TypeRef> return_type;
    /// `only`, or empty.
    std::string qualifier;
    /// `expression`, `undefined` or `external`.
    std::string implementation;
    /// The body of an `is expression ...` method.
    ExprPtr expression;
    /// The name and arguments of an `is external f(...)` method.
    std::string external_name;
    std::vector<Argument> external_arguments;
    SourceRange range;
};

/// A `cover(...)` or `record(...)` declaration (§7.2.2.4.5).
struct CoverageDecl {
    bool is_record = false;
    std::vector<Argument> arguments;
    SourceRange range;
};

/// Everything a structured type may contain. Not every kind is legal in every
/// container — that is a §7.3.5 question the type system answers, and keeping
/// one member type means the parser reports "not allowed here" with the member
/// in hand rather than failing to parse it at all.
struct Member {
    enum class Kind : std::uint8_t {
        Field,
        Event,
        Constraint,
        Method,
        Coverage,
        ModifierApplication,
        Behavior,
        On,
    } kind = Kind::Field;

    struct Field field;
    EventDecl event;
    struct Constraint constraint;
    MethodDecl method;
    CoverageDecl coverage;
    struct ModifierApplication modifier;
    DoMemberPtr behavior; ///< the `do` directive's single member
    OnDirective on;
    SourceRange range;
};

// --- declarations (§7.2.2.1.2) ---------------------------------------------

/// One SI base exponent of a unit specifier (§7.2.2.2.1).
struct SiExponent {
    std::string unit; ///< kg, m, s, A, K, mol, cd, rad
    std::int64_t exponent = 0;
    SourceRange range;
};

/// `type X is SI(...)` (§7.2.2.2.1).
struct PhysicalTypeDecl {
    std::string name;
    std::vector<SiExponent> exponents;
    SourceRange range;
};

/// `unit u of X is SI(..., factor: f, offset: o)` (§7.2.2.2.1).
struct UnitDecl {
    std::string name;
    std::string physical_type;
    std::vector<SiExponent> exponents;
    std::optional<double> factor;
    std::optional<double> offset;
    SourceRange range;
};

/// One `enum-member-decl` (§7.2.2.2.2).
struct EnumMember {
    std::string name;
    /// The explicit `= value`, when written as a literal.
    std::optional<std::int64_t> value;
    /// The explicit `= other` when written as an enum-value-reference: either
    /// a bare member name or `enum-name!member-name`. Resolved against the
    /// symbol table (§7.3.3), which is also where reference cycles are caught.
    std::string value_reference;
    SourceRange range;
};

/// `enum E: [a, b = 2]`, or `extend E: [...]` when `is_extension`.
struct EnumDecl {
    std::string name;
    std::vector<EnumMember> members;
    bool is_extension = false;
    SourceRange range;
};

/// The structured types of §7.2.2.2.4, plus modifiers (§7.2.2.2.5) and
/// extensions (§7.2.2.2.6) — one node, because they share a body grammar and
/// differ only in which members are legal.
enum class StructuredKind : std::uint8_t { Struct, Actor, Scenario, Action, Modifier, Extension };

struct StructuredDecl {
    StructuredKind kind = StructuredKind::Struct;
    /// The declared name. For a scenario or action this is the
    /// `qualified-behavior-name`, actor prefix included.
    std::string name;
    /// The `inherits X` base, if any.
    std::string base;
    /// A conditional inheritance guard: `inherits X(field == value)`
    /// (§7.3.8). `constraint_field` names the field.
    std::string constraint_field;
    ExprPtr constraint_value;
    /// A modifier's `of qualified-behavior-name`.
    std::string modifies;
    std::vector<Member> members;
    SourceRange range;
};

/// `global parameter-declaration` (§7.2.2.3).
struct GlobalParameterDecl {
    Field field;
    SourceRange range;
};

/// `import` (§7.2.2.1.1).
struct ImportDecl {
    /// The structured identifier or the string-literal path, as written.
    std::string reference;
    /// True when the reference was a string literal (a file path) rather than
    /// a dotted identifier.
    bool is_path = false;
    SourceRange range;
};

/// `namespace N [use A, B]` (§7.2.2.1.2).
struct NamespaceDecl {
    std::string name;
    std::vector<std::string> uses;
    SourceRange range;
};

/// `export a, b::*` (§7.2.2.1.2).
struct ExportDecl {
    std::vector<std::string> names;
    SourceRange range;
};

/// One top-level statement.
struct Declaration {
    enum class Kind : std::uint8_t {
        Import,
        Namespace,
        Export,
        PhysicalType,
        Unit,
        Enum,
        Structured,
        GlobalParameter,
    } kind = Kind::Import;

    ImportDecl import;
    NamespaceDecl name_space;
    ExportDecl export_decl;
    PhysicalTypeDecl physical_type;
    UnitDecl unit;
    EnumDecl enumeration;
    StructuredDecl structured;
    GlobalParameterDecl global_parameter;
    SourceRange range;
};

/// A parsed `.osc` file (§7.2.2.1): prelude statements then main statements.
struct File {
    std::string path;
    std::vector<Declaration> declarations;
    /// True for a file the implementation bundles as the §8.16 standard
    /// library. §7.7.4 reserves the `std`-prefixed namespaces for the standard,
    /// so only such a file may declare one without being warned about it.
    bool is_standard_library = false;
};

} // namespace scena::dsl
