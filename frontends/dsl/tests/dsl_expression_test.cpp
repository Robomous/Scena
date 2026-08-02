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
// Expression typing and constant evaluation (p7-s4, #42): §7.4's operators,
// the §7.4.2.3.1 numeric conversion rules, `is()`/`as()`, list and range
// operators, and what a constant context can fold.
//
// Every source fragment is written from the specification text (ADR-0002).
//

#include <string>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

#include "scena/diagnostic.h"
#include "scena/dsl/expression.h"
#include "scena/dsl/parser.h"
#include "scena/dsl/resolve.h"
#include "scena/dsl/types.h"
#include "scena/status.h"

namespace {

using scena::DiagnosticSink;
using scena::Severity;
using scena::Status;
using scena::dsl::builtin_prelude;
using scena::dsl::ExpressionContext;
using scena::dsl::File;
using scena::dsl::kInvalidType;
using scena::dsl::Program;
using scena::dsl::TypeId;
using scena::dsl::Value;

/// A program with the prelude and a `host` scenario carrying `members`.
/// Everything an expression needs is reachable from `host`.
Program host_program(std::string_view members, bool expect_ok = true) {
    const std::string source = std::string(builtin_prelude()) +
                               "enum side: [left, right]\n"
                               "struct point:\n"
                               "    x: float\n"
                               "actor vehicle:\n"
                               "    length: float\n"
                               "    s: side\n"
                               "actor car inherits vehicle(s == left)\n"
                               "scenario host:\n" +
                               std::string(members);
    DiagnosticSink sink;
    File file;
    (void)scena::dsl::parse_source(source, file, sink);
    Program program;
    const Status status = scena::dsl::resolve(file, program, sink);
    if (expect_ok) {
        for (const scena::Diagnostic& diagnostic : sink.diagnostics()) {
            EXPECT_NE(diagnostic.severity, Severity::Error) << diagnostic.message;
        }
        EXPECT_EQ(status, Status::Ok);
    }
    return program;
}

/// The diagnostics of resolving a `host` scenario with `members`.
std::vector<scena::Diagnostic> host_diagnostics(std::string_view members) {
    const std::string source = std::string(builtin_prelude()) +
                               "enum side: [left, right]\n"
                               "struct point:\n"
                               "    x: float\n"
                               "actor vehicle:\n"
                               "    length: float\n"
                               "    s: side\n"
                               "actor car inherits vehicle(s == left)\n"
                               "scenario host:\n" +
                               std::string(members);
    DiagnosticSink sink;
    File file;
    (void)scena::dsl::parse_source(source, file, sink);
    Program program;
    (void)scena::dsl::resolve(file, program, sink);
    return sink.diagnostics();
}

bool mentions(const std::vector<scena::Diagnostic>& diagnostics, std::string_view needle) {
    for (const scena::Diagnostic& diagnostic : diagnostics) {
        if (diagnostic.message.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

bool has_error(const std::vector<scena::Diagnostic>& diagnostics) {
    for (const scena::Diagnostic& diagnostic : diagnostics) {
        if (diagnostic.severity == Severity::Error) {
            return true;
        }
    }
    return false;
}

/// Parses `expression` on its own and types it against `program`'s `host`.
TypeId type_of_expression(Program& program, std::string_view expression, DiagnosticSink& sink) {
    const std::string source = "struct probe:\n    f: int = " + std::string(expression) + "\n";
    DiagnosticSink parse_sink;
    File file;
    EXPECT_EQ(scena::dsl::parse_source(source, file, parse_sink), Status::Ok);
    const auto& probe = file.declarations.front().structured;
    ExpressionContext context;
    const auto host = program.types_by_name.find("::host");
    context.self = host == program.types_by_name.end() ? kInvalidType : host->second;
    return scena::dsl::type_of(program, *probe.members.front().field.default_value, context, sink);
}

/// The type name an expression takes, or "" when it did not type.
std::string type_name_of(Program& program, std::string_view expression) {
    DiagnosticSink sink;
    const TypeId id = type_of_expression(program, expression, sink);
    return id == kInvalidType ? std::string() : program.types[id].name;
}

/// Folds an expression in a constant context.
bool fold(const Program& program, std::string_view expression, Value& out) {
    const std::string source = "struct probe:\n    f: int = " + std::string(expression) + "\n";
    DiagnosticSink sink;
    File file;
    EXPECT_EQ(scena::dsl::parse_source(source, file, sink), Status::Ok);
    const auto& probe = file.declarations.front().structured;
    ExpressionContext context;
    return scena::dsl::evaluate_constant(program, *probe.members.front().field.default_value,
                                         context, out);
}

// --- atoms (§7.4.1) --------------------------------------------------------

TEST(DslExpressionTest, LiteralsTakeTheirSpecifiedTypes) {
    Program program = host_program("    a: int\n");
    EXPECT_EQ(type_name_of(program, "true"), "bool");
    EXPECT_EQ(type_name_of(program, "42"), "uint");
    EXPECT_EQ(type_name_of(program, "-42"), "int");
    EXPECT_EQ(type_name_of(program, "1.5"), "float");
    EXPECT_EQ(type_name_of(program, "\"text\""), "string");
    EXPECT_EQ(type_name_of(program, "30.0kph"), "::speed");
}

TEST(DslExpressionTest, AnIdentifierTakesTheTypeOfWhatItNames) {
    // §7.4.1.1: "Identifiers that occur in expressions are evaluated to the
    // values of the fields that they identify."
    Program program = host_program("    v: vehicle\n    n: int\n");
    EXPECT_EQ(type_name_of(program, "n"), "int");
    EXPECT_EQ(type_name_of(program, "v"), "::vehicle");
    EXPECT_EQ(type_name_of(program, "v.length"), "float");
}

TEST(DslExpressionTest, AnUnknownNameIsReported) {
    Program program = host_program("    a: int\n");
    DiagnosticSink sink;
    EXPECT_EQ(type_of_expression(program, "nowhere", sink), kInvalidType);
    EXPECT_TRUE(mentions(sink.diagnostics(), "unknown name 'nowhere'"));
}

TEST(DslExpressionTest, ItRefersToTheEnclosingInstance) {
    // §7.4.1.3.
    Program program = host_program("    a: int\n");
    EXPECT_EQ(type_name_of(program, "it"), "::host");
}

TEST(DslExpressionTest, AnEnumMemberIsALiteralOfItsType) {
    Program program = host_program("    a: int\n");
    EXPECT_EQ(type_name_of(program, "left"), "::side");
    EXPECT_EQ(type_name_of(program, "side!right"), "::side");
}

TEST(DslExpressionTest, AnOverloadedEnumLiteralNeedsItsEnumName) {
    // §7.3.3: "If this ambiguity cannot be resolved uniquely ... an error is
    // signaled."
    const std::string source = std::string(builtin_prelude()) +
                               "enum a: [shared]\nenum b: [shared]\n"
                               "scenario host:\n    keep(shared == shared)\n";
    DiagnosticSink sink;
    File file;
    (void)scena::dsl::parse_source(source, file, sink);
    Program program;
    EXPECT_EQ(scena::dsl::resolve(file, program, sink), Status::ValidationError);
    EXPECT_TRUE(mentions(sink.diagnostics(), "more than one enum"));
}

// --- logical operators (§7.4.2.2) ------------------------------------------

TEST(DslExpressionTest, LogicalOperatorsTakeAndYieldBooleans) {
    Program program = host_program("    a: bool\n    b: bool\n");
    EXPECT_EQ(type_name_of(program, "a and b"), "bool");
    EXPECT_EQ(type_name_of(program, "a or b"), "bool");
    EXPECT_EQ(type_name_of(program, "a => b"), "bool");
    EXPECT_EQ(type_name_of(program, "not a"), "bool");
}

TEST(DslExpressionTest, ALogicalOperatorOverNonBooleansIsReported) {
    Program program = host_program("    n: int\n");
    DiagnosticSink sink;
    EXPECT_EQ(type_of_expression(program, "n and n", sink), kInvalidType);
    EXPECT_TRUE(mentions(sink.diagnostics(), "§7.4.2.2"));
}

// --- arithmetic and numeric conversion (§7.4.2.3) --------------------------

TEST(DslExpressionTest, NumericConversionFollowsTheSpecifiedRules) {
    // §7.4.2.3.1, in order: float wins; same kind stays; int+uint → int.
    Program program = host_program("    i: int\n    u: uint\n    f: float\n");
    EXPECT_EQ(type_name_of(program, "f + i"), "float");
    EXPECT_EQ(type_name_of(program, "i + f"), "float");
    EXPECT_EQ(type_name_of(program, "u + u"), "uint");
    EXPECT_EQ(type_name_of(program, "i + i"), "int");
    EXPECT_EQ(type_name_of(program, "i + u"), "int");
}

TEST(DslExpressionTest, UnaryMinusOnAUintYieldsAnInt) {
    // §7.4.2.3.1: "the result type of the operator is the type of its operand,
    // or int if the operand is of type uint".
    Program program = host_program("    u: uint\n    f: float\n");
    EXPECT_EQ(type_name_of(program, "- u"), "int");
    EXPECT_EQ(type_name_of(program, "- f"), "float");
}

TEST(DslExpressionTest, PhysicalTypesAreNeverConverted) {
    // §7.4.2.3.1, first rule.
    Program program = host_program("    v: speed\n    d: length\n    f: float\n");
    EXPECT_EQ(type_name_of(program, "v + v"), "::speed");
    EXPECT_EQ(type_name_of(program, "v * f"), "::speed"); // scaling keeps the type
    DiagnosticSink sink;
    EXPECT_EQ(type_of_expression(program, "v + d", sink), kInvalidType);
    EXPECT_TRUE(mentions(sink.diagnostics(), "§7.4.2.3.1"));
    DiagnosticSink mixed;
    EXPECT_EQ(type_of_expression(program, "v + f", mixed), kInvalidType);
    EXPECT_TRUE(mentions(mixed.diagnostics(), "§7.4.2.3.1"));
}

TEST(DslExpressionTest, ArithmeticOverNonNumericIsReported) {
    Program program = host_program("    s: string\n");
    DiagnosticSink sink;
    EXPECT_EQ(type_of_expression(program, "s * s", sink), kInvalidType);
    EXPECT_TRUE(mentions(sink.diagnostics(), "§7.4.2.3"));
}

// --- relational operators (§7.4.2.4) ---------------------------------------

TEST(DslExpressionTest, RelationalOperatorsYieldBooleans) {
    Program program = host_program("    i: int\n    f: float\n    v: speed\n");
    for (const char* expression :
         {"i < f", "i <= f", "i > f", "i >= f", "i == f", "i != f", "v == v", "v < v"}) {
        EXPECT_EQ(type_name_of(program, expression), "bool") << expression;
    }
}

TEST(DslExpressionTest, OrderingOverNonNumericTypesIsReported) {
    // §7.4.2.4.2 gives non-numeric types only ==, != and in.
    Program program = host_program("    a: string\n    b: string\n");
    EXPECT_EQ(type_name_of(program, "a == b"), "bool");
    DiagnosticSink sink;
    EXPECT_EQ(type_of_expression(program, "a < b", sink), kInvalidType);
    EXPECT_TRUE(mentions(sink.diagnostics(), "§7.4.2.4.2"));
}

TEST(DslExpressionTest, EqualityNeedsIdenticalOrRelatedTypes) {
    // §7.4.2.4.2: "identical or one must inherit from the other".
    Program program = host_program("    v: vehicle\n    c: car\n    p: point\n");
    EXPECT_EQ(type_name_of(program, "v == c"), "bool"); // car inherits vehicle
    DiagnosticSink sink;
    EXPECT_EQ(type_of_expression(program, "v == p", sink), kInvalidType);
    EXPECT_TRUE(mentions(sink.diagnostics(), "§7.4.2.4.2"));
}

TEST(DslExpressionTest, ComparingTwoPhysicalTypesIsReported) {
    Program program = host_program("    v: speed\n    d: length\n");
    DiagnosticSink sink;
    EXPECT_EQ(type_of_expression(program, "v < d", sink), kInvalidType);
    EXPECT_TRUE(mentions(sink.diagnostics(), "§7.4.2.4.1"));
}

// --- is() and as() (§7.4.2.5, §7.4.2.6) ------------------------------------

TEST(DslExpressionTest, TypeCheckYieldsABoolean) {
    Program program = host_program("    v: vehicle\n");
    EXPECT_EQ(type_name_of(program, "v.is(car)"), "bool");
}

TEST(DslExpressionTest, TypeConversionYieldsTheTargetType) {
    // §7.4.2.6's downcast to a latent subtype.
    Program program = host_program("    v: vehicle\n");
    EXPECT_EQ(type_name_of(program, "v.as(car)"), "::car");
    EXPECT_EQ(type_name_of(program, "v.as(car).length"), "float");
}

TEST(DslExpressionTest, ConvertingBetweenUnrelatedTypesIsReported) {
    Program program = host_program("    v: vehicle\n");
    DiagnosticSink sink;
    EXPECT_EQ(type_of_expression(program, "v.as(point)", sink), kInvalidType);
    EXPECT_TRUE(mentions(sink.diagnostics(), "§7.4.2.6"));
}

TEST(DslExpressionTest, AnUnknownTargetTypeIsReported) {
    Program program = host_program("    v: vehicle\n");
    DiagnosticSink sink;
    EXPECT_EQ(type_of_expression(program, "v.is(nowhere)", sink), kInvalidType);
    EXPECT_TRUE(mentions(sink.diagnostics(), "unknown type 'nowhere'"));
}

// --- lists (§7.4.2.7) ------------------------------------------------------

TEST(DslExpressionTest, ListOperatorsTakeTheirSpecifiedTypes) {
    Program program = host_program("    l: list of int\n    v: list of vehicle\n");
    EXPECT_EQ(type_name_of(program, "l.size()"), "uint"); // §7.4.2.7.2
    EXPECT_EQ(type_name_of(program, "l[0]"), "int");      // §7.4.2.7.2
    EXPECT_EQ(type_name_of(program, "v.filter(it.is(car))"), "list of ::vehicle");
    EXPECT_EQ(type_name_of(program, "v.first_index(it.is(car))"), "int");
    EXPECT_EQ(type_name_of(program, "v.count(it.is(car))"), "uint");
    EXPECT_EQ(type_name_of(program, "v.has(it.is(car))"), "bool");
}

TEST(DslExpressionTest, ItInsideAMemberEvaluationIsTheListMember) {
    // §7.4.2.7.3: "The variable `it` is defined in the expression scope,
    // referring to the current list member."
    Program program = host_program("    v: list of vehicle\n");
    EXPECT_EQ(type_name_of(program, "v.has(it.length > 4.0)"), "bool");
}

TEST(DslExpressionTest, AListConstructorTakesTheCommonElementType) {
    // §7.4.2.7.4's common-type rules.
    Program program = host_program("    l: list of float\n    v: list of vehicle\n    c: car\n");
    EXPECT_EQ(type_name_of(program, "[1.0, 2.0]"), "list of float");
    EXPECT_EQ(type_name_of(program, "[1, 2.0]"), "list of float"); // int and float → float
    EXPECT_EQ(type_name_of(program, "[c, c]"), "list of ::car");
}

TEST(DslExpressionTest, AListWithNoCommonTypeIsReported) {
    Program program = host_program("    p: point\n    v: vehicle\n");
    DiagnosticSink sink;
    EXPECT_EQ(type_of_expression(program, "[p, v]", sink), kInvalidType);
    EXPECT_TRUE(mentions(sink.diagnostics(), "§7.4.2.7.4"));
}

TEST(DslExpressionTest, IndexingSomethingThatIsNotAListIsReported) {
    Program program = host_program("    n: int\n");
    DiagnosticSink sink;
    EXPECT_EQ(type_of_expression(program, "n[0]", sink), kInvalidType);
    EXPECT_TRUE(mentions(sink.diagnostics(), "§7.4.2.7.2"));
}

// --- ranges (§7.4.2.8) -----------------------------------------------------

TEST(DslExpressionTest, RangeConstructorsTakeBothFormsAndACommonBase) {
    Program program = host_program("    r: range of float\n    s: range of speed\n");
    EXPECT_EQ(type_name_of(program, "[1.5...3.5]"), "range of float");
    EXPECT_EQ(type_name_of(program, "range(1.5, 3.5)"), "range of float");
    // §7.4.2.8.1: one float and one int converts to float.
    EXPECT_EQ(type_name_of(program, "[10...20.5]"), "range of float");
}

TEST(DslExpressionTest, RangeBoundOperatorsYieldTheBaseType) {
    // §7.4.2.8.4.
    Program program = host_program("    r: range of float\n");
    EXPECT_EQ(type_name_of(program, "r.lower()"), "float");
    EXPECT_EQ(type_name_of(program, "r.upper()"), "float");
}

TEST(DslExpressionTest, ARangeOverTwoDifferentPhysicalTypesIsReported) {
    Program program = host_program("    v: speed\n    d: length\n");
    DiagnosticSink sink;
    EXPECT_EQ(type_of_expression(program, "[1.0kph...2.0m]", sink), kInvalidType);
    EXPECT_TRUE(mentions(sink.diagnostics(), "§7.4.2.8.1"));
}

TEST(DslExpressionTest, MembershipAppliesToListsAndRanges) {
    Program program = host_program("    n: float\n    l: list of float\n    r: range of float\n");
    EXPECT_EQ(type_name_of(program, "n in l"), "bool");
    EXPECT_EQ(type_name_of(program, "n in r"), "bool");
    DiagnosticSink sink;
    EXPECT_EQ(type_of_expression(program, "n in n", sink), kInvalidType);
    EXPECT_TRUE(mentions(sink.diagnostics(), "§7.4.2.4.1"));
}

// --- ternary (§7.4.2.9) ----------------------------------------------------

TEST(DslExpressionTest, TheTernaryNeedsABoolAndACommonArmType) {
    Program program = host_program("    b: bool\n    i: int\n    f: float\n    p: point\n");
    EXPECT_EQ(type_name_of(program, "b ? i : f"), "float");
    DiagnosticSink condition;
    EXPECT_EQ(type_of_expression(program, "i ? i : i", condition), kInvalidType);
    EXPECT_TRUE(mentions(condition.diagnostics(), "bool condition"));
    DiagnosticSink arms;
    EXPECT_EQ(type_of_expression(program, "b ? i : p", arms), kInvalidType);
    EXPECT_TRUE(mentions(arms.diagnostics(), "no common type"));
}

// --- methods (§7.4.2.1) ----------------------------------------------------

TEST(DslExpressionTest, AMethodBodySeesItsParameters) {
    Program program = host_program("    def scaled(by: float) -> float is expression by * 2.0\n");
    EXPECT_NE(program.find("::host"), nullptr);
}

TEST(DslExpressionTest, AMethodBodyMustReturnItsDeclaredType) {
    const std::vector<scena::Diagnostic> errors =
        host_diagnostics("    def f() -> int is expression \"text\"\n");
    EXPECT_TRUE(mentions(errors, "this method returns"));
}

TEST(DslExpressionTest, CallingAMissingMethodIsReported) {
    Program program = host_program("    v: vehicle\n");
    DiagnosticSink sink;
    EXPECT_EQ(type_of_expression(program, "v.nowhere()", sink), kInvalidType);
    EXPECT_TRUE(mentions(sink.diagnostics(), "has no method 'nowhere'"));
}

TEST(DslExpressionTest, UsingAMethodAsAFieldIsReported) {
    Program program = host_program("    v: vehicle\n    def m() -> int is undefined\n");
    DiagnosticSink sink;
    EXPECT_EQ(type_of_expression(program, "m", sink), kInvalidType);
    EXPECT_TRUE(mentions(sink.diagnostics(), "unknown name"));
}

// --- field defaults --------------------------------------------------------

TEST(DslExpressionTest, ANonLiteralDefaultIsCheckedAgainstTheField) {
    const std::vector<scena::Diagnostic> errors =
        host_diagnostics("    a: int\n    b: string = a\n");
    EXPECT_TRUE(mentions(errors, "§7.4.2.6"));
}

TEST(DslExpressionTest, ImplicitConversionsAreAcceptedInADefault) {
    // §7.4.2.6: int or uint to float, and a numeric to a range of it
    // (§7.4.2.8.2).
    Program program = host_program("    i: int\n"
                                   "    f: float = i\n"
                                   "    r: range of float = i\n"
                                   "    l: list of float = [1, 2]\n");
    EXPECT_NE(program.find("::host"), nullptr);
}

TEST(DslExpressionTest, AnUpcastIsAcceptedInADefault) {
    Program program = host_program("    c: car\n    v: vehicle = c\n");
    EXPECT_NE(program.find("::host"), nullptr);
}

// --- constant evaluation ---------------------------------------------------

TEST(DslExpressionTest, ArithmeticFoldsInAConstantContext) {
    Program program = host_program("    a: int\n");
    Value value;
    ASSERT_TRUE(fold(program, "1 + 2 * 3", value));
    EXPECT_EQ(value.as_double(), 7.0);
    ASSERT_TRUE(fold(program, "10.0 / 4.0", value));
    EXPECT_DOUBLE_EQ(value.number, 2.5);
}

TEST(DslExpressionTest, IntegerArithmeticStaysIntegral) {
    // A constraint over ints is decided in integer arithmetic, not in a double
    // that has already rounded.
    Program program = host_program("    a: int\n");
    Value value;
    ASSERT_TRUE(fold(program, "-7 % -4", value));
    EXPECT_EQ(value.kind, Value::Kind::Int);
    EXPECT_EQ(value.integer, -3);
}

TEST(DslExpressionTest, PhysicalLiteralsFoldToTheirBaseUnit) {
    // §7.3.4: two units of one type compare directly once both are in the base
    // unit.
    Program program = host_program("    a: int\n");
    Value in_kph;
    Value in_mps;
    ASSERT_TRUE(fold(program, "36.0kph", in_kph));
    ASSERT_TRUE(fold(program, "10.0mps", in_mps));
    EXPECT_NEAR(in_kph.number, in_mps.number, 1e-9);
    Value comparison;
    ASSERT_TRUE(fold(program, "36.0kph > 5.0mps", comparison));
    EXPECT_TRUE(comparison.boolean);
}

TEST(DslExpressionTest, LogicalOperatorsShortCircuit) {
    // §7.4.2.2: sub-expressions that cannot affect the result are not
    // evaluated — here the right-hand side would fail to fold.
    Program program = host_program("    a: int\n");
    Value value;
    ASSERT_TRUE(fold(program, "true ? 1 : (1 / 0)", value));
    EXPECT_EQ(value.as_double(), 1.0);
}

TEST(DslExpressionTest, DivisionByZeroDoesNotFold) {
    Program program = host_program("    a: int\n");
    Value value;
    EXPECT_FALSE(fold(program, "1 / 0", value));
    EXPECT_FALSE(fold(program, "1.0 % 0.0", value));
}

TEST(DslExpressionTest, ListsFlattenWhenConstructed) {
    // §7.4.2.7.4: "If any of the expressions returns a list, the members of
    // that list are added to the result."
    Program program = host_program("    a: int\n");
    Value value;
    ASSERT_TRUE(fold(program, "[1, [2, 3], 4]", value));
    EXPECT_EQ(value.kind, Value::Kind::List);
    EXPECT_EQ(value.items.size(), 4U);
}

TEST(DslExpressionTest, RangeBoundsAreOrderedByValue) {
    // §7.4.2.8.1: "The expression with the lower numeric value will provide the
    // lower bound" — either order may be written.
    Program program = host_program("    a: int\n");
    Value forward;
    Value backward;
    ASSERT_TRUE(fold(program, "[1.0...5.0]", forward));
    ASSERT_TRUE(fold(program, "[5.0...1.0]", backward));
    ASSERT_EQ(forward.items.size(), 2U);
    EXPECT_EQ(forward.items[0].as_double(), 1.0);
    EXPECT_EQ(backward.items[0].as_double(), 1.0);
    EXPECT_EQ(backward.items[1].as_double(), 5.0);
}

TEST(DslExpressionTest, MembershipFoldsOverListsAndRanges) {
    Program program = host_program("    a: int\n");
    Value value;
    ASSERT_TRUE(fold(program, "3.0 in [1.0...5.0]", value));
    EXPECT_TRUE(value.boolean);
    ASSERT_TRUE(fold(program, "7.0 in [1.0...5.0]", value));
    EXPECT_FALSE(value.boolean);
    ASSERT_TRUE(fold(program, "2 in [1, 2, 3]", value));
    EXPECT_TRUE(value.boolean);
}

TEST(DslExpressionTest, EnumMembersFoldToTheirValue) {
    Program program = host_program("    a: int\n");
    Value value;
    ASSERT_TRUE(fold(program, "side!right", value));
    EXPECT_EQ(value.kind, Value::Kind::Enum);
    EXPECT_EQ(value.enum_value, 1U);
    ASSERT_TRUE(fold(program, "side!left == side!right", value));
    EXPECT_FALSE(value.boolean);
}

TEST(DslExpressionTest, AFieldReadIsNotConstant) {
    // This is what tells the constraint checker that a keep would need search.
    Program program = host_program("    a: int\n");
    const std::string source = "struct probe:\n    f: int = other + 1\n";
    DiagnosticSink sink;
    File file;
    ASSERT_EQ(scena::dsl::parse_source(source, file, sink), Status::Ok);
    ExpressionContext context;
    Value value;
    EXPECT_FALSE(scena::dsl::evaluate_constant(
        program, *file.declarations.front().structured.members.front().field.default_value, context,
        value));
}

TEST(DslExpressionTest, TypingIsDeterministicAcrossRepeats) {
    Program program = host_program("    i: int\n    f: float\n    v: speed\n");
    for (int repeat = 0; repeat < 3; ++repeat) {
        EXPECT_EQ(type_name_of(program, "i + f * 2"), "float");
        EXPECT_EQ(type_name_of(program, "v == v"), "bool");
    }
}

TEST(DslExpressionTest, OneMistakeProducesOneMessage) {
    // A subexpression that failed to type must not make its parent complain
    // again about the type it never got.
    Program program = host_program("    i: int\n");
    DiagnosticSink sink;
    EXPECT_EQ(type_of_expression(program, "nowhere + nothing * 2", sink), kInvalidType);
    EXPECT_TRUE(has_error(sink.diagnostics()));
    EXPECT_EQ(sink.diagnostics().size(), 2U); // one per unknown name, and no more
}

} // namespace
