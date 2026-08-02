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
// DSL parser and AST (p7-s2, #40): the §7.2.2 grammar — top-level structure,
// every type declaration, structured-type members, behavior specification and
// the expression precedence ladder — plus the error recovery the pillar's exit
// criteria ask for.
//
// Every source fragment here is written from the grammar in the specification
// text; none is taken from another implementation's corpus (ADR-0002).

#include <string>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

#include "scena/diagnostic.h"
#include "scena/dsl/ast.h"
#include "scena/dsl/parser.h"
#include "scena/status.h"

namespace {

using scena::DiagnosticSink;
using scena::Severity;
using scena::Status;
using scena::dsl::Declaration;
using scena::dsl::DoMemberKind;
using scena::dsl::EventConditionKind;
using scena::dsl::ExprKind;
using scena::dsl::File;
using scena::dsl::LiteralType;
using scena::dsl::Member;
using scena::dsl::StructuredKind;

File parse_ok(std::string_view source) {
    DiagnosticSink sink;
    File file;
    const Status status = scena::dsl::parse_source(source, file, sink);
    for (const scena::Diagnostic& diagnostic : sink.diagnostics()) {
        EXPECT_NE(diagnostic.severity, Severity::Error) << diagnostic.message;
    }
    EXPECT_EQ(status, Status::Ok);
    return file;
}

std::vector<scena::Diagnostic> parse_errors(std::string_view source) {
    DiagnosticSink sink;
    File file;
    EXPECT_EQ(scena::dsl::parse_source(source, file, sink), Status::ValidationError);
    return sink.diagnostics();
}

/// The single structured declaration of a one-declaration file.
const scena::dsl::StructuredDecl& only_structured(const File& file) {
    EXPECT_EQ(file.declarations.size(), 1U);
    EXPECT_EQ(file.declarations.front().kind, Declaration::Kind::Structured);
    return file.declarations.front().structured;
}

/// Parses `expr` as the default value of a parameter and returns it.
scena::dsl::ExprPtr parse_expr(std::string_view expr) {
    const std::string source = "struct s:\n    f: int = " + std::string(expr) + "\n";
    const File file = parse_ok(source);
    const scena::dsl::StructuredDecl& decl = only_structured(file);
    EXPECT_EQ(decl.members.size(), 1U);
    EXPECT_EQ(decl.members.front().kind, Member::Kind::Field);
    return decl.members.front().field.default_value;
}

// --- top-level structure (§7.2.2.1) ----------------------------------------

TEST(DslParserTest, ImportsTakeBothReferenceForms) {
    const File file = parse_ok("import osc.standard.all\nimport \"local/types.osc\"\n");
    ASSERT_EQ(file.declarations.size(), 2U);
    EXPECT_EQ(file.declarations[0].kind, Declaration::Kind::Import);
    EXPECT_EQ(file.declarations[0].import.reference, "osc.standard.all");
    EXPECT_FALSE(file.declarations[0].import.is_path);
    EXPECT_EQ(file.declarations[1].import.reference, "local/types.osc");
    EXPECT_TRUE(file.declarations[1].import.is_path);
}

TEST(DslParserTest, AnImportAfterAMainStatementIsReported) {
    // §7.2.2.1.1: "Prelude statements must occur before all other statements".
    const std::vector<scena::Diagnostic> errors = parse_errors("struct s\nimport osc.standard\n");
    ASSERT_FALSE(errors.empty());
    EXPECT_NE(errors.front().message.find("before every other statement"), std::string::npos);
    EXPECT_NE(errors.front().message.find("7.2.2.1.1"), std::string::npos);
}

TEST(DslParserTest, NamespaceStatementsCarryTheirUseList) {
    const File file = parse_ok("namespace dut use osc, other\n");
    ASSERT_EQ(file.declarations.size(), 1U);
    EXPECT_EQ(file.declarations[0].kind, Declaration::Kind::Namespace);
    EXPECT_EQ(file.declarations[0].name_space.name, "dut");
    EXPECT_EQ(file.declarations[0].name_space.uses.size(), 2U);
    // 'null' names the global namespace (§7.2.2.1.2).
    const File global = parse_ok("namespace null\n");
    EXPECT_EQ(global.declarations[0].name_space.name, "null");
}

TEST(DslParserTest, ExportsIncludeTheWildcardForms) {
    const File file = parse_ok("export a, b, ns::*, ::*\n");
    ASSERT_EQ(file.declarations.size(), 1U);
    const std::vector<std::string>& names = file.declarations[0].export_decl.names;
    ASSERT_EQ(names.size(), 4U);
    EXPECT_EQ(names[0], "a");
    EXPECT_EQ(names[2], "ns::*");
    EXPECT_EQ(names[3], "::*");
}

// --- type declarations (§7.2.2.2) ------------------------------------------

TEST(DslParserTest, APhysicalTypeCarriesItsSiExponents) {
    const File file = parse_ok("type length is SI(m: 1)\ntype speed is SI(m: 1, s: -1)\n");
    ASSERT_EQ(file.declarations.size(), 2U);
    EXPECT_EQ(file.declarations[0].kind, Declaration::Kind::PhysicalType);
    EXPECT_EQ(file.declarations[0].physical_type.name, "length");
    ASSERT_EQ(file.declarations[0].physical_type.exponents.size(), 1U);
    EXPECT_EQ(file.declarations[0].physical_type.exponents[0].unit, "m");
    EXPECT_EQ(file.declarations[0].physical_type.exponents[0].exponent, 1);
    const auto& speed = file.declarations[1].physical_type.exponents;
    ASSERT_EQ(speed.size(), 2U);
    EXPECT_EQ(speed[1].unit, "s");
    EXPECT_EQ(speed[1].exponent, -1);
}

TEST(DslParserTest, AUnitCarriesItsFactorAndOffset) {
    const File file =
        parse_ok("unit kph of speed is SI(m: 1, s: -1, factor: 0.277778)\n"
                 "unit celsius of temperature is SI(K: 1, factor: 1, offset: 273.15)\n");
    ASSERT_EQ(file.declarations.size(), 2U);
    EXPECT_EQ(file.declarations[0].unit.name, "kph");
    EXPECT_EQ(file.declarations[0].unit.physical_type, "speed");
    ASSERT_TRUE(file.declarations[0].unit.factor.has_value());
    EXPECT_DOUBLE_EQ(*file.declarations[0].unit.factor, 0.277778);
    EXPECT_FALSE(file.declarations[0].unit.offset.has_value());
    ASSERT_TRUE(file.declarations[1].unit.offset.has_value());
    EXPECT_DOUBLE_EQ(*file.declarations[1].unit.offset, 273.15);
}

TEST(DslParserTest, APhysicalTypesBaseUnitTakesNoFactor) {
    // base-unit-specifier is an SI-base-unit-specifier, which has no factor or
    // offset (§7.2.2.2.1) — only a unit declaration does.
    const std::vector<scena::Diagnostic> errors =
        parse_errors("type speed is SI(m: 1, s: -1, factor: 2)\n");
    ASSERT_FALSE(errors.empty());
    EXPECT_NE(errors.front().message.find("no factor or offset"), std::string::npos);
}

TEST(DslParserTest, EnumsDeclareMembersWithOptionalValues) {
    const File file = parse_ok("enum side: [left, right = 5, middle]\n");
    ASSERT_EQ(file.declarations.size(), 1U);
    const auto& members = file.declarations[0].enumeration.members;
    ASSERT_EQ(members.size(), 3U);
    EXPECT_EQ(members[0].name, "left");
    EXPECT_FALSE(members[0].value.has_value());
    ASSERT_TRUE(members[1].value.has_value());
    EXPECT_EQ(*members[1].value, 5);
    EXPECT_FALSE(file.declarations[0].enumeration.is_extension);
}

TEST(DslParserTest, AnEnumExtensionIsMarkedAsOne) {
    // §7.2.2.2.6: enum-type-extension ::= 'extend' enum-name ':' '[' .. ']'
    const File file = parse_ok("extend side: [up, down]\n");
    ASSERT_EQ(file.declarations.size(), 1U);
    EXPECT_EQ(file.declarations[0].kind, Declaration::Kind::Enum);
    EXPECT_TRUE(file.declarations[0].enumeration.is_extension);
    EXPECT_EQ(file.declarations[0].enumeration.name, "side");
    EXPECT_EQ(file.declarations[0].enumeration.members.size(), 2U);
}

TEST(DslParserTest, EveryStructuredTypeKeywordIsRecognized) {
    struct Case {
        std::string_view source;
        StructuredKind kind;
    };
    for (const Case& test_case :
         {Case{"struct s\n", StructuredKind::Struct}, Case{"actor a\n", StructuredKind::Actor},
          Case{"scenario sc\n", StructuredKind::Scenario},
          Case{"action ac\n", StructuredKind::Action},
          Case{"modifier m\n", StructuredKind::Modifier}}) {
        const File file = parse_ok(test_case.source);
        EXPECT_EQ(only_structured(file).kind, test_case.kind) << test_case.source;
    }
    // §7.2.2.2.6's structured-type-extension.
    const File extension = parse_ok("extend s:\n    f: int\n");
    EXPECT_EQ(only_structured(extension).kind, StructuredKind::Extension);
}

TEST(DslParserTest, AnEmptyStructuredDeclarationIsLegal) {
    // The body is `(':' NEWLINE INDENT members DEDENT) | NEWLINE` — the second
    // alternative is a declaration with nothing in it (§7.2.2.2.4).
    const File file = parse_ok("struct empty\n");
    EXPECT_TRUE(only_structured(file).members.empty());
}

TEST(DslParserTest, InheritanceCarriesItsOptionalGuard) {
    const File plain = parse_ok("actor car inherits vehicle\n");
    EXPECT_EQ(only_structured(plain).base, "vehicle");
    EXPECT_TRUE(only_structured(plain).constraint_field.empty());

    // Conditional inheritance: inherits X(field == value) (§7.3.8).
    const File guarded = parse_ok("actor ev inherits car(propulsion == propulsion!electric)\n");
    const auto& decl = only_structured(guarded);
    EXPECT_EQ(decl.base, "car");
    EXPECT_EQ(decl.constraint_field, "propulsion");
    ASSERT_NE(decl.constraint_value, nullptr);
    EXPECT_EQ(decl.constraint_value->kind, ExprKind::EnumValue);
    EXPECT_EQ(decl.constraint_value->type_name, "propulsion");
    EXPECT_EQ(decl.constraint_value->text, "electric");
}

TEST(DslParserTest, AQualifiedBehaviorNameKeepsItsActorPrefix) {
    // qualified-behavior-name ::= [actor-name '.'] behavior-name (§7.2.2.2.4).
    const File file = parse_ok("scenario dut.cut_in\n");
    EXPECT_EQ(only_structured(file).name, "dut.cut_in");
}

TEST(DslParserTest, AModifierMayNameWhatItModifies) {
    const File file = parse_ok("modifier speed_profile of vehicle.drive\n");
    EXPECT_EQ(only_structured(file).name, "speed_profile");
    EXPECT_EQ(only_structured(file).modifies, "vehicle.drive");
}

// --- structured-type members (§7.2.2.4) ------------------------------------

TEST(DslParserTest, FieldsCoverParametersVariablesAndMultipleNames) {
    const File file = parse_ok("struct s:\n"
                               "    a: int\n"
                               "    b, c: float = 1.5\n"
                               "    var v: speed\n"
                               "    l: list of int\n"
                               "    r: range of float\n");
    const auto& members = only_structured(file).members;
    ASSERT_EQ(members.size(), 5U);
    EXPECT_EQ(members[0].field.names, (std::vector<std::string>{"a"}));
    EXPECT_EQ(members[0].field.type.name, "int");
    EXPECT_FALSE(members[0].field.is_variable);
    // One declaration may name several fields (§7.2.2.4.2).
    EXPECT_EQ(members[1].field.names, (std::vector<std::string>{"b", "c"}));
    ASSERT_NE(members[1].field.default_value, nullptr);
    EXPECT_DOUBLE_EQ(members[1].field.default_value->number, 1.5);
    EXPECT_TRUE(members[2].field.is_variable);
    EXPECT_TRUE(members[3].field.type.is_list);
    EXPECT_TRUE(members[4].field.type.is_range);
}

TEST(DslParserTest, AParameterMayCarryAWithBlockOfConstraints) {
    const File file = parse_ok("struct s:\n"
                               "    speed: float with:\n"
                               "        keep(it > 0.0)\n"
                               "        keep(hard it < 100.0)\n");
    const auto& members = only_structured(file).members;
    ASSERT_EQ(members.size(), 1U);
    ASSERT_EQ(members[0].field.constraints.size(), 2U);
    EXPECT_TRUE(members[0].field.constraints[0].qualifier.empty());
    EXPECT_EQ(members[0].field.constraints[1].qualifier, "hard");
}

TEST(DslParserTest, ASampledVariableKeepsItsEventSpecification) {
    // sample-expression ::= 'sample' '(' expr ',' event-spec [',' default] ')'
    const File file = parse_ok("struct s:\n    var v: float = sample(x.speed, @tick, 0.0)\n");
    const auto& field = only_structured(file).members.front().field;
    EXPECT_TRUE(field.is_sampled);
    ASSERT_TRUE(field.sample_event.has_value());
    EXPECT_EQ(field.sample_event->kind, EventConditionKind::Reference);
    EXPECT_EQ(field.sample_event->event_path, "tick");
}

TEST(DslParserTest, ConstraintsComeInKeepAndRemoveDefaultForms) {
    const File file = parse_ok("struct s:\n"
                               "    keep(default a == 1)\n"
                               "    remove_default(b)\n");
    const auto& members = only_structured(file).members;
    ASSERT_EQ(members.size(), 2U);
    EXPECT_EQ(members[0].kind, Member::Kind::Constraint);
    EXPECT_EQ(members[0].constraint.qualifier, "default");
    EXPECT_FALSE(members[0].constraint.is_remove_default);
    EXPECT_TRUE(members[1].constraint.is_remove_default);
}

TEST(DslParserTest, EventDeclarationsTakeParametersAndAFormula) {
    const File file = parse_ok("struct s:\n"
                               "    event bare\n"
                               "    event with_args(a: int, b: float = 1.0)\n"
                               "    event derived is @other.tick as t if t > 0\n");
    const auto& members = only_structured(file).members;
    ASSERT_EQ(members.size(), 3U);
    EXPECT_EQ(members[0].kind, Member::Kind::Event);
    EXPECT_EQ(members[0].event.name, "bare");
    EXPECT_TRUE(members[0].event.parameters.empty());
    EXPECT_EQ(members[1].event.parameters.size(), 2U);
    ASSERT_TRUE(members[2].event.spec.has_value());
    EXPECT_EQ(members[2].event.spec->kind, EventConditionKind::Reference);
    EXPECT_EQ(members[2].event.spec->event_path, "other.tick");
    EXPECT_EQ(members[2].event.spec->binding, "t");
    EXPECT_NE(members[2].event.spec->expression, nullptr);
}

TEST(DslParserTest, EveryEventConditionFormIsRecognized) {
    const File file = parse_ok("struct s:\n"
                               "    event a is rise(x > 1)\n"
                               "    event b is fall(x > 1)\n"
                               "    event c is elapsed(2s)\n"
                               "    event d is every(1s, offset: 0.5s)\n"
                               "    event e is x > 1\n");
    const auto& members = only_structured(file).members;
    ASSERT_EQ(members.size(), 5U);
    EXPECT_EQ(members[0].event.spec->kind, EventConditionKind::Rise);
    EXPECT_EQ(members[1].event.spec->kind, EventConditionKind::Fall);
    EXPECT_EQ(members[2].event.spec->kind, EventConditionKind::Elapsed);
    EXPECT_EQ(members[3].event.spec->kind, EventConditionKind::Every);
    EXPECT_NE(members[3].event.spec->offset, nullptr);
    EXPECT_EQ(members[4].event.spec->kind, EventConditionKind::Expression);
}

TEST(DslParserTest, MethodsCoverAllThreeImplementationForms) {
    const File file = parse_ok("struct s:\n"
                               "    def a(x: int) -> int is expression x + 1\n"
                               "    def b() is undefined\n"
                               "    def c(x: float) is external lib.fn(x)\n"
                               "    def d() is only expression 1\n");
    const auto& members = only_structured(file).members;
    ASSERT_EQ(members.size(), 4U);
    EXPECT_EQ(members[0].kind, Member::Kind::Method);
    EXPECT_EQ(members[0].method.implementation, "expression");
    ASSERT_TRUE(members[0].method.return_type.has_value());
    EXPECT_EQ(members[0].method.return_type->name, "int");
    EXPECT_EQ(members[1].method.implementation, "undefined");
    EXPECT_EQ(members[2].method.implementation, "external");
    EXPECT_EQ(members[2].method.external_name, "lib.fn");
    EXPECT_EQ(members[3].method.qualifier, "only");
}

TEST(DslParserTest, CoverAndRecordAreBothCoverageDeclarations) {
    const File file = parse_ok("struct s:\n    cover(x, name: \"n\")\n    record(y)\n");
    const auto& members = only_structured(file).members;
    ASSERT_EQ(members.size(), 2U);
    EXPECT_EQ(members[0].kind, Member::Kind::Coverage);
    EXPECT_FALSE(members[0].coverage.is_record);
    EXPECT_EQ(members[0].coverage.arguments.size(), 2U);
    EXPECT_EQ(members[0].coverage.arguments[1].name, "name");
    EXPECT_TRUE(members[1].coverage.is_record);
}

TEST(DslParserTest, AModifierApplicationIsToldApartFromAFieldDeclaration) {
    // Both start with an identifier; ':' makes it a field, '(' a modifier
    // application (§7.2.2.4.2 vs §7.2.2.4.6).
    const File file = parse_ok("scenario s:\n"
                               "    f: int\n"
                               "    speed(30kph)\n"
                               "    ego.position(10m)\n");
    const auto& members = only_structured(file).members;
    ASSERT_EQ(members.size(), 3U);
    EXPECT_EQ(members[0].kind, Member::Kind::Field);
    EXPECT_EQ(members[1].kind, Member::Kind::ModifierApplication);
    EXPECT_EQ(members[1].modifier.name, "speed");
    EXPECT_EQ(members[1].modifier.arguments.size(), 1U);
    EXPECT_EQ(members[2].kind, Member::Kind::ModifierApplication);
    EXPECT_EQ(members[2].modifier.name, "position");
    ASSERT_NE(members[2].modifier.actor, nullptr);
    EXPECT_EQ(members[2].modifier.actor->text, "ego");
}

// --- behavior specification (§7.2.2.4.7) -----------------------------------

TEST(DslParserTest, EveryCompositionOperatorIsRecognized) {
    const File file = parse_ok("scenario s:\n"
                               "    do serial:\n"
                               "        parallel:\n"
                               "            a()\n"
                               "        one_of:\n"
                               "            b()\n"
                               "            c()\n");
    const auto& members = only_structured(file).members;
    ASSERT_EQ(members.size(), 1U);
    ASSERT_EQ(members[0].kind, Member::Kind::Behavior);
    const auto& root = members[0].behavior;
    ASSERT_NE(root, nullptr);
    EXPECT_EQ(root->kind, DoMemberKind::Composition);
    EXPECT_EQ(root->composition, scena::dsl::CompositionOperator::Serial);
    ASSERT_EQ(root->members.size(), 2U);
    EXPECT_EQ(root->members[0]->composition, scena::dsl::CompositionOperator::Parallel);
    EXPECT_EQ(root->members[1]->composition, scena::dsl::CompositionOperator::OneOf);
    EXPECT_EQ(root->members[1]->members.size(), 2U);
}

TEST(DslParserTest, ACompositionTakesArgumentsAndALabel) {
    const File file = parse_ok("scenario s:\n"
                               "    do phase1: parallel(duration: 5s):\n"
                               "        a()\n");
    const auto& root = only_structured(file).members.front().behavior;
    EXPECT_EQ(root->label, "phase1");
    ASSERT_EQ(root->composition_arguments.size(), 1U);
    EXPECT_EQ(root->composition_arguments[0].name, "duration");
    EXPECT_EQ(root->composition_arguments[0].value->kind, ExprKind::PhysicalLiteral);
    EXPECT_EQ(root->composition_arguments[0].value->text, "s");
}

TEST(DslParserTest, BehaviorInvocationsCarryTheirActorAndWithBlock) {
    const File file = parse_ok("scenario s:\n"
                               "    do serial:\n"
                               "        ego.drive(fast: true) with:\n"
                               "            speed(30kph)\n"
                               "            keep(it > 0)\n"
                               "            until @done\n"
                               "        plain()\n");
    const auto& root = only_structured(file).members.front().behavior;
    ASSERT_EQ(root->members.size(), 2U);
    const auto& drive = root->members[0];
    EXPECT_EQ(drive->kind, DoMemberKind::Invocation);
    EXPECT_EQ(drive->name, "drive");
    ASSERT_NE(drive->actor, nullptr);
    EXPECT_EQ(drive->actor->text, "ego");
    EXPECT_EQ(drive->arguments.size(), 1U);
    EXPECT_EQ(drive->arguments[0].name, "fast");
    EXPECT_EQ(drive->with.modifiers.size(), 1U);
    EXPECT_EQ(drive->with.modifiers[0].name, "speed");
    EXPECT_EQ(drive->with.constraints.size(), 1U);
    ASSERT_EQ(drive->with.until.size(), 1U);
    EXPECT_EQ(drive->with.until[0].event_path, "done");
    // An invocation with no actor and no with-block still parses.
    EXPECT_EQ(root->members[1]->name, "plain");
    EXPECT_EQ(root->members[1]->actor, nullptr);
}

TEST(DslParserTest, WaitEmitAndCallAreDoMembers) {
    const File file = parse_ok("scenario s:\n"
                               "    do serial:\n"
                               "        wait @start\n"
                               "        emit done(value: 1)\n"
                               "        call obj.method(2)\n");
    const auto& root = only_structured(file).members.front().behavior;
    ASSERT_EQ(root->members.size(), 3U);
    EXPECT_EQ(root->members[0]->kind, DoMemberKind::Wait);
    EXPECT_EQ(root->members[0]->event.event_path, "start");
    EXPECT_EQ(root->members[1]->kind, DoMemberKind::Emit);
    EXPECT_EQ(root->members[1]->emit_name, "done");
    EXPECT_EQ(root->members[1]->arguments.size(), 1U);
    EXPECT_EQ(root->members[2]->kind, DoMemberKind::Call);
}

TEST(DslParserTest, AnOnDirectiveHoldsCallAndEmitMembers) {
    const File file = parse_ok("scenario s:\n"
                               "    on @collision:\n"
                               "        emit failed\n");
    const auto& members = only_structured(file).members;
    ASSERT_EQ(members.size(), 1U);
    EXPECT_EQ(members[0].kind, Member::Kind::On);
    EXPECT_EQ(members[0].on.event.event_path, "collision");
    ASSERT_EQ(members[0].on.members.size(), 1U);
    EXPECT_EQ(members[0].on.members[0]->kind, DoMemberKind::Emit);
}

TEST(DslParserTest, AnOnDirectiveRejectsOtherMembers) {
    // §7.2.2.4.7: on-member ::= call-directive | emit-directive.
    const std::vector<scena::Diagnostic> errors =
        parse_errors("scenario s:\n    on @e:\n        wait @other\n");
    ASSERT_FALSE(errors.empty());
    EXPECT_NE(errors.front().message.find("only 'call' and 'emit'"), std::string::npos);
}

// --- expressions (§7.2.2.6) ------------------------------------------------

TEST(DslParserTest, LiteralsKeepTheirTypeAndValue) {
    EXPECT_EQ(parse_expr("42")->literal_type, LiteralType::UnsignedInteger);
    EXPECT_EQ(parse_expr("42")->unsigned_value, 42U);
    EXPECT_EQ(parse_expr("-7")->literal_type, LiteralType::Integer);
    EXPECT_EQ(parse_expr("-7")->signed_value, -7);
    EXPECT_EQ(parse_expr("1.5")->literal_type, LiteralType::Float);
    EXPECT_EQ(parse_expr("true")->literal_type, LiteralType::Bool);
    EXPECT_TRUE(parse_expr("true")->boolean);
    EXPECT_EQ(parse_expr("\"s\"")->literal_type, LiteralType::String);
    EXPECT_EQ(parse_expr("\"s\"")->text, "s");
    const auto physical = parse_expr("30kph");
    EXPECT_EQ(physical->kind, ExprKind::PhysicalLiteral);
    EXPECT_EQ(physical->text, "kph");
    EXPECT_DOUBLE_EQ(physical->number, 30.0);
}

TEST(DslParserTest, ArithmeticBindsTighterThanComparison) {
    // sum sits below relation in the ladder (§7.2.2.6.3, §7.2.2.6.4), so
    // `1 + 2 < 4` parses as `(1 + 2) < 4`.
    const auto expr = parse_expr("1 + 2 < 4");
    ASSERT_EQ(expr->kind, ExprKind::Binary);
    EXPECT_EQ(expr->text, "<");
    EXPECT_EQ(expr->operands[0]->text, "+");
}

TEST(DslParserTest, MultiplicationBindsTighterThanAddition) {
    const auto expr = parse_expr("1 + 2 * 3");
    ASSERT_EQ(expr->text, "+");
    EXPECT_EQ(expr->operands[1]->text, "*");
    // ... and parentheses override it (§7.2.2.6.6).
    const auto parenthesised = parse_expr("(1 + 2) * 3");
    EXPECT_EQ(parenthesised->text, "*");
    EXPECT_EQ(parenthesised->operands[0]->text, "+");
}

TEST(DslParserTest, TheLogicalLadderNestsInSpecOrder) {
    // implication > disjunction > conjunction > inversion (§7.2.2.6.2).
    const auto expr = parse_expr("a or b and not c => d");
    ASSERT_EQ(expr->text, "=>");
    EXPECT_EQ(expr->operands[0]->text, "or");
    EXPECT_EQ(expr->operands[0]->operands[1]->text, "and");
    EXPECT_EQ(expr->operands[0]->operands[1]->operands[1]->text, "not");
}

TEST(DslParserTest, BinaryOperatorsAreLeftAssociative) {
    const auto expr = parse_expr("1 - 2 - 3");
    ASSERT_EQ(expr->text, "-");
    // (1 - 2) - 3, not 1 - (2 - 3).
    EXPECT_EQ(expr->operands[0]->text, "-");
}

TEST(DslParserTest, ANegativeLiteralAfterAnOperandIsSubtraction) {
    // The lexer produces a single Integer token for `-2` (§7.2.1.5.2 puts the
    // sign inside int-literal), so the parser re-splits it where a value has
    // already been seen. Otherwise `a -2` would be two expressions.
    const auto expr = parse_expr("a -2");
    ASSERT_EQ(expr->kind, ExprKind::Binary);
    EXPECT_EQ(expr->text, "-");
    EXPECT_EQ(expr->operands[0]->kind, ExprKind::Name);
    EXPECT_EQ(expr->operands[1]->signed_value, 2);
}

TEST(DslParserTest, TheTernaryOperatorNestsToTheRight) {
    const auto expr = parse_expr("a ? b : c ? d : e");
    ASSERT_EQ(expr->kind, ExprKind::Ternary);
    ASSERT_EQ(expr->operands.size(), 3U);
    EXPECT_EQ(expr->operands[2]->kind, ExprKind::Ternary);
}

TEST(DslParserTest, PostfixOperatorsChain) {
    const auto expr = parse_expr("a.b[1].c(2)");
    ASSERT_EQ(expr->kind, ExprKind::Call);
    const auto& callee = expr->operands[0];
    EXPECT_EQ(callee->kind, ExprKind::FieldAccess);
    EXPECT_EQ(callee->text, "c");
    EXPECT_EQ(callee->operands[0]->kind, ExprKind::ElementAccess);
}

TEST(DslParserTest, CastAndTypeTestAreDistinctFromFieldAccess) {
    // `.as(T)` and `.is(T)` are postfix operators, not fields (§7.2.2.6.5).
    const auto cast = parse_expr("x.as(int)");
    EXPECT_EQ(cast->kind, ExprKind::Cast);
    EXPECT_EQ(cast->type_name, "int");
    const auto test = parse_expr("x.is(car)");
    EXPECT_EQ(test->kind, ExprKind::TypeTest);
    EXPECT_EQ(test->type_name, "car");
    // A plain field is still a field.
    EXPECT_EQ(parse_expr("x.speed")->kind, ExprKind::FieldAccess);
}

TEST(DslParserTest, ListAndRangeConstructorsTakeBothForms) {
    const auto list = parse_expr("[1, 2, 3]");
    EXPECT_EQ(list->kind, ExprKind::ListConstructor);
    EXPECT_EQ(list->operands.size(), 3U);
    // range-constructor ::= 'range' '(' a ',' b ')' | '[' a '..' b ']'
    const auto bracket_range = parse_expr("[1..5]");
    EXPECT_EQ(bracket_range->kind, ExprKind::RangeConstructor);
    EXPECT_EQ(bracket_range->operands.size(), 2U);
    const auto call_range = parse_expr("range(1, 5)");
    EXPECT_EQ(call_range->kind, ExprKind::RangeConstructor);
}

TEST(DslParserTest, EnumValueReferencesTakeBothForms) {
    const auto qualified = parse_expr("side!left");
    EXPECT_EQ(qualified->kind, ExprKind::EnumValue);
    EXPECT_EQ(qualified->type_name, "side");
    EXPECT_EQ(qualified->text, "left");
    // Without the enum name it is just a name until the type system resolves it.
    EXPECT_EQ(parse_expr("left")->kind, ExprKind::Name);
}

TEST(DslParserTest, ItIsAPrimaryExpression) {
    const auto expr = parse_expr("it");
    EXPECT_EQ(expr->kind, ExprKind::Name);
    EXPECT_EQ(expr->text, "it");
}

TEST(DslParserTest, InIsARelationalOperator) {
    const auto expr = parse_expr("x in [1..5]");
    ASSERT_EQ(expr->kind, ExprKind::Binary);
    EXPECT_EQ(expr->text, "in");
}

TEST(DslParserTest, ArgumentsComePositionalThenNamed) {
    const File file = parse_ok("scenario s:\n    do a(1, 2, x: 3, y: 4)\n");
    const auto& arguments = only_structured(file).members.front().behavior->arguments;
    ASSERT_EQ(arguments.size(), 4U);
    EXPECT_TRUE(arguments[0].name.empty());
    EXPECT_TRUE(arguments[1].name.empty());
    EXPECT_EQ(arguments[2].name, "x");
    EXPECT_EQ(arguments[3].name, "y");
}

TEST(DslParserTest, APositionalArgumentAfterANamedOneIsReported) {
    // §7.2.2.5.2 puts every positional argument before every named one.
    const std::vector<scena::Diagnostic> errors = parse_errors("scenario s:\n    do a(x: 1, 2)\n");
    ASSERT_FALSE(errors.empty());
    EXPECT_NE(errors.front().message.find("7.2.2.5.2"), std::string::npos);
}

// --- reserved words in non-keyword positions -------------------------------

TEST(DslParserTest, AReservedWordIsUsableWhereTheGrammarWantsAnIdentifier) {
    // §7.2.1.5.1: keywords are recognized only "in the places identified in the
    // grammar". A field named `type` is legal, and this is the test that the
    // lexer's no-Keyword-token-kind decision buys (ADR-0027).
    const File file = parse_ok("struct s:\n    type: int\n    factor: float\n");
    const auto& members = only_structured(file).members;
    ASSERT_EQ(members.size(), 2U);
    EXPECT_EQ(members[0].field.names.front(), "type");
    EXPECT_EQ(members[1].field.names.front(), "factor");
}

TEST(DslParserTest, AnEscapedIdentifierNamesAKeyword) {
    const File file = parse_ok("struct s:\n    |scenario|: int\n");
    EXPECT_EQ(only_structured(file).members.front().field.names.front(), "scenario");
}

// --- error recovery --------------------------------------------------------

TEST(DslParserTest, ParsingContinuesAfterAnError) {
    // The exit criteria ask for many useful diagnostics, not the first one. A
    // bad declaration must not swallow the good ones after it.
    DiagnosticSink sink;
    File file;
    EXPECT_EQ(scena::dsl::parse_source("struct a\nnonsense here\nstruct b\n", file, sink),
              Status::ValidationError);
    ASSERT_GE(file.declarations.size(), 2U);
    EXPECT_EQ(file.declarations.front().structured.name, "a");
    EXPECT_EQ(file.declarations.back().structured.name, "b");
}

TEST(DslParserTest, ABadMemberDoesNotLoseItsSiblings) {
    DiagnosticSink sink;
    File file;
    EXPECT_EQ(
        scena::dsl::parse_source("struct s:\n    a: int\n    def\n    b: float\n", file, sink),
        Status::ValidationError);
    ASSERT_EQ(file.declarations.size(), 1U);
    const auto& members = file.declarations.front().structured.members;
    // `a` and `b` both parsed; only the malformed `def` was dropped.
    ASSERT_EQ(members.size(), 2U);
    EXPECT_EQ(members[0].field.names.front(), "a");
    EXPECT_EQ(members[1].field.names.front(), "b");
}

TEST(DslParserTest, ManyErrorsAreReportedInOneRun) {
    const std::vector<scena::Diagnostic> errors =
        parse_errors("struct a:\n    def\n    def\nstruct b:\n    def\n");
    EXPECT_GE(errors.size(), 3U);
}

TEST(DslParserTest, DiagnosticsSayWhatWasExpectedAndWhereAndCiteTheSection) {
    const std::vector<scena::Diagnostic> errors = parse_errors("struct s:\n    a: = 1\n");
    ASSERT_FALSE(errors.empty());
    const scena::Diagnostic& first = errors.front();
    EXPECT_NE(first.message.find("expected"), std::string::npos);
    EXPECT_NE(first.message.find("7.2.2"), std::string::npos);
    EXPECT_GT(first.location.line, 0);
    EXPECT_GT(first.location.column, 0);
    // The DSL standard defines no rule ids at all, so this stays empty.
    EXPECT_TRUE(first.rule_id.empty());
}

TEST(DslParserTest, ALexicalErrorDoesNotPreventParsing) {
    // The token stream is well formed whatever the lexer found, so both sets of
    // diagnostics reach the caller in one run.
    DiagnosticSink sink;
    File file;
    EXPECT_EQ(scena::dsl::parse_source("struct a\nstruct $\nstruct b\n", file, sink),
              Status::ValidationError);
    EXPECT_GE(file.declarations.size(), 2U);
}

// --- a whole file ----------------------------------------------------------

TEST(DslParserTest, ARealisticScenarioFileParses) {
    constexpr std::string_view kSource = R"(import osc.standard.all

namespace dut use osc

type speed is SI(m: 1, s: -1)
unit kph of speed is SI(m: 1, s: -1, factor: 0.277778)

enum side: [left, right]

actor vehicle:
    length: float = 4.5
    var current_speed: speed

scenario dut.cut_in:
    ego: vehicle
    cutter: vehicle
    gap: float with:
        keep(it > 5.0)
    event started is rise(cutter.current_speed > 0.0kph)
    do serial:
        phase1: parallel(duration: 5s):
            ego.drive() with:
                speed(30kph)
            cutter.drive() with:
                lane(side: left)
                until @started
        phase2: cutter.change_lane(side: side!right) with:
            keep(it.duration == 3.0s)
        wait @started
)";
    const File file = parse_ok(kSource);
    ASSERT_EQ(file.declarations.size(), 7U);
    EXPECT_EQ(file.declarations[0].kind, Declaration::Kind::Import);
    EXPECT_EQ(file.declarations[1].kind, Declaration::Kind::Namespace);
    EXPECT_EQ(file.declarations[2].kind, Declaration::Kind::PhysicalType);
    EXPECT_EQ(file.declarations[3].kind, Declaration::Kind::Unit);
    EXPECT_EQ(file.declarations[4].kind, Declaration::Kind::Enum);
    EXPECT_EQ(file.declarations[5].structured.kind, StructuredKind::Actor);

    const auto& scenario = file.declarations[6].structured;
    EXPECT_EQ(scenario.kind, StructuredKind::Scenario);
    EXPECT_EQ(scenario.name, "dut.cut_in");
    // Two actor fields, one constrained parameter, one event, one do directive.
    ASSERT_EQ(scenario.members.size(), 5U);
    EXPECT_EQ(scenario.members[4].kind, Member::Kind::Behavior);
    const auto& root = scenario.members[4].behavior;
    ASSERT_EQ(root->members.size(), 3U);
    EXPECT_EQ(root->members[0]->label, "phase1");
    EXPECT_EQ(root->members[0]->members.size(), 2U);
    EXPECT_EQ(root->members[1]->label, "phase2");
    EXPECT_EQ(root->members[2]->kind, DoMemberKind::Wait);
}

TEST(DslParserTest, ParsingIsDeterministicAcrossRepeats) {
    constexpr std::string_view kSource =
        "scenario s:\n    a: int = 1\n    do serial:\n        x(1.25m)\n";
    const File first = parse_ok(kSource);
    const File second = parse_ok(kSource);
    ASSERT_EQ(first.declarations.size(), second.declarations.size());
    const auto& a = first.declarations.front().structured;
    const auto& b = second.declarations.front().structured;
    ASSERT_EQ(a.members.size(), b.members.size());
    for (std::size_t i = 0; i < a.members.size(); ++i) {
        EXPECT_EQ(static_cast<int>(a.members[i].kind), static_cast<int>(b.members[i].kind));
        EXPECT_EQ(a.members[i].range.line, b.members[i].range.line);
        EXPECT_EQ(a.members[i].range.column, b.members[i].range.column);
    }
}

} // namespace
