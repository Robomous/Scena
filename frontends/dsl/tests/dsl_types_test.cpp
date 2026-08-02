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
// Symbols and the type system (p7-s3, #41): name resolution across namespaces,
// the §7.3 type rules, unit conversion, inheritance (both kinds), extension,
// and a negative fixture for each rule the resolver enforces.
//
// Every source fragment is written from the specification text (ADR-0002).
//

#include <string>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

#include "scena/diagnostic.h"
#include "scena/dsl/parser.h"
#include "scena/dsl/resolve.h"
#include "scena/dsl/stdlib.h"
#include "scena/dsl/types.h"
#include "scena/status.h"

namespace {

using scena::DiagnosticSink;
using scena::Severity;
using scena::Status;
using scena::dsl::File;
using scena::dsl::Program;
using scena::dsl::to_base_units;
using scena::dsl::TypeInfo;
using scena::dsl::TypeKind;
using scena::dsl::Unit;

/// Parses and resolves `source`, asserting that nothing was reported.
Program resolve_ok(std::string_view source) {
    DiagnosticSink sink;
    File file;
    EXPECT_EQ(scena::dsl::parse_source(source, file, sink), Status::Ok);
    Program program;
    const Status status = scena::dsl::resolve(file, program, sink);
    for (const scena::Diagnostic& diagnostic : sink.diagnostics()) {
        EXPECT_NE(diagnostic.severity, Severity::Error) << diagnostic.message;
    }
    EXPECT_EQ(status, Status::Ok);
    return program;
}

/// The bundled types sub-module, parsed once, as a file of its own. The
/// standard library is a separate translation unit (§7.7.5.2), never text
/// pasted in front of the source under test.
const File& standard_types() {
    static const File parsed = [] {
        File file;
        DiagnosticSink sink;
        (void)scena::dsl::parse_source(
            scena::dsl::standard_module_source(scena::dsl::kStandardTypesModule),
            std::string(scena::dsl::kStandardTypesModule), file, sink);
        file.is_standard_library = true;
        return file;
    }();
    return parsed;
}

/// `resolve_errors`, with the bundled types sub-module alongside `source`.
std::vector<scena::Diagnostic> resolve_errors_std(std::string_view source) {
    DiagnosticSink sink;
    File file;
    (void)scena::dsl::parse_source(source, file, sink);
    Program program;
    const std::vector<const File*> files{&standard_types(), &file};
    EXPECT_EQ(scena::dsl::resolve(files, program, sink), Status::ValidationError);
    return sink.diagnostics();
}

/// Resolves `source` and returns the diagnostics, asserting that it failed.
std::vector<scena::Diagnostic> resolve_errors(std::string_view source) {
    DiagnosticSink sink;
    File file;
    (void)scena::dsl::parse_source(source, file, sink);
    Program program;
    EXPECT_EQ(scena::dsl::resolve(file, program, sink), Status::ValidationError);
    return sink.diagnostics();
}

/// True when some diagnostic's message contains `needle`.
bool mentions(const std::vector<scena::Diagnostic>& diagnostics, std::string_view needle) {
    for (const scena::Diagnostic& diagnostic : diagnostics) {
        if (diagnostic.message.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

TEST(DslTypesTest, PrimitiveTypesAreAlwaysPresent) {
    // §7.3.2: their names are keywords, not namespaced identifiers.
    const Program program = resolve_ok("struct s\n");
    for (const char* name : {"bool", "int", "uint", "float", "string"}) {
        const TypeInfo* type = program.find(name);
        ASSERT_NE(type, nullptr) << name;
        EXPECT_EQ(type->name, name);
    }
}

// --- physical types and units (§7.3.4) -------------------------------------

TEST(DslTypesTest, SiExponentsAccumulateIntoADimension) {
    const Program program = resolve_ok("type acceleration is SI(m: 1, s: -2)\n");
    const TypeInfo* acceleration = program.find("::acceleration");
    ASSERT_NE(acceleration, nullptr);
    EXPECT_EQ(acceleration->dimension.to_string(), "m*s^-2");
    EXPECT_FALSE(acceleration->dimension.is_dimensionless());
}

TEST(DslTypesTest, AUnitMustCarryItsPhysicalTypesDimension) {
    // §7.3.4: "All units of a physical type have identical exponents for all SI
    // base units and the radian" — that is what makes conversion direct.
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("type speed is SI(m: 1, s: -1)\nunit bad of speed is SI(m: 1)\n");
    EXPECT_TRUE(mentions(errors, "§7.3.4"));
    EXPECT_TRUE(mentions(errors, "m*s^-1"));
}

TEST(DslTypesTest, UnitNamesAreGloballyUnique) {
    // §7.3.4: "Unit names form their own separate global namespace."
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("type length is SI(m: 1)\ntype other is SI(m: 1)\n"
                       "unit u of length is SI(m: 1)\nunit u of other is SI(m: 1)\n");
    EXPECT_TRUE(mentions(errors, "already declared"));
}

TEST(DslTypesTest, AnUnknownSiBaseUnitIsReported) {
    const std::vector<scena::Diagnostic> errors = resolve_errors("type bogus is SI(furlong: 1)\n");
    EXPECT_TRUE(mentions(errors, "not an SI base unit"));
    EXPECT_TRUE(mentions(errors, "kg, m, s, A, K, mol, cd and rad"));
}

TEST(DslTypesTest, AUnitOfSomethingThatIsNotAPhysicalTypeIsReported) {
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("struct thing\nunit u of thing is SI(m: 1)\n");
    EXPECT_TRUE(mentions(errors, "not a physical type"));
}

// --- enumerations (§7.3.3) -------------------------------------------------

TEST(DslTypesTest, EnumValuesSucceedTheLastExplicitOne) {
    // §7.3.3: implicit values derive "using succeeding integer values from the
    // last explicitly given literal integer value, or from 0".
    const Program program = resolve_ok("enum cmyk: [cyan = 1, magenta = 2, yellow, black]\n");
    const TypeInfo* cmyk = program.find("::cmyk");
    ASSERT_NE(cmyk, nullptr);
    ASSERT_EQ(cmyk->enum_members.size(), 4U);
    EXPECT_EQ(cmyk->enum_members[0].value, 1U);
    EXPECT_EQ(cmyk->enum_members[1].value, 2U);
    EXPECT_EQ(cmyk->enum_members[2].value, 3U);
    EXPECT_EQ(cmyk->enum_members[3].value, 4U);
}

TEST(DslTypesTest, AnEnumMemberMayTakeItsValueFromAnother) {
    // §7.3.3's `gray = grey`: equivalent members compare equal, and a reference
    // "is ignored for the purposes of deriving succeeding implicit values".
    const Program program =
        resolve_ok("enum named: [tan, mauve, pink, grey, gray = grey, violet, brown]\n");
    const TypeInfo* named = program.find("::named");
    ASSERT_NE(named, nullptr);
    ASSERT_EQ(named->enum_members.size(), 7U);
    EXPECT_EQ(named->enum_members[3].value, 3U); // grey
    EXPECT_EQ(named->enum_members[4].value, 3U); // gray == grey
    EXPECT_EQ(named->enum_members[5].value, 4U); // violet keeps counting from grey
    EXPECT_EQ(named->enum_members[6].value, 5U);
}

TEST(DslTypesTest, AnEnumExtensionContinuesTheNumbering) {
    // §7.3.3: on extension, "the last existing enumeration member giving the
    // starting point".
    const Program program = resolve_ok("enum rgb: [red, green, blue]\nextend rgb: [alpha]\n");
    const TypeInfo* rgb = program.find("::rgb");
    ASSERT_NE(rgb, nullptr);
    ASSERT_EQ(rgb->enum_members.size(), 4U);
    EXPECT_EQ(rgb->enum_members[3].name, "alpha");
    EXPECT_EQ(rgb->enum_members[3].value, 3U);
}

TEST(DslTypesTest, ADuplicateEnumMemberIsReported) {
    const std::vector<scena::Diagnostic> errors = resolve_errors("enum side: [left, left]\n");
    EXPECT_TRUE(mentions(errors, "declared twice"));
    EXPECT_TRUE(mentions(errors, "§7.3.3"));
}

TEST(DslTypesTest, AnEnumValueReferenceCycleIsReported) {
    const std::vector<scena::Diagnostic> errors = resolve_errors("enum e: [a = b, b = a]\n");
    EXPECT_TRUE(mentions(errors, "cycle"));
}

TEST(DslTypesTest, ExtendingSomethingThatIsNotAnEnumIsReported) {
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("struct thing\nextend thing: [a]\n");
    EXPECT_TRUE(mentions(errors, "not an enum"));
}

TEST(DslTypesTest, AnOverloadedEnumLiteralNamesEveryCandidate) {
    // §7.3.3: a member name used by more than one enumeration is overloaded;
    // which one is meant is a type-resolution question (p7-s4 resolves it).
    const Program program = resolve_ok("enum rgb: [red, black]\nenum cmyk: [cyan, black]\n");
    const std::vector<scena::dsl::TypeId> candidates = program.enums_declaring("black");
    ASSERT_EQ(candidates.size(), 2U);
    EXPECT_EQ(program.types[candidates[0]].name, "::cmyk"); // name order, always
    EXPECT_EQ(program.types[candidates[1]].name, "::rgb");
    EXPECT_EQ(program.enums_declaring("red").size(), 1U);
}

// --- fields and aggregates (§7.3.1, §7.3.6) --------------------------------

TEST(DslTypesTest, FieldsAreTypedAndKeepDeclarationOrder) {
    const Program program = resolve_ok("struct s:\n"
                                       "    a: int\n"
                                       "    b, c: float\n"
                                       "    var v: bool\n");
    const TypeInfo* type = program.find("::s");
    ASSERT_NE(type, nullptr);
    EXPECT_EQ(type->field_order, (std::vector<std::string>{"a", "b", "c", "v"}));
    EXPECT_EQ(program.types[type->fields.at("a").type].kind, TypeKind::Int);
    EXPECT_EQ(program.types[type->fields.at("b").type].kind, TypeKind::Float);
    EXPECT_TRUE(type->fields.at("v").is_variable);
    EXPECT_FALSE(type->fields.at("a").is_variable);
}

TEST(DslTypesTest, AggregateTypesAreStructuralAndShared) {
    const Program program = resolve_ok("struct s:\n    a: list of int\n    b: list of int\n");
    const TypeInfo* type = program.find("::s");
    ASSERT_NE(type, nullptr);
    EXPECT_EQ(type->fields.at("a").type, type->fields.at("b").type);
    EXPECT_EQ(program.types[type->fields.at("a").type].kind, TypeKind::List);
    EXPECT_EQ(type->fields.at("a").element, program.types_by_name.at("int"));
}

TEST(DslTypesTest, AListOfListsIsReported) {
    // §7.3.1: "The elements in a list can be any other type, except another
    // list type."
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("struct s:\n    a: list of int\nstruct t:\n    b: list of |list of int|\n");
    EXPECT_TRUE(mentions(errors, "§7"));
}

TEST(DslTypesTest, ARangeNeedsANumericBaseType) {
    // §7.3.1: a range is "a closed interval over a numeric base type".
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("struct s:\n    a: range of bool\n");
    EXPECT_TRUE(mentions(errors, "'range of' needs a numeric type"));
    // ... and every numeric type is accepted, physical types included.
    const Program program = resolve_ok("type speed is SI(m: 1, s: -1)\n"
                                       "struct s:\n"
                                       "    a: range of int\n"
                                       "    b: range of float\n"
                                       "    c: range of speed\n");
    EXPECT_NE(program.find("::s"), nullptr);
}

TEST(DslTypesTest, AnUnknownFieldTypeIsReported) {
    const std::vector<scena::Diagnostic> errors = resolve_errors("struct s:\n    a: nowhere\n");
    EXPECT_TRUE(mentions(errors, "unknown type 'nowhere'"));
}

TEST(DslTypesTest, ADuplicateFieldIsReported) {
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("struct s:\n    a: int\n    a: float\n");
    EXPECT_TRUE(mentions(errors, "declared twice"));
}

// --- literal defaults (§7.3.2, §7.3.4) -------------------------------------

TEST(DslTypesTest, APhysicalFieldNeedsAUnitOnItsValue) {
    // §7.3.4: "A unit specification is a mandatory part of any physical type
    // literal."
    const std::vector<scena::Diagnostic> errors =
        resolve_errors_std("struct s:\n    v: speed = 10.0\n");
    EXPECT_TRUE(mentions(errors, "needs a unit"));
}

TEST(DslTypesTest, AUnitOfTheWrongDimensionIsReported) {
    const std::vector<scena::Diagnostic> errors =
        resolve_errors_std("struct s:\n    v: speed = 10.0m\n");
    EXPECT_TRUE(mentions(errors, "but 'speed' is"));
}

TEST(DslTypesTest, ALiteralOfTheWrongPrimitiveTypeIsReported) {
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("struct s:\n    a: int = true\n    b: string = 1\n");
    EXPECT_TRUE(mentions(errors, "cannot hold this literal"));
}

TEST(DslTypesTest, IntegerLiteralsConvertImplicitlyToFloat) {
    // §7.3.2.3: "The conversion from int and uint to float is implicit".
    const Program program = resolve_ok("struct s:\n    a: float = 1\n    b: float = -1\n");
    EXPECT_NE(program.find("::s"), nullptr);
}

TEST(DslTypesTest, AnUnknownUnitIsReported) {
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("type speed is SI(m: 1, s: -1)\nstruct s:\n    v: speed = 10.0furlongs\n");
    EXPECT_TRUE(mentions(errors, "unknown unit 'furlongs'"));
}

// --- inheritance (§7.3.8) --------------------------------------------------

TEST(DslTypesTest, AnInheritingTypeSeesItsSupertypesFields) {
    const Program program =
        resolve_ok("struct base:\n    f1: bool\nstruct derived inherits base:\n    f2: bool\n");
    const TypeInfo* derived = program.find("::derived");
    ASSERT_NE(derived, nullptr);
    ASSERT_NE(derived->base, scena::dsl::kInvalidType);
    const scena::dsl::TypeId id = program.types_by_name.at("::derived");
    EXPECT_NE(program.find_field(id, "f1"), nullptr); // inherited
    EXPECT_NE(program.find_field(id, "f2"), nullptr); // its own
    EXPECT_TRUE(program.is_derived_from(id, program.types_by_name.at("::base")));
    EXPECT_FALSE(program.is_derived_from(program.types_by_name.at("::base"), id));
}

TEST(DslTypesTest, AFieldCannotShadowAnInheritedOne) {
    // §7.3.9: "Features in an extension cannot shadow previously declared
    // features."
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("struct base:\n    f: bool\nstruct derived inherits base:\n    f: int\n");
    EXPECT_TRUE(mentions(errors, "cannot be shadowed"));
}

TEST(DslTypesTest, ConditionalInheritanceRecordsItsDeterminant) {
    const Program program = resolve_ok("enum category: [car, truck]\n"
                                       "actor vehicle:\n"
                                       "    vehicle_category: category\n"
                                       "    is_electric: bool\n"
                                       "actor truck inherits vehicle(vehicle_category == truck)\n"
                                       "actor ev inherits vehicle(is_electric == true)\n");
    const TypeInfo* truck = program.find("::truck");
    ASSERT_NE(truck, nullptr);
    EXPECT_TRUE(truck->is_conditional);
    EXPECT_EQ(truck->constraint_field, "vehicle_category");
    const TypeInfo* ev = program.find("::ev");
    ASSERT_NE(ev, nullptr);
    EXPECT_TRUE(ev->is_conditional);
    EXPECT_EQ(ev->constraint_field, "is_electric");
}

TEST(DslTypesTest, AConditionalTypeCannotBeInheritedUnconditionally) {
    // §7.3.8.2.3 Rule 1.
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("enum category: [car, truck]\n"
                       "actor vehicle:\n"
                       "    vehicle_category: category\n"
                       "actor car inherits vehicle(vehicle_category == car)\n"
                       "actor coupe inherits car\n");
    EXPECT_TRUE(mentions(errors, "§7.3.8.2.3"));
}

TEST(DslTypesTest, AConditionalSubtypeMayInheritConditionally) {
    // §7.3.8.2.5's police car: conditional subtrees may nest.
    const Program program =
        resolve_ok("enum category: [car, truck]\n"
                   "actor vehicle:\n"
                   "    vehicle_category: category\n"
                   "    emergency_vehicle: bool\n"
                   "actor car inherits vehicle(vehicle_category == car)\n"
                   "actor police_car inherits car(emergency_vehicle == true)\n");
    const TypeInfo* police = program.find("::police_car");
    ASSERT_NE(police, nullptr);
    EXPECT_TRUE(police->is_conditional);
    EXPECT_TRUE(program.is_derived_from(program.types_by_name.at("::police_car"),
                                        program.types_by_name.at("::vehicle")));
}

TEST(DslTypesTest, AGuardOnAnUnknownFieldIsReported) {
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("actor vehicle\nactor truck inherits vehicle(nothing == true)\n");
    EXPECT_TRUE(mentions(errors, "has no field 'nothing'"));
}

TEST(DslTypesTest, AGuardNeedsABoolOrEnumField) {
    // §7.3.8.2: subtypes depend "upon a value of a Boolean or enumerated field".
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("actor vehicle:\n    weight: int\n"
                       "actor heavy inherits vehicle(weight == 1)\n");
    EXPECT_TRUE(mentions(errors, "needs a bool or enum field"));
}

TEST(DslTypesTest, AGuardValueMustBelongToTheEnum) {
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("enum category: [car, truck]\n"
                       "actor vehicle:\n    vehicle_category: category\n"
                       "actor bus inherits vehicle(vehicle_category == tractor)\n");
    EXPECT_TRUE(mentions(errors, "has no member 'tractor'"));
}

TEST(DslTypesTest, InheritanceAcrossKindsIsReported) {
    // §7.3.8.1 applies inheritance within structs, actors, scenarios, actions.
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("struct base\nactor derived inherits base\n");
    EXPECT_TRUE(mentions(errors, "cannot inherit from"));
}

TEST(DslTypesTest, AnInheritanceCycleIsReported) {
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("struct a inherits b\nstruct b inherits a\n");
    EXPECT_TRUE(mentions(errors, "inherits from itself"));
}

TEST(DslTypesTest, AnActorBehaviorMustInheritAnActorBehavior) {
    // §7.3.8.1: "scenarios and actions not belonging to an actor must only
    // inherit from a scenario or action not belonging to an actor".
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("actor vehicle\nscenario free_form\n"
                       "scenario vehicle.drive inherits free_form\n");
    EXPECT_TRUE(mentions(errors, "§7.3.8.1"));
}

// --- extension (§7.3.9) ----------------------------------------------------

TEST(DslTypesTest, AnExtensionAddsMembersToTheTypeItself) {
    const Program program = resolve_ok("struct car:\n    color: int\n"
                                       "extend car:\n    weight: float\n");
    const TypeInfo* car = program.find("::car");
    ASSERT_NE(car, nullptr);
    EXPECT_EQ(car->field_order, (std::vector<std::string>{"color", "weight"}));
    EXPECT_EQ(car->declarations.size(), 2U);
}

TEST(DslTypesTest, AnExtensionMayComeBeforeTheDeclaration) {
    // §7.3.15: "there are no restrictions on the ordering of type use and type
    // declaration in terms of textual ordering".
    const Program program = resolve_ok("extend car:\n    weight: float\n"
                                       "struct car:\n    color: int\n");
    const TypeInfo* car = program.find("::car");
    ASSERT_NE(car, nullptr);
    EXPECT_EQ(car->fields.size(), 2U);
}

TEST(DslTypesTest, AnExtensionCannotShadowAnExistingField) {
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("struct car:\n    color: int\nextend car:\n    color: float\n");
    EXPECT_TRUE(mentions(errors, "declared twice"));
}

TEST(DslTypesTest, AnExtensionCannotIntroduceInheritance) {
    // §7.2.2.2.6's structured-type-extension has no `inherits` clause —
    // extension modifies a type, inheritance declares a new one (§7.3.9). The
    // grammar already rules it out, so the parser is where it is caught; the
    // resolver keeps the same check for an AST built by any other route.
    DiagnosticSink sink;
    File file;
    EXPECT_EQ(
        scena::dsl::parse_source("struct base\nstruct car\nextend car inherits base\n", file, sink),
        Status::ValidationError);
    EXPECT_TRUE(mentions(sink.diagnostics(), "found 'inherits'"));
}

TEST(DslTypesTest, ExtendingAnUnknownTypeIsReported) {
    const std::vector<scena::Diagnostic> errors = resolve_errors("extend nowhere:\n    a: int\n");
    EXPECT_TRUE(mentions(errors, "unknown type 'nowhere'"));
}

TEST(DslTypesTest, ADuplicateDeclarationIsReported) {
    const std::vector<scena::Diagnostic> errors = resolve_errors("struct s\nstruct s\n");
    EXPECT_TRUE(mentions(errors, "already declared"));
    EXPECT_TRUE(mentions(errors, "use 'extend'"));
}

// --- methods (§7.3.7) ------------------------------------------------------

TEST(DslTypesTest, MethodsAreTypedWithTheirParameters) {
    const Program program = resolve_ok("struct s:\n"
                                       "    def area(w: float, h: float) -> float is expression w\n"
                                       "    def nothing() is undefined\n");
    const TypeInfo* type = program.find("::s");
    ASSERT_NE(type, nullptr);
    ASSERT_EQ(type->methods.size(), 2U);
    const scena::dsl::MethodInfo& area = type->methods.at("area");
    ASSERT_EQ(area.parameters.size(), 2U);
    EXPECT_EQ(area.parameters[0].name, "w");
    EXPECT_EQ(program.types[area.parameters[0].type].kind, TypeKind::Float);
    EXPECT_EQ(program.types[area.return_type].kind, TypeKind::Float);
    EXPECT_EQ(type->methods.at("nothing").implementation, "undefined");
    EXPECT_EQ(type->methods.at("nothing").return_type, scena::dsl::kInvalidType);
}

TEST(DslTypesTest, IsOnlyOverridesASupertypeMethod) {
    // §7.3.7.2.
    const Program program =
        resolve_ok("struct base:\n    def f() -> int is undefined\n"
                   "struct derived inherits base:\n    def f() -> int is only expression 42\n");
    const TypeInfo* derived = program.find("::derived");
    ASSERT_NE(derived, nullptr);
    EXPECT_EQ(derived->methods.at("f").qualifier, "only");
}

TEST(DslTypesTest, IsOnlyWithoutASupertypeMethodIsReported) {
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("struct s:\n    def f() -> int is only expression 1\n");
    EXPECT_TRUE(mentions(errors, "§7.3.7.2"));
}

TEST(DslTypesTest, AnOverrideKeepsTheReturnType) {
    const std::vector<scena::Diagnostic> errors = resolve_errors(
        "struct base:\n    def f() -> int is undefined\n"
        "struct derived inherits base:\n    def f() -> float is only expression 1\n");
    EXPECT_TRUE(mentions(errors, "must keep the supertype's return type"));
}

TEST(DslTypesTest, ADuplicateMethodParameterIsReported) {
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("struct s:\n    def f(a: int, a: int) is undefined\n");
    EXPECT_TRUE(mentions(errors, "declared twice"));
}

// --- events (§7.3.10) ------------------------------------------------------

TEST(DslTypesTest, EventsAreRecordedWithTheirParameters) {
    const Program program = resolve_ok("scenario s:\n    event started(at: float)\n");
    const TypeInfo* type = program.find("::s");
    ASSERT_NE(type, nullptr);
    ASSERT_EQ(type->events.size(), 1U);
    EXPECT_EQ(type->events.at("started").parameters.size(), 1U);
    EXPECT_EQ(type->event_order, (std::vector<std::string>{"started"}));
}

TEST(DslTypesTest, ADuplicateEventIsReported) {
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("scenario s:\n    event e\n    event e\n");
    EXPECT_TRUE(mentions(errors, "§7.3.10.2"));
}

// --- structured-type restrictions (§7.3.5) ---------------------------------

TEST(DslTypesTest, AStructHasNoBehaviorSpecification) {
    // §7.3.5.1: scenarios and actions carry the do statement, not structs.
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("struct s:\n    do serial:\n        a()\n");
    EXPECT_TRUE(mentions(errors, "has no behavior specification"));
}

TEST(DslTypesTest, TwoDoDirectivesAreReported) {
    // §7.3.9: "not more than one do directive can be effectively present".
    const std::vector<scena::Diagnostic> errors = resolve_errors("scenario s:\n"
                                                                 "    do serial:\n"
                                                                 "        a()\n"
                                                                 "    do serial:\n"
                                                                 "        b()\n");
    EXPECT_TRUE(mentions(errors, "more than one 'do'"));
}

TEST(DslTypesTest, AnExtensionAddingASecondDoIsReported) {
    const std::vector<scena::Diagnostic> errors = resolve_errors("scenario s:\n"
                                                                 "    do serial:\n"
                                                                 "        a()\n"
                                                                 "extend s:\n"
                                                                 "    do serial:\n"
                                                                 "        b()\n");
    EXPECT_TRUE(mentions(errors, "after extension"));
}

// --- modifiers (§7.3.12) ---------------------------------------------------

TEST(DslTypesTest, ModifierAssociationIsResolved) {
    const Program program = resolve_ok("actor vehicle\n"
                                       "scenario vehicle.drive\n"
                                       "modifier force_lane\n"
                                       "modifier vehicle.keep_lane\n"
                                       "modifier vehicle.follow of vehicle.drive\n");
    // Unassociated, actor-associated and scenario-associated (§7.3.12.3).
    const TypeInfo* unassociated = program.find("::force_lane");
    ASSERT_NE(unassociated, nullptr);
    EXPECT_TRUE(unassociated->actor.empty());
    const TypeInfo* actor_associated = program.find("::vehicle.keep_lane");
    ASSERT_NE(actor_associated, nullptr);
    EXPECT_EQ(actor_associated->actor, "vehicle");
    EXPECT_EQ(actor_associated->actor_type, program.types_by_name.at("::vehicle"));
    const TypeInfo* scenario_associated = program.find("::vehicle.follow");
    ASSERT_NE(scenario_associated, nullptr);
    EXPECT_EQ(scenario_associated->modifies_type, program.types_by_name.at("::vehicle.drive"));
}

TEST(DslTypesTest, AModifiersOfNamesAScenarioOrAction) {
    // §7.3.12.2.
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("struct thing\nmodifier m of thing\n");
    EXPECT_TRUE(mentions(errors, "§7.3.12.2"));
}

TEST(DslTypesTest, ApplyingAnUnknownModifierIsReported) {
    const std::vector<scena::Diagnostic> errors = resolve_errors("scenario s:\n    nowhere(1)\n");
    EXPECT_TRUE(mentions(errors, "unknown modifier 'nowhere'"));
}

TEST(DslTypesTest, AModifierArgumentMustNameAParameter) {
    // §7.3.12.4: an argument is a constraint on a parameter field of the
    // modifier, so it has to name one.
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("modifier speed:\n    target: int\n"
                       "scenario s:\n    speed(nothing: 1)\n");
    EXPECT_TRUE(mentions(errors, "has no parameter 'nothing'"));
    // The declared one is accepted.
    const Program program = resolve_ok("modifier speed:\n    target: int\n"
                                       "scenario s:\n    speed(target: 1)\n");
    EXPECT_NE(program.find("::s"), nullptr);
}

// --- modifier application (§7.3.12.4) --------------------------------------
//
// Until #100 was fixed an actor-associated modifier could not be applied at
// all: the declaration `modifier thing.tweak` interns the name in the actor
// scope (§7.3.12.2), and the application site looked it up in the namespace's
// type table, where it never was.

TEST(DslTypesTest, AnActorAssociatedModifierAppliesToItsActor) {
    const Program program = resolve_ok("actor thing\n"
                                       "modifier thing.tweak:\n    v: int\n"
                                       "scenario thing.demo:\n"
                                       "    t: thing\n"
                                       "    t.tweak(1)\n");
    EXPECT_NE(program.find("::thing.tweak"), nullptr);
}

TEST(DslTypesTest, AnActorAssociatedModifierReachesASubtype) {
    // §7.3.12.4.1 resolves the actor expression's type; a subtype of the
    // associated actor is still that actor.
    const Program program = resolve_ok("actor base\n"
                                       "actor derived inherits base\n"
                                       "modifier base.tweak:\n    v: int\n"
                                       "scenario base.demo:\n"
                                       "    d: derived\n"
                                       "    d.tweak(1)\n");
    EXPECT_NE(program.find("::base.tweak"), nullptr);
}

TEST(DslTypesTest, AnActorAppliesItsOwnModifierWithoutNamingItself) {
    // §7.3.12.4.2's third example: applied within an actor declaration, so it
    // applies to every instance of that actor. No actor expression to write.
    const Program program = resolve_ok("actor base\n"
                                       "modifier base.tweak:\n    v: int\n"
                                       "actor derived inherits base:\n"
                                       "    tweak(1)\n");
    EXPECT_NE(program.find("::derived"), nullptr);
}

TEST(DslTypesTest, AModifierOfAnUnrelatedActorIsRejected) {
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("actor thing\nactor other\n"
                       "modifier thing.tweak:\n    v: int\n"
                       "scenario thing.demo:\n"
                       "    o: other\n"
                       "    o.tweak(1)\n");
    EXPECT_TRUE(mentions(errors, "belongs to 'thing'"));
    EXPECT_TRUE(mentions(errors, "§7.3.12.4"));
}

TEST(DslTypesTest, AWithBlockValidatesTheModifiersItApplies) {
    // The larger half of #100: a `with:` block used to accept any name at all,
    // and it is where the domain model expects nearly every modifier to be
    // applied.
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("actor thing:\n    def go() is undefined\n"
                       "scenario thing.demo:\n"
                       "    t: thing\n"
                       "    do serial:\n"
                       "        t.go() with:\n"
                       "            no_such_modifier(1)\n");
    EXPECT_TRUE(mentions(errors, "unknown modifier 'no_such_modifier'"));
}

TEST(DslTypesTest, AWithBlockOmitsTheActorItIsAlreadyApplyingTo) {
    // §7.3.12.4.1: "A modifier-associated actor can be omitted when it is the
    // same as the scenario actor the modifier is applied in" — the invocation's
    // actor is the receiver.
    const Program program = resolve_ok("actor thing:\n    def go() is undefined\n"
                                       "modifier thing.tweak:\n    v: int\n"
                                       "scenario thing.demo:\n"
                                       "    t: thing\n"
                                       "    do serial:\n"
                                       "        t.go() with:\n"
                                       "            tweak(1)\n");
    EXPECT_NE(program.find("::thing.tweak"), nullptr);
}

TEST(DslTypesTest, AWithBlockChecksArgumentNamesToo) {
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("actor thing:\n    def go() is undefined\n"
                       "modifier thing.tweak:\n    v: int\n"
                       "scenario thing.demo:\n"
                       "    t: thing\n"
                       "    do serial:\n"
                       "        t.go() with:\n"
                       "            tweak(nope: 1)\n");
    EXPECT_TRUE(mentions(errors, "has no parameter 'nope'"));
}

TEST(DslTypesTest, ANameThatIsNotAModifierSaysWhatItIs) {
    // Better than "unknown": at library scale the interesting failure is a name
    // that exists and is the wrong kind of thing.
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("struct tweak\nscenario s:\n    tweak(1)\n");
    EXPECT_TRUE(mentions(errors, "not a modifier"));
}

TEST(DslTypesTest, AScenarioAssociatedModifierIsRejectedElsewhere) {
    const std::vector<scena::Diagnostic> errors = resolve_errors("scenario drive\n"
                                                                 "modifier follow of drive\n"
                                                                 "scenario other:\n"
                                                                 "    follow()\n");
    EXPECT_TRUE(mentions(errors, "cannot be applied here"));
}

// --- namespaces (§7.7.4) ---------------------------------------------------

TEST(DslTypesTest, ADefinitionLandsInTheActiveNamespace) {
    const Program program = resolve_ok("namespace demo_foo\nstruct bar:\n    baz: uint\n");
    EXPECT_NE(program.find("demo_foo::bar"), nullptr);
    EXPECT_EQ(program.find("::bar"), nullptr);
}

TEST(DslTypesTest, AQualifiedIdentifierReachesAnotherNamespace) {
    const Program program = resolve_ok("namespace one\nstruct bar\n"
                                       "namespace two\nstruct baz:\n    b: one::bar\n");
    const TypeInfo* baz = program.find("two::baz");
    ASSERT_NE(baz, nullptr);
    EXPECT_EQ(baz->fields.at("b").type, program.types_by_name.at("one::bar"));
}

TEST(DslTypesTest, TheUseListMakesExportedNamesVisible) {
    const Program program = resolve_ok("namespace foo\nexport bar\nstruct bar\n"
                                       "namespace bazzle use foo\nstruct foobar inherits bar\n");
    const TypeInfo* foobar = program.find("bazzle::foobar");
    ASSERT_NE(foobar, nullptr);
    EXPECT_EQ(foobar->base, program.types_by_name.at("foo::bar"));
}

TEST(DslTypesTest, AWildcardExportsEverythingInTheNamespace) {
    const Program program = resolve_ok("namespace foo\nexport *\nstruct bar\nstruct baz\n"
                                       "namespace other use foo\nstruct derived inherits baz\n");
    EXPECT_EQ(program.find("other::derived")->base, program.types_by_name.at("foo::baz"));
}

TEST(DslTypesTest, TheCurrentNamespaceShadowsTheUseList) {
    // §7.7.4.2 rule 2: "any identifiers from the current namespace shadow any
    // identifiers accessible from the used namespaces".
    const Program program = resolve_ok("namespace foo\nexport bar\nstruct bar:\n    marker: int\n"
                                       "namespace moo use foo\nstruct bar:\n    other: float\n"
                                       "struct user:\n    f: bar\n");
    const TypeInfo* user = program.find("moo::user");
    ASSERT_NE(user, nullptr);
    EXPECT_EQ(user->fields.at("f").type, program.types_by_name.at("moo::bar"));
}

TEST(DslTypesTest, AnUnresolvedNameIsReported) {
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("namespace one\nstruct bar\nnamespace two\nstruct s:\n    b: bar\n");
    EXPECT_TRUE(mentions(errors, "unknown type 'bar'"));
    EXPECT_TRUE(mentions(errors, "§7.7.4.2"));
}

TEST(DslTypesTest, AnAmbiguousNameFromTwoUsedNamespacesIsReported) {
    // §7.7.4.2: "An error is raised during identifier resolution if more than
    // one same-named identifier is accessible from the used namespaces."
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("namespace one\nexport bar\nstruct bar\n"
                       "namespace two\nexport bar\nstruct bar\n"
                       "namespace three use one, two\nstruct s:\n    b: bar\n");
    EXPECT_TRUE(mentions(errors, "unknown type 'bar'"));
}

TEST(DslTypesTest, TheStdPrefixIsReservedAndWarnedAbout) {
    // §7.7.4: "all identifiers starting with std are reserved". A warning, not
    // a rejection — the file is still well formed.
    DiagnosticSink sink;
    File file;
    ASSERT_EQ(scena::dsl::parse_source("namespace std_extra\nstruct s\n", file, sink), Status::Ok);
    Program program;
    EXPECT_EQ(scena::dsl::resolve(file, program, sink), Status::Ok);
    ASSERT_FALSE(sink.diagnostics().empty());
    EXPECT_EQ(sink.diagnostics().front().severity, Severity::Warning);
    EXPECT_NE(sink.diagnostics().front().message.find("reserves"), std::string::npos);
}

TEST(DslTypesTest, EachFileStartsInTheNullNamespace) {
    // §7.7.4: "Each file processing starts in the implicit null namespace" —
    // the effect of a namespace statement does not cross a file boundary.
    DiagnosticSink sink;
    File first;
    File second;
    ASSERT_EQ(scena::dsl::parse_source("namespace foo\nstruct bar\n", first, sink), Status::Ok);
    ASSERT_EQ(scena::dsl::parse_source("struct baz\n", second, sink), Status::Ok);
    Program program;
    const std::vector<const File*> files{&first, &second};
    ASSERT_EQ(scena::dsl::resolve(files, program, sink), Status::Ok);
    EXPECT_NE(program.find("foo::bar"), nullptr);
    EXPECT_NE(program.find("::baz"), nullptr);
}

// --- globals and 'it' (§7.3.14, §7.4.1.3) ----------------------------------

TEST(DslTypesTest, GlobalParametersAreTypedAndOrdered) {
    const Program program = resolve_ok("global speed_limit: float = 30.0\nglobal name: string\n");
    ASSERT_EQ(program.globals.size(), 2U);
    EXPECT_EQ(program.global_order, (std::vector<std::string>{"::speed_limit", "::name"}));
    EXPECT_EQ(program.types[program.globals.at("::speed_limit").field.type].kind, TypeKind::Float);
}

TEST(DslTypesTest, ADuplicateGlobalIsReported) {
    const std::vector<scena::Diagnostic> errors =
        resolve_errors("global a: int\nglobal a: float\n");
    EXPECT_TRUE(mentions(errors, "§7.3.14"));
}

TEST(DslTypesTest, ItHasNothingToBindToInAGlobal) {
    // §7.4.1.3: "`it` is a reference to the instance of a type in whose scope
    // it occurs".
    const std::vector<scena::Diagnostic> errors = resolve_errors("global a: int = it\n");
    EXPECT_TRUE(mentions(errors, "§7.4.1.3"));
}

// --- ordering and determinism ----------------------------------------------

TEST(DslTypesTest, DeclarationOrderDoesNotMatter) {
    // §7.3.15, for types, fields and units alike.
    const Program program = resolve_ok("struct user:\n"
                                       "    v: speed = 10.0kph\n"
                                       "unit kph of speed is SI(m: 1, s: -1, factor: 0.2777)\n"
                                       "type speed is SI(m: 1, s: -1)\n");
    const TypeInfo* user = program.find("::user");
    ASSERT_NE(user, nullptr);
    EXPECT_EQ(user->fields.at("v").type, program.types_by_name.at("::speed"));
}

TEST(DslTypesTest, ResolutionIsDeterministicAcrossRepeats) {
    // Load time is inside the bit-identity contract: the same sources must
    // produce the same table, in the same order, every run.
    constexpr std::string_view kSource = "namespace demo\n"
                                         "enum side: [left, right]\n"
                                         "actor vehicle:\n"
                                         "    s: side\n"
                                         "actor car inherits vehicle(s == left)\n"
                                         "scenario vehicle.drive:\n"
                                         "    speed: int\n";
    const Program first = resolve_ok(kSource);
    const Program second = resolve_ok(kSource);
    ASSERT_EQ(first.types.size(), second.types.size());
    for (std::size_t i = 0; i < first.types.size(); ++i) {
        EXPECT_EQ(first.types[i].name, second.types[i].name);
        EXPECT_EQ(first.types[i].field_order, second.types[i].field_order);
        EXPECT_EQ(first.types[i].base, second.types[i].base);
    }
    ASSERT_EQ(first.types_by_name.size(), second.types_by_name.size());
    EXPECT_TRUE(std::equal(first.types_by_name.begin(), first.types_by_name.end(),
                           second.types_by_name.begin()));
}

TEST(DslTypesTest, ResolutionReportsManyProblemsInOneRun) {
    const std::vector<scena::Diagnostic> errors = resolve_errors("struct a:\n"
                                                                 "    f: nowhere\n"
                                                                 "struct b:\n"
                                                                 "    g: alsonowhere\n"
                                                                 "struct c inherits missing\n");
    EXPECT_GE(errors.size(), 3U);
    for (const scena::Diagnostic& diagnostic : errors) {
        // The DSL standard defines no rule ids; diagnostics cite sections.
        EXPECT_TRUE(diagnostic.rule_id.empty());
        EXPECT_GT(diagnostic.location.line, 0);
    }
}

} // namespace
