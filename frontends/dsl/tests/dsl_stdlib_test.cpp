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
// The bundled standard library (p7-s5, #43). The pillar's gate is that the
// library checks clean, so the first test here is the whole point: it loads
// with zero diagnostics of any severity. The rest pin the §8 surface, so that
// a later edit cannot quietly drop a type, a unit or a conversion factor.
//
// The library is authored from the normative §8 text (ADR-0002); §8.16 says the
// files shipped with the standard are non-normative and the document is the
// normative part.
//

#include <string>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

#include "scena/diagnostic.h"
#include "scena/dsl/load.h"
#include "scena/dsl/stdlib.h"
#include "scena/dsl/types.h"
#include "scena/status.h"

namespace {

using scena::DiagnosticSink;
using scena::Status;
using scena::dsl::kStandardTypesModule;
using scena::dsl::LoadOptions;
using scena::dsl::LoadResult;
using scena::dsl::Program;
using scena::dsl::to_base_units;
using scena::dsl::TypeInfo;
using scena::dsl::TypeKind;

/// The bundled library, checked on its own.
struct Library {
    LoadResult loaded;
    Program program;
    DiagnosticSink sink;
    Status status = Status::Ok;
};

void check_library(Library& library) {
    library.status = scena::dsl::check_source("", "<library>", LoadOptions{}, library.loaded,
                                              library.program, library.sink);
}

// --- the gate ---------------------------------------------------------------

TEST(DslStdlibTest, TheBundledLibraryChecksClean) {
    // p7-s5's exit criterion. Not "no errors" — no diagnostics at all: a
    // warning in the library would show up in every user's output.
    Library library;
    check_library(library);
    for (const scena::Diagnostic& diagnostic : library.sink.diagnostics()) {
        ADD_FAILURE() << diagnostic.location.file << ":" << diagnostic.location.line << ": "
                      << diagnostic.message;
    }
    EXPECT_EQ(library.status, Status::Ok);
}

TEST(DslStdlibTest, TheLibraryIsDslSourceGoingThroughTheSamePasses) {
    // It is DSL text, not a hand-built table, so the lexer, the parser and the
    // resolver all see it — a bug in any of them shows up here first.
    EXPECT_FALSE(scena::dsl::standard_module_source(kStandardTypesModule).empty());
    EXPECT_EQ(scena::dsl::standard_module_namespace(kStandardTypesModule), "stdtypes");
}

// --- §8.14.1 scalar physical types ------------------------------------------

TEST(DslStdlibTest, EveryScalarPhysicalTypeIsDeclaredWithItsDimension) {
    Library library;
    check_library(library);
    struct Case {
        const char* name;
        const char* dimension;
    };
    for (const Case& expected :
         {Case{"stdtypes::length", "m"}, Case{"stdtypes::time", "s"},
          Case{"stdtypes::speed", "m*s^-1"}, Case{"stdtypes::acceleration", "m*s^-2"},
          Case{"stdtypes::jerk", "m*s^-3"}, Case{"stdtypes::angle", "rad"},
          Case{"stdtypes::angular_rate", "s^-1*rad"},
          Case{"stdtypes::angular_acceleration", "s^-2*rad"}, Case{"stdtypes::mass", "kg"},
          Case{"stdtypes::temperature", "K"}, Case{"stdtypes::pressure", "kg*m^-1*s^-2"},
          Case{"stdtypes::luminous_intensity", "cd"}, Case{"stdtypes::luminous_flux", "cd*rad^2"},
          Case{"stdtypes::illuminance", "m^-2*cd*rad^2"}, Case{"stdtypes::electrical_current", "A"},
          Case{"stdtypes::amount_of_substance", "mol"}}) {
        const TypeInfo* type = library.program.find(expected.name);
        ASSERT_NE(type, nullptr) << expected.name;
        EXPECT_EQ(type->kind, TypeKind::Physical) << expected.name;
        EXPECT_EQ(type->dimension.to_string(), expected.dimension) << expected.name;
    }
}

TEST(DslStdlibTest, UnitsCarryTheFactorsTheStandardPrints) {
    // §7.3.4: base_unit_value = unit_value * factor + offset. The factors are
    // the ones §8.14.1 prints, not rounder values of our own.
    Library library;
    check_library(library);
    struct Case {
        const char* unit;
        double value;
        double expected;
    };
    for (const Case& test_case :
         {Case{"m", 3.0, 3.0}, Case{"km", 1.5, 1500.0}, Case{"cm", 250.0, 2.5},
          Case{"feet", 10.0, 3.048}, Case{"mile", 1.0, 1609.344}, Case{"s", 2.0, 2.0},
          Case{"min", 2.0, 120.0}, Case{"hour", 0.5, 1800.0}, Case{"h", 0.5, 1800.0},
          Case{"celsius", 0.0, 273.15}, Case{"gram", 500.0, 0.5}, Case{"ton", 2.0, 2000.0},
          Case{"atm", 1.0, 101325.0}, Case{"mps", 10.0, 10.0}}) {
        const auto unit = library.program.units.find(test_case.unit);
        ASSERT_NE(unit, library.program.units.end()) << test_case.unit;
        EXPECT_NEAR(to_base_units(test_case.value, unit->second), test_case.expected, 1e-9)
            << test_case.unit;
    }
}

TEST(DslStdlibTest, TheRoundedFactorsOfTheStandardAreCarriedVerbatim) {
    // §8.14.1.3 prints the kph factor as 0.277777778, a rounded decimal rather
    // than 1/3.6. The library carries what the standard prints, so 36kph is
    // 10.000000008 m/s and not exactly 10 — the observable consequence is that
    // `36kph == 10mps` is false. Pinned here so the rounding is a decision on
    // the record rather than a surprise in a scenario.
    Library library;
    check_library(library);
    const auto kph = library.program.units.find("kph");
    ASSERT_NE(kph, library.program.units.end());
    EXPECT_EQ(kph->second.factor, 0.277777778);
    EXPECT_NE(to_base_units(36.0, kph->second), 10.0);
    EXPECT_NEAR(to_base_units(36.0, kph->second), 10.0, 1e-6);
}

TEST(DslStdlibTest, TheSpokenAndAbbreviatedUnitNamesBothExist) {
    // §8.14.1 gives most units two spellings; a scenario may write either.
    Library library;
    check_library(library);
    for (const char* name : {"meter",
                             "m",
                             "kilometer",
                             "km",
                             "second",
                             "sec",
                             "s",
                             "minute",
                             "min",
                             "hour",
                             "h",
                             "meter_per_second",
                             "mps",
                             "kilometer_per_hour",
                             "kph",
                             "kmph",
                             "mile_per_hour",
                             "mph",
                             "miph",
                             "degree",
                             "deg",
                             "radian",
                             "rad",
                             "kilogram",
                             "kg",
                             "gram",
                             "pound",
                             "lb",
                             "kelvin",
                             "celsius",
                             "fahrenheit",
                             "pascal",
                             "Pa",
                             "candela",
                             "cd",
                             "lumen",
                             "lm",
                             "lux",
                             "lx",
                             "ampere",
                             "A",
                             "mole",
                             "mol"}) {
        EXPECT_NE(library.program.units.find(name), library.program.units.end()) << name;
    }
}

TEST(DslStdlibTest, EveryUnitPointsAtItsPhysicalType) {
    Library library;
    check_library(library);
    for (const auto& [name, unit] : library.program.units) {
        ASSERT_NE(unit.physical_type, scena::dsl::kInvalidType) << name;
        const TypeInfo* type = &library.program.types[unit.physical_type];
        EXPECT_EQ(type->kind, TypeKind::Physical) << name;
        // §7.3.4: a unit's dimension is the dimension of its physical type.
        EXPECT_EQ(unit.dimension.to_string(), type->dimension.to_string()) << name;
    }
}

// --- §8.14.2 compound structs -----------------------------------------------

TEST(DslStdlibTest, TheCompoundStructsAreDeclaredWithTheirFields) {
    Library library;
    check_library(library);
    struct Case {
        const char* name;
        std::vector<const char*> fields;
    };
    for (const Case& expected :
         {Case{"stdtypes::position_3d", {"x", "y", "z"}},
          Case{"stdtypes::geodetic_position_2d", {"latitude", "longitude"}},
          Case{"stdtypes::celestial_position_2d", {"azimuth", "elevation"}},
          Case{"stdtypes::orientation_3d", {"roll", "pitch", "yaw"}},
          Case{"stdtypes::pose_3d", {"position", "orientation"}},
          Case{"stdtypes::translational_velocity_3d", {"x", "y", "z"}},
          Case{"stdtypes::orientation_rate_3d", {"roll", "pitch", "yaw"}},
          Case{"stdtypes::velocity_6d", {"translational", "angular"}},
          Case{"stdtypes::translational_acceleration_3d", {"x", "y", "z"}},
          Case{"stdtypes::orientation_acceleration_3d", {"roll", "pitch", "yaw"}},
          Case{"stdtypes::acceleration_6d", {"translational", "angular"}}}) {
        const TypeInfo* type = library.program.find(expected.name);
        ASSERT_NE(type, nullptr) << expected.name;
        EXPECT_EQ(type->kind, TypeKind::Struct) << expected.name;
        for (const char* field : expected.fields) {
            EXPECT_NE(
                library.program.find_field(library.program.types_by_name.at(expected.name), field),
                nullptr)
                << expected.name << "." << field;
        }
    }
}

TEST(DslStdlibTest, ThePositionStructsInheritTheGenericPosition) {
    // §8.14.2.2–.4: all three are "Inherited from generic position class".
    Library library;
    check_library(library);
    const auto base = library.program.types_by_name.find("stdtypes::position");
    ASSERT_NE(base, library.program.types_by_name.end());
    for (const char* derived : {"stdtypes::position_3d", "stdtypes::geodetic_position_2d",
                                "stdtypes::celestial_position_2d"}) {
        const auto id = library.program.types_by_name.find(derived);
        ASSERT_NE(id, library.program.types_by_name.end()) << derived;
        EXPECT_TRUE(library.program.is_derived_from(id->second, base->second)) << derived;
    }
}

TEST(DslStdlibTest, TheNormMethodsReturnTheMatchingPhysicalType) {
    // §8.14.2.13: norm() is declared on the three translational compounds.
    Library library;
    check_library(library);
    struct Case {
        const char* type;
        const char* returns;
    };
    for (const Case& expected :
         {Case{"stdtypes::position_3d", "stdtypes::length"},
          Case{"stdtypes::translational_velocity_3d", "stdtypes::speed"},
          Case{"stdtypes::translational_acceleration_3d", "stdtypes::acceleration"}}) {
        const auto id = library.program.types_by_name.find(expected.type);
        ASSERT_NE(id, library.program.types_by_name.end()) << expected.type;
        const scena::dsl::MethodInfo* method = library.program.find_method(id->second, "norm");
        ASSERT_NE(method, nullptr) << expected.type;
        EXPECT_TRUE(method->parameters.empty()) << expected.type;
        ASSERT_NE(method->return_type, scena::dsl::kInvalidType) << expected.type;
        EXPECT_EQ(library.program.types[method->return_type].name, expected.returns)
            << expected.type;
    }
}

// --- §8.13 string methods ---------------------------------------------------

TEST(DslStdlibTest, TheStringMethodsAreDeclared) {
    Library library;
    check_library(library);
    const auto string_type = library.program.types_by_name.find("string");
    ASSERT_NE(string_type, library.program.types_by_name.end());
    for (const char* name :
         {"length", "contains", "substring", "split", "join", "replace", "trim"}) {
        EXPECT_NE(library.program.find_method(string_type->second, name), nullptr) << name;
    }
}

// --- the library through the user-facing path -------------------------------

TEST(DslStdlibTest, AScenarioCanUseTheLibraryUnqualified) {
    DiagnosticSink sink;
    LoadResult loaded;
    Program program;
    ASSERT_EQ(scena::dsl::check_source("struct waypoint:\n"
                                       "    at: position_3d\n"
                                       "    heading: angle = 90deg\n"
                                       "    cruise: speed = 100kph\n"
                                       "    weight: mass = 1500kg\n",
                                       "<test>", LoadOptions{}, loaded, program, sink),
              Status::Ok)
        << (sink.diagnostics().empty() ? std::string() : sink.diagnostics().front().message);
    EXPECT_FALSE(sink.has_errors());
}

TEST(DslStdlibTest, AWrongUnitAgainstALibraryTypeIsStillReported) {
    DiagnosticSink sink;
    LoadResult loaded;
    Program program;
    EXPECT_EQ(scena::dsl::check_source("struct s:\n    v: speed = 10m\n", "<test>", LoadOptions{},
                                       loaded, program, sink),
              Status::ValidationError);
    EXPECT_TRUE(sink.has_errors());
}

TEST(DslStdlibTest, CheckingTheLibraryIsDeterministic) {
    // Load time is inside the determinism contract: the same sources must give
    // the same program and the same diagnostics, in the same order.
    Library first;
    Library second;
    check_library(first);
    check_library(second);
    ASSERT_EQ(first.program.types_by_name.size(), second.program.types_by_name.size());
    EXPECT_TRUE(std::equal(first.program.types_by_name.begin(), first.program.types_by_name.end(),
                           second.program.types_by_name.begin()));
    ASSERT_EQ(first.program.units.size(), second.program.units.size());
    auto left = first.program.units.begin();
    auto right = second.program.units.begin();
    for (; left != first.program.units.end(); ++left, ++right) {
        EXPECT_EQ(left->first, right->first);
        EXPECT_EQ(left->second.factor, right->second.factor) << left->first;
        EXPECT_EQ(left->second.offset, right->second.offset) << left->first;
    }
}

} // namespace
