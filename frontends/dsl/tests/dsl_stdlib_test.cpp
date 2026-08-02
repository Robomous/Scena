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

/// The whole bundled library, types and domain, reached the way a scenario
/// reaches it (§7.7.5.2.1).
void check_full_library(Library& library) {
    library.status =
        scena::dsl::check_source("import osc.standard.all\n", "<library>", LoadOptions{},
                                 library.loaded, library.program, library.sink);
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

// --- §8.7 the domain sub-module ---------------------------------------------

TEST(DslStdlibTest, TheDomainSubModuleChecksClean) {
    // The other half of p7-s5's exit criterion, reached the way a scenario
    // reaches the library: `import osc.standard.all` (§7.7.5.2.1).
    Library library;
    check_full_library(library);
    for (const scena::Diagnostic& diagnostic : library.sink.diagnostics()) {
        ADD_FAILURE() << diagnostic.location.file << ":" << diagnostic.location.line << ": "
                      << diagnostic.message;
    }
    EXPECT_EQ(library.status, Status::Ok);
    EXPECT_EQ(scena::dsl::standard_module_namespace(scena::dsl::kStandardDomainModule), "std");
}

TEST(DslStdlibTest, TheCompleteLibraryReferenceBringsBothSubModules) {
    // §7.7.5.2.1: `osc.standard.all` imports "all the definitions of the
    // standard library ... in their respective namespaces".
    Library library;
    check_full_library(library);
    EXPECT_NE(library.program.find("stdtypes::speed"), nullptr);
    EXPECT_NE(library.program.find("std::vehicle"), nullptr);
}

TEST(DslStdlibTest, TheActorHierarchyMatchesTheDomainModel) {
    // §8.7.2–§8.7.10's Parents rows, as an inheritance chain.
    Library library;
    check_full_library(library);
    struct Case {
        const char* derived;
        const char* base;
    };
    for (const Case& expected :
         {Case{"std::physical_object", "std::osc_actor"},
          Case{"std::stationary_object", "std::physical_object"},
          Case{"std::movable_object", "std::physical_object"},
          Case{"std::traffic_participant", "std::movable_object"},
          Case{"std::vehicle", "std::traffic_participant"}, Case{"std::trailer", "std::vehicle"},
          Case{"std::person", "std::traffic_participant"},
          Case{"std::animal", "std::traffic_participant"},
          // transitively, every participant is an actor
          Case{"std::vehicle", "std::osc_actor"}}) {
        const auto derived = library.program.types_by_name.find(expected.derived);
        const auto base = library.program.types_by_name.find(expected.base);
        ASSERT_NE(derived, library.program.types_by_name.end()) << expected.derived;
        ASSERT_NE(base, library.program.types_by_name.end()) << expected.base;
        EXPECT_EQ(library.program.types[derived->second].kind, TypeKind::Actor) << expected.derived;
        EXPECT_TRUE(library.program.is_derived_from(derived->second, base->second))
            << expected.derived << " inherits " << expected.base;
    }
}

TEST(DslStdlibTest, ActorParametersAndStateVariablesAreDeclaredAndInherited) {
    // §8.7.3/§8.7.5/§8.7.6/§8.7.7: a table row is a field, whether the standard
    // calls it a parameter or a state variable — the language draws no
    // declaration-level distinction.
    Library library;
    check_full_library(library);
    const auto vehicle = library.program.types_by_name.find("std::vehicle");
    ASSERT_NE(vehicle, library.program.types_by_name.end());
    for (const char* field :
         {"vehicle_category", "axles", "rear_overhang", "trailer_receiver",
          // inherited from traffic_participant
          "intended_infrastructure", "role",
          // inherited from movable_object
          "velocity", "acceleration", "speed",
          // inherited from physical_object
          "bounding_box", "color", "geometry_reference", "center_of_gravity", "pose"}) {
        EXPECT_NE(library.program.find_field(vehicle->second, field), nullptr) << field;
    }
}

TEST(DslStdlibTest, TheDomainStructsAreDeclaredWithTheirFields) {
    Library library;
    check_full_library(library);
    struct Case {
        const char* name;
        std::vector<const char*> fields;
    };
    for (const Case& expected :
         {Case{"std::bounding_box", {"center", "length", "width", "height"}},
          Case{"std::axle",
               {"max_steering", "wheel_diameter", "track_width", "position_x", "position_z",
                "number_of_wheels"}},
          Case{"std::hitch_receiver",
               {"hitch_type", "position_x", "position_z", "max_rotation", "max_tilt", "is_towing"}},
          Case{"std::hitch_coupler", {"hitch_type", "position_x", "position_z", "is_towed"}}}) {
        const auto id = library.program.types_by_name.find(expected.name);
        ASSERT_NE(id, library.program.types_by_name.end()) << expected.name;
        EXPECT_EQ(library.program.types[id->second].kind, TypeKind::Struct) << expected.name;
        for (const char* field : expected.fields) {
            EXPECT_NE(library.program.find_field(id->second, field), nullptr)
                << expected.name << "." << field;
        }
    }
}

TEST(DslStdlibTest, TheDomainEnumsCarryTheirValues) {
    Library library;
    check_full_library(library);
    struct Case {
        const char* name;
        std::size_t values;
        const char* sample;
    };
    for (const Case& expected :
         {Case{"std::color", 20, "maroon"}, Case{"std::vehicle_category", 21, "heavy_truck"},
          Case{"std::trailer_category", 3, "full_trailer"},
          Case{"std::hitch_type", 5, "fifth_wheel"},
          Case{"std::intended_infrastructure", 8, "biking"},
          Case{"std::traffic_participant_role", 18, "ambulance"},
          Case{"std::distance_direction", 4, "euclidean"},
          Case{"std::road_distance_direction", 2, "lateral"},
          Case{"std::distance_mode", 2, "bounding_boxes"},
          Case{"std::on_route_type", 4, "on_lane_section"},
          Case{"std::route_distance_enum", 2, "from_end"}}) {
        const TypeInfo* type = library.program.find(expected.name);
        ASSERT_NE(type, nullptr) << expected.name;
        EXPECT_EQ(type->kind, TypeKind::Enum) << expected.name;
        EXPECT_EQ(type->enum_members.size(), expected.values) << expected.name;
        bool found = false;
        for (const auto& member : type->enum_members) {
            found = found || member.name == expected.sample;
        }
        EXPECT_TRUE(found) << expected.name << "!" << expected.sample;
    }
}

TEST(DslStdlibTest, TheBackwardCompatibilitySpellingsShareTheirValue) {
    // §8.7.16 and §8.7.20 keep the earlier releases' names "equal to" their
    // replacements, which §7.2.2.2.2's `= other_member` form expresses exactly.
    Library library;
    check_full_library(library);
    struct Case {
        const char* type;
        const char* legacy;
        const char* current;
    };
    for (const Case& expected :
         {Case{"std::vehicle_category", "truck", "heavy_truck"},
          Case{"std::vehicle_category", "vru_vehicle", "micro_mobility_device"},
          Case{"std::traffic_participant_role", "fire", "fire_brigade"},
          Case{"std::traffic_participant_role", "road_assistance", "roadside_assistance"},
          Case{"std::traffic_participant_role", "road_construction", "construction"}}) {
        const TypeInfo* type = library.program.find(expected.type);
        ASSERT_NE(type, nullptr) << expected.type;
        bool matched = false;
        for (const auto& legacy : type->enum_members) {
            if (legacy.name != expected.legacy) {
                continue;
            }
            for (const auto& current : type->enum_members) {
                if (current.name == expected.current) {
                    EXPECT_EQ(legacy.value, current.value)
                        << expected.type << ": " << expected.legacy << " vs " << expected.current;
                    matched = true;
                }
            }
        }
        EXPECT_TRUE(matched) << expected.type << "!" << expected.legacy;
    }
}

TEST(DslStdlibTest, TheMeasurementMethodsAreDeclaredWithTheirSignatures) {
    // §8.7.6.1 and §8.7.7: the prototypes the standard prints as `extend` blocks.
    Library library;
    check_full_library(library);
    struct Case {
        const char* type;
        const char* method;
        std::size_t parameters;
        const char* returns;
    };
    for (const Case& expected :
         {Case{"std::traffic_participant", "time_to_collision", 1, "stdtypes::time"},
          Case{"std::traffic_participant", "time_gap", 2, "stdtypes::time"},
          Case{"std::traffic_participant", "space_gap", 2, "stdtypes::length"},
          Case{"std::vehicle", "time_headway", 1, "stdtypes::time"},
          Case{"std::vehicle", "space_headway", 1, "stdtypes::length"}}) {
        const auto id = library.program.types_by_name.find(expected.type);
        ASSERT_NE(id, library.program.types_by_name.end()) << expected.type;
        const scena::dsl::MethodInfo* method =
            library.program.find_method(id->second, expected.method);
        ASSERT_NE(method, nullptr) << expected.type << "." << expected.method;
        EXPECT_EQ(method->parameters.size(), expected.parameters) << expected.method;
        ASSERT_NE(method->return_type, scena::dsl::kInvalidType) << expected.method;
        EXPECT_EQ(library.program.types[method->return_type].name, expected.returns)
            << expected.method;
    }
    // Inherited: a vehicle is a traffic_participant, so it measures gaps too.
    const auto vehicle = library.program.types_by_name.find("std::vehicle");
    ASSERT_NE(vehicle, library.program.types_by_name.end());
    EXPECT_NE(library.program.find_method(vehicle->second, "time_to_collision"), nullptr);
}

TEST(DslStdlibTest, TheActorModifiersAreAssociatedWithTheirActor) {
    // §8.7.4.1.1 and §8.7.7.1.1 write them `stationary_object.location()` and
    // `vehicle.tow_trailer()` — the prefixed form of §7.2.2.2.9, not `of`,
    // because §7.3.12.2's `of` names a scenario or an action.
    Library library;
    check_full_library(library);
    struct Case {
        const char* name;
        const char* actor;
        const char* parameter;
    };
    for (const Case& expected :
         {Case{"std::stationary_object.location", "std::stationary_object", "pose"},
          Case{"std::vehicle.tow_trailer", "std::vehicle", "trailer"}}) {
        const auto id = library.program.types_by_name.find(expected.name);
        ASSERT_NE(id, library.program.types_by_name.end()) << expected.name;
        const TypeInfo& modifier = library.program.types[id->second];
        EXPECT_EQ(modifier.kind, TypeKind::Modifier) << expected.name;
        ASSERT_NE(modifier.actor_type, scena::dsl::kInvalidType) << expected.name;
        EXPECT_EQ(library.program.types[modifier.actor_type].name, expected.actor) << expected.name;
        EXPECT_NE(library.program.find_field(id->second, expected.parameter), nullptr)
            << expected.name << "." << expected.parameter;
    }
}

TEST(DslStdlibTest, AScenarioCanUseTheDomainModel) {
    // The end-to-end shape a scenario author writes: import, use, declare.
    DiagnosticSink sink;
    LoadResult loaded;
    Program program;
    ASSERT_EQ(scena::dsl::check_source("import osc.standard.all\n"
                                       "namespace demo use std, stdtypes\n"
                                       "scenario cut_in:\n"
                                       "    ego: vehicle\n"
                                       "    other: vehicle\n"
                                       "    keep(ego.vehicle_category == car)\n"
                                       "    keep(other.bounding_box.width == 1.95m)\n",
                                       "<test>", LoadOptions{}, loaded, program, sink),
              Status::Ok)
        << (sink.diagnostics().empty() ? std::string() : sink.diagnostics().front().message);
    EXPECT_FALSE(sink.has_errors());
}

TEST(DslStdlibTest, TheDomainModelNeedsAnImportOrAQualifiedName) {
    // Only `stdtypes` is built in; `std` arrives with an import (ADR-0029).
    DiagnosticSink sink;
    LoadResult loaded;
    Program program;
    EXPECT_EQ(scena::dsl::check_source("struct s:\n    v: vehicle\n", "<test>", LoadOptions{},
                                       loaded, program, sink),
              Status::ValidationError);
    EXPECT_TRUE(sink.has_errors());
}

TEST(DslStdlibTest, CheckingTheLibraryIsDeterministic) {
    // Load time is inside the determinism contract: the same sources must give
    // the same program and the same diagnostics, in the same order.
    Library first;
    Library second;
    check_full_library(first);
    check_full_library(second);
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
