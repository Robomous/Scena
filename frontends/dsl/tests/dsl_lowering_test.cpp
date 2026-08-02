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
// Lowering a checked DSL program to the Scenario IR (p8-s1, #44, ADR-0030).
//
// The scenarios here are hand-authored from the specification's own shapes.
// What each test pins is the *mapping* — which DSL construct denotes which IR
// construct — not runtime behaviour, which is the same runtime the XML frontend
// already feeds.
//

#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include <gtest/gtest.h>

#include "scena/diagnostic.h"
#include "scena/dsl/load.h"
#include "scena/dsl/lower.h"
#include "scena/dsl/types.h"
#include "scena/ir/entity.h"
#include "scena/ir/scenario.h"
#include "scena/status.h"

namespace {

using scena::DiagnosticSink;
using scena::Severity;
using scena::Status;
using scena::dsl::LoadOptions;
using scena::dsl::LoadResult;
using scena::dsl::LowerOptions;
using scena::dsl::Program;

/// A checked program plus what lowering made of it, kept together because the
/// LoadResult owns the ASTs the Program points into.
struct Lowered {
    LoadResult loaded;
    Program program;
    DiagnosticSink check_sink;
    DiagnosticSink sink;
    scena::ir::Scenario scenario;
    Status check_status = Status::Ok;
    Status status = Status::Ok;
};

/// Checks `source`, then lowers it. Both statuses are recorded rather than
/// asserted, so a test can pin either half.
void run(std::string_view source, Lowered& out, const std::string& entry_point = {}) {
    out.check_status = scena::dsl::check_source(source, "<test>", LoadOptions{}, out.loaded,
                                                out.program, out.check_sink);
    if (out.check_status != Status::Ok) {
        return;
    }
    LowerOptions options;
    options.entry_point = entry_point;
    out.status = scena::dsl::lower(out.program, out.loaded, options, out.scenario, out.sink);
}

/// The header every scenario below shares: the standard library, and a
/// namespace that uses it.
constexpr std::string_view kPrelude = "import osc.standard.all\n"
                                      "namespace demo use std, stdtypes\n";

[[nodiscard]] const scena::ir::Entity* entity(const scena::ir::Scenario& scenario,
                                              std::string_view id) {
    for (const scena::ir::Entity& candidate : scenario.entities) {
        if (candidate.id == id) {
            return &candidate;
        }
    }
    return nullptr;
}

std::string first_message(const DiagnosticSink& sink) {
    return sink.diagnostics().empty() ? std::string() : sink.diagnostics().front().message;
}

// --- entry point (§7.7.2) ---------------------------------------------------

TEST(DslLoweringTest, TheOnlyScenarioIsTheEntryPointWithoutBeingNamed) {
    // §7.7.2 leaves entry-point selection to the implementation. One scenario
    // in the file is the common case, and naming it again would add nothing.
    Lowered result;
    run(std::string(kPrelude).append("scenario overtake:\n    ego: vehicle\n"), result);
    ASSERT_EQ(result.check_status, Status::Ok) << first_message(result.check_sink);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    EXPECT_EQ(result.scenario.name, "overtake");
    EXPECT_EQ(result.scenario.entities.size(), 1U);
}

TEST(DslLoweringTest, MoreThanOneScenarioMustBeChosenBetween) {
    // Picking one silently would make the run depend on declaration order.
    Lowered result;
    run(std::string(kPrelude).append("actor top\n"
                                     "scenario top.first:\n    ego: vehicle\n"
                                     "scenario top.second:\n    ego: vehicle\n"),
        result);
    ASSERT_EQ(result.check_status, Status::Ok) << first_message(result.check_sink);
    EXPECT_EQ(result.status, Status::SemanticError);
    EXPECT_NE(first_message(result.sink).find("§7.7.2"), std::string::npos);
    // The message lists what it could have run, so the fix is copy-pasteable.
    EXPECT_NE(first_message(result.sink).find("demo::top.first"), std::string::npos);
    EXPECT_NE(first_message(result.sink).find("demo::top.second"), std::string::npos);
}

TEST(DslLoweringTest, AnEntryPointIsNamedQualifiedOrAsWritten) {
    const std::string source = std::string(kPrelude).append("actor top\n"
                                                            "scenario top.first:\n    a: vehicle\n"
                                                            "scenario top.second:\n"
                                                            "    b: vehicle\n"
                                                            "    c: vehicle\n");
    Lowered qualified;
    run(source, qualified, "demo::top.second");
    ASSERT_EQ(qualified.status, Status::Ok) << first_message(qualified.sink);
    EXPECT_EQ(qualified.scenario.entities.size(), 2U);

    // A file with one namespace makes the prefix pure ceremony, so the name as
    // written is accepted too.
    Lowered written;
    run(source, written, "top.second");
    ASSERT_EQ(written.status, Status::Ok) << first_message(written.sink);
    EXPECT_EQ(written.scenario.entities.size(), 2U);
}

TEST(DslLoweringTest, AnEntryPointThatIsNotThereIsReported) {
    Lowered result;
    run(std::string(kPrelude).append("scenario overtake:\n    ego: vehicle\n"), result, "nowhere");
    EXPECT_EQ(result.status, Status::SemanticError);
    EXPECT_NE(first_message(result.sink).find("'nowhere'"), std::string::npos);
    EXPECT_NE(first_message(result.sink).find("demo::overtake"), std::string::npos);
}

TEST(DslLoweringTest, TheEntryPointsAreListedInDeclarationOrder) {
    // Declaration order, not name order: this is what the file *offers*, and a
    // reader matches it against the file in front of them.
    Lowered result;
    run(std::string(kPrelude).append("actor top\n"
                                     "scenario top.zulu:\n    a: vehicle\n"
                                     "scenario top.alpha:\n    b: vehicle\n"),
        result, "top.zulu");
    ASSERT_EQ(result.check_status, Status::Ok) << first_message(result.check_sink);
    const std::vector<std::string> names = scena::dsl::entry_points(result.program, result.loaded);
    ASSERT_EQ(names.size(), 2U);
    EXPECT_EQ(names[0], "demo::top.zulu");
    EXPECT_EQ(names[1], "demo::top.alpha");
}

// --- actors become entities (§8.7 → p2-s1 taxonomy) -------------------------

TEST(DslLoweringTest, EveryPhysicalObjectFieldBecomesAnEntity) {
    Lowered result;
    run(std::string(kPrelude).append("scenario mixed:\n"
                                     "    ego: vehicle\n"
                                     "    walker: person\n"
                                     "    cone: stationary_object\n"
                                     "    lap_count: int\n"),
        result);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    // Three participants, in declaration order; `lap_count` is not one.
    ASSERT_EQ(result.scenario.entities.size(), 3U);
    EXPECT_EQ(result.scenario.entities[0].id, "ego");
    EXPECT_EQ(result.scenario.entities[1].id, "walker");
    EXPECT_EQ(result.scenario.entities[2].id, "cone");
    EXPECT_EQ(scena::ir::object_type_of(result.scenario.entities[0]),
              scena::ir::ObjectType::Vehicle);
    EXPECT_EQ(scena::ir::object_type_of(result.scenario.entities[1]),
              scena::ir::ObjectType::Pedestrian);
    EXPECT_EQ(scena::ir::object_type_of(result.scenario.entities[2]),
              scena::ir::ObjectType::MiscObject);
}

TEST(DslLoweringTest, AParticipantWithNoTaxonomyCounterpartStaysUnclassified) {
    // §8.7.10's `animal` is a sibling actor, not a pedestrian category. An
    // entity with an identity and a control mode is all the runtime needs; a
    // wrong classification would be worse than none.
    Lowered result;
    run(std::string(kPrelude).append("scenario safari:\n    deer: animal\n"), result);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    ASSERT_EQ(result.scenario.entities.size(), 1U);
    EXPECT_FALSE(result.scenario.entities.front().object.has_value());
    EXPECT_EQ(result.scenario.entities.front().control_mode,
              scena::ir::ControlMode::EngineControlled);
}

TEST(DslLoweringTest, ADerivedActorIsStillTheParticipantItInheritsFrom) {
    Lowered result;
    run(std::string(kPrelude).append("actor car inherits vehicle(vehicle_category == car)\n"
                                     "scenario drive:\n    ego: car\n"),
        result);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    ASSERT_EQ(result.scenario.entities.size(), 1U);
    EXPECT_EQ(scena::ir::object_type_of(result.scenario.entities.front()),
              scena::ir::ObjectType::Vehicle);
}

// --- concrete values (§7.3.11) ----------------------------------------------

TEST(DslLoweringTest, AnEqualityKeepFixesTheGeometry) {
    Lowered result;
    run(std::string(kPrelude).append("scenario overtake:\n"
                                     "    ego: vehicle\n"
                                     "    keep(ego.bounding_box.length == 4.5m)\n"
                                     "    keep(ego.bounding_box.width == 2.0m)\n"
                                     "    keep(ego.bounding_box.height == 1.5m)\n"),
        result);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    const scena::ir::Entity* ego = entity(result.scenario, "ego");
    ASSERT_NE(ego, nullptr);
    const std::optional<scena::ir::BoundingBox> box = scena::ir::bounding_box_of(*ego);
    ASSERT_TRUE(box.has_value());
    EXPECT_DOUBLE_EQ(box->length, 4.5);
    EXPECT_DOUBLE_EQ(box->width, 2.0);
    EXPECT_DOUBLE_EQ(box->height, 1.5);
}

TEST(DslLoweringTest, AKeepIsReadFromEitherSide) {
    // `keep(4.5m == x)` says exactly what `keep(x == 4.5m)` says, and the
    // standard prints both orders.
    Lowered result;
    run(std::string(kPrelude).append("scenario overtake:\n"
                                     "    ego: vehicle\n"
                                     "    keep(4.5m == ego.bounding_box.length)\n"),
        result);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    const scena::ir::Entity* ego = entity(result.scenario, "ego");
    ASSERT_NE(ego, nullptr);
    EXPECT_DOUBLE_EQ(scena::ir::bounding_box_of(*ego)->length, 4.5);
}

TEST(DslLoweringTest, APhysicalValueArrivesInItsBaseUnit) {
    // §7.3.4 folding happens during checking, so lowering never converts — and
    // therefore never re-applies the standard's printed factors (ADR-0029).
    // 450cm is 4.5m exactly; the kph factor is the case that would show a
    // second conversion, and it is deliberately not one.
    Lowered result;
    run(std::string(kPrelude).append("scenario overtake:\n"
                                     "    ego: vehicle\n"
                                     "    keep(ego.bounding_box.length == 450cm)\n"),
        result);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    EXPECT_DOUBLE_EQ(scena::ir::bounding_box_of(*entity(result.scenario, "ego"))->length, 4.5);
}

TEST(DslLoweringTest, AKeepFixesTheVehicleCategory) {
    Lowered result;
    run(std::string(kPrelude).append("scenario ride:\n"
                                     "    b: vehicle\n"
                                     "    keep(b.vehicle_category == vehicle_category!bus)\n"),
        result);
    ASSERT_EQ(result.check_status, Status::Ok) << first_message(result.check_sink);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    const scena::ir::Entity* bus = entity(result.scenario, "b");
    ASSERT_NE(bus, nullptr);
    ASSERT_TRUE(bus->object.has_value());
    EXPECT_EQ(std::get<scena::ir::Vehicle>(*bus->object).category, scena::ir::VehicleCategory::Bus);
}

TEST(DslLoweringTest, ConditionalInheritanceFixesTheCategoryToo) {
    // §7.3.8.2's `inherits vehicle(vehicle_category == heavy_truck)` fixes the
    // category on the type rather than in the scenario — the spelling §8.7's
    // own examples use.
    Lowered result;
    run(std::string(kPrelude).append(
            "actor lorry inherits vehicle(vehicle_category == heavy_truck)\n"
            "scenario haul:\n    t: lorry\n"),
        result);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    const scena::ir::Entity* lorry = entity(result.scenario, "t");
    ASSERT_NE(lorry, nullptr);
    ASSERT_TRUE(lorry->object.has_value());
    EXPECT_EQ(std::get<scena::ir::Vehicle>(*lorry->object).category,
              scena::ir::VehicleCategory::HeavyTruck);
}

TEST(DslLoweringTest, AnUnfixedCategoryKeepsTheIrDefault) {
    // Nothing is guessed at: a category the scenario does not fix is the IR's
    // own default, not an invention of the lowering.
    Lowered result;
    run(std::string(kPrelude).append("scenario drive:\n    ego: vehicle\n"), result);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    ASSERT_TRUE(entity(result.scenario, "ego")->object.has_value());
    EXPECT_EQ(std::get<scena::ir::Vehicle>(*entity(result.scenario, "ego")->object).category,
              scena::ir::VehicleCategory::Car);
}

TEST(DslLoweringTest, PerformanceLimitsHaveNoDslSourceAndStayUnconstrained) {
    // §8.7 declares no performance limits at all — the domain model has no
    // counterpart to XML's Performance. The IR's zeros are the faithful
    // lowering: the runtime reads a non-positive limit as "unconstrained".
    Lowered result;
    run(std::string(kPrelude).append("scenario drive:\n    ego: vehicle\n"), result);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    const scena::ir::Performance* performance =
        scena::ir::performance_of(*entity(result.scenario, "ego"));
    ASSERT_NE(performance, nullptr);
    EXPECT_DOUBLE_EQ(performance->max_speed, 0.0);
}

TEST(DslLoweringTest, AScenarioWithNoParticipantsSaysSo) {
    Lowered result;
    run(std::string(kPrelude).append("scenario empty:\n    laps: int\n"), result);
    // A warning, not an error: the file is well-formed, it simply has nothing
    // to run.
    EXPECT_EQ(result.status, Status::Ok);
    ASSERT_EQ(result.sink.diagnostics().size(), 1U);
    EXPECT_EQ(result.sink.diagnostics().front().severity, Severity::Warning);
    EXPECT_TRUE(result.scenario.entities.empty());
}

// --- determinism ------------------------------------------------------------

TEST(DslLoweringTest, LoweringTheSameSourceTwiceGivesTheSameIr) {
    // Load time is inside the bit-identity contract, and lowering is load time.
    const std::string source =
        std::string(kPrelude).append("scenario overtake:\n"
                                     "    ego: vehicle\n"
                                     "    walker: person\n"
                                     "    keep(ego.bounding_box.length == 4.5m)\n"
                                     "    keep(ego.vehicle_category == vehicle_category!van)\n");
    Lowered first;
    Lowered second;
    run(source, first);
    run(source, second);
    ASSERT_EQ(first.status, Status::Ok) << first_message(first.sink);
    ASSERT_EQ(second.status, Status::Ok);
    ASSERT_EQ(first.scenario.entities.size(), second.scenario.entities.size());
    for (std::size_t index = 0; index < first.scenario.entities.size(); ++index) {
        const scena::ir::Entity& left = first.scenario.entities[index];
        const scena::ir::Entity& right = second.scenario.entities[index];
        EXPECT_EQ(left.id, right.id);
        EXPECT_EQ(scena::ir::object_type_of(left), scena::ir::object_type_of(right));
        const std::optional<scena::ir::BoundingBox> left_box = scena::ir::bounding_box_of(left);
        const std::optional<scena::ir::BoundingBox> right_box = scena::ir::bounding_box_of(right);
        ASSERT_EQ(left_box.has_value(), right_box.has_value());
        if (left_box.has_value()) {
            // Bit-identical, not merely close: this is the contract.
            EXPECT_EQ(left_box->length, right_box->length);
            EXPECT_EQ(left_box->width, right_box->width);
        }
    }
}

} // namespace
