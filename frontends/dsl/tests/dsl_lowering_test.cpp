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
#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/entity.h"
#include "scena/ir/position.h"
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
    scena::dsl::LowerResult lowered;
    Status check_status = Status::Ok;
    Status status = Status::Ok;
};

/// Checks `source`, then lowers it. Both statuses are recorded rather than
/// asserted, so a test can pin either half.
void run(std::string_view source, Lowered& out, const std::string& entry_point = {},
         const std::string& alternative = {}) {
    out.check_status = scena::dsl::check_source(source, "<test>", LoadOptions{}, out.loaded,
                                                out.program, out.check_sink);
    if (out.check_status != Status::Ok) {
        return;
    }
    LowerOptions options;
    options.entry_point = entry_point;
    options.alternative = alternative;
    out.status = scena::dsl::lower(out.program, out.loaded, options, out.lowered, out.sink);
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
    EXPECT_EQ(result.lowered.scenario.name, "overtake");
    EXPECT_EQ(result.lowered.scenario.entities.size(), 1U);
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
    EXPECT_EQ(qualified.lowered.scenario.entities.size(), 2U);

    // A file with one namespace makes the prefix pure ceremony, so the name as
    // written is accepted too.
    Lowered written;
    run(source, written, "top.second");
    ASSERT_EQ(written.status, Status::Ok) << first_message(written.sink);
    EXPECT_EQ(written.lowered.scenario.entities.size(), 2U);
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
    ASSERT_EQ(result.lowered.scenario.entities.size(), 3U);
    EXPECT_EQ(result.lowered.scenario.entities[0].id, "ego");
    EXPECT_EQ(result.lowered.scenario.entities[1].id, "walker");
    EXPECT_EQ(result.lowered.scenario.entities[2].id, "cone");
    EXPECT_EQ(scena::ir::object_type_of(result.lowered.scenario.entities[0]),
              scena::ir::ObjectType::Vehicle);
    EXPECT_EQ(scena::ir::object_type_of(result.lowered.scenario.entities[1]),
              scena::ir::ObjectType::Pedestrian);
    EXPECT_EQ(scena::ir::object_type_of(result.lowered.scenario.entities[2]),
              scena::ir::ObjectType::MiscObject);
}

TEST(DslLoweringTest, AParticipantWithNoTaxonomyCounterpartStaysUnclassified) {
    // §8.7.10's `animal` is a sibling actor, not a pedestrian category. An
    // entity with an identity and a control mode is all the runtime needs; a
    // wrong classification would be worse than none.
    Lowered result;
    run(std::string(kPrelude).append("scenario safari:\n    deer: animal\n"), result);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    ASSERT_EQ(result.lowered.scenario.entities.size(), 1U);
    EXPECT_FALSE(result.lowered.scenario.entities.front().object.has_value());
    EXPECT_EQ(result.lowered.scenario.entities.front().control_mode,
              scena::ir::ControlMode::EngineControlled);
}

TEST(DslLoweringTest, ADerivedActorIsStillTheParticipantItInheritsFrom) {
    Lowered result;
    run(std::string(kPrelude).append("actor car inherits vehicle(vehicle_category == car)\n"
                                     "scenario drive:\n    ego: car\n"),
        result);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    ASSERT_EQ(result.lowered.scenario.entities.size(), 1U);
    EXPECT_EQ(scena::ir::object_type_of(result.lowered.scenario.entities.front()),
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
    const scena::ir::Entity* ego = entity(result.lowered.scenario, "ego");
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
    const scena::ir::Entity* ego = entity(result.lowered.scenario, "ego");
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
    EXPECT_DOUBLE_EQ(scena::ir::bounding_box_of(*entity(result.lowered.scenario, "ego"))->length,
                     4.5);
}

TEST(DslLoweringTest, AKeepFixesTheVehicleCategory) {
    Lowered result;
    run(std::string(kPrelude).append("scenario ride:\n"
                                     "    b: vehicle\n"
                                     "    keep(b.vehicle_category == vehicle_category!bus)\n"),
        result);
    ASSERT_EQ(result.check_status, Status::Ok) << first_message(result.check_sink);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    const scena::ir::Entity* bus = entity(result.lowered.scenario, "b");
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
    const scena::ir::Entity* lorry = entity(result.lowered.scenario, "t");
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
    ASSERT_TRUE(entity(result.lowered.scenario, "ego")->object.has_value());
    EXPECT_EQ(
        std::get<scena::ir::Vehicle>(*entity(result.lowered.scenario, "ego")->object).category,
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
        scena::ir::performance_of(*entity(result.lowered.scenario, "ego"));
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
    EXPECT_TRUE(result.lowered.scenario.entities.empty());
}

/// The events of the lowered storyboard, flattened in document order. One
/// ManeuverGroup per phase is the shape lowering builds, so this reads back as
/// "the phases, in order".
struct Phase {
    std::string name;
    std::string actor;
    bool has_start_trigger = false;
    const scena::ir::Event* event = nullptr;
};

std::vector<Phase> phases(const scena::ir::Scenario& scenario) {
    std::vector<Phase> out;
    for (const scena::ir::Story& story : scenario.storyboard.stories) {
        for (const scena::ir::Act& act : story.acts) {
            for (const scena::ir::ManeuverGroup& group : act.groups) {
                for (const scena::ir::Maneuver& maneuver : group.maneuvers) {
                    for (const scena::ir::Event& event : maneuver.events) {
                        out.push_back(Phase{group.name,
                                            group.actors.empty() ? std::string() : group.actors[0],
                                            event.start_trigger.has_value(), &event});
                    }
                }
            }
        }
    }
    return out;
}

/// The single action of a phase, as `T`, or nullptr.
template <typename T> const T* only_action(const Phase& phase) {
    if (phase.event == nullptr || phase.event->actions.size() != 1) {
        return nullptr;
    }
    return dynamic_cast<const T*>(phase.event->actions.front().get());
}

bool sink_says(const DiagnosticSink& sink, std::string_view fragment) {
    for (const scena::Diagnostic& diagnostic : sink.diagnostics()) {
        if (diagnostic.message.find(fragment) != std::string::npos) {
            return true;
        }
    }
    return false;
}

// --- Â§8.8 movement actions -------------------------------------------------

TEST(DslLoweringTest, AnAssignSpeedIsAStepSpeedChange) {
    // Â§8.8.2.6: the actor's speed *is* the value from that point on, which is
    // what a Step transition means (Â§7.4.1.2).
    Lowered result;
    run(std::string(kPrelude).append("scenario go:\n"
                                     "    ego: vehicle\n"
                                     "    do launch: ego.assign_speed(speed: 10mps)\n"),
        result);
    ASSERT_EQ(result.check_status, Status::Ok) << first_message(result.check_sink);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    const std::vector<Phase> lowered = phases(result.lowered.scenario);
    ASSERT_EQ(lowered.size(), 1U);
    EXPECT_EQ(lowered[0].name, "launch");
    EXPECT_EQ(lowered[0].actor, "ego");
    const scena::ir::SpeedAction* action = only_action<scena::ir::SpeedAction>(lowered[0]);
    ASSERT_NE(action, nullptr);
    EXPECT_EQ(action->entity_id(), "ego");
    EXPECT_EQ(action->target_speed(), 10.0);
    EXPECT_EQ(action->dynamics().shape, scena::ir::DynamicsShape::Step);
}

TEST(DslLoweringTest, AChangeSpeedTakesItsShapeFromTheRateProfile) {
    // Â§8.8.2.18's `smooth` is the profile whose gradient vanishes at both ends,
    // and `rate_peak` is the magnitude â together, a rate-dimensioned Cubic.
    Lowered result;
    run(std::string(kPrelude).append(
            "scenario go:\n"
            "    ego: vehicle\n"
            "    do ramp: ego.change_speed(target: 25mps, rate_profile: dynamic_profile!smooth, "
            "rate_peak: 2.0)\n"),
        result);
    ASSERT_EQ(result.check_status, Status::Ok) << first_message(result.check_sink);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    const std::vector<Phase> lowered = phases(result.lowered.scenario);
    ASSERT_EQ(lowered.size(), 1U);
    const scena::ir::SpeedAction* action = only_action<scena::ir::SpeedAction>(lowered[0]);
    ASSERT_NE(action, nullptr);
    EXPECT_EQ(action->target_speed(), 25.0);
    EXPECT_EQ(action->dynamics().shape, scena::ir::DynamicsShape::Cubic);
    EXPECT_EQ(action->dynamics().dimension, scena::ir::DynamicsDimension::Rate);
    EXPECT_EQ(action->dynamics().value, 2.0);
}

TEST(DslLoweringTest, AChangeSpeedWithoutAPeakRateIsInstantaneous) {
    // Neither `dynamic_profile` nor Â§8.7 supplies a duration, so with no peak
    // rate there is no number to ramp over. Step is the honest reading.
    Lowered result;
    run(std::string(kPrelude).append("scenario go:\n"
                                     "    ego: vehicle\n"
                                     "    do ramp: ego.change_speed(target: 25mps)\n"),
        result);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    const std::vector<Phase> lowered = phases(result.lowered.scenario);
    ASSERT_EQ(lowered.size(), 1U);
    const scena::ir::SpeedAction* action = only_action<scena::ir::SpeedAction>(lowered[0]);
    ASSERT_NE(action, nullptr);
    EXPECT_EQ(action->dynamics().shape, scena::ir::DynamicsShape::Step);
}

TEST(DslLoweringTest, RemainStationaryIsSpeedZero) {
    Lowered result;
    run(std::string(kPrelude).append("scenario go:\n"
                                     "    ego: vehicle\n"
                                     "    do hold: ego.remain_stationary()\n"),
        result);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    const std::vector<Phase> lowered = phases(result.lowered.scenario);
    ASSERT_EQ(lowered.size(), 1U);
    const scena::ir::SpeedAction* action = only_action<scena::ir::SpeedAction>(lowered[0]);
    ASSERT_NE(action, nullptr);
    EXPECT_EQ(action->target_speed(), 0.0);
}

TEST(DslLoweringTest, AnAssignPositionReadsItsCoordinatesFromTheConstraints) {
    // The DSL has no struct constructor (Â§7.2.2.6.7 declares list and range
    // constructors and nothing else), so a struct-valued argument names a
    // declaration and the `keep`s on it are where the numbers are.
    Lowered result;
    run(std::string(kPrelude).append("scenario go:\n"
                                     "    ego: vehicle\n"
                                     "    start: position_3d\n"
                                     "    keep(start.x == 12m)\n"
                                     "    keep(start.y == 300cm)\n"
                                     "    do place: ego.assign_position(position: start)\n"),
        result);
    ASSERT_EQ(result.check_status, Status::Ok) << first_message(result.check_sink);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    const std::vector<Phase> lowered = phases(result.lowered.scenario);
    ASSERT_EQ(lowered.size(), 1U);
    const scena::ir::TeleportAction* action = only_action<scena::ir::TeleportAction>(lowered[0]);
    ASSERT_NE(action, nullptr);
    const scena::ir::WorldPosition* world =
        std::get_if<scena::ir::WorldPosition>(&action->position());
    ASSERT_NE(world, nullptr);
    EXPECT_EQ(world->x, 12.0);
    // Folded to metres once, during checking; lowering never converts again.
    EXPECT_EQ(world->y, 3.0);
    EXPECT_EQ(world->z, 0.0);
}

TEST(DslLoweringTest, APositionNothingConstrainsIsReportedNotAssumed) {
    Lowered result;
    run(std::string(kPrelude).append("scenario go:\n"
                                     "    ego: vehicle\n"
                                     "    start: position_3d\n"
                                     "    do place: ego.assign_position(position: start)\n"),
        result);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    EXPECT_TRUE(sink_says(result.sink, "world origin"));
}

TEST(DslLoweringTest, AChangeLaneNeedsAnExplicitSide) {
    // Â§8.8.3.14's `inside`/`outside`/`same` need the road geometry to say which
    // way that is, and an unstated side would have to be chosen â a choice the
    // determinism contract does not let lowering make.
    Lowered result;
    run(std::string(kPrelude).append(
            "scenario go:\n"
            "    ego: vehicle\n"
            "    do swerve: ego.change_lane(side: lane_change_side!same)\n"),
        result);
    ASSERT_EQ(result.check_status, Status::Ok) << first_message(result.check_sink);
    EXPECT_EQ(result.status, Status::ValidationError);
    EXPECT_TRUE(sink_says(result.sink, "explicit 'side'"));
}

TEST(DslLoweringTest, AChangeLaneLowersToARelativeLaneTarget) {
    Lowered result;
    run(std::string(kPrelude).append(
            "scenario go:\n"
            "    ego: vehicle\n"
            "    do swerve: ego.change_lane(side: lane_change_side!left, num_of_lanes: 2, "
            "rate_peak: 3.0)\n"),
        result);
    ASSERT_EQ(result.check_status, Status::Ok) << first_message(result.check_sink);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    const std::vector<Phase> lowered = phases(result.lowered.scenario);
    ASSERT_EQ(lowered.size(), 1U);
    const scena::ir::LaneChangeAction* action =
        only_action<scena::ir::LaneChangeAction>(lowered[0]);
    ASSERT_NE(action, nullptr);
    ASSERT_TRUE(action->is_relative());
    // Positive counts go left (Â§7.4.1.4), and the reference defaults to the
    // actor itself (Â§8.8.3.3's `Default=it.actor`).
    EXPECT_EQ(action->relative_target()->value, 2);
    EXPECT_EQ(action->relative_target()->entity_ref, "ego");
}

TEST(DslLoweringTest, AGapActionKeepsTheDistanceToItsReference) {
    Lowered result;
    run(std::string(kPrelude).append(
            "scenario go:\n"
            "    ego: vehicle\n"
            "    lead: vehicle\n"
            "    do gap: ego.change_space_gap(target: 20m, direction: gap_direction!behind, "
            "reference: lead)\n"),
        result);
    ASSERT_EQ(result.check_status, Status::Ok) << first_message(result.check_sink);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    const std::vector<Phase> lowered = phases(result.lowered.scenario);
    ASSERT_EQ(lowered.size(), 1U);
    const scena::ir::LongitudinalDistanceAction* action =
        only_action<scena::ir::LongitudinalDistanceAction>(lowered[0]);
    ASSERT_NE(action, nullptr);
    EXPECT_EQ(action->entity_ref(), "lead");
    ASSERT_TRUE(action->distance().has_value());
    EXPECT_EQ(*action->distance(), 20.0);
    EXPECT_FALSE(action->time_gap().has_value());
    EXPECT_EQ(action->displacement(),
              scena::ir::LongitudinalDisplacement::TrailingReferencedEntity);
}

TEST(DslLoweringTest, AGapActionNeedsItsReferenceToBeAParticipant) {
    Lowered result;
    run(std::string(kPrelude).append(
            "scenario go:\n"
            "    ego: vehicle\n"
            "    do gap: ego.change_time_gap(target: 2s, reference: ego.bounding_box)\n"),
        result);
    ASSERT_EQ(result.check_status, Status::Ok) << first_message(result.check_sink);
    EXPECT_EQ(result.status, Status::ValidationError);
    EXPECT_TRUE(sink_says(result.sink, "participant of this scenario"));
}

TEST(DslLoweringTest, TheGenericDriveContributesNoActionOfItsOwn) {
    // Â§8.8.3.1's `drive` carries no target: it exists to be shaped by Â§8.9
    // modifiers (p8-s3), and on its own says "keep doing what you are doing".
    Lowered result;
    run(std::string(kPrelude).append("scenario go:\n"
                                     "    ego: vehicle\n"
                                     "    do cruise: ego.drive()\n"),
        result);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    EXPECT_TRUE(result.lowered.scenario.storyboard.stories.empty());
}

TEST(DslLoweringTest, AMovementActionWithNoCounterpartIsReportedNotDropped) {
    Lowered result;
    run(std::string(kPrelude).append("scenario go:\n"
                                     "    ego: vehicle\n"
                                     "    do hold: ego.keep_speed()\n"),
        result);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    EXPECT_TRUE(sink_says(result.sink, "no runtime counterpart"));
}

// --- the `do` directive -----------------------------------------------------

TEST(DslLoweringTest, SerialPhasesChainOnTheirPredecessor) {
    // Â§7.6.2.1.2: a member starts when its predecessor ends. In the runtime that
    // is a start trigger on the previous element reaching completeState.
    Lowered result;
    run(std::string(kPrelude).append("scenario go:\n"
                                     "    ego: vehicle\n"
                                     "    do serial:\n"
                                     "        launch: ego.assign_speed(speed: 10mps)\n"
                                     "        stop: ego.remain_stationary()\n"),
        result);
    ASSERT_EQ(result.check_status, Status::Ok) << first_message(result.check_sink);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    const std::vector<Phase> lowered = phases(result.lowered.scenario);
    ASSERT_EQ(lowered.size(), 2U);
    EXPECT_EQ(lowered[0].name, "launch");
    EXPECT_FALSE(lowered[0].has_start_trigger);
    EXPECT_EQ(lowered[1].name, "stop");
    EXPECT_TRUE(lowered[1].has_start_trigger);
}

TEST(DslLoweringTest, ParallelPhasesStartWithTheirAct) {
    Lowered result;
    run(std::string(kPrelude).append("scenario go:\n"
                                     "    ego: vehicle\n"
                                     "    other: vehicle\n"
                                     "    do parallel:\n"
                                     "        a: ego.assign_speed(speed: 10mps)\n"
                                     "        b: other.assign_speed(speed: 12mps)\n"),
        result);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    const std::vector<Phase> lowered = phases(result.lowered.scenario);
    ASSERT_EQ(lowered.size(), 2U);
    EXPECT_FALSE(lowered[0].has_start_trigger);
    EXPECT_FALSE(lowered[1].has_start_trigger);
    EXPECT_EQ(lowered[0].actor, "ego");
    EXPECT_EQ(lowered[1].actor, "other");
}

TEST(DslLoweringTest, AnUnlabelledPhaseGetsItsPositionAsAName) {
    // The runtime addresses storyboard elements by name path, so a phase needs
    // a name whether or not the author wrote a label.
    Lowered result;
    run(std::string(kPrelude).append("scenario go:\n"
                                     "    ego: vehicle\n"
                                     "    do serial:\n"
                                     "        ego.assign_speed(speed: 10mps)\n"
                                     "        ego.remain_stationary()\n"),
        result);
    ASSERT_EQ(result.check_status, Status::Ok) << first_message(result.check_sink);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    const std::vector<Phase> lowered = phases(result.lowered.scenario);
    ASSERT_EQ(lowered.size(), 2U);
    EXPECT_EQ(lowered[0].name, "phase_1");
    EXPECT_EQ(lowered[1].name, "phase_2");
}

// --- composition (§7.6.2.1) -------------------------------------------------

/// The start condition of a phase, rendered as text so a test can read it.
std::string trigger_text(const Phase& phase) {
    if (phase.event == nullptr || !phase.event->start_trigger.has_value()) {
        return "(with parent)";
    }
    std::string out;
    for (const scena::ir::ConditionGroup& group : phase.event->start_trigger->groups) {
        for (const scena::ir::TriggerCondition& condition : group.conditions) {
            if (const auto* time = dynamic_cast<const scena::ir::SimulationTimeCondition*>(
                    condition.expression.get())) {
                out += "t>=" + std::to_string(time->value()).substr(0, 4) + " ";
            } else if (const auto* element =
                           dynamic_cast<const scena::ir::StoryboardElementStateCondition*>(
                               condition.expression.get())) {
                out += element->element_ref() + " complete ";
            }
        }
    }
    return out;
}

TEST(DslLoweringTest, ConcreteDurationsBecomeAbsoluteStartTimes) {
    // The storyboard starts at t = 0 and every duration that lowers is a
    // constant, so a phase's start time is the sum of the ones before it —
    // arithmetic load time can do, with no runtime feedback.
    Lowered result;
    run(std::string(kPrelude).append(
            "scenario go:\n"
            "    ego: vehicle\n"
            "    do serial:\n"
            "        a: ego.assign_speed(speed: 10mps, duration: 4s)\n"
            "        b: ego.change_speed(target: 20mps, rate_peak: 2.0, duration: 6s)\n"),
        result);
    ASSERT_EQ(result.check_status, Status::Ok) << first_message(result.check_sink);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    const std::vector<Phase> lowered = phases(result.lowered.scenario);
    ASSERT_EQ(lowered.size(), 2U);
    // The first phase starts with its parent: `t >= 0` would be a tautology.
    EXPECT_EQ(trigger_text(lowered[0]), "(with parent)");
    EXPECT_EQ(trigger_text(lowered[1]), "t>=4.00 ");
    // The scenario ends when its `do` directive does, and here that is a
    // constant, so the storyboard can say so.
    ASSERT_TRUE(result.lowered.scenario.storyboard.stop_trigger.has_value());
}

TEST(DslLoweringTest, APhaseWithoutADurationChainsOnCompletionInstead) {
    Lowered result;
    run(std::string(kPrelude).append("scenario go:\n"
                                     "    ego: vehicle\n"
                                     "    do serial:\n"
                                     "        a: ego.assign_speed(speed: 10mps)\n"
                                     "        b: ego.remain_stationary()\n"),
        result);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    const std::vector<Phase> lowered = phases(result.lowered.scenario);
    ASSERT_EQ(lowered.size(), 2U);
    EXPECT_EQ(trigger_text(lowered[1]), "a complete ");
    // Nothing fixes when the run ends, so nothing claims to.
    EXPECT_FALSE(result.lowered.scenario.storyboard.stop_trigger.has_value());
}

TEST(DslLoweringTest, AParallelCompositionEndsWhenItsLastMemberDoes) {
    // §7.6.2.1.4's default overlap is `start`: members begin together and may
    // end apart, so what follows waits for the last of them.
    Lowered result;
    run(std::string(kPrelude).append(
            "scenario go:\n"
            "    ego: vehicle\n"
            "    lead: vehicle\n"
            "    do serial:\n"
            "        setup: ego.assign_speed(speed: 10mps, duration: 2s)\n"
            "        both: parallel:\n"
            "            x: ego.change_speed(target: 20mps, rate_peak: 2.0, duration: 5s)\n"
            "            y: lead.assign_speed(speed: 30mps, duration: 3s)\n"
            "        after: ego.remain_stationary()\n"),
        result);
    ASSERT_EQ(result.check_status, Status::Ok) << first_message(result.check_sink);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    const std::vector<Phase> lowered = phases(result.lowered.scenario);
    ASSERT_EQ(lowered.size(), 4U);
    // Both members of the parallel start when the composition does ...
    EXPECT_EQ(trigger_text(lowered[1]), "t>=2.00 ");
    EXPECT_EQ(trigger_text(lowered[2]), "t>=2.00 ");
    EXPECT_EQ(lowered[1].actor, "ego");
    EXPECT_EQ(lowered[2].actor, "lead");
    // ... and the phase after it waits for the longer of the two (2 + 5).
    EXPECT_EQ(trigger_text(lowered[3]), "t>=7.00 ");
}

TEST(DslLoweringTest, AParallelJoinAndsTheMembersThatEndOnCompletion) {
    // A ConditionGroup is an AND, which is what makes "all members done"
    // expressible when their end times are not constants.
    Lowered result;
    run(std::string(kPrelude).append("scenario go:\n"
                                     "    ego: vehicle\n"
                                     "    lead: vehicle\n"
                                     "    do serial:\n"
                                     "        both: parallel:\n"
                                     "            x: ego.assign_speed(speed: 10mps)\n"
                                     "            y: lead.assign_speed(speed: 30mps)\n"
                                     "        after: ego.remain_stationary()\n"),
        result);
    ASSERT_EQ(result.check_status, Status::Ok) << first_message(result.check_sink);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    const std::vector<Phase> lowered = phases(result.lowered.scenario);
    ASSERT_EQ(lowered.size(), 3U);
    EXPECT_EQ(trigger_text(lowered[2]), "x complete y complete ");
}

TEST(DslLoweringTest, WaitElapsedAdvancesTheClockAndNothingElse) {
    // §7.6.2.4.2: a phase of the given length in which nothing is specified.
    // Nothing is exactly what it lowers to — the clock is already running.
    Lowered result;
    run(std::string(kPrelude).append("scenario go:\n"
                                     "    ego: vehicle\n"
                                     "    do serial:\n"
                                     "        a: ego.assign_speed(speed: 10mps, duration: 2s)\n"
                                     "        wait elapsed(3s)\n"
                                     "        b: ego.remain_stationary()\n"),
        result);
    ASSERT_EQ(result.check_status, Status::Ok) << first_message(result.check_sink);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    const std::vector<Phase> lowered = phases(result.lowered.scenario);
    ASSERT_EQ(lowered.size(), 2U);
    EXPECT_EQ(trigger_text(lowered[1]), "t>=5.00 ");
}

TEST(DslLoweringTest, OneOfRunsItsFirstAlternativeUnlessOneIsNamed) {
    // §7.6.2.1.3 says at least one alternative must hold and says nothing about
    // which, so the choice is an input rather than a coin toss.
    const std::string source =
        std::string(kPrelude).append("scenario go:\n"
                                     "    ego: vehicle\n"
                                     "    do one_of:\n"
                                     "        slow: ego.assign_speed(speed: 10mps)\n"
                                     "        fast: ego.assign_speed(speed: 30mps)\n");
    Lowered first;
    run(source, first);
    ASSERT_EQ(first.status, Status::Ok) << first_message(first.sink);
    ASSERT_EQ(phases(first.lowered.scenario).size(), 1U);
    EXPECT_EQ(phases(first.lowered.scenario)[0].name, "slow");
}

TEST(DslLoweringTest, AnAlternativeThatIsNotThereIsReported) {
    Lowered result;
    run(std::string(kPrelude).append("scenario go:\n"
                                     "    ego: vehicle\n"
                                     "    do one_of:\n"
                                     "        slow: ego.assign_speed(speed: 10mps)\n"
                                     "        fast: ego.assign_speed(speed: 30mps)\n"),
        result, "", "nope");
    EXPECT_EQ(result.status, Status::ValidationError);
    EXPECT_TRUE(sink_says(result.sink, "not an alternative"));
}

TEST(DslLoweringTest, ARangeDurationBoundsTracesRatherThanFixingATime) {
    // §7.6.2.4's range is a constraint on accepted traces; choosing a value
    // from it needs a solver, which is post-v0.0.1 (ADR-0004).
    Lowered result;
    run(std::string(kPrelude).append(
            "scenario go:\n"
            "    ego: vehicle\n"
            "    do a: ego.assign_speed(speed: 10mps, duration: [2s..4s])\n"),
        result);
    ASSERT_EQ(result.check_status, Status::Ok) << first_message(result.check_sink);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    EXPECT_TRUE(sink_says(result.sink, "not a single value"));
    EXPECT_FALSE(result.lowered.scenario.storyboard.stop_trigger.has_value());
}

TEST(DslLoweringTest, ANonDefaultParallelOverlapIsReported) {
    Lowered result;
    run(std::string(kPrelude).append("scenario go:\n"
                                     "    ego: vehicle\n"
                                     "    lead: vehicle\n"
                                     "    do parallel(overlap: equal):\n"
                                     "        x: ego.assign_speed(speed: 10mps)\n"
                                     "        y: lead.assign_speed(speed: 30mps)\n"),
        result);
    ASSERT_EQ(result.check_status, Status::Ok) << first_message(result.check_sink);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    EXPECT_TRUE(sink_says(result.sink, "overlap 'equal'"));
}

TEST(DslLoweringTest, ModifiersAreReportedAsDeferred) {
    // A modifier is a real construct of the invocation that belongs to a later
    // sprint. Reporting beats silently running a scenario that says more than
    // the engine was told.
    Lowered result;
    run(std::string(kPrelude).append("scenario go:\n"
                                     "    ego: vehicle\n"
                                     "    do phase: ego.drive() with:\n"
                                     "        speed(speed: 30kph)\n"),
        result);
    ASSERT_EQ(result.check_status, Status::Ok) << first_message(result.check_sink);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    EXPECT_TRUE(sink_says(result.sink, "p8-s3 (#46)"));
}

// --- Â§8.5.4 the map file ------------------------------------------------------

TEST(DslLoweringTest, TheMapFileComesFromSetMapFile) {
    // Code 61's spelling, where `map` names the actor type: the road network is
    // a singleton no scenario declares a field for.
    Lowered result;
    run(std::string(kPrelude).append("scenario go:\n"
                                     "    map.set_map_file(\"flat.xodr\")\n"
                                     "    ego: vehicle\n"),
        result);
    ASSERT_EQ(result.check_status, Status::Ok) << first_message(result.check_sink);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    EXPECT_EQ(result.lowered.map_file, "flat.xodr");
}

TEST(DslLoweringTest, TheMapFileCanAlsoComeFromAKeep) {
    // Code 62's spelling. The two say the same thing, so they lower the same.
    Lowered result;
    run(std::string(kPrelude).append("scenario go:\n"
                                     "    my_map: map\n"
                                     "    ego: vehicle\n"
                                     "    keep(my_map.map_file == \"junction.xodr\")\n"),
        result);
    ASSERT_EQ(result.check_status, Status::Ok) << first_message(result.check_sink);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    EXPECT_EQ(result.lowered.map_file, "junction.xodr");
}

TEST(DslLoweringTest, AScenarioThatNamesNoMapLeavesTheReferenceEmpty) {
    Lowered result;
    run(std::string(kPrelude).append("scenario go:\n    ego: vehicle\n"), result);
    ASSERT_EQ(result.status, Status::Ok) << first_message(result.sink);
    EXPECT_TRUE(result.lowered.map_file.empty());
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
    ASSERT_EQ(first.lowered.scenario.entities.size(), second.lowered.scenario.entities.size());
    for (std::size_t index = 0; index < first.lowered.scenario.entities.size(); ++index) {
        const scena::ir::Entity& left = first.lowered.scenario.entities[index];
        const scena::ir::Entity& right = second.lowered.scenario.entities[index];
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
