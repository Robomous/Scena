// SPDX-License-Identifier: MIT
#include "scena/diagnostic.h"

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "scena/engine.h"
#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/trigger.h"
#include "scena/status.h"

using scena::Diagnostic;
using scena::DiagnosticSink;
using scena::Engine;
using scena::Severity;
using scena::SourceLocation;
using scena::Status;
using scena::ir::ControlMode;
using scena::ir::Scenario;
using scena::ir::SimulationTimeCondition;
using scena::ir::SpeedAction;

namespace {

Diagnostic make_diagnostic(Severity severity, std::string message) {
    Diagnostic diagnostic;
    diagnostic.severity = severity;
    diagnostic.code = Status::ValidationError;
    diagnostic.message = std::move(message);
    return diagnostic;
}

scena::ir::Event make_speed_event(std::string name, double at_time, std::string entity_id,
                                  double target_speed) {
    scena::ir::Event event;
    event.name = std::move(name);
    event.start_trigger =
        scena::ir::make_trigger(std::make_shared<SimulationTimeCondition>(at_time));
    event.actions.push_back(std::make_shared<SpeedAction>(std::move(entity_id), target_speed));
    return event;
}

/// One story/act/group/maneuver chain (all trigger-less) around the events;
/// mirrors the fixture in engine_test.cpp so the paths below are stable.
Scenario make_scenario(std::vector<scena::ir::Event> events = {
                           make_speed_event("event-1", 0.0, "ego", 10.0)}) {
    Scenario scenario;
    scenario.name = "diagnostics-test";
    scenario.entities.push_back(
        {.id = "ego", .name = "ego vehicle", .control_mode = ControlMode::EngineControlled});

    scena::ir::Maneuver maneuver;
    maneuver.name = "maneuver";
    maneuver.events = std::move(events);
    scena::ir::ManeuverGroup group;
    group.name = "group";
    group.actors.push_back("ego");
    group.maneuvers.push_back(std::move(maneuver));
    scena::ir::Act act;
    act.name = "act";
    act.groups.push_back(std::move(group));
    scena::ir::Story story;
    story.name = "story";
    story.acts.push_back(std::move(act));
    scenario.storyboard.stories.push_back(std::move(story));
    return scenario;
}

/// The single diagnostic of a one-defect init; fails the test if init did
/// not report exactly one.
const Diagnostic& only_diagnostic(const Engine& engine) {
    EXPECT_EQ(engine.diagnostics().size(), 1U);
    return engine.diagnostics().front();
}

/// An action kind the engine does not implement, so applying it takes the
/// runtime's unsupported-action path. It targets a valid entity, so init
/// validation accepts it.
class UnsupportedAction final : public scena::ir::Action {
public:
    explicit UnsupportedAction(std::string entity_id) : entity_id_(std::move(entity_id)) {}
    [[nodiscard]] const std::string& entity_id() const override { return entity_id_; }
    [[nodiscard]] std::string_view kind() const noexcept override { return "TeleportAction"; }

private:
    std::string entity_id_;
};

/// A trigger-less event carrying one action, so it fires at t = 0.
scena::ir::Event make_event(std::string name, std::shared_ptr<scena::ir::Action> action) {
    scena::ir::Event event;
    event.name = std::move(name);
    event.actions.push_back(std::move(action));
    return event;
}

} // namespace

TEST(DiagnosticSinkTest, StartsEmpty) {
    const DiagnosticSink sink;
    EXPECT_TRUE(sink.diagnostics().empty());
    EXPECT_FALSE(sink.has_errors());
}

TEST(DiagnosticSinkTest, PreservesReportOrder) {
    DiagnosticSink sink;
    sink.report(make_diagnostic(Severity::Info, "first"));
    sink.report(make_diagnostic(Severity::Warning, "second"));
    sink.report(make_diagnostic(Severity::Error, "third"));

    ASSERT_EQ(sink.diagnostics().size(), 3U);
    EXPECT_EQ(sink.diagnostics()[0].message, "first");
    EXPECT_EQ(sink.diagnostics()[1].message, "second");
    EXPECT_EQ(sink.diagnostics()[2].message, "third");
}

TEST(DiagnosticSinkTest, HasErrorsOnlyForErrorSeverity) {
    DiagnosticSink sink;
    sink.report(make_diagnostic(Severity::Info, "info"));
    sink.report(make_diagnostic(Severity::Warning, "warning"));
    EXPECT_FALSE(sink.has_errors());
    sink.report(make_diagnostic(Severity::Error, "error"));
    EXPECT_TRUE(sink.has_errors());
}

TEST(DiagnosticSinkTest, ClearDropsEverything) {
    DiagnosticSink sink;
    sink.report(make_diagnostic(Severity::Error, "error"));
    ASSERT_FALSE(sink.diagnostics().empty());
    sink.clear();
    EXPECT_TRUE(sink.diagnostics().empty());
    EXPECT_FALSE(sink.has_errors());
}

TEST(DiagnosticSinkTest, DefaultsAreUnknownLocationAndNoRule) {
    const Diagnostic diagnostic;
    EXPECT_EQ(diagnostic.severity, Severity::Error);
    EXPECT_EQ(diagnostic.code, Status::Ok);
    EXPECT_TRUE(diagnostic.path.empty());
    EXPECT_TRUE(diagnostic.rule_id.empty());

    const SourceLocation& location = diagnostic.location;
    EXPECT_TRUE(location.file.empty());
    EXPECT_EQ(location.line, 0);
    EXPECT_EQ(location.column, 0);
}

TEST(DiagnosticsTest, EngineStartsWithNoDiagnostics) {
    const Engine engine;
    EXPECT_TRUE(engine.diagnostics().empty());
}

TEST(DiagnosticsTest, ValidScenarioProducesNoDiagnostics) {
    Engine engine;
    scena::ir::Scenario scenario;
    scenario.name = "empty";
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    EXPECT_TRUE(engine.diagnostics().empty());
    EXPECT_EQ(engine.close(), Status::Ok);
    EXPECT_TRUE(engine.diagnostics().empty());
}

TEST(DiagnosticsTest, ClearDiagnosticsIsAvailableToHosts) {
    Engine engine;
    engine.clear_diagnostics(); // no-op on a fresh engine, never a failure
    EXPECT_TRUE(engine.diagnostics().empty());
}

TEST(DiagnosticsTest, EmptyEntityId) {
    Engine engine;
    Scenario scenario = make_scenario();
    scenario.entities.push_back(
        {.id = "", .name = "empty id", .control_mode = ControlMode::EngineControlled});
    ASSERT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    const Diagnostic& d = only_diagnostic(engine);
    EXPECT_EQ(d.severity, Severity::Error);
    EXPECT_EQ(d.code, Status::ValidationError);
    EXPECT_EQ(d.path, "entities[1]");
    EXPECT_TRUE(d.rule_id.empty());
}

TEST(DiagnosticsTest, DuplicateEntityId) {
    Engine engine;
    Scenario scenario = make_scenario();
    scenario.entities.push_back(
        {.id = "ego", .name = "duplicate", .control_mode = ControlMode::EngineControlled});
    ASSERT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    const Diagnostic& d = only_diagnostic(engine);
    EXPECT_EQ(d.code, Status::ValidationError);
    EXPECT_EQ(d.path, "entities/ego");
    EXPECT_EQ(d.message, "duplicate entity id 'ego'");
    EXPECT_EQ(d.rule_id, "asam.net:xosc:1.0.0:naming.unique_element_names_on_same_level");
}

TEST(DiagnosticsTest, InvalidBoundingBoxDimensions) {
    Engine engine;
    Scenario scenario = make_scenario();
    scena::ir::MiscObject obj;
    // Negative length is a content defect (§Dimensions range [0..inf[).
    obj.bounding_box = scena::ir::BoundingBox{0.0, 0.0, 0.0, -1.0, 2.0, 1.5};
    scenario.entities.push_back({.id = "boxed",
                                 .name = "boxed",
                                 .control_mode = ControlMode::HostControlled,
                                 .object = obj});
    ASSERT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    const Diagnostic& d = only_diagnostic(engine);
    EXPECT_EQ(d.code, Status::ValidationError);
    EXPECT_EQ(d.path, "entities/boxed");
    EXPECT_EQ(d.message, "entity 'boxed' has an invalid bounding box");
}

TEST(DiagnosticsTest, InvalidPerformanceLimits) {
    Engine engine;
    Scenario scenario = make_scenario();
    scena::ir::Vehicle veh;
    // Negative maxAcceleration is a content defect (§Performance range [0..inf[).
    veh.performance = scena::ir::Performance{50.0, -1.0, 9.0, std::nullopt, std::nullopt};
    scenario.entities.push_back({.id = "veh",
                                 .name = "veh",
                                 .control_mode = ControlMode::HostControlled,
                                 .object = veh});
    ASSERT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    const Diagnostic& d = only_diagnostic(engine);
    EXPECT_EQ(d.code, Status::ValidationError);
    EXPECT_EQ(d.path, "entities/veh");
    EXPECT_EQ(d.message, "entity 'veh' has invalid performance limits");
    // §Performance defines no asam.net checker rule id.
    EXPECT_TRUE(d.rule_id.empty());
}

TEST(DiagnosticsTest, NullInitAction) {
    Engine engine;
    Scenario scenario = make_scenario();
    scenario.init_actions.push_back(nullptr);
    ASSERT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    const Diagnostic& d = only_diagnostic(engine);
    EXPECT_EQ(d.code, Status::ValidationError);
    EXPECT_EQ(d.path, "init/action[0]");
}

TEST(DiagnosticsTest, InitActionTargetsUnknownEntity) {
    Engine engine;
    Scenario scenario = make_scenario();
    scenario.init_actions.push_back(std::make_shared<SpeedAction>("missing", 5.0));
    ASSERT_EQ(engine.init(std::move(scenario)), Status::SemanticError);
    const Diagnostic& d = only_diagnostic(engine);
    EXPECT_EQ(d.severity, Severity::Error);
    EXPECT_EQ(d.code, Status::SemanticError);
    EXPECT_EQ(d.path, "init/action[0]");
    EXPECT_EQ(d.message, "action targets unknown entity 'missing'");
    // No checker rule exists for entity-reference resolvability (Annex C).
    EXPECT_TRUE(d.rule_id.empty());
}

TEST(DiagnosticsTest, DuplicateSiblingEventName) {
    Engine engine;
    Scenario scenario = make_scenario({make_speed_event("event-1", 1.0, "ego", 5.0),
                                       make_speed_event("event-1", 2.0, "ego", 7.0)});
    ASSERT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    const Diagnostic& d = only_diagnostic(engine);
    EXPECT_EQ(d.code, Status::ValidationError);
    EXPECT_EQ(d.path, "story/act/group/maneuver/event-1");
    EXPECT_EQ(d.rule_id, "asam.net:xosc:1.0.0:naming.unique_element_names_on_same_level");
}

TEST(DiagnosticsTest, EmptyStoryName) {
    Engine engine;
    Scenario scenario = make_scenario();
    scenario.storyboard.stories[0].name.clear();
    ASSERT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    const Diagnostic& d = only_diagnostic(engine);
    EXPECT_EQ(d.code, Status::ValidationError);
    EXPECT_EQ(d.path, "[0]");
}

TEST(DiagnosticsTest, ActorReferencesUnknownEntity) {
    Engine engine;
    Scenario scenario = make_scenario();
    scenario.storyboard.stories[0].acts[0].groups[0].actors.push_back("missing");
    ASSERT_EQ(engine.init(std::move(scenario)), Status::SemanticError);
    const Diagnostic& d = only_diagnostic(engine);
    EXPECT_EQ(d.code, Status::SemanticError);
    EXPECT_EQ(d.path, "story/act/group");
    EXPECT_EQ(d.message, "actor references unknown entity 'missing'");
}

TEST(DiagnosticsTest, NegativeMaximumExecutionCount) {
    Engine engine;
    Scenario scenario = make_scenario();
    scenario.storyboard.stories[0]
        .acts[0]
        .groups[0]
        .maneuvers[0]
        .events[0]
        .maximum_execution_count = -1;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    const Diagnostic& d = only_diagnostic(engine);
    EXPECT_EQ(d.code, Status::ValidationError);
    EXPECT_EQ(d.path, "story/act/group/maneuver/event-1");
}

TEST(DiagnosticsTest, EventHasNoActions) {
    Engine engine;
    Scenario scenario = make_scenario();
    scenario.storyboard.stories[0].acts[0].groups[0].maneuvers[0].events[0].actions.clear();
    ASSERT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    const Diagnostic& d = only_diagnostic(engine);
    EXPECT_EQ(d.code, Status::ValidationError);
    EXPECT_EQ(d.path, "story/act/group/maneuver/event-1");
    EXPECT_EQ(d.message, "event has no actions");
}

TEST(DiagnosticsTest, NullEventAction) {
    Engine engine;
    Scenario scenario = make_scenario();
    scenario.storyboard.stories[0].acts[0].groups[0].maneuvers[0].events[0].actions.push_back(
        nullptr);
    ASSERT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    const Diagnostic& d = only_diagnostic(engine);
    EXPECT_EQ(d.code, Status::ValidationError);
    EXPECT_EQ(d.path, "story/act/group/maneuver/event-1/action[1]");
}

TEST(DiagnosticsTest, EventActionTargetsUnknownEntity) {
    Engine engine;
    Scenario scenario = make_scenario({make_speed_event("event-1", 1.0, "missing", 5.0)});
    ASSERT_EQ(engine.init(std::move(scenario)), Status::SemanticError);
    const Diagnostic& d = only_diagnostic(engine);
    EXPECT_EQ(d.code, Status::SemanticError);
    EXPECT_EQ(d.path, "story/act/group/maneuver/event-1/action[0]");
    EXPECT_EQ(d.message, "action targets unknown entity 'missing'");
}

TEST(DiagnosticsTest, EmptyConditionGroup) {
    Engine engine;
    Scenario scenario = make_scenario();
    scenario.storyboard.stories[0]
        .acts[0]
        .groups[0]
        .maneuvers[0]
        .events[0]
        .start_trigger->groups[0]
        .conditions.clear();
    ASSERT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    const Diagnostic& d = only_diagnostic(engine);
    EXPECT_EQ(d.code, Status::ValidationError);
    EXPECT_EQ(d.path, "story/act/group/maneuver/event-1/startTrigger/group[0]");
}

TEST(DiagnosticsTest, NullConditionExpression) {
    Engine engine;
    Scenario scenario = make_scenario();
    scenario.storyboard.stories[0]
        .acts[0]
        .groups[0]
        .maneuvers[0]
        .events[0]
        .start_trigger->groups[0]
        .conditions[0]
        .expression = nullptr;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    const Diagnostic& d = only_diagnostic(engine);
    EXPECT_EQ(d.code, Status::ValidationError);
    EXPECT_EQ(d.path, "story/act/group/maneuver/event-1/startTrigger/group[0]/condition[0]");
}

TEST(DiagnosticsTest, NegativeConditionDelayCitesRule) {
    Engine engine;
    Scenario scenario = make_scenario();
    scenario.storyboard.stories[0]
        .acts[0]
        .groups[0]
        .maneuvers[0]
        .events[0]
        .start_trigger->groups[0]
        .conditions[0]
        .delay = -0.5;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    const Diagnostic& d = only_diagnostic(engine);
    EXPECT_EQ(d.code, Status::ValidationError);
    EXPECT_EQ(d.message, "condition delay is negative");
    EXPECT_EQ(d.rule_id, "asam.net:xosc:1.0.0:data_type.condition_delay_not_negative");
}

TEST(DiagnosticsTest, NamedConditionUsesNameInPath) {
    Engine engine;
    Scenario scenario = make_scenario();
    auto& condition = scenario.storyboard.stories[0]
                          .acts[0]
                          .groups[0]
                          .maneuvers[0]
                          .events[0]
                          .start_trigger->groups[0]
                          .conditions[0];
    condition.name = "reached-time";
    condition.delay = -1.0;
    ASSERT_EQ(engine.init(std::move(scenario)), Status::ValidationError);
    EXPECT_EQ(only_diagnostic(engine).path,
              "story/act/group/maneuver/event-1/startTrigger/group[0]/reached-time");
}

TEST(DiagnosticsTest, AccumulatesMultipleDefectsInDocumentOrder) {
    // Two independent defects: a semantic one on the actor (earlier in the
    // walk) and a structural one on the event's condition delay.
    Engine engine;
    Scenario scenario = make_scenario();
    scenario.storyboard.stories[0].acts[0].groups[0].actors.push_back("missing");
    scenario.storyboard.stories[0]
        .acts[0]
        .groups[0]
        .maneuvers[0]
        .events[0]
        .start_trigger->groups[0]
        .conditions[0]
        .delay = -1.0;
    // init() returns the code of the first error in document order — the
    // actor is validated before the event's trigger.
    ASSERT_EQ(engine.init(std::move(scenario)), Status::SemanticError);
    ASSERT_EQ(engine.diagnostics().size(), 2U);
    EXPECT_EQ(engine.diagnostics()[0].code, Status::SemanticError);
    EXPECT_EQ(engine.diagnostics()[0].path, "story/act/group");
    EXPECT_EQ(engine.diagnostics()[1].code, Status::ValidationError);
    EXPECT_EQ(engine.diagnostics()[1].message, "condition delay is negative");
}

TEST(DiagnosticsTest, FailedInitLeavesEngineUninitializedWithReadableDiagnostics) {
    Engine engine;
    Scenario scenario = make_scenario({make_speed_event("event-1", 1.0, "missing", 5.0)});
    ASSERT_EQ(engine.init(std::move(scenario)), Status::SemanticError);
    EXPECT_FALSE(engine.initialized());
    // Diagnostics from a failed init remain readable.
    EXPECT_FALSE(engine.diagnostics().empty());
}

TEST(DiagnosticsTest, SuccessfulReinitClearsPriorDiagnostics) {
    Engine engine;
    Scenario bad = make_scenario({make_speed_event("event-1", 1.0, "missing", 5.0)});
    ASSERT_EQ(engine.init(std::move(bad)), Status::SemanticError);
    ASSERT_FALSE(engine.diagnostics().empty());

    ASSERT_EQ(engine.init(make_scenario()), Status::Ok);
    EXPECT_TRUE(engine.diagnostics().empty());
}

TEST(DiagnosticsTest, RejectedReinitPreservesRecord) {
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario()), Status::Ok);
    // A second init on an already-initialized engine is rejected before the
    // sink is touched, so an earlier record would survive; here it is empty.
    ASSERT_EQ(engine.init(make_scenario()), Status::AlreadyInitialized);
    EXPECT_TRUE(engine.diagnostics().empty());
}

TEST(DiagnosticsTest, ClosePreservesDiagnostics) {
    Engine engine;
    Scenario scenario = make_scenario();
    scenario.storyboard.stories[0].acts[0].groups[0].actors.push_back("missing");
    ASSERT_EQ(engine.init(std::move(scenario)), Status::SemanticError);
    // The failed init never initialized the engine, so re-run against a valid
    // scenario, then close and confirm the record survives for post-mortem.
    ASSERT_EQ(engine.init(make_scenario({make_speed_event("event-1", 1.0, "missing", 5.0)})),
              Status::SemanticError);
    const std::size_t count = engine.diagnostics().size();
    ASSERT_GT(count, 0U);
    ASSERT_EQ(engine.init(make_scenario()), Status::Ok); // clears
    ASSERT_TRUE(engine.diagnostics().empty());
    ASSERT_EQ(engine.close(), Status::Ok);
    EXPECT_TRUE(engine.diagnostics().empty()); // valid run produced none
}

TEST(DiagnosticsTest, UnsupportedActionKindWarnsButStepStaysOk) {
    Engine engine;
    Scenario scenario =
        make_scenario({make_event("event-1", std::make_shared<UnsupportedAction>("ego"))});
    // A trigger-less event fires during init (§8.4.7), so the warning is
    // already present; init still succeeds.
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    ASSERT_EQ(engine.diagnostics().size(), 1U);
    const Diagnostic& d = engine.diagnostics().front();
    EXPECT_EQ(d.severity, Severity::Warning);
    EXPECT_EQ(d.code, Status::UnsupportedFeature);
    EXPECT_EQ(d.path, "entities/ego");
    // The kind name appears in the message so a host can identify it.
    EXPECT_NE(d.message.find("TeleportAction"), std::string::npos);

    // Stepping keeps returning Ok: a degraded scenario is not a broken engine.
    EXPECT_EQ(engine.step(0.01), Status::Ok);
    EXPECT_FALSE(engine.diagnostics().empty());
}

TEST(DiagnosticsTest, RuntimeWarningsAccumulateAcrossSteps) {
    // An event with maximumExecutionCount 2 fires twice, on two evaluations.
    Engine engine;
    scena::ir::Event event = make_event("event-1", std::make_shared<UnsupportedAction>("ego"));
    event.start_trigger = scena::ir::make_trigger(std::make_shared<SimulationTimeCondition>(1.0));
    event.maximum_execution_count = 1;
    Scenario scenario = make_scenario({std::move(event)});
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    EXPECT_TRUE(engine.diagnostics().empty()); // has not fired yet

    for (int i = 0; i < 150; ++i) {
        ASSERT_EQ(engine.step(0.01), Status::Ok);
    }
    EXPECT_EQ(engine.diagnostics().size(), 1U);
    EXPECT_EQ(engine.diagnostics().front().code, Status::UnsupportedFeature);
}

TEST(DiagnosticsTest, IdenticalDefectiveScenariosProduceIdenticalDiagnostics) {
    Scenario scenario_a = make_scenario();
    scenario_a.storyboard.stories[0].acts[0].groups[0].actors.push_back("missing");
    scenario_a.storyboard.stories[0]
        .acts[0]
        .groups[0]
        .maneuvers[0]
        .events[0]
        .start_trigger->groups[0]
        .conditions[0]
        .delay = -1.0;
    Scenario scenario_b = scenario_a;

    Engine engine_a;
    Engine engine_b;
    ASSERT_EQ(engine_a.init(std::move(scenario_a)), Status::SemanticError);
    ASSERT_EQ(engine_b.init(std::move(scenario_b)), Status::SemanticError);

    const auto& a = engine_a.diagnostics();
    const auto& b = engine_b.diagnostics();
    ASSERT_EQ(a.size(), b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        EXPECT_EQ(a[i].severity, b[i].severity);
        EXPECT_EQ(a[i].code, b[i].code);
        EXPECT_EQ(a[i].message, b[i].message);
        EXPECT_EQ(a[i].path, b[i].path);
        EXPECT_EQ(a[i].rule_id, b[i].rule_id);
    }
}
