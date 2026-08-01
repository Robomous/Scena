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

// Multi-version support: the same scenario written for 1.0, 1.1, 1.2 and 1.3
// must load to the same IR, and a construct deprecated by a later revision
// must still execute — reported only in the versions where its successor
// exists.

#include <string>
#include <string_view>
#include <variant>

#include <gtest/gtest.h>

#include "scena/diagnostic.h"
#include "scena/engine.h"
#include "scena/ir/action.h"
#include "scena/ir/position.h"
#include "scena/xml/loader.h"

namespace {

using scena::Diagnostic;
using scena::DiagnosticSink;
using scena::Severity;
using scena::Status;
using scena::xml::Document;

std::string document(int rev_minor, std::string_view body) {
    return std::string("<OpenSCENARIO><FileHeader revMajor=\"1\" revMinor=\"") +
           std::to_string(rev_minor) +
           "\" date=\"2026-08-01T00:00:00\" description=\"version fixture\" author=\"Scena\"/>" +
           std::string(body) + "</OpenSCENARIO>";
}

/// A cut-in-shaped scenario using only constructs that exist unchanged in
/// 1.0 through 1.3: two vehicles, an init teleport and speed, and an event
/// on a simulation-time condition.
constexpr std::string_view kCommonBody = R"(<Entities>
  <ScenarioObject name="ego">
    <Vehicle name="v1" vehicleCategory="car">
      <BoundingBox><Center x="1.4" y="0" z="0.9"/><Dimensions width="2" length="4.6" height="1.8"/></BoundingBox>
      <Performance maxSpeed="60" maxAcceleration="5" maxDeceleration="9"/>
      <Axles><RearAxle maxSteering="0" wheelDiameter="0.6" trackWidth="1.7" positionX="0" positionZ="0.3"/></Axles>
    </Vehicle>
  </ScenarioObject>
</Entities>
<Storyboard>
  <Init><Actions><Private entityRef="ego">
    <PrivateAction><TeleportAction>
      <Position><WorldPosition x="0" y="0" z="0" h="0"/></Position>
    </TeleportAction></PrivateAction>
    <PrivateAction><LongitudinalAction><SpeedAction>
      <SpeedActionDynamics dynamicsShape="step" dynamicsDimension="time" value="0"/>
      <SpeedActionTarget><AbsoluteTargetSpeed value="20"/></SpeedActionTarget>
    </SpeedAction></LongitudinalAction></PrivateAction>
  </Private></Actions></Init>
  <Story name="s"><Act name="a"><ManeuverGroup name="g">
    <Actors selectTriggeringEntities="false"><EntityRef entityRef="ego"/></Actors>
    <Maneuver name="m"><Event name="e" priority="parallel">
      <Action name="accelerate"><PrivateAction><LongitudinalAction><SpeedAction>
        <SpeedActionDynamics dynamicsShape="linear" dynamicsDimension="time" value="4"/>
        <SpeedActionTarget><AbsoluteTargetSpeed value="30"/></SpeedActionTarget>
      </SpeedAction></LongitudinalAction></PrivateAction></Action>
      <StartTrigger><ConditionGroup><Condition name="t5" delay="0" conditionEdge="none">
        <ByValueCondition><SimulationTimeCondition value="5" rule="greaterThan"/></ByValueCondition>
      </Condition></ConditionGroup></StartTrigger>
    </Event></Maneuver>
  </ManeuverGroup>
  <StartTrigger><ConditionGroup><Condition name="start" delay="0" conditionEdge="none">
    <ByValueCondition><SimulationTimeCondition value="0" rule="greaterThan"/></ByValueCondition>
  </Condition></ConditionGroup></StartTrigger>
  </Act></Story>
</Storyboard>)";

/// The entity state after a fixed run, which is what "equivalent IR" means
/// in practice: the same document, whatever revision it declares, must
/// simulate identically.
scena::EntityState run(const Document& document) {
    scena::Engine engine;
    EXPECT_EQ(engine.init(document.scenario), Status::Ok);
    for (int step = 0; step < 100; ++step) {
        EXPECT_EQ(engine.step(0.1), Status::Ok);
    }
    return engine.state("ego").value();
}

class VersionMatrix : public testing::TestWithParam<int> {};

TEST_P(VersionMatrix, TheSameScenarioLoadsAndRunsIdentically) {
    Document reference;
    DiagnosticSink reference_sink;
    ASSERT_EQ(scena::xml::load_string(document(0, kCommonBody), reference, reference_sink),
              Status::Ok);
    const scena::EntityState expected = run(reference);

    Document document_under_test;
    DiagnosticSink sink;
    ASSERT_EQ(scena::xml::load_string(document(GetParam(), kCommonBody), document_under_test, sink),
              Status::Ok);
    const scena::EntityState actual = run(document_under_test);

    // Bit-identical, not merely close: §5 says the corrected position and
    // orientation calculations apply to every version, so Scena runs one set
    // of semantics for the whole range and the revision changes nothing.
    EXPECT_EQ(actual.x, expected.x);
    EXPECT_EQ(actual.y, expected.y);
    EXPECT_EQ(actual.heading, expected.heading);
    EXPECT_EQ(actual.speed, expected.speed);
}

INSTANTIATE_TEST_SUITE_P(OnePointZeroToOnePointThree, VersionMatrix, testing::Values(0, 1, 2, 3));

// --- deprecated constructs ------------------------------------------------

/// The entity every deprecated-construct fixture acts on. The validation
/// pass checks references, so a fixture must declare what it names.
constexpr std::string_view kEgo = R"(<Entities>
  <ScenarioObject name="ego">
    <Vehicle name="v" vehicleCategory="car">
      <BoundingBox><Center x="0" y="0" z="0"/><Dimensions width="2" length="4" height="1.5"/></BoundingBox>
      <Performance maxSpeed="60" maxAcceleration="5" maxDeceleration="9"/>
      <Axles><RearAxle maxSteering="0" wheelDiameter="0.6" trackWidth="1.7" positionX="0" positionZ="0.3"/></Axles>
    </Vehicle>
  </ScenarioObject>
</Entities>)";

/// Wraps a private action into the smallest scenario at the given revision.
std::string with_private_action(int rev_minor, std::string_view action) {
    return document(rev_minor, std::string(kEgo) +
                                   std::string(R"(<Storyboard><Story name="s"><Act name="a">
      <ManeuverGroup name="g">
        <Actors selectTriggeringEntities="false"><EntityRef entityRef="ego"/></Actors>
        <Maneuver name="m"><Event name="e"><Action name="x"><PrivateAction>)") +
                                   std::string(action) +
                                   R"(</PrivateAction></Action></Event></Maneuver>
      </ManeuverGroup></Act></Story></Storyboard>)");
}

bool has_deprecation(const DiagnosticSink& sink) {
    for (const Diagnostic& diagnostic : sink.diagnostics()) {
        if (diagnostic.code == Status::DeprecatedFeature) {
            return true;
        }
    }
    return false;
}

TEST(Deprecated, TheOldActivateControllerPlacementExecutesEitherWay) {
    constexpr std::string_view kAction =
        R"(<ActivateControllerAction lateral="true" longitudinal="true"/>)";

    // 1.0: the placement under PrivateAction is the only one there is, so
    // there is nothing to warn about.
    Document early;
    DiagnosticSink early_sink;
    ASSERT_EQ(scena::xml::load_string(with_private_action(0, kAction), early, early_sink),
              Status::Ok);
    EXPECT_FALSE(has_deprecation(early_sink));

    // 1.2: ControllerAction/ActivateControllerAction exists, so the old
    // placement is reported — and still executed.
    Document late;
    DiagnosticSink late_sink;
    ASSERT_EQ(scena::xml::load_string(with_private_action(2, kAction), late, late_sink),
              Status::Ok);
    EXPECT_TRUE(has_deprecation(late_sink));

    const auto& early_action = early.scenario.storyboard.stories.at(0)
                                   .acts.at(0)
                                   .groups.at(0)
                                   .maneuvers.at(0)
                                   .events.at(0)
                                   .actions.front();
    EXPECT_EQ(early_action->kind(), "ActivateControllerAction");
}

TEST(Deprecated, ParameterActionsExecuteAndAreReportedFromOneTwo) {
    const auto load = [](int rev_minor, DiagnosticSink& sink) {
        const std::string source = document(rev_minor,
                                            R"(<ParameterDeclarations>
                 <ParameterDeclaration name="p" parameterType="double" value="1.0"/>
               </ParameterDeclarations>)" + std::string(kEgo) +
                                                R"(<Storyboard><Init><Actions><GlobalAction>
                 <ParameterAction parameterRef="p"><SetAction value="2.0"/></ParameterAction>
               </GlobalAction></Actions></Init></Storyboard>)");
        Document document_under_test;
        EXPECT_EQ(scena::xml::load_string(source, document_under_test, sink), Status::Ok);
        return document_under_test;
    };

    DiagnosticSink early_sink;
    const Document early = load(1, early_sink);
    EXPECT_FALSE(has_deprecation(early_sink));
    ASSERT_EQ(early.scenario.init_actions.size(), 1U);
    EXPECT_EQ(early.scenario.init_actions.front()->kind(), "ParameterSetAction");

    DiagnosticSink late_sink;
    const Document late = load(2, late_sink);
    EXPECT_TRUE(has_deprecation(late_sink));
    EXPECT_EQ(late.scenario.init_actions.front()->kind(), "ParameterSetAction");
}

TEST(Deprecated, ReachPositionConditionExecutesAndIsReportedFromOneTwo) {
    const auto source = [](int rev_minor) {
        return document(rev_minor, std::string(kEgo) + R"(<Storyboard><Story name="s"><Act name="a">
          <ManeuverGroup name="g">
            <Actors selectTriggeringEntities="false"><EntityRef entityRef="ego"/></Actors>
            <Maneuver name="m"><Event name="e">
              <Action name="x"><PrivateAction><TeleportAction>
                <Position><WorldPosition x="0" y="0"/></Position>
              </TeleportAction></PrivateAction></Action>
              <StartTrigger><ConditionGroup><Condition name="c" delay="0" conditionEdge="none">
                <ByEntityCondition>
                  <TriggeringEntities triggeringEntitiesRule="any">
                    <EntityRef entityRef="ego"/>
                  </TriggeringEntities>
                  <EntityCondition><ReachPositionCondition tolerance="2.0">
                    <Position><WorldPosition x="100" y="0"/></Position>
                  </ReachPositionCondition></EntityCondition>
                </ByEntityCondition>
              </Condition></ConditionGroup></StartTrigger>
            </Event></Maneuver>
          </ManeuverGroup></Act></Story></Storyboard>)");
    };

    Document early;
    DiagnosticSink early_sink;
    ASSERT_EQ(scena::xml::load_string(source(1), early, early_sink), Status::Ok);
    EXPECT_FALSE(has_deprecation(early_sink));

    Document late;
    DiagnosticSink late_sink;
    ASSERT_EQ(scena::xml::load_string(source(3), late, late_sink), Status::Ok);
    EXPECT_TRUE(has_deprecation(late_sink));
}

TEST(Deprecated, TheOverwritePriorityIsASynonymOfOverride) {
    const auto source = [](int rev_minor, const char* priority) {
        return document(rev_minor, std::string(kEgo) +
                                       std::string(R"(<Storyboard><Story name="s"><Act name="a">
          <ManeuverGroup name="g">
            <Actors selectTriggeringEntities="false"><EntityRef entityRef="ego"/></Actors>
            <Maneuver name="m"><Event name="e" priority=")") +
                                       priority + R"(">
              <Action name="x"><PrivateAction><TeleportAction>
                <Position><WorldPosition x="0" y="0"/></Position>
              </TeleportAction></PrivateAction></Action>
            </Event></Maneuver>
          </ManeuverGroup></Act></Story></Storyboard>)");
    };

    Document overwrite;
    DiagnosticSink overwrite_sink;
    ASSERT_EQ(scena::xml::load_string(source(1, "overwrite"), overwrite, overwrite_sink),
              Status::Ok);
    Document override_form;
    DiagnosticSink override_sink;
    ASSERT_EQ(scena::xml::load_string(source(1, "override"), override_form, override_sink),
              Status::Ok);

    const auto priority_of = [](const Document& document_under_test) {
        return document_under_test.scenario.storyboard.stories.at(0)
            .acts.at(0)
            .groups.at(0)
            .maneuvers.at(0)
            .events.at(0)
            .priority;
    };
    // One value in the IR, not two: the standard's own descriptions of the
    // two literals are word for word identical (ADR-0005).
    EXPECT_EQ(priority_of(overwrite), priority_of(override_form));
    EXPECT_FALSE(has_deprecation(overwrite_sink)); // 1.1 knows no successor

    Document late;
    DiagnosticSink late_sink;
    ASSERT_EQ(scena::xml::load_string(source(3, "overwrite"), late, late_sink), Status::Ok);
    EXPECT_TRUE(has_deprecation(late_sink));
}

TEST(Deprecated, TheOldGeoPositionAttributeNamesStillLoad) {
    const auto source = [](int rev_minor) {
        return with_private_action(rev_minor, R"(<TeleportAction><Position>
          <GeoPosition latitude="48.1" longitude="11.5" height="520"/>
        </Position></TeleportAction>)");
    };
    Document early;
    DiagnosticSink early_sink;
    ASSERT_EQ(scena::xml::load_string(source(0), early, early_sink), Status::Ok);
    EXPECT_FALSE(has_deprecation(early_sink));

    Document late;
    DiagnosticSink late_sink;
    ASSERT_EQ(scena::xml::load_string(source(2), late, late_sink), Status::Ok);
    EXPECT_TRUE(has_deprecation(late_sink));

    const auto* teleport =
        dynamic_cast<const scena::ir::TeleportAction*>(early.scenario.storyboard.stories.at(0)
                                                           .acts.at(0)
                                                           .groups.at(0)
                                                           .maneuvers.at(0)
                                                           .events.at(0)
                                                           .actions.front()
                                                           .get());
    ASSERT_NE(teleport, nullptr);
    const auto* geo = std::get_if<scena::ir::GeoPosition>(&teleport->position());
    ASSERT_NE(geo, nullptr);
    EXPECT_DOUBLE_EQ(geo->latitude_deg, 48.1);
    EXPECT_DOUBLE_EQ(geo->altitude, 520.0);
}

TEST(Deprecated, EveryDeprecationLeavesTheDocumentLoadable) {
    // The contract behind "accepted, deprecated": a warning, never an error,
    // and the construct still in the IR.
    constexpr std::string_view kAction =
        R"(<ActivateControllerAction lateral="true" longitudinal="true"/>)";
    Document document_under_test;
    DiagnosticSink sink;
    ASSERT_EQ(scena::xml::load_string(with_private_action(3, kAction), document_under_test, sink),
              Status::Ok);
    for (const Diagnostic& diagnostic : sink.diagnostics()) {
        if (diagnostic.code == Status::DeprecatedFeature) {
            EXPECT_EQ(diagnostic.severity, Severity::Warning);
        }
    }
}

} // namespace
