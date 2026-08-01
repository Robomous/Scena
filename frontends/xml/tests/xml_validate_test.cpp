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

// File-level validation: the defects that can only be seen once the whole
// document has been read. Every rule gets a red fixture and a green one.

#include <string>
#include <string_view>

#include <gtest/gtest.h>

#include "scena/diagnostic.h"
#include "scena/xml/loader.h"

namespace {

using scena::Diagnostic;
using scena::DiagnosticSink;
using scena::Severity;
using scena::Status;
using scena::xml::Document;

std::string document(std::string_view body) {
    return std::string(R"(<OpenSCENARIO><FileHeader revMajor="1" revMinor="2"
        date="2026-08-01T00:00:00" description="validation fixture" author="Scena"/>)") +
           std::string(body) + "</OpenSCENARIO>";
}

/// One vehicle named `ego`, so a reference has something valid to point at.
constexpr std::string_view kEntities = R"(<Entities>
  <ScenarioObject name="ego">
    <Vehicle name="v" vehicleCategory="car">
      <BoundingBox><Center x="0" y="0" z="0"/><Dimensions width="2" length="4" height="1.5"/></BoundingBox>
      <Performance maxSpeed="60" maxAcceleration="5" maxDeceleration="9"/>
      <Axles><RearAxle maxSteering="0" wheelDiameter="0.6" trackWidth="1.7" positionX="0" positionZ="0.3"/></Axles>
    </Vehicle>
  </ScenarioObject>
</Entities>)";

/// A storyboard with one event whose action and trigger the caller supplies.
std::string storyboard_with(std::string_view action, std::string_view trigger = "",
                            std::string_view actor = "ego") {
    std::string body = std::string(R"(<Storyboard><Story name="s"><Act name="a">
      <ManeuverGroup name="g">
        <Actors selectTriggeringEntities="false"><EntityRef entityRef=")") +
                       std::string(actor) + R"("/></Actors>
        <Maneuver name="m"><Event name="e">
          <Action name="x"><PrivateAction>)" +
                       std::string(action) + R"(</PrivateAction></Action>)";
    if (!trigger.empty()) {
        body +=
            R"(<StartTrigger><ConditionGroup><Condition name="c" delay="0" conditionEdge="none">)";
        body += std::string(trigger);
        body += R"(</Condition></ConditionGroup></StartTrigger>)";
    }
    body += R"(</Event></Maneuver></ManeuverGroup></Act></Story></Storyboard>)";
    return body;
}

constexpr std::string_view kTeleport =
    R"(<TeleportAction><Position><WorldPosition x="0" y="0"/></Position></TeleportAction>)";

bool has_rule_id(const DiagnosticSink& sink, std::string_view rule_id) {
    for (const Diagnostic& diagnostic : sink.diagnostics()) {
        if (diagnostic.rule_id == rule_id) {
            return true;
        }
    }
    return false;
}

bool has_message_containing(const DiagnosticSink& sink, std::string_view needle) {
    for (const Diagnostic& diagnostic : sink.diagnostics()) {
        if (diagnostic.message.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

// --- the API --------------------------------------------------------------

TEST(Validate, ASoundDocumentPassesWithNoErrors) {
    Document document_under_test;
    DiagnosticSink sink;
    ASSERT_EQ(
        scena::xml::validate_string(document(std::string(kEntities) + storyboard_with(kTeleport)),
                                    document_under_test, sink),
        Status::Ok);
    for (const Diagnostic& diagnostic : sink.diagnostics()) {
        EXPECT_NE(diagnostic.severity, Severity::Error);
    }
}

TEST(Validate, ValidationRunsAsPartOfLoading) {
    // validate_string is load_string plus a name: a caller cannot forget the
    // check, so a broken reference fails the plain load too.
    const std::string source =
        document(std::string(kEntities) + storyboard_with(kTeleport, "", "ghost"));
    Document loaded;
    DiagnosticSink load_sink;
    const Status load_status = scena::xml::load_string(source, loaded, load_sink);
    Document validated;
    DiagnosticSink validate_sink;
    const Status validate_status = scena::xml::validate_string(source, validated, validate_sink);
    EXPECT_EQ(load_status, validate_status);
    EXPECT_EQ(load_sink.diagnostics().size(), validate_sink.diagnostics().size());
}

// --- referential integrity ------------------------------------------------

TEST(Validate, AnUndeclaredActorIsReported) {
    Document document_under_test;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::validate_string(
                  document(std::string(kEntities) + storyboard_with(kTeleport, "", "ghost")),
                  document_under_test, sink),
              Status::SemanticError);
    EXPECT_TRUE(
        has_rule_id(sink, "asam.net:xosc:1.2.0:reference_control.references_to_scenario_object"));
}

TEST(Validate, AnUndeclaredEntityInAnActionIsReported) {
    constexpr std::string_view kAction = R"(<LongitudinalAction>
      <LongitudinalDistanceAction entityRef="ghost" distance="10" freespace="false" continuous="true"/>
    </LongitudinalAction>)";
    Document document_under_test;
    DiagnosticSink sink;
    EXPECT_EQ(
        scena::xml::validate_string(document(std::string(kEntities) + storyboard_with(kAction)),
                                    document_under_test, sink),
        Status::SemanticError);
    EXPECT_TRUE(has_message_containing(sink, "reference to entity 'ghost'"));
}

TEST(Validate, AnUndeclaredEntityInAConditionIsReported) {
    constexpr std::string_view kTrigger = R"(<ByEntityCondition>
      <TriggeringEntities triggeringEntitiesRule="any"><EntityRef entityRef="ghost"/></TriggeringEntities>
      <EntityCondition><SpeedCondition value="10" rule="greaterThan"/></EntityCondition>
    </ByEntityCondition>)";
    Document document_under_test;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::validate_string(
                  document(std::string(kEntities) + storyboard_with(kTeleport, kTrigger)),
                  document_under_test, sink),
              Status::SemanticError);
    EXPECT_TRUE(
        has_rule_id(sink, "asam.net:xosc:1.2.0:reference_control.references_to_scenario_object"));
}

TEST(Validate, AnEntitySelectionIsAValidReference) {
    constexpr std::string_view kEntitiesWithSelection = R"(<Entities>
      <ScenarioObject name="ego">
        <Vehicle name="v" vehicleCategory="car">
          <BoundingBox><Center x="0" y="0" z="0"/><Dimensions width="2" length="4" height="1.5"/></BoundingBox>
          <Performance maxSpeed="60" maxAcceleration="5" maxDeceleration="9"/>
          <Axles><RearAxle maxSteering="0" wheelDiameter="0.6" trackWidth="1.7" positionX="0" positionZ="0.3"/></Axles>
        </Vehicle>
      </ScenarioObject>
      <EntitySelection name="all_vehicles">
        <Members><ByType objectType="vehicle"/></Members>
      </EntitySelection>
    </Entities>)";
    Document document_under_test;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::validate_string(document(std::string(kEntitiesWithSelection) +
                                                   storyboard_with(kTeleport, "", "all_vehicles")),
                                          document_under_test, sink),
              Status::Ok);
}

TEST(Validate, AnUnresolvableStoryboardElementReferenceIsReported) {
    constexpr std::string_view kTrigger = R"(<ByValueCondition>
      <StoryboardElementStateCondition storyboardElementType="event"
        storyboardElementRef="no_such_event" state="completeState"/>
    </ByValueCondition>)";
    Document document_under_test;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::validate_string(
                  document(std::string(kEntities) + storyboard_with(kTeleport, kTrigger)),
                  document_under_test, sink),
              Status::SemanticError);
    EXPECT_TRUE(has_rule_id(
        sink, "asam.net:xosc:1.0.0:reference_control.resolvable_storyboard_element_ref"));
}

TEST(Validate, AStoryboardElementOfTheWrongTypeIsReported) {
    // "e" exists, but it is an Event, not a Maneuver: the condition would
    // wait forever for a state that element never reports.
    constexpr std::string_view kTrigger = R"(<ByValueCondition>
      <StoryboardElementStateCondition storyboardElementType="maneuver"
        storyboardElementRef="e" state="completeState"/>
    </ByValueCondition>)";
    Document document_under_test;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::validate_string(
                  document(std::string(kEntities) + storyboard_with(kTeleport, kTrigger)),
                  document_under_test, sink),
              Status::SemanticError);
    EXPECT_TRUE(has_message_containing(sink, "is not of the type the condition claims"));
}

TEST(Validate, AnUndeclaredVariableIsReported) {
    constexpr std::string_view kTrigger =
        R"(<ByValueCondition><VariableCondition variableRef="ghost" value="1" rule="equalTo"/></ByValueCondition>)";
    Document document_under_test;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::validate_string(
                  document(std::string(kEntities) + storyboard_with(kTeleport, kTrigger)),
                  document_under_test, sink),
              Status::SemanticError);
    EXPECT_TRUE(
        has_rule_id(sink, "asam.net:xosc:1.2.0:reference_control.resolvable_variable_reference"));
}

TEST(Validate, AnUndeclaredTrafficSignalControllerIsReported) {
    constexpr std::string_view kTrigger = R"(<ByValueCondition>
      <TrafficSignalControllerCondition trafficSignalControllerRef="ghost" phase="green"/>
    </ByValueCondition>)";
    Document document_under_test;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::validate_string(
                  document(std::string(kEntities) + storyboard_with(kTeleport, kTrigger)),
                  document_under_test, sink),
              Status::SemanticError);
    EXPECT_TRUE(has_rule_id(
        sink, "asam.net:xosc:1.0.0:reference_control.traffic_signal_controller_references"));
}

// --- naming ---------------------------------------------------------------

TEST(Validate, DuplicateSiblingNamesAreReported) {
    constexpr std::string_view kBody = R"(<Storyboard><Story name="s"><Act name="a">
      <ManeuverGroup name="g">
        <Actors selectTriggeringEntities="false"><EntityRef entityRef="ego"/></Actors>
        <Maneuver name="m">
          <Event name="e"><Action name="x"><PrivateAction><TeleportAction>
            <Position><WorldPosition x="0" y="0"/></Position>
          </TeleportAction></PrivateAction></Action></Event>
          <Event name="e"><Action name="y"><PrivateAction><TeleportAction>
            <Position><WorldPosition x="1" y="0"/></Position>
          </TeleportAction></PrivateAction></Action></Event>
        </Maneuver>
      </ManeuverGroup></Act></Story></Storyboard>)";
    Document document_under_test;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::validate_string(document(std::string(kEntities) + std::string(kBody)),
                                          document_under_test, sink),
              Status::ValidationError);
    EXPECT_TRUE(has_rule_id(sink, "asam.net:xosc:1.1.0:naming.unique_element_names_on_same_level"));
}

TEST(Validate, DuplicateEntityNamesAreReported) {
    constexpr std::string_view kDuplicated = R"(<Entities>
      <ScenarioObject name="ego">
        <Vehicle name="v" vehicleCategory="car">
          <BoundingBox><Center x="0" y="0" z="0"/><Dimensions width="2" length="4" height="1.5"/></BoundingBox>
          <Performance maxSpeed="60" maxAcceleration="5" maxDeceleration="9"/>
          <Axles><RearAxle maxSteering="0" wheelDiameter="0.6" trackWidth="1.7" positionX="0" positionZ="0.3"/></Axles>
        </Vehicle>
      </ScenarioObject>
      <ScenarioObject name="ego">
        <Vehicle name="v2" vehicleCategory="car">
          <BoundingBox><Center x="0" y="0" z="0"/><Dimensions width="2" length="4" height="1.5"/></BoundingBox>
          <Performance maxSpeed="60" maxAcceleration="5" maxDeceleration="9"/>
          <Axles><RearAxle maxSteering="0" wheelDiameter="0.6" trackWidth="1.7" positionX="0" positionZ="0.3"/></Axles>
        </Vehicle>
      </ScenarioObject>
    </Entities>)";
    Document document_under_test;
    DiagnosticSink sink;
    EXPECT_EQ(
        scena::xml::validate_string(document(std::string(kDuplicated) + storyboard_with(kTeleport)),
                                    document_under_test, sink),
        Status::ValidationError);
    EXPECT_TRUE(has_message_containing(sink, "entity name 'ego' is used more than once"));
}

// --- unused declarations --------------------------------------------------

TEST(Validate, AnUnreferencedParameterIsAWarning) {
    const std::string source = document(R"(<ParameterDeclarations>
        <ParameterDeclaration name="unused" parameterType="double" value="1.0"/>
      </ParameterDeclarations>)" + std::string(kEntities) +
                                        storyboard_with(kTeleport));
    Document document_under_test;
    DiagnosticSink sink;
    // Harmless to execute, so a warning — but almost always a typo in the
    // reference rather than a deliberate spare.
    EXPECT_EQ(scena::xml::validate_string(source, document_under_test, sink), Status::Ok);
    EXPECT_TRUE(
        has_message_containing(sink, "parameter 'unused' is declared but never referenced"));
}

TEST(Validate, AReferencedParameterIsNotWarnedAbout) {
    const std::string source = document(R"(<ParameterDeclarations>
        <ParameterDeclaration name="x0" parameterType="double" value="12.0"/>
      </ParameterDeclarations>)" + std::string(kEntities) +
                                        storyboard_with(R"(<TeleportAction>
          <Position><WorldPosition x="$x0" y="0"/></Position>
        </TeleportAction>)"));
    Document document_under_test;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::validate_string(source, document_under_test, sink), Status::Ok);
    EXPECT_FALSE(has_message_containing(sink, "declared but never referenced"));
}

TEST(Validate, AnUnreferencedVariableIsAWarning) {
    const std::string source = document(R"(<VariableDeclarations>
        <VariableDeclaration name="spare" variableType="int" value="0"/>
      </VariableDeclarations>)" + std::string(kEntities) +
                                        storyboard_with(kTeleport));
    Document document_under_test;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::validate_string(source, document_under_test, sink), Status::Ok);
    EXPECT_TRUE(has_message_containing(sink, "variable 'spare' is declared but never referenced"));
}

TEST(Validate, AVariableReferencedByAConditionIsNotWarnedAbout) {
    const std::string source = document(
        R"(<VariableDeclarations>
        <VariableDeclaration name="armed" variableType="string" value="no"/>
      </VariableDeclarations>)" +
        std::string(kEntities) +
        storyboard_with(
            kTeleport,
            R"(<ByValueCondition><VariableCondition variableRef="armed" value="yes" rule="equalTo"/></ByValueCondition>)"));
    Document document_under_test;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::validate_string(source, document_under_test, sink), Status::Ok);
    EXPECT_FALSE(has_message_containing(sink, "declared but never referenced"));
}

} // namespace
