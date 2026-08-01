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

// The ByValueCondition family end to end: every member lowers with its
// payload intact, the rule literal set maps and is gated on the document's
// revision, and the dateTime attribute parses locale-independently across
// the ISO 8601 forms the standard admits.

#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

#include "scena/diagnostic.h"
#include "scena/engine.h"
#include "scena/ir/condition.h"
#include "scena/ir/date_time.h"
#include "scena/ir/evaluation_context.h"
#include "scena/ir/rule.h"
#include "scena/xml/loader.h"

namespace {

using scena::Diagnostic;
using scena::DiagnosticSink;
using scena::Severity;
using scena::Status;
using scena::xml::Document;

/// Wraps one ByValueCondition in the smallest scenario that carries a
/// trigger, at the requested revision.
std::string document_with(std::string_view condition, int rev_minor = 2) {
    return std::string("<OpenSCENARIO><FileHeader revMajor=\"1\" revMinor=\"") +
           std::to_string(rev_minor) +
           "\" date=\"2026-08-01T00:00:00\" description=\"fixture\" author=\"Scena\"/>"
           R"(<Storyboard><Story name="s"><Act name="a"><ManeuverGroup name="g">
             <Actors selectTriggeringEntities="false"><EntityRef entityRef="ego"/></Actors>
             <Maneuver name="m"><Event name="e">
               <Action name="x"><PrivateAction><TeleportAction>
                 <Position><WorldPosition x="0" y="0"/></Position>
               </TeleportAction></PrivateAction></Action>
               <StartTrigger><ConditionGroup><Condition name="c" delay="0" conditionEdge="none">
                 <ByValueCondition>)" +
           std::string(condition) +
           R"(</ByValueCondition>
               </Condition></ConditionGroup></StartTrigger>
             </Event></Maneuver></ManeuverGroup></Act></Story></Storyboard></OpenSCENARIO>)";
}

/// Loads one ByValueCondition and returns the lowered logical expression.
std::shared_ptr<scena::ir::Condition> load(std::string_view condition, DiagnosticSink& sink,
                                           Status& status, int rev_minor = 2) {
    Document document;
    status = scena::xml::load_string(document_with(condition, rev_minor), document, sink);
    const auto& stories = document.scenario.storyboard.stories;
    if (stories.empty()) {
        return nullptr;
    }
    const auto& event = stories.at(0).acts.at(0).groups.at(0).maneuvers.at(0).events.at(0);
    if (!event.start_trigger.has_value() || event.start_trigger->groups.empty()) {
        return nullptr;
    }
    return event.start_trigger->groups.front().conditions.front().expression;
}

bool has_rule_id(const DiagnosticSink& sink, std::string_view rule_id) {
    for (const Diagnostic& diagnostic : sink.diagnostics()) {
        if (diagnostic.rule_id == rule_id) {
            return true;
        }
    }
    return false;
}

// --- the six members ------------------------------------------------------

TEST(ByValue, SimulationTimeConditionCarriesValueAndRule) {
    DiagnosticSink sink;
    Status status = Status::Ok;
    const auto condition =
        load(R"(<SimulationTimeCondition value="12.5" rule="greaterOrEqual"/>)", sink, status);
    ASSERT_NE(condition, nullptr);
    EXPECT_EQ(status, Status::Ok);
    const auto* typed = dynamic_cast<const scena::ir::SimulationTimeCondition*>(condition.get());
    ASSERT_NE(typed, nullptr);
    EXPECT_DOUBLE_EQ(typed->value(), 12.5);
    EXPECT_EQ(typed->rule(), scena::ir::Rule::GreaterOrEqual);
}

TEST(ByValue, ParameterAndVariableConditionsCarryTheirReference) {
    DiagnosticSink sink;
    Status status = Status::Ok;
    const auto parameter = load(
        R"(<ParameterCondition parameterRef="speed" value="30" rule="equalTo"/>)", sink, status);
    ASSERT_NE(parameter, nullptr);
    const auto* typed_parameter =
        dynamic_cast<const scena::ir::ParameterCondition*>(parameter.get());
    ASSERT_NE(typed_parameter, nullptr);
    EXPECT_EQ(typed_parameter->parameter_ref(), "speed");
    EXPECT_EQ(typed_parameter->value(), "30");
    EXPECT_EQ(typed_parameter->rule(), scena::ir::Rule::EqualTo);

    DiagnosticSink variable_sink;
    const auto variable =
        load(R"(<VariableCondition variableRef="phase" value="two" rule="notEqualTo"/>)",
             variable_sink, status);
    ASSERT_NE(variable, nullptr);
    const auto* typed_variable = dynamic_cast<const scena::ir::VariableCondition*>(variable.get());
    ASSERT_NE(typed_variable, nullptr);
    EXPECT_EQ(typed_variable->variable_ref(), "phase");
    EXPECT_EQ(typed_variable->value(), "two");
    EXPECT_EQ(typed_variable->rule(), scena::ir::Rule::NotEqualTo);
}

TEST(ByValue, UserDefinedValueConditionCarriesItsName) {
    DiagnosticSink sink;
    Status status = Status::Ok;
    const auto condition = load(
        R"(<UserDefinedValueCondition name="host.gear" value="3" rule="equalTo"/>)", sink, status);
    ASSERT_NE(condition, nullptr);
    const auto* typed = dynamic_cast<const scena::ir::UserDefinedValueCondition*>(condition.get());
    ASSERT_NE(typed, nullptr);
    EXPECT_EQ(typed->name(), "host.gear");
    EXPECT_EQ(typed->value(), "3");
}

TEST(ByValue, StoryboardElementStateConditionCarriesTypeRefAndState) {
    DiagnosticSink sink;
    Status status = Status::Ok;
    const auto condition = load(
        R"(<StoryboardElementStateCondition storyboardElementType="maneuverGroup"
             storyboardElementRef="group_1" state="startTransition"/>)",
        sink, status);
    ASSERT_NE(condition, nullptr);
    const auto* typed =
        dynamic_cast<const scena::ir::StoryboardElementStateCondition*>(condition.get());
    ASSERT_NE(typed, nullptr);
    EXPECT_EQ(typed->element_type(), scena::ir::StoryboardElementType::ManeuverGroup);
    EXPECT_EQ(typed->element_ref(), "group_1");
    EXPECT_EQ(typed->state(), scena::ir::StoryboardElementState::StartTransition);
}

TEST(ByValue, TrafficSignalConditionsCarryTheirReferences) {
    DiagnosticSink sink;
    Status status = Status::Ok;
    const auto signal =
        load(R"(<TrafficSignalCondition name="s1" state="off;off;on"/>)", sink, status);
    ASSERT_NE(signal, nullptr);
    const auto* typed_signal = dynamic_cast<const scena::ir::TrafficSignalCondition*>(signal.get());
    ASSERT_NE(typed_signal, nullptr);
    EXPECT_EQ(typed_signal->name(), "s1");
    EXPECT_EQ(typed_signal->state(), "off;off;on");

    DiagnosticSink controller_sink;
    const auto controller = load(
        R"(<TrafficSignalControllerCondition trafficSignalControllerRef="ctrl" phase="green"/>)",
        controller_sink, status);
    ASSERT_NE(controller, nullptr);
    const auto* typed_controller =
        dynamic_cast<const scena::ir::TrafficSignalControllerCondition*>(controller.get());
    ASSERT_NE(typed_controller, nullptr);
    EXPECT_EQ(typed_controller->traffic_signal_controller_ref(), "ctrl");
    EXPECT_EQ(typed_controller->phase(), "green");
}

// --- the rule attribute ---------------------------------------------------

TEST(Rule, EveryLiteralMaps) {
    const struct {
        std::string_view literal;
        scena::ir::Rule rule;
    } cases[] = {
        {"equalTo", scena::ir::Rule::EqualTo},
        {"greaterThan", scena::ir::Rule::GreaterThan},
        {"lessThan", scena::ir::Rule::LessThan},
        {"greaterOrEqual", scena::ir::Rule::GreaterOrEqual},
        {"lessOrEqual", scena::ir::Rule::LessOrEqual},
        {"notEqualTo", scena::ir::Rule::NotEqualTo},
    };
    for (const auto& test_case : cases) {
        DiagnosticSink sink;
        Status status = Status::Ok;
        const std::string condition = std::string(R"(<SimulationTimeCondition value="1" rule=")") +
                                      std::string(test_case.literal) + R"("/>)";
        const auto lowered = load(condition, sink, status);
        ASSERT_NE(lowered, nullptr) << test_case.literal;
        EXPECT_EQ(status, Status::Ok) << test_case.literal;
        const auto* typed = dynamic_cast<const scena::ir::SimulationTimeCondition*>(lowered.get());
        ASSERT_NE(typed, nullptr);
        EXPECT_EQ(typed->rule(), test_case.rule) << test_case.literal;
    }
}

TEST(Rule, AnUnknownLiteralIsAValidationError) {
    DiagnosticSink sink;
    Status status = Status::Ok;
    EXPECT_EQ(load(R"(<SimulationTimeCondition value="1" rule="approximately"/>)", sink, status),
              nullptr);
    EXPECT_EQ(status, Status::ValidationError);
}

TEST(Rule, NewerLiteralsInAnOlderDocumentAreWarnedNotRejected) {
    for (const char* literal : {"greaterOrEqual", "lessOrEqual", "notEqualTo"}) {
        DiagnosticSink sink;
        Status status = Status::Ok;
        const std::string condition =
            std::string(R"(<SimulationTimeCondition value="1" rule=")") + literal + R"("/>)";
        // A 1.0 document using a later operator: loaded with its meaning
        // intact, and told. The introduction versions cannot be confirmed
        // from the local reference text (OQ-4), so this must not reject.
        const auto lowered = load(condition, sink, status, /*rev_minor=*/0);
        ASSERT_NE(lowered, nullptr) << literal;
        EXPECT_EQ(status, Status::Ok) << literal;
        bool warned = false;
        for (const Diagnostic& diagnostic : sink.diagnostics()) {
            warned = warned || (diagnostic.severity == Severity::Warning &&
                                diagnostic.message.find(literal) != std::string::npos);
        }
        EXPECT_TRUE(warned) << literal;
    }
}

TEST(Rule, TheOnePointZeroLiteralsAreNeverWarnedAbout) {
    for (const char* literal : {"equalTo", "greaterThan", "lessThan"}) {
        DiagnosticSink sink;
        Status status = Status::Ok;
        const std::string condition =
            std::string(R"(<SimulationTimeCondition value="1" rule=")") + literal + R"("/>)";
        ASSERT_NE(load(condition, sink, status, /*rev_minor=*/0), nullptr) << literal;
        EXPECT_TRUE(sink.diagnostics().empty()) << literal;
    }
}

// --- the dateTime attribute ----------------------------------------------

/// The DateTime a TimeOfDayCondition lowered, for the parsing matrix.
scena::ir::DateTime parsed_date_time(std::string_view text, DiagnosticSink& sink, Status& status) {
    const std::string condition = std::string(R"(<TimeOfDayCondition dateTime=")") +
                                  std::string(text) + R"(" rule="greaterThan"/>)";
    const auto lowered = load(condition, sink, status);
    if (lowered == nullptr) {
        return {};
    }
    const auto* typed = dynamic_cast<const scena::ir::TimeOfDayCondition*>(lowered.get());
    return typed == nullptr ? scena::ir::DateTime{} : typed->date_time();
}

TEST(DateTime, BasicNotationParses) {
    DiagnosticSink sink;
    Status status = Status::Ok;
    const scena::ir::DateTime value = parsed_date_time("2026-08-01T13:45:07", sink, status);
    EXPECT_EQ(status, Status::Ok);
    EXPECT_EQ(value.year, 2026);
    EXPECT_EQ(value.month, 8);
    EXPECT_EQ(value.day, 1);
    EXPECT_EQ(value.hour, 13);
    EXPECT_EQ(value.minute, 45);
    EXPECT_EQ(value.second, 7);
    EXPECT_EQ(value.millisecond, 0);
    EXPECT_EQ(value.utc_offset_minutes, 0);
}

TEST(DateTime, FractionalSecondsScaleToMilliseconds) {
    const struct {
        std::string_view text;
        int milliseconds;
    } cases[] = {
        {"2026-08-01T00:00:00.5", 500},
        {"2026-08-01T00:00:00.25", 250},
        {"2026-08-01T00:00:00.125", 125},
        // More than three digits: the standard's pattern writes milliseconds,
        // so the rest is truncated rather than rounded.
        {"2026-08-01T00:00:00.1259", 125},
    };
    for (const auto& test_case : cases) {
        DiagnosticSink sink;
        Status status = Status::Ok;
        const scena::ir::DateTime value = parsed_date_time(test_case.text, sink, status);
        EXPECT_EQ(status, Status::Ok) << test_case.text;
        EXPECT_EQ(value.millisecond, test_case.milliseconds) << test_case.text;
    }
}

TEST(DateTime, ZoneDesignatorsParse) {
    const struct {
        std::string_view text;
        int offset_minutes;
    } cases[] = {
        {"2026-08-01T00:00:00Z", 0},           {"2026-08-01T00:00:00+02:00", 120},
        {"2026-08-01T00:00:00-05:30", -330},   {"2026-08-01T00:00:00+0200", 120},
        {"2026-08-01T00:00:00.500+01:00", 60},
    };
    for (const auto& test_case : cases) {
        DiagnosticSink sink;
        Status status = Status::Ok;
        const scena::ir::DateTime value = parsed_date_time(test_case.text, sink, status);
        EXPECT_EQ(status, Status::Ok) << test_case.text;
        EXPECT_EQ(value.utc_offset_minutes, test_case.offset_minutes) << test_case.text;
    }
}

TEST(DateTime, MalformedOrImpossibleValuesCiteTheTimeFormatRule) {
    for (const char* text : {
             "not-a-date",
             "2026-08-01",            // date only
             "2026-08-01 13:45:07",   // space instead of 'T'
             "2026-13-01T00:00:00",   // month 13
             "2026-02-30T00:00:00",   // day does not exist
             "2026-08-01T24:00:00",   // hour 24
             "2026-08-01T00:00:00.",  // '.' with no digits
             "2026-08-01T00:00:00+2", // truncated offset
         }) {
        DiagnosticSink sink;
        Status status = Status::Ok;
        const std::string condition =
            std::string(R"(<TimeOfDayCondition dateTime=")") + text + R"(" rule="greaterThan"/>)";
        EXPECT_EQ(load(condition, sink, status), nullptr) << text;
        EXPECT_EQ(status, Status::ValidationError) << text;
        EXPECT_TRUE(has_rule_id(sink, "asam.net:xosc:1.0.0:data_type.time_format")) << text;
    }
}

TEST(DateTime, LeapDayIsAccepted) {
    DiagnosticSink sink;
    Status status = Status::Ok;
    const scena::ir::DateTime value = parsed_date_time("2024-02-29T12:00:00", sink, status);
    EXPECT_EQ(status, Status::Ok);
    EXPECT_EQ(value.day, 29);
    EXPECT_TRUE(value.valid());
}

// --- through the engine ---------------------------------------------------

TEST(ByValue, ALoweredParameterConditionFiresInTheEngine) {
    // The whole point of the lowering: a condition read from XML drives the
    // storyboard exactly as a hand-built one does.
    constexpr std::string_view kSource =
        R"(<OpenSCENARIO><FileHeader revMajor="1" revMinor="2" date="2026-08-01T00:00:00"
             description="byvalue" author="Scena"/>
           <ParameterDeclarations>
             <ParameterDeclaration name="armed" parameterType="string" value="yes"/>
           </ParameterDeclarations>
           <Entities><ScenarioObject name="ego">
             <Vehicle name="v" vehicleCategory="car">
               <BoundingBox><Center x="0" y="0" z="0"/><Dimensions width="2" length="4" height="1.5"/></BoundingBox>
               <Performance maxSpeed="60" maxAcceleration="5" maxDeceleration="9"/>
               <Axles>
                 <RearAxle maxSteering="0" wheelDiameter="0.6" trackWidth="1.7" positionX="0" positionZ="0.3"/>
               </Axles>
             </Vehicle>
           </ScenarioObject></Entities>
           <Storyboard>
             <Init><Actions><Private entityRef="ego"><PrivateAction>
               <TeleportAction><Position><WorldPosition x="0" y="0"/></Position></TeleportAction>
             </PrivateAction></Private></Actions></Init>
             <Story name="s"><Act name="a"><ManeuverGroup name="g">
               <Actors selectTriggeringEntities="false"><EntityRef entityRef="ego"/></Actors>
               <Maneuver name="m"><Event name="e">
                 <Action name="go"><PrivateAction><LongitudinalAction><SpeedAction>
                   <SpeedActionDynamics dynamicsShape="step" dynamicsDimension="time" value="0"/>
                   <SpeedActionTarget><AbsoluteTargetSpeed value="15"/></SpeedActionTarget>
                 </SpeedAction></LongitudinalAction></PrivateAction></Action>
                 <StartTrigger><ConditionGroup>
                   <Condition name="armed" delay="0" conditionEdge="none">
                     <ByValueCondition><ParameterCondition parameterRef="armed" value="yes" rule="equalTo"/></ByValueCondition>
                   </Condition>
                 </ConditionGroup></StartTrigger>
               </Event></Maneuver>
             </ManeuverGroup></Act></Story>
           </Storyboard></OpenSCENARIO>)";

    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(scena::xml::load_string(kSource, document, sink), Status::Ok);
    EXPECT_EQ(document.scenario.parameters.at("armed"), "yes");

    scena::Engine engine;
    ASSERT_EQ(engine.init(document.scenario), Status::Ok);
    ASSERT_EQ(engine.step(0.1), Status::Ok);
    const std::optional<scena::EntityState> state = engine.state("ego");
    ASSERT_TRUE(state.has_value());
    EXPECT_DOUBLE_EQ(state->speed, 15.0);
    EXPECT_EQ(engine.close(), Status::Ok);
}

} // namespace
