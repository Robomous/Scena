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

// Parameters and expressions (§9.1, §9.2): declaration, typing, constraint
// groups, scope shadowing, the operator/precedence matrix, and the rule ids
// the diagnostics cite.

#include <limits>
#include <string>
#include <string_view>
#include <variant>

#include <gtest/gtest.h>

#include "scena/diagnostic.h"
#include "scena/ir/action.h"
#include "scena/ir/position.h"
#include "scena/xml/loader.h"

namespace {

using scena::Diagnostic;
using scena::DiagnosticSink;
using scena::Severity;
using scena::Status;
using scena::xml::Document;

/// A 1.2 document that declares `declarations` and then uses the resulting
/// values in a WorldPosition, whose x/y are read back by the assertions.
std::string document_with(std::string_view declarations, std::string_view x,
                          std::string_view y = "0") {
    return std::string(R"(<OpenSCENARIO><FileHeader revMajor="1" revMinor="2"
             date="2026-08-01T00:00:00" description="fixture" author="Scena"/>)") +
           std::string(declarations) +
           R"(<Storyboard><Init><Actions><Private entityRef="ego"><PrivateAction>
             <TeleportAction><Position><WorldPosition x=")" +
           std::string(x) + R"(" y=")" + std::string(y) + R"("/></Position></TeleportAction>
           </PrivateAction></Private></Actions></Init></Storyboard></OpenSCENARIO>)";
}

/// Loads the document and returns the teleport target's x, or NaN when the
/// load failed.
double loaded_x(std::string_view declarations, std::string_view x, DiagnosticSink& sink,
                Status& status) {
    Document document;
    status = scena::xml::load_string(document_with(declarations, x), document, sink);
    if (document.scenario.init_actions.empty()) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const auto* teleport = dynamic_cast<const scena::ir::TeleportAction*>(
        document.scenario.init_actions.front().get());
    if (teleport == nullptr) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    const auto* world = std::get_if<scena::ir::WorldPosition>(&teleport->position());
    return world == nullptr ? std::numeric_limits<double>::quiet_NaN() : world->x;
}

/// The value an expression evaluates to, with no declarations in scope.
double evaluated(std::string_view expression, DiagnosticSink& sink, Status& status) {
    return loaded_x("", expression, sink, status);
}

bool has_rule_id(const DiagnosticSink& sink, std::string_view rule_id) {
    for (const Diagnostic& diagnostic : sink.diagnostics()) {
        if (diagnostic.rule_id == rule_id) {
            return true;
        }
    }
    return false;
}

constexpr std::string_view kSpeedDeclaration = R"(<ParameterDeclarations>
  <ParameterDeclaration name="speed" parameterType="double" value="30.0"/>
  <ParameterDeclaration name="count" parameterType="int" value="3"/>
  <ParameterDeclaration name="armed" parameterType="boolean" value="true"/>
  <ParameterDeclaration name="label" parameterType="string" value="ego"/>
</ParameterDeclarations>)";

// --- references -----------------------------------------------------------

TEST(Parameters, AReferenceSubstitutesTheDeclaredValue) {
    DiagnosticSink sink;
    Status status = Status::Ok;
    EXPECT_DOUBLE_EQ(loaded_x(kSpeedDeclaration, "$speed", sink, status), 30.0);
    EXPECT_EQ(status, Status::Ok);
    EXPECT_TRUE(sink.diagnostics().empty());
}

TEST(Parameters, AnUndeclaredReferenceIsASemanticError) {
    DiagnosticSink sink;
    Status status = Status::Ok;
    (void)loaded_x(kSpeedDeclaration, "$missing", sink, status);
    EXPECT_EQ(status, Status::SemanticError);
    EXPECT_TRUE(
        has_rule_id(sink, "asam.net:xosc:1.1.0:parameters.parameter_declaration_parameter_scope"));
}

TEST(Parameters, ADeclarationMayBuildOnAnEarlierOne) {
    constexpr std::string_view kDeclarations = R"(<ParameterDeclarations>
      <ParameterDeclaration name="base" parameterType="double" value="10.0"/>
      <ParameterDeclaration name="derived" parameterType="double" value="${$base * 2 + 1}"/>
    </ParameterDeclarations>)";
    DiagnosticSink sink;
    Status status = Status::Ok;
    EXPECT_DOUBLE_EQ(loaded_x(kDeclarations, "$derived", sink, status), 21.0);
    EXPECT_EQ(status, Status::Ok);
}

TEST(Parameters, ValuesReachTheScenarioIr) {
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(scena::xml::load_string(document_with(kSpeedDeclaration, "0"), document, sink),
              Status::Ok);
    // §9.1: parameters are evaluated at load time and immutable afterwards,
    // so a ParameterCondition compares against these.
    EXPECT_EQ(document.scenario.parameters.at("speed"), "30");
    EXPECT_EQ(document.scenario.parameters.at("count"), "3");
    EXPECT_EQ(document.scenario.parameters.at("armed"), "true");
    EXPECT_EQ(document.scenario.parameters.at("label"), "ego");
}

TEST(Parameters, AnInnerDeclarationShadowsAnOuterOne) {
    // The Story declares its own "offset"; the global one stays visible
    // outside that subtree (§9.1: smallest scope wins).
    constexpr std::string_view kSource = R"(<OpenSCENARIO><FileHeader revMajor="1" revMinor="2"
        date="2026-08-01T00:00:00" description="f" author="Scena"/>
      <ParameterDeclarations>
        <ParameterDeclaration name="offset" parameterType="double" value="1.0"/>
      </ParameterDeclarations>
      <Storyboard>
        <Init><Actions><Private entityRef="ego"><PrivateAction><TeleportAction>
          <Position><WorldPosition x="$offset" y="0"/></Position>
        </TeleportAction></PrivateAction></Private></Actions></Init>
        <Story name="s">
          <ParameterDeclarations>
            <ParameterDeclaration name="offset" parameterType="double" value="7.0"/>
          </ParameterDeclarations>
          <Act name="a"><ManeuverGroup name="g">
            <Actors selectTriggeringEntities="false"><EntityRef entityRef="ego"/></Actors>
            <Maneuver name="m"><Event name="e"><Action name="x"><PrivateAction><TeleportAction>
              <Position><WorldPosition x="$offset" y="0"/></Position>
            </TeleportAction></PrivateAction></Action></Event></Maneuver>
          </ManeuverGroup></Act>
        </Story>
      </Storyboard></OpenSCENARIO>)";

    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(scena::xml::load_string(kSource, document, sink), Status::Ok);

    const auto* init = dynamic_cast<const scena::ir::TeleportAction*>(
        document.scenario.init_actions.front().get());
    ASSERT_NE(init, nullptr);
    EXPECT_DOUBLE_EQ(std::get<scena::ir::WorldPosition>(init->position()).x, 1.0);

    const auto& event_action = document.scenario.storyboard.stories.at(0)
                                   .acts.at(0)
                                   .groups.at(0)
                                   .maneuvers.at(0)
                                   .events.at(0)
                                   .actions.front();
    const auto* inner = dynamic_cast<const scena::ir::TeleportAction*>(event_action.get());
    ASSERT_NE(inner, nullptr);
    EXPECT_DOUBLE_EQ(std::get<scena::ir::WorldPosition>(inner->position()).x, 7.0);
}

TEST(Parameters, AStoryDeclarationIsNotVisibleOutsideItsSubtree) {
    constexpr std::string_view kSource = R"(<OpenSCENARIO><FileHeader revMajor="1" revMinor="2"
        date="2026-08-01T00:00:00" description="f" author="Scena"/>
      <Storyboard>
        <Story name="s">
          <ParameterDeclarations>
            <ParameterDeclaration name="local" parameterType="double" value="7.0"/>
          </ParameterDeclarations>
          <Act name="a"><ManeuverGroup name="g">
            <Actors selectTriggeringEntities="false"><EntityRef entityRef="ego"/></Actors>
            <Maneuver name="m"><Event name="e"><Action name="x"><PrivateAction><TeleportAction>
              <Position><WorldPosition x="0" y="0"/></Position>
            </TeleportAction></PrivateAction></Action></Event></Maneuver>
          </ManeuverGroup></Act>
        </Story>
        <Story name="other">
          <Act name="a"><ManeuverGroup name="g">
            <Actors selectTriggeringEntities="false"><EntityRef entityRef="ego"/></Actors>
            <Maneuver name="m"><Event name="e"><Action name="x"><PrivateAction><TeleportAction>
              <Position><WorldPosition x="$local" y="0"/></Position>
            </TeleportAction></PrivateAction></Action></Event></Maneuver>
          </ManeuverGroup></Act>
        </Story>
      </Storyboard></OpenSCENARIO>)";
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::load_string(kSource, document, sink), Status::SemanticError);
}

TEST(Parameters, NameSyntaxAndReservedPrefixAreChecked) {
    DiagnosticSink bad_name;
    Status status = Status::Ok;
    (void)loaded_x(R"(<ParameterDeclarations>
        <ParameterDeclaration name="2fast" parameterType="double" value="1"/>
      </ParameterDeclarations>)",
                   "0", bad_name, status);
    EXPECT_EQ(status, Status::ValidationError);
    EXPECT_TRUE(
        has_rule_id(bad_name, "asam.net:xosc:1.1.0:naming.parameter_declaration_parameter_name"));

    DiagnosticSink reserved;
    (void)loaded_x(R"(<ParameterDeclarations>
        <ParameterDeclaration name="OSCthing" parameterType="double" value="1"/>
      </ParameterDeclarations>)",
                   "0", reserved, status);
    EXPECT_EQ(status, Status::Ok); // a "should", so a warning
    EXPECT_TRUE(has_rule_id(
        reserved, "asam.net:xosc:1.0.0:naming.parameter_declaration_name_prefix_reserved"));
}

TEST(Parameters, AValueOfTheWrongTypeCitesTypeInference) {
    DiagnosticSink sink;
    Status status = Status::Ok;
    (void)loaded_x(R"(<ParameterDeclarations>
        <ParameterDeclaration name="count" parameterType="int" value="1.5"/>
      </ParameterDeclarations>)",
                   "0", sink, status);
    EXPECT_EQ(status, Status::ValidationError);
    EXPECT_TRUE(has_rule_id(
        sink, "asam.net:xosc:1.0.0:parameters.parameter_declaration_parameter_type_inference"));
}

TEST(Parameters, ConstraintGroupsAreEnforced) {
    constexpr std::string_view kSatisfied = R"(<ParameterDeclarations>
      <ParameterDeclaration name="speed" parameterType="double" value="30.0">
        <ConstraintGroup>
          <ValueConstraint rule="greaterThan" value="0"/>
          <ValueConstraint rule="lessOrEqual" value="50"/>
        </ConstraintGroup>
      </ParameterDeclaration>
    </ParameterDeclarations>)";
    DiagnosticSink satisfied_sink;
    Status status = Status::Ok;
    EXPECT_DOUBLE_EQ(loaded_x(kSatisfied, "$speed", satisfied_sink, status), 30.0);
    EXPECT_EQ(status, Status::Ok);

    constexpr std::string_view kViolated = R"(<ParameterDeclarations>
      <ParameterDeclaration name="speed" parameterType="double" value="80.0">
        <ConstraintGroup>
          <ValueConstraint rule="lessOrEqual" value="50"/>
        </ConstraintGroup>
      </ParameterDeclaration>
    </ParameterDeclarations>)";
    DiagnosticSink violated_sink;
    (void)loaded_x(kViolated, "0", violated_sink, status);
    EXPECT_EQ(status, Status::ValidationError);

    // Groups are alternatives: satisfying one is enough.
    constexpr std::string_view kAlternatives = R"(<ParameterDeclarations>
      <ParameterDeclaration name="speed" parameterType="double" value="80.0">
        <ConstraintGroup><ValueConstraint rule="lessOrEqual" value="50"/></ConstraintGroup>
        <ConstraintGroup><ValueConstraint rule="greaterThan" value="70"/></ConstraintGroup>
      </ParameterDeclaration>
    </ParameterDeclarations>)";
    DiagnosticSink alternatives_sink;
    EXPECT_DOUBLE_EQ(loaded_x(kAlternatives, "$speed", alternatives_sink, status), 80.0);
    EXPECT_EQ(status, Status::Ok);
}

// --- the expression matrix ------------------------------------------------

TEST(Expressions, ArithmeticAndPrecedence) {
    const struct {
        std::string_view expression;
        double expected;
    } cases[] = {
        {"${1 + 2}", 3.0},
        {"${1 + 2 * 3}", 7.0},         // multiplication binds tighter
        {"${(1 + 2) * 3}", 9.0},       // brackets override
        {"${10 - 4 - 3}", 3.0},        // left associative
        {"${7 / 2}", 3.5},             // "/" is the double division
        {"${7 % 3}", 1.0},             // remainder
        {"${-7 % 3}", -1.0},           // remainder, not modulo
        {"${-3 + 1}", -2.0},           // unary minus
        {"${1 + sqrt(9) * 2.5}", 8.5}, // the spec's own example shape
        {"${pow(2, 8) - 1}", 255.0},   // §9.2.1 example
        {"${-round(2.6)}", -3.0},      // §9.2.1 example
        {"${floor(2.9)}", 2.0},
        {"${ceil(2.1)}", 3.0},
        {"${abs(-4)}", 4.0},
        {"${sign(-2.5)}", -1.0},
        {"${max(3, 7)}", 7.0},
        {"${min(3, 7)}", 3.0},
        {"${pow(2, -2)}", 0.25},
    };
    for (const auto& test_case : cases) {
        DiagnosticSink sink;
        Status status = Status::Ok;
        EXPECT_DOUBLE_EQ(evaluated(test_case.expression, sink, status), test_case.expected)
            << test_case.expression;
        EXPECT_EQ(status, Status::Ok) << test_case.expression;
    }
}

TEST(Expressions, TrigonometryIsDeterministic) {
    // The values come from the deterministic math layer, so they are
    // bit-identical on every platform; asserting exact equality between two
    // loads is the property that matters here.
    for (const char* expression : {"${sin(0.5)}", "${cos(0.5)}", "${tan(0.5)}", "${asin(0.5)}",
                                   "${acos(0.5)}", "${atan(0.5)}"}) {
        DiagnosticSink first_sink;
        DiagnosticSink second_sink;
        Status status = Status::Ok;
        const double first = evaluated(expression, first_sink, status);
        EXPECT_EQ(status, Status::Ok) << expression;
        const double second = evaluated(expression, second_sink, status);
        EXPECT_EQ(first, second) << expression;
    }
    DiagnosticSink sink;
    Status status = Status::Ok;
    EXPECT_NEAR(evaluated("${sin(0)}", sink, status), 0.0, 0.0);
    EXPECT_NEAR(evaluated("${cos(0)}", sink, status), 1.0, 0.0);
}

TEST(Expressions, BooleanOperatorsAndTheirPrecedence) {
    // `${not $a and $b}` is `${(not $a) and $b}` (§9.2.1); the value lands in
    // a boolean attribute so the assertions read it back through the IR.
    constexpr std::string_view kSource = R"(<OpenSCENARIO><FileHeader revMajor="1" revMinor="2"
        date="2026-08-01T00:00:00" description="f" author="Scena"/>
      <ParameterDeclarations>
        <ParameterDeclaration name="a" parameterType="boolean" value="false"/>
        <ParameterDeclaration name="b" parameterType="boolean" value="true"/>
        <ParameterDeclaration name="c" parameterType="boolean" value="${not $a and $b}"/>
        <ParameterDeclaration name="d" parameterType="boolean" value="${$a or $b and not $b}"/>
      </ParameterDeclarations>
      <Storyboard/></OpenSCENARIO>)";
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(scena::xml::load_string(kSource, document, sink), Status::Ok);
    EXPECT_EQ(document.scenario.parameters.at("c"), "true");
    EXPECT_EQ(document.scenario.parameters.at("d"), "false");
}

TEST(Expressions, IntegerArithmeticStaysIntegral) {
    // "the result of such an operator has the same data type as its
    // arguments" (§9.2.1): 7 % 2 is an int, and an int-typed parameter
    // accepts it.
    constexpr std::string_view kSource = R"(<OpenSCENARIO><FileHeader revMajor="1" revMinor="2"
        date="2026-08-01T00:00:00" description="f" author="Scena"/>
      <ParameterDeclarations>
        <ParameterDeclaration name="n" parameterType="int" value="${7 % 2 + 3 * 2}"/>
      </ParameterDeclarations>
      <Storyboard/></OpenSCENARIO>)";
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(scena::xml::load_string(kSource, document, sink), Status::Ok);
    EXPECT_EQ(document.scenario.parameters.at("n"), "7");
}

TEST(Expressions, AnIntegerWidensToADoubleButNotTheOtherWay) {
    constexpr std::string_view kWidening = R"(<ParameterDeclarations>
      <ParameterDeclaration name="x" parameterType="double" value="${3}"/>
    </ParameterDeclarations>)";
    DiagnosticSink widening_sink;
    Status status = Status::Ok;
    EXPECT_DOUBLE_EQ(loaded_x(kWidening, "$x", widening_sink, status), 3.0);
    EXPECT_EQ(status, Status::Ok);

    constexpr std::string_view kNarrowing = R"(<ParameterDeclarations>
      <ParameterDeclaration name="n" parameterType="int" value="${3.5}"/>
    </ParameterDeclarations>)";
    DiagnosticSink narrowing_sink;
    (void)loaded_x(kNarrowing, "0", narrowing_sink, status);
    EXPECT_EQ(status, Status::ValidationError);
}

TEST(Expressions, TypeErrorsCiteTheirRules) {
    const struct {
        std::string_view expression;
        std::string_view rule_id;
    } cases[] = {
        // An arithmetic operator where a boolean sits (rule type_of_boolean).
        {"${true + 1}", "asam.net:xosc:1.1.0:expressions.type_of_boolean"},
        {"${not 1}", "asam.net:xosc:1.1.0:expressions.type_of_boolean"},
        // An operator outside the whitelist (rule allowed_operators).
        {"${log(2)}", "asam.net:xosc:1.1.0:expressions.allowed_operators"},
        {"${exp(2)}", "asam.net:xosc:1.1.0:expressions.allowed_operators"},
        // Arguments must follow the operator name in parentheses.
        {"${round 2.6}", "asam.net:xosc:1.1.0:expressions.arguments_of_operators"},
        // The result must be evaluable.
        {"${1 / 0}", "asam.net:xosc:1.1.0:expressions.evaluation_of_expressions_possible"},
        {"${sqrt(-1)}", "asam.net:xosc:1.1.0:expressions.evaluation_of_expressions_possible"},
        {"${1 + }", "asam.net:xosc:1.1.0:expressions.evaluation_of_expressions_possible"},
        {"${(1 + 2}", "asam.net:xosc:1.1.0:expressions.evaluation_of_expressions_possible"},
    };
    for (const auto& test_case : cases) {
        DiagnosticSink sink;
        Status status = Status::Ok;
        (void)evaluated(test_case.expression, sink, status);
        EXPECT_EQ(status, Status::ValidationError) << test_case.expression;
        EXPECT_TRUE(has_rule_id(sink, test_case.rule_id)) << test_case.expression;
    }
}

TEST(Expressions, PowWithANonIntegerExponentIsRejected) {
    // A general pow needs a platform exp/log pair, which is not bit-identical
    // across platforms — the determinism contract, reported rather than
    // silently approximated.
    DiagnosticSink sink;
    Status status = Status::Ok;
    (void)evaluated("${pow(2, 0.5)}", sink, status);
    EXPECT_EQ(status, Status::ValidationError);
    EXPECT_TRUE(
        has_rule_id(sink, "asam.net:xosc:1.1.0:expressions.evaluation_of_expressions_possible"));
}

TEST(Expressions, ThePiConstantIsRejectedForTheTargetedVersions) {
    DiagnosticSink sink;
    Status status = Status::Ok;
    (void)evaluated("${cos(0.25 * pi)}", sink, status);
    // `pi` is a 1.4 addition, and Scena targets 1.0-1.3.
    EXPECT_EQ(status, Status::ValidationError);
    EXPECT_TRUE(has_rule_id(sink, "asam.net:xosc:1.1.0:expressions.allowed_operators"));
}

TEST(Expressions, AStringParameterCannotEnterAnExpression) {
    DiagnosticSink sink;
    Status status = Status::Ok;
    (void)loaded_x(kSpeedDeclaration, "${$label + 1}", sink, status);
    EXPECT_EQ(status, Status::ValidationError);
    EXPECT_TRUE(has_rule_id(sink, "asam.net:xosc:1.1.0:expressions.allowed_operators"));
}

TEST(Expressions, ResolutionIsIdenticalOnRepeatedLoads) {
    // The whole point of doing this at load time: two loads of one document
    // must produce the same IR, bit for bit.
    DiagnosticSink first_sink;
    DiagnosticSink second_sink;
    Status status = Status::Ok;
    const double first =
        loaded_x(kSpeedDeclaration, "${$speed / 3 + sin(0.25)}", first_sink, status);
    const double second =
        loaded_x(kSpeedDeclaration, "${$speed / 3 + sin(0.25)}", second_sink, status);
    EXPECT_EQ(first, second);
    EXPECT_EQ(first_sink.diagnostics().size(), second_sink.diagnostics().size());
}

// --- variables ------------------------------------------------------------

TEST(Variables, DeclarationsLowerWithTheirInitialValue) {
    constexpr std::string_view kSource = R"(<OpenSCENARIO><FileHeader revMajor="1" revMinor="2"
        date="2026-08-01T00:00:00" description="f" author="Scena"/>
      <ParameterDeclarations>
        <ParameterDeclaration name="start" parameterType="int" value="4"/>
      </ParameterDeclarations>
      <VariableDeclarations>
        <VariableDeclaration name="counter" variableType="int" value="${$start * 2}"/>
        <VariableDeclaration name="phase" variableType="string" value="idle"/>
      </VariableDeclarations>
      <Storyboard/></OpenSCENARIO>)";
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(scena::xml::load_string(kSource, document, sink), Status::Ok);
    // A variable's initial value is read at load time, so it may be computed
    // from parameters — but nothing may reference a variable in an
    // expression, because it changes while the scenario runs.
    EXPECT_EQ(document.scenario.variables.at("counter"), "8");
    EXPECT_EQ(document.scenario.variables.at("phase"), "idle");
}

TEST(Variables, AValueOfTheWrongTypeIsRejected) {
    constexpr std::string_view kSource = R"(<OpenSCENARIO><FileHeader revMajor="1" revMinor="2"
        date="2026-08-01T00:00:00" description="f" author="Scena"/>
      <VariableDeclarations>
        <VariableDeclaration name="counter" variableType="int" value="soon"/>
      </VariableDeclarations>
      <Storyboard/></OpenSCENARIO>)";
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::load_string(kSource, document, sink), Status::ValidationError);
    EXPECT_TRUE(has_rule_id(sink, "asam.net:xosc:1.2.0:data_type.variable_correctly_typed"));
}

TEST(Variables, AVariableIsNotAnExpressionInput) {
    // §6.12: variables are runtime state. Referencing one from an attribute
    // is an undeclared-parameter error, not a silent read of its initial
    // value.
    constexpr std::string_view kSource = R"(<OpenSCENARIO><FileHeader revMajor="1" revMinor="2"
        date="2026-08-01T00:00:00" description="f" author="Scena"/>
      <VariableDeclarations>
        <VariableDeclaration name="counter" variableType="int" value="1"/>
      </VariableDeclarations>
      <Storyboard><Init><Actions><Private entityRef="ego"><PrivateAction><TeleportAction>
        <Position><WorldPosition x="$counter" y="0"/></Position>
      </TeleportAction></PrivateAction></Private></Actions></Init></Storyboard></OpenSCENARIO>)";
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::load_string(kSource, document, sink), Status::SemanticError);
}

} // namespace
