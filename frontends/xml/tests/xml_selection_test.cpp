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

// EntitySelection (§7.2.2.2-7.2.2.5): a selection is a way of writing a set
// of entities, so it expands at load time — into a ManeuverGroup's actors
// and into a condition's triggering entities.

#include <string>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

#include "scena/diagnostic.h"
#include "scena/ir/entity_condition.h"
#include "scena/xml/loader.h"

namespace {

using scena::Diagnostic;
using scena::DiagnosticSink;
using scena::Status;
using scena::xml::Document;

/// Three entities: two vehicles and a pedestrian, so a by-type selection has
/// something to discriminate.
constexpr std::string_view kEntities = R"(<Entities>
  <ScenarioObject name="ego">
    <Vehicle name="v1" vehicleCategory="car">
      <BoundingBox><Center x="0" y="0" z="0"/><Dimensions width="2" length="4" height="1.5"/></BoundingBox>
      <Performance maxSpeed="60" maxAcceleration="5" maxDeceleration="9"/>
      <Axles><RearAxle maxSteering="0" wheelDiameter="0.6" trackWidth="1.7" positionX="0" positionZ="0.3"/></Axles>
    </Vehicle>
  </ScenarioObject>
  <ScenarioObject name="lead">
    <Vehicle name="v2" vehicleCategory="truck">
      <BoundingBox><Center x="0" y="0" z="0"/><Dimensions width="2.5" length="8" height="3"/></BoundingBox>
      <Performance maxSpeed="30" maxAcceleration="2" maxDeceleration="6"/>
      <Axles><RearAxle maxSteering="0" wheelDiameter="0.9" trackWidth="2" positionX="0" positionZ="0.45"/></Axles>
    </Vehicle>
  </ScenarioObject>
  <ScenarioObject name="walker">
    <Pedestrian name="p1" pedestrianCategory="pedestrian" mass="80">
      <BoundingBox><Center x="0" y="0" z="0.9"/><Dimensions width="0.6" length="0.4" height="1.8"/></BoundingBox>
    </Pedestrian>
  </ScenarioObject>
  <EntitySelection name="both_cars">
    <Members>
      <EntityRef entityRef="ego"/>
      <EntityRef entityRef="lead"/>
    </Members>
  </EntitySelection>
  <EntitySelection name="all_vehicles">
    <Members><ByType objectType="vehicle"/></Members>
  </EntitySelection>
  <EntitySelection name="everyone">
    <Members>
      <EntityRef entityRef="all_vehicles"/>
      <EntityRef entityRef="walker"/>
    </Members>
  </EntitySelection>
</Entities>)";

std::string scenario_with(std::string_view actors, std::string_view triggering_entity = "ego") {
    return std::string(R"(<OpenSCENARIO><FileHeader revMajor="1" revMinor="2"
        date="2026-08-01T00:00:00" description="selection fixture" author="Scena"/>)") +
           std::string(kEntities) +
           R"(<Storyboard><Story name="s"><Act name="a"><ManeuverGroup name="g">
        <Actors selectTriggeringEntities="false">)" +
           std::string(actors) + R"(</Actors>
        <Maneuver name="m"><Event name="e">
          <Action name="x"><PrivateAction><TeleportAction>
            <Position><WorldPosition x="0" y="0"/></Position>
          </TeleportAction></PrivateAction></Action>
          <StartTrigger><ConditionGroup><Condition name="c" delay="0" conditionEdge="none">
            <ByEntityCondition>
              <TriggeringEntities triggeringEntitiesRule="any">
                <EntityRef entityRef=")" +
           std::string(triggering_entity) + R"("/>
              </TriggeringEntities>
              <EntityCondition><SpeedCondition value="10" rule="greaterThan"/></EntityCondition>
            </ByEntityCondition>
          </Condition></ConditionGroup></StartTrigger>
        </Event></Maneuver>
      </ManeuverGroup></Act></Story></Storyboard></OpenSCENARIO>)";
}

const std::vector<std::string>& actors_of(const Document& document) {
    return document.scenario.storyboard.stories.at(0).acts.at(0).groups.at(0).actors;
}

const scena::ir::TriggeringEntities& triggering_of(const Document& document) {
    const auto& event = document.scenario.storyboard.stories.at(0)
                            .acts.at(0)
                            .groups.at(0)
                            .maneuvers.at(0)
                            .events.at(0);
    const auto* condition = dynamic_cast<const scena::ir::ByEntityCondition*>(
        event.start_trigger->groups.front().conditions.front().expression.get());
    return condition->triggering_entities();
}

TEST(EntitySelection, ExplicitMembersExpandIntoActors) {
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(scena::xml::load_string(scenario_with(R"(<EntityRef entityRef="both_cars"/>)"),
                                      document, sink),
              Status::Ok);
    // §7.2.2.2: the group's private actions apply individually to each member.
    EXPECT_EQ(actors_of(document), (std::vector<std::string>{"ego", "lead"}));
    // One action per actor, the §8.3.3.3 bulk rule the actors now carry.
    EXPECT_EQ(document.scenario.storyboard.stories.at(0)
                  .acts.at(0)
                  .groups.at(0)
                  .maneuvers.at(0)
                  .events.at(0)
                  .actions.size(),
              2U);
}

TEST(EntitySelection, ByTypeSelectsInDeclarationOrder) {
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(scena::xml::load_string(scenario_with(R"(<EntityRef entityRef="all_vehicles"/>)"),
                                      document, sink),
              Status::Ok);
    // The two vehicles, in the order the entities were declared — never the
    // order a map or a hash would produce.
    EXPECT_EQ(actors_of(document), (std::vector<std::string>{"ego", "lead"}));
}

TEST(EntitySelection, ASelectionMayNameAnotherSelection) {
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(scena::xml::load_string(scenario_with(R"(<EntityRef entityRef="everyone"/>)"),
                                      document, sink),
              Status::Ok);
    EXPECT_EQ(actors_of(document), (std::vector<std::string>{"ego", "lead", "walker"}));
}

TEST(EntitySelection, AMemberNamedTwiceIsOneMember) {
    constexpr std::string_view kActors = R"(<EntityRef entityRef="all_vehicles"/>
      <EntityRef entityRef="ego"/>)";
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(scena::xml::load_string(scenario_with(kActors), document, sink), Status::Ok);
    // The selection expands to ego+lead; naming ego again adds a second
    // actor entry, which is what the document asked for — the deduplication
    // is inside a selection, not across the actor list.
    EXPECT_EQ(actors_of(document), (std::vector<std::string>{"ego", "lead", "ego"}));
}

TEST(EntitySelection, ASelectionMayBeATriggeringEntity) {
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(scena::xml::load_string(scenario_with(R"(<EntityRef entityRef="ego"/>)", "both_cars"),
                                      document, sink),
              Status::Ok);
    // The any/all reduction then runs over the members (§7.6.5.1).
    EXPECT_EQ(triggering_of(document).entity_refs, (std::vector<std::string>{"ego", "lead"}));
    EXPECT_EQ(triggering_of(document).rule, scena::ir::TriggeringEntitiesRule::Any);
}

TEST(EntitySelection, AnUndeclaredMemberIsReported) {
    constexpr std::string_view kSource = R"(<OpenSCENARIO><FileHeader revMajor="1" revMinor="2"
        date="2026-08-01T00:00:00" description="f" author="Scena"/>
      <Entities>
        <EntitySelection name="ghosts">
          <Members><EntityRef entityRef="nobody"/></Members>
        </EntitySelection>
      </Entities><Storyboard/></OpenSCENARIO>)";
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::load_string(kSource, document, sink), Status::SemanticError);
    bool cited = false;
    for (const Diagnostic& diagnostic : sink.diagnostics()) {
        cited = cited || diagnostic.rule_id ==
                             "asam.net:xosc:1.2.0:reference_control.references_to_scenario_object";
    }
    EXPECT_TRUE(cited);
}

TEST(EntitySelection, AnEmptySelectionExpandsToNothing) {
    constexpr std::string_view kSource = R"(<OpenSCENARIO><FileHeader revMajor="1" revMinor="2"
        date="2026-08-01T00:00:00" description="f" author="Scena"/>
      <Entities>
        <ScenarioObject name="walker">
          <Pedestrian name="p" pedestrianCategory="pedestrian" mass="80">
            <BoundingBox><Center x="0" y="0" z="0.9"/><Dimensions width="0.6" length="0.4" height="1.8"/></BoundingBox>
          </Pedestrian>
        </ScenarioObject>
        <EntitySelection name="no_vehicles">
          <Members><ByType objectType="vehicle"/></Members>
        </EntitySelection>
      </Entities>
      <Storyboard><Story name="s"><Act name="a"><ManeuverGroup name="g">
        <Actors selectTriggeringEntities="false"><EntityRef entityRef="no_vehicles"/></Actors>
        <Maneuver name="m"><Event name="e"><Action name="x"><PrivateAction><TeleportAction>
          <Position><WorldPosition x="0" y="0"/></Position>
        </TeleportAction></PrivateAction></Action></Event></Maneuver>
      </ManeuverGroup></Act></Story></Storyboard></OpenSCENARIO>)";
    Document document;
    DiagnosticSink sink;
    // No actor means a private action has no subject, which the storyboard
    // reader reports rather than silently dropping.
    EXPECT_EQ(scena::xml::load_string(kSource, document, sink), Status::ValidationError);
}

} // namespace
