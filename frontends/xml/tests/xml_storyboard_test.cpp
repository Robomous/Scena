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

// p4-s2 structural lowering: entities, the storyboard hierarchy, init
// actions, triggers, and the action/condition payloads. The IR is checked
// through a stable text dump (the snapshot invariant) and, for GS-1, by
// running the loaded scenario through the engine.

#include <filesystem>
#include <sstream>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include <gtest/gtest.h>

#include "scena/diagnostic.h"
#include "scena/engine.h"
#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/entity_condition.h"
#include "scena/ir/interaction_condition.h"
#include "scena/xml/loader.h"

namespace {

using scena::Diagnostic;
using scena::DiagnosticSink;
using scena::Severity;
using scena::Status;
using scena::xml::Document;
using scena::xml::DocumentKind;

/// Wraps `body` in the smallest valid 1.2 scenario document.
std::string document_with(std::string_view body) {
    return std::string("<OpenSCENARIO><FileHeader revMajor=\"1\" revMinor=\"2\" "
                       "date=\"2026-08-01T00:00:00\" description=\"fixture\" author=\"Scena\"/>") +
           std::string(body) + "</OpenSCENARIO>";
}

Status load(std::string_view body, Document& document, DiagnosticSink& sink) {
    return scena::xml::load_string(document_with(body), document, sink);
}

bool has_message_containing(const DiagnosticSink& sink, std::string_view needle) {
    for (const Diagnostic& diagnostic : sink.diagnostics()) {
        if (diagnostic.message.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

std::vector<Diagnostic> errors(const DiagnosticSink& sink) {
    std::vector<Diagnostic> result;
    for (const Diagnostic& diagnostic : sink.diagnostics()) {
        if (diagnostic.severity == Severity::Error) {
            result.push_back(diagnostic);
        }
    }
    return result;
}

// --- the IR dump ----------------------------------------------------------

/// Renders a scenario as a stable, line-oriented text tree.
///
/// The dump names elements and action/condition kinds and never prints a
/// floating-point value, so a snapshot compares the *structure* the loader
/// built without depending on how a platform formats doubles. Two loads of
/// the same document produce the same text — that is the round-trip
/// invariant the sprint's exit criterion asks for.
void dump_trigger(std::ostringstream& out, const std::string& indent,
                  const scena::ir::Trigger& trigger, const char* label) {
    out << indent << label << " groups=" << trigger.groups.size() << "\n";
    for (const scena::ir::ConditionGroup& group : trigger.groups) {
        out << indent << "  group\n";
        for (const scena::ir::TriggerCondition& condition : group.conditions) {
            out << indent << "    condition name=" << condition.name
                << " edge=" << static_cast<int>(condition.edge)
                << " delayed=" << (condition.delay > 0.0 ? "yes" : "no") << "\n";
        }
    }
}

std::string dump(const scena::ir::Scenario& scenario) {
    std::ostringstream out;
    out << "scenario\n";
    for (const auto& [name, value] : scenario.parameters) {
        out << "  parameter " << name << "=" << value << "\n";
    }
    for (const auto& [name, value] : scenario.variables) {
        out << "  variable " << name << "=" << value << "\n";
    }
    for (const scena::ir::Entity& entity : scenario.entities) {
        out << "  entity " << entity.id;
        if (const std::optional<scena::ir::ObjectType> type = scena::ir::object_type_of(entity)) {
            out << " type=" << static_cast<int>(*type);
        } else {
            out << " type=none";
        }
        out << "\n";
    }
    for (const auto& controller : scenario.traffic_signal_controllers) {
        out << "  signalController " << controller.name << " phases=" << controller.phases.size()
            << "\n";
    }
    for (const auto& action : scenario.init_actions) {
        out << "  init " << action->kind() << " actor=" << action->entity_id() << "\n";
    }
    for (const scena::ir::Story& story : scenario.storyboard.stories) {
        out << "  story " << story.name << "\n";
        for (const scena::ir::Act& act : story.acts) {
            out << "    act " << act.name << "\n";
            if (act.start_trigger.has_value()) {
                dump_trigger(out, "      ", *act.start_trigger, "startTrigger");
            }
            if (act.stop_trigger.has_value()) {
                dump_trigger(out, "      ", *act.stop_trigger, "stopTrigger");
            }
            for (const scena::ir::ManeuverGroup& group : act.groups) {
                out << "      group " << group.name << " actors=" << group.actors.size() << "\n";
                for (const scena::ir::Maneuver& maneuver : group.maneuvers) {
                    out << "        maneuver " << maneuver.name << "\n";
                    for (const scena::ir::Event& event : maneuver.events) {
                        out << "          event " << event.name
                            << " priority=" << static_cast<int>(event.priority)
                            << " count=" << event.maximum_execution_count << "\n";
                        if (event.start_trigger.has_value()) {
                            dump_trigger(out, "            ", *event.start_trigger, "startTrigger");
                        }
                        for (const auto& action : event.actions) {
                            out << "            action " << action->kind()
                                << " actor=" << action->entity_id() << "\n";
                        }
                    }
                }
            }
        }
    }
    if (scenario.storyboard.stop_trigger.has_value()) {
        dump_trigger(out, "  ", *scenario.storyboard.stop_trigger, "storyboardStopTrigger");
    }
    return out.str();
}

// --- entities -------------------------------------------------------------

constexpr std::string_view kVehicleEntity = R"(<Entities>
  <ScenarioObject name="ego">
    <Vehicle name="ego_vehicle" vehicleCategory="car" role="police" mass="1500">
      <BoundingBox>
        <Center x="1.4" y="0.1" z="0.9"/>
        <Dimensions width="2.0" length="4.6" height="1.8"/>
      </BoundingBox>
      <Performance maxSpeed="60.0" maxAcceleration="5.0" maxDeceleration="9.0"/>
      <Axles>
        <FrontAxle maxSteering="0.5" wheelDiameter="0.6" trackWidth="1.7" positionX="2.8" positionZ="0.3"/>
        <RearAxle maxSteering="0.0" wheelDiameter="0.6" trackWidth="1.7" positionX="0.0" positionZ="0.3"/>
      </Axles>
      <Properties>
        <Property name="model" value="generic"/>
      </Properties>
    </Vehicle>
  </ScenarioObject>
</Entities>)";

TEST(Entities, VehicleLowersCompletely) {
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(load(kVehicleEntity, document, sink), Status::Ok) << sink.diagnostics().size();
    ASSERT_EQ(document.scenario.entities.size(), 1U);

    const scena::ir::Entity& entity = document.scenario.entities.front();
    EXPECT_EQ(entity.id, "ego");
    EXPECT_EQ(entity.name, "ego");
    EXPECT_EQ(entity.control_mode, scena::ir::ControlMode::EngineControlled);
    ASSERT_TRUE(entity.object.has_value());
    const auto* vehicle = std::get_if<scena::ir::Vehicle>(&*entity.object);
    ASSERT_NE(vehicle, nullptr);
    EXPECT_EQ(vehicle->category, scena::ir::VehicleCategory::Car);
    EXPECT_EQ(vehicle->role, scena::ir::Role::Police);
    ASSERT_TRUE(vehicle->mass.has_value());
    EXPECT_DOUBLE_EQ(*vehicle->mass, 1500.0);
    EXPECT_DOUBLE_EQ(vehicle->bounding_box.center_x, 1.4);
    EXPECT_DOUBLE_EQ(vehicle->bounding_box.length, 4.6);
    EXPECT_DOUBLE_EQ(vehicle->performance.max_speed, 60.0);
    EXPECT_DOUBLE_EQ(vehicle->axles.rear.wheel_diameter, 0.6);
    ASSERT_TRUE(vehicle->axles.front.has_value());
    EXPECT_DOUBLE_EQ(vehicle->axles.front->position_x, 2.8);
    ASSERT_EQ(vehicle->properties.size(), 1U);
    EXPECT_EQ(vehicle->properties.front().name, "model");
}

TEST(Entities, PedestrianAndMiscObjectLower) {
    constexpr std::string_view kBody = R"(<Entities>
  <ScenarioObject name="walker">
    <Pedestrian name="p" pedestrianCategory="pedestrian" mass="80">
      <BoundingBox><Center x="0" y="0" z="0.9"/><Dimensions width="0.6" length="0.4" height="1.8"/></BoundingBox>
    </Pedestrian>
  </ScenarioObject>
  <ScenarioObject name="cone">
    <MiscObject miscObjectCategory="obstacle" mass="5">
      <BoundingBox><Center x="0" y="0" z="0.3"/><Dimensions width="0.4" length="0.4" height="0.6"/></BoundingBox>
    </MiscObject>
  </ScenarioObject>
</Entities>)";
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(load(kBody, document, sink), Status::Ok);
    ASSERT_EQ(document.scenario.entities.size(), 2U);
    EXPECT_EQ(*scena::ir::object_type_of(document.scenario.entities[0]),
              scena::ir::ObjectType::Pedestrian);
    EXPECT_EQ(*scena::ir::object_type_of(document.scenario.entities[1]),
              scena::ir::ObjectType::MiscObject);
}

TEST(Entities, MissingRequiredChildIsReported) {
    constexpr std::string_view kBody = R"(<Entities>
  <ScenarioObject name="ego">
    <Vehicle name="v" vehicleCategory="car">
      <BoundingBox><Center x="0" y="0" z="0"/><Dimensions width="1" length="1" height="1"/></BoundingBox>
    </Vehicle>
  </ScenarioObject>
</Entities>)";
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(load(kBody, document, sink), Status::ValidationError);
    EXPECT_TRUE(has_message_containing(sink, "no required 'Performance' child"));
    // The entity still exists, unclassified — the loader keeps going.
    ASSERT_EQ(document.scenario.entities.size(), 1U);
    EXPECT_FALSE(document.scenario.entities.front().object.has_value());
}

TEST(Entities, CatalogReferencesAndSelectionsAreDeferred) {
    constexpr std::string_view kBody = R"(<Entities>
  <ScenarioObject name="ego">
    <CatalogReference catalogName="Vehicles" entryName="car"/>
  </ScenarioObject>
  <EntitySelection name="cars"><Members/></EntitySelection>
</Entities>)";
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(load(kBody, document, sink), Status::Ok);
    EXPECT_TRUE(has_message_containing(sink, "p4-s4"));
}

// --- storyboard -----------------------------------------------------------

constexpr std::string_view kStoryboard = R"(<Storyboard>
  <Init>
    <Actions>
      <GlobalAction>
        <InfrastructureAction>
          <TrafficSignalAction>
            <TrafficSignalStateAction name="signal_1" state="green"/>
          </TrafficSignalAction>
        </InfrastructureAction>
      </GlobalAction>
      <Private entityRef="ego">
        <PrivateAction>
          <TeleportAction><Position><WorldPosition x="1.0" y="2.0"/></Position></TeleportAction>
        </PrivateAction>
      </Private>
    </Actions>
  </Init>
  <Story name="story_1">
    <Act name="act_1">
      <ManeuverGroup name="group_1">
        <Actors selectTriggeringEntities="false"><EntityRef entityRef="ego"/></Actors>
        <Maneuver name="maneuver_1">
          <Event name="event_1" priority="overwrite" maximumExecutionCount="3">
            <Action name="brake">
              <PrivateAction>
                <LongitudinalAction>
                  <SpeedAction>
                    <SpeedActionDynamics dynamicsShape="linear" dynamicsDimension="time" value="2.0"/>
                    <SpeedActionTarget><AbsoluteTargetSpeed value="0.0"/></SpeedActionTarget>
                  </SpeedAction>
                </LongitudinalAction>
              </PrivateAction>
            </Action>
            <StartTrigger>
              <ConditionGroup>
                <Condition name="c1" delay="1.5" conditionEdge="rising">
                  <ByEntityCondition>
                    <TriggeringEntities triggeringEntitiesRule="any">
                      <EntityRef entityRef="ego"/>
                    </TriggeringEntities>
                    <EntityCondition><SpeedCondition value="10.0" rule="greaterThan"/></EntityCondition>
                  </ByEntityCondition>
                </Condition>
              </ConditionGroup>
            </StartTrigger>
          </Event>
        </Maneuver>
      </ManeuverGroup>
      <StartTrigger>
        <ConditionGroup>
          <Condition name="act_start" delay="0" conditionEdge="none">
            <ByValueCondition><SimulationTimeCondition value="0.0" rule="greaterOrEqual"/></ByValueCondition>
          </Condition>
        </ConditionGroup>
      </StartTrigger>
      <StopTrigger>
        <ConditionGroup>
          <Condition name="act_stop" delay="0" conditionEdge="none">
            <ByValueCondition><SimulationTimeCondition value="30.0" rule="greaterOrEqual"/></ByValueCondition>
          </Condition>
        </ConditionGroup>
      </StopTrigger>
    </Act>
  </Story>
  <StopTrigger>
    <ConditionGroup>
      <Condition name="done" delay="0" conditionEdge="none">
        <ByValueCondition><SimulationTimeCondition value="60.0" rule="greaterOrEqual"/></ByValueCondition>
      </Condition>
    </ConditionGroup>
  </StopTrigger>
</Storyboard>)";

TEST(Storyboard, HierarchyLowersToASnapshot) {
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(load(kStoryboard, document, sink), Status::Ok);

    constexpr std::string_view kExpected = "scenario\n"
                                           "  init TrafficSignalStateAction actor=\n"
                                           "  init TeleportAction actor=ego\n"
                                           "  story story_1\n"
                                           "    act act_1\n"
                                           "      startTrigger groups=1\n"
                                           "        group\n"
                                           "          condition name=act_start edge=0 delayed=no\n"
                                           "      stopTrigger groups=1\n"
                                           "        group\n"
                                           "          condition name=act_stop edge=0 delayed=no\n"
                                           "      group group_1 actors=1\n"
                                           "        maneuver maneuver_1\n"
                                           "          event event_1 priority=0 count=3\n"
                                           "            startTrigger groups=1\n"
                                           "              group\n"
                                           "                condition name=c1 edge=1 delayed=yes\n"
                                           "            action SpeedAction actor=ego\n"
                                           "  storyboardStopTrigger groups=1\n"
                                           "    group\n"
                                           "      condition name=done edge=0 delayed=no\n";
    EXPECT_EQ(dump(document.scenario), kExpected);
}

TEST(Storyboard, LoadingTwiceProducesTheSameIr) {
    Document first;
    DiagnosticSink first_sink;
    ASSERT_EQ(load(kStoryboard, first, first_sink), Status::Ok);
    Document second;
    DiagnosticSink second_sink;
    ASSERT_EQ(load(kStoryboard, second, second_sink), Status::Ok);
    EXPECT_EQ(dump(first.scenario), dump(second.scenario));
    EXPECT_EQ(first_sink.diagnostics().size(), second_sink.diagnostics().size());
}

TEST(Storyboard, PrivateActionAppliesToEveryActor) {
    constexpr std::string_view kBody = R"(<Storyboard>
  <Story name="s"><Act name="a">
    <ManeuverGroup name="g">
      <Actors selectTriggeringEntities="false">
        <EntityRef entityRef="ego"/><EntityRef entityRef="target"/>
      </Actors>
      <Maneuver name="m"><Event name="e">
        <Action name="teleport">
          <PrivateAction><TeleportAction><Position><WorldPosition x="0" y="0"/></Position></TeleportAction></PrivateAction>
        </Action>
      </Event></Maneuver>
    </ManeuverGroup>
  </Act></Story>
</Storyboard>)";
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(load(kBody, document, sink), Status::Ok);
    const auto& actions = document.scenario.storyboard.stories.at(0)
                              .acts.at(0)
                              .groups.at(0)
                              .maneuvers.at(0)
                              .events.at(0)
                              .actions;
    // §8.3.3.3: one private action, applied to each actor of the group.
    ASSERT_EQ(actions.size(), 2U);
    EXPECT_EQ(actions[0]->entity_id(), "ego");
    EXPECT_EQ(actions[1]->entity_id(), "target");
}

TEST(Storyboard, EmptyEventIsRejected) {
    constexpr std::string_view kBody = R"(<Storyboard>
  <Story name="s"><Act name="a">
    <ManeuverGroup name="g">
      <Actors selectTriggeringEntities="false"><EntityRef entityRef="ego"/></Actors>
      <Maneuver name="m"><Event name="e"/></Maneuver>
    </ManeuverGroup>
  </Act></Story>
</Storyboard>)";
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(load(kBody, document, sink), Status::ValidationError);
    EXPECT_TRUE(has_message_containing(sink, "Event declares no action"));
}

TEST(Storyboard, ManeuverGroupExecutionCountIsReportedNotSilentlyIgnored) {
    constexpr std::string_view kBody = R"(<Storyboard>
  <Story name="s"><Act name="a">
    <ManeuverGroup name="g" maximumExecutionCount="4">
      <Actors selectTriggeringEntities="false"><EntityRef entityRef="ego"/></Actors>
      <Maneuver name="m"><Event name="e">
        <Action name="t"><PrivateAction><TeleportAction><Position><WorldPosition x="0" y="0"/></Position></TeleportAction></PrivateAction></Action>
      </Event></Maneuver>
    </ManeuverGroup>
  </Act></Story>
</Storyboard>)";
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(load(kBody, document, sink), Status::Ok);
    EXPECT_TRUE(has_message_containing(sink, "maximumExecutionCount is not honoured yet"));
}

// --- action and condition payloads ---------------------------------------

/// Loads one private action inside a minimal storyboard and returns it.
std::shared_ptr<scena::ir::Action> load_private_action(std::string_view action_xml,
                                                       DiagnosticSink& sink, Status& status) {
    const std::string body = std::string(R"(<Storyboard><Story name="s"><Act name="a">
      <ManeuverGroup name="g">
        <Actors selectTriggeringEntities="false"><EntityRef entityRef="ego"/></Actors>
        <Maneuver name="m"><Event name="e"><Action name="x"><PrivateAction>)") +
                             std::string(action_xml) +
                             R"(</PrivateAction></Action></Event></Maneuver>
      </ManeuverGroup></Act></Story></Storyboard>)";
    Document document;
    status = load(body, document, sink);
    const auto& stories = document.scenario.storyboard.stories;
    if (stories.empty()) {
        return nullptr;
    }
    const auto& actions =
        stories.at(0).acts.at(0).groups.at(0).maneuvers.at(0).events.at(0).actions;
    return actions.empty() ? nullptr : actions.front();
}

TEST(Actions, LongitudinalFamilyLowers) {
    struct Case {
        std::string_view xml;
        std::string_view kind;
    };
    const Case cases[] = {
        {R"(<LongitudinalAction><SpeedAction>
             <SpeedActionDynamics dynamicsShape="cubic" dynamicsDimension="rate" value="3.0"/>
             <SpeedActionTarget><RelativeTargetSpeed entityRef="lead" value="-2.0" speedTargetValueType="delta" continuous="true"/></SpeedActionTarget>
           </SpeedAction></LongitudinalAction>)",
         "SpeedAction"},
        {R"(<LongitudinalAction><SpeedProfileAction followingMode="position">
             <SpeedProfileEntry speed="10.0" time="1.0"/><SpeedProfileEntry speed="20.0" time="2.0"/>
           </SpeedProfileAction></LongitudinalAction>)",
         "SpeedProfileAction"},
        {R"(<LongitudinalAction><LongitudinalDistanceAction entityRef="lead" distance="12.0"
             freespace="true" continuous="true" displacement="trailingReferencedEntity">
             <DynamicConstraints maxAcceleration="3.0"/>
           </LongitudinalDistanceAction></LongitudinalAction>)",
         "LongitudinalDistanceAction"},
    };
    for (const Case& test_case : cases) {
        DiagnosticSink sink;
        Status status = Status::Ok;
        const std::shared_ptr<scena::ir::Action> action =
            load_private_action(test_case.xml, sink, status);
        ASSERT_NE(action, nullptr) << test_case.kind;
        EXPECT_EQ(status, Status::Ok) << test_case.kind;
        EXPECT_EQ(action->kind(), test_case.kind);
    }
}

TEST(Actions, LateralAndRoutingFamiliesLower) {
    struct Case {
        std::string_view xml;
        std::string_view kind;
    };
    const Case cases[] = {
        {R"(<LateralAction><LaneChangeAction targetLaneOffset="0.2">
             <LaneChangeActionDynamics dynamicsShape="sinusoidal" dynamicsDimension="time" value="3.0"/>
             <LaneChangeTarget><RelativeTargetLane entityRef="ego" value="-1"/></LaneChangeTarget>
           </LaneChangeAction></LateralAction>)",
         "LaneChangeAction"},
        {R"(<LateralAction><LaneOffsetAction continuous="false">
             <LaneOffsetActionDynamics maxLateralAcc="2.0" dynamicsShape="linear"/>
             <LaneOffsetTarget><AbsoluteTargetLaneOffset value="1.0"/></LaneOffsetTarget>
           </LaneOffsetAction></LateralAction>)",
         "LaneOffsetAction"},
        {R"(<LateralAction><LateralDistanceAction entityRef="lead" distance="2.0" freespace="true"
             continuous="true"/></LateralAction>)",
         "LateralDistanceAction"},
        {R"(<RoutingAction><AssignRouteAction><Route name="r" closed="false">
             <Waypoint routeStrategy="shortest"><Position><WorldPosition x="0" y="0"/></Position></Waypoint>
             <Waypoint routeStrategy="shortest"><Position><WorldPosition x="10" y="0"/></Position></Waypoint>
           </Route></AssignRouteAction></RoutingAction>)",
         "AssignRouteAction"},
        {R"(<RoutingAction><FollowTrajectoryAction initialDistanceOffset="0.0">
             <TrajectoryRef><Trajectory name="t" closed="false"><Shape><Polyline>
               <Vertex time="0"><Position><WorldPosition x="0" y="0"/></Position></Vertex>
               <Vertex time="1"><Position><WorldPosition x="5" y="0"/></Position></Vertex>
             </Polyline></Shape></Trajectory></TrajectoryRef>
             <TimeReference><Timing domainAbsoluteRelative="absolute" scale="1.0" offset="0.0"/></TimeReference>
             <TrajectoryFollowingMode followingMode="position"/>
           </FollowTrajectoryAction></RoutingAction>)",
         "FollowTrajectoryAction"},
        {R"(<RoutingAction><AcquirePositionAction><Position><WorldPosition x="50" y="0"/></Position></AcquirePositionAction></RoutingAction>)",
         "AcquirePositionAction"},
        {R"(<ControllerAction><AssignControllerAction activateLateral="true" activateLongitudinal="false">
             <Controller name="c" controllerType="movement"><Properties><Property name="k" value="v"/></Properties></Controller>
           </AssignControllerAction></ControllerAction>)",
         "AssignControllerAction"},
        {R"(<AppearanceAction><VisibilityAction graphics="true" sensors="false" traffic="true"/></AppearanceAction>)",
         "VisibilityAction"},
    };
    for (const Case& test_case : cases) {
        DiagnosticSink sink;
        Status status = Status::Ok;
        const std::shared_ptr<scena::ir::Action> action =
            load_private_action(test_case.xml, sink, status);
        ASSERT_NE(action, nullptr) << test_case.kind;
        EXPECT_EQ(status, Status::Ok) << test_case.kind;
        EXPECT_EQ(action->kind(), test_case.kind);
    }
}

TEST(Actions, DeprecatedActivateControllerPlacementIsAcceptedWithAWarning) {
    DiagnosticSink sink;
    Status status = Status::Ok;
    const std::shared_ptr<scena::ir::Action> action = load_private_action(
        R"(<ActivateControllerAction lateral="true" longitudinal="true"/>)", sink, status);
    ASSERT_NE(action, nullptr);
    EXPECT_EQ(status, Status::Ok);
    EXPECT_EQ(action->kind(), "ActivateControllerAction");
    bool deprecated = false;
    for (const Diagnostic& diagnostic : sink.diagnostics()) {
        deprecated = deprecated || diagnostic.code == Status::DeprecatedFeature;
    }
    EXPECT_TRUE(deprecated);
}

TEST(Actions, GlobalActionsLowerInInit) {
    constexpr std::string_view kBody = R"(<Storyboard><Init><Actions>
      <GlobalAction><EnvironmentAction><Environment name="env">
        <TimeOfDay animation="false" dateTime="2026-08-01T12:00:00"/>
        <Weather><Sun azimuth="1.0" elevation="0.5" illuminance="10000"/><Fog visualRange="1000"/></Weather>
        <RoadCondition frictionScaleFactor="1.0"/>
      </Environment></EnvironmentAction></GlobalAction>
      <GlobalAction><VariableAction variableRef="v"><SetAction value="7"/></VariableAction></GlobalAction>
      <GlobalAction><ParameterAction parameterRef="p"><ModifyAction><Rule><AddValue value="2"/></Rule></ModifyAction></ParameterAction></GlobalAction>
      <GlobalAction><EntityAction entityRef="ghost"><DeleteEntityAction/></EntityAction></GlobalAction>
      <UserDefinedAction><CustomCommandAction type="log">hello</CustomCommandAction></UserDefinedAction>
    </Actions></Init></Storyboard>)";
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(load(kBody, document, sink), Status::Ok);
    ASSERT_EQ(document.scenario.init_actions.size(), 5U);
    EXPECT_EQ(document.scenario.init_actions[0]->kind(), "EnvironmentAction");
    EXPECT_EQ(document.scenario.init_actions[1]->kind(), "VariableSetAction");
    EXPECT_EQ(document.scenario.init_actions[2]->kind(), "ParameterModifyAction");
    EXPECT_EQ(document.scenario.init_actions[3]->kind(), "DeleteEntityAction");
    EXPECT_EQ(document.scenario.init_actions[4]->kind(), "CustomCommandAction");
}

/// Loads one condition expression inside an event start trigger.
std::shared_ptr<scena::ir::Condition> load_condition(std::string_view condition_xml,
                                                     DiagnosticSink& sink, Status& status) {
    const std::string body =
        std::string(R"(<Storyboard><Story name="s"><Act name="a"><ManeuverGroup name="g">
        <Actors selectTriggeringEntities="false"><EntityRef entityRef="ego"/></Actors>
        <Maneuver name="m"><Event name="e">
          <Action name="x"><PrivateAction><TeleportAction><Position><WorldPosition x="0" y="0"/></Position></TeleportAction></PrivateAction></Action>
          <StartTrigger><ConditionGroup><Condition name="c" delay="0" conditionEdge="none">)") +
        std::string(condition_xml) +
        R"(</Condition></ConditionGroup></StartTrigger>
        </Event></Maneuver></ManeuverGroup></Act></Story></Storyboard>)";
    Document document;
    status = load(body, document, sink);
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

TEST(Conditions, ByValueFamilyLowers) {
    const std::string_view cases[] = {
        R"(<ByValueCondition><SimulationTimeCondition value="5.0" rule="greaterThan"/></ByValueCondition>)",
        R"(<ByValueCondition><ParameterCondition parameterRef="p" value="1" rule="equalTo"/></ByValueCondition>)",
        R"(<ByValueCondition><VariableCondition variableRef="v" value="1" rule="equalTo"/></ByValueCondition>)",
        R"(<ByValueCondition><UserDefinedValueCondition name="u" value="1" rule="equalTo"/></ByValueCondition>)",
        R"(<ByValueCondition><TimeOfDayCondition dateTime="2026-08-01T12:00:00" rule="greaterThan"/></ByValueCondition>)",
        R"(<ByValueCondition><TrafficSignalCondition name="s" state="green"/></ByValueCondition>)",
        R"(<ByValueCondition><TrafficSignalControllerCondition trafficSignalControllerRef="c" phase="p"/></ByValueCondition>)",
        R"(<ByValueCondition><StoryboardElementStateCondition storyboardElementType="event" storyboardElementRef="e" state="completeState"/></ByValueCondition>)",
    };
    for (const std::string_view& condition : cases) {
        DiagnosticSink sink;
        Status status = Status::Ok;
        EXPECT_NE(load_condition(condition, sink, status), nullptr) << condition;
        EXPECT_EQ(status, Status::Ok) << condition;
    }
}

TEST(Conditions, ByEntityFamilyLowers) {
    const std::string_view predicates[] = {
        R"(<SpeedCondition value="10" rule="greaterThan" direction="longitudinal"/>)",
        R"(<RelativeSpeedCondition entityRef="lead" value="-2" rule="lessThan"/>)",
        R"(<AccelerationCondition value="1" rule="greaterThan"/>)",
        R"(<StandStillCondition duration="2.0"/>)",
        R"(<TraveledDistanceCondition value="100"/>)",
        R"(<EndOfRoadCondition duration="0.5"/>)",
        R"(<OffroadCondition duration="0.5"/>)",
        R"(<CollisionCondition><EntityRef entityRef="lead"/></CollisionCondition>)",
        R"(<TimeHeadwayCondition entityRef="lead" value="1.5" freespace="true" rule="lessThan"/>)",
        R"(<TimeToCollisionCondition value="2.0" freespace="true" rule="lessThan">
             <TimeToCollisionConditionTarget><EntityRef entityRef="lead"/></TimeToCollisionConditionTarget>
           </TimeToCollisionCondition>)",
        R"(<RelativeDistanceCondition entityRef="lead" value="5" freespace="false"
             relativeDistanceType="longitudinal" rule="lessThan"/>)",
        R"(<DistanceCondition value="5" freespace="false" rule="lessThan">
             <Position><WorldPosition x="10" y="0"/></Position>
           </DistanceCondition>)",
        R"(<ReachPositionCondition tolerance="1.0">
             <Position><WorldPosition x="10" y="0"/></Position>
           </ReachPositionCondition>)",
        R"(<RelativeClearanceCondition freeSpace="true" oppositeLanes="false"
             distanceForward="10" distanceBackward="5">
             <RelativeLaneRange from="-1" to="1"/>
           </RelativeClearanceCondition>)",
    };
    for (const std::string_view& predicate : predicates) {
        const std::string condition =
            std::string(R"(<ByEntityCondition><TriggeringEntities triggeringEntitiesRule="all">
              <EntityRef entityRef="ego"/></TriggeringEntities><EntityCondition>)") +
            std::string(predicate) + R"(</EntityCondition></ByEntityCondition>)";
        DiagnosticSink sink;
        Status status = Status::Ok;
        EXPECT_NE(load_condition(condition, sink, status), nullptr) << predicate;
        EXPECT_EQ(status, Status::Ok) << predicate;
    }
}

TEST(Conditions, NegativeDelayCitesItsRule) {
    DiagnosticSink sink;
    Status status = Status::Ok;
    const std::string body =
        R"(<Storyboard><Story name="s"><Act name="a"><ManeuverGroup name="g">
        <Actors selectTriggeringEntities="false"><EntityRef entityRef="ego"/></Actors>
        <Maneuver name="m"><Event name="e">
          <Action name="x"><PrivateAction><TeleportAction><Position><WorldPosition x="0" y="0"/></Position></TeleportAction></PrivateAction></Action>
          <StartTrigger><ConditionGroup><Condition name="c" delay="-1" conditionEdge="none">
            <ByValueCondition><SimulationTimeCondition value="1" rule="greaterThan"/></ByValueCondition>
          </Condition></ConditionGroup></StartTrigger>
        </Event></Maneuver></ManeuverGroup></Act></Story></Storyboard>)";
    Document document;
    status = load(body, document, sink);
    EXPECT_EQ(status, Status::ValidationError);
    bool cited = false;
    for (const Diagnostic& diagnostic : sink.diagnostics()) {
        cited = cited ||
                diagnostic.rule_id == "asam.net:xosc:1.0.0:data_type.condition_delay_not_negative";
    }
    EXPECT_TRUE(cited);
}

// --- scenario definition --------------------------------------------------

TEST(ScenarioDefinition, RoadNetworkGoesToTheHostAndSignalsToTheIr) {
    constexpr std::string_view kBody = R"(<RoadNetwork>
      <LogicFile filepath="maps/town.xodr"/>
      <SceneGraphFile filepath="maps/town.osgb"/>
      <TrafficSignals>
        <TrafficSignalController name="ctrl" delay="2.0" reference="other">
          <Phase name="green" duration="10.0">
            <TrafficSignalState trafficSignalId="1" state="green"/>
          </Phase>
        </TrafficSignalController>
      </TrafficSignals>
    </RoadNetwork>)";
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(load(kBody, document, sink), Status::Ok);
    EXPECT_EQ(document.road_network.logic_file, "maps/town.xodr");
    EXPECT_EQ(document.road_network.scene_graph_file, "maps/town.osgb");
    ASSERT_EQ(document.scenario.traffic_signal_controllers.size(), 1U);
    const auto& controller = document.scenario.traffic_signal_controllers.front();
    EXPECT_EQ(controller.name, "ctrl");
    ASSERT_TRUE(controller.delay.has_value());
    EXPECT_DOUBLE_EQ(*controller.delay, 2.0);
    ASSERT_EQ(controller.phases.size(), 1U);
    EXPECT_EQ(controller.phases.front().signal_states.front().state, "green");
}

TEST(ScenarioDefinition, LiteralDeclarationsLowerAndReferencesDefer) {
    constexpr std::string_view kBody = R"(<ParameterDeclarations>
      <ParameterDeclaration name="speed" parameterType="double" value="30.0"/>
      <ParameterDeclaration name="derived" parameterType="double" value="${$speed * 2}"/>
    </ParameterDeclarations>
    <VariableDeclarations>
      <VariableDeclaration name="counter" variableType="int" value="0"/>
    </VariableDeclarations>)";
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(load(kBody, document, sink), Status::Ok);
    ASSERT_EQ(document.scenario.parameters.size(), 1U);
    EXPECT_EQ(document.scenario.parameters.at("speed"), "30.0");
    EXPECT_EQ(document.scenario.variables.at("counter"), "0");
    EXPECT_TRUE(has_message_containing(sink, "p4-s3"));
}

TEST(ScenarioDefinition, CatalogLocationsAreDeferred) {
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(
        load(
            R"(<CatalogLocations><VehicleCatalog><Directory path="./catalogs"/></VehicleCatalog></CatalogLocations>)",
            document, sink),
        Status::Ok);
    EXPECT_TRUE(has_message_containing(sink, "p4-s4"));
}

// --- GS-1 -----------------------------------------------------------------

TEST(GoldenScenario, Gs1LoadsAndRunsThroughTheEngine) {
    const std::filesystem::path path =
        std::filesystem::path(SCENA_TEST_SCENARIO_DIR) / "gs1_cruise.xosc";
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(scena::xml::load_file(path, document, sink), Status::Ok)
        << (errors(sink).empty() ? std::string("no error") : errors(sink).front().message);
    EXPECT_EQ(document.kind, DocumentKind::Scenario);
    EXPECT_EQ(document.road_network.logic_file, "straight.xodr");
    ASSERT_EQ(document.scenario.entities.size(), 1U);
    ASSERT_EQ(document.scenario.init_actions.size(), 2U);

    scena::Engine engine;
    ASSERT_EQ(engine.init(document.scenario), Status::Ok);

    // Init teleport and init speed took effect before simulation time.
    const std::optional<scena::EntityState> initial = engine.state("ego");
    ASSERT_TRUE(initial.has_value());
    EXPECT_DOUBLE_EQ(initial->x, 0.0);
    EXPECT_DOUBLE_EQ(initial->speed, 20.0);

    // 10 s at 0.1 s: the event fires at t = 5 s and the linear 4 s ramp
    // reaches the 30 m/s target.
    for (int step = 0; step < 100; ++step) {
        ASSERT_EQ(engine.step(0.1), Status::Ok);
    }
    const std::optional<scena::EntityState> final_state = engine.state("ego");
    ASSERT_TRUE(final_state.has_value());
    EXPECT_NEAR(final_state->speed, 30.0, 1e-9);
    EXPECT_DOUBLE_EQ(final_state->heading, 0.0); // heading constant, GS-1 pass criterion
    EXPECT_GT(final_state->x, 200.0);
    EXPECT_EQ(engine.close(), Status::Ok);
}

TEST(GoldenScenario, Gs1RunsBitIdenticallyTwice) {
    const std::filesystem::path path =
        std::filesystem::path(SCENA_TEST_SCENARIO_DIR) / "gs1_cruise.xosc";
    const auto run = [&path]() {
        Document document;
        DiagnosticSink sink;
        EXPECT_EQ(scena::xml::load_file(path, document, sink), Status::Ok);
        scena::Engine engine;
        EXPECT_EQ(engine.init(document.scenario), Status::Ok);
        for (int step = 0; step < 100; ++step) {
            EXPECT_EQ(engine.step(0.1), Status::Ok);
        }
        return engine.state("ego").value();
    };
    const scena::EntityState first = run();
    const scena::EntityState second = run();
    // Bit-identical, not merely close: the loader must not introduce any
    // ordering or formatting dependence into the IR it builds.
    EXPECT_EQ(first.x, second.x);
    EXPECT_EQ(first.y, second.y);
    EXPECT_EQ(first.speed, second.speed);
    EXPECT_EQ(first.heading, second.heading);
}

} // namespace
