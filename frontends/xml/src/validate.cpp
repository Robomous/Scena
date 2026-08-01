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

#include "validate.h"

#include <algorithm>
#include <map>
#include <set>
#include <string>
#include <string_view>
#include <vector>

#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/entity_condition.h"
#include "scena/ir/evaluation_context.h"
#include "scena/ir/interaction_condition.h"

namespace scena::xml::detail {

namespace {

constexpr const char* kRuleEntityRef =
    "asam.net:xosc:1.2.0:reference_control.references_to_scenario_object";
constexpr const char* kRuleElementRef =
    "asam.net:xosc:1.0.0:reference_control.resolvable_storyboard_element_ref";
constexpr const char* kRuleVariableRef =
    "asam.net:xosc:1.2.0:reference_control.resolvable_variable_reference";
constexpr const char* kRuleControllerRef =
    "asam.net:xosc:1.0.0:reference_control.traffic_signal_controller_references";
constexpr const char* kRuleUniqueNames =
    "asam.net:xosc:1.1.0:naming.unique_element_names_on_same_level";

/// Everything the pass needs to answer a reference: what exists, and what
/// was referenced.
struct Index {
    std::set<std::string, std::less<>> entities;
    std::set<std::string, std::less<>> variables;
    std::set<std::string, std::less<>> parameters;
    std::set<std::string, std::less<>> signal_controllers;
    /// Storyboard element name to its type, for the element-state condition.
    std::map<std::string, ir::StoryboardElementType, std::less<>> elements;

    std::set<std::string, std::less<>> referenced_variables;
    std::set<std::string, std::less<>> referenced_parameters;
};

void check_entity(ReadContext& ctx, const Index& index, std::string_view reference,
                  std::string_view path) {
    if (reference.empty() || index.entities.count(reference) != 0) {
        return;
    }
    ctx.report(Severity::Error, Status::SemanticError, std::string(path),
               "reference to entity '" + std::string(reference) +
                   "', which the scenario does not "
                   "declare",
               kRuleEntityRef);
}

/// Collects the entity references an action carries, so each can be checked
/// against the declared entities.
void check_action(ReadContext& ctx, Index& index, const ir::Action& action,
                  const std::string& path) {
    check_entity(ctx, index, action.entity_id(), path);

    if (const auto* speed = dynamic_cast<const ir::SpeedAction*>(&action)) {
        if (const std::optional<ir::RelativeTargetSpeed>& target = speed->relative_target()) {
            check_entity(ctx, index, target->entity_ref, path);
        }
    } else if (const auto* distance =
                   dynamic_cast<const ir::LongitudinalDistanceAction*>(&action)) {
        check_entity(ctx, index, distance->entity_ref(), path);
    } else if (const auto* lateral = dynamic_cast<const ir::LateralDistanceAction*>(&action)) {
        check_entity(ctx, index, lateral->entity_ref(), path);
    } else if (const auto* add = dynamic_cast<const ir::AddEntityAction*>(&action)) {
        check_entity(ctx, index, add->entity_ref(), path);
    } else if (const auto* remove = dynamic_cast<const ir::DeleteEntityAction*>(&action)) {
        check_entity(ctx, index, remove->entity_ref(), path);
    } else if (const auto* set_variable = dynamic_cast<const ir::VariableSetAction*>(&action)) {
        index.referenced_variables.insert(set_variable->variable_ref());
        if (index.variables.count(set_variable->variable_ref()) == 0) {
            ctx.report(Severity::Error, Status::SemanticError, path,
                       "reference to variable '" + set_variable->variable_ref() +
                           "', which the scenario does not declare",
                       kRuleVariableRef);
        }
    } else if (const auto* modify = dynamic_cast<const ir::VariableModifyAction*>(&action)) {
        index.referenced_variables.insert(modify->variable_ref());
        if (index.variables.count(modify->variable_ref()) == 0) {
            ctx.report(Severity::Error, Status::SemanticError, path,
                       "reference to variable '" + modify->variable_ref() +
                           "', which the scenario does not declare",
                       kRuleVariableRef);
        }
    } else if (const auto* controller =
                   dynamic_cast<const ir::TrafficSignalControllerAction*>(&action)) {
        if (index.signal_controllers.count(controller->traffic_signal_controller_ref()) == 0) {
            ctx.report(Severity::Error, Status::SemanticError, path,
                       "reference to traffic signal controller '" +
                           controller->traffic_signal_controller_ref() +
                           "', which the scenario does not declare",
                       kRuleControllerRef);
        }
    }
}

void check_condition(ReadContext& ctx, Index& index, const ir::Condition& condition,
                     const std::string& path) {
    if (const auto* by_entity = dynamic_cast<const ir::ByEntityCondition*>(&condition)) {
        for (const std::string& entity : by_entity->triggering_entities().entity_refs) {
            check_entity(ctx, index, entity, path);
        }
    }
    if (const auto* relative_speed = dynamic_cast<const ir::RelativeSpeedCondition*>(&condition)) {
        check_entity(ctx, index, relative_speed->entity_ref(), path);
    } else if (const auto* relative_distance =
                   dynamic_cast<const ir::RelativeDistanceCondition*>(&condition)) {
        check_entity(ctx, index, relative_distance->entity_ref(), path);
    } else if (const auto* headway = dynamic_cast<const ir::TimeHeadwayCondition*>(&condition)) {
        check_entity(ctx, index, headway->entity_ref(), path);
    } else if (const auto* collision = dynamic_cast<const ir::CollisionCondition*>(&condition)) {
        check_entity(ctx, index, collision->entity_ref(), path);
    } else if (const auto* ttc = dynamic_cast<const ir::TimeToCollisionCondition*>(&condition)) {
        if (const auto* entity = std::get_if<std::string>(&ttc->target())) {
            check_entity(ctx, index, *entity, path);
        }
    } else if (const auto* parameter = dynamic_cast<const ir::ParameterCondition*>(&condition)) {
        index.referenced_parameters.insert(parameter->parameter_ref());
        if (index.parameters.count(parameter->parameter_ref()) == 0) {
            ctx.report(Severity::Error, Status::SemanticError, path,
                       "reference to parameter '" + parameter->parameter_ref() +
                           "', which the scenario does not declare",
                       "asam.net:xosc:1.1.0:parameters.parameter_declaration_parameter_scope");
        }
    } else if (const auto* variable = dynamic_cast<const ir::VariableCondition*>(&condition)) {
        index.referenced_variables.insert(variable->variable_ref());
        if (index.variables.count(variable->variable_ref()) == 0) {
            ctx.report(Severity::Error, Status::SemanticError, path,
                       "reference to variable '" + variable->variable_ref() +
                           "', which the scenario does not declare",
                       kRuleVariableRef);
        }
    } else if (const auto* controller =
                   dynamic_cast<const ir::TrafficSignalControllerCondition*>(&condition)) {
        if (index.signal_controllers.count(controller->traffic_signal_controller_ref()) == 0) {
            ctx.report(Severity::Error, Status::SemanticError, path,
                       "reference to traffic signal controller '" +
                           controller->traffic_signal_controller_ref() +
                           "', which the scenario does not declare",
                       kRuleControllerRef);
        }
    } else if (const auto* element =
                   dynamic_cast<const ir::StoryboardElementStateCondition*>(&condition)) {
        const auto found = index.elements.find(element->element_ref());
        if (found == index.elements.end()) {
            ctx.report(Severity::Error, Status::SemanticError, path,
                       "reference to storyboard element '" + element->element_ref() +
                           "', which the scenario does not declare",
                       kRuleElementRef);
        } else if (found->second != element->element_type()) {
            // Naming an element of a different type is the same defect as
            // naming one that does not exist: the condition would never see
            // the state it waits for.
            ctx.report(Severity::Error, Status::SemanticError, path,
                       "storyboard element '" + element->element_ref() +
                           "' is not of the type the condition claims",
                       kRuleElementRef);
        }
    }
}

void check_trigger(ReadContext& ctx, Index& index, const ir::Trigger& trigger,
                   const std::string& path) {
    for (std::size_t group = 0; group < trigger.groups.size(); ++group) {
        for (const ir::TriggerCondition& condition : trigger.groups[group].conditions) {
            if (condition.expression != nullptr) {
                check_condition(ctx, index, *condition.expression,
                                path + "/group[" + std::to_string(group) + "]/" + condition.name);
            }
        }
    }
}

/// Reports duplicate names among siblings, the one naming rule a reader
/// cannot see on its own.
template <typename Named>
void check_unique_names(ReadContext& ctx, const std::vector<Named>& items, const std::string& path,
                        const char* what) {
    std::set<std::string, std::less<>> seen;
    for (const Named& item : items) {
        if (!seen.insert(item.name).second) {
            ctx.report(Severity::Error, Status::ValidationError, path,
                       std::string(what) + " name '" + item.name +
                           "' is used more than once at "
                           "the same level",
                       kRuleUniqueNames);
        }
    }
}

void index_storyboard(Index& index, const ir::Storyboard& storyboard) {
    for (const ir::Story& story : storyboard.stories) {
        index.elements.insert_or_assign(story.name, ir::StoryboardElementType::Story);
        for (const ir::Act& act : story.acts) {
            index.elements.insert_or_assign(act.name, ir::StoryboardElementType::Act);
            for (const ir::ManeuverGroup& group : act.groups) {
                index.elements.insert_or_assign(group.name,
                                                ir::StoryboardElementType::ManeuverGroup);
                for (const ir::Maneuver& maneuver : group.maneuvers) {
                    index.elements.insert_or_assign(maneuver.name,
                                                    ir::StoryboardElementType::Maneuver);
                    for (const ir::Event& event : maneuver.events) {
                        index.elements.insert_or_assign(event.name,
                                                        ir::StoryboardElementType::Event);
                    }
                }
            }
        }
    }
}

} // namespace

void validate_document(ReadContext& ctx, const Document& document) {
    const ir::Scenario& scenario = document.scenario;

    Index index;
    for (const ir::Entity& entity : scenario.entities) {
        index.entities.insert(entity.id);
    }
    for (const std::string& selection : ctx.entity_selection_names()) {
        index.entities.insert(selection);
    }
    for (const auto& [name, value] : scenario.variables) {
        index.variables.insert(name);
    }
    for (const auto& [name, value] : scenario.parameters) {
        index.parameters.insert(name);
    }
    for (const auto& controller : scenario.traffic_signal_controllers) {
        index.signal_controllers.insert(controller.name);
    }
    index_storyboard(index, scenario.storyboard);

    check_unique_names(ctx, scenario.entities, "entities", "entity");
    check_unique_names(ctx, scenario.traffic_signal_controllers, "roadNetwork/trafficSignals",
                       "traffic signal controller");
    check_unique_names(ctx, scenario.storyboard.stories, "storyboard", "story");

    for (std::size_t at = 0; at < scenario.init_actions.size(); ++at) {
        if (scenario.init_actions[at] != nullptr) {
            check_action(ctx, index, *scenario.init_actions[at],
                         "init/action[" + std::to_string(at) + "]");
        }
    }

    for (const ir::Story& story : scenario.storyboard.stories) {
        check_unique_names(ctx, story.acts, story.name, "act");
        for (const ir::Act& act : story.acts) {
            const std::string act_path = story.name + "/" + act.name;
            check_unique_names(ctx, act.groups, act_path, "maneuver group");
            if (act.start_trigger.has_value()) {
                check_trigger(ctx, index, *act.start_trigger, act_path + "/startTrigger");
            }
            if (act.stop_trigger.has_value()) {
                check_trigger(ctx, index, *act.stop_trigger, act_path + "/stopTrigger");
            }
            for (const ir::ManeuverGroup& group : act.groups) {
                const std::string group_path = act_path + "/" + group.name;
                check_unique_names(ctx, group.maneuvers, group_path, "maneuver");
                for (const std::string& actor : group.actors) {
                    check_entity(ctx, index, actor, group_path + "/actors");
                }
                for (const ir::Maneuver& maneuver : group.maneuvers) {
                    const std::string maneuver_path = group_path + "/" + maneuver.name;
                    check_unique_names(ctx, maneuver.events, maneuver_path, "event");
                    for (const ir::Event& event : maneuver.events) {
                        const std::string event_path = maneuver_path + "/" + event.name;
                        if (event.start_trigger.has_value()) {
                            check_trigger(ctx, index, *event.start_trigger,
                                          event_path + "/startTrigger");
                        }
                        for (std::size_t at = 0; at < event.actions.size(); ++at) {
                            if (event.actions[at] != nullptr) {
                                check_action(ctx, index, *event.actions[at],
                                             event_path + "/action[" + std::to_string(at) + "]");
                            }
                        }
                    }
                }
            }
        }
    }
    if (scenario.storyboard.stop_trigger.has_value()) {
        check_trigger(ctx, index, *scenario.storyboard.stop_trigger, "stopTrigger");
    }

    // Unused declarations: harmless to execute, but almost always a typo in
    // the reference rather than a deliberate spare, so a warning names them.
    for (const std::string& name : ctx.referenced_parameters()) {
        index.referenced_parameters.insert(name);
    }
    for (const auto& [name, value] : scenario.parameters) {
        if (index.referenced_parameters.count(name) == 0) {
            ctx.report(Severity::Warning, Status::ValidationError, "parameterDeclarations",
                       "parameter '" + name + "' is declared but never referenced");
        }
    }
    for (const auto& [name, value] : scenario.variables) {
        if (index.referenced_variables.count(name) == 0) {
            ctx.report(Severity::Warning, Status::ValidationError, "variableDeclarations",
                       "variable '" + name + "' is declared but never referenced");
        }
    }
}

} // namespace scena::xml::detail
