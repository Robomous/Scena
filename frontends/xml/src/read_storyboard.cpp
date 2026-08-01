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

#include "read_storyboard.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "catalog.h"
#include "parameters.h"
#include "read_actions.h"
#include "read_common.h"
#include "read_conditions.h"
#include "scena/ir/storyboard.h"

namespace scena::xml::detail {

namespace {

constexpr std::initializer_list<EnumEntry<ir::EventPriority>> kPriorities = {
    {"override", ir::EventPriority::Override},
    {"parallel", ir::EventPriority::Parallel},
    {"skip", ir::EventPriority::Skip},
    // Pre-1.3 spelling whose normative description is word for word the one
    // of `override`, so it is a lexical synonym, not a fourth priority
    // (ADR-0005).
    {"overwrite", ir::EventPriority::Override},
};

/// Reads the Init phase (§8.5): global and user-defined actions first, then
/// the per-entity private actions, all in document order. Every init action
/// is applied concurrently during Engine::init, so the order only decides how
/// the IR reads back, never when an action takes effect.
void read_init(ReadContext& ctx, const pugi::xml_node& node, ir::Scenario& out) {
    static const char* const kConsumed[] = {"Actions", nullptr};
    warn_unconsumed_children(ctx, node, kConsumed);

    const pugi::xml_node actions = node.child("Actions");
    if (!actions) {
        return; // An Init with no actions is legal and does nothing.
    }
    static const char* const kActionsConsumed[] = {"GlobalAction", "UserDefinedAction", "Private",
                                                   nullptr};
    warn_unconsumed_children(ctx, actions, kActionsConsumed);

    for (pugi::xml_node global : actions.children("GlobalAction")) {
        if (std::shared_ptr<ir::Action> action = read_global_action(ctx, global)) {
            out.init_actions.push_back(std::move(action));
        }
    }
    for (pugi::xml_node user_defined : actions.children("UserDefinedAction")) {
        if (std::shared_ptr<ir::Action> action = read_global_action(ctx, user_defined)) {
            out.init_actions.push_back(std::move(action));
        }
    }
    for (pugi::xml_node private_node : actions.children("Private")) {
        std::string entity_ref;
        if (!require_string(ctx, private_node, "entityRef", entity_ref)) {
            continue;
        }
        static const char* const kPrivateConsumed[] = {"PrivateAction", nullptr};
        warn_unconsumed_children(ctx, private_node, kPrivateConsumed);
        for (pugi::xml_node action_node : private_node.children("PrivateAction")) {
            if (std::shared_ptr<ir::Action> action =
                    read_private_action(ctx, action_node, entity_ref)) {
                out.init_actions.push_back(std::move(action));
            }
        }
    }
}

/// Reads one `Action` of an Event: a private, global or user-defined action.
/// The actor of a private action is the ManeuverGroup's actor list, so a
/// private action lowers once per actor — the bulk application of §8.3.3.3.
void read_event_action(ReadContext& ctx, const pugi::xml_node& node,
                       const std::vector<std::string>& actors, ir::Event& out) {
    static const char* const kVariants[] = {"GlobalAction", "UserDefinedAction", "PrivateAction",
                                            nullptr};
    const pugi::xml_node variant = read_choice(ctx, node, kVariants);
    if (!variant) {
        return;
    }
    const std::string_view name = variant.name();
    if (name != "PrivateAction") {
        if (std::shared_ptr<ir::Action> action = read_global_action(ctx, variant)) {
            out.actions.push_back(std::move(action));
        }
        return;
    }
    if (actors.empty()) {
        ctx.report_at(variant, Severity::Error, Status::ValidationError, element_path(variant),
                      "a private action needs at least one actor in its ManeuverGroup");
        return;
    }
    for (const std::string& actor : actors) {
        if (std::shared_ptr<ir::Action> action = read_private_action(ctx, variant, actor)) {
            out.actions.push_back(std::move(action));
        }
    }
}

bool read_event(ReadContext& ctx, const pugi::xml_node& node,
                const std::vector<std::string>& actors, ir::Event& out) {
    static const char* const kConsumed[] = {"StartTrigger", "Action", nullptr};
    warn_unconsumed_children(ctx, node, kConsumed);

    bool ok = require_string(ctx, node, "name", out.name);
    ok = read_enum(ctx, node, "priority", kPriorities, out.priority) && ok;
    ok = optional_int(ctx, node, "maximumExecutionCount", out.maximum_execution_count) && ok;

    if (const pugi::xml_node trigger = node.child("StartTrigger")) {
        // Absent start trigger: the event starts with its parent (§8.4.2);
        // that is the absent optional, not an empty Trigger.
        out.start_trigger = read_trigger(ctx, trigger);
        if (!out.start_trigger.has_value()) {
            ok = false;
        }
    }
    for (pugi::xml_node action : node.children("Action")) {
        read_event_action(ctx, action, actors, out);
    }
    if (out.actions.empty()) {
        ctx.report_at(node, Severity::Error, Status::ValidationError, element_path(node),
                      "Event declares no action");
        ok = false;
    }
    return ok;
}

bool read_maneuver(ReadContext& ctx, const pugi::xml_node& node,
                   const std::vector<std::string>& actors, ir::Maneuver& out) {
    static const char* const kConsumed[] = {"ParameterDeclarations", "Event", nullptr};
    warn_unconsumed_children(ctx, node, kConsumed);
    // A Maneuver's declarations are visible in every Event, Action and
    // Trigger below it, and nowhere else (§9.1).
    const ParameterFrame frame(ctx.parameters());
    if (const pugi::xml_node declarations = node.child("ParameterDeclarations")) {
        read_parameter_declarations(ctx, declarations);
    }

    bool ok = require_string(ctx, node, "name", out.name);
    for (pugi::xml_node event_node : node.children("Event")) {
        ir::Event event;
        if (read_event(ctx, event_node, actors, event)) {
            out.events.push_back(std::move(event));
        } else {
            ok = false;
        }
    }
    if (out.events.empty()) {
        ctx.report_at(node, Severity::Error, Status::ValidationError, element_path(node),
                      "Maneuver declares no event");
        ok = false;
    }
    return ok;
}

bool read_maneuver_group(ReadContext& ctx, const pugi::xml_node& node, ir::ManeuverGroup& out) {
    static const char* const kConsumed[] = {"Actors", "CatalogReference", "Maneuver", nullptr};
    warn_unconsumed_children(ctx, node, kConsumed);

    bool ok = require_string(ctx, node, "name", out.name);
    if (const pugi::xml_attribute count = node.attribute("maximumExecutionCount")) {
        // The group's own execution count (§8.4.4) has no IR field yet; a
        // document that asks for anything but a single execution is told the
        // count is not honoured rather than silently executed once.
        if (std::string_view(count.value()) != "1") {
            ctx.report_at(node, Severity::Warning, Status::UnsupportedFeature,
                          attribute_path(node, "maximumExecutionCount"),
                          "ManeuverGroup maximumExecutionCount is not honoured yet; the group "
                          "executes once");
        }
    }

    const pugi::xml_node actors = require_child(ctx, node, "Actors");
    if (!actors) {
        return false;
    }
    bool select_triggering = false;
    ok = optional_bool(ctx, actors, "selectTriggeringEntities", select_triggering) && ok;
    if (select_triggering) {
        // The actor set would be the triggering entities of the Act's start
        // trigger, which is decided at run time; the IR names actors
        // statically.
        warn_out_of_scope(ctx, actors,
                          "selectTriggeringEntities needs a run-time actor set (deferred)");
    }
    for (pugi::xml_node entity : actors.children("EntityRef")) {
        std::string entity_ref;
        if (!require_string(ctx, entity, "entityRef", entity_ref)) {
            ok = false;
            continue;
        }
        // An actor may be an EntitySelection (§7.2.2.2: "all private actions
        // within the maneuver group are applied individually, to each
        // ScenarioObject instance in the entity selection"), so a selection
        // expands here into its members.
        if (const std::vector<std::string>* selected = ctx.entity_selection(entity_ref)) {
            for (const std::string& member : *selected) {
                out.actors.push_back(member);
            }
            continue;
        }
        out.actors.push_back(std::move(entity_ref));
    }
    for (pugi::xml_node reference : node.children("CatalogReference")) {
        const pugi::xml_node entry =
            ctx.catalogs().resolve(ctx, reference, CatalogKind::Maneuver, "Maneuver");
        if (!entry) {
            ok = false;
            continue;
        }
        const CatalogEntryScope scope(ctx, reference, entry);
        ir::Maneuver maneuver;
        if (read_maneuver(ctx, entry, out.actors, maneuver)) {
            out.maneuvers.push_back(std::move(maneuver));
        } else {
            ok = false;
        }
    }
    for (pugi::xml_node maneuver_node : node.children("Maneuver")) {
        ir::Maneuver maneuver;
        if (read_maneuver(ctx, maneuver_node, out.actors, maneuver)) {
            out.maneuvers.push_back(std::move(maneuver));
        } else {
            ok = false;
        }
    }
    return ok;
}

bool read_act(ReadContext& ctx, const pugi::xml_node& node, ir::Act& out) {
    static const char* const kConsumed[] = {"ManeuverGroup", "StartTrigger", "StopTrigger",
                                            nullptr};
    warn_unconsumed_children(ctx, node, kConsumed);

    bool ok = require_string(ctx, node, "name", out.name);
    if (const pugi::xml_node trigger = node.child("StartTrigger")) {
        out.start_trigger = read_trigger(ctx, trigger);
        if (!out.start_trigger.has_value()) {
            ok = false;
        }
    }
    if (const pugi::xml_node trigger = node.child("StopTrigger")) {
        out.stop_trigger = read_trigger(ctx, trigger);
        if (!out.stop_trigger.has_value()) {
            ok = false;
        }
    }
    for (pugi::xml_node group_node : node.children("ManeuverGroup")) {
        ir::ManeuverGroup group;
        if (read_maneuver_group(ctx, group_node, group)) {
            out.groups.push_back(std::move(group));
        } else {
            ok = false;
        }
    }
    if (out.groups.empty()) {
        ctx.report_at(node, Severity::Error, Status::ValidationError, element_path(node),
                      "Act declares no maneuver group");
        ok = false;
    }
    return ok;
}

bool read_story(ReadContext& ctx, const pugi::xml_node& node, ir::Story& out) {
    static const char* const kConsumed[] = {"ParameterDeclarations", "Act", nullptr};
    warn_unconsumed_children(ctx, node, kConsumed);
    const ParameterFrame frame(ctx.parameters());
    if (const pugi::xml_node declarations = node.child("ParameterDeclarations")) {
        read_parameter_declarations(ctx, declarations);
    }

    bool ok = require_string(ctx, node, "name", out.name);
    for (pugi::xml_node act_node : node.children("Act")) {
        ir::Act act;
        if (read_act(ctx, act_node, act)) {
            out.acts.push_back(std::move(act));
        } else {
            ok = false;
        }
    }
    if (out.acts.empty()) {
        ctx.report_at(node, Severity::Error, Status::ValidationError, element_path(node),
                      "Story declares no act");
        ok = false;
    }
    return ok;
}

} // namespace

void read_storyboard(ReadContext& ctx, const pugi::xml_node& node, ir::Scenario& out) {
    static const char* const kConsumed[] = {"Init", "Story", "StopTrigger", nullptr};
    warn_unconsumed_children(ctx, node, kConsumed);

    if (const pugi::xml_node init = node.child("Init")) {
        read_init(ctx, init, out);
    }
    for (pugi::xml_node story_node : node.children("Story")) {
        ir::Story story;
        if (read_story(ctx, story_node, story)) {
            out.storyboard.stories.push_back(std::move(story));
        }
    }
    if (const pugi::xml_node trigger = node.child("StopTrigger")) {
        // A storyboard without a stop trigger never completes on its own
        // (§8.4.7), which is the absent optional.
        out.storyboard.stop_trigger = read_trigger(ctx, trigger);
    }
}

} // namespace scena::xml::detail
