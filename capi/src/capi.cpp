// SPDX-License-Identifier: MIT
#include "scena/capi.h"

#include <memory>
#include <new>
#include <string>
#include <utility>

#include "scena/diagnostic.h"
#include "scena/engine.h"
#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/scenario.h"
#include "scena/status.h"
#include "scena/version.h"

struct scn_engine {
    scena::ir::Scenario scenario;
    scena::Engine engine;
};

namespace {

scn_status to_c_status(scena::Status status) {
    switch (status) {
    case scena::Status::Ok:
        return SCN_OK;
    case scena::Status::AlreadyInitialized:
        return SCN_ERROR_ALREADY_INITIALIZED;
    case scena::Status::NotInitialized:
        return SCN_ERROR_NOT_INITIALIZED;
    case scena::Status::UnknownEntity:
        return SCN_ERROR_UNKNOWN_ENTITY;
    case scena::Status::InvalidControlMode:
        return SCN_ERROR_INVALID_CONTROL_MODE;
    case scena::Status::InvalidArgument:
        return SCN_ERROR_INVALID_ARGUMENT;
    case scena::Status::ParseError:
        return SCN_ERROR_PARSE;
    case scena::Status::ValidationError:
        return SCN_ERROR_VALIDATION;
    case scena::Status::SemanticError:
        return SCN_ERROR_SEMANTIC;
    case scena::Status::UnsupportedFeature:
        return SCN_ERROR_UNSUPPORTED_FEATURE;
    case scena::Status::UnknownName:
        return SCN_ERROR_UNKNOWN_NAME;
    }
    return SCN_ERROR_INTERNAL;
}

scn_severity to_c_severity(scena::Severity severity) {
    switch (severity) {
    case scena::Severity::Info:
        return SCN_SEVERITY_INFO;
    case scena::Severity::Warning:
        return SCN_SEVERITY_WARNING;
    case scena::Severity::Error:
        return SCN_SEVERITY_ERROR;
    }
    return SCN_SEVERITY_ERROR;
}

scena::EntityState from_c_state(const scn_entity_state& state) {
    return scena::EntityState{state.x, state.y, state.z, state.heading, state.speed};
}

scn_entity_state to_c_state(const scena::EntityState& state) {
    return scn_entity_state{state.x, state.y, state.z, state.heading, state.speed};
}

} // namespace

const char* scn_version(void) {
    static const std::string version = scena::version_string();
    return version.c_str();
}

scn_engine* scn_engine_create(void) {
    return new (std::nothrow) scn_engine();
}

void scn_engine_destroy(scn_engine* engine) {
    delete engine;
}

scn_status scn_engine_add_entity(scn_engine* engine, const char* id, const char* name,
                                 scn_control_mode control_mode) {
    if (engine == nullptr || id == nullptr || name == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        scena::ir::Entity entity;
        entity.id = id;
        entity.name = name;
        entity.control_mode = control_mode == SCN_CONTROL_HOST
                                  ? scena::ir::ControlMode::HostControlled
                                  : scena::ir::ControlMode::EngineControlled;
        engine->scenario.entities.push_back(std::move(entity));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_add_speed_action(scn_engine* engine, const char* entity_id,
                                       double target_speed, double at_time) {
    return scn_engine_add_speed_action_ex(engine, entity_id, target_speed, at_time,
                                          SCN_PRIORITY_PARALLEL, 1);
}

scn_status scn_engine_add_speed_action_ex(scn_engine* engine, const char* entity_id,
                                          double target_speed, double at_time,
                                          scn_event_priority priority,
                                          int maximum_execution_count) {
    if (engine == nullptr || entity_id == nullptr || maximum_execution_count < 0) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    // The enumeration cannot be trusted across the ABI, so it is range
    // checked here rather than static_cast blindly into the IR enum.
    scena::ir::EventPriority ir_priority = scena::ir::EventPriority::Parallel;
    switch (priority) {
    case SCN_PRIORITY_OVERRIDE:
        ir_priority = scena::ir::EventPriority::Override;
        break;
    case SCN_PRIORITY_PARALLEL:
        ir_priority = scena::ir::EventPriority::Parallel;
        break;
    case SCN_PRIORITY_SKIP:
        ir_priority = scena::ir::EventPriority::Skip;
        break;
    default:
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        // The C builder surface stays flat: each call appends one event
        // (SimulationTimeCondition start trigger + SpeedAction) to a lazily
        // created default Story -> Act -> ManeuverGroup -> Maneuver chain.
        scena::ir::Storyboard& storyboard = engine->scenario.storyboard;
        if (storyboard.stories.empty()) {
            scena::ir::Story story;
            story.name = "story";
            scena::ir::Act act;
            act.name = "act";
            scena::ir::ManeuverGroup group;
            group.name = "group";
            scena::ir::Maneuver maneuver;
            maneuver.name = "maneuver";
            group.maneuvers.push_back(std::move(maneuver));
            act.groups.push_back(std::move(group));
            story.acts.push_back(std::move(act));
            storyboard.stories.push_back(std::move(story));
        }
        scena::ir::Maneuver& maneuver =
            storyboard.stories.front().acts.front().groups.front().maneuvers.front();
        scena::ir::Event event;
        event.name = "event-" + std::to_string(maneuver.events.size() + 1);
        event.start_trigger =
            scena::ir::make_trigger(std::make_shared<scena::ir::SimulationTimeCondition>(at_time));
        event.priority = ir_priority;
        event.maximum_execution_count = maximum_execution_count;
        event.actions.push_back(std::make_shared<scena::ir::SpeedAction>(entity_id, target_speed));
        maneuver.events.push_back(std::move(event));
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_init(scn_engine* engine) {
    if (engine == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        // Init on a copy so the builder scenario stays intact for re-init
        // after close.
        return to_c_status(engine->engine.init(engine->scenario));
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_step(scn_engine* engine, double dt) {
    if (engine == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        return to_c_status(engine->engine.step(dt));
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_close(scn_engine* engine) {
    if (engine == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        return to_c_status(engine->engine.close());
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_diagnostic_count(scn_engine* engine, size_t* out_count) {
    if (engine == nullptr || out_count == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        *out_count = engine->engine.diagnostics().size();
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_diagnostic_at(scn_engine* engine, size_t index, scn_diagnostic* out) {
    if (engine == nullptr || out == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        const auto& diagnostics = engine->engine.diagnostics();
        if (index >= diagnostics.size()) {
            return SCN_ERROR_INVALID_ARGUMENT; // out left untouched
        }
        const scena::Diagnostic& diagnostic = diagnostics[index];
        // Strings borrow from the diagnostic's std::strings, which live in the
        // engine and are stable until the next mutating call (see capi.h).
        out->severity = to_c_severity(diagnostic.severity);
        out->code = to_c_status(diagnostic.code);
        out->message = diagnostic.message.c_str();
        out->path = diagnostic.path.c_str();
        out->file = diagnostic.location.file.c_str();
        out->line = diagnostic.location.line;
        out->column = diagnostic.location.column;
        out->rule_id = diagnostic.rule_id.c_str();
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_clear_diagnostics(scn_engine* engine) {
    if (engine == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        engine->engine.clear_diagnostics();
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_get_state(scn_engine* engine, const char* entity_id, scn_entity_state* out) {
    if (engine == nullptr || entity_id == nullptr || out == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        if (!engine->engine.initialized()) {
            return SCN_ERROR_NOT_INITIALIZED;
        }
        const auto state = engine->engine.state(entity_id);
        if (!state.has_value()) {
            return SCN_ERROR_UNKNOWN_ENTITY;
        }
        *out = to_c_state(*state);
        return SCN_OK;
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}

scn_status scn_engine_report_state(scn_engine* engine, const char* entity_id,
                                   const scn_entity_state* state) {
    if (engine == nullptr || entity_id == nullptr || state == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        return to_c_status(engine->engine.report_state(entity_id, from_c_state(*state)));
    } catch (...) {
        return SCN_ERROR_INTERNAL;
    }
}
