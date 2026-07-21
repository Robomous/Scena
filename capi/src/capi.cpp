// SPDX-License-Identifier: MIT
#include "scena/capi.h"

#include <memory>
#include <new>
#include <string>
#include <utility>

#include "scena/engine.h"
#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/scenario.h"
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
    }
    return SCN_ERROR_INTERNAL;
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
    if (engine == nullptr || entity_id == nullptr) {
        return SCN_ERROR_INVALID_ARGUMENT;
    }
    try {
        engine->scenario.storyboard.entries.push_back(
            {std::make_shared<scena::ir::SimulationTimeCondition>(at_time),
             std::make_shared<scena::ir::SpeedAction>(entity_id, target_speed)});
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
