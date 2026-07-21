// SPDX-License-Identifier: MIT
#include "kinema/capi.h"

#include <memory>
#include <new>
#include <string>
#include <utility>

#include "kinema/engine.h"
#include "kinema/ir/action.h"
#include "kinema/ir/condition.h"
#include "kinema/ir/scenario.h"
#include "kinema/version.h"

struct knm_engine {
    kinema::ir::Scenario scenario;
    kinema::Engine engine;
};

namespace {

knm_status to_c_status(kinema::Status status) {
    switch (status) {
    case kinema::Status::Ok:
        return KNM_OK;
    case kinema::Status::AlreadyInitialized:
        return KNM_ERROR_ALREADY_INITIALIZED;
    case kinema::Status::NotInitialized:
        return KNM_ERROR_NOT_INITIALIZED;
    case kinema::Status::UnknownEntity:
        return KNM_ERROR_UNKNOWN_ENTITY;
    case kinema::Status::InvalidControlMode:
        return KNM_ERROR_INVALID_CONTROL_MODE;
    case kinema::Status::InvalidArgument:
        return KNM_ERROR_INVALID_ARGUMENT;
    }
    return KNM_ERROR_INTERNAL;
}

kinema::EntityState from_c_state(const knm_entity_state& state) {
    return kinema::EntityState{state.x, state.y, state.z, state.heading, state.speed};
}

knm_entity_state to_c_state(const kinema::EntityState& state) {
    return knm_entity_state{state.x, state.y, state.z, state.heading, state.speed};
}

} // namespace

const char* knm_version(void) {
    static const std::string version = kinema::version_string();
    return version.c_str();
}

knm_engine* knm_engine_create(void) {
    return new (std::nothrow) knm_engine();
}

void knm_engine_destroy(knm_engine* engine) {
    delete engine;
}

knm_status knm_engine_add_entity(knm_engine* engine, const char* id, const char* name,
                                 knm_control_mode control_mode) {
    if (engine == nullptr || id == nullptr || name == nullptr) {
        return KNM_ERROR_INVALID_ARGUMENT;
    }
    try {
        kinema::ir::Entity entity;
        entity.id = id;
        entity.name = name;
        entity.control_mode = control_mode == KNM_CONTROL_HOST
                                  ? kinema::ir::ControlMode::HostControlled
                                  : kinema::ir::ControlMode::EngineControlled;
        engine->scenario.entities.push_back(std::move(entity));
        return KNM_OK;
    } catch (...) {
        return KNM_ERROR_INTERNAL;
    }
}

knm_status knm_engine_add_speed_action(knm_engine* engine, const char* entity_id,
                                       double target_speed, double at_time) {
    if (engine == nullptr || entity_id == nullptr) {
        return KNM_ERROR_INVALID_ARGUMENT;
    }
    try {
        engine->scenario.storyboard.entries.push_back(
            {std::make_shared<kinema::ir::SimulationTimeCondition>(at_time),
             std::make_shared<kinema::ir::SpeedAction>(entity_id, target_speed)});
        return KNM_OK;
    } catch (...) {
        return KNM_ERROR_INTERNAL;
    }
}

knm_status knm_engine_init(knm_engine* engine) {
    if (engine == nullptr) {
        return KNM_ERROR_INVALID_ARGUMENT;
    }
    try {
        // Init on a copy so the builder scenario stays intact for re-init
        // after close.
        return to_c_status(engine->engine.init(engine->scenario));
    } catch (...) {
        return KNM_ERROR_INTERNAL;
    }
}

knm_status knm_engine_step(knm_engine* engine, double dt) {
    if (engine == nullptr) {
        return KNM_ERROR_INVALID_ARGUMENT;
    }
    try {
        return to_c_status(engine->engine.step(dt));
    } catch (...) {
        return KNM_ERROR_INTERNAL;
    }
}

knm_status knm_engine_close(knm_engine* engine) {
    if (engine == nullptr) {
        return KNM_ERROR_INVALID_ARGUMENT;
    }
    try {
        return to_c_status(engine->engine.close());
    } catch (...) {
        return KNM_ERROR_INTERNAL;
    }
}

knm_status knm_engine_get_state(knm_engine* engine, const char* entity_id, knm_entity_state* out) {
    if (engine == nullptr || entity_id == nullptr || out == nullptr) {
        return KNM_ERROR_INVALID_ARGUMENT;
    }
    try {
        if (!engine->engine.initialized()) {
            return KNM_ERROR_NOT_INITIALIZED;
        }
        const auto state = engine->engine.state(entity_id);
        if (!state.has_value()) {
            return KNM_ERROR_UNKNOWN_ENTITY;
        }
        *out = to_c_state(*state);
        return KNM_OK;
    } catch (...) {
        return KNM_ERROR_INTERNAL;
    }
}

knm_status knm_engine_report_state(knm_engine* engine, const char* entity_id,
                                   const knm_entity_state* state) {
    if (engine == nullptr || entity_id == nullptr || state == nullptr) {
        return KNM_ERROR_INVALID_ARGUMENT;
    }
    try {
        return to_c_status(engine->engine.report_state(entity_id, from_c_state(*state)));
    } catch (...) {
        return KNM_ERROR_INTERNAL;
    }
}
