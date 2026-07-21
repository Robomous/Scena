// SPDX-License-Identifier: MIT
#include "scena/engine.h"

#include <cmath>
#include <utility>

#include "scena/gateway/simulator_gateway.h"

namespace scena {

Engine::Engine(gateway::ISimulatorGateway* gateway) : gateway_(gateway) {}

Status Engine::init(ir::Scenario scenario) {
    if (initialized_) {
        return Status::AlreadyInitialized;
    }

    // Validate into a temporary record map so a failed init leaves the engine
    // untouched.
    std::map<std::string, EntityRecord> records;
    for (const ir::Entity& entity : scenario.entities) {
        if (entity.id.empty()) {
            return Status::InvalidArgument;
        }
        auto [it, inserted] = records.try_emplace(entity.id);
        if (!inserted) {
            return Status::InvalidArgument; // duplicate entity id
        }
        it->second.mode = entity.control_mode;
    }
    for (const ir::StoryboardEntry& entry : scenario.storyboard.entries) {
        if (entry.condition == nullptr || entry.action == nullptr) {
            return Status::InvalidArgument;
        }
        if (records.find(entry.action->entity_id()) == records.end()) {
            return Status::UnknownEntity;
        }
    }

    scenario_ = std::move(scenario);
    entities_ = std::move(records);
    clock_.reset();
    scheduler_.bind(scenario_.storyboard);
    initialized_ = true;
    return Status::Ok;
}

Status Engine::step(double dt) {
    if (!initialized_) {
        return Status::NotInitialized;
    }
    if (!(dt >= 0.0)) { // rejects negative values and NaN
        return Status::InvalidArgument;
    }

    clock_.advance(dt);

    if (gateway_ != nullptr) {
        for (auto& [id, record] : entities_) {
            if (record.mode == ir::ControlMode::HostControlled) {
                EntityState polled;
                if (gateway_->poll_state(id, polled)) {
                    record.state = polled;
                }
            }
        }
    }

    scheduler_.step(clock_.now(), [this](const ir::Action& action) { apply(action); });

    for (auto& [id, record] : entities_) {
        (void)id;
        if (record.mode == ir::ControlMode::EngineControlled) {
            // Straight-line kinematics: placeholder physics for this phase.
            record.state.x += record.state.speed * std::cos(record.state.heading) * dt;
            record.state.y += record.state.speed * std::sin(record.state.heading) * dt;
        }
    }

    if (gateway_ != nullptr) {
        for (const auto& [id, record] : entities_) {
            if (record.mode == ir::ControlMode::EngineControlled) {
                gateway_->publish_state(id, record.state);
            }
        }
    }

    return Status::Ok;
}

std::optional<EntityState> Engine::state(const std::string& entity_id) const {
    const auto it = entities_.find(entity_id);
    if (it == entities_.end()) {
        return std::nullopt;
    }
    return it->second.state;
}

Status Engine::report_state(const std::string& entity_id, const EntityState& state) {
    if (!initialized_) {
        return Status::NotInitialized;
    }
    const auto it = entities_.find(entity_id);
    if (it == entities_.end()) {
        return Status::UnknownEntity;
    }
    if (it->second.mode != ir::ControlMode::HostControlled) {
        return Status::InvalidControlMode;
    }
    it->second.state = state;
    return Status::Ok;
}

double Engine::time() const noexcept {
    return clock_.now();
}

bool Engine::initialized() const noexcept {
    return initialized_;
}

Status Engine::close() {
    if (!initialized_) {
        return Status::NotInitialized;
    }
    scheduler_.reset();
    clock_.reset();
    entities_.clear();
    scenario_ = ir::Scenario{};
    initialized_ = false;
    return Status::Ok;
}

void Engine::apply(const ir::Action& action) {
    if (const auto* speed_action = dynamic_cast<const ir::SpeedAction*>(&action)) {
        const auto it = entities_.find(speed_action->entity_id());
        if (it != entities_.end()) {
            it->second.state.speed = speed_action->target_speed();
        }
    }
    // Unknown action kinds are ignored in this phase.
}

} // namespace scena
