// SPDX-License-Identifier: MIT
#include "scena/engine.h"

#include <cmath>
#include <optional>
#include <set>
#include <utility>

#include "scena/gateway/simulator_gateway.h"

namespace scena {

namespace {

/// True when every name in the range is non-empty and unique among its
/// siblings — element names address the storyboard state query, so they are
/// part of the runtime contract.
template <typename Range, typename NameOf> bool names_valid(const Range& range, NameOf name_of) {
    std::set<std::string> seen;
    for (const auto& element : range) {
        const std::string& name = name_of(element);
        if (name.empty() || !seen.insert(name).second) {
            return false;
        }
    }
    return true;
}

/// Validates one trigger site. The scheduler evaluates triggers on the hot
/// path and relies on these guarantees instead of re-checking them, so
/// everything it assumes is established here. An absent trigger is always
/// valid, and so is an engaged but empty one (§7.6.1: always false).
Status validate_trigger(const std::optional<ir::Trigger>& trigger) {
    if (!trigger.has_value()) {
        return Status::Ok;
    }
    for (const ir::ConditionGroup& group : trigger->groups) {
        // A condition group holds 1..* conditions (class reference,
        // ConditionGroup). An empty one would be a vacuously true
        // conjunction — a meaning the standard does not give it — so it is
        // rejected rather than silently made always-true.
        if (group.conditions.empty()) {
            return Status::InvalidArgument;
        }
        for (const ir::TriggerCondition& condition : group.conditions) {
            if (condition.expression == nullptr) {
                return Status::InvalidArgument;
            }
            // per rule asam.net:xosc:1.0.0:data_type.condition_delay_not_negative
            // ("The condition delay shall be non negative"). The negated
            // comparison also rejects NaN.
            if (!(condition.delay >= 0.0)) {
                return Status::InvalidArgument;
            }
        }
    }
    return Status::Ok;
}

/// Validates the storyboard tree: element naming, triggers, non-null
/// actions, and action targets that exist in `records`. Returns Status::Ok
/// when valid.
template <typename Records>
Status validate_storyboard(const ir::Storyboard& storyboard, const Records& records) {
    if (!names_valid(storyboard.stories, [](const ir::Story& s) { return s.name; })) {
        return Status::InvalidArgument;
    }
    if (const Status status = validate_trigger(storyboard.stop_trigger); status != Status::Ok) {
        return status;
    }
    for (const ir::Story& story : storyboard.stories) {
        if (!names_valid(story.acts, [](const ir::Act& a) { return a.name; })) {
            return Status::InvalidArgument;
        }
        for (const ir::Act& act : story.acts) {
            if (!names_valid(act.groups, [](const ir::ManeuverGroup& g) { return g.name; })) {
                return Status::InvalidArgument;
            }
            if (const Status status = validate_trigger(act.start_trigger); status != Status::Ok) {
                return status;
            }
            if (const Status status = validate_trigger(act.stop_trigger); status != Status::Ok) {
                return status;
            }
            for (const ir::ManeuverGroup& group : act.groups) {
                if (!names_valid(group.maneuvers, [](const ir::Maneuver& m) { return m.name; })) {
                    return Status::InvalidArgument;
                }
                for (const std::string& actor : group.actors) {
                    if (records.find(actor) == records.end()) {
                        return Status::UnknownEntity;
                    }
                }
                for (const ir::Maneuver& maneuver : group.maneuvers) {
                    if (!names_valid(maneuver.events, [](const ir::Event& e) { return e.name; })) {
                        return Status::InvalidArgument;
                    }
                    for (const ir::Event& event : maneuver.events) {
                        if (const Status status = validate_trigger(event.start_trigger);
                            status != Status::Ok) {
                            return status;
                        }
                        // §8.3.3.2/§8.4.2.1: the XSD type of
                        // maximumExecutionCount is unsignedInt, so a
                        // negative budget has no meaning at all. Zero is
                        // schema-valid and is accepted — §8.4.2.1 already
                        // gives it a coherent reading (the event is
                        // exhausted in standbyState and completes with a
                        // skipTransition without ever executing), so there
                        // is nothing to invent. The standard defines no
                        // rule id for this constraint.
                        if (event.maximum_execution_count < 0) {
                            return Status::InvalidArgument;
                        }
                        if (event.actions.empty()) {
                            return Status::InvalidArgument;
                        }
                        for (const std::shared_ptr<ir::Action>& action : event.actions) {
                            if (action == nullptr) {
                                return Status::InvalidArgument;
                            }
                            if (records.find(action->entity_id()) == records.end()) {
                                return Status::UnknownEntity;
                            }
                        }
                    }
                }
            }
        }
    }
    return Status::Ok;
}

} // namespace

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
    for (const std::shared_ptr<ir::Action>& action : scenario.init_actions) {
        if (action == nullptr) {
            return Status::InvalidArgument;
        }
        if (records.find(action->entity_id()) == records.end()) {
            return Status::UnknownEntity;
        }
    }
    if (const Status status = validate_storyboard(scenario.storyboard, records);
        status != Status::Ok) {
        return status;
    }

    scenario_ = std::move(scenario);
    entities_ = std::move(records);
    clock_.reset();
    initialized_ = true;

    // Init phase (§8.5): init actions are applied before simulation time
    // starts. All actions are instantaneous in this phase, so each one
    // completes during init; their document order does not imply an
    // execution order, but applying them in that order is deterministic.
    for (const std::shared_ptr<ir::Action>& action : scenario_.init_actions) {
        apply(*action);
    }

    // The storyboard enters runningState and simulation time starts with it
    // (§8.4.7): evaluate once at t = 0 so trigger-less chains and start
    // conditions that already hold fire before the first host step.
    scheduler_.bind(scenario_.storyboard);
    scheduler_.step(0.0, [this](const ir::Action& action) { return apply(action); });

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

    scheduler_.step(clock_.now(), [this](const ir::Action& action) { return apply(action); });

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

std::optional<runtime::ElementState>
Engine::storyboard_element_state(const std::string& path) const {
    if (!initialized_) {
        return std::nullopt;
    }
    return scheduler_.element_state(path);
}

std::optional<runtime::TransitionKind>
Engine::storyboard_element_transition(const std::string& path) const {
    if (!initialized_) {
        return std::nullopt;
    }
    return scheduler_.element_transition(path);
}

const std::vector<Diagnostic>& Engine::diagnostics() const noexcept {
    return diagnostics_.diagnostics();
}

void Engine::clear_diagnostics() noexcept {
    diagnostics_.clear();
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

runtime::ActionOutcome Engine::apply(const ir::Action& action) {
    if (const auto* speed_action = dynamic_cast<const ir::SpeedAction*>(&action)) {
        const auto it = entities_.find(speed_action->entity_id());
        if (it != entities_.end()) {
            it->second.state.speed = speed_action->target_speed();
        }
    }
    // Unknown action kinds are ignored in this phase.

    // Every action the engine can apply sets a state instantaneously, so it
    // reaches its goal in the evaluation it was applied in (§7.4.1.2). The
    // engine will report other outcomes once it gains actions whose end is
    // governed by transition dynamics (p2-s2); until then an event driven
    // through Engine always completes in one evaluation, exactly as before.
    return runtime::ActionOutcome::Complete;
}

} // namespace scena
