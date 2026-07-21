// SPDX-License-Identifier: MIT
#include "scena/engine.h"

#include <cmath>
#include <cstddef>
#include <optional>
#include <set>
#include <string>
#include <utility>

#include "scena/gateway/simulator_gateway.h"

namespace scena {

namespace {

// Checker rule UIDs quoted from ASAM OpenSCENARIO XML 1.4.0 Annex C. Only
// constraints the standard actually names carry a rule id; the rest emit an
// empty rule_id.
constexpr const char* kRuleUniqueNames =
    "asam.net:xosc:1.0.0:naming.unique_element_names_on_same_level";
constexpr const char* kRuleConditionDelay =
    "asam.net:xosc:1.0.0:data_type.condition_delay_not_negative";

/// Appends one Error diagnostic to `sink`.
void error(DiagnosticSink& sink, Status code, std::string message, std::string path,
           std::string rule_id = {}) {
    Diagnostic diagnostic;
    diagnostic.severity = Severity::Error;
    diagnostic.code = code;
    diagnostic.message = std::move(message);
    diagnostic.path = std::move(path);
    diagnostic.rule_id = std::move(rule_id);
    sink.report(std::move(diagnostic));
}

std::string join_path(const std::string& prefix, const std::string& child) {
    return prefix.empty() ? child : prefix + "/" + child;
}

/// Reports empty and sibling-duplicate names in `range`. Element names
/// address the storyboard state query, so they are part of the runtime
/// contract: they must be non-empty and unique among siblings
/// (asam.net:xosc:1.0.0:naming.unique_element_names_on_same_level). `label`
/// names the element kind for the message; `parent_path` is the path of the
/// enclosing element (empty at the storyboard root).
template <typename Range, typename NameOf>
void check_sibling_names(const Range& range, NameOf name_of, const char* label,
                         const std::string& parent_path, DiagnosticSink& sink) {
    std::set<std::string> seen;
    std::size_t index = 0;
    for (const auto& element : range) {
        const std::string& name = name_of(element);
        if (name.empty()) {
            error(sink, Status::ValidationError, std::string(label) + " name is empty",
                  join_path(parent_path, "[" + std::to_string(index) + "]"));
        } else if (!seen.insert(name).second) {
            error(sink, Status::ValidationError,
                  "duplicate " + std::string(label) + " name '" + name + "'",
                  join_path(parent_path, name), kRuleUniqueNames);
        }
        ++index;
    }
}

/// Validates one trigger site, appending a diagnostic for every defect. The
/// scheduler evaluates triggers on the hot path and relies on these
/// guarantees instead of re-checking them, so everything it assumes is
/// established here. An absent trigger is always valid, and so is an engaged
/// but empty one (§7.6.1: always false). `owner_path` is the element that
/// hosts the trigger (empty for the storyboard); `trigger_kind` is
/// "startTrigger" or "stopTrigger".
void validate_trigger(const std::optional<ir::Trigger>& trigger, const std::string& owner_path,
                      const char* trigger_kind, DiagnosticSink& sink) {
    if (!trigger.has_value()) {
        return;
    }
    const std::string base = join_path(owner_path, trigger_kind);
    std::size_t group_index = 0;
    for (const ir::ConditionGroup& group : trigger->groups) {
        const std::string group_path = base + "/group[" + std::to_string(group_index) + "]";
        ++group_index;
        // A condition group holds 1..* conditions (class reference,
        // ConditionGroup). An empty one would be a vacuously true
        // conjunction — a meaning the standard does not give it — so it is
        // rejected rather than silently made always-true.
        if (group.conditions.empty()) {
            error(sink, Status::ValidationError, "condition group is empty", group_path);
            continue;
        }
        std::size_t condition_index = 0;
        for (const ir::TriggerCondition& condition : group.conditions) {
            const std::string condition_path =
                group_path + "/" +
                (condition.name.empty() ? "condition[" + std::to_string(condition_index) + "]"
                                        : condition.name);
            ++condition_index;
            if (condition.expression == nullptr) {
                error(sink, Status::ValidationError, "condition expression is null",
                      condition_path);
            }
            // per rule asam.net:xosc:1.0.0:data_type.condition_delay_not_negative
            // ("The condition delay shall be non negative"). The negated
            // comparison also rejects NaN.
            if (!(condition.delay >= 0.0)) {
                error(sink, Status::ValidationError, "condition delay is negative", condition_path,
                      kRuleConditionDelay);
            }
        }
    }
}

/// Validates the storyboard tree in document order: element naming,
/// triggers, non-null actions, and action targets that exist in `records`.
/// Every defect is appended to `sink`; the walk never stops early.
template <typename Records>
void validate_storyboard(const ir::Storyboard& storyboard, const Records& records,
                         DiagnosticSink& sink) {
    check_sibling_names(
        storyboard.stories, [](const ir::Story& s) { return s.name; }, "story", "", sink);
    validate_trigger(storyboard.stop_trigger, "", "stopTrigger", sink);
    for (const ir::Story& story : storyboard.stories) {
        const std::string story_path = story.name;
        check_sibling_names(
            story.acts, [](const ir::Act& a) { return a.name; }, "act", story_path, sink);
        for (const ir::Act& act : story.acts) {
            const std::string act_path = join_path(story_path, act.name);
            check_sibling_names(
                act.groups, [](const ir::ManeuverGroup& g) { return g.name; }, "maneuver group",
                act_path, sink);
            validate_trigger(act.start_trigger, act_path, "startTrigger", sink);
            validate_trigger(act.stop_trigger, act_path, "stopTrigger", sink);
            for (const ir::ManeuverGroup& group : act.groups) {
                const std::string group_path = join_path(act_path, group.name);
                check_sibling_names(
                    group.maneuvers, [](const ir::Maneuver& m) { return m.name; }, "maneuver",
                    group_path, sink);
                for (const std::string& actor : group.actors) {
                    // The standard defines no checker rule for entity-reference
                    // resolvability (Annex C.7.20 covers only
                    // storyboardElementRef; C.7.21 references_to_scenario_object
                    // is a type constraint, deferred to p2-s1), so no rule id.
                    if (records.find(actor) == records.end()) {
                        error(sink, Status::SemanticError,
                              "actor references unknown entity '" + actor + "'", group_path);
                    }
                }
                for (const ir::Maneuver& maneuver : group.maneuvers) {
                    const std::string maneuver_path = join_path(group_path, maneuver.name);
                    check_sibling_names(
                        maneuver.events, [](const ir::Event& e) { return e.name; }, "event",
                        maneuver_path, sink);
                    for (const ir::Event& event : maneuver.events) {
                        const std::string event_path = join_path(maneuver_path, event.name);
                        validate_trigger(event.start_trigger, event_path, "startTrigger", sink);
                        // §8.3.3.2/§8.4.2.1: the XSD type of
                        // maximumExecutionCount is unsignedInt, so a negative
                        // budget has no meaning at all. Zero is schema-valid
                        // and is accepted — §8.4.2.1 already gives it a
                        // coherent reading (the event is exhausted in
                        // standbyState and completes with a skipTransition
                        // without ever executing). The standard defines no
                        // rule id for this constraint.
                        if (event.maximum_execution_count < 0) {
                            error(sink, Status::ValidationError,
                                  "event maximumExecutionCount is negative", event_path);
                        }
                        if (event.actions.empty()) {
                            error(sink, Status::ValidationError, "event has no actions",
                                  event_path);
                        }
                        std::size_t action_index = 0;
                        for (const std::shared_ptr<ir::Action>& action : event.actions) {
                            const std::string action_path =
                                event_path + "/action[" + std::to_string(action_index) + "]";
                            ++action_index;
                            if (action == nullptr) {
                                error(sink, Status::ValidationError, "event action is null",
                                      action_path);
                                continue;
                            }
                            if (records.find(action->entity_id()) == records.end()) {
                                error(sink, Status::SemanticError,
                                      "action targets unknown entity '" + action->entity_id() + "'",
                                      action_path);
                            }
                        }
                    }
                }
            }
        }
    }
}

/// Code of the first Error diagnostic in report order, which is document
/// order — that becomes init()'s return code.
Status first_error_code(const DiagnosticSink& sink) {
    for (const Diagnostic& diagnostic : sink.diagnostics()) {
        if (diagnostic.severity == Severity::Error) {
            return diagnostic.code;
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

    // A rejected re-init preserves the previous record; a fresh init starts
    // from an empty sink so diagnostics() reflects only this scenario.
    diagnostics_.clear();

    // Validate into a temporary record map so a failed init leaves the engine
    // untouched. Validation walks the whole scenario in document order and
    // accumulates every defect rather than stopping at the first.
    std::map<std::string, EntityRecord> records;
    std::size_t entity_index = 0;
    for (const ir::Entity& entity : scenario.entities) {
        if (entity.id.empty()) {
            error(diagnostics_, Status::ValidationError, "entity id is empty",
                  "entities[" + std::to_string(entity_index) + "]");
            ++entity_index;
            continue;
        }
        ++entity_index;
        auto [it, inserted] = records.try_emplace(entity.id);
        if (!inserted) {
            error(diagnostics_, Status::ValidationError, "duplicate entity id '" + entity.id + "'",
                  "entities/" + entity.id, kRuleUniqueNames);
            continue;
        }
        it->second.mode = entity.control_mode;
    }
    std::size_t init_action_index = 0;
    for (const std::shared_ptr<ir::Action>& action : scenario.init_actions) {
        const std::string action_path = "init/action[" + std::to_string(init_action_index) + "]";
        ++init_action_index;
        if (action == nullptr) {
            error(diagnostics_, Status::ValidationError, "init action is null", action_path);
            continue;
        }
        if (records.find(action->entity_id()) == records.end()) {
            error(diagnostics_, Status::SemanticError,
                  "action targets unknown entity '" + action->entity_id() + "'", action_path);
        }
    }
    validate_storyboard(scenario.storyboard, records, diagnostics_);

    if (diagnostics_.has_errors()) {
        return first_error_code(diagnostics_);
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
