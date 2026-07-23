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

#include "scena/engine.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <variant>

#include "scena/gateway/simulator_gateway.h"
#include "scena/ir/condition.h"
#include "scena/ir/controller.h"
#include "scena/ir/entity_condition.h"
#include "scena/ir/evaluation_context.h"
#include "scena/ir/interaction_condition.h"
#include "scena/ir/rule.h"
#include "scena/ir/traffic_signal.h"
#include "scena/runtime/detmath.h"
#include "scena/runtime/distance_measure.h"
#include "scena/runtime/element_ref.h"
#include "scena/runtime/obb2.h"

namespace scena {

namespace {

// Checker rule UIDs quoted from ASAM OpenSCENARIO XML 1.4.0 Annex C. Only
// constraints the standard actually names carry a rule id; the rest emit an
// empty rule_id.
constexpr const char* kRuleUniqueNames =
    "asam.net:xosc:1.0.0:naming.unique_element_names_on_same_level";
constexpr const char* kRuleConditionDelay =
    "asam.net:xosc:1.0.0:data_type.condition_delay_not_negative";
constexpr const char* kRuleResolvableVariable =
    "asam.net:xosc:1.2.0:reference_control.resolvable_variable_reference";
constexpr const char* kRuleResolvableStoryboardElement =
    "asam.net:xosc:1.0.0:reference_control.resolvable_storyboard_element_ref";
constexpr const char* kRuleDistancesNotNegative =
    "asam.net:xosc:1.1.0:data_type.distances_are_not_negative";
constexpr const char* kRuleTrajectoryTiming =
    "asam.net:xosc:1.0.0:routing.trajectory_timing_exists_if_requested";
constexpr const char* kRuleControllerActivation =
    "asam.net:xosc:1.2.0:scenario_logic.controller_activation";
constexpr const char* kRuleTrajectoryOffset =
    "asam.net:xosc:1.1.0:routing.offset_should_be_less_than_trajectory_length";
constexpr const char* kRuleTimeFormat = "asam.net:xosc:1.0.0:data_type.time_format";
constexpr const char* kRulePhaseDuration = "asam.net:xosc:1.0.0:data_type.phase_duration_positive";
constexpr const char* kRuleControllerReferences =
    "asam.net:xosc:1.0.0:reference_control.traffic_signal_controller_references";
constexpr const char* kRuleControllerActionReferences =
    "asam.net:xosc:1.0.0:reference_control.traffic_signal_controller_action_references";
constexpr const char* kRuleControllerConditionReferences =
    "asam.net:xosc:1.0.0:reference_control.traffic_signal_controller_condition_references";

using NamedValueStore = std::map<std::string, std::string, std::less<>>;

/// The runtime EvaluationContext the engine hands the scheduler each step. It
/// exposes simulation time and the named-value / time-of-day / entity facets
/// from the engine's live stores, and warns once when a
/// UserDefinedValueCondition reads a value the host never set. The stores and
/// diagnostic sink are borrowed and must outlive the context (they are engine
/// members; the context is a per-step temporary).
///
/// Templated over the entity-map type because the record type is private to
/// Engine (the same trick validate_storyboard uses): the facet reads a
/// record's public members through the dependent type without naming it.
template <typename EntityMap> class RuntimeContext final : public ir::EvaluationContext {
public:
    RuntimeContext(double simulation_time, const NamedValueStore& parameters,
                   const NamedValueStore& parameter_overrides, const NamedValueStore& variables,
                   const NamedValueStore& user_defined_values, const EntityMap& entities,
                   const NamedValueStore& signal_states, const NamedValueStore& controller_phases,
                   std::optional<double> date_time_seconds, DiagnosticSink& sink,
                   std::set<std::string>& warned)
        : simulation_time_(simulation_time), parameters_(&parameters),
          parameter_overrides_(&parameter_overrides), variables_(&variables),
          user_defined_values_(&user_defined_values), entities_(&entities),
          signal_states_(&signal_states), controller_phases_(&controller_phases),
          date_time_seconds_(date_time_seconds), sink_(&sink), warned_(&warned) {}

    [[nodiscard]] double simulation_time() const override { return simulation_time_; }

    [[nodiscard]] std::optional<ir::EntityKinematics>
    entity_kinematics(std::string_view id) const override {
        const auto it = entities_->find(id); // heterogeneous lookup (std::less<>)
        // An entity removed by a DeleteEntityAction is absent as far as the
        // by-entity conditions are concerned, so every expression over it is a
        // deterministic false (the ADR-0007 absent-facet grain).
        if (it == entities_->end() || !it->second.active) {
            return std::nullopt;
        }
        const auto& record = it->second;
        return ir::EntityKinematics{record.state, record.acceleration, record.traveled_distance,
                                    record.standstill_seconds, record.bounding_box};
    }

    [[nodiscard]] std::optional<std::string_view>
    traffic_signal_state(std::string_view name) const override {
        const auto it = signal_states_->find(name); // heterogeneous lookup
        if (it == signal_states_->end()) {
            // A signal id is a road-network reference Scena cannot resolve
            // (rule C.7.10 needs a road network), so an id no controller phase
            // and no state action ever wrote is a degraded-but-running
            // condition rather than a load-time error.
            warn_once("trafficSignals/" + std::string(name),
                      "traffic signal '" + std::string(name) +
                          "' has no state yet; condition false");
            return std::nullopt;
        }
        return std::string_view{it->second};
    }

    [[nodiscard]] std::optional<std::string_view>
    traffic_signal_controller_phase(std::string_view controller_ref) const override {
        const auto it = controller_phases_->find(controller_ref);
        if (it == controller_phases_->end()) {
            // Not a defect: a controller waiting out its §6.11.3 delay simply
            // has no phase yet, which init() cannot warn about either.
            return std::nullopt;
        }
        return std::string_view{it->second};
    }

    [[nodiscard]] std::optional<double> date_time_seconds() const override {
        if (!date_time_seconds_.has_value()) {
            warn_once("timeOfDay", "time of day is not set; condition false");
        }
        return date_time_seconds_;
    }

    [[nodiscard]] std::optional<std::string_view>
    named_value(ir::NamedValueKind kind, std::string_view name) const override {
        const NamedValueStore* store = nullptr;
        switch (kind) {
        case ir::NamedValueKind::Parameter: {
            // The deprecated parameter actions write an overlay that shadows the
            // immutable §9.1 declaration; it is consulted first so a 1.0/1.1
            // file's ParameterSetAction is visible to its ParameterCondition.
            const auto overridden = parameter_overrides_->find(name);
            if (overridden != parameter_overrides_->end()) {
                return std::string_view{overridden->second};
            }
            store = parameters_;
            break;
        }
        case ir::NamedValueKind::Variable:
            store = variables_;
            break;
        case ir::NamedValueKind::UserDefinedValue:
            store = user_defined_values_;
            break;
        }
        const auto it = store->find(name);
        if (it != store->end()) {
            return std::string_view{it->second};
        }
        // Parameters and variables cannot dangle at runtime — init rejects an
        // undeclared ref — so only an unset external value reaches here, and
        // that is a degraded-but-running condition, not a failure.
        if (kind == ir::NamedValueKind::UserDefinedValue) {
            warn_once("values/userDefined/" + std::string(name),
                      "user-defined value '" + std::string(name) + "' is not set; condition false");
        }
        return std::nullopt;
    }

private:
    void warn_once(std::string path, std::string message) const {
        if (!warned_->insert(path).second) {
            return; // already reported this name in this run
        }
        Diagnostic diagnostic;
        diagnostic.severity = Severity::Warning;
        diagnostic.code = Status::UnknownName;
        diagnostic.message = std::move(message);
        diagnostic.path = std::move(path);
        sink_->report(std::move(diagnostic));
    }

    double simulation_time_;
    const NamedValueStore* parameters_;
    const NamedValueStore* parameter_overrides_;
    const NamedValueStore* variables_;
    const NamedValueStore* user_defined_values_;
    const EntityMap* entities_;
    const NamedValueStore* signal_states_;
    const NamedValueStore* controller_phases_;
    std::optional<double> date_time_seconds_;
    DiagnosticSink* sink_;
    std::set<std::string>* warned_;
};

/// Index of the controller named `name` in `controllers`, or npos.
std::size_t find_controller(const std::vector<ir::TrafficSignalController>& controllers,
                            std::string_view name) {
    for (std::size_t index = 0; index < controllers.size(); ++index) {
        if (controllers[index].name == name) {
            return index;
        }
    }
    return static_cast<std::size_t>(-1);
}

/// True for the four ordering rules, which the by-value conditions support
/// only between scalar-convertible operands (ParameterCondition clause).
bool is_ordering_rule(ir::Rule rule) {
    switch (rule) {
    case ir::Rule::GreaterThan:
    case ir::Rule::LessThan:
    case ir::Rule::GreaterOrEqual:
    case ir::Rule::LessOrEqual:
        return true;
    case ir::Rule::EqualTo:
    case ir::Rule::NotEqualTo:
        return false;
    }
    return false;
}

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

/// Appends one Warning diagnostic to `sink` (no rule id).
void warn(DiagnosticSink& sink, Status code, std::string message, std::string path) {
    Diagnostic diagnostic;
    diagnostic.severity = Severity::Warning;
    diagnostic.code = code;
    diagnostic.message = std::move(message);
    diagnostic.path = std::move(path);
    sink.report(std::move(diagnostic));
}

std::string join_path(const std::string& prefix, const std::string& child) {
    return prefix.empty() ? child : prefix + "/" + child;
}

/// Warns when an ordering rule is applied to operands that are not both
/// scalar-convertible: the comparison is defined to be false in that case
/// (ParameterCondition / VariableCondition clause), so it is a degraded
/// condition worth surfacing at load time rather than a hard error.
void warn_if_ordering_non_numeric(DiagnosticSink& sink, const std::string& path, ir::Rule rule,
                                  std::string_view stored, std::string_view literal) {
    if (!is_ordering_rule(rule)) {
        return;
    }
    if (ir::parse_scalar(stored).has_value() && ir::parse_scalar(literal).has_value()) {
        return;
    }
    warn(sink, Status::UnsupportedFeature,
         "ordering rule on a non-scalar value; condition is always false", path);
}

/// Warns for the shared distance-parameter modes of the interaction conditions
/// (Distance / RelativeDistance / TimeHeadway / TimeToCollision): a deprecated
/// alongRoute, the deprecated cartesianDistance literal, and a road-based
/// effective coordinate system that is deferred to a real road network. All
/// three leave the condition running (deterministic false for the deferred CS),
/// so they are warnings, not errors. `authored_rdt` is the authored
/// relativeDistanceType (nullopt when defaulted); `along_route` is the authored
/// alongRoute (nullopt when absent — RelativeDistance never has one).
void warn_interaction_modes(DiagnosticSink& sink, const std::string& path,
                            ir::CoordinateSystem effective_cs,
                            std::optional<ir::RelativeDistanceType> authored_rdt,
                            std::optional<bool> along_route) {
    if (along_route.has_value()) {
        // Deprecated attribute; the standard defines no rule id for it.
        warn(sink, Status::DeprecatedFeature,
             "alongRoute is deprecated (§ DistanceCondition); it is ignored when "
             "coordinateSystem or relativeDistanceType is set",
             path);
    }
    if (authored_rdt.has_value() && *authored_rdt == ir::RelativeDistanceType::CartesianDistance) {
        warn(sink, Status::DeprecatedFeature,
             "cartesianDistance is deprecated; treated as euclidianDistance", path);
    }
    if (effective_cs == ir::CoordinateSystem::Lane || effective_cs == ir::CoordinateSystem::Road ||
        effective_cs == ir::CoordinateSystem::Trajectory) {
        // A road prerequisite has no asam.net rule id, so the diagnostic cites
        // section numbers (p5-s1 precedent) rather than inventing one.
        warn(sink, Status::UnsupportedFeature,
             "road/lane/trajectory coordinate system requires road network topology "
             "(§7.6.5.1, §6.4.5); condition is a deterministic false until p3-s4",
             path);
    }
}

/// Counts the storyboard elements of `type` whose name reference matches
/// `segments`, walking the tree in document order. The path carries a leading
/// empty root name so the self-to-root chains match the scheduler's tree (its
/// storyboard root has an empty name), keeping validation and evaluation in
/// exact agreement about what resolves.
int count_element_matches(const ir::Storyboard& storyboard, ir::StoryboardElementType type,
                          const std::vector<std::string>& segments) {
    int matches = 0;
    std::vector<std::string> path{""};
    const auto consider = [&](ir::StoryboardElementType candidate) {
        if (candidate != type) {
            return;
        }
        const std::vector<std::string> chain(path.rbegin(), path.rend());
        if (runtime::element_ref_matches(segments, chain)) {
            ++matches;
        }
    };
    for (const ir::Story& story : storyboard.stories) {
        path.push_back(story.name);
        consider(ir::StoryboardElementType::Story);
        for (const ir::Act& act : story.acts) {
            path.push_back(act.name);
            consider(ir::StoryboardElementType::Act);
            for (const ir::ManeuverGroup& group : act.groups) {
                path.push_back(group.name);
                consider(ir::StoryboardElementType::ManeuverGroup);
                for (const ir::Maneuver& maneuver : group.maneuvers) {
                    path.push_back(maneuver.name);
                    consider(ir::StoryboardElementType::Maneuver);
                    for (const ir::Event& event : maneuver.events) {
                        path.push_back(event.name);
                        consider(ir::StoryboardElementType::Event);
                        path.pop_back();
                    }
                    path.pop_back();
                }
                path.pop_back();
            }
            path.pop_back();
        }
        path.pop_back();
    }
    return matches;
}

/// Validates one trigger condition's logical expression against the scenario's
/// named-value declarations and storyboard hierarchy. dynamic_cast selects the
/// concrete by-value condition; the expression may also be a plain time/entity
/// condition, which carries no named reference and needs no check here. A null
/// expression is reported by the caller.
template <typename Records>
void validate_condition_expression(const ir::TriggerCondition& condition,
                                   const std::string& condition_path,
                                   const ir::Storyboard& storyboard, const Records& records,
                                   const NamedValueStore& parameters,
                                   const NamedValueStore& variables,
                                   const std::vector<ir::TrafficSignalController>& controllers,
                                   DiagnosticSink& sink) {
    const ir::Condition* expression = condition.expression.get();
    if (expression == nullptr) {
        return;
    }
    if (const auto* parameter = dynamic_cast<const ir::ParameterCondition*>(expression)) {
        const auto it = parameters.find(parameter->parameter_ref());
        if (it == parameters.end()) {
            // The standard defines no checker rule for parameter-reference
            // resolvability, so the diagnostic carries no rule id.
            error(sink, Status::SemanticError,
                  "parameter '" + parameter->parameter_ref() + "' is not declared", condition_path);
            return;
        }
        warn_if_ordering_non_numeric(sink, condition_path, parameter->rule(), it->second,
                                     parameter->value());
    } else if (const auto* variable = dynamic_cast<const ir::VariableCondition*>(expression)) {
        const auto it = variables.find(variable->variable_ref());
        if (it == variables.end()) {
            error(sink, Status::SemanticError,
                  "variable '" + variable->variable_ref() + "' is not declared", condition_path,
                  kRuleResolvableVariable);
            return;
        }
        warn_if_ordering_non_numeric(sink, condition_path, variable->rule(), it->second,
                                     variable->value());
    } else if (const auto* user = dynamic_cast<const ir::UserDefinedValueCondition*>(expression)) {
        // The external value is unknown at load time, so only the literal can
        // be checked: an ordering rule on a non-scalar literal can never hold.
        if (is_ordering_rule(user->rule()) && !ir::parse_scalar(user->value()).has_value()) {
            warn(sink, Status::UnsupportedFeature,
                 "ordering rule on a non-scalar value; condition is always false", condition_path);
        }
    } else if (const auto* sim = dynamic_cast<const ir::SimulationTimeCondition*>(expression)) {
        // A NaN reference value can never be reached by any rule; the XSD type
        // is double, so this is a content defect.
        if (std::isnan(sim->value())) {
            error(sink, Status::ValidationError, "simulation time condition value is NaN",
                  condition_path);
        }
    } else if (const auto* tod = dynamic_cast<const ir::TimeOfDayCondition*>(expression)) {
        // An out-of-range date-time (Feb 30, hour 24, ...) has no instant to
        // compare against. The string-format rule (data_type.time_format) is a
        // frontend concern; at the IR level the fields are already parsed.
        if (!tod->date_time().valid()) {
            error(sink, Status::ValidationError, "time of day condition has an invalid date-time",
                  condition_path);
        }
    } else if (const auto* controller_condition =
                   dynamic_cast<const ir::TrafficSignalControllerCondition*>(expression)) {
        // per rule
        // asam.net:xosc:1.0.0:reference_control.traffic_signal_controller_condition_references:
        // the phase "shall reference a valid element ... within the scenario".
        // The trafficSignalControllerRef half of that rule points at the road
        // network, but the controller is declared in the scenario too, so both
        // are checkable here.
        const std::size_t index =
            find_controller(controllers, controller_condition->traffic_signal_controller_ref());
        if (index == static_cast<std::size_t>(-1)) {
            error(sink, Status::SemanticError,
                  "traffic signal controller '" +
                      controller_condition->traffic_signal_controller_ref() + "' is not declared",
                  condition_path, kRuleControllerConditionReferences);
            return;
        }
        bool phase_found = false;
        for (const ir::Phase& phase : controllers[index].phases) {
            if (phase.name == controller_condition->phase()) {
                phase_found = true;
                break;
            }
        }
        if (!phase_found) {
            error(sink, Status::SemanticError,
                  "traffic signal controller '" +
                      controller_condition->traffic_signal_controller_ref() + "' has no phase '" +
                      controller_condition->phase() + "'",
                  condition_path, kRuleControllerConditionReferences);
        }
    } else if (dynamic_cast<const ir::TrafficSignalCondition*>(expression) != nullptr) {
        // The signal id names a road-network element (rule
        // traffic_signal_condition_references, C.7.10) Scena cannot reach until
        // p3-s4, and the state notation is simulator-specific (§6.11.4) — so
        // there is nothing checkable at load time. An id nothing writes warns
        // once at evaluation instead.
    } else if (const auto* element =
                   dynamic_cast<const ir::StoryboardElementStateCondition*>(expression)) {
        const ir::StoryboardElementType type = element->element_type();
        const ir::StoryboardElementState state = element->state();
        if (type == ir::StoryboardElementType::Action) {
            // Per-action nodes arrive with p5-s4; until then an action
            // reference is unobservable and the condition is constant false.
            warn(sink, Status::UnsupportedFeature,
                 "storyboardElementType 'action' is not supported yet; condition is always false",
                 condition_path);
            return;
        }
        // skipTransition is defined only for Event instances
        // (§ StoryboardElementState); on any other type it can never occur.
        if (state == ir::StoryboardElementState::SkipTransition &&
            type != ir::StoryboardElementType::Event) {
            warn(sink, Status::UnsupportedFeature,
                 "skipTransition is only defined for events; condition is always false",
                 condition_path);
        }
        const std::vector<std::string> segments =
            runtime::split_element_ref(element->element_ref());
        const int matches = count_element_matches(storyboard, type, segments);
        if (matches == 0) {
            // Zero matches covers both a dangling reference and a reference to
            // an element of a different type (no node of this kind matches).
            error(sink, Status::SemanticError,
                  "storyboard element '" + element->element_ref() + "' is not found",
                  condition_path, kRuleResolvableStoryboardElement);
        } else if (matches > 1) {
            error(sink, Status::SemanticError,
                  "storyboard element reference '" + element->element_ref() + "' is ambiguous",
                  condition_path, kRuleResolvableStoryboardElement);
        }
    } else if (const auto* by_entity = dynamic_cast<const ir::ByEntityCondition*>(expression)) {
        // Common to every by-entity condition (§7.6.5.1): 1..* triggering
        // entities, each resolvable. The standard defines no checker rule for
        // entity-reference resolvability, so the diagnostics carry no rule id
        // (actor-reference precedent).
        const ir::TriggeringEntities& triggering = by_entity->triggering_entities();
        if (triggering.entity_refs.empty()) {
            error(sink, Status::ValidationError, "triggering entities list is empty",
                  condition_path);
        }
        for (const std::string& ref : triggering.entity_refs) {
            if (records.find(ref) == records.end()) {
                error(sink, Status::SemanticError, "triggering entity '" + ref + "' is unknown",
                      condition_path);
            }
        }
        // Condition-specific numeric ranges and references (the negated >=
        // comparisons reject NaN too; messages carry no floating-point value).
        if (const auto* speed = dynamic_cast<const ir::SpeedCondition*>(expression)) {
            if (std::isnan(speed->value())) {
                error(sink, Status::ValidationError, "speed condition value is NaN",
                      condition_path);
            }
        } else if (const auto* relative =
                       dynamic_cast<const ir::RelativeSpeedCondition*>(expression)) {
            if (records.find(relative->entity_ref()) == records.end()) {
                error(sink, Status::SemanticError,
                      "reference entity '" + relative->entity_ref() + "' is unknown",
                      condition_path);
            }
            if (std::isnan(relative->value())) {
                error(sink, Status::ValidationError, "relative speed condition value is NaN",
                      condition_path);
            }
        } else if (const auto* accel = dynamic_cast<const ir::AccelerationCondition*>(expression)) {
            if (std::isnan(accel->value())) {
                error(sink, Status::ValidationError, "acceleration condition value is NaN",
                      condition_path);
            }
        } else if (const auto* stand = dynamic_cast<const ir::StandStillCondition*>(expression)) {
            if (!(stand->duration() >= 0.0)) {
                error(sink, Status::ValidationError, "standstill condition duration is negative",
                      condition_path);
            }
        } else if (const auto* traveled =
                       dynamic_cast<const ir::TraveledDistanceCondition*>(expression)) {
            if (!(traveled->value() >= 0.0)) {
                error(sink, Status::ValidationError,
                      "traveled distance condition value is negative", condition_path);
            }
        } else if (const auto* reach =
                       dynamic_cast<const ir::ReachPositionCondition*>(expression)) {
            const ir::WorldPosition& position = reach->position();
            if (!(reach->tolerance() >= 0.0) || std::isnan(position.x) || std::isnan(position.y) ||
                std::isnan(position.z)) {
                error(sink, Status::ValidationError,
                      "reach position condition has an invalid position or tolerance",
                      condition_path);
            }
            // Deprecated with version 1.2 (superseded by DistanceCondition); a
            // warning, and the condition still fully evaluates.
            warn(sink, Status::DeprecatedFeature,
                 "reach position condition is deprecated with version 1.2; superseded by the "
                 "distance condition",
                 condition_path);
        } else if (const auto* distance = dynamic_cast<const ir::DistanceCondition*>(expression)) {
            // value in [0..inf[; a NaN or negative target distance is a content
            // defect (rule distances_are_not_negative). The negated >= rejects
            // NaN too.
            if (!(distance->value() >= 0.0)) {
                error(sink, Status::ValidationError, "distance condition value is negative or NaN",
                      condition_path, kRuleDistancesNotNegative);
            }
            const ir::WorldPosition& position = distance->position();
            if (std::isnan(position.x) || std::isnan(position.y) || std::isnan(position.z)) {
                error(sink, Status::ValidationError, "distance condition position is NaN",
                      condition_path);
            }
            warn_interaction_modes(sink, condition_path, distance->effective_coordinate_system(),
                                   distance->relative_distance_type(), distance->along_route());
        } else if (const auto* relative_distance =
                       dynamic_cast<const ir::RelativeDistanceCondition*>(expression)) {
            if (!(relative_distance->value() >= 0.0)) {
                error(sink, Status::ValidationError,
                      "relative distance condition value is negative or NaN", condition_path,
                      kRuleDistancesNotNegative);
            }
            if (records.find(relative_distance->entity_ref()) == records.end()) {
                error(sink, Status::SemanticError,
                      "reference entity '" + relative_distance->entity_ref() + "' is unknown",
                      condition_path);
            }
            // RelativeDistance has no alongRoute; relativeDistanceType is
            // required, so it is always "authored".
            warn_interaction_modes(sink, condition_path,
                                   relative_distance->effective_coordinate_system(),
                                   relative_distance->relative_distance_type(), std::nullopt);
        } else if (const auto* headway =
                       dynamic_cast<const ir::TimeHeadwayCondition*>(expression)) {
            // A time value in [0..inf[; the standard defines no rule id for the
            // headway range, so the diagnostic cites the section only. The
            // negated >= rejects NaN.
            if (!(headway->value() >= 0.0)) {
                error(sink, Status::ValidationError,
                      "time headway condition value is negative or NaN (§ TimeHeadwayCondition)",
                      condition_path);
            }
            if (records.find(headway->entity_ref()) == records.end()) {
                error(sink, Status::SemanticError,
                      "reference entity '" + headway->entity_ref() + "' is unknown",
                      condition_path);
            }
            warn_interaction_modes(sink, condition_path, headway->effective_coordinate_system(),
                                   headway->relative_distance_type(), headway->along_route());
        } else if (const auto* ttc =
                       dynamic_cast<const ir::TimeToCollisionCondition*>(expression)) {
            if (!(ttc->value() >= 0.0)) {
                error(sink, Status::ValidationError,
                      "time to collision condition value is negative or NaN "
                      "(§ TimeToCollisionCondition)",
                      condition_path);
            }
            // The target is an entity XOR a position (holds_alternative rather
            // than std::visit to keep MSVC /W4 quiet).
            if (std::holds_alternative<std::string>(ttc->target())) {
                const std::string& target_ref = std::get<std::string>(ttc->target());
                if (records.find(target_ref) == records.end()) {
                    error(sink, Status::SemanticError,
                          "target entity '" + target_ref + "' is unknown", condition_path);
                }
            } else {
                const ir::WorldPosition& position = std::get<ir::WorldPosition>(ttc->target());
                if (std::isnan(position.x) || std::isnan(position.y) || std::isnan(position.z)) {
                    error(sink, Status::ValidationError,
                          "time to collision condition target position is NaN", condition_path);
                }
            }
            warn_interaction_modes(sink, condition_path, ttc->effective_coordinate_system(),
                                   ttc->relative_distance_type(), ttc->along_route());
        } else if (const auto* collision =
                       dynamic_cast<const ir::CollisionCondition*>(expression)) {
            // Only the EntityRef target is modeled (ByObjectType needs the p2-s1
            // category); the referenced entity must resolve.
            if (records.find(collision->entity_ref()) == records.end()) {
                error(sink, Status::SemanticError,
                      "collision reference entity '" + collision->entity_ref() + "' is unknown",
                      condition_path);
            }
        } else if (const auto* end_of_road =
                       dynamic_cast<const ir::EndOfRoadCondition*>(expression)) {
            if (!(end_of_road->duration() >= 0.0)) {
                error(sink, Status::ValidationError, "end of road condition duration is negative",
                      condition_path);
            }
            warn(sink, Status::UnsupportedFeature,
                 "end of road condition requires road network topology (§7.6.5.1); "
                 "deterministic false until p3-s4",
                 condition_path);
        } else if (const auto* offroad = dynamic_cast<const ir::OffroadCondition*>(expression)) {
            if (!(offroad->duration() >= 0.0)) {
                error(sink, Status::ValidationError, "offroad condition duration is negative",
                      condition_path);
            }
            warn(sink, Status::UnsupportedFeature,
                 "offroad condition requires road network topology (§7.6.5.1); "
                 "deterministic false until p3-s4",
                 condition_path);
        } else if (const auto* clearance =
                       dynamic_cast<const ir::RelativeClearanceCondition*>(expression)) {
            if (!(clearance->distance_backward() >= 0.0) ||
                !(clearance->distance_forward() >= 0.0)) {
                error(sink, Status::ValidationError,
                      "relative clearance condition distance is negative or NaN", condition_path,
                      kRuleDistancesNotNegative);
            }
            for (const ir::RelativeLaneRange& range : clearance->lane_ranges()) {
                // Both limits present and inverted (from > to) is an empty
                // range — a content defect.
                if (range.from.has_value() && range.to.has_value() && *range.from > *range.to) {
                    error(sink, Status::ValidationError,
                          "relative clearance lane range has from > to", condition_path);
                }
            }
            for (const std::string& ref : clearance->entity_refs()) {
                if (records.find(ref) == records.end()) {
                    error(sink, Status::SemanticError,
                          "relative clearance entity '" + ref + "' is unknown", condition_path);
                }
            }
            warn(sink, Status::UnsupportedFeature,
                 "relative clearance condition requires lane coordinates (§6.4.5); "
                 "deterministic false until p3-s4",
                 condition_path);
        }
    }
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
template <typename Records>
void validate_trigger(const std::optional<ir::Trigger>& trigger, const std::string& owner_path,
                      const char* trigger_kind, const ir::Storyboard& storyboard,
                      const Records& records, const NamedValueStore& parameters,
                      const NamedValueStore& variables,
                      const std::vector<ir::TrafficSignalController>& controllers,
                      DiagnosticSink& sink) {
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
            // By-value and by-entity conditions carry named references and
            // numeric fields whose resolvability and ranges are checked here.
            validate_condition_expression(condition, condition_path, storyboard, records,
                                          parameters, variables, controllers, sink);
        }
    }
}

/// Straight-line length between two world positions, in metres. Three
/// dimensional: a trajectory may climb, and the arc length has to match the
/// geometry the follower interpolates. Fixed operand order; sqrt is IEEE-exact.
double segment_length(const ir::WorldPosition& from, const ir::WorldPosition& to) {
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double dz = to.z - from.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/// Linear interpolation between two world positions at fraction `t` in [0, 1].
/// Written as `from + (to - from) * t` so t == 0 and t == 1 reproduce the
/// endpoints exactly.
ir::WorldPosition interpolate_position(const ir::WorldPosition& from, const ir::WorldPosition& to,
                                       double t) {
    return ir::WorldPosition{from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t,
                             from.z + (to.z - from.z) * t};
}

/// Index of the segment containing `value` in the ascending vector `bounds`
/// (`bounds[i] <= value <= bounds[i+1]`), clamped to the first and last
/// segment. A linear scan, not std::lower_bound: the vertex counts are small
/// and a fixed scan keeps the search order obvious.
std::size_t segment_index(const std::vector<double>& bounds, double value) {
    const std::size_t last_segment = bounds.size() - 2;
    std::size_t index = 0;
    while (index < last_segment && bounds[index + 1] < value) {
        ++index;
    }
    return index;
}

/// Index of the half-open interval `[bounds[i], bounds[i+1])` containing
/// `value` in the ascending vector `bounds`, clamped to the last interval.
///
/// Deliberately not segment_index: a trajectory segment owns both its
/// endpoints (the vertex is on the path either way), but a signal phase owns
/// its start and not its end — at exactly the cumulative duration of phase i
/// the cycle has already moved into phase i+1 (§6.11.4). The same half-open
/// rule makes a zero-duration phase occupy no time at all.
std::size_t phase_index(const std::vector<double>& bounds, double value) {
    const std::size_t last = bounds.size() - 2;
    std::size_t index = 0;
    while (index < last && bounds[index + 1] <= value) {
        ++index;
    }
    return index;
}

/// Validates action content beyond target existence. A SpeedAction's transition
/// dynamics value carries a time [s], distance [m] or rate [delta/s] and must be
/// finite and in range [0..inf[ (per ASAM OpenSCENARIO XML 1.4.0
/// §TransitionDynamics); the standard defines no asam.net rule id, so the
/// diagnostic cites the section only. A relative target (§RelativeTargetSpeed)
/// additionally requires a resolvable reference entity, and a continuous target
/// must not be combined with a time- or distance-dimensioned transition (§7.5.3).
///
/// The global actions (§7.4.2) validate their own references here too — they
/// have no actor entity, so the caller's target check does not apply to them.
template <typename Records>
void validate_action_content(const ir::Action& action, const std::string& path,
                             const Records& records, const NamedValueStore& parameters,
                             const NamedValueStore& variables,
                             const std::vector<ir::TrafficSignalController>& controllers,
                             DiagnosticSink& sink) {
    if (const auto* controller_action =
            dynamic_cast<const ir::TrafficSignalControllerAction*>(&action)) {
        // per rule
        // asam.net:xosc:1.0.0:reference_control.traffic_signal_controller_action_references:
        // both the controller and the phase it names must exist within the
        // scenario — unlike signal ids, these are fully checkable here.
        const std::size_t index =
            find_controller(controllers, controller_action->traffic_signal_controller_ref());
        if (index == static_cast<std::size_t>(-1)) {
            error(sink, Status::SemanticError,
                  "traffic signal controller '" +
                      controller_action->traffic_signal_controller_ref() + "' is not declared",
                  path, kRuleControllerActionReferences);
            return;
        }
        bool phase_found = false;
        for (const ir::Phase& phase : controllers[index].phases) {
            if (phase.name == controller_action->phase()) {
                phase_found = true;
                break;
            }
        }
        if (!phase_found) {
            error(sink, Status::SemanticError,
                  "traffic signal controller '" +
                      controller_action->traffic_signal_controller_ref() + "' has no phase '" +
                      controller_action->phase() + "'",
                  path, kRuleControllerActionReferences);
        }
        return;
    }
    if (dynamic_cast<const ir::TrafficSignalStateAction*>(&action) != nullptr) {
        // The signal id names an element of the road network file (rule
        // traffic_signal_state_action_references, C.7.14), which Scena cannot
        // reach yet — the id stays free-form until p3-s4. The state string's
        // notation is simulator-specific by definition (§6.11.4).
        return;
    }
    if (const auto* environment = dynamic_cast<const ir::EnvironmentAction*>(&action)) {
        const ir::Environment& update = environment->environment();
        if (update.time_of_day.has_value() && !update.time_of_day->date_time.valid()) {
            // per rule asam.net:xosc:1.0.0:data_type.time_format — the IR
            // fields are already parsed, so what is checkable here is that
            // they describe a real instant.
            error(sink, Status::ValidationError,
                  "environment action time of day has an invalid date-time", path, kRuleTimeFormat);
        }
        if (update.road_condition.has_value()) {
            const double friction = update.road_condition->friction_scale_factor;
            // §RoadCondition frictionScaleFactor: Range [0..inf[. The negated
            // comparison rejects NaN too.
            if (!(friction >= 0.0) || !std::isfinite(friction)) {
                error(sink, Status::ValidationError,
                      "environment road condition frictionScaleFactor must be finite and in "
                      "range [0..inf[",
                      path);
            }
        }
        if (update.weather.has_value()) {
            const ir::Weather& weather = *update.weather;
            const auto in_range = [](const std::optional<double>& value, double low, double high) {
                return !value.has_value() || (*value >= low && *value <= high);
            };
            const auto non_negative = [](double value) {
                return value >= 0.0 && std::isfinite(value);
            };
            bool valid = true;
            if (weather.sun.has_value()) {
                const ir::Sun& sun = *weather.sun;
                valid = valid && std::isfinite(sun.azimuth) && std::isfinite(sun.elevation) &&
                        non_negative(sun.illuminance);
            }
            if (weather.fog.has_value()) {
                valid = valid && non_negative(weather.fog->visual_range);
            }
            if (weather.precipitation.has_value()) {
                valid = valid && non_negative(weather.precipitation->intensity);
            }
            if (weather.wind.has_value()) {
                valid = valid && std::isfinite(weather.wind->direction) &&
                        non_negative(weather.wind->speed);
            }
            // §Weather ranges: temperature [170..340] K, atmospheric pressure
            // [80000..120000] Pa.
            valid = valid && in_range(weather.temperature, 170.0, 340.0) &&
                    in_range(weather.atmospheric_pressure, 80000.0, 120000.0);
            if (!valid) {
                error(sink, Status::ValidationError,
                      "environment action weather value is outside its documented range (§Weather)",
                      path);
            }
            // §FractionalCloudCover [1.2]: zeroOktas..nineOktas.
            if (weather.fractional_cloud_cover_oktas.has_value() &&
                (*weather.fractional_cloud_cover_oktas < 0 ||
                 *weather.fractional_cloud_cover_oktas > 9)) {
                error(sink, Status::ValidationError,
                      "environment action fractionalCloudCover must be 0..9 oktas", path);
            }
        }
        return;
    }
    if (const auto* add = dynamic_cast<const ir::AddEntityAction*>(&action)) {
        // §EntityAction: "Entities to be added or deleted must be defined in
        // the Entities section." The standard names no checker rule for it.
        if (records.find(add->entity_ref()) == records.end()) {
            error(sink, Status::SemanticError,
                  "add entity action references undeclared entity '" + add->entity_ref() + "'",
                  path);
        }
        const ir::WorldPosition& position = add->position();
        if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
            !std::isfinite(position.z)) {
            error(sink, Status::ValidationError, "add entity action position must be finite", path);
        }
        return;
    }
    if (const auto* remove = dynamic_cast<const ir::DeleteEntityAction*>(&action)) {
        if (records.find(remove->entity_ref()) == records.end()) {
            error(sink, Status::SemanticError,
                  "delete entity action references undeclared entity '" + remove->entity_ref() +
                      "'",
                  path);
        }
        return;
    }
    if (const auto* variable_set = dynamic_cast<const ir::VariableSetAction*>(&action)) {
        // per rule asam.net:xosc:1.2.0:reference_control.resolvable_variable_reference
        if (variables.find(variable_set->variable_ref()) == variables.end()) {
            error(sink, Status::SemanticError,
                  "variable '" + variable_set->variable_ref() + "' is not declared", path,
                  kRuleResolvableVariable);
        }
        return;
    }
    if (const auto* variable_modify = dynamic_cast<const ir::VariableModifyAction*>(&action)) {
        if (variables.find(variable_modify->variable_ref()) == variables.end()) {
            error(sink, Status::SemanticError,
                  "variable '" + variable_modify->variable_ref() + "' is not declared", path,
                  kRuleResolvableVariable);
        }
        // The operand is the right-hand side of a single IEEE expression; a
        // non-finite one would poison the store irrecoverably.
        if (!std::isfinite(variable_modify->value())) {
            error(sink, Status::ValidationError, "variable modify action value must be finite",
                  path);
        }
        return;
    }
    if (const auto* parameter_set = dynamic_cast<const ir::ParameterSetAction*>(&action)) {
        // The standard defines no rule id for parameter-reference resolvability
        // (the ParameterCondition precedent), so the diagnostic cites §9.1.
        if (parameters.find(parameter_set->parameter_ref()) == parameters.end()) {
            error(sink, Status::SemanticError,
                  "parameter '" + parameter_set->parameter_ref() + "' is not declared (§9.1)",
                  path);
        }
        return;
    }
    if (const auto* parameter_modify = dynamic_cast<const ir::ParameterModifyAction*>(&action)) {
        if (parameters.find(parameter_modify->parameter_ref()) == parameters.end()) {
            error(sink, Status::SemanticError,
                  "parameter '" + parameter_modify->parameter_ref() + "' is not declared (§9.1)",
                  path);
        }
        if (!std::isfinite(parameter_modify->value())) {
            error(sink, Status::ValidationError, "parameter modify action value must be finite",
                  path);
        }
        return;
    }
    if (const auto* speed = dynamic_cast<const ir::SpeedAction*>(&action)) {
        const ir::TransitionDynamics& td = speed->dynamics();
        if (!std::isfinite(td.value) || td.value < 0.0) {
            error(sink, Status::ValidationError,
                  "speed action transition dynamics value must be finite and in range [0..inf[",
                  path);
        }
        if (speed->is_relative()) {
            const ir::RelativeTargetSpeed& rel = *speed->relative_target();
            if (records.find(rel.entity_ref) == records.end()) {
                error(sink, Status::SemanticError,
                      "speed action reference entity '" + rel.entity_ref + "' is unknown", path);
            }
            if (!std::isfinite(rel.value)) {
                error(sink, Status::ValidationError, "relative speed target value must be finite",
                      path);
            }
            // §RelativeTargetSpeed: "This may not be used together with
            // Dynamics.time or Dynamics.distance." (§7.5.3). A positive time or
            // distance defines exactly such a transition.
            if (rel.continuous && td.value > 0.0 &&
                (td.dimension == ir::DynamicsDimension::Time ||
                 td.dimension == ir::DynamicsDimension::Distance)) {
                error(sink, Status::ValidationError,
                      "continuous relative speed target must not use a time- or "
                      "distance-dimensioned transition",
                      path);
            }
        }
        return;
    }
    if (const auto* profile = dynamic_cast<const ir::SpeedProfileAction*>(&action)) {
        // §SpeedProfileAction requires at least one entry; each entry's speed is
        // finite and its optional time is finite and in range [0..inf[.
        if (profile->entries().empty()) {
            error(sink, Status::ValidationError, "speed profile action has no entries", path);
        }
        std::size_t entry_index = 0;
        for (const ir::SpeedProfileEntry& entry : profile->entries()) {
            const std::string entry_path = path + "/entry[" + std::to_string(entry_index) + "]";
            ++entry_index;
            if (!std::isfinite(entry.speed)) {
                error(sink, Status::ValidationError, "speed profile entry speed must be finite",
                      entry_path);
            }
            if (entry.time.has_value() && (!std::isfinite(*entry.time) || *entry.time < 0.0)) {
                error(sink, Status::ValidationError,
                      "speed profile entry time must be finite and in range [0..inf[", entry_path);
            }
        }
        return;
    }
    if (const auto* teleport = dynamic_cast<const ir::TeleportAction*>(&action)) {
        const ir::WorldPosition& position = teleport->position();
        if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
            !std::isfinite(position.z)) {
            error(sink, Status::ValidationError, "teleport action position must be finite", path);
        }
        return;
    }
    if (const auto* keeping = dynamic_cast<const ir::LongitudinalDistanceAction*>(&action)) {
        // §LongitudinalDistanceAction: the reference entity is a prerequisite
        // (§7.5.2.2) and must resolve.
        if (records.find(keeping->entity_ref()) == records.end()) {
            error(sink, Status::SemanticError,
                  "longitudinal distance action reference entity '" + keeping->entity_ref() +
                      "' is unknown",
                  path);
        }
        // "distance: not to be used together with timeGap attribute" — and one
        // of the two must be there or the action has no target at all.
        const bool has_distance = keeping->distance().has_value();
        const bool has_time_gap = keeping->time_gap().has_value();
        if (has_distance == has_time_gap) {
            error(sink, Status::ValidationError,
                  "longitudinal distance action needs exactly one of distance or timeGap", path);
        }
        // Both are Range [0..inf[; the negated comparison also rejects NaN.
        if (has_distance && !(*keeping->distance() >= 0.0 && std::isfinite(*keeping->distance()))) {
            error(sink, Status::ValidationError,
                  "longitudinal distance action distance must be finite and in range [0..inf[",
                  path, kRuleDistancesNotNegative);
        }
        if (has_time_gap && !(*keeping->time_gap() >= 0.0 && std::isfinite(*keeping->time_gap()))) {
            error(sink, Status::ValidationError,
                  "longitudinal distance action timeGap must be finite and in range [0..inf[",
                  path);
        }
        if (keeping->constraints().has_value()) {
            const ir::DynamicConstraints& constraints = *keeping->constraints();
            const auto invalid = [](const std::optional<double>& value) {
                return value.has_value() && !(*value >= 0.0 && std::isfinite(*value));
            };
            if (invalid(constraints.max_acceleration) || invalid(constraints.max_deceleration) ||
                invalid(constraints.max_acceleration_rate) ||
                invalid(constraints.max_deceleration_rate) || invalid(constraints.max_speed)) {
                error(sink, Status::ValidationError,
                      "longitudinal distance action dynamic constraints must be finite and in "
                      "range [0..inf[",
                      path);
            }
        }
        // A road-based coordinate system needs a road network (p3-s4). The
        // action then completes immediately at runtime rather than keeping a
        // distance it cannot measure, so this is a warning, not an error —
        // the ADR-0009 precedent for the interaction conditions.
        const ir::CoordinateSystem cs = keeping->coordinate_system();
        if (cs == ir::CoordinateSystem::Lane || cs == ir::CoordinateSystem::Road ||
            cs == ir::CoordinateSystem::Trajectory) {
            warn(sink, Status::UnsupportedFeature,
                 "road-based coordinate system needs a road network (§6.4); the longitudinal "
                 "distance action completes immediately",
                 path);
        }
        return;
    }
    if (const auto* assign_route = dynamic_cast<const ir::AssignRouteAction*>(&action)) {
        // §Route: "At least two waypoints are needed to define a route."
        const ir::Route& route = assign_route->route();
        if (route.waypoints.size() < 2) {
            error(sink, Status::ValidationError, "route needs at least two waypoints", path);
        }
        std::size_t waypoint_index = 0;
        for (const ir::Waypoint& waypoint : route.waypoints) {
            if (!std::isfinite(waypoint.position.x) || !std::isfinite(waypoint.position.y) ||
                !std::isfinite(waypoint.position.z)) {
                error(sink, Status::ValidationError, "route waypoint position must be finite",
                      path + "/waypoint[" + std::to_string(waypoint_index) + "]");
            }
            ++waypoint_index;
        }
        return;
    }
    if (const auto* acquire = dynamic_cast<const ir::AcquirePositionAction*>(&action)) {
        const ir::WorldPosition& position = acquire->position();
        if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
            !std::isfinite(position.z)) {
            error(sink, Status::ValidationError, "acquire position action position must be finite",
                  path);
        }
        return;
    }
    if (const auto* follow = dynamic_cast<const ir::FollowTrajectoryAction*>(&action)) {
        const ir::Trajectory& trajectory = follow->trajectory();
        // §Polyline: an ordered chain of at least two vertices.
        if (trajectory.vertices.size() < 2) {
            error(sink, Status::ValidationError, "trajectory needs at least two vertices", path);
            return;
        }
        double arc_length = 0.0;
        bool finite_geometry = true;
        for (std::size_t index = 0; index < trajectory.vertices.size(); ++index) {
            const ir::WorldPosition& position = trajectory.vertices[index].position;
            if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
                !std::isfinite(position.z)) {
                error(sink, Status::ValidationError, "trajectory vertex position must be finite",
                      path + "/vertex[" + std::to_string(index) + "]");
                finite_geometry = false;
                continue;
            }
            if (index > 0) {
                arc_length += segment_length(trajectory.vertices[index - 1].position, position);
            }
        }
        if (follow->time_reference().has_value()) {
            const ir::Timing& timing = *follow->time_reference();
            // §Timing: scale is Range ]0..inf[, offset is finite.
            if (!(timing.scale > 0.0) || !std::isfinite(timing.scale) ||
                !std::isfinite(timing.offset)) {
                error(sink, Status::ValidationError,
                      "trajectory timing needs a positive finite scale and a finite offset", path);
            }
            // per rule asam.net:xosc:1.0.0:routing.trajectory_timing_exists_if_requested
            std::optional<double> previous;
            for (std::size_t index = 0; index < trajectory.vertices.size(); ++index) {
                const std::optional<double>& time = trajectory.vertices[index].time;
                if (!time.has_value() || !std::isfinite(*time)) {
                    error(sink, Status::ValidationError,
                          "trajectory timing requires a time on every vertex",
                          path + "/vertex[" + std::to_string(index) + "]", kRuleTrajectoryTiming);
                    break;
                }
                if (previous.has_value() && !(*time > *previous)) {
                    error(sink, Status::ValidationError,
                          "trajectory vertex times must be strictly increasing",
                          path + "/vertex[" + std::to_string(index) + "]");
                    break;
                }
                previous = time;
            }
        }
        // per rule asam.net:xosc:1.1.0:routing.offset_should_be_less_than_trajectory_length
        const double offset = follow->initial_distance_offset();
        if (!std::isfinite(offset) || offset < 0.0 || (finite_geometry && offset > arc_length)) {
            error(sink, Status::ValidationError,
                  "trajectory initialDistanceOffset must be in range [0..arclength]", path,
                  kRuleTrajectoryOffset);
        }
        if (trajectory.closed) {
            // §Trajectory: a closed trajectory loops and the action then has no
            // regular ending. The loop needs the other shapes' machinery, so it
            // is deferred to p2-s5 and the open path is followed instead.
            warn(sink, Status::UnsupportedFeature,
                 "closed trajectories are not implemented yet (§6.9); the open path is followed",
                 path);
        }
        if (follow->following_mode() == ir::FollowingMode::Follow) {
            // ADR-0011 precedent: follow is accepted and executed as position
            // until a steering controller exists (p2-s5).
            warn(sink, Status::UnsupportedFeature,
                 "trajectoryFollowingMode 'follow' is executed as 'position' (§6.9)", path);
        }
        return;
    }
    if (const auto* assign_controller = dynamic_cast<const ir::AssignControllerAction*>(&action)) {
        // per rule asam.net:xosc:1.2.0:scenario_logic.controller_activation: a
        // controller "shall not be activated in a domain where it is not
        // defined through the controllerType".
        const ir::Controller& controller = assign_controller->controller();
        const auto activates = [](const std::optional<bool>& flag) {
            return flag.has_value() && *flag;
        };
        if (activates(assign_controller->activate_lateral()) &&
            !ir::controls_lateral(controller.type)) {
            error(sink, Status::ValidationError,
                  "controller '" + controller.name +
                      "' is not defined for the lateral domain it activates",
                  path, kRuleControllerActivation);
        }
        if (activates(assign_controller->activate_longitudinal()) &&
            !ir::controls_longitudinal(controller.type)) {
            error(sink, Status::ValidationError,
                  "controller '" + controller.name +
                      "' is not defined for the longitudinal domain it activates",
                  path, kRuleControllerActivation);
        }
    }
}

/// Validates the storyboard tree in document order: element naming,
/// triggers, non-null actions, and action targets that exist in `records`.
/// Every defect is appended to `sink`; the walk never stops early.
template <typename Records>
void validate_storyboard(const ir::Storyboard& storyboard, const Records& records,
                         const NamedValueStore& parameters, const NamedValueStore& variables,
                         const std::vector<ir::TrafficSignalController>& controllers,
                         DiagnosticSink& sink) {
    check_sibling_names(
        storyboard.stories, [](const ir::Story& s) { return s.name; }, "story", "", sink);
    validate_trigger(storyboard.stop_trigger, "", "stopTrigger", storyboard, records, parameters,
                     variables, controllers, sink);
    for (const ir::Story& story : storyboard.stories) {
        const std::string story_path = story.name;
        check_sibling_names(
            story.acts, [](const ir::Act& a) { return a.name; }, "act", story_path, sink);
        for (const ir::Act& act : story.acts) {
            const std::string act_path = join_path(story_path, act.name);
            check_sibling_names(
                act.groups, [](const ir::ManeuverGroup& g) { return g.name; }, "maneuver group",
                act_path, sink);
            validate_trigger(act.start_trigger, act_path, "startTrigger", storyboard, records,
                             parameters, variables, controllers, sink);
            validate_trigger(act.stop_trigger, act_path, "stopTrigger", storyboard, records,
                             parameters, variables, controllers, sink);
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
                        validate_trigger(event.start_trigger, event_path, "startTrigger",
                                         storyboard, records, parameters, variables, controllers,
                                         sink);
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
                            // A global action (§7.4.2/§7.4.3) has no actor, so
                            // the target check does not apply to it; it
                            // validates its own references in
                            // validate_action_content.
                            if (dynamic_cast<const ir::GlobalAction*>(action.get()) == nullptr &&
                                records.find(action->entity_id()) == records.end()) {
                                error(sink, Status::SemanticError,
                                      "action targets unknown entity '" + action->entity_id() + "'",
                                      action_path);
                            }
                            validate_action_content(*action, action_path, records, parameters,
                                                    variables, controllers, sink);
                        }
                    }
                }
            }
        }
    }
}

/// Half-open completion band of a non-continuous LongitudinalDistanceAction,
/// in metres. The deadbeat command lands the gap exactly on its target in the
/// point-mass model, so this only has to absorb the rounding of the position
/// integration (~1e-14 m at scenario scale), never a modeling error
/// (ADR-0014).
constexpr double kDistanceKeepingEpsilon = 1e-9;

/// The signed longitudinal separation from `actor` to `reference` along the
/// effective coordinate system's longitudinal axis, in metres: positive when
/// the reference is ahead of the actor, negative when behind.
///
/// With `freespace` false this is the projected reference-point separation
/// (§6.4.7.1). With `freespace` true it is the bumper-to-bumper gap on that
/// axis (§6.4.7.2) carrying the sign of the reference-point separation, so
/// overlapping boxes report a negative gap instead of the unsigned zero the
/// conditions want. Returns std::nullopt when a freespace measurement is asked
/// for and either entity has no geometry — a missing prerequisite (§7.5.2.2).
///
/// Only the Entity and World coordinate systems reach here; the road-based
/// systems are handled before the call (p3-s4).
std::optional<double> signed_longitudinal_gap(const EntityState& actor,
                                              const std::optional<ir::BoundingBox>& actor_box,
                                              const EntityState& reference,
                                              const std::optional<ir::BoundingBox>& reference_box,
                                              ir::CoordinateSystem cs, bool freespace) {
    double ux = 0.0;
    double uy = 0.0;
    runtime::projection_axis(cs, ir::RelativeDistanceType::Longitudinal, actor.heading, ux, uy);
    const double rx = reference.x - actor.x;
    const double ry = reference.y - actor.y;
    const double reference_point_gap = rx * ux + ry * uy;
    if (!freespace) {
        return reference_point_gap;
    }
    if (!actor_box.has_value() || !reference_box.has_value()) {
        return std::nullopt;
    }
    const runtime::Obb2 a = runtime::make_obb(actor, *actor_box);
    const runtime::Obb2 b = runtime::make_obb(reference, *reference_box);
    double a_lo = 0.0;
    double a_hi = 0.0;
    double b_lo = 0.0;
    double b_hi = 0.0;
    runtime::obb_project(a, ux, uy, a_lo, a_hi);
    runtime::obb_project(b, ux, uy, b_lo, b_hi);
    // Reference ahead: the gap runs from the actor's front to the reference's
    // rear. Reference behind: from the reference's front to the actor's rear,
    // negated so the sign convention holds.
    return reference_point_gap >= 0.0 ? b_lo - a_hi : b_hi - a_lo;
}

/// The tighter of two optional limits, where an absent or non-positive limit
/// does not constrain (§DynamicConstraints "missing value is interpreted as
/// 'inf'"; the p2-s2 Performance clamp likewise only honours a positive
/// limit). Returns infinity when neither constrains.
double effective_limit(const std::optional<double>& constraint,
                       const std::optional<double>& envelope) {
    double limit = std::numeric_limits<double>::infinity();
    if (constraint.has_value() && *constraint > 0.0) {
        limit = *constraint;
    }
    if (envelope.has_value() && *envelope > 0.0) {
        limit = std::fmin(limit, *envelope);
    }
    return limit;
}

/// Validates the traffic signal controllers of a scenario (§6.11) in document
/// order, appending every defect to `sink`. The engine's cycle clock relies on
/// what is established here: unique names, non-negative finite durations, a
/// resolvable and acyclic reference chain, and delay only where a reference
/// backs it.
void validate_traffic_signals(const std::vector<ir::TrafficSignalController>& controllers,
                              DiagnosticSink& sink) {
    std::set<std::string> seen_names;
    std::size_t controller_index = 0;
    for (const ir::TrafficSignalController& controller : controllers) {
        const std::string path = controller.name.empty()
                                     ? "trafficSignals[" + std::to_string(controller_index) + "]"
                                     : "trafficSignals/" + controller.name;
        ++controller_index;
        if (controller.name.empty()) {
            error(sink, Status::ValidationError, "traffic signal controller name is empty", path);
        } else if (!seen_names.insert(controller.name).second) {
            error(sink, Status::ValidationError,
                  "duplicate traffic signal controller name '" + controller.name + "'", path,
                  kRuleUniqueNames);
        }

        // §6.11.3 / §TrafficSignalController: "If delay is set, reference is
        // required", and the reference must name an existing controller.
        if (controller.delay.has_value() && !controller.reference.has_value()) {
            error(sink, Status::ValidationError,
                  "traffic signal controller delay requires a reference (§6.11.3)", path);
        }
        if (controller.delay.has_value() &&
            (!(*controller.delay >= 0.0) || !std::isfinite(*controller.delay))) {
            error(sink, Status::ValidationError,
                  "traffic signal controller delay must be finite and in range [0..inf[", path);
        }
        if (controller.reference.has_value()) {
            if (*controller.reference == controller.name) {
                error(sink, Status::ValidationError,
                      "traffic signal controller '" + controller.name + "' references itself", path,
                      kRuleControllerReferences);
            } else if (find_controller(controllers, *controller.reference) ==
                       static_cast<std::size_t>(-1)) {
                error(sink, Status::SemanticError,
                      "traffic signal controller reference '" + *controller.reference +
                          "' is not declared",
                      path, kRuleControllerReferences);
            } else {
                // Walk the chain; more hops than there are controllers means
                // it loops. A cycle would make the start offset undefined.
                std::size_t hops = 0;
                std::optional<std::string> next = controller.reference;
                while (next.has_value() && hops <= controllers.size()) {
                    const std::size_t index = find_controller(controllers, *next);
                    if (index == static_cast<std::size_t>(-1)) {
                        break; // dangling further up; already reported on that controller
                    }
                    next = controllers[index].reference;
                    ++hops;
                }
                if (hops > controllers.size()) {
                    error(sink, Status::ValidationError,
                          "traffic signal controller reference chain from '" + controller.name +
                              "' is cyclic",
                          path, kRuleControllerReferences);
                }
            }
        }

        std::set<std::string> seen_phases;
        double total = 0.0;
        std::size_t phase_index = 0;
        for (const ir::Phase& phase : controller.phases) {
            const std::string phase_path = path + "/phase[" + std::to_string(phase_index) + "]";
            ++phase_index;
            // §6.11.4: a phase has "a name that is unique within its
            // controller" — the name is what an action and a condition
            // address it by, so it is part of the runtime contract.
            if (phase.name.empty()) {
                error(sink, Status::ValidationError, "traffic signal phase name is empty",
                      phase_path);
            } else if (!seen_phases.insert(phase.name).second) {
                error(sink, Status::ValidationError,
                      "duplicate traffic signal phase name '" + phase.name + "'", phase_path,
                      kRuleUniqueNames);
            }
            // per rule asam.net:xosc:1.0.0:data_type.phase_duration_positive
            // ("shall contain non-negative values"); the negated comparison
            // rejects NaN too.
            if (!(phase.duration >= 0.0) || !std::isfinite(phase.duration)) {
                error(sink, Status::ValidationError,
                      "traffic signal phase duration must be finite and non-negative", phase_path,
                      kRulePhaseDuration);
            } else {
                total += phase.duration;
            }
            for (const ir::TrafficSignalState& state : phase.signal_states) {
                // The state string is free-form (§6.11.4 leaves its notation to
                // the simulator), but a signal with no id addresses nothing.
                if (state.traffic_signal_id.empty()) {
                    error(sink, Status::ValidationError, "traffic signal state has an empty id",
                          phase_path);
                }
            }
        }
        if (!controller.phases.empty() && total == 0.0) {
            // §6.11.4 makes the cycle duration the sum of the phase durations;
            // a zero-length cycle has no next phase to move to. The engine
            // pins the first phase rather than dividing by it.
            warn(sink, Status::UnsupportedFeature,
                 "traffic signal controller cycle has zero total duration; the first phase is "
                 "held for the whole run (§6.11.4)",
                 path);
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
    std::map<std::string, EntityRecord, std::less<>> records;
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
        // Geometry is copied once and is immutable at runtime. All three
        // concrete entity objects carry a bounding box (§Vehicle/§Pedestrian/
        // §MiscObject); an unclassified participant has none. A zero-size box
        // is a valid degenerate point, so only NaN/negative dimensions or a
        // NaN center are content defects (per ASAM OpenSCENARIO XML 1.4.0
        // BoundingBox; dimensions and center are the only fields freespace
        // math consumes this phase).
        if (const std::optional<ir::BoundingBox> box_opt = ir::bounding_box_of(entity)) {
            const ir::BoundingBox& box = *box_opt;
            const bool bad_dimensions =
                !(box.length >= 0.0) || !(box.width >= 0.0) || !(box.height >= 0.0);
            const bool bad_center =
                std::isnan(box.center_x) || std::isnan(box.center_y) || std::isnan(box.center_z);
            if (bad_dimensions || bad_center) {
                error(diagnostics_, Status::ValidationError,
                      "entity '" + entity.id + "' has an invalid bounding box",
                      "entities/" + entity.id);
            }
            it->second.bounding_box = box_opt;
        }
        // Performance limits, present only on a Vehicle. The standard ranges
        // are [0..inf[ for maxSpeed / maxAcceleration / maxDeceleration and the
        // optional rate limits, so a negative, NaN, or non-finite value is a
        // content defect (per ASAM OpenSCENARIO XML 1.4.0 §Performance). A zero
        // maxDeceleration is spec-permitted though degenerate; the default
        // controller (p2-s2) handles that, so it is not rejected here.
        if (const ir::Performance* perf = ir::performance_of(entity)) {
            const auto invalid = [](double value) { return !std::isfinite(value) || value < 0.0; };
            const auto invalid_opt = [&](const std::optional<double>& value) {
                return value.has_value() && invalid(*value);
            };
            if (invalid(perf->max_speed) || invalid(perf->max_acceleration) ||
                invalid(perf->max_deceleration) || invalid_opt(perf->max_acceleration_rate) ||
                invalid_opt(perf->max_deceleration_rate)) {
                error(diagnostics_, Status::ValidationError,
                      "entity '" + entity.id + "' has invalid performance limits",
                      "entities/" + entity.id);
            }
            // Copied once for the default longitudinal controller (p2-s2).
            it->second.performance = *perf;
        }
    }
    std::size_t init_action_index = 0;
    for (const std::shared_ptr<ir::Action>& action : scenario.init_actions) {
        const std::string action_path = "init/action[" + std::to_string(init_action_index) + "]";
        ++init_action_index;
        if (action == nullptr) {
            error(diagnostics_, Status::ValidationError, "init action is null", action_path);
            continue;
        }
        // Global actions are legal in the init phase (§8.5) and carry no actor.
        if (dynamic_cast<const ir::GlobalAction*>(action.get()) == nullptr &&
            records.find(action->entity_id()) == records.end()) {
            error(diagnostics_, Status::SemanticError,
                  "action targets unknown entity '" + action->entity_id() + "'", action_path);
        }
        validate_action_content(*action, action_path, records, scenario.parameters,
                                scenario.variables, scenario.traffic_signal_controllers,
                                diagnostics_);
    }
    validate_traffic_signals(scenario.traffic_signal_controllers, diagnostics_);
    validate_storyboard(scenario.storyboard, records, scenario.parameters, scenario.variables,
                        scenario.traffic_signal_controllers, diagnostics_);

    if (diagnostics_.has_errors()) {
        return first_error_code(diagnostics_);
    }

    scenario_ = std::move(scenario);
    entities_ = std::move(records);
    // Seed the runtime variable store from the declarations (§6.12: variables
    // take their initialization value at load time). Parameters are read
    // straight from scenario_.parameters — immutable at runtime, so no copy.
    // User-defined values are deliberately not cleared here: a host may stage
    // them before init so they are visible at the t = 0 evaluation, and they
    // persist across init() until close().
    variables_ = scenario_.variables;
    // The deprecated parameter overlay and the environment store are per-run
    // state, not host state: a fresh init starts from the declared §9.1 values
    // and an empty environment. The time-of-day anchor is host state and
    // deliberately survives (a host may stage it before init), but its
    // animation flag goes back to the host setter's meaning.
    parameter_overrides_.clear();
    environment_ = ir::Environment{};
    date_time_animated_ = true;
    signal_states_.clear();
    controller_phases_.clear();
    signal_controllers_.clear();
    warned_values_.clear();
    clock_.reset();
    initialized_ = true;

    // Build the signal cycle clocks (§6.11.4). The cumulative offsets are
    // summed once, in declared order, so every later phase lookup reads fixed
    // partial sums instead of re-adding them.
    for (const ir::TrafficSignalController& controller : scenario_.traffic_signal_controllers) {
        SignalControllerRuntime runtime;
        runtime.controller = &controller;
        runtime.cumulative.reserve(controller.phases.size() + 1);
        double total = 0.0;
        runtime.cumulative.push_back(0.0);
        for (const ir::Phase& phase : controller.phases) {
            total += phase.duration;
            runtime.cumulative.push_back(total);
        }
        runtime.total = total;
        signal_controllers_.emplace(controller.name, std::move(runtime));
    }
    // §6.11.3: "its first phase virtually starts delaytime seconds after the
    // start of the reference's first phase", so a chained controller's offset
    // is the sum of the delays up its chain. Validation already rejected
    // cycles and dangling references, so the walk terminates.
    for (const ir::TrafficSignalController& controller : scenario_.traffic_signal_controllers) {
        double offset = 0.0;
        const ir::TrafficSignalController* current = &controller;
        while (current->reference.has_value() && current->delay.has_value()) {
            offset += *current->delay;
            const std::size_t index =
                find_controller(scenario_.traffic_signal_controllers, *current->reference);
            if (index == static_cast<std::size_t>(-1)) {
                break;
            }
            current = &scenario_.traffic_signal_controllers[index];
        }
        const auto it = signal_controllers_.find(controller.name);
        if (it != signal_controllers_.end()) {
            it->second.start_offset = offset;
            it->second.anchor = offset;
        }
    }
    // Seed the first-phase states before the init actions run, so an init-phase
    // TrafficSignalStateAction overrides the seed rather than being overwritten
    // by it (actions win, ADR-0015).
    advance_signal_controllers(0.0);

    // Init phase (§8.5): init actions are applied before simulation time
    // starts. All actions are instantaneous in this phase, so each one
    // completes during init; their document order does not imply an
    // execution order, but applying them in that order is deterministic.
    for (const std::shared_ptr<ir::Action>& action : scenario_.init_actions) {
        if (apply(*action) == runtime::ActionOutcome::Running) {
            // §8.5: init actions take effect instantaneously. A transition
            // installed here would otherwise never be re-polled, so snap it to
            // its terminal value and release the entity.
            const auto it = entities_.find(action->entity_id());
            if (it != entities_.end()) {
                finalize_longitudinal(*action, it->second);
            }
        }
    }

    // Seed the derived-observation baseline after init actions, before the
    // t = 0 evaluation: the post-init state is the reference the first dt > 0
    // step differences against. Acceleration stays absent (no phantom at t=0),
    // the odometer starts at 0, and a stationary entity's standstill timer is
    // 0 — so a StandStill with duration 0 holds at t = 0 while Acceleration
    // does not.
    for (auto& [id, record] : entities_) {
        (void)id;
        record.prev_sample = record.state;
        record.has_prev_sample = true;
    }

    // The storyboard enters runningState and simulation time starts with it
    // (§8.4.7): evaluate once at t = 0 so trigger-less chains and start
    // conditions that already hold fire before the first host step.
    scheduler_.bind(scenario_.storyboard);
    RuntimeContext context{0.0,
                           scenario_.parameters,
                           parameter_overrides_,
                           variables_,
                           user_defined_values_,
                           entities_,
                           signal_states_,
                           controller_phases_,
                           current_date_time_seconds(0.0),
                           diagnostics_,
                           warned_values_};
    scheduler_.step(context, [this](const ir::Action& action) { return apply(action); });

    // The init phase has no integrate stage, so a follower that placed an
    // entity at t = 0 must not make the first step skip its integration.
    for (auto& [id, record] : entities_) {
        (void)id;
        record.trajectory_moved = false;
    }

    return Status::Ok;
}

Status Engine::step(double dt) {
    if (!initialized_) {
        return Status::NotInitialized;
    }
    if (!(dt >= 0.0)) { // rejects negative values and NaN
        return Status::InvalidArgument;
    }

    // The scheduler's fire callback re-polls longitudinal actions without a dt
    // argument; expose the current step to it.
    last_dt_ = dt;

    clock_.advance(dt);

    if (gateway_ != nullptr) {
        for (auto& [id, record] : entities_) {
            if (!record.active) {
                continue; // not in the scenario (§EntityAction)
            }
            if (record.mode == ir::ControlMode::HostControlled) {
                EntityState polled;
                if (gateway_->poll_state(id, polled)) {
                    record.state = polled;
                }
            }
        }
    }

    // Phase 2b: refresh the derived observations the by-entity conditions read
    // (§7.6.5.1). These are finite differences of exactly the snapshots the
    // storyboard is about to observe, so they iterate entities_ in std::map
    // order and treat both control modes uniformly. A dt == 0 step is skipped
    // entirely: no accumulator update and prev_sample is left untouched, so
    // there is no 0/0 (which would poison NotEqualTo into true) and no motion
    // is lost — displacement accrues at the next dt > 0 step. The one-step
    // observation lag for engine-controlled entities (integration is phase 4)
    // is intentional and unchanged.
    if (dt > 0.0) {
        refresh_observations(dt);
    }

    // Top of phase 3: the signal cycles advance to t' before the storyboard is
    // evaluated, so conditions read this step's phase and a
    // TrafficSignalStateAction fired below still wins over it (§6.11.4).
    advance_signal_controllers(clock_.now());

    RuntimeContext context{
        clock_.now(),   scenario_.parameters, parameter_overrides_,
        variables_,     user_defined_values_, entities_,
        signal_states_, controller_phases_,   current_date_time_seconds(clock_.now()),
        diagnostics_,   warned_values_};
    scheduler_.step(context, [this](const ir::Action& action) { return apply(action); });

    for (auto& [id, record] : entities_) {
        (void)id;
        if (!record.active) {
            continue; // a deleted entity does not move
        }
        if (record.mode == ir::ControlMode::EngineControlled) {
            if (record.trajectory_moved) {
                // A trajectory follower already placed this entity for this
                // step (§6.9): its position comes from the path, not from the
                // straight-line model. Only the source of the integrate phase
                // changes — the phase order of step() is untouched (ADR-0014).
                record.trajectory_moved = false;
                continue;
            }
            // Straight-line kinematics: placeholder physics for this phase.
            // det_sincos, not libm, so the integration is bit-identical across
            // platforms (see scena/runtime/detmath.h and the determinism
            // contract in docs/user-guide/determinism.md).
            const runtime::SinCos hs = runtime::det_sincos(record.state.heading);
            record.state.x += record.state.speed * hs.cos * dt;
            record.state.y += record.state.speed * hs.sin * dt;
            // z, pitch, and roll are left untouched: the ground-plane
            // straight-line model integrates position from speed and heading
            // only. A host-controlled entity may carry any pose via report_state.
        }
    }

    if (gateway_ != nullptr) {
        for (const auto& [id, record] : entities_) {
            if (record.active && record.mode == ir::ControlMode::EngineControlled) {
                gateway_->publish_state(id, record.state);
            }
        }
    }

    return Status::Ok;
}

void Engine::refresh_observations(double dt) {
    // Precondition: dt > 0 (the caller skips dt == 0). Iterates entities_ in
    // std::map order for determinism.
    for (auto& [id, record] : entities_) {
        (void)id;
        if (!record.active) {
            // Nothing accrues while an entity is out of the scenario, and
            // prev_sample stays where the delete left it — the re-add seeds a
            // fresh baseline anyway.
            continue;
        }
        if (record.has_prev_sample) {
            // Finite differences of exactly the observed snapshots. Fixed
            // operand order; std::sqrt is IEEE-exact.
            record.acceleration = (record.state.speed - record.prev_sample.speed) / dt;
            const double dx = record.state.x - record.prev_sample.x;
            const double dy = record.state.y - record.prev_sample.y;
            const double dz = record.state.z - record.prev_sample.z;
            record.traveled_distance += std::sqrt(dx * dx + dy * dy + dz * dz);
            // Standstill is exact speed == 0.0 (true for -0.0 under IEEE); the
            // standard is silent on a threshold, so none is invented (ADR-0008).
            if (record.state.speed == 0.0) {
                record.standstill_seconds += dt;
            } else {
                record.standstill_seconds = 0.0;
            }
        }
        record.prev_sample = record.state;
        record.has_prev_sample = true;
    }
}

std::optional<EntityState> Engine::state(const std::string& entity_id) const {
    const auto it = entities_.find(entity_id);
    // An entity taken out by a DeleteEntityAction reports nothing until it is
    // added back — the host sees exactly what the conditions see.
    if (it == entities_.end() || !it->second.active) {
        return std::nullopt;
    }
    return it->second.state;
}

std::optional<bool> Engine::entity_active(const std::string& entity_id) const {
    const auto it = entities_.find(entity_id);
    if (it == entities_.end()) {
        return std::nullopt; // not declared at all
    }
    return it->second.active;
}

const ir::Route* Engine::route_of(const std::string& entity_id) const {
    const auto it = entities_.find(entity_id);
    if (it == entities_.end() || !it->second.active || !it->second.route.has_value()) {
        return nullptr;
    }
    return &*it->second.route;
}

const ir::Controller* Engine::assigned_controller_of(const std::string& entity_id) const {
    const auto it = entities_.find(entity_id);
    if (it == entities_.end() || !it->second.active ||
        !it->second.assigned_controller.has_value()) {
        return nullptr;
    }
    return &*it->second.assigned_controller;
}

std::optional<ControllerActivation>
Engine::controller_activation_of(const std::string& entity_id) const {
    const auto it = entities_.find(entity_id);
    if (it == entities_.end() || !it->second.active) {
        return std::nullopt;
    }
    return it->second.activation;
}

std::optional<EntityVisibility> Engine::visibility_of(const std::string& entity_id) const {
    const auto it = entities_.find(entity_id);
    if (it == entities_.end() || !it->second.active) {
        return std::nullopt;
    }
    return it->second.visibility;
}

Status Engine::report_state(const std::string& entity_id, const EntityState& state) {
    if (!initialized_) {
        return Status::NotInitialized;
    }
    const auto it = entities_.find(entity_id);
    // An inactive entity is not currently in the scenario, so there is nothing
    // to report a state for (§EntityAction).
    if (it == entities_.end() || !it->second.active) {
        return Status::UnknownEntity;
    }
    if (it->second.mode != ir::ControlMode::HostControlled) {
        return Status::InvalidControlMode;
    }
    it->second.state = state;
    return Status::Ok;
}

Status Engine::set_variable(const std::string& name, std::string value) {
    if (!initialized_) {
        return Status::NotInitialized;
    }
    const auto it = variables_.find(name);
    if (it == variables_.end()) {
        // §6.12: variables must be declared. Setting an undeclared name is
        // host misuse, reported as a Status only (no diagnostic).
        return Status::UnknownName;
    }
    it->second = std::move(value);
    return Status::Ok;
}

std::optional<std::string> Engine::variable(const std::string& name) const {
    if (!initialized_) {
        return std::nullopt;
    }
    const auto it = variables_.find(name);
    if (it == variables_.end()) {
        return std::nullopt;
    }
    return it->second;
}

Status Engine::set_user_defined_value(const std::string& name, std::string value) {
    // External values are not declared in the scenario, so any name is
    // accepted and the value is created or updated. Allowed before and after
    // init (a pre-init value is visible at the t = 0 evaluation).
    user_defined_values_[name] = std::move(value);
    return Status::Ok;
}

std::optional<std::string> Engine::user_defined_value(const std::string& name) const {
    const auto it = user_defined_values_.find(name);
    if (it == user_defined_values_.end()) {
        return std::nullopt;
    }
    return it->second;
}

Status Engine::set_date_time(const ir::DateTime& date_time) {
    if (!date_time.valid()) {
        return Status::InvalidArgument;
    }
    // Anchor the given instant to the current simulation time; it advances
    // one-for-one from there. Settable before init (anchor_sim = 0) so a
    // pre-staged time of day is visible at the t = 0 evaluation. The host
    // setter always means an advancing clock — only an EnvironmentAction can
    // freeze it (§TimeOfDay animation="false").
    date_time_anchor_epoch_ = date_time.to_epoch_seconds();
    date_time_anchor_sim_ = clock_.now();
    date_time_set_ = true;
    date_time_animated_ = true;
    return Status::Ok;
}

const ir::Environment& Engine::environment() const noexcept {
    return environment_;
}

std::optional<double> Engine::date_time() const {
    return current_date_time_seconds(clock_.now());
}

std::optional<double> Engine::current_date_time_seconds(double simulation_time) const {
    if (!date_time_set_) {
        return std::nullopt;
    }
    // Fixed IEEE expression, no wall clock: the simulated instant is the
    // anchor plus the simulation time elapsed since the anchor was set — or
    // just the anchor, when a §TimeOfDay with animation="false" froze it.
    return date_time_anchor_epoch_ +
           (date_time_animated_ ? (simulation_time - date_time_anchor_sim_) : 0.0);
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
    variables_.clear();
    parameter_overrides_.clear();
    // User-defined values and the time-of-day anchor persist across init()
    // but not across close(): a closed engine forgets everything the host
    // staged.
    user_defined_values_.clear();
    date_time_set_ = false;
    date_time_animated_ = true;
    environment_ = ir::Environment{};
    // The controller runtimes borrow into scenario_, so they must go before it.
    signal_controllers_.clear();
    signal_states_.clear();
    controller_phases_.clear();
    warned_values_.clear();
    scenario_ = ir::Scenario{};
    initialized_ = false;
    return Status::Ok;
}

Engine::EntityRecord* Engine::record_for(const ir::Action& action) {
    const auto it = entities_.find(action.entity_id());
    if (it != entities_.end() && it->second.active) {
        return &it->second;
    }
    if (it != entities_.end()) {
        // The entity is declared but a DeleteEntityAction took it out of the
        // scenario. "A general prerequisite for private actions is that the
        // actor is a valid entity" (§7.5.2.2), so the action stops; the
        // scheduler sees the Complete its caller returns and ends the owning
        // event on the next evaluation. Warn-once: an event re-polled every
        // step must not flood the sink.
        warn_once("entities/" + action.entity_id(), Status::UnknownEntity,
                  "action '" + std::string(action.kind()) + "' targets inactive entity '" +
                      action.entity_id() + "'; action stopped (§7.5.2.2)");
        return nullptr;
    }
    // init() validates every action target, so this is defensive: a
    // Warning, not a failure. The run continues; the action is skipped.
    Diagnostic diagnostic;
    diagnostic.severity = Severity::Warning;
    diagnostic.code = Status::UnknownEntity;
    diagnostic.message =
        "action targets unknown entity '" + action.entity_id() + "'; action skipped";
    diagnostic.path = "entities/" + action.entity_id();
    diagnostics_.report(std::move(diagnostic));
    return nullptr;
}

void Engine::deactivate_entity(EntityRecord& record) {
    record.active = false;
    // Motion and domain ownership: nothing may keep driving an entity that is
    // no longer in the scenario, and nothing may survive into a later re-add.
    record.longitudinal.reset();
    record.active_longitudinal_action = nullptr;
    record.continuous_speed.reset();
    record.trajectory.reset();
    record.trajectory_moved = false;
    record.retired_actions.clear();
    // Assignments (§6.8.2, §AssignControllerAction, §VisibilityAction) are
    // runtime state too, so a re-added entity starts from the defaults.
    record.route.reset();
    record.assigned_controller.reset();
    record.activation = ControllerActivation{};
    record.visibility = EntityVisibility{};
    // Derived observations: the odometer and the standstill timer restart, and
    // acceleration goes absent so no finite difference straddles the gap.
    record.has_prev_sample = false;
    record.acceleration.reset();
    record.traveled_distance = 0.0;
    record.standstill_seconds = 0.0;
    // The declared immutables (mode, bounding_box, performance) stay: they come
    // from the Entities section, which a delete does not touch.
}

void Engine::activate_entity(EntityRecord& record, const ir::WorldPosition& position) {
    record.active = true;
    record.state = EntityState{};
    record.state.x = position.x;
    record.state.y = position.y;
    record.state.z = position.z;
    // Seed the observation baseline from the entity's own arrival state, the
    // same way init() seeds it after the init actions: the first step after the
    // add differences against where the entity actually appeared, so there is
    // no phantom acceleration or teleport-sized odometer jump.
    record.prev_sample = record.state;
    record.has_prev_sample = true;
}

void Engine::apply_signal_phase(const std::string& controller_name,
                                SignalControllerRuntime& runtime, std::size_t index) {
    const ir::Phase& phase = runtime.controller->phases[index];
    // Document order within the phase: two states naming the same signal make
    // the later one win, deterministically.
    for (const ir::TrafficSignalState& state : phase.signal_states) {
        signal_states_[state.traffic_signal_id] = state.state;
    }
    controller_phases_[controller_name] = phase.name;
    runtime.applied_phase = index;
}

void Engine::advance_signal_controllers(double t) {
    // Sorted-map order over controller names: deterministic, and the only
    // ordering that matters — two controllers driving the same signal id in
    // the same evaluation resolve the same way on every platform.
    for (auto& [name, runtime] : signal_controllers_) {
        if (runtime.controller == nullptr || runtime.controller->phases.empty()) {
            continue;
        }
        const double local = t - runtime.anchor;
        if (local < 0.0) {
            // §6.11.3: the controller's first phase has not started yet, so it
            // drives nothing and its phase is unobservable.
            continue;
        }
        std::size_t index = 0;
        if (runtime.total > 0.0) {
            // std::fmod is exact in IEEE — the cycle position is recomputed
            // from t every step rather than accumulated, so it cannot drift
            // with the host's step pattern.
            index = phase_index(runtime.cumulative, std::fmod(local, runtime.total));
        }
        // total == 0 with phases present: the cycle has no length to advance
        // through, so the first phase is held (no fmod(x, 0) NaN). Warned about
        // at load time.
        if (runtime.applied_phase.has_value() && *runtime.applied_phase == index) {
            continue; // write on transition only, so an action's override stands
        }
        apply_signal_phase(name, runtime, index);
    }
}

std::optional<std::string> Engine::traffic_signal_state(const std::string& name) const {
    const auto it = signal_states_.find(name);
    if (it == signal_states_.end()) {
        return std::nullopt;
    }
    return it->second;
}

std::optional<std::string> Engine::traffic_signal_controller_phase(const std::string& name) const {
    const auto it = controller_phases_.find(name);
    if (it == controller_phases_.end()) {
        return std::nullopt;
    }
    return it->second;
}

void Engine::warn_once(std::string path, Status code, std::string message) {
    if (!warned_values_.insert(path).second) {
        return; // already reported for this path in this run
    }
    warn(diagnostics_, code, std::move(message), std::move(path));
}

std::optional<runtime::ActionOutcome> Engine::apply_global_action(const ir::GlobalAction& action) {
    // Every action here is a cheap state write that "completes immediately (does
    // not consume simulation time)" (Annex A Tables 11 and 12), so none of them
    // touches record_for or the domain-ownership machinery.

    /// Applies a modify operator to a stored string value, or reports rule
    /// C.2.6 and leaves it alone. `store` is the map the value lives in.
    const auto modify_named_value = [this](NamedValueStore& store, const std::string& name,
                                           ir::ModifyOperator op, double operand,
                                           const std::string& path) {
        const auto it = store.find(name);
        if (it == store.end()) {
            return; // init rejects an undeclared ref; defensive
        }
        const std::optional<double> current = ir::parse_scalar(it->second);
        if (!current.has_value()) {
            // per rule
            // asam.net:xosc:1.2.0:data_type.variable_modification_or_comparison_possible:
            // a modify action "shall only act on int, unsignedInt,
            // unsignedShort or double types". Scena has no typed declarations
            // yet (p4-s3), so scalar-convertibility stands in for the type
            // check and the action is a no-op rather than a run-ending error.
            warn_once(path, Status::UnsupportedFeature,
                      "modify action on non-numeric value '" + name + "' = '" + it->second +
                          "'; action ignored");
            return;
        }
        // One fixed IEEE expression per operator — no reassociation, so the
        // result is bit-identical everywhere.
        const double result =
            op == ir::ModifyOperator::Add ? *current + operand : *current * operand;
        it->second = ir::format_scalar(result);
    };

    /// Warns once that a deprecated parameter action ran, then hands back the
    /// overlay slot it writes. The overlay is seeded from the declaration so a
    /// modify action reads the §9.1 value the first time.
    const auto parameter_slot = [this](const std::string& name, const char* successor) {
        warn_once("parameters/" + name, Status::DeprecatedFeature,
                  "parameter action on '" + name + "' is deprecated with version 1.2; use the " +
                      successor);
        const auto existing = parameter_overrides_.find(name);
        if (existing == parameter_overrides_.end()) {
            const auto declared = scenario_.parameters.find(name);
            parameter_overrides_[name] =
                declared != scenario_.parameters.end() ? declared->second : std::string{};
        }
    };

    if (const auto* variable_set = dynamic_cast<const ir::VariableSetAction*>(&action)) {
        // §6.12: a variable's value changes during the run; the store is the one
        // the VariableCondition and the host both read.
        const auto it = variables_.find(variable_set->variable_ref());
        if (it != variables_.end()) {
            it->second = variable_set->value();
        }
        return runtime::ActionOutcome::Complete;
    }

    if (const auto* variable_modify = dynamic_cast<const ir::VariableModifyAction*>(&action)) {
        modify_named_value(variables_, variable_modify->variable_ref(), variable_modify->op(),
                           variable_modify->value(),
                           "variables/" + variable_modify->variable_ref());
        return runtime::ActionOutcome::Complete;
    }

    if (const auto* parameter_set = dynamic_cast<const ir::ParameterSetAction*>(&action)) {
        parameter_slot(parameter_set->parameter_ref(), "variable set action");
        parameter_overrides_[parameter_set->parameter_ref()] = parameter_set->value();
        return runtime::ActionOutcome::Complete;
    }

    if (const auto* parameter_modify = dynamic_cast<const ir::ParameterModifyAction*>(&action)) {
        parameter_slot(parameter_modify->parameter_ref(), "variable modify action");
        modify_named_value(parameter_overrides_, parameter_modify->parameter_ref(),
                           parameter_modify->op(), parameter_modify->value(),
                           "parameters/" + parameter_modify->parameter_ref() + "/modify");
        return runtime::ActionOutcome::Complete;
    }

    if (const auto* command = dynamic_cast<const ir::CustomCommandAction*>(&action)) {
        // §7.4.3: the type and content are a host↔author contract, handed over
        // verbatim. Without a gateway there is nobody to hand them to, and a
        // host that ignores the action is the documented contract — so no
        // diagnostic either way.
        if (gateway_ != nullptr) {
            gateway_->on_custom_command(command->type(), command->content());
        }
        return runtime::ActionOutcome::Complete;
    }

    if (const auto* signal = dynamic_cast<const ir::TrafficSignalStateAction*>(&action)) {
        // Actions win: this write lands after the controllers have ticked for
        // this evaluation, so a forced state (the §11.12 broken traffic light)
        // stands until the controlling cycle's next phase transition.
        signal_states_[signal->name()] = signal->state();
        return runtime::ActionOutcome::Complete;
    }

    if (const auto* controller_action =
            dynamic_cast<const ir::TrafficSignalControllerAction*>(&action)) {
        const auto it =
            signal_controllers_.find(controller_action->traffic_signal_controller_ref());
        if (it != signal_controllers_.end() && it->second.controller != nullptr) {
            SignalControllerRuntime& runtime = it->second;
            const std::vector<ir::Phase>& phases = runtime.controller->phases;
            for (std::size_t index = 0; index < phases.size(); ++index) {
                if (phases[index].name != controller_action->phase()) {
                    continue;
                }
                // Restart the cycle at the named phase: re-anchor so that the
                // current instant sits exactly at that phase's start, and the
                // cycle continues from there in declared order (§6.11.4).
                runtime.anchor = clock_.now() - runtime.cumulative[index];
                runtime.applied_phase.reset();
                // Tick this one controller now, so conditions evaluated later
                // in the same walk observe the phase the action just set.
                apply_signal_phase(controller_action->traffic_signal_controller_ref(), runtime,
                                   index);
                break;
            }
        }
        return runtime::ActionOutcome::Complete;
    }

    if (const auto* environment = dynamic_cast<const ir::EnvironmentAction*>(&action)) {
        // §Environment: "If one of the conditions is missing it means that it
        // doesn't change" — so the update is a member-wise merge, at both
        // levels (the Environment's members, and the Weather's within it).
        const ir::Environment& update = environment->environment();
        if (!update.name.empty()) {
            environment_.name = update.name;
        }
        if (update.road_condition.has_value()) {
            environment_.road_condition = update.road_condition;
        }
        if (update.weather.has_value()) {
            const ir::Weather& incoming = *update.weather;
            ir::Weather& current = environment_.weather.has_value()
                                       ? *environment_.weather
                                       : environment_.weather.emplace();
            const auto merge = [](auto& target, const auto& source) {
                if (source.has_value()) {
                    target = source;
                }
            };
            merge(current.sun, incoming.sun);
            merge(current.fog, incoming.fog);
            merge(current.precipitation, incoming.precipitation);
            merge(current.wind, incoming.wind);
            merge(current.temperature, incoming.temperature);
            merge(current.atmospheric_pressure, incoming.atmospheric_pressure);
            merge(current.fractional_cloud_cover_oktas, incoming.fractional_cloud_cover_oktas);
        }
        if (update.time_of_day.has_value()) {
            const ir::TimeOfDay& time_of_day = *update.time_of_day;
            environment_.time_of_day = time_of_day;
            // The one member with runtime meaning: it re-anchors the same
            // simulated clock Engine::set_date_time drives, and decides
            // whether that clock advances (§TimeOfDay animation).
            if (time_of_day.date_time.valid()) {
                date_time_anchor_epoch_ = time_of_day.date_time.to_epoch_seconds();
                date_time_anchor_sim_ = clock_.now();
                date_time_set_ = true;
                date_time_animated_ = time_of_day.animation;
            }
        }
        return runtime::ActionOutcome::Complete;
    }

    if (const auto* add = dynamic_cast<const ir::AddEntityAction*>(&action)) {
        // §EntityAction: "An entity can only exist in one copy. Adding an
        // already active entity will have no effect."
        const auto it = entities_.find(add->entity_ref());
        if (it != entities_.end() && !it->second.active) {
            activate_entity(it->second, add->position());
        }
        return runtime::ActionOutcome::Complete;
    }

    if (const auto* remove = dynamic_cast<const ir::DeleteEntityAction*>(&action)) {
        // "neither will deleting an already inactive entity" — a no-op, not a
        // diagnostic.
        const auto it = entities_.find(remove->entity_ref());
        if (it != entities_.end() && it->second.active) {
            deactivate_entity(it->second);
        }
        return runtime::ActionOutcome::Complete;
    }

    return std::nullopt; // not implemented; the caller reports it
}

runtime::ActionOutcome Engine::apply(const ir::Action& action) {
    // Global actions first: they write engine-level state, never an entity
    // record, so they short-circuit the whole private-action chain below.
    if (const auto* global = dynamic_cast<const ir::GlobalAction*>(&action)) {
        if (const std::optional<runtime::ActionOutcome> outcome = apply_global_action(*global)) {
            return *outcome;
        }
    }

    const auto* speed_action = dynamic_cast<const ir::SpeedAction*>(&action);
    const auto* profile_action = dynamic_cast<const ir::SpeedProfileAction*>(&action);
    const auto* distance_action = dynamic_cast<const ir::LongitudinalDistanceAction*>(&action);
    if (speed_action != nullptr || profile_action != nullptr || distance_action != nullptr) {
        // Longitudinal actions share the default controller and the single
        // longitudinal-domain ownership slot (Annex A Table 10: SpeedAction,
        // SpeedProfileAction and LongitudinalDistanceAction all assign a
        // longitudinal control strategy).
        EntityRecord* found = record_for(action);
        if (found == nullptr) {
            return runtime::ActionOutcome::Complete;
        }
        EntityRecord& record = *found;

        // A relative speed target needs its reference entity to be in the
        // scenario (§7.5.2.2 prerequisites); a DeleteEntityAction on the
        // reference stops the action, one-shot and continuous alike.
        if (speed_action != nullptr && speed_action->is_relative()) {
            const std::string& reference = speed_action->relative_target()->entity_ref;
            const auto reference_it = entities_.find(reference);
            if (reference_it == entities_.end() || !reference_it->second.active) {
                warn_once("entities/" + reference + "/speedReference", Status::UnknownEntity,
                          "speed action reference entity '" + reference +
                              "' is not in the scenario (§7.5.2.2); action stopped");
                if (record.active_longitudinal_action == &action) {
                    record.active_longitudinal_action = nullptr;
                    record.longitudinal.reset();
                    record.continuous_speed.reset();
                }
                return runtime::ActionOutcome::Complete;
            }
        }

        // A longitudinal action superseded by a newer one, re-polled while still
        // in its event's running set: retire it. It completes so its event can
        // end (§8.4.2) and never reinstalls itself to fight the current owner.
        const auto retired =
            std::find(record.retired_actions.begin(), record.retired_actions.end(), &action);
        if (retired != record.retired_actions.end()) {
            record.retired_actions.erase(retired);
            return runtime::ActionOutcome::Complete;
        }

        // A longitudinal action fired while the engine does not control that
        // domain has a missing prerequisite: it is reported and skipped rather
        // than fighting a host controller for the entity (§7.5.2.2, ADR-0014).
        if (!record.activation.longitudinal) {
            report_inactive_domain(action, "longitudinal");
            return runtime::ActionOutcome::Complete;
        }

        const bool is_owner = record.active_longitudinal_action == &action;

        // Distance keeping (§LongitudinalDistanceAction): a controller that
        // re-measures the gap every step rather than a finite transition, so it
        // installs no LongitudinalController — the same shape as a continuous
        // relative speed target.
        if (distance_action != nullptr) {
            if (!is_owner) {
                supersede_longitudinal(record, &action);
                record.longitudinal.reset();
                record.continuous_speed.reset();
                record.active_longitudinal_action = &action;
            }
            return drive_distance_keeping(*distance_action, record);
        }

        // Continuous relative speed (§7.5.3): a controller keeps matching the
        // reference entity's speed and the action never ends by itself. Modeled
        // as a re-polled action that reports Running indefinitely; the tracking
        // ends only when a later longitudinal action supersedes it or a
        // stopTransition releases it. This branch handles both the first fire
        // and every re-poll (is_owner already true).
        if (speed_action != nullptr && speed_action->is_relative() &&
            speed_action->relative_target()->continuous) {
            if (!is_owner) {
                supersede_longitudinal(record, &action);
            }
            record.longitudinal.reset(); // continuous uses no finite controller
            record.continuous_speed = *speed_action->relative_target();
            record.active_longitudinal_action = &action;
            double target = resolve_relative_speed(*record.continuous_speed, record);
            if (record.performance.has_value() && record.performance->max_speed > 0.0 &&
                target > record.performance->max_speed) {
                target = record.performance->max_speed;
            }
            record.state.speed = target;
            return runtime::ActionOutcome::Running;
        }

        if (is_owner) {
            // Owning finite controller's re-poll: the controller argument is unused.
            return drive_longitudinal(action, record, runtime::LongitudinalController{});
        }
        // First application, or a finite longitudinal action superseding whatever
        // drove this entity — including a running continuous relative target.
        supersede_longitudinal(record, &action);
        record.continuous_speed.reset();
        return drive_longitudinal(action, record,
                                  speed_action != nullptr
                                      ? build_speed_controller(*speed_action, record)
                                      : build_profile_controller(*profile_action, record));
    }

    if (const auto* teleport = dynamic_cast<const ir::TeleportAction*>(&action)) {
        // §TeleportAction: a step (instantaneous) action. Scena resolves the
        // world-frame target only; the PositionResolver and the other §6.3.8
        // variants arrive with p2-s4/p3-s4. Orientation is part of the full
        // Position and is not modeled yet, so heading/pitch/roll are left as-is.
        EntityRecord* found = record_for(action);
        if (found == nullptr) {
            return runtime::ActionOutcome::Complete;
        }
        EntityRecord& record = *found;
        const ir::WorldPosition& position = teleport->position();
        record.state.x = position.x;
        record.state.y = position.y;
        record.state.z = position.z;
        // A host-controlled entity's reported state overwrites this on the next
        // poll; the formal host round-trip for teleport is p2-s4.
        return runtime::ActionOutcome::Complete;
    }

    if (const auto* assign_route = dynamic_cast<const ir::AssignRouteAction*>(&action)) {
        // §AssignRouteAction: installs routing intent and completes immediately
        // (Annex A Table 10). It "does not override any action that controls
        // either lateral or longitudinal domain" (§7.4.1.4), so it deliberately
        // does not touch the domain ownership. Route-following motion needs a
        // road network (p3-s4); until then the route is state a host can read.
        EntityRecord* found = record_for(action);
        if (found == nullptr) {
            return runtime::ActionOutcome::Complete;
        }
        found->route = assign_route->route(); // overwrites any prior route (§6.8.2)
        return runtime::ActionOutcome::Complete;
    }

    if (const auto* acquire = dynamic_cast<const ir::AcquirePositionAction*>(&action)) {
        // §7.4.1.4: "a route with two waypoints is created: current position as
        // first and specified position as last waypoint". The entity "aims to
        // take the shortest distance", hence the Shortest strategy. The current
        // position is read at apply time, so when the action fires matters.
        EntityRecord* found = record_for(action);
        if (found == nullptr) {
            return runtime::ActionOutcome::Complete;
        }
        ir::Route route;
        route.closed = false;
        route.waypoints.push_back(
            ir::Waypoint{ir::WorldPosition{found->state.x, found->state.y, found->state.z},
                         ir::RouteStrategy::Shortest});
        route.waypoints.push_back(ir::Waypoint{acquire->position(), ir::RouteStrategy::Shortest});
        found->route = std::move(route);
        return runtime::ActionOutcome::Complete;
    }

    if (const auto* follow = dynamic_cast<const ir::FollowTrajectoryAction*>(&action)) {
        EntityRecord* found = record_for(action);
        if (found == nullptr) {
            return runtime::ActionOutcome::Complete;
        }
        EntityRecord& record = *found;
        // A follower superseded by a newer trajectory (or by a longitudinal
        // action, for a timing-mode one) still gets re-polled by its event:
        // retire it rather than let it reinstall itself.
        const auto retired =
            std::find(record.retired_actions.begin(), record.retired_actions.end(), &action);
        if (retired != record.retired_actions.end()) {
            record.retired_actions.erase(retired);
            return runtime::ActionOutcome::Complete;
        }
        // Every trajectory steers (Table 10: all four rows assign a lateral
        // control strategy), and a timed one drives the speed as well; if the
        // engine does not control a domain the action needs, it is skipped.
        if (!record.activation.lateral) {
            report_inactive_domain(action, "lateral");
            return runtime::ActionOutcome::Complete;
        }
        if (follow->time_reference().has_value() && !record.activation.longitudinal) {
            report_inactive_domain(action, "longitudinal");
            return runtime::ActionOutcome::Complete;
        }
        return drive_trajectory(*follow, record);
    }

    if (const auto* assign_controller = dynamic_cast<const ir::AssignControllerAction*>(&action)) {
        // §AssignControllerAction: assigns the controller model and optionally
        // activates or deactivates domains; completes immediately (Table 10).
        // Scena implements no controller models, so the assignment is state the
        // host reads — the gateway hears about it at this exact point in the
        // step (ADR-0014).
        EntityRecord* found = record_for(action);
        if (found == nullptr) {
            return runtime::ActionOutcome::Complete;
        }
        found->assigned_controller = assign_controller->controller();
        if (gateway_ != nullptr) {
            gateway_->on_controller_assigned(action.entity_id(), *found->assigned_controller);
        }
        apply_activation(*found, assign_controller->activate_lateral(),
                         assign_controller->activate_longitudinal());
        return runtime::ActionOutcome::Complete;
    }

    if (const auto* activate = dynamic_cast<const ir::ActivateControllerAction*>(&action)) {
        // §ActivateControllerAction: toggles controlled behavior per domain and
        // completes immediately (Table 10). Scena's engine is the default
        // controller, so this toggles the engine's own control (ADR-0014).
        EntityRecord* found = record_for(action);
        if (found == nullptr) {
            return runtime::ActionOutcome::Complete;
        }
        apply_activation(*found, activate->lateral(), activate->longitudinal());
        return runtime::ActionOutcome::Complete;
    }

    if (const auto* visibility = dynamic_cast<const ir::VisibilityAction*>(&action)) {
        // §VisibilityAction: all three flags are required by the XSD, so the
        // action always states a complete visibility. Completes immediately
        // (Table 10).
        EntityRecord* found = record_for(action);
        if (found == nullptr) {
            return runtime::ActionOutcome::Complete;
        }
        found->visibility =
            EntityVisibility{visibility->graphics(), visibility->sensors(), visibility->traffic()};
        if (gateway_ != nullptr) {
            gateway_->on_visibility_changed(action.entity_id(), found->visibility);
        }
        return runtime::ActionOutcome::Complete;
    }

    // An action kind the engine does not implement yet. A parser must never
    // silently drop input, and neither does the runtime: emit a Warning and
    // keep going. Scheduling is unchanged — the event completes in one
    // evaluation.
    Diagnostic diagnostic;
    diagnostic.severity = Severity::Warning;
    diagnostic.code = Status::UnsupportedFeature;
    if (action.entity_id().empty()) {
        // An actor-less action has no entity to key the path on (§7.4.2), so it
        // is addressed by its kind instead.
        diagnostic.message =
            "unsupported action kind '" + std::string(action.kind()) + "'; action ignored";
        diagnostic.path = "actions/" + std::string(action.kind());
    } else {
        diagnostic.message = "unsupported action kind '" + std::string(action.kind()) +
                             "' targeting entity '" + action.entity_id() + "'; action ignored";
        diagnostic.path = "entities/" + action.entity_id();
    }
    diagnostics_.report(std::move(diagnostic));
    return runtime::ActionOutcome::Complete;
}

runtime::ActionOutcome Engine::drive_longitudinal(const ir::Action& action, EntityRecord& record,
                                                  runtime::LongitudinalController controller) {
    if (record.active_longitudinal_action == &action) {
        // Owner: advance the installed controller by the current step. The
        // distance travelled this step (for a distance-dimensioned transition)
        // is estimated from the speed at the start of the step — the same
        // explicit scheme the position integrator uses (ADR-0011).
        const double step_distance = record.state.speed * last_dt_;
        record.state.speed = record.longitudinal->advance(last_dt_, step_distance);
        if (record.longitudinal->done()) {
            record.longitudinal.reset();
            record.active_longitudinal_action = nullptr;
            return runtime::ActionOutcome::Complete;
        }
        return runtime::ActionOutcome::Running;
    }

    // First application (or a later action superseding whatever drove this
    // entity): advance the fresh controller by a zero step to consume any
    // instantaneous (Step / zero-duration) segments up front.
    const double settled = controller.advance(0.0, 0.0);
    record.state.speed = settled;
    if (controller.done()) {
        // Instantaneous: reached its goal in this evaluation. Any previous
        // transition on this entity is dropped.
        record.longitudinal.reset();
        record.active_longitudinal_action = nullptr;
        return runtime::ActionOutcome::Complete;
    }
    record.longitudinal = std::move(controller);
    record.active_longitudinal_action = &action;
    return runtime::ActionOutcome::Running;
}

runtime::ActionOutcome Engine::drive_distance_keeping(const ir::LongitudinalDistanceAction& action,
                                                      EntityRecord& record) {
    // Releases the longitudinal domain and completes: used for every path that
    // cannot keep the requested distance (§7.5.2.2 missing prerequisites end an
    // action with a stopTransition).
    const auto give_up = [&record, &action](DiagnosticSink& sink, std::string message) {
        record.active_longitudinal_action = nullptr;
        warn(sink, Status::UnsupportedFeature, std::move(message),
             "entities/" + action.entity_id());
        return runtime::ActionOutcome::Complete;
    };

    const ir::CoordinateSystem cs = action.coordinate_system();
    if (cs == ir::CoordinateSystem::Lane || cs == ir::CoordinateSystem::Road ||
        cs == ir::CoordinateSystem::Trajectory) {
        // Road-based gaps need IRoadQuery (p3-s4); warned about at init too.
        return give_up(diagnostics_, "longitudinal distance action needs a road network for its "
                                     "coordinate system (§6.4); action completed");
    }
    const auto reference_it = entities_.find(action.entity_ref()); // heterogeneous lookup
    if (reference_it == entities_.end()) {
        return give_up(diagnostics_, "longitudinal distance action reference entity '" +
                                         action.entity_ref() + "' is unknown; action completed");
    }
    if (!reference_it->second.active) {
        // §7.5.2.1: "if the referenced entity of an instance of a
        // LongitudinalDistanceAction disappears" the action is missing a
        // prerequisite and stops. A DeleteEntityAction is exactly that.
        return give_up(diagnostics_, "longitudinal distance action reference entity '" +
                                         action.entity_ref() +
                                         "' is no longer in the scenario (§7.5.2.2); action "
                                         "completed");
    }
    // Read-only view of the reference; the actor's own record is written below,
    // and the two may be the same entity (a degenerate but harmless self-gap).
    const EntityRecord& reference = reference_it->second;

    const std::optional<double> gap =
        signed_longitudinal_gap(record.state, record.bounding_box, reference.state,
                                reference.bounding_box, cs, action.freespace());
    if (!gap.has_value()) {
        return give_up(diagnostics_, "longitudinal distance action needs bounding boxes for a "
                                     "freespace gap (§6.4.7.2); action completed");
    }

    // The target gap: an absolute distance, or a headway read against the
    // actor's own speed — the same arithmetic TimeHeadwayCondition uses.
    const double target = action.distance().has_value() ? *action.distance()
                                                        : *action.time_gap() * record.state.speed;
    // Which side of the reference the actor holds (§LongitudinalDisplacement).
    // `Any` keeps whichever side the actor is on right now.
    double desired = target;
    switch (action.displacement()) {
    case ir::LongitudinalDisplacement::TrailingReferencedEntity:
        desired = target;
        break;
    case ir::LongitudinalDisplacement::LeadingReferencedEntity:
        desired = -target;
        break;
    case ir::LongitudinalDisplacement::Any:
        desired = *gap >= 0.0 ? target : -target;
        break;
    }
    const double error = *gap - desired;

    // Table 10: a non-continuous action ends "by reaching the targeted
    // distance" — including immediately, when the gap already holds at the
    // start of the action (§7.5.2.1). A continuous one never ends (§7.5.3).
    if (!action.continuous() && std::fabs(error) <= kDistanceKeepingEpsilon) {
        record.active_longitudinal_action = nullptr;
        return runtime::ActionOutcome::Complete;
    }

    // Deadbeat command (ADR-0014): matching the reference's speed holds the gap
    // where it is, and the error term closes what is left over in exactly one
    // step, before the clamps. A zero-length step commands nothing — there is
    // no step to close the error over, and no division by zero.
    if (last_dt_ > 0.0) {
        const runtime::SinCos reference_heading = runtime::det_sincos(reference.state.heading);
        double ux = 0.0;
        double uy = 0.0;
        runtime::projection_axis(cs, ir::RelativeDistanceType::Longitudinal, record.state.heading,
                                 ux, uy);
        const double reference_speed =
            reference.state.speed * (reference_heading.cos * ux + reference_heading.sin * uy);

        const std::optional<ir::DynamicConstraints>& constraints = action.constraints();
        const std::optional<ir::Performance>& performance = record.performance;
        const auto constraint_of =
            [&constraints](
                std::optional<double> ir::DynamicConstraints::*field) -> std::optional<double> {
            return constraints.has_value() ? (*constraints).*field : std::nullopt;
        };
        const double acceleration_limit = effective_limit(
            constraint_of(&ir::DynamicConstraints::max_acceleration),
            performance.has_value() ? std::optional<double>(performance->max_acceleration)
                                    : std::nullopt);
        const double deceleration_limit = effective_limit(
            constraint_of(&ir::DynamicConstraints::max_deceleration),
            performance.has_value() ? std::optional<double>(performance->max_deceleration)
                                    : std::nullopt);
        // Approach rate: the deadbeat term closes the whole error this step,
        // but only while the entity could still brake off the resulting
        // relative speed within the error it has left. sqrt(2*a*|e|) is that
        // glide limit; without it a rate-limited controller with a large error
        // saturates its acceleration and overshoots the target (ADR-0014).
        // Closing a gap is bounded by the deceleration it will take to stop
        // closing, opening one by the acceleration it will take to stop
        // opening. Both are IEEE-exact operations, sqrt included.
        double approach = std::fabs(error) / last_dt_;
        const double glide_limit = error >= 0.0 ? deceleration_limit : acceleration_limit;
        if (std::isfinite(glide_limit)) {
            approach = std::fmin(approach, std::sqrt(2.0 * glide_limit * std::fabs(error)));
        }
        const double command = reference_speed + std::copysign(approach, error);

        double delta = command - record.state.speed;
        if (delta > acceleration_limit * last_dt_) {
            delta = acceleration_limit * last_dt_;
        } else if (delta < -deceleration_limit * last_dt_) {
            delta = -deceleration_limit * last_dt_;
        }
        double speed = record.state.speed + delta;
        // The scalar-speed model has no reverse gear, so a distance controller
        // never commands a negative speed; the entity stops instead.
        if (speed < 0.0) {
            speed = 0.0;
        }
        const double speed_limit = effective_limit(
            constraint_of(&ir::DynamicConstraints::max_speed),
            performance.has_value() ? std::optional<double>(performance->max_speed) : std::nullopt);
        if (speed > speed_limit) {
            speed = speed_limit;
        }
        record.state.speed = speed;
    }
    return runtime::ActionOutcome::Running;
}

runtime::ActionOutcome Engine::drive_trajectory(const ir::FollowTrajectoryAction& action,
                                                EntityRecord& record) {
    const bool timing = action.time_reference().has_value();

    // --- install, on the first fire -------------------------------------
    if (!record.trajectory.has_value() || record.trajectory->action != &action) {
        const std::vector<ir::TrajectoryVertex>& vertices = action.trajectory().vertices;
        if (vertices.size() < 2) {
            // init() rejects this; defensive, so a hand-built scenario cannot
            // walk off the end of the vertex list.
            warn(diagnostics_, Status::UnsupportedFeature,
                 "trajectory needs at least two vertices; action completed",
                 "entities/" + action.entity_id());
            return runtime::ActionOutcome::Complete;
        }
        // A new trajectory supersedes whatever was following before, and a
        // timing-mode one additionally takes the longitudinal domain (Table 10:
        // timeReference=timing assigns a longitudinal control strategy).
        if (record.trajectory.has_value() && record.trajectory->action != nullptr) {
            const ir::Action* outgoing = record.trajectory->action;
            if (std::find(record.retired_actions.begin(), record.retired_actions.end(), outgoing) ==
                record.retired_actions.end()) {
                record.retired_actions.push_back(outgoing);
            }
        }
        if (timing) {
            supersede_longitudinal(record, &action);
            record.longitudinal.reset();
            record.continuous_speed.reset();
            record.active_longitudinal_action = &action;
        }

        TrajectoryFollower follower;
        follower.action = &action;
        follower.timing = timing;
        follower.points.reserve(vertices.size());
        follower.arc.reserve(vertices.size());
        double arc_length = 0.0;
        for (std::size_t index = 0; index < vertices.size(); ++index) {
            if (index > 0) {
                arc_length +=
                    segment_length(vertices[index - 1].position, vertices[index].position);
            }
            follower.points.push_back(vertices[index].position);
            follower.arc.push_back(arc_length);
            if (timing && vertices[index].time.has_value()) {
                // §Timing: effective time = time * scale + offset, measured from
                // the action's start (relative) or from simulation time zero.
                const ir::Timing& adjustment = *action.time_reference();
                double effective = *vertices[index].time * adjustment.scale + adjustment.offset;
                if (adjustment.domain == ir::ReferenceContext::Relative) {
                    effective += clock_.now();
                }
                follower.times.push_back(effective);
            }
        }
        // §initialDistanceOffset: the trajectory is logically truncated so it
        // starts at that arc length. "Where a timing TimeReference is provided,
        // the time that would be taken to reach this point is deducted from all
        // calculated waypoint time values" — which is exactly what dropping the
        // skipped vertices and keeping the first time does.
        const double offset = action.initial_distance_offset();
        if (offset > 0.0 && offset < arc_length) {
            const std::size_t index = segment_index(follower.arc, offset);
            const double span = follower.arc[index + 1] - follower.arc[index];
            const double fraction = span > 0.0 ? (offset - follower.arc[index]) / span : 0.0;
            const ir::WorldPosition start =
                interpolate_position(follower.points[index], follower.points[index + 1], fraction);
            const double start_time =
                follower.times.empty() ? 0.0
                                       : follower.times.front(); // the deducted origin (see above)
            follower.points.erase(follower.points.begin(),
                                  follower.points.begin() + static_cast<std::ptrdiff_t>(index));
            follower.arc.erase(follower.arc.begin(),
                               follower.arc.begin() + static_cast<std::ptrdiff_t>(index));
            follower.points.front() = start;
            follower.arc.front() = offset;
            if (!follower.times.empty()) {
                follower.times.erase(follower.times.begin(),
                                     follower.times.begin() + static_cast<std::ptrdiff_t>(index));
                follower.times.front() = start_time;
            }
        }
        // Per-segment headings: the single det_atan2 site, resolved once so a
        // step only reads them back.
        follower.heading.reserve(follower.points.size() - 1);
        for (std::size_t index = 0; index + 1 < follower.points.size(); ++index) {
            follower.heading.push_back(
                runtime::det_atan2(follower.points[index + 1].y - follower.points[index].y,
                                   follower.points[index + 1].x - follower.points[index].x));
        }
        follower.traveled = follower.arc.front();
        record.trajectory = std::move(follower);
    }

    TrajectoryFollower& follower = *record.trajectory;
    const std::size_t last = follower.points.size() - 1;
    const double arc_end = follower.arc[last];

    // Writes the pose the follower has decided on and marks the step so the
    // straight-line integrator leaves this entity alone.
    const auto place = [&record](const ir::WorldPosition& position, double heading) {
        record.state.x = position.x;
        record.state.y = position.y;
        record.state.z = position.z;
        record.state.heading = heading;
        record.trajectory_moved = true;
    };
    // Ends the action at the final vertex (Table 10: "by reaching the end of
    // the trajectory"), snapping exactly onto it.
    const auto finish = [&](double speed) {
        place(follower.points[last], follower.heading[last - 1]);
        if (follower.timing) {
            record.state.speed = speed;
            record.active_longitudinal_action = nullptr;
        }
        record.trajectory.reset();
        return runtime::ActionOutcome::Complete;
    };

    if (!follower.timing) {
        // timeReference=none: the trajectory is pure geometry and the entity's
        // own longitudinal control sets the pace (Table 10 assigns no
        // longitudinal strategy). §6.9.1: on start the entity teleports to the
        // beginning of the trajectory.
        if (!follower.started) {
            follower.started = true;
            std::size_t index = segment_index(follower.arc, follower.traveled);
            place(follower.points.front(), follower.heading[index]);
            return runtime::ActionOutcome::Running;
        }
        follower.traveled += record.state.speed * last_dt_;
        if (follower.traveled >= arc_end) {
            return finish(record.state.speed);
        }
        const std::size_t index = segment_index(follower.arc, follower.traveled);
        const double span = follower.arc[index + 1] - follower.arc[index];
        const double fraction = span > 0.0 ? (follower.traveled - follower.arc[index]) / span : 0.0;
        place(interpolate_position(follower.points[index], follower.points[index + 1], fraction),
              follower.heading[index]);
        return runtime::ActionOutcome::Running;
    }

    // timeReference=timing: the vertex times drive the motion, so the action
    // owns the longitudinal domain too.
    const double now = clock_.now();
    if (now < follower.times.front()) {
        // §6.9.2: the action has started but its first time reference is still
        // in the future — "the entity keeps moving as before until t1".
        return runtime::ActionOutcome::Running;
    }
    if (now >= follower.times[last]) {
        // §6.9.3 in the limit: at or past the last time reference the entity is
        // at the end of the trajectory.
        const double final_span = follower.times[last] - follower.times[last - 1];
        const double final_length = follower.arc[last] - follower.arc[last - 1];
        return finish(final_span > 0.0 ? final_length / final_span : 0.0);
    }
    // §6.9.2 / §6.9.3: at t1 (or already past it) the entity is placed on the
    // time-interpolated point of the trajectory and continues from there.
    follower.started = true;
    const std::size_t index = segment_index(follower.times, now);
    const double span = follower.times[index + 1] - follower.times[index];
    const double fraction = span > 0.0 ? (now - follower.times[index]) / span : 0.0;
    place(interpolate_position(follower.points[index], follower.points[index + 1], fraction),
          follower.heading[index]);
    follower.traveled =
        follower.arc[index] + (follower.arc[index + 1] - follower.arc[index]) * fraction;
    // The speed the timed segment implies, so a trace and the by-entity
    // conditions see a consistent state (deterministic division).
    const double length = follower.arc[index + 1] - follower.arc[index];
    record.state.speed = span > 0.0 ? length / span : 0.0;
    return runtime::ActionOutcome::Running;
}

runtime::LongitudinalController Engine::build_speed_controller(const ir::SpeedAction& action,
                                                               const EntityRecord& record) const {
    const ir::TransitionDynamics& td = action.dynamics();
    const std::optional<ir::Performance>& perf = record.performance;

    runtime::LongitudinalController::Segment seg;
    seg.from = record.state.speed;
    seg.shape = td.shape;

    // A non-continuous relative target (§RelativeTargetSpeed) is resolved once,
    // against the reference entity's speed now, then reached through `td` exactly
    // like an absolute target. The continuous case never reaches this builder —
    // it is handled in apply() and re-matched every step.
    double target = action.is_relative() ? resolve_relative_speed(*action.relative_target(), record)
                                         : action.target_speed();
    // Clamp the target to the entity's maximum speed (§Performance). Only a
    // positive maxSpeed constrains; other targets pass through.
    if (perf.has_value() && perf->max_speed > 0.0 && target > perf->max_speed) {
        target = perf->max_speed;
    }
    seg.to = target;

    if (td.dimension == ir::DynamicsDimension::Distance) {
        seg.by_distance = true;
        seg.span = td.value; // metres; <= 0 ⇒ instantaneous
    } else {
        double duration = runtime::transition_duration(td, seg.from, seg.to);
        // Performance acceleration clamp (time/rate dimensions): extend the
        // duration so the transition's peak acceleration stays within the
        // envelope. Direction picks the accel or decel limit (ADR-0011).
        if (perf.has_value() && td.shape != ir::DynamicsShape::Step) {
            const double delta = std::fabs(seg.to - seg.from);
            const double limit =
                seg.to >= seg.from ? perf->max_acceleration : perf->max_deceleration;
            if (delta > 0.0 && limit > 0.0) {
                const double min_duration =
                    runtime::shape_peak_gradient_factor(td.shape) * delta / limit;
                if (min_duration > duration) {
                    duration = min_duration;
                }
            }
        }
        seg.span = duration;
    }

    runtime::LongitudinalController controller;
    controller.segments.push_back(seg);
    return controller;
}

void Engine::release_longitudinal_domain(EntityRecord& record) {
    // Retires the current owner (it completes on its next re-poll, the
    // stopTransition analog of §7.5.2.1) and leaves the entity at its current
    // speed — the engine simply stops commanding it.
    supersede_longitudinal(record, nullptr);
    record.longitudinal.reset();
    record.continuous_speed.reset();
    record.active_longitudinal_action = nullptr;
}

void Engine::release_lateral_domain(EntityRecord& record) {
    if (!record.trajectory.has_value()) {
        return;
    }
    const ir::Action* outgoing = record.trajectory->action;
    if (outgoing != nullptr &&
        std::find(record.retired_actions.begin(), record.retired_actions.end(), outgoing) ==
            record.retired_actions.end()) {
        record.retired_actions.push_back(outgoing);
    }
    // A timed trajectory also owned the longitudinal domain; releasing the
    // lateral one ends the whole action, so that ownership goes too.
    if (record.active_longitudinal_action == outgoing) {
        record.active_longitudinal_action = nullptr;
    }
    record.trajectory.reset();
}

void Engine::apply_activation(EntityRecord& record, const std::optional<bool>& lateral,
                              const std::optional<bool>& longitudinal) {
    // "If not specified: No change for controlling the dimension is applied."
    if (lateral.has_value()) {
        if (!*lateral && record.activation.lateral) {
            release_lateral_domain(record);
        }
        record.activation.lateral = *lateral;
    }
    if (longitudinal.has_value()) {
        if (!*longitudinal && record.activation.longitudinal) {
            release_longitudinal_domain(record);
        }
        record.activation.longitudinal = *longitudinal;
    }
}

void Engine::report_inactive_domain(const ir::Action& action, const char* domain) {
    Diagnostic diagnostic;
    diagnostic.severity = Severity::Warning;
    diagnostic.code = Status::InvalidControlMode;
    diagnostic.message = "action '" + std::string(action.kind()) + "' needs the " +
                         std::string(domain) + " domain of entity '" + action.entity_id() +
                         "', which is deactivated; action skipped";
    diagnostic.path = "entities/" + action.entity_id();
    diagnostics_.report(std::move(diagnostic));
}

void Engine::supersede_longitudinal(EntityRecord& record, const ir::Action* incoming) {
    const ir::Action* outgoing = record.active_longitudinal_action;
    if (outgoing == nullptr || outgoing == incoming) {
        return;
    }
    // The outgoing owner is still Running in its own event; mark it so its next
    // re-poll reports Complete rather than reinstalling itself. Guard against a
    // duplicate entry (a paranoid check — a given action owns the entity once).
    if (std::find(record.retired_actions.begin(), record.retired_actions.end(), outgoing) ==
        record.retired_actions.end()) {
        record.retired_actions.push_back(outgoing);
    }
    // A timing-mode trajectory owns the longitudinal domain through its
    // follower, so superseding it must also stop the follower — otherwise the
    // path would keep writing positions the new owner knows nothing about.
    if (record.trajectory.has_value() && record.trajectory->action == outgoing) {
        record.trajectory.reset();
    }
}

double Engine::resolve_relative_speed(const ir::RelativeTargetSpeed& target,
                                      const EntityRecord& record) const {
    // The reference is validated to exist at init; the fallback to the actor's
    // own speed is defensive (yields a no-op delta / identity factor) and never
    // taken on a validated scenario.
    // An inactive reference is treated like a missing one: apply() stops the
    // action before it gets here, so this fallback is defensive only.
    const auto it = entities_.find(target.entity_ref); // heterogeneous lookup
    const double reference_speed =
        it != entities_.end() && it->second.active ? it->second.state.speed : record.state.speed;
    switch (target.value_type) {
    case ir::SpeedTargetValueType::Delta:
        return reference_speed + target.value;
    case ir::SpeedTargetValueType::Factor:
        return reference_speed * target.value;
    }
    return reference_speed;
}

runtime::LongitudinalController
Engine::build_profile_controller(const ir::SpeedProfileAction& action,
                                 const EntityRecord& record) const {
    const std::optional<ir::Performance>& perf = record.performance;
    runtime::LongitudinalController controller;
    double from = record.state.speed;
    for (const ir::SpeedProfileEntry& entry : action.entries()) {
        double to = entry.speed;
        if (perf.has_value() && perf->max_speed > 0.0 && to > perf->max_speed) {
            to = perf->max_speed;
        }
        runtime::LongitudinalController::Segment seg;
        seg.from = from;
        seg.to = to;
        // Position mode: strictly linear interpolation between targets
        // (§SpeedProfileAction).
        seg.shape = ir::DynamicsShape::Linear;
        seg.by_distance = false;
        if (entry.time.has_value()) {
            // Authored duration is honoured exactly (strict linear); a zero
            // time is an instantaneous jump.
            seg.span = *entry.time;
        } else {
            // No time: reach the target as soon as the Performance envelope
            // allows. Without a Performance limit the jump is instantaneous.
            const double delta = std::fabs(to - from);
            const double limit = to >= from ? (perf.has_value() ? perf->max_acceleration : 0.0)
                                            : (perf.has_value() ? perf->max_deceleration : 0.0);
            seg.span = limit > 0.0 && delta > 0.0 ? delta / limit : 0.0;
        }
        controller.segments.push_back(seg);
        from = to;
    }
    return controller;
}

void Engine::finalize_longitudinal(const ir::Action& action, EntityRecord& record) {
    if (record.active_longitudinal_action == &action && record.longitudinal.has_value() &&
        !record.longitudinal->segments.empty()) {
        // §8.5: init actions are instantaneous — jump to the terminal target.
        record.state.speed = record.longitudinal->segments.back().to;
    }
    // A continuous relative target used as an init action has already set the
    // resolved speed in apply(); §8.5 makes it instantaneous, so just release
    // the tracking rather than leaving it live into the run.
    record.longitudinal.reset();
    record.continuous_speed.reset();
    record.retired_actions.clear();
    record.active_longitudinal_action = nullptr;
    // A trajectory follower installed by an init action has already placed the
    // entity on the path; §8.5 makes that instantaneous, so the follower is
    // released rather than left live into the run.
    record.trajectory.reset();
}

} // namespace scena
