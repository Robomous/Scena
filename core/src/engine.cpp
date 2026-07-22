// SPDX-License-Identifier: Apache-2.0
#include "scena/engine.h"

#include <cmath>
#include <cstddef>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <variant>

#include "scena/gateway/simulator_gateway.h"
#include "scena/ir/condition.h"
#include "scena/ir/entity_condition.h"
#include "scena/ir/evaluation_context.h"
#include "scena/ir/interaction_condition.h"
#include "scena/ir/rule.h"
#include "scena/runtime/detmath.h"
#include "scena/runtime/element_ref.h"

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
                   const NamedValueStore& variables, const NamedValueStore& user_defined_values,
                   const EntityMap& entities, std::optional<double> date_time_seconds,
                   DiagnosticSink& sink, std::set<std::string>& warned)
        : simulation_time_(simulation_time), parameters_(&parameters), variables_(&variables),
          user_defined_values_(&user_defined_values), entities_(&entities),
          date_time_seconds_(date_time_seconds), sink_(&sink), warned_(&warned) {}

    [[nodiscard]] double simulation_time() const override { return simulation_time_; }

    [[nodiscard]] std::optional<ir::EntityKinematics>
    entity_kinematics(std::string_view id) const override {
        const auto it = entities_->find(id); // heterogeneous lookup (std::less<>)
        if (it == entities_->end()) {
            return std::nullopt;
        }
        const auto& record = it->second;
        return ir::EntityKinematics{record.state, record.acceleration, record.traveled_distance,
                                    record.standstill_seconds, record.bounding_box};
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
        case ir::NamedValueKind::Parameter:
            store = parameters_;
            break;
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
    const NamedValueStore* variables_;
    const NamedValueStore* user_defined_values_;
    const EntityMap* entities_;
    std::optional<double> date_time_seconds_;
    DiagnosticSink* sink_;
    std::set<std::string>* warned_;
};

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
                                   const NamedValueStore& variables, DiagnosticSink& sink) {
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
                      const NamedValueStore& variables, DiagnosticSink& sink) {
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
                                          parameters, variables, sink);
        }
    }
}

/// Validates action content beyond target existence. A SpeedAction's transition
/// dynamics value carries a time [s], distance [m] or rate [delta/s] and must be
/// finite and in range [0..inf[ (per ASAM OpenSCENARIO XML 1.4.0
/// §TransitionDynamics); the standard defines no asam.net rule id, so the
/// diagnostic cites the section only.
void validate_action_content(const ir::Action& action, const std::string& path,
                             DiagnosticSink& sink) {
    if (const auto* speed = dynamic_cast<const ir::SpeedAction*>(&action)) {
        const ir::TransitionDynamics& td = speed->dynamics();
        if (!std::isfinite(td.value) || td.value < 0.0) {
            error(sink, Status::ValidationError,
                  "speed action transition dynamics value must be finite and in range [0..inf[",
                  path);
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
    }
}

/// Validates the storyboard tree in document order: element naming,
/// triggers, non-null actions, and action targets that exist in `records`.
/// Every defect is appended to `sink`; the walk never stops early.
template <typename Records>
void validate_storyboard(const ir::Storyboard& storyboard, const Records& records,
                         const NamedValueStore& parameters, const NamedValueStore& variables,
                         DiagnosticSink& sink) {
    check_sibling_names(
        storyboard.stories, [](const ir::Story& s) { return s.name; }, "story", "", sink);
    validate_trigger(storyboard.stop_trigger, "", "stopTrigger", storyboard, records, parameters,
                     variables, sink);
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
                             parameters, variables, sink);
            validate_trigger(act.stop_trigger, act_path, "stopTrigger", storyboard, records,
                             parameters, variables, sink);
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
                                         storyboard, records, parameters, variables, sink);
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
                            validate_action_content(*action, action_path, sink);
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
        if (records.find(action->entity_id()) == records.end()) {
            error(diagnostics_, Status::SemanticError,
                  "action targets unknown entity '" + action->entity_id() + "'", action_path);
        }
        validate_action_content(*action, action_path, diagnostics_);
    }
    validate_storyboard(scenario.storyboard, records, scenario.parameters, scenario.variables,
                        diagnostics_);

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
    warned_values_.clear();
    clock_.reset();
    initialized_ = true;

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
    RuntimeContext context{
        0.0,       scenario_.parameters,           variables_,   user_defined_values_,
        entities_, current_date_time_seconds(0.0), diagnostics_, warned_values_};
    scheduler_.step(context, [this](const ir::Action& action) { return apply(action); });

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

    RuntimeContext context{clock_.now(), scenario_.parameters,
                           variables_,   user_defined_values_,
                           entities_,    current_date_time_seconds(clock_.now()),
                           diagnostics_, warned_values_};
    scheduler_.step(context, [this](const ir::Action& action) { return apply(action); });

    for (auto& [id, record] : entities_) {
        (void)id;
        if (record.mode == ir::ControlMode::EngineControlled) {
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
            if (record.mode == ir::ControlMode::EngineControlled) {
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
    // pre-staged time of day is visible at the t = 0 evaluation.
    date_time_anchor_epoch_ = date_time.to_epoch_seconds();
    date_time_anchor_sim_ = clock_.now();
    date_time_set_ = true;
    return Status::Ok;
}

std::optional<double> Engine::date_time() const {
    return current_date_time_seconds(clock_.now());
}

std::optional<double> Engine::current_date_time_seconds(double simulation_time) const {
    if (!date_time_set_) {
        return std::nullopt;
    }
    // Fixed IEEE expression, no wall clock: the simulated instant is the
    // anchor plus the simulation time elapsed since the anchor was set.
    return date_time_anchor_epoch_ + (simulation_time - date_time_anchor_sim_);
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
    // User-defined values and the time-of-day anchor persist across init()
    // but not across close(): a closed engine forgets everything the host
    // staged.
    user_defined_values_.clear();
    date_time_set_ = false;
    warned_values_.clear();
    scenario_ = ir::Scenario{};
    initialized_ = false;
    return Status::Ok;
}

runtime::ActionOutcome Engine::apply(const ir::Action& action) {
    const auto* speed_action = dynamic_cast<const ir::SpeedAction*>(&action);
    const auto* profile_action = dynamic_cast<const ir::SpeedProfileAction*>(&action);
    if (speed_action != nullptr || profile_action != nullptr) {
        // Longitudinal actions share the default controller.
        const auto it = entities_.find(action.entity_id());
        if (it == entities_.end()) {
            // init() validates every action target, so this is defensive: a
            // Warning, not a failure. The run continues; the action is skipped.
            Diagnostic diagnostic;
            diagnostic.severity = Severity::Warning;
            diagnostic.code = Status::UnknownEntity;
            diagnostic.message =
                "action targets unknown entity '" + action.entity_id() + "'; action skipped";
            diagnostic.path = "entities/" + action.entity_id();
            diagnostics_.report(std::move(diagnostic));
            return runtime::ActionOutcome::Complete;
        }
        EntityRecord& record = it->second;
        if (record.active_longitudinal_action == &action) {
            // Owning action's re-poll: the controller argument is unused.
            return drive_longitudinal(action, record, runtime::LongitudinalController{});
        }
        return drive_longitudinal(action, record,
                                  speed_action != nullptr
                                      ? build_speed_controller(*speed_action, record)
                                      : build_profile_controller(*profile_action, record));
    }

    // An action kind the engine does not implement yet. A parser must never
    // silently drop input, and neither does the runtime: emit a Warning and
    // keep going. Scheduling is unchanged — the event completes in one
    // evaluation.
    Diagnostic diagnostic;
    diagnostic.severity = Severity::Warning;
    diagnostic.code = Status::UnsupportedFeature;
    diagnostic.message = "unsupported action kind '" + std::string(action.kind()) +
                         "' targeting entity '" + action.entity_id() + "'; action ignored";
    diagnostic.path = "entities/" + action.entity_id();
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

runtime::LongitudinalController Engine::build_speed_controller(const ir::SpeedAction& action,
                                                               const EntityRecord& record) const {
    const ir::TransitionDynamics& td = action.dynamics();
    const std::optional<ir::Performance>& perf = record.performance;

    runtime::LongitudinalController::Segment seg;
    seg.from = record.state.speed;
    seg.shape = td.shape;

    // Clamp the target to the entity's maximum speed (§Performance). Only a
    // positive maxSpeed constrains; other targets pass through.
    double target = action.target_speed();
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
    record.longitudinal.reset();
    record.active_longitudinal_action = nullptr;
}

} // namespace scena
