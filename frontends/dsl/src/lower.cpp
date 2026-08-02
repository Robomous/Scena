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

//
// Lowering a checked DSL program to the Scenario IR (p8-s1, ADR-0030).
//
// The DSL and the XML frontend compile into the same IR, so nothing here
// decides runtime semantics: it decides which DSL construct denotes which IR
// construct, and reports whatever it cannot make concrete.
//

#include "scena/dsl/lower.h"

#include <algorithm>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "scena/dsl/ast.h"
#include "scena/dsl/expression.h"
#include "scena/ir/action.h"
#include "scena/ir/bounding_box.h"
#include "scena/ir/condition.h"
#include "scena/ir/dynamics.h"
#include "scena/ir/entity.h"
#include "scena/ir/entity_types.h"
#include "scena/ir/evaluation_context.h"
#include "scena/ir/position.h"
#include "scena/ir/rule.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/trigger.h"

namespace scena::dsl {
namespace {

/// The standard-library types the mapping keys off. Looked up once, by
/// qualified name, so a scenario that never imports the library simply finds
/// none of them and lowers no entities.
struct LibraryTypes {
    TypeId physical_object = kInvalidType;
    TypeId vehicle = kInvalidType;
    TypeId person = kInvalidType;
    TypeId stationary_object = kInvalidType;
    TypeId map = kInvalidType;
};

[[nodiscard]] TypeId find_type(const Program& program, const std::string& name) {
    const auto found = program.types_by_name.find(name);
    return found == program.types_by_name.end() ? kInvalidType : found->second;
}

[[nodiscard]] LibraryTypes library_types(const Program& program) {
    LibraryTypes types;
    types.physical_object = find_type(program, "std::physical_object");
    types.vehicle = find_type(program, "std::vehicle");
    types.person = find_type(program, "std::person");
    types.stationary_object = find_type(program, "std::stationary_object");
    types.map = find_type(program, "std::map");
    return types;
}

void report(DiagnosticSink& sink, Severity severity, Status code, const std::string& file,
            const SourceRange& at, std::string message) {
    Diagnostic diagnostic;
    diagnostic.severity = severity;
    diagnostic.code = code;
    diagnostic.message = std::move(message);
    diagnostic.location.file = file;
    diagnostic.location.line = at.line;
    diagnostic.location.column = at.column;
    // The DSL standard defines no `asam.net:` rule identifiers, so the citation
    // stays in the message and rule_id stays empty.
    sink.report(std::move(diagnostic));
}

/// One `keep(<field> == <constant>)` the entry scenario fixes.
///
/// §7.3.11 lets a constraint say far more than this. Everything else needs a
/// solver, which is post-v0.0.1 (ADR-0004), so lowering reads exactly the shape
/// that resolves to a value without one.
struct Binding {
    /// The field path as written, e.g. `ego.bounding_box.length`.
    std::vector<std::string> path;
    Value value;
    SourceRange range;
};

/// Reads `a.b.c` out of an expression, or returns false for anything else.
bool field_path(const Expr& expression, std::vector<std::string>& out) {
    if (expression.kind == ExprKind::Name) {
        out.push_back(expression.text);
        return true;
    }
    if (expression.kind == ExprKind::FieldAccess && expression.operands.size() == 1 &&
        expression.operands.front() != nullptr) {
        if (!field_path(*expression.operands.front(), out)) {
            return false;
        }
        out.push_back(expression.text);
        return true;
    }
    return false;
}

/// Collects the equality bindings of a scenario, in declaration order.
///
/// Both operand orders are read: `keep(x == 3)` and `keep(3 == x)` say the same
/// thing, and the standard prints both.
void collect_bindings(const Program& program, const StructuredDecl& decl,
                      const ExpressionContext& context, std::vector<Binding>& out) {
    for (const Member& member : decl.members) {
        if (member.kind != Member::Kind::Constraint || member.constraint.expression == nullptr) {
            continue;
        }
        const Expr& expression = *member.constraint.expression;
        if (expression.kind != ExprKind::Binary || expression.text != "==" ||
            expression.operands.size() != 2) {
            continue;
        }
        for (int side = 0; side < 2; ++side) {
            const ExprPtr& name = expression.operands[static_cast<std::size_t>(side)];
            const ExprPtr& value = expression.operands[static_cast<std::size_t>(1 - side)];
            if (name == nullptr || value == nullptr) {
                continue;
            }
            Binding binding;
            if (!field_path(*name, binding.path)) {
                continue;
            }
            if (!evaluate_constant(program, *value, context, binding.value)) {
                continue; // needs search; the caller reports the remnant
            }
            binding.range = member.constraint.range;
            out.push_back(std::move(binding));
            break;
        }
    }
}

/// The binding for `<entity>.<a>.<b>...`, or nullptr.
const Value* binding_for(const std::vector<Binding>& bindings, const std::string& entity,
                         std::initializer_list<const char*> tail) {
    for (const Binding& binding : bindings) {
        if (binding.path.size() != tail.size() + 1 || binding.path.front() != entity) {
            continue;
        }
        std::size_t index = 1;
        bool matched = true;
        for (const char* step : tail) {
            if (binding.path[index++] != step) {
                matched = false;
                break;
            }
        }
        if (matched) {
            return &binding.value;
        }
    }
    return nullptr;
}

/// Writes a bound number into `out`, if there is one. Physical quantities have
/// already been folded to their base unit (§7.3.4), which is the unit the IR
/// stores — so lowering never converts, and never re-applies the standard's
/// printed factors (ADR-0029).
void bind_number(const std::vector<Binding>& bindings, const std::string& entity,
                 std::initializer_list<const char*> tail, double& out) {
    const Value* value = binding_for(bindings, entity, tail);
    if (value != nullptr && value->is_numeric()) {
        out = value->as_double();
    }
}

/// The enum member a binding names, or empty.
std::string bound_enum(const Program& program, const std::vector<Binding>& bindings,
                       const std::string& entity, std::initializer_list<const char*> tail) {
    const Value* value = binding_for(bindings, entity, tail);
    if (value == nullptr || value->kind != Value::Kind::Enum || value->type == kInvalidType) {
        return {};
    }
    for (const EnumMemberInfo& member : program.types[value->type].enum_members) {
        if (member.value == value->enum_value) {
            return member.name;
        }
    }
    return {};
}

ir::BoundingBox lower_bounding_box(const std::vector<Binding>& bindings, const std::string& id) {
    ir::BoundingBox box;
    bind_number(bindings, id, {"bounding_box", "length"}, box.length);
    bind_number(bindings, id, {"bounding_box", "width"}, box.width);
    bind_number(bindings, id, {"bounding_box", "height"}, box.height);
    bind_number(bindings, id, {"bounding_box", "center_x"}, box.center_x);
    bind_number(bindings, id, {"bounding_box", "center_y"}, box.center_y);
    bind_number(bindings, id, {"bounding_box", "center_z"}, box.center_z);
    return box;
}

/// §8.7.16's `vehicle_category` mapped onto the XML-side taxonomy (p2-s1).
///
/// The two standards enumerate the same twenty things under slightly different
/// spellings, so this is a spelling bridge and nothing more — no category is
/// invented and none is dropped. `truck` and `vru_vehicle` are §8.7.16's
/// backward-compatibility aliases, equal in value to `heavy_truck` and
/// `micro_mobility_device`, so they never appear here: a lookup by value finds
/// the replacement first. XML's `Truck` and `Motorbike` are the mirror case —
/// deprecated 1.3 spellings with no DSL counterpart at all.
///
/// A category nothing fixes is left at the IR's own default rather than guessed
/// at, which is why this returns an optional.
std::optional<ir::VehicleCategory> vehicle_category_of(const std::string& name) {
    static const std::map<std::string, ir::VehicleCategory> kCategories{
        {"aircraft", ir::VehicleCategory::Aircraft},
        {"bicycle", ir::VehicleCategory::Bicycle},
        {"bus", ir::VehicleCategory::Bus},
        {"car", ir::VehicleCategory::Car},
        {"heavy_truck", ir::VehicleCategory::HeavyTruck},
        {"land_vehicle", ir::VehicleCategory::LandVehicle},
        {"micro_mobility_device", ir::VehicleCategory::MicromobilityDevice},
        {"motorcycle", ir::VehicleCategory::Motorcycle},
        {"other", ir::VehicleCategory::Other},
        {"semi_tractor", ir::VehicleCategory::Semitractor},
        {"semi_trailer", ir::VehicleCategory::Semitrailer},
        {"stand_up_scooter", ir::VehicleCategory::StandupScooter},
        {"trailer", ir::VehicleCategory::Trailer},
        {"train", ir::VehicleCategory::Train},
        {"tram", ir::VehicleCategory::Tram},
        {"van", ir::VehicleCategory::Van},
        {"watercraft", ir::VehicleCategory::Watercraft},
        {"wheelchair", ir::VehicleCategory::Wheelchair},
        {"work_machine", ir::VehicleCategory::WorkMachine},
    };
    const auto found = kCategories.find(name);
    if (found == kCategories.end()) {
        return std::nullopt;
    }
    return found->second;
}

// --- §8.8 movement actions -------------------------------------------------

/// One argument of a behavior invocation, taken as far as lowering can take it:
/// a folded constant, or the path of the field it names.
///
/// The two cases are different in kind, not in degree. `target: 20mps` is a
/// value; `reference: lead` and `position: start_point` name a *declaration*,
/// and what makes them concrete is the equality constraints the scenario put on
/// it — the same binding table §7.3.11 already gives the entities (ADR-0030).
struct ArgumentValue {
    bool folded = false;
    Value value;
    std::vector<std::string> path;
    SourceRange range;
};

using ArgumentMap = std::map<std::string, ArgumentValue>;

/// Binds an invocation's arguments to the invoked behavior's parameter names.
///
/// Positional arguments bind in declaration order and named ones by name
/// (§7.2.2.5.2). A name the behavior does not declare is left out: the resolver
/// has already reported it, and lowering does not repeat a diagnostic.
ArgumentMap bind_arguments(const Program& program, TypeId behavior,
                           const std::vector<Argument>& arguments,
                           const ExpressionContext& context) {
    ArgumentMap bound;
    const std::vector<std::string>& order = program.types[behavior].field_order;
    std::size_t positional = 0;
    for (const Argument& argument : arguments) {
        if (argument.value == nullptr) {
            continue;
        }
        std::string name = argument.name;
        if (name.empty()) {
            if (positional >= order.size()) {
                continue;
            }
            name = order[positional++];
        }
        ArgumentValue value;
        value.range = argument.range;
        value.folded = evaluate_constant(program, *argument.value, context, value.value);
        if (!value.folded) {
            (void)field_path(*argument.value, value.path);
        }
        bound.insert_or_assign(std::move(name), std::move(value));
    }
    return bound;
}

/// The folded number an argument carries, if it carries one.
std::optional<double> argument_number(const ArgumentMap& arguments, const char* name) {
    const auto found = arguments.find(name);
    if (found == arguments.end() || !found->second.folded || !found->second.value.is_numeric()) {
        return std::nullopt;
    }
    return found->second.value.as_double();
}

/// The enum member an argument names, or empty.
std::string argument_enum(const Program& program, const ArgumentMap& arguments, const char* name) {
    const auto found = arguments.find(name);
    if (found == arguments.end() || !found->second.folded ||
        found->second.value.kind != Value::Kind::Enum || found->second.value.type == kInvalidType) {
        return {};
    }
    for (const EnumMemberInfo& member : program.types[found->second.value.type].enum_members) {
        if (member.value == found->second.value.enum_value) {
            return member.name;
        }
    }
    return {};
}

/// The single field name an argument refers to, or empty when it is not a plain
/// reference to a declaration of the scenario.
std::string argument_reference(const ArgumentMap& arguments, const char* name) {
    const auto found = arguments.find(name);
    if (found == arguments.end() || found->second.folded || found->second.path.size() != 1) {
        return {};
    }
    return found->second.path.front();
}

/// The behavior an invocation names, found the way §7.3.12.2 scopes it: through
/// the receiver's inheritance chain, one exact lookup per supertype.
TypeId lookup_behavior_on(const Program& program, TypeId receiver, const std::string& name) {
    for (TypeId current = receiver; current != kInvalidType;
         current = program.types[current].base) {
        const auto found = program.types_by_name.find(program.types[current].name + "." + name);
        if (found != program.types_by_name.end() &&
            program.types[found->second].kind == TypeKind::Action) {
            return found->second;
        }
    }
    return kInvalidType;
}

/// The transition a §8.8 `rate_profile`/`rate_peak` pair asks for.
///
/// §8.8.2.18's `dynamic_profile` names the *shape* of the change and
/// `rate_peak` its magnitude. Neither carries a duration, so with no peak rate
/// there is no number to ramp over and the change is instantaneous — which is
/// what a Step shape means (§7.4.1.2). `asap` is a Step from the other
/// direction: §8.7 declares no performance envelope at all (ADR-0030), so "as
/// soon as possible" is bounded by nothing.
ir::TransitionDynamics rate_dynamics(const std::string& profile,
                                     const std::optional<double>& peak) {
    ir::TransitionDynamics dynamics;
    if (profile == "asap" || !peak.has_value() || *peak <= 0.0) {
        dynamics.shape = ir::DynamicsShape::Step;
        dynamics.dimension = ir::DynamicsDimension::Time;
        dynamics.value = 0.0;
        return dynamics;
    }
    // `smooth` is the profile whose gradient vanishes at both ends, which is
    // the Cubic shape (§DynamicsShape); `constant` and an unstated profile are
    // both a constant rate, which is Linear.
    dynamics.shape = profile == "smooth" ? ir::DynamicsShape::Cubic : ir::DynamicsShape::Linear;
    dynamics.dimension = ir::DynamicsDimension::Rate;
    dynamics.value = *peak;
    return dynamics;
}

/// The `use` list the file gives `name_space` (§7.7.4).
std::vector<std::string> uses_of(const File& file, const std::string& name_space) {
    for (const Declaration& declaration : file.declarations) {
        if (declaration.kind == Declaration::Kind::Namespace &&
            declaration.name_space.name == name_space) {
            return declaration.name_space.uses;
        }
    }
    return {};
}

// --- the `do` directive ----------------------------------------------------

/// Everything the behavior half of lowering reads.
struct Behaviors {
    const Program& program;
    const LibraryTypes& library;
    const std::string& file;
    const ExpressionContext& context;
    const std::vector<Binding>& bindings;
    std::set<std::string> entities;
    /// The `one_of` alternative to run, by label; empty means the first.
    std::string alternative;
    /// Phase names already used, so siblings stay unique: the runtime
    /// addresses storyboard elements by name path.
    std::set<std::string> taken;
    DiagnosticSink& sink;

    void warn(const SourceRange& at, std::string message) {
        report(sink, Severity::Warning, Status::UnsupportedFeature, file, at, std::move(message));
    }
    void error(const SourceRange& at, std::string message) {
        report(sink, Severity::Error, Status::SemanticError, file, at, std::move(message));
    }

    /// The world position a `position_3d`-typed declaration is constrained to.
    ///
    /// The DSL has no struct constructor — §7.2.2.6.7 declares list and range
    /// constructors and nothing else — so a struct-valued argument can only
    /// name a declaration, and the values come from the `keep`s on it. That is
    /// the same route ADR-0030 already takes for an actor's own attributes.
    ir::WorldPosition world_position(const std::string& field, const SourceRange& at) {
        ir::WorldPosition position;
        bind_number(bindings, field, {"x"}, position.x);
        bind_number(bindings, field, {"y"}, position.y);
        bind_number(bindings, field, {"z"}, position.z);
        if (binding_for(bindings, field, {"x"}) == nullptr &&
            binding_for(bindings, field, {"y"}) == nullptr &&
            binding_for(bindings, field, {"z"}) == nullptr) {
            warn(at, "no constraint fixes any coordinate of '" + field +
                         "', so it lowers as the world origin (§7.3.11)");
        }
        return position;
    }

    /// The entity an argument refers to, reported when it names anything else.
    std::string entity_argument(const ArgumentMap& arguments, const char* name,
                                const SourceRange& at, const std::string& action) {
        const std::string reference = argument_reference(arguments, name);
        if (reference.empty() || entities.count(reference) == 0) {
            error(at, "'" + action + "' needs '" + name +
                          "' to name a §8.7 participant of this scenario (§8.8.3)");
            return {};
        }
        return reference;
    }

    /// One §8.8 invocation as the IR actions it denotes, or none.
    std::vector<std::shared_ptr<ir::Action>>
    actions_of(const DoMember& member, const std::string& actor, TypeId actor_type) {
        std::vector<std::shared_ptr<ir::Action>> actions;
        const TypeId behavior = lookup_behavior_on(program, actor_type, member.name);
        if (behavior == kInvalidType) {
            // A scenario invocation, a user-declared behavior, or something the
            // resolver already complained about. Composition of scenarios is
            // p8-s2 (#45), so this stays a note rather than an error.
            warn(member.range, "'" + member.name +
                                   "' is not a §8.8 movement action, so it contributes no IR "
                                   "action (scenario invocation is p8-s2, #45)");
            return actions;
        }
        const std::string name = program.types[behavior].simple_name;
        const ArgumentMap arguments = bind_arguments(program, behavior, member.arguments, context);

        if (name == "movable_object.assign_speed") {
            // §8.8.2.6: the actor's speed *is* the value from this point on, so
            // it is a Step transition (§7.4.1.2) — the same reading the XML
            // frontend gives an init SpeedAction.
            const std::optional<double> speed = argument_number(arguments, "speed");
            if (!speed.has_value()) {
                error(member.range, "'assign_speed' needs a concrete 'speed' (§8.8.2.6)");
                return actions;
            }
            actions.push_back(std::make_shared<ir::SpeedAction>(actor, *speed));
            return actions;
        }
        if (name == "movable_object.change_speed") {
            const std::optional<double> target = argument_number(arguments, "target");
            if (!target.has_value()) {
                error(member.range, "'change_speed' needs a concrete 'target' (§8.8.2.12)");
                return actions;
            }
            actions.push_back(std::make_shared<ir::SpeedAction>(
                actor, *target,
                rate_dynamics(argument_enum(program, arguments, "rate_profile"),
                              argument_number(arguments, "rate_peak"))));
            return actions;
        }
        if (name == "movable_object.remain_stationary") {
            // §8.8.2.10 is "the actor does not move": speed zero, immediately.
            actions.push_back(std::make_shared<ir::SpeedAction>(actor, 0.0));
            return actions;
        }
        if (name == "movable_object.assign_position") {
            if (arguments.count("route_point") != 0 || arguments.count("odr_point") != 0) {
                warn(member.range, "'assign_position' by route or OpenDRIVE point is not lowered "
                                   "yet (§8.8.2.4); use 'position'");
                return actions;
            }
            const std::string field = argument_reference(arguments, "position");
            if (field.empty()) {
                error(member.range, "'assign_position' needs 'position' to name a 'position_3d' "
                                    "declaration (§8.8.2.4)");
                return actions;
            }
            actions.push_back(std::make_shared<ir::TeleportAction>(
                actor, ir::Position{world_position(field, member.range)}));
            return actions;
        }
        if (name == "vehicle.change_lane") {
            const std::string side = argument_enum(program, arguments, "side");
            if (side != "left" && side != "right") {
                // §8.8.3.14's `inside`/`outside`/`same` need the road geometry
                // to say which way that is, and an unstated side would have to
                // be chosen — a choice the determinism contract does not allow
                // lowering to make.
                error(member.range,
                      "'change_lane' needs an explicit 'side' of left or right (§8.8.3.3); '" +
                          (side.empty() ? std::string("none given") : side) + "' is not lowered");
                return actions;
            }
            const std::optional<double> count = argument_number(arguments, "num_of_lanes");
            ir::RelativeTargetLane target;
            // The reference defaults to the actor itself (§8.8.3.3's
            // `Default=it.actor`), and positive counts go left (§7.4.1.4).
            target.entity_ref = argument_reference(arguments, "reference");
            if (target.entity_ref.empty() || entities.count(target.entity_ref) == 0) {
                target.entity_ref = actor;
            }
            const int lanes = count.has_value() ? static_cast<int>(*count) : 1;
            target.value = side == "left" ? lanes : -lanes;
            actions.push_back(std::make_shared<ir::LaneChangeAction>(
                actor, target,
                rate_dynamics(argument_enum(program, arguments, "rate_profile"),
                              argument_number(arguments, "rate_peak"))));
            return actions;
        }
        if (name == "vehicle.change_space_gap" || name == "vehicle.change_time_gap") {
            const bool space = name == "vehicle.change_space_gap";
            const std::optional<double> target = argument_number(arguments, "target");
            if (!target.has_value()) {
                error(member.range, "'" + member.name + "' needs a concrete 'target' (§8.8.3." +
                                        (space ? "6" : "4") + ")");
                return actions;
            }
            const std::string reference =
                entity_argument(arguments, "reference", member.range, member.name);
            if (reference.empty()) {
                return actions;
            }
            // §8.8.3.15's `ahead`/`behind` say which side of the reference the
            // actor ends up on, which is exactly §LongitudinalDisplacement; the
            // four lateral members have no longitudinal reading.
            const std::string direction = argument_enum(program, arguments, "direction");
            ir::LongitudinalDisplacement displacement =
                ir::LongitudinalDisplacement::TrailingReferencedEntity;
            if (direction == "ahead") {
                displacement = ir::LongitudinalDisplacement::LeadingReferencedEntity;
            } else if (!direction.empty() && direction != "behind") {
                warn(member.range, "gap direction '" + direction +
                                       "' is lateral and is not lowered (§8.8.3.15); the "
                                       "longitudinal gap is kept behind the reference");
            }
            // A "gap" in §8.8.3 is the clear space between the two objects, so
            // it is measured between bounding boxes, not reference points
            // (§6.4.7). `change_*` reaches the gap and ends; `keep_*` is the
            // continuous form and is not lowered yet.
            actions.push_back(std::make_shared<ir::LongitudinalDistanceAction>(
                actor, reference, space ? target : std::nullopt, space ? std::nullopt : target,
                /*freespace=*/true, /*continuous=*/false, ir::CoordinateSystem::Entity,
                displacement));
            return actions;
        }
        if (name == "movable_object.move" || name == "vehicle.drive" || name == "person.walk") {
            // §8.8.2.3/§8.8.3.1/§8.8.4.1 are the generic actions: they carry no
            // target of their own and exist to be shaped by §8.9 modifiers,
            // which are p8-s3 (#46). On their own they say "keep doing what you
            // are doing", which the runtime already does, so they lower to
            // nothing and say so only when they carry modifiers that would
            // have changed that.
            return actions;
        }
        warn(member.range, "'" + name +
                               "' has no runtime counterpart in v0.0.1 (§8.8); see the "
                               "DSL coverage matrix");
        return actions;
    }

    /// The actor a `do` member is invoked on, and its type.
    bool actor_of(const DoMember& member, std::string& actor, TypeId& actor_type) {
        std::vector<std::string> path;
        if (member.actor == nullptr || !field_path(*member.actor, path) || path.size() != 1) {
            error(member.range,
                  "a movement action is invoked on one participant of this scenario (§7.2.2.4.7)");
            return false;
        }
        actor = path.front();
        if (entities.count(actor) == 0) {
            error(member.range, "'" + actor + "' is not a §8.7 participant of this scenario");
            return false;
        }
        const auto field = program.types[context.self].fields.find(actor);
        if (field == program.types[context.self].fields.end()) {
            return false;
        }
        actor_type = field->second.type;
        return true;
    }

    /// Reports the parts of an invocation that p8-s2 and p8-s3 will lower.
    void report_deferred(const DoMember& member) {
        for (const ModifierApplication& modifier : member.with.modifiers) {
            warn(modifier.range,
                 "movement modifier '" + modifier.name + "' is lowered in p8-s3 (#46), §8.9");
        }
        for (const EventSpec& until : member.with.until) {
            (void)until;
            // §7.6.2.5.4 ends the invocation *exactly* at an event, and the
            // events it names have no runtime carrier in v0.0.1.
            warn(member.range, "'until' ends the invocation at an event, which has no runtime "
                               "carrier in v0.0.1 (§7.6.2.5.4)");
        }
    }

    /// A name for the phase that is unique among its siblings, because the
    /// runtime addresses storyboard elements by name path.
    std::string phase_name(const DoMember& member, std::size_t index) {
        std::string name =
            member.label.empty() ? "phase_" + std::to_string(index + 1) : member.label;
        std::string candidate = name;
        for (std::size_t suffix = 2; taken.count(candidate) != 0; ++suffix) {
            candidate = name + "_" + std::to_string(suffix);
        }
        taken.insert(candidate);
        return candidate;
    }

    /// The `do` directive as a storyboard.
    ///
    /// One Story, one Act, and one ManeuverGroup per phase — the group is where
    /// the actor lives, so a phase per group is what keeps each invocation's
    /// actor with its actions. `serial` chains a phase's Event on the previous
    /// group reaching completeState, which is the trigger form §7.6.2.1.2's
    /// "starts when its predecessor ends" already has in the runtime;
    /// `parallel` leaves the triggers absent, so every phase starts with the
    /// Act (§7.6.1.1). Everything else about composition is p8-s2 (#45).
    /// Where a phase begins or ends, as far as *load time* can know it.
    ///
    /// A concrete `duration` makes the time arithmetic: the storyboard starts
    /// at t = 0 and every duration that lowers is a constant, so an absolute
    /// time is exact and needs no runtime feedback. When a phase ends by its
    /// actions finishing instead, the only thing load time can say is "when
    /// that group completes". A trigger ANDs whatever of the two is known,
    /// which is what makes a parallel join expressible: a ConditionGroup is an
    /// AND, so "all members done" is one group with one condition per member.
    struct TimePoint {
        std::optional<double> at;       ///< absolute simulation time [s]
        std::vector<std::string> after; ///< groups that must all be complete

        [[nodiscard]] bool unset() const { return !at.has_value() && after.empty(); }
    };

    /// The trigger a TimePoint asks for, or none when it is "with the parent".
    [[nodiscard]] std::optional<ir::Trigger> trigger_for(const TimePoint& point) const {
        if (point.unset() || (point.after.empty() && *point.at <= 0.0)) {
            // §7.6.1.1: an element with no start trigger starts with its
            // parent, which is what "at t = 0" means — and `t >= 0` as a
            // condition would be a tautology in the IR rather than a fact.
            return std::nullopt;
        }
        ir::ConditionGroup group;
        if (point.at.has_value()) {
            ir::TriggerCondition condition;
            condition.expression =
                std::make_shared<ir::SimulationTimeCondition>(*point.at, ir::Rule::GreaterOrEqual);
            group.conditions.push_back(std::move(condition));
        }
        for (const std::string& element : point.after) {
            ir::TriggerCondition condition;
            condition.expression = std::make_shared<ir::StoryboardElementStateCondition>(
                ir::StoryboardElementType::ManeuverGroup, element,
                ir::StoryboardElementState::CompleteState);
            group.conditions.push_back(std::move(condition));
        }
        ir::Trigger trigger;
        trigger.groups.push_back(std::move(group));
        return trigger;
    }

    /// Both points reached: the later time, and every completion either wanted.
    [[nodiscard]] static TimePoint both(TimePoint left, const TimePoint& right) {
        if (right.at.has_value()) {
            left.at = left.at.has_value() ? std::max(*left.at, *right.at) : right.at;
        }
        for (const std::string& element : right.after) {
            left.after.push_back(element);
        }
        return left;
    }

    /// A concrete `duration` argument, or none.
    ///
    /// §7.6.2.4 lets a duration be a range, which is a constraint on accepted
    /// traces rather than a value — choosing one needs a solver, which is
    /// post-v0.0.1 (ADR-0004). A constant is a value, and lowers.
    [[nodiscard]] std::optional<double> duration_of(const std::vector<Argument>& arguments,
                                                    const std::string& what) {
        for (const Argument& argument : arguments) {
            if (argument.name != "duration" || argument.value == nullptr) {
                continue;
            }
            Value value;
            if (evaluate_constant(program, *argument.value, context, value) && value.is_numeric()) {
                return value.as_double();
            }
            warn(argument.range, "the duration of " + what +
                                     " is not a single value, so it bounds accepted traces rather "
                                     "than fixing a time (§7.6.2.4); it is not lowered");
            return std::nullopt;
        }
        return std::nullopt;
    }

    /// The `elapsed(<duration>)` of a `wait` directive, or none.
    [[nodiscard]] std::optional<double> elapsed_of(const EventSpec& event) {
        if (event.kind != EventConditionKind::Elapsed || event.expression == nullptr) {
            return std::nullopt;
        }
        Value value;
        if (evaluate_constant(program, *event.expression, context, value) && value.is_numeric()) {
            return value.as_double();
        }
        return std::nullopt;
    }

    /// Reports the composition arguments §7.6.2.1.4 defines but v0.0.1 does not
    /// realise. `overlap: start` is the default and is what absent triggers
    /// already give, so only a *different* overlap is worth a word.
    void report_composition_arguments(const DoMember& composition) {
        for (const Argument& argument : composition.composition_arguments) {
            if (argument.name == "duration") {
                continue; // read by duration_of
            }
            if (argument.name == "overlap" && argument.value != nullptr) {
                Value value;
                if (evaluate_constant(program, *argument.value, context, value) &&
                    value.kind == Value::Kind::Enum) {
                    // The default is `start`, which is exactly "every member
                    // starts with its parent" — nothing to do.
                    const std::string named = argument_enum_name(value);
                    if (named.empty() || named == "start") {
                        continue;
                    }
                    warn(argument.range, "parallel overlap '" + named +
                                             "' is not realised in v0.0.1; members start together "
                                             "and end when their actions do (§7.6.2.1.4)");
                    continue;
                }
            }
            warn(argument.range,
                 "composition argument '" +
                     (argument.name.empty() ? std::string("(positional)") : argument.name) +
                     "' is not realised in v0.0.1 (§7.6.2.1.4)");
        }
    }

    [[nodiscard]] std::string argument_enum_name(const Value& value) const {
        if (value.kind != Value::Kind::Enum || value.type == kInvalidType) {
            return {};
        }
        for (const EnumMemberInfo& member : program.types[value.type].enum_members) {
            if (member.value == value.enum_value) {
                return member.name;
            }
        }
        return {};
    }

    /// The alternative a `one_of` runs.
    ///
    /// §7.6.2.1.3 says at least one alternative must hold, and says nothing
    /// about which — so an executor picks. Picking at random would put a hidden
    /// input in the run, so the choice is an *input*: `LowerOptions::alternative`
    /// (which `scena-run --select` feeds), defaulting to the first alternative
    /// in declaration order.
    [[nodiscard]] const DoMember* chosen_alternative(const DoMember& composition) {
        const DoMember* first = nullptr;
        for (const DoMemberPtr& member : composition.members) {
            if (member == nullptr) {
                continue;
            }
            if (first == nullptr) {
                first = member.get();
            }
            if (!alternative.empty() && member->label == alternative) {
                return member.get();
            }
        }
        if (!alternative.empty()) {
            std::string message = "'" + alternative +
                                  "' is not an alternative of this one_of (§7.6.2.1.3); it offers:";
            for (const DoMemberPtr& member : composition.members) {
                if (member != nullptr) {
                    message +=
                        " " + (member->label.empty() ? std::string("(unlabelled)") : member->label);
                }
            }
            error(composition.range, std::move(message));
            return nullptr;
        }
        return first;
    }

    /// Lowers one `do` member starting at `start`, and answers where it ends.
    TimePoint lower_member(const DoMember& member, const TimePoint& start, ir::Act& act) {
        switch (member.kind) {
        case DoMemberKind::Composition:
            return lower_composition(member, start, act);
        case DoMemberKind::Wait:
            return lower_wait(member, start);
        case DoMemberKind::Invocation:
            return lower_invocation(member, start, act);
        case DoMemberKind::Emit:
        case DoMemberKind::Call:
            break;
        }
        // §7.6.2.5.2's emit and §7.3.7's call are zero-time directives whose
        // meaning is an event or a method call, neither of which the runtime
        // has a carrier for yet.
        warn(member.range, "'" + std::string(member.kind == DoMemberKind::Emit ? "emit" : "call") +
                               "' is a zero-time directive with no runtime carrier in v0.0.1 "
                               "(§7.6.2.5)");
        return start;
    }

    TimePoint lower_composition(const DoMember& composition, const TimePoint& start, ir::Act& act) {
        report_composition_arguments(composition);
        const std::optional<double> bound =
            duration_of(composition.composition_arguments, "this composition");

        TimePoint end = start;
        if (composition.composition == CompositionOperator::Serial) {
            for (const DoMemberPtr& member : composition.members) {
                if (member != nullptr) {
                    end = lower_member(*member, end, act);
                }
            }
        } else if (composition.composition == CompositionOperator::Parallel) {
            // §7.6.2.1.4's default overlap is `start`: every member begins with
            // the composition. It ends when all of them have.
            TimePoint joined;
            bool any = false;
            for (const DoMemberPtr& member : composition.members) {
                if (member == nullptr) {
                    continue;
                }
                const TimePoint member_end = lower_member(*member, start, act);
                joined = any ? both(std::move(joined), member_end) : member_end;
                any = true;
            }
            end = any ? joined : start;
        } else {
            const DoMember* alternative_member = chosen_alternative(composition);
            if (alternative_member != nullptr) {
                end = lower_member(*alternative_member, start, act);
            }
        }

        if (bound.has_value()) {
            // A composition duration bounds the whole composition from its own
            // start (§7.6.2.1.2), so it only fixes an end time when the start
            // itself is a time.
            if (start.at.has_value()) {
                end.at = *start.at + *bound;
                end.after.clear();
            } else {
                warn(composition.range,
                     "this composition's duration is measured from a start no constant fixes, so "
                     "it does not bound an absolute time (§7.6.2.4)");
            }
        }
        return end;
    }

    TimePoint lower_wait(const DoMember& member, const TimePoint& start) {
        // §7.6.2.4.2: `wait elapsed(d)` adds a phase of length d in which
        // nothing is specified. Nothing is exactly what it lowers to — the
        // clock is what advances, and the clock is already running.
        const std::optional<double> elapsed = elapsed_of(member.event);
        if (elapsed.has_value()) {
            if (start.at.has_value()) {
                TimePoint end = start;
                end.at = *start.at + *elapsed;
                end.after.clear();
                return end;
            }
            warn(member.range, "'wait elapsed' follows a phase whose end no constant fixes, so the "
                               "wait has no absolute end (§7.6.2.4.2)");
            return start;
        }
        warn(member.range, "only 'wait elapsed(<duration>)' is lowered in v0.0.1; waiting for an "
                           "event needs the event carrier §7.6.2.5 has no runtime counterpart for");
        return start;
    }

    TimePoint lower_invocation(const DoMember& member, const TimePoint& start, ir::Act& act) {
        std::string actor;
        TypeId actor_type = kInvalidType;
        if (!actor_of(member, actor, actor_type)) {
            return start;
        }
        report_deferred(member);
        const std::optional<double> duration = duration_of(member.arguments, "this invocation");
        std::vector<std::shared_ptr<ir::Action>> actions = actions_of(member, actor, actor_type);

        TimePoint end = start;
        if (duration.has_value()) {
            if (start.at.has_value()) {
                end.at = *start.at + *duration;
                end.after.clear();
            } else {
                warn(member.range,
                     "this invocation's duration is measured from a start no constant fixes, so it "
                     "does not bound an absolute time (§7.6.2.4.1)");
            }
        }
        if (actions.empty()) {
            // Nothing to run — a generic action, or one already reported. The
            // phase still takes its duration, because the actor is still doing
            // something for that long.
            return end;
        }

        const std::string name = phase_name(member, act.groups.size());
        ir::Event event;
        event.name = name;
        event.actions = std::move(actions);
        event.start_trigger = trigger_for(start);

        ir::Maneuver maneuver;
        maneuver.name = name;
        maneuver.events.push_back(std::move(event));

        ir::ManeuverGroup group;
        group.name = name;
        group.actors.push_back(actor);
        group.maneuvers.push_back(std::move(maneuver));
        act.groups.push_back(std::move(group));

        if (!duration.has_value() || !start.at.has_value()) {
            // The phase ends when its actions do, and only the runtime knows
            // when that is.
            end.at = start.at.has_value() && duration.has_value() ? end.at : std::nullopt;
            end.after = {name};
        }
        return end;
    }

    /// The `do` directive as a storyboard.
    ///
    /// One Story, one Act, and one ManeuverGroup per phase — the group is where
    /// an actor lives, so a group per phase keeps each invocation's actor with
    /// its actions. Composition decides only *when* a phase starts, which is a
    /// start trigger on its Event; the runtime is unchanged.
    void build(const DoMember& root, const std::string& story_name, ir::Storyboard& out) {
        ir::Act act;
        act.name = "act";
        const TimePoint end = lower_member(root, TimePoint{0.0, {}}, act);

        if (act.groups.empty()) {
            return; // an empty Act would be an init-time validation error
        }
        ir::Story story;
        story.name = story_name;
        story.acts.push_back(std::move(act));
        out.stories.push_back(std::move(story));

        // §7.6.2: the scenario ends when its `do` directive ends. When that
        // moment is a constant the storyboard can say so, which is what makes
        // the run's element states finish rather than hang at running.
        if (end.at.has_value() && end.after.empty() && *end.at > 0.0) {
            out.stop_trigger = trigger_for(end);
        }
    }
};

} // namespace

std::vector<std::string> entry_points(const Program& program, const LoadResult& loaded) {
    std::vector<std::string> names;
    const File* root = loaded.root();
    if (root == nullptr) {
        return names;
    }
    // Declaration order, not name order: this is what a file *offers*, and a
    // reader matches it against the file they are looking at.
    std::string name_space;
    for (const Declaration& declaration : root->declarations) {
        if (declaration.kind == Declaration::Kind::Namespace) {
            name_space = declaration.name_space.name;
            continue;
        }
        if (declaration.kind != Declaration::Kind::Structured ||
            declaration.structured.kind != StructuredKind::Scenario) {
            continue;
        }
        const std::string qualified = name_space + "::" + declaration.structured.name;
        if (program.types_by_name.count(qualified) != 0) {
            names.push_back(qualified);
        }
    }
    return names;
}

Status lower(const Program& program, const LoadResult& loaded, const LowerOptions& options,
             LowerResult& result, DiagnosticSink& sink) {
    ir::Scenario& out = result.scenario;
    const File* root = loaded.root();
    if (root == nullptr) {
        // Nothing parsed. Host misuse: lowering is only defined for a program
        // that checked.
        return Status::InvalidArgument;
    }

    const std::vector<std::string> available = entry_points(program, loaded);
    std::string entry;
    if (options.entry_point.empty()) {
        if (available.size() == 1) {
            entry = available.front();
        } else {
            std::string message =
                available.empty()
                    ? "the file declares no scenario to run (§7.7.2)"
                    : "the file declares more than one scenario; name the entry point (§7.7.2):";
            for (const std::string& name : available) {
                message += " " + name;
            }
            report(sink, Severity::Error, Status::SemanticError, root->path, SourceRange{},
                   std::move(message));
            return Status::SemanticError;
        }
    } else {
        for (const std::string& name : available) {
            // Accept both the qualified name and the name as written, since a
            // file with one namespace makes the prefix pure ceremony.
            const std::size_t separator = name.rfind("::");
            const std::string simple =
                separator == std::string::npos ? name : name.substr(separator + 2);
            if (name == options.entry_point || simple == options.entry_point) {
                entry = name;
                break;
            }
        }
        if (entry.empty()) {
            std::string message =
                "'" + options.entry_point + "' is not a scenario this file declares (§7.7.2);";
            message += available.empty() ? " it declares none" : " it declares:";
            for (const std::string& name : available) {
                message += " " + name;
            }
            report(sink, Severity::Error, Status::SemanticError, root->path, SourceRange{},
                   std::move(message));
            return Status::SemanticError;
        }
    }

    const TypeId scenario_id = program.types_by_name.at(entry);
    const TypeInfo& scenario = program.types[scenario_id];
    out.name = scenario.simple_name;

    ExpressionContext context;
    context.self = scenario_id;
    context.name_space = scenario.name_space;
    context.file = root->path;
    // The use list is not kept on a resolved type — it belongs to the scope the
    // declaration was written in — so it is read back off the file. Without it
    // an enum literal like `vehicle_category!bus` cannot name its enum, and
    // every enum-valued constraint would silently fail to bind.
    context.uses = uses_of(*root, scenario.name_space);

    std::vector<Binding> bindings;
    for (const StructuredDecl* declaration : scenario.declarations) {
        if (declaration != nullptr) {
            collect_bindings(program, *declaration, context, bindings);
        }
    }

    const LibraryTypes library = library_types(program);
    for (const std::string& field_name : scenario.field_order) {
        const auto found = scenario.fields.find(field_name);
        if (found == scenario.fields.end()) {
            continue;
        }
        const FieldInfo& field = found->second;
        if (library.physical_object == kInvalidType ||
            !program.is_derived_from(field.type, library.physical_object)) {
            continue; // not a participant; §8.7's root is what makes one
        }

        ir::Entity entity;
        entity.id = field.name;
        entity.name = field.name;
        // Every lowered participant is engine-controlled: the DSL has no way to
        // say otherwise, and the host reassigns ownership through the engine
        // API (ADR-0003).
        entity.control_mode = ir::ControlMode::EngineControlled;

        if (library.vehicle != kInvalidType &&
            program.is_derived_from(field.type, library.vehicle)) {
            ir::Vehicle vehicle;
            vehicle.bounding_box = lower_bounding_box(bindings, field.name);
            const std::string category =
                bound_enum(program, bindings, field.name, {"vehicle_category"});
            if (const std::optional<ir::VehicleCategory> mapped = vehicle_category_of(category);
                mapped.has_value()) {
                vehicle.category = *mapped;
            } else {
                // A conditional `inherits vehicle(vehicle_category == car)`
                // (§7.3.8.2) fixes it on the type rather than in the scenario.
                Value value;
                for (TypeId current = field.type; current != kInvalidType;
                     current = program.types[current].base) {
                    const TypeInfo& type = program.types[current];
                    if (type.constraint_field == "vehicle_category" &&
                        type.constraint_value != nullptr &&
                        evaluate_constant(program, *type.constraint_value, context, value) &&
                        value.kind == Value::Kind::Enum && value.type != kInvalidType) {
                        for (const EnumMemberInfo& member :
                             program.types[value.type].enum_members) {
                            if (member.value != value.enum_value) {
                                continue;
                            }
                            if (const std::optional<ir::VehicleCategory> inherited =
                                    vehicle_category_of(member.name);
                                inherited.has_value()) {
                                vehicle.category = *inherited;
                            }
                            break;
                        }
                        break;
                    }
                }
            }
            // §8.7 declares no performance limits at all — the DSL domain
            // model has no counterpart to XML's Performance element. The IR's
            // zeros are the faithful lowering, not a gap: the runtime already
            // reads a non-positive limit as "unconstrained"
            // (`actor_max_speed`), so a DSL vehicle is simply unlimited until
            // the scenario says otherwise.
            entity.object = std::move(vehicle);
        } else if (library.person != kInvalidType &&
                   program.is_derived_from(field.type, library.person)) {
            ir::Pedestrian pedestrian;
            pedestrian.bounding_box = lower_bounding_box(bindings, field.name);
            // §8.7.9 declares `person` with no members of its own, so there is
            // no category to read; the IR's default stands. `animal` is a
            // sibling actor rather than a person category, and lowers as an
            // unclassified participant until the taxonomy has somewhere to put
            // it.
            entity.object = std::move(pedestrian);
        } else if (library.stationary_object != kInvalidType &&
                   program.is_derived_from(field.type, library.stationary_object)) {
            ir::MiscObject object;
            object.bounding_box = lower_bounding_box(bindings, field.name);
            entity.object = std::move(object);
        }
        // Anything else deriving from physical_object stays an unclassified
        // participant: it has an identity and a control mode, which is all the
        // runtime needs of it.

        out.entities.push_back(std::move(entity));
    }

    // A scenario that says nothing about who is in it has nothing to run.
    if (out.entities.empty()) {
        report(sink, Severity::Warning, Status::UnsupportedFeature, root->path, SourceRange{},
               "'" + entry + "' declares no §8.7 participant, so the lowered scenario is empty");
    }

    // §8.5.4's map_file, in either spelling the standard prints: the
    // `map.set_map_file("m.xodr")` modifier of Code 61, or a `keep` on a
    // declared `map` field (Code 62). Both are the same statement about which
    // road network the scenario means, and neither is kernel state — the file
    // reference goes to the host, exactly as `LogicFile` does on the XML side.
    for (const Binding& binding : bindings) {
        if (binding.path.size() != 2 || binding.path[1] != "map_file" ||
            binding.value.kind != Value::Kind::String) {
            continue;
        }
        const auto field = scenario.fields.find(binding.path.front());
        if (field != scenario.fields.end() && library.map != kInvalidType &&
            program.is_derived_from(field->second.type, library.map)) {
            result.map_file = binding.value.text;
        }
    }

    Behaviors behaviors{program, library, root->path, context, bindings, {}, options.alternative,
                        {},      sink};
    for (const ir::Entity& entity : out.entities) {
        behaviors.entities.insert(entity.id);
    }

    for (const StructuredDecl* declaration : scenario.declarations) {
        if (declaration == nullptr) {
            continue;
        }
        for (const Member& member : declaration->members) {
            const TypeId set_map_file = find_type(program, "std::map.set_map_file");
            if (member.kind == Member::Kind::ModifierApplication &&
                member.modifier.name == "set_map_file" && set_map_file != kInvalidType) {
                const ArgumentMap arguments =
                    bind_arguments(program, set_map_file, member.modifier.arguments, context);
                const auto file = arguments.find("file");
                if (file != arguments.end() && file->second.folded &&
                    file->second.value.kind == Value::Kind::String) {
                    result.map_file = file->second.value.text;
                }
                continue;
            }
            if (member.kind == Member::Kind::Behavior && member.behavior != nullptr) {
                behaviors.build(*member.behavior, scenario.simple_name, out.storyboard);
            }
        }
    }

    return sink.has_errors() ? Status::ValidationError : Status::Ok;
}

} // namespace scena::dsl
