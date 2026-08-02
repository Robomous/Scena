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

#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "scena/dsl/ast.h"
#include "scena/dsl/expression.h"
#include "scena/ir/bounding_box.h"
#include "scena/ir/entity.h"
#include "scena/ir/entity_types.h"

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
             ir::Scenario& out, DiagnosticSink& sink) {
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
                            if (const std::optional<ir::VehicleCategory> mapped =
                                    vehicle_category_of(member.name);
                                mapped.has_value()) {
                                vehicle.category = *mapped;
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

    return sink.has_errors() ? Status::ValidationError : Status::Ok;
}

} // namespace scena::dsl
