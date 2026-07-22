// SPDX-License-Identifier: MIT
#pragma once

#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "scena/ir/bounding_box.h"
#include "scena/ir/entity_types.h"

/// The scenario entity model, per ASAM OpenSCENARIO XML 1.4.0 §7.2.2. An
/// `Entity` is the runtime's view of a `ScenarioObject`: a name-identified
/// participant optionally classified as one concrete `EntityObject`
/// (Vehicle / Pedestrian / MiscObject). The concrete objects carry the
/// geometry, categories, performance, axles, and properties the standard
/// defines; the value types live in entity_types.h and bounding_box.h.
namespace scena::ir {

/// Who drives an entity each step.
enum class ControlMode {
    EngineControlled, ///< The engine integrates the entity's motion (default behavior).
    HostControlled,   ///< The host simulator reports the entity's state each step.
};

/// A vehicle object, per ASAM OpenSCENARIO XML 1.4.0 §Vehicle. Rendering
/// (model3d) and catalog parameterDeclarations are out of scope for v0.0.1;
/// mass is optional here (spec cardinality: Vehicle 0..1) and preserved as
/// data — the straight-line runtime does not read it.
struct Vehicle {
    VehicleCategory category = VehicleCategory::Car;
    Role role = Role::None;     ///< Default None when unspecified (§Vehicle).
    std::optional<double> mass; ///< kg, Range [0..inf[; absent ⇒ unspecified.
    BoundingBox bounding_box;
    Performance performance;
    Axles axles;
    std::vector<Property> properties; ///< Ordered, document order (§Properties).
};

/// A pedestrian object, per ASAM OpenSCENARIO XML 1.4.0 §Pedestrian. No
/// Performance/Axles in the standard. mass is required by the spec (1..1); it
/// is modeled optional here so a frontend that omits it lowers cleanly, and is
/// carried as data only.
struct Pedestrian {
    PedestrianCategory category = PedestrianCategory::Pedestrian;
    Role role = Role::None;     ///< Default None when unspecified (§Pedestrian).
    std::optional<double> mass; ///< kg, Range [0..inf[; absent ⇒ unspecified.
    BoundingBox bounding_box;
    std::vector<Property> properties; ///< Ordered, document order (§Properties).
};

/// A miscellaneous object, per ASAM OpenSCENARIO XML 1.4.0 §MiscObject. No
/// role and no Performance/Axles in the standard.
struct MiscObject {
    MiscObjectCategory category = MiscObjectCategory::Obstacle;
    std::optional<double> mass; ///< kg, Range [0..inf[; absent ⇒ unspecified.
    BoundingBox bounding_box;
    std::vector<Property> properties; ///< Ordered, document order (§Properties).
};

/// One concrete entity object, the choice in a ScenarioObject (§7.2.2). The
/// alternative order (Vehicle, Pedestrian, MiscObject) matches ObjectType.
using EntityObject = std::variant<Vehicle, Pedestrian, MiscObject>;

/// A scenario participant, i.e. a `ScenarioObject` (§7.2.2): a stable id and
/// display name, a control-ownership mode, and — when the scenario classifies
/// it — one concrete `EntityObject`. `object` is optional so a bare
/// participant (identity + control mode only) remains valid, matching the F0
/// minimal entity. The geometry the interaction conditions need (ADR-0009)
/// now lives inside the concrete object; read it through `bounding_box_of`.
struct Entity {
    std::string id;
    std::string name;
    ControlMode control_mode = ControlMode::EngineControlled;
    std::optional<EntityObject> object; ///< Absent ⇒ an unclassified participant.
};

/// The ObjectType of an entity's concrete object, or nullopt when it has none
/// (an unclassified participant). Derived from the `EntityObject` alternative
/// (§ObjectType).
[[nodiscard]] std::optional<ObjectType> object_type_of(const Entity& entity);

/// The bounding box of an entity's concrete object, or nullopt when it has
/// none. All three concrete objects carry a box; this is the single geometry
/// source the runtime freespace path reads.
[[nodiscard]] std::optional<BoundingBox> bounding_box_of(const Entity& entity);

/// The Performance limits of an entity, or nullptr when it is not a Vehicle
/// (only Vehicle defines Performance in the standard). The pointer is valid
/// for the lifetime of `entity`.
[[nodiscard]] const Performance* performance_of(const Entity& entity);

} // namespace scena::ir
