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

#include "read_entities.h"

#include <string>
#include <utility>

#include "read_common.h"
#include "scena/ir/entity_types.h"

namespace scena::xml::detail {

namespace {

// Enumeration literals, spelled exactly as the XSD spells them. Only the
// 1.0-1.3 literal set is reachable from a document Scena accepts, but the IR
// carries the full 1.4 enumeration (p2-s1), so the 1.4-only literals map too
// — a document declaring one is already rejected by version detection.
constexpr std::initializer_list<EnumEntry<ir::VehicleCategory>> kVehicleCategories = {
    {"aircraft", ir::VehicleCategory::Aircraft},
    {"bicycle", ir::VehicleCategory::Bicycle},
    {"bus", ir::VehicleCategory::Bus},
    {"car", ir::VehicleCategory::Car},
    {"heavyTruck", ir::VehicleCategory::HeavyTruck},
    {"landVehicle", ir::VehicleCategory::LandVehicle},
    {"micromobilityDevice", ir::VehicleCategory::MicromobilityDevice},
    {"motorbike", ir::VehicleCategory::Motorbike},
    {"motorcycle", ir::VehicleCategory::Motorcycle},
    {"other", ir::VehicleCategory::Other},
    {"semitractor", ir::VehicleCategory::Semitractor},
    {"semitrailer", ir::VehicleCategory::Semitrailer},
    {"standupScooter", ir::VehicleCategory::StandupScooter},
    {"trailer", ir::VehicleCategory::Trailer},
    {"train", ir::VehicleCategory::Train},
    {"tram", ir::VehicleCategory::Tram},
    {"truck", ir::VehicleCategory::Truck},
    {"van", ir::VehicleCategory::Van},
    {"watercraft", ir::VehicleCategory::Watercraft},
    {"wheelchair", ir::VehicleCategory::Wheelchair},
    {"workMachine", ir::VehicleCategory::WorkMachine},
};

constexpr std::initializer_list<EnumEntry<ir::PedestrianCategory>> kPedestrianCategories = {
    {"animal", ir::PedestrianCategory::Animal},
    {"pedestrian", ir::PedestrianCategory::Pedestrian},
    {"wheelchair", ir::PedestrianCategory::Wheelchair},
};

constexpr std::initializer_list<EnumEntry<ir::MiscObjectCategory>> kMiscObjectCategories = {
    {"barrier", ir::MiscObjectCategory::Barrier},
    {"building", ir::MiscObjectCategory::Building},
    {"crosswalk", ir::MiscObjectCategory::Crosswalk},
    {"gantry", ir::MiscObjectCategory::Gantry},
    {"none", ir::MiscObjectCategory::None},
    {"obstacle", ir::MiscObjectCategory::Obstacle},
    {"parkingSpace", ir::MiscObjectCategory::ParkingSpace},
    {"patch", ir::MiscObjectCategory::Patch},
    {"pole", ir::MiscObjectCategory::Pole},
    {"railing", ir::MiscObjectCategory::Railing},
    {"roadMark", ir::MiscObjectCategory::RoadMark},
    {"soundBarrier", ir::MiscObjectCategory::SoundBarrier},
    {"streetLamp", ir::MiscObjectCategory::StreetLamp},
    {"trafficIsland", ir::MiscObjectCategory::TrafficIsland},
    {"tree", ir::MiscObjectCategory::Tree},
    {"vegetation", ir::MiscObjectCategory::Vegetation},
    {"wind", ir::MiscObjectCategory::Wind},
};

constexpr std::initializer_list<EnumEntry<ir::Role>> kRoles = {
    {"none", ir::Role::None},
    {"agriculture", ir::Role::Agriculture},
    {"ambulance", ir::Role::Ambulance},
    {"civil", ir::Role::Civil},
    {"construction", ir::Role::Construction},
    {"dangerousGoodsTransport", ir::Role::DangerousGoodsTransport},
    {"fire", ir::Role::Fire},
    {"fireBrigade", ir::Role::FireBrigade},
    {"freightTransport", ir::Role::FreightTransport},
    {"garbageCollection", ir::Role::GarbageCollection},
    {"military", ir::Role::Military},
    {"other", ir::Role::Other},
    {"police", ir::Role::Police},
    {"publicTransport", ir::Role::PublicTransport},
    {"roadAssistance", ir::Role::RoadAssistance},
    {"roadsideAssistance", ir::Role::RoadsideAssistance},
    {"specialTransport", ir::Role::SpecialTransport},
    {"trafficControl", ir::Role::TrafficControl},
};

bool read_performance(ReadContext& ctx, const pugi::xml_node& node, ir::Performance& out) {
    bool ok = require_double(ctx, node, "maxSpeed", out.max_speed);
    ok = require_double(ctx, node, "maxAcceleration", out.max_acceleration) && ok;
    ok = require_double(ctx, node, "maxDeceleration", out.max_deceleration) && ok;
    // The rate limits arrived in 1.2 and mean "unbounded" when absent.
    ok = optional_double(ctx, node, "maxAccelerationRate", out.max_acceleration_rate) && ok;
    ok = optional_double(ctx, node, "maxDecelerationRate", out.max_deceleration_rate) && ok;
    return ok;
}

bool read_axle(ReadContext& ctx, const pugi::xml_node& node, ir::Axle& out) {
    bool ok = require_double(ctx, node, "maxSteering", out.max_steering);
    ok = require_double(ctx, node, "positionX", out.position_x) && ok;
    ok = require_double(ctx, node, "positionZ", out.position_z) && ok;
    ok = require_double(ctx, node, "trackWidth", out.track_width) && ok;
    ok = require_double(ctx, node, "wheelDiameter", out.wheel_diameter) && ok;
    return ok;
}

bool read_axles(ReadContext& ctx, const pugi::xml_node& node, ir::Axles& out) {
    static const char* const kConsumed[] = {"FrontAxle", "RearAxle", "AdditionalAxle", nullptr};
    warn_unconsumed_children(ctx, node, kConsumed);

    bool ok = true;
    if (const pugi::xml_node front = node.child("FrontAxle")) {
        ir::Axle axle;
        ok = read_axle(ctx, front, axle) && ok;
        out.front = axle;
    }
    const pugi::xml_node rear = require_child(ctx, node, "RearAxle");
    if (!rear) {
        return false;
    }
    ok = read_axle(ctx, rear, out.rear) && ok;
    for (pugi::xml_node additional : node.children("AdditionalAxle")) {
        ir::Axle axle;
        ok = read_axle(ctx, additional, axle) && ok;
        out.additional.push_back(axle);
    }
    return ok;
}

bool read_vehicle(ReadContext& ctx, const pugi::xml_node& node, ir::Vehicle& out) {
    static const char* const kConsumed[] = {
        "BoundingBox", "Performance",           "Axles",
        "Properties",  "TrailerHitch",          "TrailerCoupler",
        "Trailer",     "ParameterDeclarations", nullptr};
    warn_unconsumed_children(ctx, node, kConsumed);
    for (const char* deferred : {"TrailerHitch", "TrailerCoupler", "Trailer"}) {
        if (const pugi::xml_node child = node.child(deferred)) {
            warn_out_of_scope(ctx, child, "trailers are outside the v0.0.1 scope");
        }
    }
    if (const pugi::xml_node declarations = node.child("ParameterDeclarations")) {
        warn_deferred(ctx, declarations, "p4-s3");
    }

    bool ok = read_enum(ctx, node, "vehicleCategory", kVehicleCategories, out.category);
    ok = read_enum(ctx, node, "role", kRoles, out.role) && ok;
    ok = optional_double(ctx, node, "mass", out.mass) && ok;

    const pugi::xml_node box = require_child(ctx, node, "BoundingBox");
    ok = box && read_bounding_box(ctx, box, out.bounding_box) && ok;
    const pugi::xml_node performance = require_child(ctx, node, "Performance");
    ok = performance && read_performance(ctx, performance, out.performance) && ok;
    const pugi::xml_node axles = require_child(ctx, node, "Axles");
    ok = axles && read_axles(ctx, axles, out.axles) && ok;
    read_properties(ctx, node.child("Properties"), out.properties);
    return ok;
}

bool read_pedestrian(ReadContext& ctx, const pugi::xml_node& node, ir::Pedestrian& out) {
    static const char* const kConsumed[] = {"BoundingBox", "Properties", "ParameterDeclarations",
                                            nullptr};
    warn_unconsumed_children(ctx, node, kConsumed);
    if (const pugi::xml_node declarations = node.child("ParameterDeclarations")) {
        warn_deferred(ctx, declarations, "p4-s3");
    }

    bool ok = read_enum(ctx, node, "pedestrianCategory", kPedestrianCategories, out.category);
    ok = read_enum(ctx, node, "role", kRoles, out.role) && ok;
    ok = optional_double(ctx, node, "mass", out.mass) && ok;
    const pugi::xml_node box = require_child(ctx, node, "BoundingBox");
    ok = box && read_bounding_box(ctx, box, out.bounding_box) && ok;
    read_properties(ctx, node.child("Properties"), out.properties);
    return ok;
}

bool read_misc_object(ReadContext& ctx, const pugi::xml_node& node, ir::MiscObject& out) {
    static const char* const kConsumed[] = {"BoundingBox", "Properties", "ParameterDeclarations",
                                            nullptr};
    warn_unconsumed_children(ctx, node, kConsumed);
    if (const pugi::xml_node declarations = node.child("ParameterDeclarations")) {
        warn_deferred(ctx, declarations, "p4-s3");
    }

    bool ok = read_enum(ctx, node, "miscObjectCategory", kMiscObjectCategories, out.category);
    ok = optional_double(ctx, node, "mass", out.mass) && ok;
    const pugi::xml_node box = require_child(ctx, node, "BoundingBox");
    ok = box && read_bounding_box(ctx, box, out.bounding_box) && ok;
    read_properties(ctx, node.child("Properties"), out.properties);
    return ok;
}

/// Reads one ScenarioObject (§7.2.2) — the entity identity, its concrete
/// object, and the controller assignment that decides control ownership.
void read_scenario_object(ReadContext& ctx, const pugi::xml_node& node,
                          std::vector<ir::Entity>& out) {
    static const char* const kConsumed[] = {
        "CatalogReference",        "Vehicle",          "Pedestrian", "MiscObject",
        "ExternalObjectReference", "ObjectController", nullptr};
    warn_unconsumed_children(ctx, node, kConsumed);

    ir::Entity entity;
    if (!require_string(ctx, node, "name", entity.name)) {
        return;
    }
    // One name, one identity: the standard references entities by their
    // ScenarioObject name everywhere, so the IR id is that name.
    entity.id = entity.name;

    if (const pugi::xml_node external = node.child("ExternalObjectReference")) {
        warn_out_of_scope(ctx, external,
                          "external object references need road-network object binding");
    }
    if (const pugi::xml_node reference = node.child("CatalogReference")) {
        warn_deferred(ctx, reference, "p4-s4");
    }
    if (const pugi::xml_node controller = node.child("ObjectController")) {
        // Control ownership and controller metadata are p4-s4's ObjectController
        // lowering; until then every loaded entity is engine-controlled, which
        // is the IR default and what the storyboard's private actions expect.
        warn_deferred(ctx, controller, "p4-s4");
    }

    if (const pugi::xml_node vehicle = node.child("Vehicle")) {
        ir::Vehicle object;
        if (read_vehicle(ctx, vehicle, object)) {
            entity.object = std::move(object);
        }
    } else if (const pugi::xml_node pedestrian = node.child("Pedestrian")) {
        ir::Pedestrian object;
        if (read_pedestrian(ctx, pedestrian, object)) {
            entity.object = std::move(object);
        }
    } else if (const pugi::xml_node misc = node.child("MiscObject")) {
        ir::MiscObject object;
        if (read_misc_object(ctx, misc, object)) {
            entity.object = std::move(object);
        }
    }
    // No concrete object (catalog reference, external reference, or a failed
    // read) leaves the entity unclassified — a valid IR entity that simply
    // carries no geometry, which the runtime reports when something needs it.
    out.push_back(std::move(entity));
}

} // namespace

bool read_bounding_box(ReadContext& ctx, const pugi::xml_node& node, ir::BoundingBox& out) {
    static const char* const kConsumed[] = {"Center", "Dimensions", nullptr};
    warn_unconsumed_children(ctx, node, kConsumed);

    bool ok = true;
    const pugi::xml_node center = require_child(ctx, node, "Center");
    if (center) {
        ok = require_double(ctx, center, "x", out.center_x) && ok;
        ok = require_double(ctx, center, "y", out.center_y) && ok;
        ok = require_double(ctx, center, "z", out.center_z) && ok;
    } else {
        ok = false;
    }
    const pugi::xml_node dimensions = require_child(ctx, node, "Dimensions");
    if (dimensions) {
        ok = require_double(ctx, dimensions, "width", out.width) && ok;
        ok = require_double(ctx, dimensions, "length", out.length) && ok;
        ok = require_double(ctx, dimensions, "height", out.height) && ok;
    } else {
        ok = false;
    }
    return ok;
}

void read_properties(ReadContext& ctx, const pugi::xml_node& node, std::vector<ir::Property>& out) {
    if (!node) {
        return;
    }
    static const char* const kConsumed[] = {"Property", "File", "CustomContent", nullptr};
    warn_unconsumed_children(ctx, node, kConsumed);
    for (pugi::xml_node file : node.children("File")) {
        warn_out_of_scope(ctx, file, "property files are not read by the engine");
    }
    for (pugi::xml_node custom : node.children("CustomContent")) {
        warn_out_of_scope(ctx, custom, "custom property content is not read by the engine");
    }
    for (pugi::xml_node property : node.children("Property")) {
        ir::Property entry;
        if (!require_string(ctx, property, "name", entry.name)) {
            continue;
        }
        if (!require_string(ctx, property, "value", entry.value)) {
            continue;
        }
        out.push_back(std::move(entry));
    }
}

void read_entities(ReadContext& ctx, const pugi::xml_node& entities, std::vector<ir::Entity>& out) {
    static const char* const kConsumed[] = {"ScenarioObject", "EntitySelection", nullptr};
    warn_unconsumed_children(ctx, entities, kConsumed);
    for (pugi::xml_node selection : entities.children("EntitySelection")) {
        warn_deferred(ctx, selection, "p4-s4");
    }
    for (pugi::xml_node object : entities.children("ScenarioObject")) {
        read_scenario_object(ctx, object, out);
    }
}

} // namespace scena::xml::detail
