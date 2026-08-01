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

#include "read_geometry.h"

#include <string>
#include <utility>
#include <vector>

#include "parameters.h"
#include "read_common.h"

namespace scena::xml::detail {

namespace {

constexpr std::initializer_list<EnumEntry<ir::ReferenceContext>> kReferenceContexts = {
    {"absolute", ir::ReferenceContext::Absolute},
    {"relative", ir::ReferenceContext::Relative},
};

constexpr std::initializer_list<EnumEntry<ir::RouteStrategy>> kRouteStrategies = {
    {"fastest", ir::RouteStrategy::Fastest},
    {"leastIntersections", ir::RouteStrategy::LeastIntersections},
    {"random", ir::RouteStrategy::Random},
    {"shortest", ir::RouteStrategy::Shortest},
};

constexpr std::initializer_list<EnumEntry<ir::DynamicsShape>> kDynamicsShapes = {
    {"linear", ir::DynamicsShape::Linear},
    {"cubic", ir::DynamicsShape::Cubic},
    {"sinusoidal", ir::DynamicsShape::Sinusoidal},
    {"step", ir::DynamicsShape::Step},
};

constexpr std::initializer_list<EnumEntry<ir::DynamicsDimension>> kDynamicsDimensions = {
    {"rate", ir::DynamicsDimension::Rate},
    {"time", ir::DynamicsDimension::Time},
    {"distance", ir::DynamicsDimension::Distance},
};

constexpr std::initializer_list<EnumEntry<ir::FollowingMode>> kFollowingModes = {
    {"position", ir::FollowingMode::Position},
    {"follow", ir::FollowingMode::Follow},
};

/// Reads the optional `Orientation` child every relative position carries.
std::optional<ir::Orientation> read_optional_orientation(ReadContext& ctx,
                                                         const pugi::xml_node& node) {
    const pugi::xml_node orientation = node.child("Orientation");
    if (!orientation) {
        return std::nullopt;
    }
    return read_orientation(ctx, orientation);
}

bool read_world_position(ReadContext& ctx, const pugi::xml_node& node, ir::WorldPosition& out) {
    // x and y are required; z/h/p/r are optional and default to 0
    // (§WorldPosition).
    bool ok = require_double(ctx, node, "x", out.x);
    ok = require_double(ctx, node, "y", out.y) && ok;
    ok = optional_double(ctx, node, "z", out.z) && ok;
    ok = optional_double(ctx, node, "h", out.h) && ok;
    ok = optional_double(ctx, node, "p", out.p) && ok;
    ok = optional_double(ctx, node, "r", out.r) && ok;
    return ok;
}

std::optional<ir::Position> read_relative_world(ReadContext& ctx, const pugi::xml_node& node) {
    ir::RelativeWorldPosition position;
    bool ok = require_string(ctx, node, "entityRef", position.entity_ref);
    ok = require_double(ctx, node, "dx", position.dx) && ok;
    ok = require_double(ctx, node, "dy", position.dy) && ok;
    ok = optional_double(ctx, node, "dz", position.dz) && ok;
    position.orientation = read_optional_orientation(ctx, node);
    return ok ? std::optional<ir::Position>(std::move(position)) : std::nullopt;
}

std::optional<ir::Position> read_relative_object(ReadContext& ctx, const pugi::xml_node& node) {
    ir::RelativeObjectPosition position;
    bool ok = require_string(ctx, node, "entityRef", position.entity_ref);
    ok = require_double(ctx, node, "dx", position.dx) && ok;
    ok = require_double(ctx, node, "dy", position.dy) && ok;
    ok = optional_double(ctx, node, "dz", position.dz) && ok;
    position.orientation = read_optional_orientation(ctx, node);
    return ok ? std::optional<ir::Position>(std::move(position)) : std::nullopt;
}

std::optional<ir::Position> read_road(ReadContext& ctx, const pugi::xml_node& node) {
    ir::RoadPosition position;
    bool ok = require_string(ctx, node, "roadId", position.road_id);
    ok = require_double(ctx, node, "s", position.s) && ok;
    ok = require_double(ctx, node, "t", position.t) && ok;
    position.orientation = read_optional_orientation(ctx, node);
    return ok ? std::optional<ir::Position>(std::move(position)) : std::nullopt;
}

std::optional<ir::Position> read_relative_road(ReadContext& ctx, const pugi::xml_node& node) {
    ir::RelativeRoadPosition position;
    bool ok = require_string(ctx, node, "entityRef", position.entity_ref);
    ok = require_double(ctx, node, "ds", position.ds) && ok;
    ok = require_double(ctx, node, "dt", position.dt) && ok;
    position.orientation = read_optional_orientation(ctx, node);
    return ok ? std::optional<ir::Position>(std::move(position)) : std::nullopt;
}

std::optional<ir::Position> read_lane(ReadContext& ctx, const pugi::xml_node& node) {
    ir::LanePosition position;
    bool ok = require_string(ctx, node, "roadId", position.road_id);
    ok = require_string(ctx, node, "laneId", position.lane_id) && ok;
    ok = require_double(ctx, node, "s", position.s) && ok;
    ok = optional_double(ctx, node, "offset", position.offset) && ok;
    position.orientation = read_optional_orientation(ctx, node);
    return ok ? std::optional<ir::Position>(std::move(position)) : std::nullopt;
}

std::optional<ir::Position> read_relative_lane(ReadContext& ctx, const pugi::xml_node& node) {
    ir::RelativeLanePosition position;
    bool ok = require_string(ctx, node, "entityRef", position.entity_ref);
    ok = require_int(ctx, node, "dLane", position.d_lane) && ok;
    // ds and dsLane are exclusive (§RelativeLanePosition); dsLane arrived in
    // 1.1.
    ok = optional_double(ctx, node, "ds", position.ds) && ok;
    ok = optional_double(ctx, node, "dsLane", position.ds_lane) && ok;
    ok = optional_double(ctx, node, "offset", position.offset) && ok;
    if (position.ds.has_value() && position.ds_lane.has_value()) {
        ctx.report_at(node, Severity::Error, Status::ValidationError, element_path(node),
                      "RelativeLanePosition declares both 'ds' and 'dsLane', which are exclusive");
        ok = false;
    }
    if (!position.ds.has_value() && !position.ds_lane.has_value()) {
        ctx.report_at(node, Severity::Error, Status::ValidationError, element_path(node),
                      "RelativeLanePosition declares neither 'ds' nor 'dsLane'");
        ok = false;
    }
    position.orientation = read_optional_orientation(ctx, node);
    return ok ? std::optional<ir::Position>(std::move(position)) : std::nullopt;
}

std::optional<ir::Position> read_geo(ReadContext& ctx, const pugi::xml_node& node) {
    ir::GeoPosition position;
    // 1.1 renamed the attributes: latitude/longitude/height became
    // latitudeDeg/longitudeDeg/altitude. Both spellings are accepted; the
    // deprecated ones warn.
    bool ok = true;
    if (node.attribute("latitudeDeg") || node.attribute("longitudeDeg")) {
        ok = require_double(ctx, node, "latitudeDeg", position.latitude_deg) && ok;
        ok = require_double(ctx, node, "longitudeDeg", position.longitude_deg) && ok;
        ok = optional_double(ctx, node, "altitude", position.altitude) && ok;
    } else {
        warn_deprecated(ctx, node, "use latitudeDeg/longitudeDeg/altitude (1.1)");
        ok = require_double(ctx, node, "latitude", position.latitude_deg) && ok;
        ok = require_double(ctx, node, "longitude", position.longitude_deg) && ok;
        ok = optional_double(ctx, node, "height", position.altitude) && ok;
    }
    position.orientation = read_optional_orientation(ctx, node);
    return ok ? std::optional<ir::Position>(std::move(position)) : std::nullopt;
}

std::optional<ir::Position> read_trajectory_position(ReadContext& ctx, const pugi::xml_node& node) {
    ir::TrajectoryPosition position;
    bool ok = require_double(ctx, node, "s", position.s);
    ok = optional_double(ctx, node, "t", position.t) && ok;
    position.orientation = read_optional_orientation(ctx, node);

    const pugi::xml_node reference = require_child(ctx, node, "TrajectoryRef");
    if (!reference) {
        return std::nullopt;
    }
    if (const pugi::xml_node catalog = reference.child("CatalogReference")) {
        warn_deferred(ctx, catalog, "p4-s4");
        return std::nullopt;
    }
    const pugi::xml_node trajectory = require_child(ctx, reference, "Trajectory");
    if (!trajectory) {
        return std::nullopt;
    }
    position.trajectory = read_trajectory(ctx, trajectory);
    if (!position.trajectory) {
        return std::nullopt;
    }
    return ok ? std::optional<ir::Position>(std::move(position)) : std::nullopt;
}

std::optional<ir::Position> read_route_position(ReadContext& ctx, const pugi::xml_node& node) {
    ir::RoutePosition position;

    const pugi::xml_node reference = require_child(ctx, node, "RouteRef");
    if (!reference) {
        return std::nullopt;
    }
    if (const pugi::xml_node catalog = reference.child("CatalogReference")) {
        warn_deferred(ctx, catalog, "p4-s4");
        return std::nullopt;
    }
    const pugi::xml_node route = require_child(ctx, reference, "Route");
    if (!route) {
        return std::nullopt;
    }
    position.route = read_route(ctx, route);
    if (!position.route) {
        return std::nullopt;
    }
    position.orientation = read_optional_orientation(ctx, node);

    // InRoutePosition is a choice of three in-route coordinate forms
    // (§InRoutePosition); exactly one is set on the IR.
    const pugi::xml_node in_route = require_child(ctx, node, "InRoutePosition");
    if (!in_route) {
        return std::nullopt;
    }
    static const char* const kForms[] = {"FromCurrentEntity", "FromRoadCoordinates",
                                         "FromLaneCoordinates", nullptr};
    const pugi::xml_node form = read_choice(ctx, in_route, kForms);
    if (!form) {
        return std::nullopt;
    }
    const std::string_view name = form.name();
    bool ok = true;
    if (name == "FromCurrentEntity") {
        std::string entity_ref;
        ok = require_string(ctx, form, "entityRef", entity_ref);
        position.from_entity = std::move(entity_ref);
    } else if (name == "FromRoadCoordinates") {
        double path_s = 0.0;
        double t = 0.0;
        ok = require_double(ctx, form, "pathS", path_s);
        ok = require_double(ctx, form, "t", t) && ok;
        position.path_s = path_s;
        position.t = t;
    } else {
        double path_s = 0.0;
        std::string lane_id;
        ok = require_double(ctx, form, "pathS", path_s);
        ok = require_string(ctx, form, "laneId", lane_id) && ok;
        ok = optional_double(ctx, form, "laneOffset", position.lane_offset) && ok;
        position.path_s = path_s;
        position.lane_id = std::move(lane_id);
    }
    return ok ? std::optional<ir::Position>(std::move(position)) : std::nullopt;
}

bool read_polyline(ReadContext& ctx, const pugi::xml_node& node, ir::Polyline& out) {
    static const char* const kConsumed[] = {"Vertex", "Interpolation", nullptr};
    warn_unconsumed_children(ctx, node, kConsumed);
    if (const pugi::xml_node interpolation = node.child("Interpolation")) {
        warn_out_of_scope(ctx, interpolation, "polyline interpolation is 1.4-only");
    }

    bool ok = true;
    for (pugi::xml_node vertex : node.children("Vertex")) {
        ir::TrajectoryVertex entry;
        ok = optional_double(ctx, vertex, "time", entry.time) && ok;
        const pugi::xml_node position = require_child(ctx, vertex, "Position");
        if (!position) {
            ok = false;
            continue;
        }
        // A trajectory vertex is a world position in the IR: the shape
        // geometry is evaluated in the world frame, so a road- or
        // entity-relative vertex would need a resolve at load time, which the
        // frontend cannot do (no entity states exist yet).
        const pugi::xml_node world = position.child("WorldPosition");
        if (!world) {
            warn_out_of_scope(ctx, position,
                              "trajectory vertices are loaded as world positions only");
            ok = false;
            continue;
        }
        ok = read_world_position(ctx, world, entry.position) && ok;
        out.vertices.push_back(std::move(entry));
    }
    if (out.vertices.size() < 2) {
        ctx.report_at(node, Severity::Error, Status::ValidationError, element_path(node),
                      "Polyline needs at least two vertices");
        ok = false;
    }
    return ok;
}

bool read_clothoid(ReadContext& ctx, const pugi::xml_node& node, ir::Clothoid& out) {
    bool ok = require_double(ctx, node, "curvature", out.curvature);
    // 1.4 renamed curvatureDot to curvaturePrime; a 1.0-1.3 document spells
    // the former, so both map onto the same field.
    if (node.attribute("curvaturePrime")) {
        ok = optional_double(ctx, node, "curvaturePrime", out.curvature_prime) && ok;
    } else if (node.attribute("curvatureDot")) {
        ok = optional_double(ctx, node, "curvatureDot", out.curvature_prime) && ok;
    }
    ok = require_double(ctx, node, "length", out.length) && ok;
    ok = optional_double(ctx, node, "startTime", out.start_time) && ok;
    ok = optional_double(ctx, node, "stopTime", out.stop_time) && ok;

    const pugi::xml_node position = require_child(ctx, node, "Position");
    if (!position) {
        return false;
    }
    const pugi::xml_node world = position.child("WorldPosition");
    if (!world) {
        warn_out_of_scope(ctx, position, "clothoid start is loaded as a world position only");
        return false;
    }
    return read_world_position(ctx, world, out.start) && ok;
}

bool read_nurbs(ReadContext& ctx, const pugi::xml_node& node, ir::Nurbs& out) {
    static const char* const kConsumed[] = {"ControlPoint", "Knot", nullptr};
    warn_unconsumed_children(ctx, node, kConsumed);

    int order = 2;
    bool ok = require_int(ctx, node, "order", order);
    if (order < 2) {
        ctx.report_at(node, Severity::Error, Status::ValidationError, attribute_path(node, "order"),
                      "Nurbs order must be at least 2");
        ok = false;
    } else {
        out.order = static_cast<unsigned int>(order);
    }

    for (pugi::xml_node point : node.children("ControlPoint")) {
        ir::ControlPoint entry;
        ok = optional_double(ctx, point, "time", entry.time) && ok;
        ok = optional_double(ctx, point, "weight", entry.weight) && ok;
        const pugi::xml_node position = require_child(ctx, point, "Position");
        if (!position) {
            ok = false;
            continue;
        }
        const pugi::xml_node world = position.child("WorldPosition");
        if (!world) {
            warn_out_of_scope(ctx, position,
                              "NURBS control points are loaded as world positions only");
            ok = false;
            continue;
        }
        ok = read_world_position(ctx, world, entry.position) && ok;
        out.control_points.push_back(std::move(entry));
    }
    for (pugi::xml_node knot : node.children("Knot")) {
        double value = 0.0;
        ok = require_double(ctx, knot, "value", value) && ok;
        out.knots.push_back(value);
    }
    // Cardinality per asam.net:xosc:1.0.0:routing.cardinality_of_control_points_in_nurbs.
    if (out.control_points.size() < out.order ||
        out.knots.size() != out.control_points.size() + out.order) {
        ctx.report_at(node, Severity::Error, Status::ValidationError, element_path(node),
                      "Nurbs needs at least 'order' control points and "
                      "control points + order knots",
                      "asam.net:xosc:1.0.0:routing.cardinality_of_control_points_in_nurbs");
        ok = false;
    }
    return ok;
}

} // namespace

ir::Orientation read_orientation(ReadContext& ctx, const pugi::xml_node& node) {
    ir::Orientation orientation;
    (void)read_enum(ctx, node, "type", kReferenceContexts, orientation.type);
    (void)optional_double(ctx, node, "h", orientation.h);
    (void)optional_double(ctx, node, "p", orientation.p);
    (void)optional_double(ctx, node, "r", orientation.r);
    return orientation;
}

bool read_transition_dynamics(ReadContext& ctx, const pugi::xml_node& node,
                              ir::TransitionDynamics& out) {
    bool ok = read_enum(ctx, node, "dynamicsShape", kDynamicsShapes, out.shape);
    ok = read_enum(ctx, node, "dynamicsDimension", kDynamicsDimensions, out.dimension) && ok;
    ok = require_double(ctx, node, "value", out.value) && ok;
    ok = read_enum(ctx, node, "followingMode", kFollowingModes, out.following_mode) && ok;
    return ok;
}

std::shared_ptr<ir::Trajectory> read_trajectory(ReadContext& ctx, const pugi::xml_node& node) {
    static const char* const kConsumed[] = {"ParameterDeclarations", "Shape", nullptr};
    warn_unconsumed_children(ctx, node, kConsumed);
    const ParameterFrame frame(ctx.parameters());
    if (const pugi::xml_node declarations = node.child("ParameterDeclarations")) {
        read_parameter_declarations(ctx, declarations);
    }

    auto trajectory = std::make_shared<ir::Trajectory>();
    bool ok = require_string(ctx, node, "name", trajectory->name);
    ok = optional_bool(ctx, node, "closed", trajectory->closed) && ok;

    const pugi::xml_node shape = require_child(ctx, node, "Shape");
    if (!shape) {
        return nullptr;
    }
    static const char* const kShapes[] = {"Polyline", "Clothoid", "ClothoidSpline", "Nurbs",
                                          nullptr};
    const pugi::xml_node chosen = read_choice(ctx, shape, kShapes);
    if (!chosen) {
        return nullptr;
    }
    const std::string_view name = chosen.name();
    if (name == "Polyline") {
        ir::Polyline polyline;
        ok = read_polyline(ctx, chosen, polyline) && ok;
        trajectory->shape = std::move(polyline);
    } else if (name == "Clothoid") {
        ir::Clothoid clothoid;
        ok = read_clothoid(ctx, chosen, clothoid) && ok;
        trajectory->shape = clothoid;
    } else if (name == "Nurbs") {
        ir::Nurbs nurbs;
        ok = read_nurbs(ctx, chosen, nurbs) && ok;
        trajectory->shape = std::move(nurbs);
    } else {
        // ClothoidSpline is Post-v0.0.1 (coverage matrix, ADR-0018).
        warn_out_of_scope(ctx, chosen, "ClothoidSpline is outside the v0.0.1 scope");
        return nullptr;
    }
    return ok ? trajectory : nullptr;
}

std::shared_ptr<ir::Route> read_route(ReadContext& ctx, const pugi::xml_node& node) {
    static const char* const kConsumed[] = {"ParameterDeclarations", "Waypoint", nullptr};
    warn_unconsumed_children(ctx, node, kConsumed);
    const ParameterFrame frame(ctx.parameters());
    if (const pugi::xml_node declarations = node.child("ParameterDeclarations")) {
        read_parameter_declarations(ctx, declarations);
    }

    auto route = std::make_shared<ir::Route>();
    bool ok = require_string(ctx, node, "name", route->name);
    ok = optional_bool(ctx, node, "closed", route->closed) && ok;

    for (pugi::xml_node waypoint : node.children("Waypoint")) {
        ir::Waypoint entry;
        ok = read_enum(ctx, waypoint, "routeStrategy", kRouteStrategies, entry.strategy) && ok;
        const pugi::xml_node position = require_child(ctx, waypoint, "Position");
        if (!position) {
            ok = false;
            continue;
        }
        // Waypoints are world positions in the IR (§Waypoint carries a full
        // Position, but a route is built before any entity exists, so only the
        // self-contained world variant can be lowered here).
        const pugi::xml_node world = position.child("WorldPosition");
        if (!world) {
            warn_out_of_scope(ctx, position, "route waypoints are loaded as world positions only");
            ok = false;
            continue;
        }
        ok = read_world_position(ctx, world, entry.position) && ok;
        route->waypoints.push_back(entry);
    }
    if (route->waypoints.size() < 2) {
        ctx.report_at(node, Severity::Error, Status::ValidationError, element_path(node),
                      "Route needs at least two waypoints");
        ok = false;
    }
    return ok ? route : nullptr;
}

std::optional<ir::Position> read_position(ReadContext& ctx, const pugi::xml_node& node) {
    static const char* const kVariants[] = {"WorldPosition",
                                            "RelativeWorldPosition",
                                            "RelativeObjectPosition",
                                            "RoadPosition",
                                            "RelativeRoadPosition",
                                            "LanePosition",
                                            "RelativeLanePosition",
                                            "RoutePosition",
                                            "GeoPosition",
                                            "TrajectoryPosition",
                                            nullptr};
    const pugi::xml_node variant = read_choice(ctx, node, kVariants);
    if (!variant) {
        return std::nullopt;
    }
    const std::string_view name = variant.name();
    if (name == "WorldPosition") {
        ir::WorldPosition position;
        if (!read_world_position(ctx, variant, position)) {
            return std::nullopt;
        }
        return ir::Position(position);
    }
    if (name == "RelativeWorldPosition") {
        return read_relative_world(ctx, variant);
    }
    if (name == "RelativeObjectPosition") {
        return read_relative_object(ctx, variant);
    }
    if (name == "RoadPosition") {
        return read_road(ctx, variant);
    }
    if (name == "RelativeRoadPosition") {
        return read_relative_road(ctx, variant);
    }
    if (name == "LanePosition") {
        return read_lane(ctx, variant);
    }
    if (name == "RelativeLanePosition") {
        return read_relative_lane(ctx, variant);
    }
    if (name == "RoutePosition") {
        return read_route_position(ctx, variant);
    }
    if (name == "GeoPosition") {
        // Post-v0.0.1: resolving it needs the road network's geodetic datum.
        // It still lowers, so the resolver reports the rule at run time.
        return read_geo(ctx, variant);
    }
    return read_trajectory_position(ctx, variant);
}

} // namespace scena::xml::detail
