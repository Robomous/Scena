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

#include "scena/opendrive/reader.h"

#include <cmath>
#include <cstddef>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>

#include <pugixml.hpp>

#include "scena/ir/rule.h" // ir::parse_scalar: locale-independent from_chars

namespace scena::opendrive {

namespace {

/// Tolerance for the ascending-s / s-sum bookkeeping checks (meters). The
/// spec states exact equalities; real exports carry rounding noise, so exact
/// comparison would reject virtually every file.
constexpr double kSTolerance = 1e-3;

struct ReadContext {
    DiagnosticSink& sink;
    std::string file; ///< Diagnostic source file; empty for in-memory input.
    Status first_error = Status::Ok;

    void report(Severity severity, Status code, std::string path, std::string message,
                std::string rule_id = {}) {
        if (severity == Severity::Error && first_error == Status::Ok) {
            first_error = code;
        }
        Diagnostic diagnostic;
        diagnostic.severity = severity;
        diagnostic.code = code;
        diagnostic.message = std::move(message);
        diagnostic.path = std::move(path);
        diagnostic.location.file = file;
        diagnostic.rule_id = std::move(rule_id);
        sink.report(std::move(diagnostic));
    }
};

/// Required double attribute: reports a ParseError when missing or not a
/// scalar. Uses ir::parse_scalar (std::from_chars), never a locale-dependent
/// conversion.
bool require_double(ReadContext& ctx, const pugi::xml_node& node, const char* name,
                    const std::string& path, double& out) {
    const pugi::xml_attribute attr = node.attribute(name);
    if (!attr) {
        ctx.report(Severity::Error, Status::ParseError, path,
                   std::string("missing required attribute '") + name + "'");
        return false;
    }
    const std::optional<double> value = ir::parse_scalar(attr.value());
    if (!value.has_value() || !std::isfinite(*value)) {
        ctx.report(Severity::Error, Status::ParseError, path,
                   std::string("attribute '") + name + "' is not a finite number");
        return false;
    }
    out = *value;
    return true;
}

int parse_int_or(const pugi::xml_attribute& attr, int fallback) {
    if (!attr) {
        return fallback;
    }
    const std::optional<double> value = ir::parse_scalar(attr.value());
    if (!value.has_value()) {
        return fallback;
    }
    return static_cast<int>(*value);
}

/// Reports every child element outside the consumed subset — the
/// never-silent rule for map features. `consumed` is a null-terminated array
/// of element names this reader understands at this level.
void warn_unconsumed_children(ReadContext& ctx, const pugi::xml_node& node, const std::string& path,
                              const char* const consumed[]) {
    for (pugi::xml_node child : node.children()) {
        if (child.type() != pugi::node_element) {
            continue;
        }
        bool known = false;
        for (const char* const* name = consumed; *name != nullptr; ++name) {
            if (child.name() == std::string_view(*name)) {
                known = true;
                break;
            }
        }
        if (!known) {
            ctx.report(Severity::Warning, Status::UnsupportedFeature, path,
                       std::string("element '") + child.name() +
                           "' is outside the consumed OpenDRIVE subset and is ignored");
        }
    }
}

bool read_geometry(ReadContext& ctx, const pugi::xml_node& node, const std::string& path,
                   Geometry& out) {
    bool ok = true;
    ok = require_double(ctx, node, "s", path, out.s) && ok;
    ok = require_double(ctx, node, "x", path, out.x) && ok;
    ok = require_double(ctx, node, "y", path, out.y) && ok;
    ok = require_double(ctx, node, "hdg", path, out.hdg) && ok;
    ok = require_double(ctx, node, "length", path, out.length) && ok;
    if (ok && out.length <= 0.0) {
        // @length is t_grZero (> 0) per §9.2 Table 18.
        ctx.report(Severity::Error, Status::ValidationError, path,
                   "geometry length must be greater than zero");
        ok = false;
    }

    // Exactly one primitive child per element
    // (asam.net:xodr:1.4.0:road.geometry.one_geom_elem_per_spec).
    int primitives = 0;
    pugi::xml_node primitive;
    for (pugi::xml_node child : node.children()) {
        if (child.type() != pugi::node_element) {
            continue;
        }
        ++primitives;
        primitive = child;
    }
    if (primitives != 1) {
        ctx.report(Severity::Error, Status::ValidationError, path,
                   "geometry must contain exactly one primitive element",
                   "asam.net:xodr:1.4.0:road.geometry.one_geom_elem_per_spec");
        return false;
    }

    const std::string_view name = primitive.name();
    if (name == "line") {
        out.kind = GeometryKind::Line;
        return ok;
    }
    if (name == "arc") {
        out.kind = GeometryKind::Arc;
        return require_double(ctx, primitive, "curvature", path, out.curvature) && ok;
    }
    if (name == "spiral") {
        out.kind = GeometryKind::Spiral;
        bool spiral_ok = require_double(ctx, primitive, "curvStart", path, out.curv_start);
        spiral_ok = require_double(ctx, primitive, "curvEnd", path, out.curv_end) && spiral_ok;
        return spiral_ok && ok;
    }
    if (name == "poly3") {
        // Deprecated since OpenDRIVE 1.6.0 (§9.7); still evaluated.
        out.kind = GeometryKind::Poly3;
        ctx.report(Severity::Warning, Status::DeprecatedFeature, path,
                   "element 'poly3' is deprecated since OpenDRIVE 1.6.0");
        bool poly_ok = require_double(ctx, primitive, "a", path, out.a);
        poly_ok = require_double(ctx, primitive, "b", path, out.b) && poly_ok;
        poly_ok = require_double(ctx, primitive, "c", path, out.c) && poly_ok;
        poly_ok = require_double(ctx, primitive, "d", path, out.d) && poly_ok;
        return poly_ok && ok;
    }
    if (name == "paramPoly3") {
        out.kind = GeometryKind::ParamPoly3;
        bool pp_ok = require_double(ctx, primitive, "aU", path, out.a_u);
        pp_ok = require_double(ctx, primitive, "bU", path, out.b_u) && pp_ok;
        pp_ok = require_double(ctx, primitive, "cU", path, out.c_u) && pp_ok;
        pp_ok = require_double(ctx, primitive, "dU", path, out.d_u) && pp_ok;
        pp_ok = require_double(ctx, primitive, "aV", path, out.a_v) && pp_ok;
        pp_ok = require_double(ctx, primitive, "bV", path, out.b_v) && pp_ok;
        pp_ok = require_double(ctx, primitive, "cV", path, out.c_v) && pp_ok;
        pp_ok = require_double(ctx, primitive, "dV", path, out.d_v) && pp_ok;
        const std::string_view p_range = primitive.attribute("pRange").value();
        if (p_range == "arcLength") {
            out.p_range = PRange::ArcLength;
        } else if (p_range == "normalized") {
            out.p_range = PRange::Normalized;
        } else {
            ctx.report(Severity::Error, Status::ParseError, path,
                       "attribute 'pRange' must be 'arcLength' or 'normalized'");
            pp_ok = false;
        }
        return pp_ok && ok;
    }
    ctx.report(Severity::Error, Status::ValidationError, path,
               std::string("unknown geometry primitive '") + primitive.name() + "'");
    return false;
}

std::optional<int> parse_lane_ref(ReadContext& ctx, const pugi::xml_node& link, const char* element,
                                  const std::string& path) {
    // Multiple predecessors/successors serve the temporary lane layer
    // (§11.6), which is outside the subset: keep the first, warn on the rest.
    std::optional<int> id;
    for (pugi::xml_node ref : link.children(element)) {
        const std::optional<double> value = ir::parse_scalar(ref.attribute("id").value());
        if (!value.has_value()) {
            ctx.report(Severity::Error, Status::ParseError, path,
                       std::string("lane ") + element + " has no numeric 'id'");
            continue;
        }
        if (id.has_value()) {
            ctx.report(Severity::Warning, Status::UnsupportedFeature, path,
                       std::string("multiple lane ") + element +
                           " links (temporary lane layer) are outside the subset; "
                           "keeping the first");
            continue;
        }
        id = static_cast<int>(*value);
    }
    return id;
}

/// Reads one `<left>`/`<center>`/`<right>` group's lanes into `section`.
void read_lane_group(ReadContext& ctx, const pugi::xml_node& group, const std::string& path,
                     LaneSection& section) {
    for (pugi::xml_node lane_node : group.children("lane")) {
        Lane lane;
        const std::optional<double> id = ir::parse_scalar(lane_node.attribute("id").value());
        if (!id.has_value()) {
            ctx.report(Severity::Error, Status::ParseError, path,
                       "lane has no numeric 'id' attribute");
            continue;
        }
        lane.id = static_cast<int>(*id);
        lane.type = lane_node.attribute("type").value();
        const std::string lane_path = path + "/lane[" + std::to_string(lane.id) + "]";

        static const char* const kConsumedLaneChildren[] = {"link", "width", nullptr};
        warn_unconsumed_children(ctx, lane_node, lane_path, kConsumedLaneChildren);

        const pugi::xml_node link = lane_node.child("link");
        if (link) {
            lane.predecessor = parse_lane_ref(ctx, link, "predecessor", lane_path);
            lane.successor = parse_lane_ref(ctx, link, "successor", lane_path);
        }

        double last_offset = -1.0;
        for (pugi::xml_node width : lane_node.children("width")) {
            WidthRecord record;
            bool ok = require_double(ctx, width, "sOffset", lane_path, record.s_offset);
            ok = require_double(ctx, width, "a", lane_path, record.a) && ok;
            ok = require_double(ctx, width, "b", lane_path, record.b) && ok;
            ok = require_double(ctx, width, "c", lane_path, record.c) && ok;
            ok = require_double(ctx, width, "d", lane_path, record.d) && ok;
            if (!ok) {
                continue;
            }
            if (record.s_offset <= last_offset) {
                ctx.report(Severity::Error, Status::ValidationError, lane_path,
                           "width records must be in ascending sOffset order",
                           "asam.net:xodr:1.4.0:road.lane.width.elem_asc_order");
                continue;
            }
            last_offset = record.s_offset;
            lane.widths.push_back(record);
        }
        if (lane.id == 0 && !lane.widths.empty()) {
            ctx.report(Severity::Error, Status::ValidationError, lane_path,
                       "the centre lane must not carry width records",
                       "asam.net:xodr:1.4.0:road.lane.center_lane_no_width");
            continue;
        }
        if (lane.id != 0 && (lane.widths.empty() || lane.widths.front().s_offset > kSTolerance)) {
            ctx.report(Severity::Error, Status::ValidationError, lane_path,
                       "lane width must be defined from the start of the lane section",
                       "asam.net:xodr:1.7.0:road.lane.width.width_defined_whole_section");
            continue;
        }
        section.lanes[lane.id] = std::move(lane);
    }
}

void read_lanes(ReadContext& ctx, const pugi::xml_node& lanes, const std::string& road_path,
                Road& road) {
    static const char* const kConsumedLanesChildren[] = {"laneSection", nullptr};
    warn_unconsumed_children(ctx, lanes, road_path + "/lanes", kConsumedLanesChildren);

    std::size_t index = 0;
    for (pugi::xml_node section_node : lanes.children("laneSection")) {
        const std::string path = road_path + "/lanes/laneSection[" + std::to_string(index) + "]";
        ++index;
        LaneSection section;
        if (!require_double(ctx, section_node, "s", path, section.s)) {
            continue;
        }
        if (!road.sections.empty() && section.s <= road.sections.back().s) {
            ctx.report(Severity::Error, Status::ValidationError, path,
                       "lane sections must be in ascending s order",
                       "asam.net:xodr:1.4.0:road.lane_section.elem_asc_order");
            continue;
        }
        static const char* const kConsumedSectionChildren[] = {"left", "center", "right", nullptr};
        warn_unconsumed_children(ctx, section_node, path, kConsumedSectionChildren);
        for (const char* const group : {"left", "center", "right"}) {
            const pugi::xml_node group_node = section_node.child(group);
            if (group_node) {
                read_lane_group(ctx, group_node, path, section);
            }
        }
        road.sections.push_back(std::move(section));
    }
    if (road.sections.empty()) {
        ctx.report(Severity::Error, Status::ValidationError, road_path + "/lanes",
                   "lanes element contains no lane sections",
                   "asam.net:xodr:1.4.0:road.lane.lane_sect_min_amount");
        return;
    }
    if (road.sections.front().s > kSTolerance) {
        ctx.report(Severity::Error, Status::ValidationError, road_path + "/lanes",
                   "the first lane section must start at s = 0");
        road.sections.clear();
    }
}

std::optional<RoadLink> read_road_link_side(ReadContext& ctx, const pugi::xml_node& side,
                                            const std::string& path) {
    if (!side) {
        return std::nullopt;
    }
    RoadLink link;
    const std::string_view type = side.attribute("elementType").value();
    if (type == "road") {
        link.kind = RoadLink::Kind::Road;
    } else if (type == "junction") {
        link.kind = RoadLink::Kind::Junction;
    } else {
        ctx.report(Severity::Error, Status::ValidationError, path,
                   "road link elementType must be 'road' or 'junction'",
                   "asam.net:xodr:1.4.0:road.linkage.road_link_attribute_usage");
        return std::nullopt;
    }
    link.element_id = side.attribute("elementId").value();
    if (link.element_id.empty()) {
        ctx.report(Severity::Error, Status::ValidationError, path, "road link has no elementId",
                   "asam.net:xodr:1.4.0:road.linkage.road_link_attribute_usage");
        return std::nullopt;
    }
    const std::string_view contact = side.attribute("contactPoint").value();
    if (contact == "start") {
        link.contact = RoadLink::Contact::Start;
    } else if (contact == "end") {
        link.contact = RoadLink::Contact::End;
    } else if (link.kind == RoadLink::Kind::Road) {
        ctx.report(Severity::Error, Status::ValidationError, path,
                   "a road-to-road link needs contactPoint 'start' or 'end'",
                   "asam.net:xodr:1.4.0:road.linkage.road_link_attribute_usage");
        return std::nullopt;
    }
    return link;
}

void read_junction(ReadContext& ctx, const pugi::xml_node& node, Map& out) {
    Junction junction;
    junction.id = node.attribute("id").value();
    const std::string path = "junctions/" + junction.id;
    if (junction.id.empty()) {
        ctx.report(Severity::Error, Status::ParseError, "junctions",
                   "junction is missing the required 'id' attribute");
        return;
    }
    // Only common junctions (@type default) are in the subset (§12.2);
    // direct/virtual/crossing junctions are diagnosed, never silent.
    const std::string_view type = node.attribute("type").value();
    if (!type.empty() && type != "default") {
        ctx.report(Severity::Warning, Status::UnsupportedFeature, path,
                   "only junctions of type 'default' are in the consumed subset; "
                   "this junction is ignored");
        return;
    }
    static const char* const kConsumedJunctionChildren[] = {"connection", nullptr};
    warn_unconsumed_children(ctx, node, path, kConsumedJunctionChildren);

    for (pugi::xml_node connection_node : node.children("connection")) {
        JunctionConnection connection;
        connection.id = connection_node.attribute("id").value();
        connection.incoming_road = connection_node.attribute("incomingRoad").value();
        connection.connecting_road = connection_node.attribute("connectingRoad").value();
        if (connection.incoming_road.empty() || connection.connecting_road.empty()) {
            ctx.report(Severity::Error, Status::ValidationError, path,
                       "junction connection needs incomingRoad and connectingRoad");
            continue;
        }
        const std::string_view contact = connection_node.attribute("contactPoint").value();
        connection.contact = contact == "end" ? RoadLink::Contact::End : RoadLink::Contact::Start;
        for (pugi::xml_node lane_link : connection_node.children("laneLink")) {
            const std::optional<double> from =
                ir::parse_scalar(lane_link.attribute("from").value());
            const std::optional<double> to = ir::parse_scalar(lane_link.attribute("to").value());
            if (!from.has_value() || !to.has_value()) {
                ctx.report(Severity::Error, Status::ParseError, path,
                           "laneLink needs numeric 'from' and 'to'");
                continue;
            }
            connection.lane_links.push_back({static_cast<int>(*from), static_cast<int>(*to)});
        }
        junction.connections.push_back(std::move(connection));
    }
    if (out.junctions.count(junction.id) != 0) {
        ctx.report(Severity::Error, Status::ValidationError, path, "duplicate junction id");
        return;
    }
    out.junctions.emplace(junction.id, std::move(junction));
}

void read_road(ReadContext& ctx, const pugi::xml_node& node, std::size_t index, Map& out) {
    Road road;
    road.id = node.attribute("id").value();
    road.name = node.attribute("name").value();
    const std::string road_path =
        road.id.empty() ? "roads[" + std::to_string(index) + "]" : "roads/" + road.id;
    if (road.id.empty()) {
        ctx.report(Severity::Error, Status::ParseError, road_path,
                   "road is missing the required 'id' attribute");
        return;
    }
    if (!require_double(ctx, node, "length", road_path, road.length)) {
        return;
    }

    road.junction = node.attribute("junction").value();
    if (road.junction == "-1") {
        road.junction.clear(); // "-1" means none (§12.2 Table 55).
    }

    static const char* const kConsumedRoadChildren[] = {"planView", "lanes", "link", nullptr};
    warn_unconsumed_children(ctx, node, road_path, kConsumedRoadChildren);

    const pugi::xml_node link = node.child("link");
    if (link) {
        road.predecessor = read_road_link_side(ctx, link.child("predecessor"), road_path + "/link");
        road.successor = read_road_link_side(ctx, link.child("successor"), road_path + "/link");
    }

    const pugi::xml_node lanes = node.child("lanes");
    if (lanes) {
        read_lanes(ctx, lanes, road_path, road);
    }

    const pugi::xml_node plan_view = node.child("planView");
    if (!plan_view) {
        ctx.report(Severity::Error, Status::ValidationError, road_path,
                   "road has no planView reference line",
                   "asam.net:xodr:1.4.0:road.geometry.refline_exists");
        return;
    }

    double expected_s = 0.0;
    std::size_t geometry_index = 0;
    bool geometry_ok = true;
    for (pugi::xml_node child : plan_view.children("geometry")) {
        const std::string path =
            road_path + "/planView/geometry[" + std::to_string(geometry_index) + "]";
        Geometry geometry;
        if (!read_geometry(ctx, child, path, geometry)) {
            geometry_ok = false;
            ++geometry_index;
            continue;
        }
        if (!road.plan_view.empty() && geometry.s + kSTolerance < road.plan_view.back().s) {
            ctx.report(Severity::Error, Status::ValidationError, path,
                       "geometry elements must be in ascending s order",
                       "asam.net:xodr:1.4.0:road.geometry.elem_asc_order");
            geometry_ok = false;
        }
        if (std::fabs(geometry.s - expected_s) > kSTolerance) {
            ctx.report(Severity::Error, Status::ValidationError, path,
                       "geometry s must equal the sum of the preceding geometry lengths",
                       "asam.net:xodr:1.9.0:road.geometry.s-value_sum");
            geometry_ok = false;
        }
        expected_s += geometry.length;
        road.plan_view.push_back(geometry);
        ++geometry_index;
    }
    if (road.plan_view.empty()) {
        ctx.report(Severity::Error, Status::ValidationError, road_path + "/planView",
                   "planView contains no geometry elements",
                   "asam.net:xodr:1.4.0:road.geometry.refline_exists");
        return;
    }
    if (!geometry_ok) {
        return;
    }

    // The plan-view sum is the authoritative evaluated length; flag a road
    // @length that disagrees, but keep the road (the operation succeeds).
    if (std::fabs(expected_s - road.length) > kSTolerance) {
        ctx.report(Severity::Warning, Status::ValidationError, road_path,
                   "road length attribute disagrees with the plan view total; "
                   "the plan view total is used");
    }

    if (out.roads.count(road.id) != 0) {
        ctx.report(Severity::Error, Status::ValidationError, road_path, "duplicate road id");
        return;
    }
    out.roads.emplace(road.id, std::move(road));
}

} // namespace

Status load_string(std::string_view xml, Map& out, DiagnosticSink& sink) {
    ReadContext ctx{sink, {}};
    out = Map{};

    pugi::xml_document document;
    const pugi::xml_parse_result parsed = document.load_buffer(xml.data(), xml.size());
    if (!parsed) {
        // pugixml gives a byte offset; report the 1-based line for anchoring.
        int line = 1;
        const std::size_t offset = std::min(static_cast<std::size_t>(parsed.offset), xml.size());
        for (std::size_t i = 0; i < offset; ++i) {
            if (xml[i] == '\n') {
                ++line;
            }
        }
        Diagnostic diagnostic;
        diagnostic.severity = Severity::Error;
        diagnostic.code = Status::ParseError;
        diagnostic.message = std::string("not well-formed xml: ") + parsed.description();
        diagnostic.location.line = line;
        sink.report(std::move(diagnostic));
        return Status::ParseError;
    }

    const pugi::xml_node root = document.child("OpenDRIVE");
    if (!root) {
        ctx.report(Severity::Error, Status::ParseError, {},
                   "document has no OpenDRIVE root element");
        return Status::ParseError;
    }

    const pugi::xml_node header = root.child("header");
    if (header) {
        out.rev_major = parse_int_or(header.attribute("revMajor"), 0);
        out.rev_minor = parse_int_or(header.attribute("revMinor"), 0);
        if (out.rev_major != 1) {
            ctx.report(Severity::Warning, Status::UnsupportedFeature, "header",
                       "unrecognized OpenDRIVE major revision; reading as 1.x");
        }
        if (header.child("geoReference")) {
            // Geo-referencing is scoped out for v0.0.1 (roadmap P3 scope).
            ctx.report(Severity::Warning, Status::UnsupportedFeature, "header",
                       "geoReference is not consumed; coordinates stay in the "
                       "local inertial frame");
        }
    } else {
        ctx.report(Severity::Warning, Status::UnsupportedFeature, {},
                   "document has no header element; revision is unknown");
    }

    static const char* const kConsumedRootChildren[] = {"header", "road", "junction", nullptr};
    warn_unconsumed_children(ctx, root, {}, kConsumedRootChildren);

    std::size_t index = 0;
    for (pugi::xml_node road : root.children("road")) {
        read_road(ctx, road, index, out);
        ++index;
    }
    if (index == 0) {
        ctx.report(Severity::Error, Status::ValidationError, {},
                   "document contains no road elements");
    }

    for (pugi::xml_node junction : root.children("junction")) {
        read_junction(ctx, junction, out);
    }

    return ctx.first_error;
}

Status load_file(const std::filesystem::path& path, Map& out, DiagnosticSink& sink) {
    // Binary mode: newline translation must not alter byte offsets or
    // checksums across platforms (cross-platform rule).
    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
        Diagnostic diagnostic;
        diagnostic.severity = Severity::Error;
        diagnostic.code = Status::InvalidArgument;
        diagnostic.message = "cannot open file";
        diagnostic.location.file = path.string();
        sink.report(std::move(diagnostic));
        return Status::InvalidArgument;
    }
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    const std::string content = buffer.str();

    // Parse into a local sink, stamp the file path onto each finding, then
    // forward in order — the caller's sink stays append-only.
    DiagnosticSink local;
    const Status status = load_string(content, out, local);
    for (const Diagnostic& entry : local.diagnostics()) {
        Diagnostic stamped = entry;
        stamped.location.file = path.string();
        sink.report(std::move(stamped));
    }
    return status;
}

} // namespace scena::opendrive
