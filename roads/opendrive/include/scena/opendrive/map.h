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

#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace scena::opendrive {

/// The plan-view primitive of one `<geometry>` element, per ASAM OpenDRIVE
/// 1.9.0 §9.1: line, spiral (clothoid), arc, parametric cubic, or the
/// deprecated cubic polynomial.
enum class GeometryKind {
    Line,       ///< §9.3 `<line>`.
    Spiral,     ///< §9.4 `<spiral>`, curvature linear in s.
    Arc,        ///< §9.5 `<arc>`, constant curvature.
    ParamPoly3, ///< §9.6 `<paramPoly3>`, u(p)/v(p) cubics.
    Poly3,      ///< §9.7 `<poly3>` (deprecated 1.6.0), v(u) cubic.
};

/// Interpolation-parameter range of a `<paramPoly3>` (§9.6 @pRange).
enum class PRange {
    ArcLength,  ///< p in [0, @length of the `<geometry>`].
    Normalized, ///< p in [0, 1].
};

/// One `<geometry>` element of a road's `<planView>` (§9.2): the shared
/// start-pose attributes plus the parameters of its single primitive. Only
/// the fields of the active `kind` are meaningful; the rest stay at their
/// defaults.
struct Geometry {
    GeometryKind kind = GeometryKind::Line;
    double s = 0.0;      ///< s-coordinate of the element start, meters.
    double x = 0.0;      ///< Inertial x of the element start, meters.
    double y = 0.0;      ///< Inertial y of the element start, meters.
    double hdg = 0.0;    ///< Inertial start heading, radians.
    double length = 0.0; ///< Reference-line length of the element, meters.

    // --- Arc (§9.5) ---
    double curvature = 0.0; ///< Constant curvature, 1/m; positive = left.

    // --- Spiral (§9.4) ---
    double curv_start = 0.0; ///< Curvature at the element start, 1/m.
    double curv_end = 0.0;   ///< Curvature at the element end, 1/m.

    // --- Poly3 (§9.7, deprecated): v(u) = a + b u + c u^2 + d u^3 ---
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    double d = 0.0;

    // --- ParamPoly3 (§9.6): u(p) and v(p) cubics ---
    double a_u = 0.0;
    double b_u = 0.0;
    double c_u = 0.0;
    double d_u = 0.0;
    double a_v = 0.0;
    double b_v = 0.0;
    double c_v = 0.0;
    double d_v = 0.0;
    PRange p_range = PRange::ArcLength;
};

/// One `<width>` record of a lane (§11.7.1): within its validity range the
/// width at distance `ds` past the record start is
/// `w(ds) = a + b·ds + c·ds² + d·ds³`, with `ds` restarting at zero per
/// record and the record starting at `s_section + s_offset`.
struct WidthRecord {
    double s_offset = 0.0; ///< Start s relative to the lane section, meters.
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    double d = 0.0;
};

/// One `<lane>` of a lane section (§11.2): centre lane 0, negative ids to
/// the right of the reference line, positive to the left (§11.1).
struct Lane {
    int id = 0;
    /// The `<lane>` @type string verbatim ("driving", "border", ...); the
    /// backend hands it through `IRoadQuery::lane_type` uninterpreted.
    std::string type;
    /// Width records in ascending s_offset order; empty for the centre lane
    /// (asam.net:xodr:1.4.0:road.lane.center_lane_no_width).
    std::vector<WidthRecord> widths;
    /// Lane-level linkage (§11.6): id of the connected lane in the previous /
    /// next lane section — or, at the road boundary, on the linked road.
    std::optional<int> predecessor;
    std::optional<int> successor;
};

/// One `<laneSection>` (§11.4): the road cross-section from `s` to the next
/// section (or the road end). Lanes are keyed by id in an ordered map —
/// deterministic iteration, never a hash order.
struct LaneSection {
    double s = 0.0;
    std::map<int, Lane> lanes;
};

/// One side of a road-level `<link>` (§10.3).
struct RoadLink {
    enum class Kind { Road, Junction };
    /// Contact point on the linked road (meaningful for Kind::Road).
    enum class Contact { Start, End };
    Kind kind = Kind::Road;
    std::string element_id;
    Contact contact = Contact::Start;
};

/// One `<road>` in the consumed subset: identity, the plan view, the lane
/// model and road-level linkage.
struct Road {
    std::string id;
    std::string name;
    /// Total length of the road reference line per the road's @length
    /// attribute, meters. The plan-view sum is the authoritative evaluation
    /// length; a disagreement beyond tolerance is diagnosed at load.
    double length = 0.0;
    /// `<geometry>` elements in ascending-s document order (§9.2).
    std::vector<Geometry> plan_view;
    /// `<laneSection>`s in ascending-s order; empty for a geometry-only road
    /// (lane queries then answer false).
    std::vector<LaneSection> sections;
    std::optional<RoadLink> predecessor;
    std::optional<RoadLink> successor;
    /// The junction this road belongs to as a connecting road, or empty
    /// ("-1" in the file means none and is stored as empty).
    std::string junction;
};

/// One `<laneLink>` of a junction connection (§12.4): incoming lane `from`
/// connects to connecting-road lane `to`.
struct JunctionLaneLink {
    int from = 0;
    int to = 0;
};

/// One `<connection>` of a common junction (§12.2): the path from an
/// incoming road onto a connecting road, entered at the given contact point.
struct JunctionConnection {
    std::string id;
    std::string incoming_road;
    std::string connecting_road;
    RoadLink::Contact contact = RoadLink::Contact::Start;
    std::vector<JunctionLaneLink> lane_links;
};

/// One `<junction>` of @type default (§12.2). Other junction types are
/// outside the subset and diagnosed at load.
struct Junction {
    std::string id;
    std::vector<JunctionConnection> connections;
};

/// A parsed OpenDRIVE map, restricted to the consumed subset.
///
/// Roads and junctions are keyed by id in ordered maps: iteration order is
/// part of the determinism contract (never a hash order).
struct Map {
    int rev_major = 0; ///< `<header>` @revMajor.
    int rev_minor = 0; ///< `<header>` @revMinor.
    std::map<std::string, Road> roads;
    std::map<std::string, Junction> junctions;
};

} // namespace scena::opendrive
