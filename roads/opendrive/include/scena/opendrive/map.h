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

/// One `<road>` restricted to what p3-s2 consumes: identity and the plan
/// view. Lanes, links and junctions join the model in p3-s3.
struct Road {
    std::string id;
    std::string name;
    /// Total length of the road reference line per the road's @length
    /// attribute, meters. The plan-view sum is the authoritative evaluation
    /// length; a disagreement beyond tolerance is diagnosed at load.
    double length = 0.0;
    /// `<geometry>` elements in ascending-s document order (§9.2).
    std::vector<Geometry> plan_view;
};

/// A parsed OpenDRIVE map, restricted to the p3-s2 subset.
///
/// Roads are keyed by id in an ordered map: iteration order is part of the
/// determinism contract (never a hash order).
struct Map {
    int rev_major = 0; ///< `<header>` @revMajor.
    int rev_minor = 0; ///< `<header>` @revMinor.
    std::map<std::string, Road> roads;
};

} // namespace scena::opendrive
