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

#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "scena/ir/position.h"

namespace scena::ir {

/// Timing adjustment applied to the time values of a trajectory, per §Timing.
/// The effective time of a vertex is `time * scale + offset`, read in the
/// `domain` context.
struct Timing {
    ReferenceContext domain = ReferenceContext::Absolute;
    /// Scaling factor for time values; 1.0 means no scaling. Range ]0..inf[.
    double scale = 1.0;
    /// Global offset added to all time values. Unit: [s]. Range ]-inf..inf[.
    double offset = 0.0;
};

/// One vertex of a polyline trajectory, per §Vertex: a position and an optional
/// time. The §Motion element (speed/acceleration at the vertex, added in 1.4)
/// is excluded from v0.0.1 together with the polyline §Interpolation element.
struct TrajectoryVertex {
    WorldPosition position;
    /// Optional time specification of the vertex. Unit: [s]. Required on every
    /// vertex when the action uses a Timing time reference
    /// (asam.net:xosc:1.0.0:routing.trajectory_timing_exists_if_requested).
    std::optional<double> time;
};

/// The `Polyline` shape (§Polyline): a concatenation of straight segments
/// across an ordered chain of at least two vertices.
struct Polyline {
    std::vector<TrajectoryVertex> vertices; ///< At least two, in document order.
};

/// The `Clothoid` shape (§Clothoid, §6.9): an Euler spiral whose curvature
/// changes linearly with arc length, `kappa(s) = curvature + curvature_prime *
/// s`. Special cases per the standard: `curvature_prime == 0` is a circular
/// arc; `curvature_prime == 0 && curvature == 0` is a straight line.
struct Clothoid {
    /// Start pose of the spiral; its heading is the initial tangent theta0.
    WorldPosition start;
    /// Curvature at the start, kappa0. Unit: [1/m].
    double curvature = 0.0;
    /// Rate of curvature change, kappa' = d(kappa)/ds. Unit: [1/m^2]. This is
    /// the 1.4 `curvaturePrime`; the 1.0-1.3 `curvatureDot` is its deprecated
    /// alias and lowers to the same field.
    double curvature_prime = 0.0;
    /// Arc length of the spiral. Unit: [m]. Range [0..inf[.
    double length = 0.0;
    /// Optional trajectory-time of the start point. Unit: [s].
    std::optional<double> start_time;
    /// Optional trajectory-time of the end point. Unit: [s].
    std::optional<double> stop_time;
};

/// One control point of a NURBS trajectory, per §ControlPoint.
struct ControlPoint {
    WorldPosition position;
    /// Optional trajectory-time at this control point. Unit: [s].
    std::optional<double> time;
    /// Homogeneous weight; 1.0 for a non-rational point. Range ]0..inf[.
    double weight = 1.0;
};

/// The `Nurbs` shape (§Nurbs, §6.9): a Non-Uniform Rational B-Spline of the
/// given `order` (order = degree + 1, range [2..inf[). Cardinality per
/// `asam.net:xosc:1.0.0:routing.cardinality_of_control_points_in_nurbs`:
/// `control_points.size() >= order` and `knots.size() == control_points.size()
/// + order`, with `knots` in ascending order.
struct Nurbs {
    unsigned int order = 2;
    std::vector<ControlPoint> control_points;
    std::vector<double> knots;
};

/// The geometry of a trajectory (§Shape, an xsd:choice). ClothoidSpline is
/// excluded from v0.0.1 (a single Clothoid segment covers the need); see the
/// coverage matrix.
using Shape = std::variant<Polyline, Clothoid, Nurbs>;

/// An intended path for entity motion, per §Trajectory (§6.9).
struct Trajectory {
    std::string name;
    /// If true the end of the trajectory connects back to its start and the
    /// FollowTrajectoryAction "doesn't end regularly but has to be stopped"
    /// (§Trajectory). Stored but not executed: the runtime reports an
    /// UnsupportedFeature warning and follows the open path.
    bool closed = false;
    /// The trajectory geometry; defaults to an empty polyline.
    Shape shape = Polyline{};

    Trajectory() = default;
    Trajectory(std::string name_in, bool closed_in, Shape shape_in)
        : name(std::move(name_in)), closed(closed_in), shape(std::move(shape_in)) {}
    /// Convenience for the common polyline case, keeping `Trajectory{name,
    /// closed, vertices}` construction concise.
    Trajectory(std::string name_in, bool closed_in, std::vector<TrajectoryVertex> vertices_in)
        : name(std::move(name_in)), closed(closed_in), shape(Polyline{std::move(vertices_in)}) {}

    /// Mutable polyline vertices. Precondition: the trajectory holds a Polyline
    /// (the default). Provided for the many polyline construction sites.
    [[nodiscard]] std::vector<TrajectoryVertex>& vertices() {
        return std::get<Polyline>(shape).vertices;
    }
    [[nodiscard]] const std::vector<TrajectoryVertex>& vertices() const {
        return std::get<Polyline>(shape).vertices;
    }
};

} // namespace scena::ir
