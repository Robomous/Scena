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
#include <vector>

#include "scena/opendrive/map.h"
#include "scena/runtime/trajectory_eval.h"

namespace scena::opendrive {

/// A point on (or relative to) a road reference line, in the inertial frame.
struct RefPose {
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0; ///< s-axis tangent direction, radians, 0 = world +x.
};

/// Result of projecting an inertial point onto a reference line: the track
/// coordinates of the closest reference-line point (OpenDRIVE 1.9.0 §8.3 —
/// t positive to the left of the s-direction).
struct TrackPosition {
    double s = 0.0;
    double t = 0.0;
};

/// Evaluates one road's reference line as a function of s, deterministically
/// and with analytic fidelity (roadmap risk R4; ASAM OpenDRIVE 1.9.0 §9).
///
/// Standalone by design, mirroring `runtime::TrajectoryEvaluator`: it
/// depends only on the parsed `Road` model, so every primitive can be tested
/// against analytic ground truth without a scenario or an engine.
///
/// Numerical methods, all through the deterministic math layer:
///  - line / arc / spiral (§9.3-§9.5) evaluate through
///    `runtime::TrajectoryEvaluator`'s clothoid path — an OpenDRIVE spiral
///    *is* the IR clothoid with kappa' = (curvEnd - curvStart) / length, an
///    arc is its kappa' = 0 closed form and a line its kappa = kappa' = 0
///    closed form — so the road backend shares one proven quadrature with
///    the trajectory follower instead of duplicating it.
///  - poly3 / paramPoly3 (§9.6-§9.7) evaluate the cubics exactly (IEEE
///    arithmetic only) and carry a fixed-resolution cumulative chord-length
///    table for the s <-> parameter mapping, linearly inverted per query;
///    tangents come from the exact derivative through `det_atan2`.
///
/// `project()` inverts pose_at approximately: fixed-stride sampling per
/// element picks the best candidate, a fixed-iteration ternary search
/// refines it, and ties keep the smallest s — a documented, deterministic
/// tie-break.
class ReferenceLine {
public:
    explicit ReferenceLine(const Road& road);

    /// False when the road has no usable plan view (empty, or an element
    /// with a non-positive length survived loading); pose_at then returns
    /// the origin and project() finds nothing.
    [[nodiscard]] bool ok() const noexcept { return ok_; }

    /// Total evaluated length: the sum of the plan-view element lengths.
    [[nodiscard]] double length() const noexcept { return total_length_; }

    /// Reference-line pose at `s`, clamped to `[0, length()]`.
    [[nodiscard]] RefPose pose_at(double s) const noexcept;

    /// Projects the inertial point (x, y) onto the reference line: the track
    /// coordinates of the nearest reference-line point. Returns nullopt for
    /// non-finite input or an unusable plan view. Ties between equally near
    /// candidates keep the smallest s.
    [[nodiscard]] std::optional<TrackPosition> project(double x, double y) const noexcept;

private:
    struct PolyTable {
        /// Parameter value at each node (u for poly3, p for paramPoly3).
        std::vector<double> param;
        /// Cumulative chord arc length at each node; front() == 0.
        std::vector<double> arc;
    };

    struct Element {
        Geometry geom;
        /// Line / arc / spiral path (kappa-linear family).
        std::optional<runtime::TrajectoryEvaluator> clothoid;
        /// poly3 / paramPoly3 s->parameter table.
        PolyTable table;
        double start_s = 0.0; ///< Cumulative s at the element start.
    };

    [[nodiscard]] RefPose element_pose(const Element& element, double local_s) const noexcept;
    [[nodiscard]] static RefPose poly_pose(const Element& element, double local_s) noexcept;

    bool ok_ = false;
    double total_length_ = 0.0;
    std::vector<Element> elements_;
};

} // namespace scena::opendrive
