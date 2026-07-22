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
#include <vector>

#include "scena/ir/position.h"

namespace scena::ir {

/// Whether a value is given in absolute or relative terms, per ASAM
/// OpenSCENARIO XML 1.4.0 §ReferenceContext. On a trajectory Timing it selects
/// the origin of the vertex time values: simulation time zero (absolute) or the
/// instant the FollowTrajectoryAction started (relative).
enum class ReferenceContext {
    Absolute,
    Relative,
};

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
/// is deferred to p2-s5 together with the non-polyline shapes.
struct TrajectoryVertex {
    WorldPosition position;
    /// Optional time specification of the vertex. Unit: [s]. Required on every
    /// vertex when the action uses a Timing time reference
    /// (asam.net:xosc:1.0.0:routing.trajectory_timing_exists_if_requested).
    std::optional<double> time;
};

/// An intended path for entity motion, per §Trajectory (§6.9).
///
/// Scena models the `Polyline` shape (§Polyline: a concatenation of line
/// segments across an ordered chain of at least two vertices). The `Clothoid`,
/// `ClothoidSpline` and `NURBS` shapes, the §Interpolation element and
/// trajectory catalogs are deferred to p2-s5; a scenario using them is rejected
/// by the frontend rather than silently reinterpreted here.
struct Trajectory {
    std::string name;
    /// If true the end of the trajectory connects back to its start and the
    /// FollowTrajectoryAction "doesn't end regularly but has to be stopped"
    /// (§Trajectory). Stored but not yet executed: the runtime reports an
    /// UnsupportedFeature warning and follows the open path (p2-s5).
    bool closed = false;
    std::vector<TrajectoryVertex> vertices; ///< At least two, in document order.
};

} // namespace scena::ir
