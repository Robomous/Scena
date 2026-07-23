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

namespace scena::ir {

/// The referential a distance is measured in, per ASAM OpenSCENARIO XML 1.4.0
/// CoordinateSystem (§6.4). In a condition it relates to the triggering
/// entity. `Entity` is the default. `Lane`, `Road`, and `Trajectory` need a
/// road network (IRoadQuery, p3-s4) and evaluate to a deterministic false with
/// an init-time UnsupportedFeature warning until then (ADR-0009).
enum class CoordinateSystem {
    Entity,
    Lane,
    Road,
    Trajectory,
    World,
};

/// Which dimension(s) a distance uses, per RelativeDistanceType (§6.4).
/// `Longitudinal`/`Lateral` project onto one axis of the effective coordinate
/// system; `EuclidianDistance` (the default) is the coordinate-system-
/// independent 3D magnitude (§6.4.3). `CartesianDistance` is deprecated and is
/// treated as EuclidianDistance with a warning.
enum class RelativeDistanceType {
    Longitudinal,
    Lateral,
    CartesianDistance, ///< Deprecated; treated as EuclidianDistance.
    EuclidianDistance,
};

/// Route-selection algorithm for road/lane distances, per RoutingAlgorithm
/// (§6.4.8.3). Only relevant to the road/lane coordinate systems, which are
/// deferred this phase, so it is stored and exposed but never changes an
/// evaluation (default `Undefined` makes silent-ignore spec-conforming).
enum class RoutingAlgorithm {
    AssignedRoute,
    Fastest,
    LeastIntersections,
    Shortest,
    Undefined,
};

} // namespace scena::ir
