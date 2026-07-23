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

#include <string>

namespace scena::gateway {

/// Lane-relative position in road coordinates, following the ASAM OpenDRIVE
/// s/t convention: `s` runs along the road reference line, `t` is the signed
/// lateral offset from it, and `lane_id` selects the lane relative to the
/// reference line.
struct LanePosition {
    std::string road_id;
    int lane_id = 0;
    double s = 0.0; ///< Longitudinal position along the road reference line, meters.
    double t = 0.0; ///< Signed lateral offset from the reference line, meters.
};

/// Abstract road-network query interface.
///
/// The runtime needs lane-relative positioning to evaluate road-based
/// conditions and actions, but it must not depend on any concrete road-network
/// representation. The host simulator (or a future OpenDRIVE module) provides
/// an implementation through the simulator gateway. No implementation exists
/// in this phase; the interface only fixes the abstraction boundary.
class IRoadQuery {
public:
    virtual ~IRoadQuery() = default;

    /// Maps a world-frame position to lane-relative road coordinates.
    /// Returns false when the position is not on any known road.
    [[nodiscard]] virtual bool to_lane_position(double x, double y, double z,
                                                LanePosition& out) const = 0;

    /// Maps lane-relative road coordinates back to a world-frame position.
    /// Returns false when the coordinates do not identify a valid location.
    [[nodiscard]] virtual bool to_world_position(const LanePosition& position, double& x, double& y,
                                                 double& z) const = 0;

    // --- Lane queries (p2-s3 forward-pull) ---------------------------------
    //
    // The three queries below are what LaneChangeAction needs to resolve a
    // target lane against a real road network. They are defaulted rather than
    // pure so that they can land ahead of the frozen v1 interface (p3-s1, #20),
    // which still owns the shape of this abstraction: an existing host
    // implementation keeps compiling, and one that does not answer them makes
    // the engine fall back to its flat-world lane model (ADR-0016).
    //
    // "Unsupported" and "no such lane" are deliberately the same answer —
    // false — because the caller does the same thing either way.

    /// Width of lane `lane_id` of road `road_id` at `s`, in metres. Returns
    /// false when the lane is unknown or widths are not available.
    [[nodiscard]] virtual bool lane_width(const std::string& road_id, int lane_id, double s,
                                          double& out_width) const {
        (void)road_id;
        (void)lane_id;
        (void)s;
        (void)out_width;
        return false;
    }

    /// Signed t-coordinate of the centre line of lane `lane_id` of road
    /// `road_id` at `s`, on the road t-axis, which points left (§6.3.2).
    /// Returns false when the lane is unknown.
    [[nodiscard]] virtual bool lane_center_offset(const std::string& road_id, int lane_id, double s,
                                                  double& out_t) const {
        (void)road_id;
        (void)lane_id;
        (void)s;
        (void)out_t;
        return false;
    }

    /// The lane `count` lanes away from lane `lane_id` of road `road_id`,
    /// counting positive towards +t (left) and skipping the road centre lane,
    /// which "is not counted as a lane and thus omitted" (§7.4.1.4). Returns
    /// false when there is no such lane.
    [[nodiscard]] virtual bool relative_lane(const std::string& road_id, int lane_id, int count,
                                             int& out_lane_id) const {
        (void)road_id;
        (void)lane_id;
        (void)count;
        (void)out_lane_id;
        return false;
    }
};

} // namespace scena::gateway
