// SPDX-License-Identifier: MIT
#pragma once

#include <string>

namespace kinema::gateway {

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
};

} // namespace kinema::gateway
