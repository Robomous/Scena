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

// Executable contract for gateway::IRoadQuery backends (p3-s1).
//
// Every backend must satisfy these invariants; they encode the frozen v1
// unsupported-reporting semantics of road_query.h, so a backend that passes
// can be swapped behind the engine without changing runtime behavior rules.
// Instantiate per backend with a Traits type providing:
//
//     static std::unique_ptr<scena::gateway::IRoadQuery> make();
//
// and register it:
//
//     using MyBackends = ::testing::Types<MyTraits>;
//     INSTANTIATE_TYPED_TEST_SUITE_P(MyBackend, RoadQueryContractTest, MyBackends);
//
// The suite runs against FlatWorldRoadQuery now (road_query_contract_test.cpp)
// and the OpenDRIVE backend from p3-s3 on. The invariants are implications
// ("if the backend answers, the answer is well-formed"), so an all-false
// backend passes vacuously where it has nothing to say and is pinned exactly
// where it must stay silent.

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "scena/gateway/road_query.h"

namespace scena::testsupport {

/// Deterministic world-frame probe grid: a 5x5 grid on z=0 spanning
/// [-50, 50] in x and y, plus one elevated point. Fixed literals, no
/// randomness — the determinism contract applies to tests too.
inline std::vector<std::array<double, 3>> road_probe_points() {
    std::vector<std::array<double, 3>> points;
    for (int ix = -2; ix <= 2; ++ix) {
        for (int iy = -2; iy <= 2; ++iy) {
            points.push_back({25.0 * ix, 25.0 * iy, 0.0});
        }
    }
    points.push_back({0.0, 0.0, 5.0});
    return points;
}

/// Synthetic (road_id, lane_id, s) triples probed in addition to whatever
/// to_lane_position resolves: ids a backend may or may not know, never ids
/// that crash it.
inline std::vector<std::pair<std::string, int>> road_lane_probes() {
    return {{"0", -1}, {"0", 1}, {"1", -2}, {"road", -1}, {"", 0}};
}

template <typename Traits> class RoadQueryContractTest : public ::testing::Test {
protected:
    RoadQueryContractTest() : query_(Traits::make()) {}

    [[nodiscard]] const scena::gateway::IRoadQuery& query() const { return *query_; }

    /// Lane positions the backend itself resolves from the probe grid —
    /// positions it definitely considers on-road.
    [[nodiscard]] std::vector<scena::gateway::LanePosition> resolved_positions() const {
        std::vector<scena::gateway::LanePosition> resolved;
        for (const auto& p : road_probe_points()) {
            scena::gateway::LanePosition lane;
            if (query().to_lane_position(p[0], p[1], p[2], lane)) {
                resolved.push_back(lane);
            }
        }
        return resolved;
    }

private:
    std::unique_ptr<scena::gateway::IRoadQuery> query_;
};

TYPED_TEST_SUITE_P(RoadQueryContractTest);

/// A position the backend maps to a lane must map back to the world, close to
/// where it came from: the two conversions are mutual inverses on the road.
TYPED_TEST_P(RoadQueryContractTest, WorldRoundTripIsConsistent) {
    for (const auto& p : road_probe_points()) {
        scena::gateway::LanePosition lane;
        if (!this->query().to_lane_position(p[0], p[1], p[2], lane)) {
            continue;
        }
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        ASSERT_TRUE(this->query().to_world_position(lane, x, y, z))
            << "backend resolved a world position it cannot convert back";
        EXPECT_NEAR(x, p[0], 1e-6);
        EXPECT_NEAR(y, p[1], 1e-6);
        EXPECT_NEAR(z, p[2], 1e-6);
    }
}

/// Non-finite world input has no answer (frozen v1 semantics).
TYPED_TEST_P(RoadQueryContractTest, NonFiniteWorldInputIsRejected) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    scena::gateway::LanePosition lane;
    EXPECT_FALSE(this->query().to_lane_position(nan, 0.0, 0.0, lane));
    EXPECT_FALSE(this->query().to_lane_position(0.0, nan, 0.0, lane));
    EXPECT_FALSE(this->query().to_lane_position(inf, 0.0, 0.0, lane));
    EXPECT_FALSE(this->query().to_lane_position(0.0, -inf, 0.0, lane));

    scena::gateway::LanePosition bad;
    bad.road_id = "0";
    bad.lane_id = -1;
    bad.s = nan;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    EXPECT_FALSE(this->query().to_world_position(bad, x, y, z));
}

/// An id that names nothing has no answer, in every query.
TYPED_TEST_P(RoadQueryContractTest, UnknownRoadHasNoAnswer) {
    const std::string unknown = "scena-contract-no-such-road";
    scena::gateway::LanePosition lane;
    lane.road_id = unknown;
    lane.lane_id = -1;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    EXPECT_FALSE(this->query().to_world_position(lane, x, y, z));

    double d = 0.0;
    int out_lane = 0;
    std::string type;
    EXPECT_FALSE(this->query().road_heading(unknown, 0.0, d));
    EXPECT_FALSE(this->query().lane_exists(unknown, -1, 0.0));
    EXPECT_FALSE(this->query().lane_width(unknown, -1, 0.0, d));
    EXPECT_FALSE(this->query().lane_center_offset(unknown, -1, 0.0, d));
    EXPECT_FALSE(this->query().lane_type(unknown, -1, 0.0, type));
    EXPECT_FALSE(this->query().relative_lane(unknown, -1, 1, out_lane));
    EXPECT_FALSE(this->query().road_length(unknown, d));
    double s0 = 0.0;
    double s1 = 0.0;
    EXPECT_FALSE(this->query().lane_s_range(unknown, -1, 0.0, s0, s1));
}

/// lane_exists is the existence probe behind the other lane queries: where it
/// answers false, the dependent lane queries answer false too.
TYPED_TEST_P(RoadQueryContractTest, LaneQueriesImplyLaneExistence) {
    for (const auto& [road, lane] : road_lane_probes()) {
        for (const double s : {0.0, 10.0}) {
            if (this->query().lane_exists(road, lane, s)) {
                continue; // the lane is there; dependent queries may answer
            }
            double d = 0.0;
            std::string type;
            EXPECT_FALSE(this->query().lane_width(road, lane, s, d))
                << road << "/" << lane << "@" << s;
            EXPECT_FALSE(this->query().lane_center_offset(road, lane, s, d))
                << road << "/" << lane << "@" << s;
            EXPECT_FALSE(this->query().lane_type(road, lane, s, type))
                << road << "/" << lane << "@" << s;
            double s0 = 0.0;
            double s1 = 0.0;
            EXPECT_FALSE(this->query().lane_s_range(road, lane, s, s0, s1))
                << road << "/" << lane << "@" << s;
        }
    }
}

/// Answered widths are finite and non-negative; answered headings, offsets,
/// lengths and ranges are finite; answered lengths are positive.
TYPED_TEST_P(RoadQueryContractTest, AnsweredValuesAreWellFormed) {
    for (const auto& lane : this->resolved_positions()) {
        double width = std::numeric_limits<double>::quiet_NaN();
        if (this->query().lane_width(lane.road_id, lane.lane_id, lane.s, width)) {
            EXPECT_TRUE(std::isfinite(width));
            EXPECT_GE(width, 0.0);
        }
        double offset = std::numeric_limits<double>::quiet_NaN();
        if (this->query().lane_center_offset(lane.road_id, lane.lane_id, lane.s, offset)) {
            EXPECT_TRUE(std::isfinite(offset));
        }
        double heading = std::numeric_limits<double>::quiet_NaN();
        if (this->query().road_heading(lane.road_id, lane.s, heading)) {
            EXPECT_TRUE(std::isfinite(heading));
        }
        double length = std::numeric_limits<double>::quiet_NaN();
        if (this->query().road_length(lane.road_id, length)) {
            EXPECT_TRUE(std::isfinite(length));
            EXPECT_GT(length, 0.0);
            EXPECT_GE(lane.s, 0.0);
            EXPECT_LE(lane.s, length);
        }
        double s0 = std::numeric_limits<double>::quiet_NaN();
        double s1 = std::numeric_limits<double>::quiet_NaN();
        if (this->query().lane_s_range(lane.road_id, lane.lane_id, lane.s, s0, s1)) {
            EXPECT_TRUE(std::isfinite(s0));
            EXPECT_TRUE(std::isfinite(s1));
            EXPECT_LE(s0, lane.s);
            EXPECT_GE(s1, lane.s);
        }
    }
}

/// relative_lane with count 0 names the lane itself, and no answered relative
/// lane is ever the centre lane 0 (skipped per ASAM OpenSCENARIO XML 1.4.0
/// §7.4.1.4).
TYPED_TEST_P(RoadQueryContractTest, RelativeLaneZeroIsIdentityAndNeverCenter) {
    for (const auto& [road, lane] : road_lane_probes()) {
        int out_lane = 0;
        if (this->query().relative_lane(road, lane, 0, out_lane)) {
            EXPECT_EQ(out_lane, lane);
        }
        for (const int count : {-2, -1, 1, 2}) {
            if (this->query().relative_lane(road, lane, count, out_lane)) {
                EXPECT_NE(out_lane, 0);
            }
        }
    }
}

/// Answered routes are well-formed: non-empty ordered spans, no span on the
/// undrivable centre lane, finite s-interval endpoints; and the first
/// waypoint projects onto the route at a finite, non-negative distance.
TYPED_TEST_P(RoadQueryContractTest, AnsweredRoutesAreWellFormed) {
    const auto resolved = this->resolved_positions();
    if (resolved.size() < 2) {
        return; // nothing this backend can route between
    }
    std::vector<scena::gateway::LanePosition> waypoints{resolved.front(), resolved.back()};
    std::vector<scena::gateway::RouteSpan> spans;
    if (!this->query().build_route(waypoints, spans)) {
        return;
    }
    ASSERT_FALSE(spans.empty());
    for (const auto& span : spans) {
        EXPECT_FALSE(span.road_id.empty());
        EXPECT_NE(span.lane_id, 0);
        EXPECT_TRUE(std::isfinite(span.s_begin));
        EXPECT_TRUE(std::isfinite(span.s_end));
    }
    double distance = std::numeric_limits<double>::quiet_NaN();
    if (this->query().position_along_route(spans, waypoints.front(), distance)) {
        EXPECT_TRUE(std::isfinite(distance));
        EXPECT_GE(distance, 0.0);
    }
}

/// Fewer than two waypoints never form a route.
TYPED_TEST_P(RoadQueryContractTest, DegenerateRouteHasNoAnswer) {
    std::vector<scena::gateway::RouteSpan> spans;
    EXPECT_FALSE(this->query().build_route({}, spans));
    const auto resolved = this->resolved_positions();
    if (!resolved.empty()) {
        EXPECT_FALSE(this->query().build_route({resolved.front()}, spans));
    }
}

/// Identical queries answer bit-identically — the determinism contract seen
/// through this interface.
TYPED_TEST_P(RoadQueryContractTest, RepeatedQueriesAreBitIdentical) {
    const auto bits = [](double v) {
        std::uint64_t u = 0;
        std::memcpy(&u, &v, sizeof(u));
        return u;
    };
    for (const auto& p : road_probe_points()) {
        scena::gateway::LanePosition first;
        scena::gateway::LanePosition second;
        const bool ok_first = this->query().to_lane_position(p[0], p[1], p[2], first);
        const bool ok_second = this->query().to_lane_position(p[0], p[1], p[2], second);
        ASSERT_EQ(ok_first, ok_second);
        if (!ok_first) {
            continue;
        }
        EXPECT_EQ(first.road_id, second.road_id);
        EXPECT_EQ(first.lane_id, second.lane_id);
        EXPECT_EQ(bits(first.s), bits(second.s));
        EXPECT_EQ(bits(first.t), bits(second.t));
    }
}

REGISTER_TYPED_TEST_SUITE_P(RoadQueryContractTest, WorldRoundTripIsConsistent,
                            NonFiniteWorldInputIsRejected, UnknownRoadHasNoAnswer,
                            LaneQueriesImplyLaneExistence, AnsweredValuesAreWellFormed,
                            RelativeLaneZeroIsIdentityAndNeverCenter, AnsweredRoutesAreWellFormed,
                            DegenerateRouteHasNoAnswer, RepeatedQueriesAreBitIdentical);

} // namespace scena::testsupport
