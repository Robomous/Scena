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

// p3-s1: the IRoadQuery contract suite, instantiated for the null-object
// backend. The OpenDRIVE backend joins with its own traits in p3-s3 and must
// pass the identical suite.

#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "scena/gateway/flat_world_road_query.h"
#include "support/road_query_contract.h"

namespace {

struct FlatWorldTraits {
    static std::unique_ptr<scena::gateway::IRoadQuery> make() {
        return std::make_unique<scena::gateway::FlatWorldRoadQuery>();
    }
};

} // namespace

// The instantiation macro must expand inside the namespace that declared the
// typed suite, so it can see the registration state it references unqualified.
namespace scena::testsupport {
using FlatWorldBackend = ::testing::Types<FlatWorldTraits>;
INSTANTIATE_TYPED_TEST_SUITE_P(FlatWorld, RoadQueryContractTest, FlatWorldBackend);
} // namespace scena::testsupport

namespace {

// Beyond the generic contract: the null object is *defined* as the backend
// with no answers, so pin every query to false, not just the implications.
TEST(FlatWorldRoadQueryTest, EveryQueryAnswersFalse) {
    const scena::gateway::FlatWorldRoadQuery query;

    scena::gateway::LanePosition lane;
    EXPECT_FALSE(query.to_lane_position(0.0, 0.0, 0.0, lane));

    lane.road_id = "0";
    lane.lane_id = -1;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    EXPECT_FALSE(query.to_world_position(lane, x, y, z));

    double value = 0.0;
    EXPECT_FALSE(query.road_heading("0", 0.0, value));
    EXPECT_FALSE(query.lane_exists("0", -1, 0.0));
    EXPECT_FALSE(query.lane_width("0", -1, 0.0, value));
    EXPECT_FALSE(query.lane_center_offset("0", -1, 0.0, value));

    std::string type;
    EXPECT_FALSE(query.lane_type("0", -1, 0.0, type));

    int out_lane = 0;
    EXPECT_FALSE(query.relative_lane("0", -1, 1, out_lane));
    EXPECT_FALSE(query.road_length("0", value));

    double s0 = 0.0;
    double s1 = 0.0;
    EXPECT_FALSE(query.lane_s_range("0", -1, 0.0, s0, s1));

    std::vector<scena::gateway::RouteSpan> spans;
    std::vector<scena::gateway::LanePosition> waypoints(2, lane);
    EXPECT_FALSE(query.build_route(waypoints, spans));

    double distance = 0.0;
    EXPECT_FALSE(query.position_along_route(spans, lane, distance));
}

// The null object is final and stateless; a fresh instance behaves like any
// other, so the engine may share or recreate it freely.
TEST(FlatWorldRoadQueryTest, InstancesAreInterchangeable) {
    const scena::gateway::FlatWorldRoadQuery a;
    const scena::gateway::FlatWorldRoadQuery b;
    scena::gateway::LanePosition lane;
    EXPECT_EQ(a.to_lane_position(1.0, 2.0, 0.0, lane), b.to_lane_position(1.0, 2.0, 0.0, lane));
}

} // namespace
