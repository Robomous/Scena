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

// p3-s3: lane sections, polynomial widths, lane queries and conversions of
// OpenDriveRoadQuery (ASAM OpenDRIVE 1.9.0 §11; relative-lane arithmetic per
// ASAM OpenSCENARIO XML 1.4.0 §7.4.1.4).

#include <string>

#include <gtest/gtest.h>

#include "scena/opendrive/reader.h"
#include "scena/opendrive/road_query.h"

namespace {

using scena::DiagnosticSink;
using scena::Severity;
using scena::Status;
using scena::gateway::LanePosition;
using scena::opendrive::Map;
using scena::opendrive::OpenDriveRoadQuery;

// Straight road "1", 100 m along +x from the origin. Two lane sections:
//   s in [0, 60):  left 1 (driving, 3.0) | centre | right -1 (driving,
//                  3.5 + 0.01 ds), -2 (border, 1.5 until ds 30, then
//                  narrowing 1.5 - 0.05 (ds - 30))
//   s in [60, 100]: left 1 | centre | right -1 only (lane -2 ends)
std::string lane_road() {
    return "<?xml version=\"1.0\"?><OpenDRIVE><header revMajor=\"1\" revMinor=\"8\"/>"
           "<road id=\"1\" length=\"100\">"
           "<planView><geometry s=\"0\" x=\"0\" y=\"0\" hdg=\"0\" length=\"100\">"
           "<line/></geometry></planView>"
           "<lanes>"
           "<laneSection s=\"0\">"
           "<left><lane id=\"1\" type=\"driving\">"
           "<width sOffset=\"0\" a=\"3.0\" b=\"0\" c=\"0\" d=\"0\"/></lane></left>"
           "<center><lane id=\"0\"/></center>"
           "<right>"
           "<lane id=\"-1\" type=\"driving\">"
           "<width sOffset=\"0\" a=\"3.5\" b=\"0.01\" c=\"0\" d=\"0\"/></lane>"
           "<lane id=\"-2\" type=\"border\">"
           "<width sOffset=\"0\" a=\"1.5\" b=\"0\" c=\"0\" d=\"0\"/>"
           "<width sOffset=\"30\" a=\"1.5\" b=\"-0.05\" c=\"0\" d=\"0\"/></lane>"
           "</right>"
           "</laneSection>"
           "<laneSection s=\"60\">"
           "<left><lane id=\"1\" type=\"driving\">"
           "<width sOffset=\"0\" a=\"3.0\" b=\"0\" c=\"0\" d=\"0\"/></lane></left>"
           "<center><lane id=\"0\"/></center>"
           "<right><lane id=\"-1\" type=\"driving\">"
           "<width sOffset=\"0\" a=\"3.5\" b=\"0\" c=\"0\" d=\"0\"/></lane></right>"
           "</laneSection>"
           "</lanes>"
           "</road></OpenDRIVE>";
}

OpenDriveRoadQuery make_query() {
    Map map;
    DiagnosticSink sink;
    const Status status = scena::opendrive::load_string(lane_road(), map, sink);
    EXPECT_EQ(status, Status::Ok);
    return OpenDriveRoadQuery(std::move(map));
}

TEST(OpenDriveLanesTest, LaneExistenceFollowsSections) {
    const OpenDriveRoadQuery query = make_query();
    EXPECT_TRUE(query.lane_exists("1", -2, 30.0));
    EXPECT_FALSE(query.lane_exists("1", -2, 70.0));
    EXPECT_TRUE(query.lane_exists("1", 1, 30.0));
    EXPECT_TRUE(query.lane_exists("1", 1, 70.0));
    EXPECT_TRUE(query.lane_exists("1", 0, 50.0));
    EXPECT_FALSE(query.lane_exists("1", 5, 50.0));
    EXPECT_FALSE(query.lane_exists("1", -1, 150.0)); // past the road end
    EXPECT_FALSE(query.lane_exists("2", -1, 10.0));  // no such road
}

TEST(OpenDriveLanesTest, WidthEvaluatesTheActivePolynomialRecord) {
    const OpenDriveRoadQuery query = make_query();
    double width = 0.0;
    // Single record with slope: w(ds) = 3.5 + 0.01 ds (§11.7.1).
    ASSERT_TRUE(query.lane_width("1", -1, 20.0, width));
    EXPECT_DOUBLE_EQ(width, 3.5 + 0.01 * 20.0);
    // Second record takes over at its sOffset; ds restarts per record.
    ASSERT_TRUE(query.lane_width("1", -2, 10.0, width));
    EXPECT_DOUBLE_EQ(width, 1.5);
    ASSERT_TRUE(query.lane_width("1", -2, 40.0, width));
    EXPECT_DOUBLE_EQ(width, 1.5 - 0.05 * 10.0);
    // In the second section the -1 width is the new section's record.
    ASSERT_TRUE(query.lane_width("1", -1, 80.0, width));
    EXPECT_DOUBLE_EQ(width, 3.5);
}

TEST(OpenDriveLanesTest, CenterLaneHasOffsetZeroAndNoWidth) {
    const OpenDriveRoadQuery query = make_query();
    double value = 1.0;
    EXPECT_FALSE(query.lane_width("1", 0, 30.0, value));
    ASSERT_TRUE(query.lane_center_offset("1", 0, 30.0, value));
    EXPECT_DOUBLE_EQ(value, 0.0);
}

TEST(OpenDriveLanesTest, LaneCenterOffsetsAccumulateWidths) {
    const OpenDriveRoadQuery query = make_query();
    double t = 0.0;
    // At s = 0: lane -1 spans [-3.5, 0], centre at -1.75; lane -2 spans
    // [-5.0, -3.5], centre at -4.25; lane 1 spans [0, 3.0], centre at 1.5.
    ASSERT_TRUE(query.lane_center_offset("1", -1, 0.0, t));
    EXPECT_DOUBLE_EQ(t, -1.75);
    ASSERT_TRUE(query.lane_center_offset("1", -2, 0.0, t));
    EXPECT_DOUBLE_EQ(t, -4.25);
    ASSERT_TRUE(query.lane_center_offset("1", 1, 0.0, t));
    EXPECT_DOUBLE_EQ(t, 1.5);
    // t points left (§8.3): right lanes negative, left lanes positive.
}

TEST(OpenDriveLanesTest, LaneTypeIsHandedThroughVerbatim) {
    const OpenDriveRoadQuery query = make_query();
    std::string type;
    ASSERT_TRUE(query.lane_type("1", -1, 10.0, type));
    EXPECT_EQ(type, "driving");
    ASSERT_TRUE(query.lane_type("1", -2, 10.0, type));
    EXPECT_EQ(type, "border");
    // The centre lane carries no type attribute here: no answer.
    EXPECT_FALSE(query.lane_type("1", 0, 10.0, type));
}

TEST(OpenDriveLanesTest, RelativeLaneSkipsTheCenterLane) {
    const OpenDriveRoadQuery query = make_query();
    int out = 99;
    // Crossing the centre: -1 one to the left (+t) is 1, and back (§7.4.1.4:
    // the centre lane "is not counted as a lane and thus omitted").
    ASSERT_TRUE(query.relative_lane("1", -1, 1, out));
    EXPECT_EQ(out, 1);
    ASSERT_TRUE(query.relative_lane("1", 1, -1, out));
    EXPECT_EQ(out, -1);
    ASSERT_TRUE(query.relative_lane("1", -2, 1, out));
    EXPECT_EQ(out, -1);
    ASSERT_TRUE(query.relative_lane("1", -1, -1, out));
    EXPECT_EQ(out, -2);
    ASSERT_TRUE(query.relative_lane("1", -2, 2, out));
    EXPECT_EQ(out, 1);
    // Identity.
    ASSERT_TRUE(query.relative_lane("1", -1, 0, out));
    EXPECT_EQ(out, -1);
    // No lane 2 anywhere on the road.
    EXPECT_FALSE(query.relative_lane("1", 1, 1, out));
    // Unknown source lane.
    EXPECT_FALSE(query.relative_lane("1", 7, 1, out));
}

TEST(OpenDriveLanesTest, LaneSRangeSpansContiguousSections) {
    const OpenDriveRoadQuery query = make_query();
    double s0 = -1.0;
    double s1 = -1.0;
    // Lane -2 exists only in the first section: [0, 60).
    ASSERT_TRUE(query.lane_s_range("1", -2, 30.0, s0, s1));
    EXPECT_DOUBLE_EQ(s0, 0.0);
    EXPECT_DOUBLE_EQ(s1, 60.0);
    // Lane -1 continues across both sections: the whole road.
    ASSERT_TRUE(query.lane_s_range("1", -1, 70.0, s0, s1));
    EXPECT_DOUBLE_EQ(s0, 0.0);
    EXPECT_DOUBLE_EQ(s1, 100.0);
    EXPECT_FALSE(query.lane_s_range("1", -2, 70.0, s0, s1));
}

TEST(OpenDriveLanesTest, ToLanePositionAssignsTheContainingLane) {
    const OpenDriveRoadQuery query = make_query();
    LanePosition lane;
    ASSERT_TRUE(query.to_lane_position(50.0, -1.7, 0.0, lane));
    EXPECT_EQ(lane.road_id, "1");
    EXPECT_EQ(lane.lane_id, -1);
    EXPECT_NEAR(lane.s, 50.0, 1e-6);
    EXPECT_NEAR(lane.t, -1.7, 1e-9);

    // Between lane -1's outer edge (-4.0 at s=50) and lane -2's: lane -2.
    ASSERT_TRUE(query.to_lane_position(50.0, -4.5, 0.0, lane));
    EXPECT_EQ(lane.lane_id, -2);

    ASSERT_TRUE(query.to_lane_position(50.0, 1.0, 0.0, lane));
    EXPECT_EQ(lane.lane_id, 1);

    // Exactly the reference line: the centre lane.
    ASSERT_TRUE(query.to_lane_position(50.0, 0.0, 0.0, lane));
    EXPECT_EQ(lane.lane_id, 0);

    // Laterally beyond the cross-section, or off the z = 0 plane: off-road.
    EXPECT_FALSE(query.to_lane_position(50.0, 10.0, 0.0, lane));
    EXPECT_FALSE(query.to_lane_position(70.0, -4.5, 0.0, lane)); // -2 ended
    EXPECT_FALSE(query.to_lane_position(50.0, -1.7, 5.0, lane));
}

TEST(OpenDriveLanesTest, WorldRoundTripInvertsExactlyOnTheFlatPlane) {
    const OpenDriveRoadQuery query = make_query();
    LanePosition lane;
    ASSERT_TRUE(query.to_lane_position(42.0, -2.25, 0.0, lane));
    double x = 0.0;
    double y = 0.0;
    double z = 1.0;
    ASSERT_TRUE(query.to_world_position(lane, x, y, z));
    EXPECT_NEAR(x, 42.0, 1e-6);
    EXPECT_NEAR(y, -2.25, 1e-6);
    EXPECT_DOUBLE_EQ(z, 0.0);
}

TEST(OpenDriveLanesTest, ToWorldPositionRejectsInvalidCoordinates) {
    const OpenDriveRoadQuery query = make_query();
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    LanePosition lane;
    lane.road_id = "1";
    lane.lane_id = -1;
    lane.s = 150.0; // past the road end
    EXPECT_FALSE(query.to_world_position(lane, x, y, z));
    lane.s = 50.0;
    lane.t = -25.0; // far outside the cross-section
    EXPECT_FALSE(query.to_world_position(lane, x, y, z));
    lane.t = 0.0;
    lane.lane_id = 7; // no such lane
    EXPECT_FALSE(query.to_world_position(lane, x, y, z));
    lane.lane_id = -1;
    lane.road_id = "none";
    EXPECT_FALSE(query.to_world_position(lane, x, y, z));
}

TEST(OpenDriveLanesTest, GeometryOnlyRoadAnswersGeometryButNotLanes) {
    Map map;
    DiagnosticSink sink;
    const Status status = scena::opendrive::load_string(
        "<?xml version=\"1.0\"?><OpenDRIVE><header revMajor=\"1\" revMinor=\"8\"/>"
        "<road id=\"9\" length=\"50\"><planView>"
        "<geometry s=\"0\" x=\"0\" y=\"0\" hdg=\"0\" length=\"50\"><line/></geometry>"
        "</planView></road></OpenDRIVE>",
        map, sink);
    ASSERT_EQ(status, Status::Ok);
    const OpenDriveRoadQuery query(std::move(map));

    double value = 0.0;
    EXPECT_TRUE(query.road_length("9", value));
    EXPECT_DOUBLE_EQ(value, 50.0);
    EXPECT_TRUE(query.road_heading("9", 10.0, value));
    EXPECT_FALSE(query.lane_exists("9", -1, 10.0));
    LanePosition lane;
    EXPECT_FALSE(query.to_lane_position(10.0, 0.0, 0.0, lane));
}

TEST(OpenDriveLanesTest, WidthRuleViolationsAreDiagnosed) {
    Map map;
    DiagnosticSink sink;
    // Centre lane with a width record violates center_lane_no_width.
    const Status status = scena::opendrive::load_string(
        "<?xml version=\"1.0\"?><OpenDRIVE><header revMajor=\"1\" revMinor=\"8\"/>"
        "<road id=\"1\" length=\"10\"><planView>"
        "<geometry s=\"0\" x=\"0\" y=\"0\" hdg=\"0\" length=\"10\"><line/></geometry>"
        "</planView><lanes><laneSection s=\"0\">"
        "<center><lane id=\"0\"><width sOffset=\"0\" a=\"1\" b=\"0\" c=\"0\" d=\"0\"/>"
        "</lane></center>"
        "<right><lane id=\"-1\" type=\"driving\">"
        "<width sOffset=\"0\" a=\"3.5\" b=\"0\" c=\"0\" d=\"0\"/></lane></right>"
        "</laneSection></lanes></road></OpenDRIVE>",
        map, sink);
    EXPECT_EQ(status, Status::ValidationError);
    bool found = false;
    for (const auto& d : sink.diagnostics()) {
        if (d.rule_id == "asam.net:xodr:1.4.0:road.lane.center_lane_no_width") {
            found = true;
        }
    }
    EXPECT_TRUE(found);
}

} // namespace
