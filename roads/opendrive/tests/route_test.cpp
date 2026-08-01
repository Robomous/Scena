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

// p3-s3: deterministic routing over road links and junction connections
// (ASAM OpenDRIVE 1.9.0 §10.3, §11.6, §12.2-§12.4), producing the frozen
// v1 route representation (RouteSpan).

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "scena/opendrive/reader.h"
#include "scena/opendrive/road_query.h"
#include "support/trace_recorder.h"

namespace {

using scena::DiagnosticSink;
using scena::Status;
using scena::gateway::LanePosition;
using scena::gateway::RouteSpan;
using scena::opendrive::Map;
using scena::opendrive::OpenDriveRoadQuery;

std::string simple_lanes(const char* pred, const char* succ) {
    // One driving lane per side; the lane-level link mirrors `pred`/`succ`
    // (§11.6: the linked id names the lane on the linked element).
    std::string lanes = "<lanes><laneSection s=\"0\">";
    lanes += "<left><lane id=\"1\" type=\"driving\"><link>";
    if (pred[0] != '\0') {
        lanes += std::string("<predecessor id=\"") + "1" + "\"/>";
    }
    lanes += "</link><width sOffset=\"0\" a=\"3.5\" b=\"0\" c=\"0\" d=\"0\"/></lane></left>";
    lanes += "<center><lane id=\"0\"/></center>";
    lanes += "<right><lane id=\"-1\" type=\"driving\"><link>";
    if (pred[0] != '\0') {
        lanes += "<predecessor id=\"-1\"/>";
    }
    if (succ[0] != '\0') {
        lanes += "<successor id=\"-1\"/>";
    }
    lanes += "</link><width sOffset=\"0\" a=\"3.5\" b=\"0\" c=\"0\" d=\"0\"/></lane></right>";
    lanes += "</laneSection></lanes>";
    return lanes;
}

std::string road(const std::string& id, double x0, double length, const std::string& link,
                 const std::string& lanes, const char* junction = "-1") {
    std::string result = "<road id=\"" + id + "\" length=\"" +
                         std::to_string(static_cast<int>(length)) + "\" junction=\"" + junction +
                         "\">";
    result += link;
    result += "<planView><geometry s=\"0\" x=\"" + std::to_string(static_cast<int>(x0)) +
              "\" y=\"0\" hdg=\"0\" length=\"" + std::to_string(static_cast<int>(length)) +
              "\"><line/></geometry></planView>";
    result += lanes;
    result += "</road>";
    return result;
}

/// Two roads chained end-to-start: 1 [0,100] -> 2 [100,200].
std::string chained_map() {
    std::string doc = "<?xml version=\"1.0\"?><OpenDRIVE>"
                      "<header revMajor=\"1\" revMinor=\"8\"/>";
    doc += road("1", 0, 100,
                "<link><successor elementType=\"road\" elementId=\"2\" "
                "contactPoint=\"start\"/></link>",
                simple_lanes("", "yes"));
    doc += road("2", 100, 100,
                "<link><predecessor elementType=\"road\" elementId=\"1\" "
                "contactPoint=\"end\"/></link>",
                simple_lanes("yes", ""));
    doc += "</OpenDRIVE>";
    return doc;
}

/// Incoming road 1 -> junction 50 with two connecting roads (2 and 3, both
/// linking lane -1 to -1) -> outgoing road 4. Lengths parameterized so the
/// tie and the shortest-path cases share a builder.
std::string junction_map(double len2, double len3) {
    std::string doc = "<?xml version=\"1.0\"?><OpenDRIVE>"
                      "<header revMajor=\"1\" revMinor=\"8\"/>";
    doc += road("1", 0, 100, "<link><successor elementType=\"junction\" elementId=\"50\"/></link>",
                simple_lanes("", "yes"));
    const auto connecting = [&](const std::string& id, double length) {
        return road(id, 100, length,
                    "<link><predecessor elementType=\"road\" elementId=\"1\" "
                    "contactPoint=\"end\"/><successor elementType=\"road\" "
                    "elementId=\"4\" contactPoint=\"start\"/></link>",
                    simple_lanes("yes", "yes"), "50");
    };
    doc += connecting("2", len2);
    doc += connecting("3", len3);
    doc +=
        road("4", 150, 100, "<link><predecessor elementType=\"junction\" elementId=\"50\"/></link>",
             simple_lanes("yes", ""));
    doc += "<junction id=\"50\" type=\"default\">"
           "<connection id=\"0\" incomingRoad=\"1\" connectingRoad=\"2\" "
           "contactPoint=\"start\"><laneLink from=\"-1\" to=\"-1\"/></connection>"
           "<connection id=\"1\" incomingRoad=\"1\" connectingRoad=\"3\" "
           "contactPoint=\"start\"><laneLink from=\"-1\" to=\"-1\"/></connection>"
           "</junction>";
    doc += "</OpenDRIVE>";
    return doc;
}

OpenDriveRoadQuery load_query(const std::string& xml) {
    Map map;
    DiagnosticSink sink;
    const Status status = scena::opendrive::load_string(xml, map, sink);
    EXPECT_EQ(status, Status::Ok) << (sink.diagnostics().empty()
                                          ? std::string("no diagnostics")
                                          : sink.diagnostics().front().message);
    return OpenDriveRoadQuery(std::move(map));
}

LanePosition at(const std::string& road_id, int lane_id, double s) {
    LanePosition position;
    position.road_id = road_id;
    position.lane_id = lane_id;
    position.s = s;
    return position;
}

TEST(RouteTest, DirectSpanOnTheSameLane) {
    const OpenDriveRoadQuery query = load_query(chained_map());
    std::vector<RouteSpan> spans;
    ASSERT_TRUE(query.build_route({at("1", -1, 10.0), at("1", -1, 80.0)}, spans));
    ASSERT_EQ(spans.size(), 1U);
    EXPECT_EQ(spans[0].road_id, "1");
    EXPECT_EQ(spans[0].lane_id, -1);
    EXPECT_DOUBLE_EQ(spans[0].s_begin, 10.0);
    EXPECT_DOUBLE_EQ(spans[0].s_end, 80.0);

    double distance = -1.0;
    ASSERT_TRUE(query.position_along_route(spans, at("1", -1, 40.0), distance));
    EXPECT_DOUBLE_EQ(distance, 30.0);
}

TEST(RouteTest, SpansCrossARoadLink) {
    const OpenDriveRoadQuery query = load_query(chained_map());
    std::vector<RouteSpan> spans;
    ASSERT_TRUE(query.build_route({at("1", -1, 20.0), at("2", -1, 50.0)}, spans));
    ASSERT_EQ(spans.size(), 2U);
    EXPECT_EQ(spans[0].road_id, "1");
    EXPECT_DOUBLE_EQ(spans[0].s_begin, 20.0);
    EXPECT_DOUBLE_EQ(spans[0].s_end, 100.0);
    EXPECT_EQ(spans[1].road_id, "2");
    EXPECT_DOUBLE_EQ(spans[1].s_begin, 0.0);
    EXPECT_DOUBLE_EQ(spans[1].s_end, 50.0);

    double distance = -1.0;
    ASSERT_TRUE(query.position_along_route(spans, at("2", -1, 10.0), distance));
    EXPECT_DOUBLE_EQ(distance, 90.0);
    // A position on a lane the route does not traverse has no answer.
    EXPECT_FALSE(query.position_along_route(spans, at("2", 1, 10.0), distance));
}

TEST(RouteTest, LeftLaneRoutesAgainstS) {
    const OpenDriveRoadQuery query = load_query(chained_map());
    std::vector<RouteSpan> spans;
    // Lane 1 drives against s (§11.3.1 right-hand traffic): from road 2 back
    // onto road 1 via the lane-level predecessor link.
    ASSERT_TRUE(query.build_route({at("2", 1, 50.0), at("1", 1, 30.0)}, spans));
    ASSERT_EQ(spans.size(), 2U);
    EXPECT_EQ(spans[0].road_id, "2");
    EXPECT_DOUBLE_EQ(spans[0].s_begin, 50.0);
    EXPECT_DOUBLE_EQ(spans[0].s_end, 0.0);
    EXPECT_EQ(spans[1].road_id, "1");
    EXPECT_DOUBLE_EQ(spans[1].s_begin, 100.0);
    EXPECT_DOUBLE_EQ(spans[1].s_end, 30.0);
}

TEST(RouteTest, BehindOnTheSameLaneIsUnreachable) {
    const OpenDriveRoadQuery query = load_query(chained_map());
    std::vector<RouteSpan> spans;
    EXPECT_FALSE(query.build_route({at("1", -1, 80.0), at("1", -1, 10.0)}, spans));
    // The centre lane is not drivable.
    EXPECT_FALSE(query.build_route({at("1", 0, 10.0), at("1", 0, 80.0)}, spans));
}

TEST(RouteTest, JunctionConnectionsAreFollowed) {
    const OpenDriveRoadQuery query = load_query(junction_map(50, 50));
    std::vector<RouteSpan> spans;
    ASSERT_TRUE(query.build_route({at("1", -1, 50.0), at("4", -1, 10.0)}, spans));
    ASSERT_EQ(spans.size(), 3U);
    EXPECT_EQ(spans[0].road_id, "1");
    EXPECT_EQ(spans[2].road_id, "4");
    EXPECT_DOUBLE_EQ(spans[2].s_end, 10.0);
}

TEST(RouteTest, EqualLengthPathsTieBreakOnSmallestRoadId) {
    // Connecting roads 2 and 3 have identical length: the documented
    // tie-break (lexicographically smallest (road id, lane id)) picks 2.
    const OpenDriveRoadQuery query = load_query(junction_map(50, 50));
    std::vector<RouteSpan> spans;
    ASSERT_TRUE(query.build_route({at("1", -1, 50.0), at("4", -1, 10.0)}, spans));
    ASSERT_EQ(spans.size(), 3U);
    EXPECT_EQ(spans[1].road_id, "2");
}

TEST(RouteTest, ShorterConnectingRoadWins) {
    const OpenDriveRoadQuery query = load_query(junction_map(50, 20));
    std::vector<RouteSpan> spans;
    ASSERT_TRUE(query.build_route({at("1", -1, 50.0), at("4", -1, 10.0)}, spans));
    ASSERT_EQ(spans.size(), 3U);
    EXPECT_EQ(spans[1].road_id, "3");
}

TEST(RouteTest, JunctionLaneLinksGateTheConnection) {
    const OpenDriveRoadQuery query = load_query(junction_map(50, 50));
    std::vector<RouteSpan> spans;
    // Lane 1 drives away from the junction and has no laneLink: no path.
    EXPECT_FALSE(query.build_route({at("1", 1, 50.0), at("4", -1, 10.0)}, spans));
}

TEST(RouteTest, MultiWaypointRoutesConcatenate) {
    const OpenDriveRoadQuery query = load_query(chained_map());
    std::vector<RouteSpan> spans;
    ASSERT_TRUE(
        query.build_route({at("1", -1, 10.0), at("1", -1, 60.0), at("2", -1, 30.0)}, spans));
    ASSERT_EQ(spans.size(), 3U);
    EXPECT_DOUBLE_EQ(spans[0].s_begin, 10.0);
    EXPECT_DOUBLE_EQ(spans[0].s_end, 60.0);
    EXPECT_DOUBLE_EQ(spans[1].s_begin, 60.0);
    EXPECT_DOUBLE_EQ(spans[1].s_end, 100.0);
    EXPECT_EQ(spans[2].road_id, "2");
}

TEST(RouteTest, RoutingIsBitIdenticalAcrossInstances) {
    const OpenDriveRoadQuery first = load_query(junction_map(50, 50));
    const OpenDriveRoadQuery second = load_query(junction_map(50, 50));
    std::vector<RouteSpan> spans_a;
    std::vector<RouteSpan> spans_b;
    ASSERT_TRUE(first.build_route({at("1", -1, 12.5), at("4", -1, 87.5)}, spans_a));
    ASSERT_TRUE(second.build_route({at("1", -1, 12.5), at("4", -1, 87.5)}, spans_b));
    ASSERT_EQ(spans_a.size(), spans_b.size());
    for (std::size_t i = 0; i < spans_a.size(); ++i) {
        EXPECT_EQ(spans_a[i].road_id, spans_b[i].road_id);
        EXPECT_EQ(spans_a[i].lane_id, spans_b[i].lane_id);
        EXPECT_EQ(scena::testsupport::hex_bits(spans_a[i].s_begin),
                  scena::testsupport::hex_bits(spans_b[i].s_begin));
        EXPECT_EQ(scena::testsupport::hex_bits(spans_a[i].s_end),
                  scena::testsupport::hex_bits(spans_b[i].s_end));
    }
}

} // namespace
