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

// p3-s2: OpenDRIVE reader + reference-line geometry (ASAM OpenDRIVE 1.9.0
// §9). Analytic fixtures are pinned through detmath expressions computed in
// the test itself — both sides of each EXPECT go through det_sincos /
// det_atan2, so equality is bit-exact on every platform (hex_bits).

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "scena/opendrive/reader.h"
#include "scena/opendrive/reference_line.h"
#include "scena/runtime/detmath.h"
#include "support/trace_recorder.h"

namespace {

using scena::DiagnosticSink;
using scena::Severity;
using scena::Status;
using scena::opendrive::Map;
using scena::opendrive::ReferenceLine;
using scena::opendrive::RefPose;
using scena::opendrive::TrackPosition;
using scena::runtime::det_atan2;
using scena::runtime::det_sincos;
using scena::testsupport::hex_bits;

std::string wrap(const std::string& roads) {
    return "<?xml version=\"1.0\"?>\n<OpenDRIVE>\n"
           "<header revMajor=\"1\" revMinor=\"8\"/>\n" +
           roads + "\n</OpenDRIVE>\n";
}

Map load_ok(const std::string& xml) {
    Map map;
    DiagnosticSink sink;
    const Status status = scena::opendrive::load_string(xml, map, sink);
    EXPECT_EQ(status, Status::Ok) << (sink.diagnostics().empty()
                                          ? std::string("no diagnostics")
                                          : sink.diagnostics().front().message);
    return map;
}

bool has_diagnostic(const DiagnosticSink& sink, Severity severity, Status code,
                    const std::string& rule_id = {}) {
    for (const auto& d : sink.diagnostics()) {
        if (d.severity == severity && d.code == code && (rule_id.empty() || d.rule_id == rule_id)) {
            return true;
        }
    }
    return false;
}

// --- straight line (§9.3) --------------------------------------------------

TEST(OpenDriveGeometryTest, LinePoseIsAnalytic) {
    const Map map =
        load_ok(wrap("<road id=\"1\" length=\"57.28\"><planView>"
                     "<geometry s=\"0\" x=\"-47.17\" y=\"0.728\" hdg=\"0.6547\" length=\"57.28\">"
                     "<line/></geometry>"
                     "</planView></road>"));
    const ReferenceLine line(map.roads.at("1"));
    ASSERT_TRUE(line.ok());
    EXPECT_DOUBLE_EQ(line.length(), 57.28);

    const auto dir = det_sincos(0.6547);
    for (const double s : {0.0, 1.0, 20.5, 57.28}) {
        const RefPose pose = line.pose_at(s);
        EXPECT_EQ(hex_bits(pose.x), hex_bits(-47.17 + s * dir.cos)) << "s=" << s;
        EXPECT_EQ(hex_bits(pose.y), hex_bits(0.728 + s * dir.sin)) << "s=" << s;
        EXPECT_EQ(hex_bits(pose.heading), hex_bits(0.6547)) << "s=" << s;
    }
}

// --- arc (§9.5) ------------------------------------------------------------

TEST(OpenDriveGeometryTest, ArcFollowsTheCircle) {
    // kappa = +0.05 (left curve), start at the origin heading +x.
    const Map map = load_ok(wrap("<road id=\"1\" length=\"40\"><planView>"
                                 "<geometry s=\"0\" x=\"0\" y=\"0\" hdg=\"0\" length=\"40\">"
                                 "<arc curvature=\"0.05\"/></geometry>"
                                 "</planView></road>"));
    const ReferenceLine line(map.roads.at("1"));
    ASSERT_TRUE(line.ok());

    // Circle analytics: x = sin(kappa s)/kappa, y = (1 - cos(kappa s))/kappa,
    // heading = kappa s.
    for (const double s : {0.0, 5.0, 17.5, 40.0}) {
        const RefPose pose = line.pose_at(s);
        const auto angle = det_sincos(0.05 * s);
        EXPECT_NEAR(pose.x, angle.sin / 0.05, 1e-9) << "s=" << s;
        EXPECT_NEAR(pose.y, (1.0 - angle.cos) / 0.05, 1e-9) << "s=" << s;
        EXPECT_NEAR(pose.heading, 0.05 * s, 1e-12) << "s=" << s;
    }
}

// --- spiral (§9.4) ---------------------------------------------------------

TEST(OpenDriveGeometryTest, SpiralHeadingIsQuadraticInS) {
    // Curvature linear from 0 to 0.02 over 30 m (§9.4), so the tangent angle
    // is theta0 + kappa0 s + kappa' s^2 / 2 with kappa' = (0.02 - 0)/30.
    const Map map =
        load_ok(wrap("<road id=\"1\" length=\"30\"><planView>"
                     "<geometry s=\"0\" x=\"38\" y=\"-1.81\" hdg=\"0.33\" length=\"30\">"
                     "<spiral curvStart=\"0\" curvEnd=\"0.02\"/></geometry>"
                     "</planView></road>"));
    const ReferenceLine line(map.roads.at("1"));
    ASSERT_TRUE(line.ok());

    const double kappa_prime = 0.02 / 30.0;
    for (const double s : {0.0, 7.5, 15.0, 30.0}) {
        const RefPose pose = line.pose_at(s);
        EXPECT_NEAR(pose.heading, 0.33 + 0.5 * kappa_prime * s * s, 1e-9) << "s=" << s;
    }
    // The spiral bends left (positive curvature): lateral deviation from the
    // start tangent grows monotonically.
    const RefPose quarter = line.pose_at(7.5);
    const RefPose end = line.pose_at(30.0);
    const auto start_dir = det_sincos(0.33);
    const auto left_of = [&](const RefPose& p) {
        return -(p.x - 38.0) * start_dir.sin + (p.y - -1.81) * start_dir.cos;
    };
    EXPECT_GT(left_of(end), left_of(quarter));
    EXPECT_GT(left_of(quarter), 0.0);
}

// --- paramPoly3 (§9.6) -----------------------------------------------------

TEST(OpenDriveGeometryTest, ParamPoly3EndpointIsExact) {
    // Aligned curve (aU=aV=bV=0, bU=1, §9.6 valid_parameters), gentle
    // parabola v = 1e-3 p^2, pRange=arcLength.
    const Map map =
        load_ok(wrap("<road id=\"1\" length=\"50\"><planView>"
                     "<geometry s=\"0\" x=\"5\" y=\"7\" hdg=\"0.25\" length=\"50\">"
                     "<paramPoly3 aU=\"0\" bU=\"1\" cU=\"0\" dU=\"0\" aV=\"0\" bV=\"0\" cV=\"1e-3\""
                     " dV=\"0\" pRange=\"arcLength\"/></geometry>"
                     "</planView></road>"));
    const ReferenceLine line(map.roads.at("1"));
    ASSERT_TRUE(line.ok());

    // s = length lands on the last table node up to one ulp of the linear
    // inversion, whose position is the cubic at p_end rotated by hdg.
    const double p_end = 50.0;
    const double u = p_end;
    const double v = 1e-3 * p_end * p_end;
    const auto rot = det_sincos(0.25);
    const RefPose end = line.pose_at(50.0);
    EXPECT_NEAR(end.x, 5.0 + u * rot.cos - v * rot.sin, 1e-9);
    EXPECT_NEAR(end.y, 7.0 + u * rot.sin + v * rot.cos, 1e-9);

    // Start pose: position (5, 7), tangent along bU (local u-axis), so the
    // heading is hdg exactly (det_atan2 of the rotated (1, 0)).
    const RefPose start = line.pose_at(0.0);
    EXPECT_EQ(hex_bits(start.x), hex_bits(5.0));
    EXPECT_EQ(hex_bits(start.y), hex_bits(7.0));
    EXPECT_EQ(hex_bits(start.heading), hex_bits(det_atan2(rot.sin, rot.cos)));
}

TEST(OpenDriveGeometryTest, ParamPoly3NormalizedRangeSpansTheElement) {
    // Same parabola expressed with p in [0, 1] (§9.6 normalized): u = 50 p.
    const Map map =
        load_ok(wrap("<road id=\"1\" length=\"50\"><planView>"
                     "<geometry s=\"0\" x=\"0\" y=\"0\" hdg=\"0\" length=\"50\">"
                     "<paramPoly3 aU=\"0\" bU=\"50\" cU=\"0\" dU=\"0\" aV=\"0\" bV=\"0\" cV=\"2.5\""
                     " dV=\"0\" pRange=\"normalized\"/></geometry>"
                     "</planView></road>"));
    const ReferenceLine line(map.roads.at("1"));
    ASSERT_TRUE(line.ok());
    const RefPose end = line.pose_at(50.0);
    EXPECT_NEAR(end.x, 50.0, 1e-9); // u(1) = 50, v(1) = 2.5
    EXPECT_NEAR(end.y, 2.5, 1e-9);
}

// --- poly3 (§9.7, deprecated) ----------------------------------------------

TEST(OpenDriveGeometryTest, Poly3IsEvaluatedAndWarnedDeprecated) {
    Map map;
    DiagnosticSink sink;
    const Status status = scena::opendrive::load_string(
        wrap("<road id=\"1\" length=\"25.6\"><planView>"
             "<geometry s=\"0\" x=\"0\" y=\"0\" hdg=\"0\" length=\"25.6\">"
             "<poly3 a=\"0\" b=\"0\" c=\"1.4e-2\" d=\"-5.7e-4\"/></geometry>"
             "</planView></road>"),
        map, sink);
    ASSERT_EQ(status, Status::Ok);
    EXPECT_TRUE(has_diagnostic(sink, Severity::Warning, Status::DeprecatedFeature));

    const ReferenceLine line(map.roads.at("1"));
    ASSERT_TRUE(line.ok());
    // v(u) = c u^2 + d u^3 curves away from the u-axis; the projection of a
    // mid point back onto the line must reproduce its s (self-consistency).
    const RefPose mid = line.pose_at(12.0);
    const auto track = line.project(mid.x, mid.y);
    ASSERT_TRUE(track.has_value());
    EXPECT_NEAR(track->s, 12.0, 1e-3);
    EXPECT_NEAR(track->t, 0.0, 1e-6);
}

// --- multi-element road, round trip, projection ----------------------------

std::string three_element_road() {
    // line (40 m) -> spiral easing 0 -> 0.02 (30 m) -> arc 0.02 (25 m),
    // C1-continuous start poses computed from the previous element ends via
    // the same detmath the evaluator uses; small mismatches are inside the
    // reader's s bookkeeping tolerance.
    return wrap("<road id=\"1\" length=\"95\"><planView>"
                "<geometry s=\"0\" x=\"0\" y=\"0\" hdg=\"0\" length=\"40\"><line/></geometry>"
                "<geometry s=\"40\" x=\"40\" y=\"0\" hdg=\"0\" length=\"30\">"
                "<spiral curvStart=\"0\" curvEnd=\"0.02\"/></geometry>"
                "<geometry s=\"70\" x=\"69.7311\" y=\"2.9807\" hdg=\"0.3\" length=\"25\">"
                "<arc curvature=\"0.02\"/></geometry>"
                "</planView></road>");
}

TEST(OpenDriveGeometryTest, ProjectInvertsPoseAtAcrossElements) {
    const Map map = load_ok(three_element_road());
    const ReferenceLine line(map.roads.at("1"));
    ASSERT_TRUE(line.ok());
    EXPECT_DOUBLE_EQ(line.length(), 95.0);

    for (const double s : {0.5, 12.0, 39.9, 41.0, 55.0, 69.5, 71.0, 94.5}) {
        const RefPose pose = line.pose_at(s);
        const auto track = line.project(pose.x, pose.y);
        ASSERT_TRUE(track.has_value()) << "s=" << s;
        EXPECT_NEAR(track->s, s, 1e-6) << "s=" << s;
        EXPECT_NEAR(track->t, 0.0, 1e-6) << "s=" << s;
    }
}

TEST(OpenDriveGeometryTest, ProjectSignsTPositiveLeft) {
    const Map map =
        load_ok(wrap("<road id=\"1\" length=\"100\"><planView>"
                     "<geometry s=\"0\" x=\"0\" y=\"0\" hdg=\"0\" length=\"100\"><line/></geometry>"
                     "</planView></road>"));
    const ReferenceLine line(map.roads.at("1"));
    ASSERT_TRUE(line.ok());

    // Heading +x: +y is to the left (t > 0, §8.3), -y to the right.
    const auto left = line.project(30.0, 2.5);
    ASSERT_TRUE(left.has_value());
    EXPECT_NEAR(left->s, 30.0, 1e-6);
    EXPECT_NEAR(left->t, 2.5, 1e-9);

    const auto right = line.project(64.0, -1.25);
    ASSERT_TRUE(right.has_value());
    EXPECT_NEAR(right->s, 64.0, 1e-6);
    EXPECT_NEAR(right->t, -1.25, 1e-9);
}

TEST(OpenDriveGeometryTest, ProjectRejectsNonFiniteInput) {
    const Map map =
        load_ok(wrap("<road id=\"1\" length=\"10\"><planView>"
                     "<geometry s=\"0\" x=\"0\" y=\"0\" hdg=\"0\" length=\"10\"><line/></geometry>"
                     "</planView></road>"));
    const ReferenceLine line(map.roads.at("1"));
    const double nan = std::nan("");
    EXPECT_FALSE(line.project(nan, 0.0).has_value());
    EXPECT_FALSE(line.project(0.0, nan).has_value());
}

// --- determinism -----------------------------------------------------------

TEST(OpenDriveGeometryTest, RepeatedLoadAndEvalIsBitIdentical) {
    const std::string xml = three_element_road();
    const Map first_map = load_ok(xml);
    const Map second_map = load_ok(xml);
    const ReferenceLine first(first_map.roads.at("1"));
    const ReferenceLine second(second_map.roads.at("1"));
    for (int i = 0; i <= 95; ++i) {
        const double s = static_cast<double>(i);
        const RefPose a = first.pose_at(s);
        const RefPose b = second.pose_at(s);
        EXPECT_EQ(hex_bits(a.x), hex_bits(b.x)) << "s=" << s;
        EXPECT_EQ(hex_bits(a.y), hex_bits(b.y)) << "s=" << s;
        EXPECT_EQ(hex_bits(a.heading), hex_bits(b.heading)) << "s=" << s;
        const auto ta = first.project(a.x + 1.5, a.y - 0.5);
        const auto tb = second.project(b.x + 1.5, b.y - 0.5);
        ASSERT_EQ(ta.has_value(), tb.has_value());
        if (ta.has_value()) {
            EXPECT_EQ(hex_bits(ta->s), hex_bits(tb->s));
            EXPECT_EQ(hex_bits(ta->t), hex_bits(tb->t));
        }
    }
}

// --- reader diagnostics ----------------------------------------------------

TEST(OpenDriveReaderTest, UnconsumedFeaturesAreWarnedNeverSilent) {
    Map map;
    DiagnosticSink sink;
    const Status status = scena::opendrive::load_string(
        "<?xml version=\"1.0\"?><OpenDRIVE>"
        "<header revMajor=\"1\" revMinor=\"8\"><geoReference><![CDATA[+proj=utm]]>"
        "</geoReference></header>"
        "<road id=\"1\" length=\"10\">"
        "<link/><elevationProfile/><lateralProfile/><lanes/><objects/><signals/>"
        "<planView><geometry s=\"0\" x=\"0\" y=\"0\" hdg=\"0\" length=\"10\">"
        "<line/></geometry></planView>"
        "</road>"
        "<junction id=\"j\"/>"
        "</OpenDRIVE>",
        map, sink);
    ASSERT_EQ(status, Status::Ok);
    ASSERT_EQ(map.roads.size(), 1U);

    // geoReference + 6 road children + junction, each its own warning.
    int unsupported = 0;
    for (const auto& d : sink.diagnostics()) {
        if (d.severity == Severity::Warning && d.code == Status::UnsupportedFeature) {
            ++unsupported;
        }
    }
    EXPECT_EQ(unsupported, 8);
}

TEST(OpenDriveReaderTest, MissingPlanViewIsAnError) {
    Map map;
    DiagnosticSink sink;
    const Status status =
        scena::opendrive::load_string(wrap("<road id=\"1\" length=\"10\"/>"), map, sink);
    EXPECT_EQ(status, Status::ValidationError);
    EXPECT_TRUE(has_diagnostic(sink, Severity::Error, Status::ValidationError,
                               "asam.net:xodr:1.4.0:road.geometry.refline_exists"));
}

TEST(OpenDriveReaderTest, GeometrySBookkeepingIsEnforced) {
    Map map;
    DiagnosticSink sink;
    // Second element claims s=5 but 10 m of geometry precede it.
    const Status status = scena::opendrive::load_string(
        wrap("<road id=\"1\" length=\"20\"><planView>"
             "<geometry s=\"0\" x=\"0\" y=\"0\" hdg=\"0\" length=\"10\"><line/></geometry>"
             "<geometry s=\"5\" x=\"10\" y=\"0\" hdg=\"0\" length=\"10\"><line/></geometry>"
             "</planView></road>"),
        map, sink);
    EXPECT_EQ(status, Status::ValidationError);
    EXPECT_TRUE(has_diagnostic(sink, Severity::Error, Status::ValidationError,
                               "asam.net:xodr:1.9.0:road.geometry.s-value_sum"));
}

TEST(OpenDriveReaderTest, DescendingSOrderIsAnError) {
    Map map;
    DiagnosticSink sink;
    const Status status = scena::opendrive::load_string(
        wrap("<road id=\"1\" length=\"30\"><planView>"
             "<geometry s=\"0\" x=\"0\" y=\"0\" hdg=\"0\" length=\"10\"><line/></geometry>"
             "<geometry s=\"10\" x=\"10\" y=\"0\" hdg=\"0\" length=\"10\"><line/></geometry>"
             "<geometry s=\"3\" x=\"20\" y=\"0\" hdg=\"0\" length=\"10\"><line/></geometry>"
             "</planView></road>"),
        map, sink);
    EXPECT_EQ(status, Status::ValidationError);
    EXPECT_TRUE(has_diagnostic(sink, Severity::Error, Status::ValidationError,
                               "asam.net:xodr:1.4.0:road.geometry.elem_asc_order"));
}

TEST(OpenDriveReaderTest, TwoPrimitivesInOneGeometryIsAnError) {
    Map map;
    DiagnosticSink sink;
    const Status status = scena::opendrive::load_string(
        wrap("<road id=\"1\" length=\"10\"><planView>"
             "<geometry s=\"0\" x=\"0\" y=\"0\" hdg=\"0\" length=\"10\">"
             "<line/><arc curvature=\"0.1\"/></geometry>"
             "</planView></road>"),
        map, sink);
    EXPECT_EQ(status, Status::ValidationError);
    EXPECT_TRUE(has_diagnostic(sink, Severity::Error, Status::ValidationError,
                               "asam.net:xodr:1.4.0:road.geometry.one_geom_elem_per_spec"));
}

TEST(OpenDriveReaderTest, LocaleStyleCommaNumberIsAParseError) {
    Map map;
    DiagnosticSink sink;
    // "1,5" is a locale spelling, never valid input: parsing is from_chars
    // only (cross-platform rule), so this must fail identically everywhere.
    const Status status = scena::opendrive::load_string(
        wrap("<road id=\"1\" length=\"10\"><planView>"
             "<geometry s=\"0\" x=\"1,5\" y=\"0\" hdg=\"0\" length=\"10\"><line/></geometry>"
             "</planView></road>"),
        map, sink);
    EXPECT_EQ(status, Status::ParseError);
}

TEST(OpenDriveReaderTest, MalformedXmlIsAParseError) {
    Map map;
    DiagnosticSink sink;
    EXPECT_EQ(scena::opendrive::load_string("<OpenDRIVE><road", map, sink), Status::ParseError);
    EXPECT_TRUE(sink.has_errors());
}

TEST(OpenDriveReaderTest, DuplicateRoadIdIsAnError) {
    Map map;
    DiagnosticSink sink;
    const std::string road =
        "<road id=\"1\" length=\"10\"><planView>"
        "<geometry s=\"0\" x=\"0\" y=\"0\" hdg=\"0\" length=\"10\"><line/></geometry>"
        "</planView></road>";
    EXPECT_EQ(scena::opendrive::load_string(wrap(road + road), map, sink), Status::ValidationError);
}

TEST(OpenDriveReaderTest, LoadFileStampsThePathAndMatchesLoadString) {
    namespace fs = std::filesystem;
    const fs::path path = fs::path(::testing::TempDir()) / "scena_p3s2_line.xodr";
    {
        std::ofstream out(path, std::ios::binary);
        out << wrap("<road id=\"1\" length=\"10\"><planView>"
                    "<geometry s=\"0\" x=\"0\" y=\"0\" hdg=\"0\" length=\"10\">"
                    "<line/></geometry></planView></road>");
    }
    Map map;
    DiagnosticSink sink;
    EXPECT_EQ(scena::opendrive::load_file(path, map, sink), Status::Ok);
    EXPECT_EQ(map.roads.size(), 1U);
    std::error_code ec;
    fs::remove(path, ec);
}

TEST(OpenDriveReaderTest, MissingFileIsInvalidArgument) {
    namespace fs = std::filesystem;
    Map map;
    DiagnosticSink sink;
    EXPECT_EQ(scena::opendrive::load_file(
                  fs::path(::testing::TempDir()) / "scena_p3s2_does_not_exist.xodr", map, sink),
              Status::InvalidArgument);
    ASSERT_FALSE(sink.diagnostics().empty());
    EXPECT_FALSE(sink.diagnostics().front().location.file.empty());
}

} // namespace
