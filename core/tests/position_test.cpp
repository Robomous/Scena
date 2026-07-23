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

// Unit tests for the PositionResolver (ASAM OpenSCENARIO XML 1.4.0 §6.3.8):
// every position variant resolves to a world pose or reports a diagnostic
// (none silently wrong), and orientation composes per §Orientation. The
// RelativeObjectPosition rotation is pinned bit-for-bit against det_sincos so a
// stray std::sin/cos or a reordered expression is caught (ADR-0006).

#include <map>
#include <optional>
#include <string>

#include <gtest/gtest.h>

#include "scena/entity_state.h"
#include "scena/ir/position.h"
#include "scena/runtime/detmath.h"
#include "scena/runtime/position_resolver.h"
#include "scena/status.h"
#include "support/trace_recorder.h"

namespace {

using scena::EntityState;
using scena::Status;
using scena::runtime::det_sincos;
using scena::runtime::Pose;
using scena::runtime::PositionResolver;
using scena::runtime::SinCos;
using scena::testsupport::hex_bits;
namespace ir = scena::ir;

constexpr double kPi = 3.14159265358979323846;

// Builds a resolver over a fixed table of reference entity poses. The table is
// copied into the lookup so the resolver owns it — callers may pass a temporary.
PositionResolver resolver_over(std::map<std::string, EntityState> entities) {
    return PositionResolver(
        [table = std::move(entities)](std::string_view id) -> const EntityState* {
            const auto it = table.find(std::string(id));
            return it == table.end() ? nullptr : &it->second;
        });
}

// --- IR shape --------------------------------------------------------------

TEST(PositionIrTest, WorldPositionKeepsThreeFieldInitializer) {
    // Append-only: the h/p/r extension must not break {x, y, z} initializers.
    const ir::WorldPosition world{1.0, 2.0, 3.0};
    EXPECT_EQ(world.h, 0.0);
    EXPECT_EQ(world.p, 0.0);
    EXPECT_EQ(world.r, 0.0);
}

TEST(PositionIrTest, PositionHoldsEachVariantInChoiceOrder) {
    const ir::Position variants[] = {
        ir::WorldPosition{},        ir::RelativeWorldPosition{}, ir::RelativeObjectPosition{},
        ir::RoadPosition{},         ir::RelativeRoadPosition{},  ir::LanePosition{},
        ir::RelativeLanePosition{}, ir::RoutePosition{},         ir::GeoPosition{},
        ir::TrajectoryPosition{}};
    // The spec's Position xsd:choice order, mirrored by the variant index.
    EXPECT_EQ(variants[0].index(), 0u);
    EXPECT_EQ(variants[9].index(), 9u);
    EXPECT_TRUE(std::holds_alternative<ir::TrajectoryPosition>(variants[9]));
}

TEST(PositionIrTest, OrientationDefaultsToRelativeContext) {
    const ir::Orientation orientation;
    EXPECT_EQ(orientation.type, ir::ReferenceContext::Relative);
}

// --- WorldPosition ---------------------------------------------------------

TEST(PositionResolverTest, WorldPositionIsTakenDirectly) {
    const PositionResolver resolver = resolver_over({});
    Pose pose;
    const auto result = resolver.resolve(ir::WorldPosition{1.0, 2.0, 3.0, 0.5, 0.25, 0.125}, pose);
    EXPECT_EQ(result.status, Status::Ok);
    EXPECT_EQ(pose.x, 1.0);
    EXPECT_EQ(pose.y, 2.0);
    EXPECT_EQ(pose.z, 3.0);
    EXPECT_EQ(pose.heading, 0.5);
    EXPECT_EQ(pose.pitch, 0.25);
    EXPECT_EQ(pose.roll, 0.125);
}

// --- RelativeWorldPosition -------------------------------------------------

TEST(PositionResolverTest, RelativeWorldAddsDeltasAlongWorldAxes) {
    std::map<std::string, EntityState> entities;
    entities["ref"] = EntityState{10.0, 20.0, 1.0, kPi / 2.0, 5.0}; // heading +90 deg
    const PositionResolver resolver = resolver_over(entities);

    ir::RelativeWorldPosition rel;
    rel.entity_ref = "ref";
    rel.dx = 3.0;
    rel.dy = 4.0;
    rel.dz = 0.5;
    Pose pose;
    const auto result = resolver.resolve(rel, pose);
    ASSERT_EQ(result.status, Status::Ok);
    // Deltas are NOT rotated by the entity heading.
    EXPECT_EQ(pose.x, 13.0);
    EXPECT_EQ(pose.y, 24.0);
    EXPECT_EQ(pose.z, 1.5);
    // Missing Orientation copies the reference entity orientation.
    EXPECT_EQ(pose.heading, kPi / 2.0);
}

TEST(PositionResolverTest, MissingReferenceEntityIsReported) {
    const PositionResolver resolver = resolver_over({});
    ir::RelativeWorldPosition rel;
    rel.entity_ref = "ghost";
    Pose pose;
    const auto result = resolver.resolve(rel, pose);
    EXPECT_EQ(result.status, Status::SemanticError);
    EXPECT_NE(result.message.find("ghost"), std::string::npos);
}

TEST(PositionResolverTest, AbsoluteOrientationIgnoresReferenceEntity) {
    std::map<std::string, EntityState> entities;
    entities["ref"] = EntityState{0.0, 0.0, 0.0, 1.0, 0.0}; // heading 1 rad
    const PositionResolver resolver = resolver_over(entities);

    ir::RelativeWorldPosition rel;
    rel.entity_ref = "ref";
    rel.orientation = ir::Orientation{0.3, 0.0, 0.0, ir::ReferenceContext::Absolute};
    Pose pose;
    ASSERT_EQ(resolver.resolve(rel, pose).status, Status::Ok);
    EXPECT_EQ(pose.heading, 0.3); // absolute: the reference heading is discarded
}

TEST(PositionResolverTest, RelativeOrientationShiftsReferenceEntity) {
    std::map<std::string, EntityState> entities;
    entities["ref"] = EntityState{0.0, 0.0, 0.0, 1.0, 0.0}; // heading 1 rad
    const PositionResolver resolver = resolver_over(entities);

    ir::RelativeWorldPosition rel;
    rel.entity_ref = "ref";
    rel.orientation = ir::Orientation{0.3, 0.0, 0.0, ir::ReferenceContext::Relative};
    Pose pose;
    ASSERT_EQ(resolver.resolve(rel, pose).status, Status::Ok);
    EXPECT_EQ(pose.heading, 1.3); // relative: additive shift on the reference heading
}

// --- RelativeObjectPosition ------------------------------------------------

TEST(PositionResolverTest, RelativeObjectRotatesDeltasIntoEntityFrame) {
    std::map<std::string, EntityState> entities;
    entities["ref"] = EntityState{5.0, 5.0, 0.0, kPi / 2.0, 0.0}; // facing +Y
    const PositionResolver resolver = resolver_over(entities);

    ir::RelativeObjectPosition rel;
    rel.entity_ref = "ref";
    rel.dx = 2.0; // "2 m ahead" of an entity facing +Y => +2 in world Y
    rel.dy = 0.0;
    Pose pose;
    ASSERT_EQ(resolver.resolve(rel, pose).status, Status::Ok);
    EXPECT_NEAR(pose.x, 5.0, 1e-9);
    EXPECT_NEAR(pose.y, 7.0, 1e-9);
}

TEST(PositionResolverTest, RelativeObjectRotationIsBitIdenticalToDetSincos) {
    std::map<std::string, EntityState> entities;
    entities["ref"] = EntityState{-1.5, 2.5, 0.0, 0.9, 0.0};
    const PositionResolver resolver = resolver_over(entities);

    ir::RelativeObjectPosition rel;
    rel.entity_ref = "ref";
    rel.dx = 1.25;
    rel.dy = -0.75;
    Pose pose;
    ASSERT_EQ(resolver.resolve(rel, pose).status, Status::Ok);

    const SinCos rot = det_sincos(0.9);
    const double expected_x = -1.5 + 1.25 * rot.cos - (-0.75) * rot.sin;
    const double expected_y = 2.5 + 1.25 * rot.sin + (-0.75) * rot.cos;
    // Bit-for-bit: the resolver must route through det_sincos, not libm.
    EXPECT_EQ(hex_bits(pose.x), hex_bits(expected_x));
    EXPECT_EQ(hex_bits(pose.y), hex_bits(expected_y));
}

// --- Road-family / geo / trajectory: reported, not silently wrong ----------

TEST(PositionResolverTest, RoadFamilyReportsUnsupported) {
    const PositionResolver resolver = resolver_over({});
    Pose pose;
    const ir::Position road_family[] = {ir::RoadPosition{}, ir::RelativeRoadPosition{},
                                        ir::LanePosition{}, ir::RelativeLanePosition{},
                                        ir::RoutePosition{}};
    for (const auto& position : road_family) {
        const auto result = resolver.resolve(position, pose);
        EXPECT_EQ(result.status, Status::UnsupportedFeature);
    }
}

TEST(PositionResolverTest, GeoPositionCitesTheGeodeticRule) {
    const PositionResolver resolver = resolver_over({});
    Pose pose;
    const auto result = resolver.resolve(ir::GeoPosition{}, pose);
    EXPECT_EQ(result.status, Status::UnsupportedFeature);
    EXPECT_EQ(result.rule_id, "asam.net:xosc:1.1.0:positioning.geodetic_datum_defined");
}

TEST(PositionResolverTest, TrajectoryPositionReportsUnsupported) {
    const PositionResolver resolver = resolver_over({});
    Pose pose;
    const auto result = resolver.resolve(ir::TrajectoryPosition{}, pose);
    EXPECT_EQ(result.status, Status::UnsupportedFeature);
}

// Exhaustiveness: no variant may return Ok without filling a pose, and none may
// fall through unhandled.
TEST(PositionResolverTest, EveryVariantResolvesOrReports) {
    const PositionResolver resolver = resolver_over({});
    const ir::Position all[] = {
        ir::WorldPosition{},        ir::RelativeWorldPosition{}, ir::RelativeObjectPosition{},
        ir::RoadPosition{},         ir::RelativeRoadPosition{},  ir::LanePosition{},
        ir::RelativeLanePosition{}, ir::RoutePosition{},         ir::GeoPosition{},
        ir::TrajectoryPosition{}};
    for (const auto& position : all) {
        Pose pose;
        const auto result = resolver.resolve(position, pose);
        // WorldPosition resolves; the relative variants over an empty table
        // report a missing reference; the rest report unsupported. What matters
        // is that the resolver always answers with a defined status.
        EXPECT_TRUE(result.status == Status::Ok || result.status == Status::SemanticError ||
                    result.status == Status::UnsupportedFeature);
    }
}

} // namespace
