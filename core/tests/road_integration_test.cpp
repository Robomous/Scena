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

// p3-s4: roads wired into the execution path. The test acts as a host: it
// loads the hand-authored map fixtures (GS-8 prerequisites, core/tests/maps)
// into the OpenDRIVE backend and injects it through the gateway, then drives
// road-relative teleports, road-coordinate distances, the road-predicate
// conditions, and the determinism fixture on the curve map.

#include <cmath>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "scena/engine.h"
#include "scena/entity_state.h"
#include "scena/gateway/road_query.h"
#include "scena/gateway/simulator_gateway.h"
#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/entity.h"
#include "scena/ir/evaluation_context.h"
#include "scena/ir/interaction_condition.h"
#include "scena/ir/position.h"
#include "scena/ir/route.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/trigger.h"
#include "scena/opendrive/reader.h"
#include "scena/opendrive/road_query.h"
#include "scena/runtime/detmath.h"
#include "scena/runtime/distance_measure.h"
#include "scena/runtime/position_resolver.h"
#include "support/trace_recorder.h"

namespace {

using scena::Engine;
using scena::EntityState;
using scena::Status;
using scena::opendrive::OpenDriveRoadQuery;
using scena::runtime::Pose;
using scena::runtime::PositionResolver;
using scena::testsupport::hex_bits;

std::unique_ptr<OpenDriveRoadQuery> load_map(const char* name) {
    const std::filesystem::path path = std::filesystem::path(SCENA_TEST_MAP_DIR) / name;
    scena::opendrive::Map map;
    scena::DiagnosticSink sink;
    const Status status = scena::opendrive::load_file(path, map, sink);
    EXPECT_EQ(status, Status::Ok) << path << ": "
                                  << (sink.diagnostics().empty()
                                          ? std::string("no diagnostics")
                                          : sink.diagnostics().front().message);
    return std::make_unique<OpenDriveRoadQuery>(std::move(map));
}

/// The host side of the test: a gateway whose only job is handing the road
/// network to the engine.
class RoadGateway final : public scena::gateway::ISimulatorGateway {
public:
    explicit RoadGateway(std::unique_ptr<OpenDriveRoadQuery> road) : road_(std::move(road)) {}

    void publish_state(const std::string& /*entity_id*/, const EntityState& /*state*/) override {}
    bool poll_state(const std::string& /*entity_id*/, EntityState& /*out*/) override {
        return false;
    }
    scena::gateway::IRoadQuery* road_query() override { return road_.get(); }

private:
    std::unique_ptr<OpenDriveRoadQuery> road_;
};

EntityState state_at(double x, double y, double heading = 0.0, double speed = 0.0) {
    EntityState state;
    state.x = x;
    state.y = y;
    state.z = 0.0;
    state.heading = heading;
    state.speed = speed;
    return state;
}

// --- PositionResolver: road-family variants against the backend ------------

class ResolverFixture : public ::testing::Test {
protected:
    ResolverFixture() : road_(load_map("straight.xodr")) {}

    [[nodiscard]] PositionResolver make_resolver() const {
        return PositionResolver(
            [this](std::string_view id) -> const EntityState* {
                const auto it = states_.find(std::string(id));
                return it != states_.end() ? &it->second : nullptr;
            },
            road_.get());
    }

    std::unique_ptr<OpenDriveRoadQuery> road_;
    std::map<std::string, EntityState> states_;
};

TEST_F(ResolverFixture, RoadPositionResolvesOnTheReferenceLine) {
    const PositionResolver resolver = make_resolver();
    Pose pose;
    scena::ir::RoadPosition target;
    target.road_id = "1";
    target.s = 30.0;
    target.t = -1.75;
    const auto result = resolver.resolve(scena::ir::Position(target), pose);
    ASSERT_EQ(result.status, Status::Ok) << result.message;
    EXPECT_NEAR(pose.x, 30.0, 1e-9);
    EXPECT_NEAR(pose.y, -1.75, 1e-9);
    EXPECT_DOUBLE_EQ(pose.heading, 0.0); // s-axis tangent of the straight road
}

TEST_F(ResolverFixture, LanePositionResolvesToTheLaneCenter) {
    const PositionResolver resolver = make_resolver();
    Pose pose;
    scena::ir::LanePosition target;
    target.road_id = "1";
    target.lane_id = "-1";
    target.s = 50.0;
    target.offset = 0.25;
    const auto result = resolver.resolve(scena::ir::Position(target), pose);
    ASSERT_EQ(result.status, Status::Ok) << result.message;
    EXPECT_NEAR(pose.x, 50.0, 1e-9);
    EXPECT_NEAR(pose.y, -1.75 + 0.25, 1e-9); // lane -1 centre -1.75, +offset left
}

TEST_F(ResolverFixture, RelativeRoadAndLanePositionsFollowTheReference) {
    states_["ref"] = state_at(20.0, -1.75);
    const PositionResolver resolver = make_resolver();

    Pose pose;
    scena::ir::RelativeRoadPosition road_rel;
    road_rel.entity_ref = "ref";
    road_rel.ds = 15.0;
    road_rel.dt = 1.0;
    auto result = resolver.resolve(scena::ir::Position(road_rel), pose);
    ASSERT_EQ(result.status, Status::Ok) << result.message;
    EXPECT_NEAR(pose.x, 35.0, 1e-6);
    EXPECT_NEAR(pose.y, -0.75, 1e-6);

    scena::ir::RelativeLanePosition lane_rel;
    lane_rel.entity_ref = "ref";
    lane_rel.d_lane = 1; // one lane to the left, skipping the centre: lane 1
    lane_rel.ds = 10.0;
    auto lane_result = resolver.resolve(scena::ir::Position(lane_rel), pose);
    ASSERT_EQ(lane_result.status, Status::Ok) << lane_result.message;
    EXPECT_NEAR(pose.x, 30.0, 1e-6);
    EXPECT_NEAR(pose.y, 1.75, 1e-6); // lane 1 centre line

    // dsLane stays outside the subset: reported, never silently approximated.
    lane_rel.ds.reset();
    lane_rel.ds_lane = 10.0;
    EXPECT_EQ(resolver.resolve(scena::ir::Position(lane_rel), pose).status,
              Status::UnsupportedFeature);

    // A delta leaving the road is reported (no continuation onto links).
    road_rel.ds = 500.0;
    EXPECT_EQ(resolver.resolve(scena::ir::Position(road_rel), pose).status,
              Status::UnsupportedFeature);
}

TEST_F(ResolverFixture, RoutePositionWalksTheRoute) {
    const PositionResolver resolver = make_resolver();
    auto route = std::make_shared<scena::ir::Route>();
    route->waypoints.push_back({scena::ir::WorldPosition{10.0, -1.75, 0.0}, {}});
    route->waypoints.push_back({scena::ir::WorldPosition{90.0, -1.75, 0.0}, {}});

    Pose pose;
    scena::ir::RoutePosition target;
    target.route = route;
    target.path_s = 25.0; // 25 m past the first waypoint at s = 10
    target.t = 0.5;
    const auto result = resolver.resolve(scena::ir::Position(target), pose);
    ASSERT_EQ(result.status, Status::Ok) << result.message;
    EXPECT_NEAR(pose.x, 35.0, 1e-6);
    EXPECT_NEAR(pose.y, 0.5, 1e-6);
    EXPECT_DOUBLE_EQ(pose.heading, 0.0);

    // Lane form: pathS + laneId + laneOffset (§PositionInLaneCoordinates).
    scena::ir::RoutePosition lane_form;
    lane_form.route = route;
    lane_form.path_s = 40.0;
    lane_form.lane_id = "1";
    lane_form.lane_offset = -0.25;
    const auto lane_result = resolver.resolve(scena::ir::Position(lane_form), pose);
    ASSERT_EQ(lane_result.status, Status::Ok) << lane_result.message;
    EXPECT_NEAR(pose.x, 50.0, 1e-6);
    EXPECT_NEAR(pose.y, 1.75 - 0.25, 1e-6);

    // Entity form: the entity's projection picks the pathS.
    states_["ego"] = state_at(60.0, -1.0);
    scena::ir::RoutePosition entity_form;
    entity_form.route = route;
    entity_form.from_entity = "ego";
    const auto entity_result = resolver.resolve(scena::ir::Position(entity_form), pose);
    ASSERT_EQ(entity_result.status, Status::Ok) << entity_result.message;
    EXPECT_NEAR(pose.x, 60.0, 1e-6);

    // Beyond the route: reported.
    target.path_s = 500.0;
    EXPECT_EQ(resolver.resolve(scena::ir::Position(target), pose).status, Status::SemanticError);
}

TEST_F(ResolverFixture, WithoutARoadNetworkRoadVariantsStayUnsupported) {
    const PositionResolver resolver([](std::string_view) -> const EntityState* { return nullptr; });
    Pose pose;
    scena::ir::RoadPosition target;
    target.road_id = "1";
    const auto result = resolver.resolve(scena::ir::Position(target), pose);
    EXPECT_EQ(result.status, Status::UnsupportedFeature);
}

// --- road-coordinate distance measurement ----------------------------------

TEST(RoadDistanceTest, ReferencePointAndFreespaceGapsOnTheStraightMap) {
    const auto road = load_map("straight.xodr");

    scena::ir::EntityKinematics a;
    a.state = state_at(20.0, -1.75);
    scena::ir::EntityKinematics b;
    b.state = state_at(50.0, -1.0);

    scena::runtime::DistanceSpec spec;
    spec.cs = scena::ir::CoordinateSystem::Road;
    spec.rdt = scena::ir::RelativeDistanceType::Longitudinal;
    spec.road = road.get();

    auto measured = scena::runtime::measure_distance(a, &b, {}, spec);
    ASSERT_TRUE(measured.has_value());
    EXPECT_NEAR(*measured, 30.0, 1e-6);

    spec.rdt = scena::ir::RelativeDistanceType::Lateral;
    measured = scena::runtime::measure_distance(a, &b, {}, spec);
    ASSERT_TRUE(measured.has_value());
    EXPECT_NEAR(*measured, 0.75, 1e-6);

    // Freespace: 4.6 m long boxes shrink the 30 m gap by a box length.
    scena::ir::BoundingBox box;
    box.length = 4.6;
    box.width = 2.0;
    a.bounding_box = box;
    b.bounding_box = box;
    spec.rdt = scena::ir::RelativeDistanceType::Longitudinal;
    spec.freespace = true;
    measured = scena::runtime::measure_distance(a, &b, {}, spec);
    ASSERT_TRUE(measured.has_value());
    EXPECT_NEAR(*measured, 30.0 - 4.6, 1e-6);

    // Signed action gaps share the kernel: reference ahead is positive.
    const auto signed_gap = scena::runtime::road_signed_longitudinal_gap(
        road.get(), a.state, a.bounding_box, b.state, b.bounding_box, true);
    ASSERT_TRUE(signed_gap.has_value());
    EXPECT_NEAR(*signed_gap, 30.0 - 4.6, 1e-6);

    const auto lateral_gap = scena::runtime::road_signed_lateral_gap(
        road.get(), a.state, a.bounding_box, b.state, b.bounding_box, false);
    ASSERT_TRUE(lateral_gap.has_value());
    EXPECT_NEAR(*lateral_gap, -0.75, 1e-6); // actor right of reference
}

TEST(RoadDistanceTest, OffNetworkOrCrossRoadMeasurementsStayDeferred) {
    const auto road = load_map("junction.xodr");
    scena::ir::EntityKinematics a;
    a.state = state_at(20.0, -1.75); // road 1
    scena::ir::EntityKinematics b;
    b.state = state_at(160.0, -1.75); // road 4

    scena::runtime::DistanceSpec spec;
    spec.cs = scena::ir::CoordinateSystem::Road;
    spec.rdt = scena::ir::RelativeDistanceType::Longitudinal;
    spec.road = road.get();
    EXPECT_FALSE(scena::runtime::measure_distance(a, &b, {}, spec).has_value());

    b.state = state_at(20.0, 100.0); // far off every road
    EXPECT_FALSE(scena::runtime::measure_distance(a, &b, {}, spec).has_value());
}

// --- engine integration: teleports, conditions, determinism ----------------

scena::ir::Scenario one_entity_scenario(std::optional<scena::ir::Position> init_teleport,
                                        double init_speed = 0.0) {
    scena::ir::Scenario scenario;
    scenario.name = "road-integration";
    scena::ir::Entity ego;
    ego.id = "ego";
    ego.name = "ego";
    ego.control_mode = scena::ir::ControlMode::EngineControlled;
    scenario.entities.push_back(std::move(ego));
    if (init_teleport.has_value()) {
        scenario.init_actions.push_back(
            std::make_shared<scena::ir::TeleportAction>("ego", *init_teleport));
    }
    if (init_speed != 0.0) {
        scenario.init_actions.push_back(
            std::make_shared<scena::ir::SpeedAction>("ego", init_speed));
    }
    return scenario;
}

/// Adds one event triggered by `condition` whose action teleports ego to the
/// world-frame marker (1000, 1000): observing the marker observes the
/// condition having fired.
void add_marker_event(scena::ir::Scenario& scenario,
                      std::shared_ptr<scena::ir::Condition> condition) {
    scena::ir::Event event;
    event.name = "marker";
    event.start_trigger =
        scena::ir::make_trigger(std::move(condition), scena::ir::ConditionEdge::None, 0.0);
    event.actions.push_back(std::make_shared<scena::ir::TeleportAction>(
        "ego", scena::ir::WorldPosition{1000.0, 1000.0, 0.0}));
    scena::ir::Maneuver maneuver;
    maneuver.name = "maneuver";
    maneuver.events.push_back(std::move(event));
    scena::ir::ManeuverGroup group;
    group.name = "group";
    group.actors.push_back("ego");
    group.maneuvers.push_back(std::move(maneuver));
    scena::ir::Act act;
    act.name = "act";
    act.groups.push_back(std::move(group));
    scena::ir::Story story;
    story.name = "story";
    story.acts.push_back(std::move(act));
    scenario.storyboard.stories.push_back(std::move(story));
}

TEST(RoadIntegrationTest, LanePositionTeleportLandsOnTheCurveMap) {
    RoadGateway gateway(load_map("curve.xodr"));
    Engine engine(&gateway);

    scena::ir::LanePosition target;
    target.road_id = "1";
    target.lane_id = "-1";
    target.s = 80.0; // on the arc element
    ASSERT_EQ(engine.init(one_entity_scenario(scena::ir::Position(target))), Status::Ok);

    const auto state = engine.state("ego");
    ASSERT_TRUE(state.has_value());
    // The pose must be exactly what the backend computes for the same
    // coordinates: same conversions, same detmath.
    const auto reference_road = load_map("curve.xodr");
    double center_t = 0.0;
    ASSERT_TRUE(reference_road->lane_center_offset("1", -1, 80.0, center_t));
    scena::gateway::LanePosition lane;
    lane.road_id = "1";
    lane.lane_id = -1;
    lane.s = 80.0;
    lane.t = center_t;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    ASSERT_TRUE(reference_road->to_world_position(lane, x, y, z));
    EXPECT_EQ(hex_bits(state->x), hex_bits(x));
    EXPECT_EQ(hex_bits(state->y), hex_bits(y));
    double heading = 0.0;
    ASSERT_TRUE(reference_road->road_heading("1", 80.0, heading));
    EXPECT_EQ(hex_bits(state->heading), hex_bits(heading));
}

TEST(RoadIntegrationTest, OffroadConditionFiresWhenTheEntityLeavesTheMap) {
    RoadGateway gateway(load_map("straight.xodr"));
    Engine engine(&gateway);
    // Start near the road end on lane -1, driving +s at 10 m/s: the entity
    // leaves the network past s = 100 and the offroad clock starts.
    scena::ir::LanePosition start;
    start.road_id = "1";
    start.lane_id = "-1";
    start.s = 95.0;
    auto scenario = one_entity_scenario(scena::ir::Position(start), 10.0);
    add_marker_event(
        scenario,
        std::make_shared<scena::ir::OffroadCondition>(
            scena::ir::TriggeringEntities{scena::ir::TriggeringEntitiesRule::Any, {"ego"}}, 0.3));
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);

    bool fired = false;
    for (int i = 0; i < 30 && !fired; ++i) {
        ASSERT_EQ(engine.step(0.1), Status::Ok);
        const auto state = engine.state("ego");
        ASSERT_TRUE(state.has_value());
        // The teleport lands at (1000, 1000); the entity keeps integrating
        // +x afterwards, but y stays put — the stable marker.
        fired = state->y == 1000.0;
    }
    EXPECT_TRUE(fired); // off-road for 0.3 s => the marker teleport ran
}

TEST(RoadIntegrationTest, EndOfRoadConditionFiresAtTheRoadBoundary) {
    RoadGateway gateway(load_map("straight.xodr"));
    Engine engine(&gateway);
    // Parked right at the road end, facing +s: at the end from t = 0, so the
    // duration clock alone decides when the condition fires.
    scena::ir::LanePosition start;
    start.road_id = "1";
    start.lane_id = "-1";
    start.s = 100.0;
    auto scenario = one_entity_scenario(scena::ir::Position(start), 0.0);
    add_marker_event(
        scenario,
        std::make_shared<scena::ir::EndOfRoadCondition>(
            scena::ir::TriggeringEntities{scena::ir::TriggeringEntitiesRule::Any, {"ego"}}, 0.4));
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);

    int steps_to_fire = 0;
    for (int i = 1; i <= 20; ++i) {
        ASSERT_EQ(engine.step(0.1), Status::Ok);
        const auto state = engine.state("ego");
        ASSERT_TRUE(state.has_value());
        if (state->y == 1000.0) {
            steps_to_fire = i;
            break;
        }
    }
    // 0.4 s of at-end time needs 4 steps of 0.1 s; the trigger evaluates at
    // the step after the clock crosses the threshold.
    EXPECT_GE(steps_to_fire, 4);
    EXPECT_LE(steps_to_fire, 6);
    EXPECT_NE(steps_to_fire, 0);
}

TEST(RoadIntegrationTest, CurveMapRunIsBitIdentical) {
    // The determinism fixture on the curve map: identical scenario +
    // identical steps => bit-identical states, with the road backend in the
    // loop (teleport resolution + road clocks each step).
    const auto run = [] {
        RoadGateway gateway(load_map("curve.xodr"));
        Engine engine(&gateway);
        scena::ir::LanePosition start;
        start.road_id = "1";
        start.lane_id = "-1";
        start.s = 10.0;
        auto scenario = one_entity_scenario(scena::ir::Position(start), 8.0);
        EXPECT_EQ(engine.init(std::move(scenario)), Status::Ok);
        std::vector<std::string> bits;
        for (int i = 0; i < 50; ++i) {
            EXPECT_EQ(engine.step(0.05), Status::Ok);
            const auto state = engine.state("ego");
            EXPECT_TRUE(state.has_value());
            bits.push_back(hex_bits(state->x) + hex_bits(state->y) + hex_bits(state->heading) +
                           hex_bits(state->speed));
        }
        return bits;
    };
    const std::vector<std::string> first = run();
    const std::vector<std::string> second = run();
    ASSERT_EQ(first.size(), second.size());
    for (std::size_t i = 0; i < first.size(); ++i) {
        EXPECT_EQ(first[i], second[i]) << "step " << i;
    }
}

} // namespace
