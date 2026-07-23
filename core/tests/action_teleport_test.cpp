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

//
// TeleportAction (p5-s4, generalized in p2-s4): teleport to any §6.3.8 Position.
// A one-shot action that resolves the target through the PositionResolver and
// writes the entity's world position and orientation, completing in the
// evaluation it fires. Self-contained variants (world, relative-world,
// relative-object) resolve; road/route/geo/trajectory targets are reported and
// the teleport is a no-op until their backends land (ADR-0017). Control-
// ownership (host-controlled) mode violations live in control_ownership_test.

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>

#include <gtest/gtest.h>

#include "scena/engine.h"
#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/entity.h"
#include "scena/ir/position.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/trigger.h"

namespace {

using scena::Engine;
using scena::EntityState;
using scena::Status;
using scena::ir::ControlMode;
using scena::ir::Entity;
using scena::ir::Position;
using scena::ir::Scenario;
using scena::ir::SimulationTimeCondition;
using scena::ir::TeleportAction;
using scena::ir::WorldPosition;

// --- IR surface -----------------------------------------------------------

TEST(TeleportActionIrTest, CarriesEntityAndWorldPosition) {
    TeleportAction action("ego", WorldPosition{10.0, -4.0, 1.5});
    EXPECT_EQ(action.entity_id(), "ego");
    EXPECT_EQ(action.kind(), "TeleportAction");
    const auto& world = std::get<WorldPosition>(action.position());
    EXPECT_DOUBLE_EQ(world.x, 10.0);
    EXPECT_DOUBLE_EQ(world.y, -4.0);
    EXPECT_DOUBLE_EQ(world.z, 1.5);
}

// --- Engine integration ---------------------------------------------------

// Scenario with one engine-controlled "ego"; `init_teleport` (when set) runs as
// an init action, and `timed_teleport` (when set) fires at t=1 via a
// SimulationTimeCondition.
Scenario make_teleport_scenario(std::optional<Position> init_teleport,
                                std::optional<Position> timed_teleport) {
    Scenario scenario;
    scenario.name = "teleport-integration";
    Entity ego;
    ego.id = "ego";
    ego.name = "ego";
    ego.control_mode = ControlMode::EngineControlled;
    scenario.entities.push_back(std::move(ego));

    if (init_teleport.has_value()) {
        scenario.init_actions.push_back(std::make_shared<TeleportAction>("ego", *init_teleport));
    }
    if (timed_teleport.has_value()) {
        scena::ir::Event event;
        event.name = "jump";
        event.start_trigger = scena::ir::make_trigger(
            std::make_shared<SimulationTimeCondition>(1.0), scena::ir::ConditionEdge::None, 0.0);
        event.actions.push_back(std::make_shared<TeleportAction>("ego", *timed_teleport));
        scena::ir::Maneuver maneuver;
        maneuver.name = "maneuver";
        maneuver.events.push_back(std::move(event));
        scena::ir::ManeuverGroup group;
        group.name = "group";
        group.maneuvers.push_back(std::move(maneuver));
        scena::ir::Act act;
        act.name = "act";
        act.groups.push_back(std::move(group));
        scena::ir::Story story;
        story.name = "story";
        story.acts.push_back(std::move(act));
        scenario.storyboard.stories.push_back(std::move(story));
    }
    return scenario;
}

TEST(TeleportActionEngineTest, InitTeleportSetsInitialWorldPosition) {
    Engine engine;
    ASSERT_EQ(engine.init(make_teleport_scenario(WorldPosition{25.0, 3.0, 0.0}, std::nullopt)),
              Status::Ok);
    const std::optional<EntityState> state = engine.state("ego");
    ASSERT_TRUE(state.has_value());
    EXPECT_DOUBLE_EQ(state->x, 25.0);
    EXPECT_DOUBLE_EQ(state->y, 3.0);
    EXPECT_DOUBLE_EQ(state->z, 0.0);
    EXPECT_DOUBLE_EQ(state->heading, 0.0); // orientation untouched
}

TEST(TeleportActionEngineTest, TimedTeleportJumpsInOneStepAndCompletes) {
    Engine engine;
    ASSERT_EQ(engine.init(make_teleport_scenario(std::nullopt, WorldPosition{100.0, -50.0, 2.0})),
              Status::Ok);
    EXPECT_DOUBLE_EQ(engine.state("ego")->x, 0.0); // not yet
    ASSERT_EQ(engine.step(1.0), Status::Ok);       // t=1: teleport fires
    const std::optional<EntityState> state = engine.state("ego");
    ASSERT_TRUE(state.has_value());
    EXPECT_DOUBLE_EQ(state->x, 100.0);
    EXPECT_DOUBLE_EQ(state->y, -50.0);
    EXPECT_DOUBLE_EQ(state->z, 2.0);
    EXPECT_EQ(*engine.storyboard_element_state("story/act/group/maneuver/jump"),
              scena::runtime::ElementState::Complete);
}

TEST(TeleportActionEngineTest, NonFinitePositionIsRejectedAtInit) {
    Engine engine;
    const Status status =
        engine.init(make_teleport_scenario(WorldPosition{std::nan(""), 0.0, 0.0}, std::nullopt));
    EXPECT_EQ(status, Status::ValidationError);
}

TEST(TeleportActionEngineTest, WorldPositionTeleportWritesOrientation) {
    // The generalized teleport now writes heading/pitch/roll from the resolved
    // pose, not just x/y/z (ADR-0017).
    Engine engine;
    ASSERT_EQ(engine.init(make_teleport_scenario(WorldPosition{5.0, 6.0, 0.0, 0.75, 0.1, -0.2},
                                                 std::nullopt)),
              Status::Ok);
    const std::optional<EntityState> state = engine.state("ego");
    ASSERT_TRUE(state.has_value());
    EXPECT_DOUBLE_EQ(state->heading, 0.75);
    EXPECT_DOUBLE_EQ(state->pitch, 0.1);
    EXPECT_DOUBLE_EQ(state->roll, -0.2);
}

TEST(TeleportActionEngineTest, RelativeWorldPositionResolvesAgainstReference) {
    // Two init teleports in order: place "lead" in the world, then teleport
    // "ego" to a world-axis delta from it. The resolver reads lead's just-set
    // state (§RelativeWorldPosition).
    Scenario scenario;
    scenario.name = "relative-teleport";
    for (const char* id : {"lead", "ego"}) {
        Entity entity;
        entity.id = id;
        entity.name = id;
        entity.control_mode = ControlMode::EngineControlled;
        scenario.entities.push_back(std::move(entity));
    }
    scenario.init_actions.push_back(
        std::make_shared<TeleportAction>("lead", WorldPosition{10.0, 20.0, 0.0}));
    scena::ir::RelativeWorldPosition relative;
    relative.entity_ref = "lead";
    relative.dx = 5.0;
    relative.dy = -3.0;
    scenario.init_actions.push_back(std::make_shared<TeleportAction>("ego", relative));

    Engine engine;
    ASSERT_EQ(engine.init(scenario), Status::Ok);
    const std::optional<EntityState> ego = engine.state("ego");
    ASSERT_TRUE(ego.has_value());
    EXPECT_DOUBLE_EQ(ego->x, 15.0); // 10 + 5, world axis (not rotated)
    EXPECT_DOUBLE_EQ(ego->y, 17.0); // 20 - 3
}

TEST(TeleportActionEngineTest, UnsupportedVariantIsReportedAndLeavesStateUntouched) {
    // A road-relative target has no backend yet: the teleport is a no-op and a
    // diagnostic is reported — never silently wrong (ADR-0017).
    scena::ir::RoadPosition road;
    road.road_id = "1";
    road.s = 5.0;
    Engine engine;
    ASSERT_EQ(engine.init(make_teleport_scenario(Position{road}, std::nullopt)), Status::Ok);
    const std::optional<EntityState> state = engine.state("ego");
    ASSERT_TRUE(state.has_value());
    EXPECT_DOUBLE_EQ(state->x, 0.0); // untouched
    EXPECT_DOUBLE_EQ(state->y, 0.0);
    const auto& diagnostics = engine.diagnostics();
    EXPECT_TRUE(std::any_of(diagnostics.begin(), diagnostics.end(), [](const scena::Diagnostic& d) {
        return d.code == Status::UnsupportedFeature;
    }));
}

} // namespace
