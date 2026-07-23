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
// TeleportAction (p5-s4): world-frame teleport. A one-shot action that writes an
// entity's world position and completes in the evaluation it fires. Only the
// §WorldPosition target is modeled (the PositionResolver and the other §6.3.8
// variants arrive with p2-s4/p3-s4).

#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>

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
using scena::ir::Scenario;
using scena::ir::SimulationTimeCondition;
using scena::ir::TeleportAction;
using scena::ir::WorldPosition;

// --- IR surface -----------------------------------------------------------

TEST(TeleportActionIrTest, CarriesEntityAndWorldPosition) {
    TeleportAction action("ego", WorldPosition{10.0, -4.0, 1.5});
    EXPECT_EQ(action.entity_id(), "ego");
    EXPECT_EQ(action.kind(), "TeleportAction");
    EXPECT_DOUBLE_EQ(action.position().x, 10.0);
    EXPECT_DOUBLE_EQ(action.position().y, -4.0);
    EXPECT_DOUBLE_EQ(action.position().z, 1.5);
}

// --- Engine integration ---------------------------------------------------

// Scenario with one engine-controlled "ego"; `init_teleport` (when set) runs as
// an init action, and `timed_teleport` (when set) fires at t=1 via a
// SimulationTimeCondition.
Scenario make_teleport_scenario(std::optional<WorldPosition> init_teleport,
                                std::optional<WorldPosition> timed_teleport) {
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

} // namespace
