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

// Per-entity control ownership and the host round-trip (p2-s4, ADR-0003 /
// ADR-0017). Engine-controlled entities are integrated and published; host-
// controlled entities are never integrated — their authoritative state comes
// from report_state() or the gateway poll, and scenario conditions still
// observe it. The in-step order is fixed: poll host states, evaluate the
// storyboard, integrate engine entities, publish engine states.

#include <algorithm>
#include <cstddef>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "scena/engine.h"
#include "scena/gateway/simulator_gateway.h"
#include "scena/ir/action.h"
#include "scena/ir/entity.h"
#include "scena/ir/entity_condition.h"
#include "scena/ir/position.h"
#include "scena/ir/rule.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/trigger.h"

namespace {

using scena::Engine;
using scena::EntityState;
using scena::Status;
using scena::ir::ControlMode;
using scena::ir::Entity;
using scena::ir::Rule;
using scena::ir::Scenario;
using scena::ir::SpeedCondition;
using scena::ir::TeleportAction;
using scena::ir::TriggeringEntities;
using scena::ir::TriggeringEntitiesRule;
using scena::ir::WorldPosition;

// Records the poll/publish exchange so tests can assert the in-step order, and
// hands back a queued state for each host-controlled entity on poll.
class RecordingGateway final : public scena::gateway::ISimulatorGateway {
public:
    bool poll_state(const std::string& entity_id, EntityState& out) override {
        log.push_back("poll:" + entity_id);
        const auto it = poll_states.find(entity_id);
        if (it == poll_states.end()) {
            return false;
        }
        out = it->second;
        return true;
    }
    void publish_state(const std::string& entity_id, const EntityState& state) override {
        log.push_back("publish:" + entity_id);
        published[entity_id] = state;
    }
    scena::gateway::IRoadQuery* road_query() override { return nullptr; }

    std::map<std::string, EntityState> poll_states;
    std::map<std::string, EntityState> published;
    std::vector<std::string> log;
};

Entity make_entity(const std::string& id, ControlMode mode) {
    Entity entity;
    entity.id = id;
    entity.name = id;
    entity.control_mode = mode;
    return entity;
}

// --- report_state contract -------------------------------------------------

TEST(ControlOwnershipTest, ReportStateBeforeInitIsRejected) {
    Engine engine;
    EXPECT_EQ(engine.report_state("npc", EntityState{}), Status::NotInitialized);
}

TEST(ControlOwnershipTest, ReportStateRejectsEngineControlledAndUnknown) {
    Scenario scenario;
    scenario.name = "ownership";
    scenario.entities.push_back(make_entity("ego", ControlMode::EngineControlled));
    scenario.entities.push_back(make_entity("npc", ControlMode::HostControlled));
    Engine engine;
    ASSERT_EQ(engine.init(scenario), Status::Ok);

    // The engine owns "ego"; the host may not report it.
    EXPECT_EQ(engine.report_state("ego", EntityState{}), Status::InvalidControlMode);
    // No such entity.
    EXPECT_EQ(engine.report_state("ghost", EntityState{}), Status::UnknownEntity);
    // The host owns "npc".
    EXPECT_EQ(engine.report_state("npc", EntityState{1.0, 2.0, 0.0, 0.0, 7.0}), Status::Ok);
    const auto state = engine.state("npc");
    ASSERT_TRUE(state.has_value());
    EXPECT_DOUBLE_EQ(state->x, 1.0);
    EXPECT_DOUBLE_EQ(state->speed, 7.0);
}

// --- host entities are never integrated ------------------------------------

TEST(ControlOwnershipTest, HostControlledEntityIsNeverIntegrated) {
    Scenario scenario;
    scenario.name = "no-integrate";
    scenario.entities.push_back(make_entity("npc", ControlMode::HostControlled));
    RecordingGateway gateway;
    // The host reports a moving entity (speed 10) that stays at x = 5: the
    // engine must not advance it from its speed.
    gateway.poll_states["npc"] = EntityState{5.0, 0.0, 0.0, 0.0, 10.0};

    Engine engine(&gateway);
    ASSERT_EQ(engine.init(scenario), Status::Ok);
    for (int i = 0; i < 3; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    const auto state = engine.state("npc");
    ASSERT_TRUE(state.has_value());
    EXPECT_DOUBLE_EQ(state->x, 5.0); // 5, not 5 + 3 * 10
}

// --- in-step poll/publish order (ADR-0003) ---------------------------------

TEST(ControlOwnershipTest, PollPrecedesPublishWithinAStep) {
    Scenario scenario;
    scenario.name = "order";
    scenario.entities.push_back(make_entity("ego", ControlMode::EngineControlled));
    scenario.entities.push_back(make_entity("npc", ControlMode::HostControlled));
    RecordingGateway gateway;
    gateway.poll_states["npc"] = EntityState{0.0, 0.0, 0.0, 0.0, 0.0};

    Engine engine(&gateway);
    ASSERT_EQ(engine.init(scenario), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);

    // Every host poll precedes every engine publish: poll brackets the front of
    // the step, publish the back.
    std::size_t last_poll = 0;
    std::size_t first_publish = gateway.log.size();
    for (std::size_t i = 0; i < gateway.log.size(); ++i) {
        if (gateway.log[i].rfind("poll:", 0) == 0) {
            last_poll = i;
        }
        if (gateway.log[i].rfind("publish:", 0) == 0 && first_publish == gateway.log.size()) {
            first_publish = i;
        }
    }
    EXPECT_LT(last_poll, first_publish);
    // The published engine state is the one the host can read back.
    ASSERT_TRUE(gateway.published.count("ego") == 1);
    EXPECT_DOUBLE_EQ(gateway.published["ego"].x, engine.state("ego")->x);
    // The engine never publishes the host-controlled entity.
    EXPECT_EQ(gateway.published.count("npc"), 0u);
}

// --- conditions observe reported host state --------------------------------

TEST(ControlOwnershipTest, ConditionObservesReportedHostState) {
    // A SpeedCondition on the host-controlled "npc" gates an action on the
    // engine-controlled "ego": when the host reports npc fast enough, the
    // condition sees it and ego is teleported. This is the round-trip.
    Scenario scenario;
    scenario.name = "observe";
    scenario.entities.push_back(make_entity("ego", ControlMode::EngineControlled));
    scenario.entities.push_back(make_entity("npc", ControlMode::HostControlled));

    scena::ir::Event event;
    event.name = "npc-fast";
    event.start_trigger = scena::ir::make_trigger(
        std::make_shared<SpeedCondition>(TriggeringEntities{TriggeringEntitiesRule::Any, {"npc"}},
                                         5.0, Rule::GreaterOrEqual),
        scena::ir::ConditionEdge::None, 0.0);
    event.actions.push_back(std::make_shared<TeleportAction>("ego", WorldPosition{99.0, 0.0, 0.0}));
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

    Engine engine;
    ASSERT_EQ(engine.init(scenario), Status::Ok);

    // npc reports a slow speed: the condition is false, ego stays put.
    ASSERT_EQ(engine.report_state("npc", EntityState{0.0, 0.0, 0.0, 0.0, 1.0}), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_DOUBLE_EQ(engine.state("ego")->x, 0.0);

    // npc reports fast: the condition observes it and ego teleports.
    ASSERT_EQ(engine.report_state("npc", EntityState{0.0, 0.0, 0.0, 0.0, 10.0}), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_DOUBLE_EQ(engine.state("ego")->x, 99.0);
}

// --- mode violation: teleporting a host-controlled entity ------------------

TEST(ControlOwnershipTest, TeleportOnHostControlledEntityIsAModeViolation) {
    Scenario scenario;
    scenario.name = "violation";
    scenario.entities.push_back(make_entity("npc", ControlMode::HostControlled));
    // An init teleport aimed at the host-controlled entity.
    scenario.init_actions.push_back(
        std::make_shared<TeleportAction>("npc", WorldPosition{42.0, 0.0, 0.0}));

    Engine engine;
    ASSERT_EQ(engine.init(scenario), Status::Ok);
    // The engine does not drive the host's entity: state is untouched.
    const auto state = engine.state("npc");
    ASSERT_TRUE(state.has_value());
    EXPECT_DOUBLE_EQ(state->x, 0.0);
    // The violation is reported, not silent.
    const auto& diagnostics = engine.diagnostics();
    EXPECT_TRUE(std::any_of(diagnostics.begin(), diagnostics.end(), [](const scena::Diagnostic& d) {
        return d.code == Status::InvalidControlMode;
    }));
}

} // namespace
