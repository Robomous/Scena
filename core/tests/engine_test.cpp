// SPDX-License-Identifier: MIT
#include "kinema/engine.h"

#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "kinema/ir/action.h"
#include "kinema/ir/condition.h"
#include "kinema/ir/scenario.h"

using kinema::Engine;
using kinema::EntityState;
using kinema::Status;
using kinema::ir::ControlMode;
using kinema::ir::Scenario;
using kinema::ir::SimulationTimeCondition;
using kinema::ir::SpeedAction;

namespace {

Scenario make_scenario() {
    Scenario scenario;
    scenario.name = "engine-test";
    scenario.entities.push_back({"ego", "ego vehicle", ControlMode::EngineControlled});
    scenario.entities.push_back({"npc", "host vehicle", ControlMode::HostControlled});
    scenario.storyboard.entries.push_back({std::make_shared<SimulationTimeCondition>(0.0),
                                           std::make_shared<SpeedAction>("ego", 10.0)});
    return scenario;
}

} // namespace

TEST(EngineTest, LifecycleInitStepClose) {
    Engine engine;
    EXPECT_FALSE(engine.initialized());
    EXPECT_EQ(engine.init(make_scenario()), Status::Ok);
    EXPECT_TRUE(engine.initialized());
    EXPECT_EQ(engine.step(0.01), Status::Ok);
    EXPECT_EQ(engine.close(), Status::Ok);
    EXPECT_FALSE(engine.initialized());
}

TEST(EngineTest, DoubleInitFails) {
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario()), Status::Ok);
    EXPECT_EQ(engine.init(make_scenario()), Status::AlreadyInitialized);
}

TEST(EngineTest, StepBeforeInitFails) {
    Engine engine;
    EXPECT_EQ(engine.step(0.01), Status::NotInitialized);
}

TEST(EngineTest, CloseBeforeInitFails) {
    Engine engine;
    EXPECT_EQ(engine.close(), Status::NotInitialized);
}

TEST(EngineTest, ReinitAfterCloseWorks) {
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario()), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    ASSERT_EQ(engine.close(), Status::Ok);
    ASSERT_EQ(engine.init(make_scenario()), Status::Ok);
    EXPECT_EQ(engine.time(), 0.0);
    const auto state = engine.state("ego");
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->speed, 0.0); // fresh zero-initialized state
}

TEST(EngineTest, NegativeOrNanDtIsRejected) {
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario()), Status::Ok);
    EXPECT_EQ(engine.step(-0.01), Status::InvalidArgument);
    EXPECT_EQ(engine.step(std::nan("")), Status::InvalidArgument);
    EXPECT_EQ(engine.step(0.0), Status::Ok); // zero dt is a valid no-advance step
}

TEST(EngineTest, StateQueries) {
    Engine engine;
    EXPECT_FALSE(engine.state("ego").has_value()); // before init
    ASSERT_EQ(engine.init(make_scenario()), Status::Ok);
    EXPECT_TRUE(engine.state("ego").has_value());
    EXPECT_TRUE(engine.state("npc").has_value());
    EXPECT_FALSE(engine.state("missing").has_value());
}

TEST(EngineTest, InitValidatesScenario) {
    {
        Engine engine;
        Scenario scenario = make_scenario();
        scenario.entities.push_back({"ego", "duplicate id", ControlMode::EngineControlled});
        EXPECT_EQ(engine.init(std::move(scenario)), Status::InvalidArgument);
        EXPECT_FALSE(engine.initialized());
    }
    {
        Engine engine;
        Scenario scenario = make_scenario();
        scenario.entities.push_back({"", "empty id", ControlMode::EngineControlled});
        EXPECT_EQ(engine.init(std::move(scenario)), Status::InvalidArgument);
    }
    {
        Engine engine;
        Scenario scenario = make_scenario();
        scenario.storyboard.entries.push_back({std::make_shared<SimulationTimeCondition>(1.0),
                                               std::make_shared<SpeedAction>("missing", 5.0)});
        EXPECT_EQ(engine.init(std::move(scenario)), Status::UnknownEntity);
    }
    {
        Engine engine;
        Scenario scenario = make_scenario();
        scenario.storyboard.entries.push_back({nullptr, nullptr});
        EXPECT_EQ(engine.init(std::move(scenario)), Status::InvalidArgument);
    }
}

TEST(EngineTest, TimeAccumulates) {
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario()), Status::Ok);
    double expected = 0.0;
    for (int i = 0; i < 100; ++i) {
        ASSERT_EQ(engine.step(0.01), Status::Ok);
        expected += 0.01;
    }
    EXPECT_EQ(engine.time(), expected);
}

TEST(EngineTest, EngineControlledKinematicIntegration) {
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario()), Status::Ok);

    // The speed action triggers at t = 0, i.e. on the first step; the entity
    // then advances along its zero heading (+X) at 10 m/s.
    const double dt = 0.01;
    double expected_x = 0.0;
    for (int i = 0; i < 100; ++i) {
        ASSERT_EQ(engine.step(dt), Status::Ok);
        expected_x += 10.0 * 1.0 * dt; // speed * cos(0) * dt, same op order as the engine
    }

    const auto state = engine.state("ego");
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->speed, 10.0);
    EXPECT_DOUBLE_EQ(state->x, expected_x);
    EXPECT_EQ(state->y, 0.0);
    EXPECT_EQ(state->z, 0.0);
}

TEST(EngineTest, HostControlledStateRoundTrip) {
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario()), Status::Ok);

    const EntityState reported{1.0, 2.0, 3.0, 0.5, 7.0};
    ASSERT_EQ(engine.report_state("npc", reported), Status::Ok);

    // The engine never integrates host-controlled entities: the reported state
    // survives a step bit-for-bit.
    ASSERT_EQ(engine.step(0.01), Status::Ok);
    const auto state = engine.state("npc");
    ASSERT_TRUE(state.has_value());
    EXPECT_EQ(state->x, reported.x);
    EXPECT_EQ(state->y, reported.y);
    EXPECT_EQ(state->z, reported.z);
    EXPECT_EQ(state->heading, reported.heading);
    EXPECT_EQ(state->speed, reported.speed);
}

TEST(EngineTest, ReportStateErrors) {
    Engine engine;
    const EntityState state{};
    EXPECT_EQ(engine.report_state("npc", state), Status::NotInitialized);
    ASSERT_EQ(engine.init(make_scenario()), Status::Ok);
    EXPECT_EQ(engine.report_state("missing", state), Status::UnknownEntity);
    EXPECT_EQ(engine.report_state("ego", state), Status::InvalidControlMode);
}
