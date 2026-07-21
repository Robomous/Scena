// SPDX-License-Identifier: MIT
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "kinema/engine.h"
#include "kinema/ir/action.h"
#include "kinema/ir/condition.h"
#include "kinema/ir/scenario.h"

using kinema::Engine;
using kinema::Status;
using kinema::ir::ControlMode;
using kinema::ir::Scenario;
using kinema::ir::SimulationTimeCondition;
using kinema::ir::SpeedAction;

namespace {

Scenario make_scenario() {
    Scenario scenario;
    scenario.name = "determinism-test";
    scenario.entities.push_back({"ego", "ego vehicle", ControlMode::EngineControlled});
    scenario.entities.push_back({"lead", "lead vehicle", ControlMode::EngineControlled});
    scenario.storyboard.entries.push_back({std::make_shared<SimulationTimeCondition>(0.5),
                                           std::make_shared<SpeedAction>("ego", 13.89)});
    scenario.storyboard.entries.push_back({std::make_shared<SimulationTimeCondition>(1.25),
                                           std::make_shared<SpeedAction>("lead", 8.33)});
    scenario.storyboard.entries.push_back({std::make_shared<SimulationTimeCondition>(3.0),
                                           std::make_shared<SpeedAction>("ego", 5.0)});
    return scenario;
}

void expect_bit_identical(const Engine& a, const Engine& b, const std::string& entity_id) {
    const auto state_a = a.state(entity_id);
    const auto state_b = b.state(entity_id);
    ASSERT_TRUE(state_a.has_value());
    ASSERT_TRUE(state_b.has_value());
    // Exact comparison on purpose: determinism means bit-identical doubles,
    // not approximately equal ones.
    ASSERT_EQ(state_a->x, state_b->x);
    ASSERT_EQ(state_a->y, state_b->y);
    ASSERT_EQ(state_a->z, state_b->z);
    ASSERT_EQ(state_a->heading, state_b->heading);
    ASSERT_EQ(state_a->speed, state_b->speed);
}

} // namespace

TEST(DeterminismTest, IdenticalRunsProduceBitIdenticalStates) {
    Engine engine_a;
    Engine engine_b;
    ASSERT_EQ(engine_a.init(make_scenario()), Status::Ok);
    ASSERT_EQ(engine_b.init(make_scenario()), Status::Ok);

    // A deterministic but non-uniform step sequence.
    for (int i = 0; i < 1000; ++i) {
        const double dt = (i % 2 == 0) ? 0.01 : 0.02;
        ASSERT_EQ(engine_a.step(dt), Status::Ok);
        ASSERT_EQ(engine_b.step(dt), Status::Ok);
        expect_bit_identical(engine_a, engine_b, "ego");
        expect_bit_identical(engine_a, engine_b, "lead");
    }

    ASSERT_EQ(engine_a.time(), engine_b.time());
}
