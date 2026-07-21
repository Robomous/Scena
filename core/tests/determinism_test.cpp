// SPDX-License-Identifier: MIT
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "scena/engine.h"
#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"

using scena::Engine;
using scena::Status;
using scena::ir::Act;
using scena::ir::ControlMode;
using scena::ir::Event;
using scena::ir::Maneuver;
using scena::ir::ManeuverGroup;
using scena::ir::Scenario;
using scena::ir::SimulationTimeCondition;
using scena::ir::SpeedAction;
using scena::ir::Story;

namespace {

Event make_speed_event(std::string name, double at_time, std::string entity_id,
                       double target_speed) {
    Event event;
    event.name = std::move(name);
    event.start_trigger = std::make_shared<SimulationTimeCondition>(at_time);
    event.actions.push_back(std::make_shared<SpeedAction>(std::move(entity_id), target_speed));
    return event;
}

/// Hierarchical fixture: two parallel stories, one act behind a start
/// trigger, several timed events — enough structure to exercise the
/// storyboard walk order, plus an init action.
Scenario make_scenario() {
    Scenario scenario;
    scenario.name = "determinism-test";
    scenario.entities.push_back({"ego", "ego vehicle", ControlMode::EngineControlled});
    scenario.entities.push_back({"lead", "lead vehicle", ControlMode::EngineControlled});
    scenario.init_actions.push_back(std::make_shared<SpeedAction>("lead", 6.0));

    {
        Maneuver maneuver;
        maneuver.name = "maneuver";
        maneuver.events.push_back(make_speed_event("cruise", 0.5, "ego", 13.89));
        maneuver.events.push_back(make_speed_event("settle", 3.0, "ego", 5.0));
        ManeuverGroup group;
        group.name = "group";
        group.actors.push_back("ego");
        group.maneuvers.push_back(std::move(maneuver));
        Act act;
        act.name = "act";
        act.groups.push_back(std::move(group));
        Story story;
        story.name = "ego-story";
        story.acts.push_back(std::move(act));
        scenario.storyboard.stories.push_back(std::move(story));
    }
    {
        Maneuver maneuver;
        maneuver.name = "maneuver";
        maneuver.events.push_back(make_speed_event("brake", 1.25, "lead", 8.33));
        ManeuverGroup group;
        group.name = "group";
        group.actors.push_back("lead");
        group.maneuvers.push_back(std::move(maneuver));
        Act act;
        act.name = "act";
        act.start_trigger = std::make_shared<SimulationTimeCondition>(1.0);
        act.groups.push_back(std::move(group));
        Story story;
        story.name = "lead-story";
        story.acts.push_back(std::move(act));
        scenario.storyboard.stories.push_back(std::move(story));
    }
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

TEST(DeterminismTest, StoryboardStatesEvolveIdentically) {
    // The storyboard walk itself must be reproducible: element states agree
    // at every step of two identical runs.
    Engine engine_a;
    Engine engine_b;
    ASSERT_EQ(engine_a.init(make_scenario()), Status::Ok);
    ASSERT_EQ(engine_b.init(make_scenario()), Status::Ok);

    const std::vector<std::string> paths = {
        "",
        "ego-story",
        "ego-story/act/group/maneuver/cruise",
        "ego-story/act/group/maneuver/settle",
        "lead-story/act",
    };
    for (int i = 0; i < 500; ++i) {
        ASSERT_EQ(engine_a.step(0.01), Status::Ok);
        ASSERT_EQ(engine_b.step(0.01), Status::Ok);
        for (const std::string& path : paths) {
            ASSERT_EQ(engine_a.storyboard_element_state(path),
                      engine_b.storyboard_element_state(path))
                << path << " at step " << i;
        }
    }
}
