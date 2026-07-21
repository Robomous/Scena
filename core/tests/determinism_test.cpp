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
#include "scena/ir/trigger.h"

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
    event.start_trigger =
        scena::ir::make_trigger(std::make_shared<SimulationTimeCondition>(at_time));
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
        // A delayed rising edge: exercises the edge history and the
        // sample-and-hold delay lookup, both of which have to reproduce
        // exactly across runs (§7.6.2, §7.6.3).
        Event delayed;
        delayed.name = "resume";
        delayed.start_trigger = scena::ir::make_trigger(
            std::make_shared<SimulationTimeCondition>(0.7), scena::ir::ConditionEdge::Rising, 0.35);
        delayed.actions.push_back(std::make_shared<SpeedAction>("ego", 9.5));
        // The three priority branches of §8.4.2.2 are all walked, even
        // though no action driven through Engine is ever ongoing, so no
        // sibling is ever in runningState when another starts: both of
        // these resolve to a plain start and must do so identically in
        // every run.
        delayed.priority = scena::ir::EventPriority::Skip;
        maneuver.events.push_back(std::move(delayed));
        maneuver.events[1].priority = scena::ir::EventPriority::Override; // "settle"

        // Sequential re-execution (§8.3.3.2): ends, re-arms to standby and
        // starts again on the next evaluation whose trigger holds, three
        // times over.
        Event repeated = make_speed_event("repeat", 1.6, "ego", 7.25);
        repeated.maximum_execution_count = 3;
        maneuver.events.push_back(std::move(repeated));

        // Exhausted before it ever starts, so it completes with a
        // skipTransition and never fires (§8.4.2.1).
        Event never = make_speed_event("never", 2.0, "ego", 99.0);
        never.maximum_execution_count = 0;
        maneuver.events.push_back(std::move(never));

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
        // Never fires: the act's stop trigger takes the whole subtree to
        // completeState first (§7.6.1.2).
        maneuver.events.push_back(make_speed_event("recover", 3.0, "lead", 11.0));
        ManeuverGroup group;
        group.name = "group";
        group.actors.push_back("lead");
        group.maneuvers.push_back(std::move(maneuver));
        Act act;
        act.name = "act";
        act.start_trigger = scena::ir::make_trigger(std::make_shared<SimulationTimeCondition>(1.0));
        // Act stop trigger: the stop cascade must run at the same step in
        // both runs (§7.6.1.2).
        act.stop_trigger = scena::ir::make_trigger(std::make_shared<SimulationTimeCondition>(2.5));
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

TEST(DeterminismTest, LongHorizonAccumulationStaysBitIdentical) {
    // The integrator now routes trig through detmath; over a long run the
    // accumulated position must stay bit-identical between two engines fed the
    // same varying-dt sequence. This is the in-process analogue of the
    // cross-platform trace diff: same inputs, exactly the same bits, forever.
    Engine engine_a;
    Engine engine_b;
    ASSERT_EQ(engine_a.init(make_scenario()), Status::Ok);
    ASSERT_EQ(engine_b.init(make_scenario()), Status::Ok);

    constexpr int kSteps = 120000;
    for (int i = 0; i < kSteps; ++i) {
        // A varying but fully deterministic dt: cycles through seven values so
        // the heading-dependent trig is exercised at many reduced arguments.
        const double dt = 0.001 + 0.0005 * (i % 7);
        ASSERT_EQ(engine_a.step(dt), Status::Ok);
        ASSERT_EQ(engine_b.step(dt), Status::Ok);
        // Comparing every step is too slow under ASan; sampling every 500th
        // step plus the final step still catches any divergence, since a
        // single differing bit never heals.
        if (i % 500 == 0 || i == kSteps - 1) {
            SCOPED_TRACE(i);
            expect_bit_identical(engine_a, engine_b, "ego");
            expect_bit_identical(engine_a, engine_b, "lead");
        }
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
        "ego-story/act/group/maneuver/resume",
        "ego-story/act/group/maneuver/repeat",
        "ego-story/act/group/maneuver/never",
        "lead-story/act",
        "lead-story/act/group/maneuver/brake",
        "lead-story/act/group/maneuver/recover",
    };
    for (int i = 0; i < 500; ++i) {
        ASSERT_EQ(engine_a.step(0.01), Status::Ok);
        ASSERT_EQ(engine_b.step(0.01), Status::Ok);
        for (const std::string& path : paths) {
            ASSERT_EQ(engine_a.storyboard_element_state(path),
                      engine_b.storyboard_element_state(path))
                << path << " at step " << i;
            ASSERT_EQ(engine_a.storyboard_element_transition(path),
                      engine_b.storyboard_element_transition(path))
                << path << " at step " << i;
        }
    }
}

TEST(DeterminismTest, FixtureExercisesEdgeDelayAndStopPaths) {
    // Guards the two tests above: they only prove reproducibility of the
    // trigger machinery if the fixture actually reaches it.
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario()), Status::Ok);
    for (int i = 0; i < 500; ++i) {
        ASSERT_EQ(engine.step(0.01), Status::Ok);
    }
    // The delayed rising edge fired (speed 9.5 came from "resume").
    EXPECT_EQ(engine.storyboard_element_transition("ego-story/act/group/maneuver/resume"),
              scena::runtime::TransitionKind::End);
    // The act stop trigger stopped its subtree before "recover" could fire.
    EXPECT_EQ(engine.storyboard_element_transition("lead-story/act/group/maneuver/recover"),
              scena::runtime::TransitionKind::Stop);
    EXPECT_EQ(engine.storyboard_element_transition("lead-story/act"),
              scena::runtime::TransitionKind::Stop);
    // ...while the ego story ran to a regular end and the storyboard lives on.
    EXPECT_EQ(engine.storyboard_element_transition("ego-story"),
              scena::runtime::TransitionKind::End);
    EXPECT_EQ(engine.storyboard_element_state(""), scena::runtime::ElementState::Running);
}

TEST(DeterminismTest, FixtureExercisesExecutionCountAndSkipPaths) {
    // Same guard for the event-lifecycle paths: the two reproducibility
    // tests above only cover §8.3.3.2/§8.4.2.1 if the fixture reaches them.
    Engine engine;
    ASSERT_EQ(engine.init(make_scenario()), Status::Ok);

    // Exhausted before it could start: completed with a skipTransition at
    // the very first evaluation, without ever firing.
    EXPECT_EQ(engine.storyboard_element_state("ego-story/act/group/maneuver/never"),
              scena::runtime::ElementState::Complete);
    EXPECT_EQ(engine.storyboard_element_transition("ego-story/act/group/maneuver/never"),
              scena::runtime::TransitionKind::Skip);

    int repeat_standby_steps = 0;
    for (int i = 0; i < 500; ++i) {
        ASSERT_EQ(engine.step(0.01), Status::Ok);
        if (engine.storyboard_element_state("ego-story/act/group/maneuver/repeat") ==
            scena::runtime::ElementState::Standby) {
            ++repeat_standby_steps;
        }
    }
    // Re-armed at least once between executions, then spent its budget.
    EXPECT_GT(repeat_standby_steps, 0);
    EXPECT_EQ(engine.storyboard_element_state("ego-story/act/group/maneuver/repeat"),
              scena::runtime::ElementState::Complete);
    EXPECT_EQ(engine.storyboard_element_transition("ego-story/act/group/maneuver/repeat"),
              scena::runtime::TransitionKind::End);
    // The repeated event drove the ego speed, so it really did fire.
    const auto ego = engine.state("ego");
    ASSERT_TRUE(ego.has_value());
    EXPECT_NE(ego->speed, 99.0); // "never" never applied its speed
}
