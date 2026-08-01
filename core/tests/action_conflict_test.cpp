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
// §7.5 actions at runtime (#51, ADR-0025): the control-domain classification
// that decides what conflicts with what (§7.4.1.1, §7.4.1.2, Annex A Table 10),
// "override by Event" (§7.5.2.1), and bulk actions over a multi-actor
// ManeuverGroup (§7.5.4, §8.3.3.3).

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "scena/engine.h"
#include "scena/ir/action.h"
#include "scena/ir/action_domain.h"
#include "scena/ir/condition.h"
#include "scena/ir/dynamics.h"
#include "scena/ir/entity.h"
#include "scena/ir/position.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/trajectory.h"
#include "scena/ir/trigger.h"

namespace {

using scena::Engine;
using scena::EntityState;
using scena::Status;
using scena::ir::AcquirePositionAction;
using scena::ir::ActionDomain;
using scena::ir::control_domains;
using scena::ir::ControlMode;
using scena::ir::DynamicsDimension;
using scena::ir::DynamicsShape;
using scena::ir::Entity;
using scena::ir::FollowTrajectoryAction;
using scena::ir::LaneChangeAction;
using scena::ir::LaneOffsetAction;
using scena::ir::LateralDistanceAction;
using scena::ir::LongitudinalDistanceAction;
using scena::ir::Polyline;
using scena::ir::RelativeTargetLane;
using scena::ir::Scenario;
using scena::ir::SimulationTimeCondition;
using scena::ir::SpeedAction;
using scena::ir::SpeedProfileAction;
using scena::ir::TeleportAction;
using scena::ir::Timing;
using scena::ir::Trajectory;
using scena::ir::TransitionDynamics;
using scena::ir::VisibilityAction;
using scena::ir::WorldPosition;

constexpr double kTol = 1e-9;

TransitionDynamics shaped(double seconds) {
    return TransitionDynamics{DynamicsShape::Linear, DynamicsDimension::Time, seconds};
}

TransitionDynamics stepped() {
    return TransitionDynamics{DynamicsShape::Step, DynamicsDimension::Time, 0.0};
}

double speed_of(const Engine& engine, const std::string& id) {
    const std::optional<EntityState> state = engine.state(id);
    return state.has_value() ? state->speed : -1.0;
}

Entity plain(std::string id) {
    Entity entity;
    entity.id = id;
    entity.name = std::move(id);
    entity.control_mode = ControlMode::EngineControlled;
    return entity;
}

scena::ir::Event timed_event(std::string name, double at,
                             std::vector<std::shared_ptr<scena::ir::Action>> actions) {
    scena::ir::Event event;
    event.name = std::move(name);
    event.start_trigger = scena::ir::make_trigger(std::make_shared<SimulationTimeCondition>(at));
    event.actions = std::move(actions);
    return event;
}

/// A storyboard with one act whose stop trigger fires at `stop_at` seconds
/// (negative ⇒ no stop trigger), one maneuver group and the given events.
Scenario wrap(std::vector<Entity> entities, std::vector<scena::ir::Event> events,
              double stop_at = -1.0,
              std::vector<std::shared_ptr<scena::ir::Action>> init_actions = {}) {
    Scenario scenario;
    scenario.name = "conflict";
    scenario.entities = std::move(entities);
    scenario.init_actions = std::move(init_actions);

    scena::ir::Maneuver maneuver;
    maneuver.name = "maneuver";
    maneuver.events = std::move(events);
    scena::ir::ManeuverGroup group;
    group.name = "group";
    group.maneuvers.push_back(std::move(maneuver));
    scena::ir::Act act;
    act.name = "act";
    if (stop_at >= 0.0) {
        act.stop_trigger =
            scena::ir::make_trigger(std::make_shared<SimulationTimeCondition>(stop_at));
    }
    act.groups.push_back(std::move(group));
    scena::ir::Story story;
    story.name = "story";
    story.acts.push_back(std::move(act));
    scenario.storyboard.stories.push_back(std::move(story));
    return scenario;
}

// --- §7.4.1.2 / Table 10: which actions assign a control strategy ----------

TEST(ActionDomainTest, MotionControlActionsClaimTheirTableDomains) {
    EXPECT_EQ(control_domains(SpeedAction("ego", 10.0, shaped(2.0))), ActionDomain::Longitudinal);
    EXPECT_EQ(control_domains(SpeedProfileAction("ego", {{10.0, 1.0}})),
              ActionDomain::Longitudinal);
    EXPECT_EQ(
        control_domains(LongitudinalDistanceAction("ego", "lead", 10.0, std::nullopt, false, true)),
        ActionDomain::Longitudinal);
    EXPECT_EQ(control_domains(LaneChangeAction("ego", RelativeTargetLane{"ego", -1}, shaped(2.0))),
              ActionDomain::Lateral);
    EXPECT_EQ(control_domains(LateralDistanceAction("ego", "lead", 1.0, false, true)),
              ActionDomain::Lateral);
}

TEST(ActionDomainTest, StepShapedActionsSetAStateAndClaimNothing) {
    // §7.4.1.2: "in this particular use case, these actions do not assign a
    // control strategy as the changes are enacted instantaneously".
    EXPECT_EQ(control_domains(SpeedAction("ego", 10.0)), ActionDomain::None);
    EXPECT_EQ(control_domains(SpeedAction("ego", 10.0, stepped())), ActionDomain::None);
    EXPECT_EQ(control_domains(LaneChangeAction("ego", RelativeTargetLane{"ego", -1}, stepped())),
              ActionDomain::None);
}

TEST(ActionDomainTest, ATimedTrajectoryOwnsBothMotionDomains) {
    Polyline line;
    line.vertices.push_back({WorldPosition{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.0});
    line.vertices.push_back({WorldPosition{50.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 5.0});
    Trajectory trajectory{"path", false, line};

    const FollowTrajectoryAction untimed("ego", trajectory);
    EXPECT_EQ(control_domains(untimed), ActionDomain::Lateral);

    const FollowTrajectoryAction timed("ego", trajectory, scena::ir::FollowingMode::Position,
                                       Timing{});
    EXPECT_TRUE(holds(control_domains(timed), ActionDomain::Lateral));
    EXPECT_TRUE(holds(control_domains(timed), ActionDomain::Longitudinal));
}

TEST(ActionDomainTest, NonMotionActionsHoldNoDomainAndNeverConflict) {
    EXPECT_EQ(control_domains(TeleportAction("ego", WorldPosition{})), ActionDomain::None);
    EXPECT_EQ(control_domains(AcquirePositionAction("ego", WorldPosition{})), ActionDomain::None);
    EXPECT_EQ(control_domains(VisibilityAction("ego", true, true, true)), ActionDomain::None);
    EXPECT_FALSE(conflicts(ActionDomain::None, ActionDomain::None));
    EXPECT_FALSE(conflicts(ActionDomain::Longitudinal, ActionDomain::Lateral));
    EXPECT_TRUE(
        conflicts(ActionDomain::Longitudinal | ActionDomain::Lateral, ActionDomain::Lateral));
}

// --- §7.5.2.1: override by Event ------------------------------------------

TEST(OverrideByEventTest, AStoppedActReleasesTheDomainsItsActionsHeld) {
    // A 10 s ramp is stopped at t = 3 by the act's stop trigger. §7.5.2.1: the
    // action moves to completeState with a stopTransition, which means the
    // control strategy it assigned is unassigned — the entity holds the speed
    // it had reached instead of continuing up the ramp.
    Engine engine;
    std::vector<scena::ir::Event> events;
    events.push_back(
        timed_event("ramp", 0.0, {std::make_shared<SpeedAction>("ego", 20.0, shaped(10.0))}));
    ASSERT_EQ(engine.init(wrap({plain("ego")}, std::move(events), /*stop_at=*/3.0)), Status::Ok);

    for (int i = 0; i < 2; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    EXPECT_NEAR(speed_of(engine, "ego"), 4.0, kTol); // 2 s into 0 -> 20 over 10 s

    // A stop trigger is checked before the elements it covers are advanced
    // (§7.6.1.2), so the evaluation at t = 3 stops the ramp instead of stepping
    // it: the speed the entity keeps is the one it had at t = 2.
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(*engine.storyboard_element_state("story/act/group/maneuver/ramp"),
              scena::runtime::ElementState::Complete);
    EXPECT_NEAR(speed_of(engine, "ego"), 4.0, kTol); // frozen where the stop found it
    for (int i = 0; i < 5; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    EXPECT_NEAR(speed_of(engine, "ego"), 4.0, kTol); // and it stays there
}

TEST(OverrideByEventTest, AStoppedNeverEndingActionStopsToo) {
    // §7.5.3: a never-ending action has no regular ending, but "all action ends
    // described in §7.5.2.1 are applicable" — including the stopTransition its
    // event takes. A continuous LaneOffsetAction holds the lateral domain
    // forever; the act's stop trigger releases it, and the entity stops being
    // steered toward the offset.
    Engine engine;
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event(
        "hold", 0.0,
        {std::make_shared<SpeedAction>("ego", 10.0, shaped(1.0)),
         std::make_shared<LaneOffsetAction>("ego", scena::ir::AbsoluteTargetLaneOffset{6.0},
                                            /*continuous=*/true, DynamicsShape::Linear, 0.5)}));
    ASSERT_EQ(engine.init(wrap({plain("ego")}, std::move(events), /*stop_at=*/4.0)), Status::Ok);

    // While the offset ramp runs, the sideways motion is not uniform.
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    const double y1 = engine.state("ego")->y;
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    const double y2 = engine.state("ego")->y;
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    const double y3 = engine.state("ego")->y;
    EXPECT_NE(y2 - y1, y3 - y2);

    // The stop trigger fires at t = 4 and the action never ends any other way.
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_EQ(*engine.storyboard_element_state("story/act/group/maneuver/hold"),
              scena::runtime::ElementState::Complete);

    // With the lateral strategy unassigned the entity simply travels on its
    // last heading: successive displacements are identical.
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    const double y4 = engine.state("ego")->y;
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    const double y5 = engine.state("ego")->y;
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    const double y6 = engine.state("ego")->y;
    EXPECT_DOUBLE_EQ(y5 - y4, y6 - y5);
}

// --- §7.5.4 / §8.3.3.3: bulk actions --------------------------------------

TEST(BulkActionTest, AConflictOnOneActorOverridesEveryInstance) {
    // One authored ramp applied to three actors (one ir::Action each, sharing a
    // bulk group). A second ramp then takes "b" — §7.5.4: "all its instances of
    // Entity are supposed to fall back to default behavior simultaneously", so
    // "a" and "c" stop ramping too and hold the speed they had reached.
    const std::size_t group = 7;
    std::vector<std::shared_ptr<scena::ir::Action>> bulk;
    for (const char* actor : {"a", "b", "c"}) {
        auto action = std::make_shared<SpeedAction>(actor, 20.0, shaped(10.0));
        action->set_bulk_group(group);
        bulk.push_back(std::move(action));
    }
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event("bulk", 0.0, std::move(bulk)));
    events.push_back(
        timed_event("single", 2.0, {std::make_shared<SpeedAction>("b", 0.0, shaped(1.0))}));

    Engine engine;
    ASSERT_EQ(engine.init(wrap({plain("a"), plain("b"), plain("c")}, std::move(events))),
              Status::Ok);
    for (int i = 0; i < 2; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    for (const char* actor : {"a", "b", "c"}) {
        EXPECT_NEAR(speed_of(engine, actor), 4.0, kTol) << actor; // 2 s of 0 -> 20 / 10 s
    }

    ASSERT_EQ(engine.step(1.0), Status::Ok); // t = 3: "single" has taken b
    // a and c are frozen at the speed the override found them at, not ramping.
    EXPECT_NEAR(speed_of(engine, "a"), 4.0, kTol);
    EXPECT_NEAR(speed_of(engine, "c"), 4.0, kTol);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_NEAR(speed_of(engine, "a"), 4.0, kTol);
    EXPECT_NEAR(speed_of(engine, "c"), 4.0, kTol);
    EXPECT_NEAR(speed_of(engine, "b"), 0.0, kTol); // the new owner reached its target
    // The bulk event ends once every instance has (§8.3.3.3's join).
    EXPECT_EQ(*engine.storyboard_element_state("story/act/group/maneuver/bulk"),
              scena::runtime::ElementState::Complete);
}

TEST(BulkActionTest, InstancesWithoutAGroupAreUnaffectedByEachOther) {
    // The same shape without bulk ids: overriding "b" leaves "a" and "c"
    // ramping, which is what makes the group id load-bearing rather than
    // incidental.
    std::vector<std::shared_ptr<scena::ir::Action>> parallel;
    for (const char* actor : {"a", "b", "c"}) {
        parallel.push_back(std::make_shared<SpeedAction>(actor, 20.0, shaped(10.0)));
    }
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event("bulk", 0.0, std::move(parallel)));
    events.push_back(
        timed_event("single", 2.0, {std::make_shared<SpeedAction>("b", 0.0, shaped(1.0))}));

    Engine engine;
    ASSERT_EQ(engine.init(wrap({plain("a"), plain("b"), plain("c")}, std::move(events))),
              Status::Ok);
    for (int i = 0; i < 4; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    EXPECT_NEAR(speed_of(engine, "a"), 8.0, kTol); // still ramping at t = 4
    EXPECT_NEAR(speed_of(engine, "c"), 8.0, kTol);
    EXPECT_NEAR(speed_of(engine, "b"), 0.0, kTol);
}

TEST(BulkActionTest, StoppingOneInstanceStopsTheWholeBulkAction) {
    // §7.5.2.1's "override by Event" reaches the bulk rule: the act stops the
    // event, every instance is released, and every actor holds its speed.
    const std::size_t group = 3;
    std::vector<std::shared_ptr<scena::ir::Action>> bulk;
    for (const char* actor : {"a", "b"}) {
        auto action = std::make_shared<SpeedAction>(actor, 20.0, shaped(10.0));
        action->set_bulk_group(group);
        bulk.push_back(std::move(action));
    }
    std::vector<scena::ir::Event> events;
    events.push_back(timed_event("bulk", 0.0, std::move(bulk)));

    Engine engine;
    ASSERT_EQ(engine.init(wrap({plain("a"), plain("b")}, std::move(events), /*stop_at=*/2.0)),
              Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    EXPECT_NEAR(speed_of(engine, "a"), 2.0, kTol);
    ASSERT_EQ(engine.step(1.0), Status::Ok); // t = 2: the stop trigger fires first
    for (int i = 0; i < 3; ++i) {
        ASSERT_EQ(engine.step(1.0), Status::Ok);
    }
    EXPECT_NEAR(speed_of(engine, "a"), 2.0, kTol);
    EXPECT_NEAR(speed_of(engine, "b"), 2.0, kTol);
}

TEST(BulkActionTest, ABulkRunIsBitIdenticalAcrossRepeats) {
    const auto run = []() {
        const std::size_t group = 11;
        std::vector<std::shared_ptr<scena::ir::Action>> bulk;
        for (const char* actor : {"a", "b", "c"}) {
            auto action = std::make_shared<SpeedAction>(actor, 17.5, shaped(6.0));
            action->set_bulk_group(group);
            bulk.push_back(std::move(action));
        }
        std::vector<scena::ir::Event> events;
        events.push_back(timed_event("bulk", 0.0, std::move(bulk)));
        events.push_back(
            timed_event("single", 1.5, {std::make_shared<SpeedAction>("c", 3.0, shaped(2.0))}));
        Engine engine;
        EXPECT_EQ(engine.init(wrap({plain("a"), plain("b"), plain("c")}, std::move(events))),
                  Status::Ok);
        std::vector<double> trace;
        for (int i = 0; i < 100; ++i) {
            EXPECT_EQ(engine.step(0.1), Status::Ok);
            for (const char* actor : {"a", "b", "c"}) {
                trace.push_back(speed_of(engine, actor));
            }
        }
        return trace;
    };
    const std::vector<double> first = run();
    const std::vector<double> second = run();
    ASSERT_EQ(first.size(), second.size());
    for (std::size_t i = 0; i < first.size(); ++i) {
        EXPECT_DOUBLE_EQ(first[i], second[i]);
    }
}

} // namespace
