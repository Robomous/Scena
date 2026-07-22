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

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "scena/engine.h"
#include "scena/entity_state.h"
#include "scena/ir/action.h"
#include "scena/ir/bounding_box.h"
#include "scena/ir/condition.h"
#include "scena/ir/date_time.h"
#include "scena/ir/entity_condition.h"
#include "scena/ir/interaction_condition.h"
#include "scena/ir/position.h"
#include "scena/ir/rule.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/traffic_signal.h"
#include "scena/ir/trajectory.h"
#include "scena/ir/trigger.h"
#include "scena/runtime/detmath.h"
#include "scena/runtime/obb2.h"
#include "support/fixtures.h"
#include "support/trace_recorder.h"

using scena::Engine;
using scena::EntityState;
using scena::Status;
using scena::testsupport::hex_bits;
using scena::testsupport::make_determinism_scenario;

namespace {

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
    ASSERT_EQ(state_a->pitch, state_b->pitch);
    ASSERT_EQ(state_a->roll, state_b->roll);
    ASSERT_EQ(state_a->speed, state_b->speed);
}

/// Two diagnostic records agree field for field, in order — the by-value
/// warnings (unknown user value, unset time of day) must be deterministic.
void expect_same_diagnostics(const Engine& a, const Engine& b) {
    const auto& da = a.diagnostics();
    const auto& db = b.diagnostics();
    ASSERT_EQ(da.size(), db.size());
    for (std::size_t i = 0; i < da.size(); ++i) {
        EXPECT_EQ(da[i].severity, db[i].severity) << i;
        EXPECT_EQ(da[i].code, db[i].code) << i;
        EXPECT_EQ(da[i].message, db[i].message) << i;
        EXPECT_EQ(da[i].path, db[i].path) << i;
        EXPECT_EQ(da[i].rule_id, db[i].rule_id) << i;
    }
}

/// A one-entity scenario whose speed is driven by by-value conditions: a
/// rising-edge VariableCondition, a UserDefinedValueCondition, an unset
/// UserDefinedValueCondition (warns once) and a TimeOfDayCondition.
scena::ir::Scenario make_byvalue_scenario() {
    using namespace scena::ir;
    Scenario scenario;
    scenario.name = "byvalue-determinism";
    Entity ego;
    ego.id = "ego";
    ego.name = "ego";
    scenario.entities.push_back(std::move(ego));
    scenario.variables["v"] = "wait";

    const auto event = [](std::string name, std::shared_ptr<Condition> expression, double speed) {
        Event e;
        e.name = std::move(name);
        e.start_trigger = make_trigger(std::move(expression));
        e.actions.push_back(std::make_shared<SpeedAction>("ego", speed));
        return e;
    };

    Maneuver maneuver;
    maneuver.name = "maneuver";
    maneuver.events.push_back([&] {
        Event e;
        e.name = "on-variable";
        e.start_trigger = make_trigger(
            std::make_shared<VariableCondition>("v", Rule::EqualTo, "go"), ConditionEdge::Rising);
        e.actions.push_back(std::make_shared<SpeedAction>("ego", 10.0));
        return e;
    }());
    maneuver.events.push_back(
        event("on-user", std::make_shared<UserDefinedValueCondition>("sig", Rule::GreaterThan, "3"),
              20.0));
    maneuver.events.push_back(
        event("on-missing",
              std::make_shared<UserDefinedValueCondition>("absent", Rule::EqualTo, "1"), 30.0));
    maneuver.events.push_back(event("on-time",
                                    std::make_shared<TimeOfDayCondition>(
                                        DateTime{2000, 1, 1, 12, 0, 5, 0, 0}, Rule::GreaterOrEqual),
                                    40.0));

    ManeuverGroup group;
    group.name = "group";
    group.maneuvers.push_back(std::move(maneuver));
    Act act;
    act.name = "act";
    act.groups.push_back(std::move(group));
    Story story;
    story.name = "story";
    story.acts.push_back(std::move(act));
    scenario.storyboard.stories.push_back(std::move(story));
    return scenario;
}

/// Applies the same host-setter sequence both engines are fed, keyed by step
/// index, so two runs stay bit-identical only if the setters are deterministic.
void apply_setters(Engine& engine, int step) {
    if (step == 3) {
        engine.set_variable("v", "go"); // rising edge fires on-variable
    }
    if (step == 5) {
        engine.set_user_defined_value("sig", "5"); // 5 > 3 fires on-user
    }
    if (step == 7) {
        engine.set_date_time(scena::ir::DateTime{2000, 1, 1, 12, 0, 4, 0, 0});
    }
}

/// A scenario whose probes are driven by the by-entity conditions, including
/// the RelativeSpeed directional path (the only trigonometry) and the
/// accumulators. Each event drives a distinct engine-controlled probe.
scena::ir::Scenario make_byentity_scenario() {
    using namespace scena::ir;
    Scenario scenario;
    scenario.name = "byentity-determinism";
    for (const char* id : {"ego", "lead"}) {
        Entity entity;
        entity.id = id;
        entity.name = id;
        entity.control_mode = ControlMode::HostControlled;
        scenario.entities.push_back(std::move(entity));
    }
    for (const char* id : {"auto", "p_speed", "p_rel", "p_dist", "p_still"}) {
        Entity entity;
        entity.id = id;
        entity.name = id;
        entity.control_mode = ControlMode::EngineControlled;
        scenario.entities.push_back(std::move(entity));
    }
    // The engine-controlled "auto" cruises, driving its odometer.
    scenario.init_actions.push_back(std::make_shared<SpeedAction>("auto", 6.0));

    const auto add_event = [&](const std::string& label, std::shared_ptr<Condition> condition,
                               std::string probe) {
        Event event;
        event.name = label;
        event.start_trigger = make_trigger(std::move(condition));
        event.actions.push_back(std::make_shared<SpeedAction>(std::move(probe), 3.0));
        Maneuver maneuver;
        maneuver.name = label + "-m";
        maneuver.events.push_back(std::move(event));
        ManeuverGroup group;
        group.name = label + "-g";
        group.maneuvers.push_back(std::move(maneuver));
        Act act;
        act.name = label + "-a";
        act.groups.push_back(std::move(group));
        Story story;
        story.name = label + "-s";
        story.acts.push_back(std::move(act));
        scenario.storyboard.stories.push_back(std::move(story));
    };
    const TriggeringEntities ego{TriggeringEntitiesRule::Any, {"ego"}};
    add_event("speed", std::make_shared<SpeedCondition>(ego, 5.0, Rule::GreaterOrEqual), "p_speed");
    // Directional relative speed exercises det_sincos with differing headings.
    add_event("rel",
              std::make_shared<RelativeSpeedCondition>(ego, "lead", 0.5, Rule::GreaterThan,
                                                       DirectionalDimension::Lateral),
              "p_rel");
    add_event("dist",
              std::make_shared<TraveledDistanceCondition>(
                  TriggeringEntities{TriggeringEntitiesRule::Any, {"auto"}}, 2.0),
              "p_dist");
    add_event("still",
              std::make_shared<StandStillCondition>(
                  TriggeringEntities{TriggeringEntitiesRule::Any, {"lead"}}, 0.3),
              "p_still");
    return scenario;
}

/// Feeds one engine the deterministic host-input for step `i`: ego accelerates
/// with a turning heading (the trig path), lead alternates rest and motion.
void drive_byentity(Engine& engine, int i) {
    EntityState ego;
    ego.speed = 4.0 + 0.2 * i;
    ego.heading = 0.05 * i;
    ego.x = 0.3 * i;
    engine.report_state("ego", ego);
    EntityState lead;
    lead.speed = (i % 3 == 0) ? 0.0 : 3.0; // periodically at rest for StandStill
    lead.heading = 1.0;
    engine.report_state("lead", lead);
}

/// A box centered on the entity origin (helper for the interaction fixture).
scena::ir::BoundingBox unit_box() {
    scena::ir::BoundingBox box;
    box.length = 4.0;
    box.width = 2.0;
    return box;
}

/// A scenario driven by the interaction metrics (Distance, RelativeDistance,
/// TimeHeadway, TimeToCollision, Collision), each firing a distinct probe. Both
/// observed entities carry bounding boxes and turn, so the freespace geometry
/// exercises make_obb / det_sincos on the hot path.
scena::ir::Scenario make_interaction_scenario() {
    using namespace scena::ir;
    Scenario scenario;
    scenario.name = "interaction-determinism";
    for (const char* id : {"ego", "lead"}) {
        Entity entity;
        entity.id = id;
        entity.name = id;
        entity.control_mode = ControlMode::HostControlled;
        MiscObject obj;
        obj.bounding_box = unit_box();
        entity.object = std::move(obj);
        scenario.entities.push_back(std::move(entity));
    }
    for (const char* id : {"p_dist", "p_rel", "p_thw", "p_ttc", "p_coll"}) {
        Entity entity;
        entity.id = id;
        entity.name = id;
        entity.control_mode = ControlMode::EngineControlled;
        scenario.entities.push_back(std::move(entity));
    }

    const auto add_event = [&](const std::string& label, std::shared_ptr<Condition> condition,
                               std::string probe) {
        Event event;
        event.name = label;
        event.start_trigger = make_trigger(std::move(condition));
        event.actions.push_back(std::make_shared<SpeedAction>(std::move(probe), 3.0));
        Maneuver maneuver;
        maneuver.name = label + "-m";
        maneuver.events.push_back(std::move(event));
        ManeuverGroup group;
        group.name = label + "-g";
        group.maneuvers.push_back(std::move(maneuver));
        Act act;
        act.name = label + "-a";
        act.groups.push_back(std::move(group));
        Story story;
        story.name = label + "-s";
        story.acts.push_back(std::move(act));
        scenario.storyboard.stories.push_back(std::move(story));
    };
    const TriggeringEntities ego{TriggeringEntitiesRule::Any, {"ego"}};
    add_event("dist",
              std::make_shared<DistanceCondition>(ego, WorldPosition{20.0, 0.0, 0.0}, 3.0, true,
                                                  Rule::LessOrEqual),
              "p_dist");
    add_event("rel",
              std::make_shared<RelativeDistanceCondition>(
                  ego, "lead", 2.0, true, RelativeDistanceType::EuclidianDistance, Rule::LessThan),
              "p_rel");
    add_event("thw",
              std::make_shared<TimeHeadwayCondition>(ego, "lead", 3.0, false, Rule::LessOrEqual),
              "p_thw");
    add_event("ttc",
              std::make_shared<TimeToCollisionCondition>(
                  ego, TimeToCollisionTarget{std::string{"lead"}}, 4.0, false, Rule::LessOrEqual),
              "p_ttc");
    add_event("coll", std::make_shared<CollisionCondition>(ego, "lead"), "p_coll");
    return scenario;
}

/// Feeds one engine the deterministic host-input for step `i`: ego closes on a
/// stationary boxed lead while turning (the freespace trig path).
void drive_interaction(Engine& engine, int i) {
    EntityState ego;
    ego.x = 0.3 * i; // approaches lead at x = 20
    ego.speed = 5.0;
    ego.heading = 0.02 * i; // turns, so make_obb rotates
    engine.report_state("ego", ego);
    EntityState lead;
    lead.x = 20.0;
    lead.speed = 0.0;
    lead.heading = 0.01 * i;
    engine.report_state("lead", lead);
}

} // namespace

TEST(DeterminismTest, IdenticalRunsProduceBitIdenticalStates) {
    Engine engine_a;
    Engine engine_b;
    ASSERT_EQ(engine_a.init(make_determinism_scenario()), Status::Ok);
    ASSERT_EQ(engine_b.init(make_determinism_scenario()), Status::Ok);

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
    ASSERT_EQ(engine_a.init(make_determinism_scenario()), Status::Ok);
    ASSERT_EQ(engine_b.init(make_determinism_scenario()), Status::Ok);

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
    ASSERT_EQ(engine_a.init(make_determinism_scenario()), Status::Ok);
    ASSERT_EQ(engine_b.init(make_determinism_scenario()), Status::Ok);

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
    ASSERT_EQ(engine.init(make_determinism_scenario()), Status::Ok);
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
    ASSERT_EQ(engine.init(make_determinism_scenario()), Status::Ok);

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

TEST(DeterminismTest, IdenticalHostSetterSequencesStayBitIdentical) {
    // By-value conditions are driven from outside the scenario. Two engines
    // fed the same variable / user-value / date-time sequence must produce
    // bit-identical entity states and an identical diagnostic stream — the
    // host interface must not smuggle in any nondeterminism.
    Engine engine_a;
    Engine engine_b;
    ASSERT_EQ(engine_a.init(make_byvalue_scenario()), Status::Ok);
    ASSERT_EQ(engine_b.init(make_byvalue_scenario()), Status::Ok);

    for (int i = 0; i < 20; ++i) {
        apply_setters(engine_a, i);
        apply_setters(engine_b, i);
        const double dt = (i % 2 == 0) ? 0.01 : 0.03;
        ASSERT_EQ(engine_a.step(dt), Status::Ok);
        ASSERT_EQ(engine_b.step(dt), Status::Ok);
        SCOPED_TRACE(i);
        expect_bit_identical(engine_a, engine_b, "ego");
    }

    // The rising-edge variable and the user value both fired, so ego was
    // actually driven (guards the determinism claim against a dead scenario).
    const auto ego = engine_a.state("ego");
    ASSERT_TRUE(ego.has_value());
    EXPECT_NE(ego->speed, 0.0);
    expect_same_diagnostics(engine_a, engine_b);
}

TEST(DeterminismTest, ByEntityConditionsAreBitIdenticalAcrossRuns) {
    // Two engines, identical scenario and identical host-input + step
    // sequence (including a zero-dt step and the RelativeSpeed trig path),
    // must produce bit-identical entity states and firing pattern.
    Engine engine_a;
    Engine engine_b;
    ASSERT_EQ(engine_a.init(make_byentity_scenario()), Status::Ok);
    ASSERT_EQ(engine_b.init(make_byentity_scenario()), Status::Ok);

    const std::vector<std::string> probes = {"ego",   "lead",   "auto",   "p_speed",
                                             "p_rel", "p_dist", "p_still"};
    for (int i = 0; i < 60; ++i) {
        drive_byentity(engine_a, i);
        drive_byentity(engine_b, i);
        const double dt = (i % 5 == 0) ? 0.0 : (0.01 + 0.005 * (i % 4));
        ASSERT_EQ(engine_a.step(dt), Status::Ok);
        ASSERT_EQ(engine_b.step(dt), Status::Ok);
        SCOPED_TRACE(i);
        for (const std::string& id : probes) {
            expect_bit_identical(engine_a, engine_b, id);
        }
    }
    expect_same_diagnostics(engine_a, engine_b);
}

TEST(DeterminismTest, InteractionMetricsAreBitIdenticalAcrossRuns) {
    // Two engines fed the same host-input + step sequence must produce
    // bit-identical states across all entities, including the probes the
    // freespace distance / TTC / collision metrics drive (which run make_obb
    // and closing_speed through det_sincos on a turning entity).
    Engine engine_a;
    Engine engine_b;
    ASSERT_EQ(engine_a.init(make_interaction_scenario()), Status::Ok);
    ASSERT_EQ(engine_b.init(make_interaction_scenario()), Status::Ok);

    const std::vector<std::string> ids = {"ego",   "lead",  "p_dist", "p_rel",
                                          "p_thw", "p_ttc", "p_coll"};
    for (int i = 0; i < 70; ++i) {
        drive_interaction(engine_a, i);
        drive_interaction(engine_b, i);
        const double dt = (i % 4 == 0) ? 0.0 : (0.01 + 0.005 * (i % 3));
        ASSERT_EQ(engine_a.step(dt), Status::Ok);
        ASSERT_EQ(engine_b.step(dt), Status::Ok);
        SCOPED_TRACE(i);
        for (const std::string& id : ids) {
            expect_bit_identical(engine_a, engine_b, id);
        }
    }
    expect_same_diagnostics(engine_a, engine_b);

    // Guard against a dead scenario: ego closes on the boxed lead, so the
    // freespace and collision probes must have fired.
    const auto collided = engine_a.state("p_coll");
    ASSERT_TRUE(collided.has_value());
    EXPECT_NE(collided->speed, 0.0);
}

TEST(DeterminismTest, RotatedBoxFreespaceDistanceIsHexPinned) {
    // The whole entity-state → make_obb (det_sincos) → obb_distance path
    // produces a stable IEEE-754 bit pattern. A turning boxed entity at a
    // reference point: the freespace distance must reproduce exactly on every
    // platform. A drift here is a determinism regression.
    using scena::runtime::det_sincos;
    using scena::runtime::Obb2;
    using scena::runtime::obb_distance;
    using scena::runtime::SinCos;

    // Two boxed entities (length 4, width 2), one turned by 0.4 rad.
    const SinCos a = det_sincos(0.0);
    const SinCos b = det_sincos(0.4);
    const Obb2 ego{0.0, 0.0, a.cos, a.sin, 2.0, 1.0};
    const Obb2 lead{8.0, 1.0, b.cos, b.sin, 2.0, 1.0};
    EXPECT_EQ(hex_bits(obb_distance(ego, lead)), "400e2b4cc750362f");
}

TEST(DeterminismTest, PoseBearingHostReportsStayBitIdentical) {
    // A host may report a full pose (pitch/roll); it must survive the
    // report_state -> state round-trip and stay bit-identical across two runs
    // fed the same sequence. Guards the new EntityState pose fields against any
    // nondeterminism in the host path.
    const auto make = [] {
        scena::ir::Scenario s;
        s.name = "pose";
        scena::ir::Entity ego;
        ego.id = "ego";
        ego.name = "ego";
        ego.control_mode = scena::ir::ControlMode::HostControlled;
        s.entities.push_back(std::move(ego));
        return s;
    };
    Engine engine_a;
    Engine engine_b;
    ASSERT_EQ(engine_a.init(make()), Status::Ok);
    ASSERT_EQ(engine_b.init(make()), Status::Ok);

    for (int i = 0; i < 10; ++i) {
        scena::EntityState pose;
        pose.x = 0.5 * i;
        pose.heading = 0.1 * i;
        pose.pitch = 0.05 * i;
        pose.roll = -0.03 * i;
        pose.speed = 2.0;
        ASSERT_EQ(engine_a.report_state("ego", pose), Status::Ok);
        ASSERT_EQ(engine_b.report_state("ego", pose), Status::Ok);
        ASSERT_EQ(engine_a.step(0.02), Status::Ok);
        ASSERT_EQ(engine_b.step(0.02), Status::Ok);
        SCOPED_TRACE(i);
        expect_bit_identical(engine_a, engine_b, "ego");
    }

    // The reported pose actually reached the state (guards a dead assertion).
    const auto ego = engine_a.state("ego");
    ASSERT_TRUE(ego.has_value());
    EXPECT_EQ(ego->pitch, 0.05 * 9);
    EXPECT_EQ(ego->roll, -0.03 * 9);
}

namespace {

/// A single engine-controlled vehicle driven through two transition-dynamics
/// speed ramps — a cubic ramp up and a sinusoidal ramp down — so the
/// determinism suite exercises the longitudinal controller and its det_cos
/// path (p2-s2), not just straight-line kinematics.
scena::ir::Scenario make_dynamics_scenario() {
    using namespace scena::ir;
    Scenario scenario;
    scenario.name = "dynamics-determinism";

    Entity ego;
    ego.id = "ego";
    ego.name = "ego";
    ego.control_mode = ControlMode::EngineControlled;
    Vehicle vehicle;
    vehicle.performance = Performance{40.0, 3.0, 3.0, std::nullopt, std::nullopt};
    ego.object = vehicle;
    scenario.entities.push_back(std::move(ego));
    scenario.init_actions.push_back(std::make_shared<SpeedAction>("ego", 2.0)); // initial speed

    const auto ramp_event = [](std::string name, double at, double target, DynamicsShape shape,
                               double duration) {
        Event event;
        event.name = std::move(name);
        event.start_trigger = make_trigger(std::make_shared<SimulationTimeCondition>(at));
        event.actions.push_back(std::make_shared<SpeedAction>(
            "ego", target, TransitionDynamics{shape, DynamicsDimension::Time, duration}));
        return event;
    };
    Maneuver maneuver;
    maneuver.name = "maneuver";
    // Two non-overlapping ramps (the second starts after the first completes),
    // both within the vehicle's acceleration envelope so neither is clamped.
    maneuver.events.push_back(ramp_event("ramp-up", 0.0, 8.0, DynamicsShape::Cubic, 4.0));
    maneuver.events.push_back(ramp_event("ramp-down", 5.0, 5.0, DynamicsShape::Sinusoidal, 3.0));
    ManeuverGroup group;
    group.name = "group";
    group.maneuvers.push_back(std::move(maneuver));
    Act act;
    act.name = "act";
    act.groups.push_back(std::move(group));
    Story story;
    story.name = "story";
    story.acts.push_back(std::move(act));
    scenario.storyboard.stories.push_back(std::move(story));
    return scenario;
}

/// GS-1, the Cruise-baseline golden scenario (docs/roadmap/golden-scenarios.md):
/// a single engine-controlled vehicle, an init teleport + init speed, and a
/// SimulationTimeCondition at t=5 s that fires an absolute linear SpeedAction.
/// This is the determinism anchor for the p5-s4 private actions — teleport plus
/// a relative-free longitudinal ramp integrated through detmath.
scena::ir::Scenario make_gs1_scenario() {
    using namespace scena::ir;
    Scenario scenario;
    scenario.name = "gs1-cruise-baseline";

    Entity ego;
    ego.id = "ego";
    ego.name = "ego";
    ego.control_mode = ControlMode::EngineControlled;
    Vehicle vehicle;
    vehicle.performance = Performance{60.0, 5.0, 5.0, std::nullopt, std::nullopt};
    ego.object = vehicle;
    scenario.entities.push_back(std::move(ego));

    scenario.init_actions.push_back(
        std::make_shared<TeleportAction>("ego", WorldPosition{0.0, 0.0, 0.0}));
    scenario.init_actions.push_back(std::make_shared<SpeedAction>("ego", 10.0));

    Event accelerate;
    accelerate.name = "accelerate";
    accelerate.start_trigger = make_trigger(std::make_shared<SimulationTimeCondition>(5.0));
    accelerate.actions.push_back(std::make_shared<SpeedAction>(
        "ego", 20.0, TransitionDynamics{DynamicsShape::Linear, DynamicsDimension::Time, 4.0}));
    Maneuver maneuver;
    maneuver.name = "maneuver";
    maneuver.events.push_back(std::move(accelerate));
    ManeuverGroup group;
    group.name = "group";
    group.maneuvers.push_back(std::move(maneuver));
    Act act;
    act.name = "act";
    act.groups.push_back(std::move(group));
    Story story;
    story.name = "story";
    story.acts.push_back(std::move(act));
    scenario.storyboard.stories.push_back(std::move(story));
    return scenario;
}

/// GS-4, the Traffic-jam approach golden scenario
/// (docs/roadmap/golden-scenarios.md): a lead vehicle brakes to a crawl, the
/// ego's TimeToCollisionCondition arms a hard deceleration, a freespace
/// LongitudinalDistanceAction then holds the gap, and a timed trigger dissolves
/// the jam. This is the determinism anchor for the p5-s5 distance-keeping
/// controller — OBB freespace geometry, the deadbeat command and its glide
/// limit, all integrated through detmath.
scena::ir::Scenario make_gs4_scenario() {
    using namespace scena::ir;
    Scenario scenario;
    scenario.name = "gs4-traffic-jam-approach";

    const auto vehicle_entity = [](const char* id, double max_deceleration) {
        Entity entity;
        entity.id = id;
        entity.name = id;
        entity.control_mode = ControlMode::EngineControlled;
        Vehicle vehicle;
        vehicle.bounding_box = BoundingBox{0.0, 0.0, 0.75, 5.0, 2.0, 1.5};
        vehicle.performance = Performance{40.0, 3.0, max_deceleration, std::nullopt, std::nullopt};
        entity.object = vehicle;
        return entity;
    };
    scenario.entities.push_back(vehicle_entity("lead", 6.0));
    scenario.entities.push_back(vehicle_entity("ego", 8.0));
    scenario.init_actions.push_back(
        std::make_shared<TeleportAction>("lead", WorldPosition{150.0, 0.0, 0.0}));
    scenario.init_actions.push_back(
        std::make_shared<TeleportAction>("ego", WorldPosition{0.0, 0.0, 0.0}));
    scenario.init_actions.push_back(std::make_shared<SpeedAction>("lead", 25.0));
    scenario.init_actions.push_back(std::make_shared<SpeedAction>("ego", 25.0));

    Maneuver maneuver;
    maneuver.name = "maneuver";

    Event jam;
    jam.name = "jam-forms";
    jam.start_trigger = make_trigger(std::make_shared<SimulationTimeCondition>(2.0));
    jam.actions.push_back(std::make_shared<SpeedAction>(
        "lead", 0.5, TransitionDynamics{DynamicsShape::Linear, DynamicsDimension::Time, 8.0}));
    maneuver.events.push_back(std::move(jam));

    Event brake;
    brake.name = "ego-brakes";
    brake.start_trigger = make_trigger(std::make_shared<TimeToCollisionCondition>(
        TriggeringEntities{TriggeringEntitiesRule::Any, {"ego"}},
        TimeToCollisionTarget{std::string("lead")}, 6.0, /*freespace=*/true, Rule::LessThan,
        CoordinateSystem::Entity, RelativeDistanceType::Longitudinal));
    brake.actions.push_back(std::make_shared<SpeedAction>(
        "ego", 2.0, TransitionDynamics{DynamicsShape::Linear, DynamicsDimension::Time, 5.0}));
    maneuver.events.push_back(std::move(brake));

    Event keep;
    keep.name = "ego-keeps-gap";
    keep.start_trigger = make_trigger(std::make_shared<StoryboardElementStateCondition>(
        StoryboardElementType::Event, "ego-brakes", StoryboardElementState::CompleteState));
    keep.actions.push_back(std::make_shared<LongitudinalDistanceAction>(
        "ego", "lead", 8.0, std::nullopt, /*freespace=*/true, /*continuous=*/true));
    maneuver.events.push_back(std::move(keep));

    Event dissolve;
    dissolve.name = "jam-dissolves";
    dissolve.start_trigger = make_trigger(std::make_shared<SimulationTimeCondition>(40.0));
    dissolve.actions.push_back(std::make_shared<SpeedAction>(
        "lead", 20.0, TransitionDynamics{DynamicsShape::Linear, DynamicsDimension::Time, 8.0}));
    maneuver.events.push_back(std::move(dissolve));

    ManeuverGroup group;
    group.name = "group";
    group.maneuvers.push_back(std::move(maneuver));
    Act act;
    act.name = "act";
    act.groups.push_back(std::move(group));
    Story story;
    story.name = "story";
    story.acts.push_back(std::move(act));
    scenario.storyboard.stories.push_back(std::move(story));
    return scenario;
}

/// An entity lifecycle scenario (p5-s6): three entities, one of which is
/// deleted mid-run and added back at a new position while the others keep
/// driving. The determinism anchor for §EntityAction — the active flag must
/// leave entity iteration order, the derived-observation accumulators and every
/// other entity's integration untouched.
scena::ir::Scenario make_lifecycle_scenario() {
    using namespace scena::ir;
    Scenario scenario;
    scenario.name = "entity-lifecycle";

    // Ids deliberately out of insertion order relative to the map's sorted
    // order, so a lifecycle change that disturbed iteration would show up.
    for (const char* id : {"charlie", "alpha", "bravo"}) {
        Entity entity;
        entity.id = id;
        entity.name = id;
        entity.control_mode = ControlMode::EngineControlled;
        Vehicle vehicle;
        vehicle.bounding_box = BoundingBox{0.0, 0.0, 0.75, 4.5, 1.9, 1.5};
        vehicle.performance = Performance{50.0, 4.0, 6.0, std::nullopt, std::nullopt};
        entity.object = vehicle;
        scenario.entities.push_back(std::move(entity));
    }
    scenario.init_actions.push_back(
        std::make_shared<TeleportAction>("alpha", WorldPosition{0.0, 0.0, 0.0}));
    scenario.init_actions.push_back(
        std::make_shared<TeleportAction>("bravo", WorldPosition{0.0, 4.0, 0.0}));
    scenario.init_actions.push_back(
        std::make_shared<TeleportAction>("charlie", WorldPosition{0.0, 8.0, 0.0}));
    scenario.init_actions.push_back(std::make_shared<SpeedAction>("alpha", 14.0));
    scenario.init_actions.push_back(std::make_shared<SpeedAction>("bravo", 11.0));
    scenario.init_actions.push_back(std::make_shared<SpeedAction>("charlie", 17.0));

    Maneuver maneuver;
    maneuver.name = "maneuver";

    // A ramp on bravo that is still running when bravo disappears, so the
    // §7.5.2.2 stop path is part of the anchor.
    Event ramp;
    ramp.name = "bravo-ramp";
    ramp.start_trigger = make_trigger(std::make_shared<SimulationTimeCondition>(1.0));
    ramp.actions.push_back(std::make_shared<SpeedAction>(
        "bravo", 30.0,
        TransitionDynamics{DynamicsShape::Sinusoidal, DynamicsDimension::Time, 9.0}));
    maneuver.events.push_back(std::move(ramp));

    Event remove;
    remove.name = "bravo-leaves";
    remove.start_trigger = make_trigger(std::make_shared<SimulationTimeCondition>(3.0));
    remove.actions.push_back(std::make_shared<DeleteEntityAction>("bravo"));
    maneuver.events.push_back(std::move(remove));

    Event restore;
    restore.name = "bravo-returns";
    restore.start_trigger = make_trigger(std::make_shared<SimulationTimeCondition>(6.0));
    restore.actions.push_back(
        std::make_shared<AddEntityAction>("bravo", WorldPosition{-25.0, 4.0, 0.0}));
    maneuver.events.push_back(std::move(restore));

    Event drive;
    drive.name = "bravo-drives-again";
    drive.start_trigger = make_trigger(std::make_shared<SimulationTimeCondition>(6.5));
    drive.actions.push_back(std::make_shared<SpeedAction>(
        "bravo", 19.0, TransitionDynamics{DynamicsShape::Cubic, DynamicsDimension::Time, 3.0}));
    maneuver.events.push_back(std::move(drive));

    ManeuverGroup group;
    group.name = "group";
    group.maneuvers.push_back(std::move(maneuver));
    Act act;
    act.name = "act";
    act.groups.push_back(std::move(group));
    Story story;
    story.name = "story";
    story.acts.push_back(std::move(act));
    scenario.storyboard.stories.push_back(std::move(story));
    return scenario;
}

/// GS-11, the Signalized-intersection golden scenario (the §11.12 worked
/// example): a controller cycles the straight and turning phases, the ego
/// triggers a phase jump when it reaches the stop line, a rising-edge
/// TrafficSignalControllerCondition releases the cross vehicle, a
/// TrafficSignalStateAction forces one bulb into a broken state, and a
/// TrafficSignalCondition notices it. The determinism anchor for p5-s6 — the
/// signal clock, the actions-win precedence and the entity motion they gate all
/// have to reproduce bit for bit.
scena::ir::Scenario make_gs11_scenario() {
    using namespace scena::ir;
    Scenario scenario;
    scenario.name = "gs11-signalized-intersection";

    const auto vehicle_entity = [](const char* id) {
        Entity entity;
        entity.id = id;
        entity.name = id;
        entity.control_mode = ControlMode::EngineControlled;
        Vehicle vehicle;
        vehicle.bounding_box = BoundingBox{0.0, 0.0, 0.75, 4.6, 1.9, 1.5};
        vehicle.performance = Performance{45.0, 3.5, 7.0, std::nullopt, std::nullopt};
        entity.object = vehicle;
        return entity;
    };
    scenario.entities.push_back(vehicle_entity("cross"));
    scenario.entities.push_back(vehicle_entity("ego"));

    // The ego approaches the stop line at x = 120 along y = 0; the cross
    // vehicle waits on the orthogonal approach.
    scenario.init_actions.push_back(
        std::make_shared<TeleportAction>("ego", WorldPosition{0.0, 0.0, 0.0}));
    scenario.init_actions.push_back(
        std::make_shared<TeleportAction>("cross", WorldPosition{130.0, -60.0, 0.0}));
    scenario.init_actions.push_back(std::make_shared<SpeedAction>("ego", 13.5));

    // Two chained controllers: the conflicting approach runs the same cycle a
    // quarter-cycle behind (§6.11.3).
    TrafficSignalController straight;
    straight.name = "straight";
    const auto phase_of = [](const char* name, double duration, const char* signal,
                             const char* state) {
        Phase phase;
        phase.name = name;
        phase.duration = duration;
        phase.signal_states.push_back(TrafficSignalState{signal, state});
        return phase;
    };
    straight.phases.push_back(phase_of("stop", 30.0, "signal-straight", "red"));
    straight.phases.push_back(phase_of("caution", 2.5, "signal-straight", "red;amber"));
    straight.phases.push_back(phase_of("go", 17.5, "signal-straight", "green"));
    scenario.traffic_signal_controllers.push_back(std::move(straight));

    TrafficSignalController turning;
    turning.name = "turning";
    turning.delay = 12.5;
    turning.reference = "straight";
    turning.phases.push_back(phase_of("go", 17.5, "signal-turning", "green"));
    turning.phases.push_back(phase_of("amber", 2.5, "signal-turning", "amber"));
    turning.phases.push_back(phase_of("stop", 30.0, "signal-turning", "red"));
    scenario.traffic_signal_controllers.push_back(std::move(turning));

    Maneuver maneuver;
    maneuver.name = "maneuver";

    // t1: the ego reaches the stop line and requests the phase change.
    Event at_stop_line;
    at_stop_line.name = "at-stop-line";
    at_stop_line.start_trigger = make_trigger(std::make_shared<TraveledDistanceCondition>(
        TriggeringEntities{TriggeringEntitiesRule::Any, {"ego"}}, 120.0));
    at_stop_line.actions.push_back(
        std::make_shared<TrafficSignalControllerAction>("straight", "caution"));
    maneuver.events.push_back(std::move(at_stop_line));

    // t2: the straight signal turns green and the cross vehicle moves off.
    Event release;
    release.name = "cross-releases";
    release.start_trigger =
        make_trigger(std::make_shared<TrafficSignalControllerCondition>("straight", "go"),
                     ConditionEdge::Rising);
    release.actions.push_back(std::make_shared<SpeedAction>(
        "cross", 11.0,
        TransitionDynamics{DynamicsShape::Sinusoidal, DynamicsDimension::Time, 6.0}));
    maneuver.events.push_back(std::move(release));

    // t3: a bulb failure forces one signal into a state no phase produces.
    Event failure;
    failure.name = "bulb-failure";
    failure.start_trigger = make_trigger(std::make_shared<SimulationTimeCondition>(14.0));
    failure.actions.push_back(
        std::make_shared<TrafficSignalStateAction>("signal-turning", "red;green"));
    maneuver.events.push_back(std::move(failure));

    // The scenario ends a while after the failure is noticed.
    Event noticed;
    noticed.name = "failure-noticed";
    noticed.start_trigger =
        make_trigger(std::make_shared<TrafficSignalCondition>("signal-turning", "red;green"),
                     ConditionEdge::Rising, /*delay=*/2.0);
    noticed.actions.push_back(std::make_shared<SpeedAction>(
        "ego", 6.0, TransitionDynamics{DynamicsShape::Linear, DynamicsDimension::Time, 4.0}));
    maneuver.events.push_back(std::move(noticed));

    ManeuverGroup group;
    group.name = "group";
    group.maneuvers.push_back(std::move(maneuver));
    Act act;
    act.name = "act";
    act.groups.push_back(std::move(group));
    Story story;
    story.name = "story";
    story.acts.push_back(std::move(act));
    scenario.storyboard.stories.push_back(std::move(story));
    return scenario;
}

/// A vehicle following a timed polyline that turns a corner, so the trajectory
/// follower's det_atan2 headings and its time interpolation reach the entity
/// state. The p5-s5 anchor for the lateral half of the sprint.
scena::ir::Scenario make_trajectory_scenario() {
    using namespace scena::ir;
    Scenario scenario;
    scenario.name = "trajectory-determinism";

    Entity ego;
    ego.id = "ego";
    ego.name = "ego";
    ego.control_mode = ControlMode::EngineControlled;
    scenario.entities.push_back(std::move(ego));
    scenario.init_actions.push_back(std::make_shared<SpeedAction>("ego", 8.0));

    Trajectory trajectory;
    trajectory.name = "dogleg";
    // Deliberately off-axis segments: the headings are irrational angles, so a
    // libm atan2 would show up as a last-ulp divergence.
    trajectory.vertices.push_back(TrajectoryVertex{WorldPosition{0.0, 0.0, 0.0}, 0.0});
    trajectory.vertices.push_back(TrajectoryVertex{WorldPosition{37.0, 11.0, 0.0}, 4.0});
    trajectory.vertices.push_back(TrajectoryVertex{WorldPosition{58.0, -23.0, 0.0}, 9.0});
    trajectory.vertices.push_back(TrajectoryVertex{WorldPosition{95.5, 4.25, 0.0}, 17.0});

    Event follow;
    follow.name = "follow";
    follow.start_trigger = make_trigger(std::make_shared<SimulationTimeCondition>(1.0));
    follow.actions.push_back(std::make_shared<FollowTrajectoryAction>(
        "ego", trajectory, FollowingMode::Position, Timing{ReferenceContext::Relative, 1.0, 0.5},
        /*initial_distance_offset=*/3.5));
    Maneuver maneuver;
    maneuver.name = "maneuver";
    maneuver.events.push_back(std::move(follow));
    ManeuverGroup group;
    group.name = "group";
    group.maneuvers.push_back(std::move(maneuver));
    Act act;
    act.name = "act";
    act.groups.push_back(std::move(group));
    Story story;
    story.name = "story";
    story.acts.push_back(std::move(act));
    scenario.storyboard.stories.push_back(std::move(story));
    return scenario;
}

} // namespace

TEST(DeterminismTest, GoldenScenario4TrafficJamApproachIsBitIdentical) {
    Engine engine_a;
    Engine engine_b;
    ASSERT_EQ(engine_a.init(make_gs4_scenario()), Status::Ok);
    ASSERT_EQ(engine_b.init(make_gs4_scenario()), Status::Ok);

    // The same non-uniform step pattern the GS-1 anchor uses, over the whole
    // approach, the jam and its dissolution (~50 s).
    const double pattern[] = {0.05, 0.13, 0.09, 0.07};
    for (int i = 0; i < 600; ++i) {
        const double dt = pattern[i % 4];
        SCOPED_TRACE("step " + std::to_string(i));
        ASSERT_EQ(engine_a.step(dt), Status::Ok);
        ASSERT_EQ(engine_b.step(dt), Status::Ok);
        expect_bit_identical(engine_a, engine_b, "ego");
        expect_bit_identical(engine_a, engine_b, "lead");
    }
    expect_same_diagnostics(engine_a, engine_b);

    // Guard against a dead scenario: the ego really did brake and close in.
    ASSERT_TRUE(engine_a.state("ego").has_value());
    ASSERT_TRUE(engine_a.state("lead").has_value());
    const double gap = engine_a.state("lead")->x - engine_a.state("ego")->x - 5.0;
    EXPECT_LT(gap, 20.0);
    EXPECT_GT(gap, 0.0);
}

TEST(DeterminismTest, TrajectoryFollowingIsBitIdentical) {
    Engine engine_a;
    Engine engine_b;
    ASSERT_EQ(engine_a.init(make_trajectory_scenario()), Status::Ok);
    ASSERT_EQ(engine_b.init(make_trajectory_scenario()), Status::Ok);

    const double pattern[] = {0.05, 0.13, 0.09, 0.07};
    double lowest_y = 0.0;
    for (int i = 0; i < 300; ++i) {
        const double dt = pattern[i % 4];
        SCOPED_TRACE("step " + std::to_string(i));
        ASSERT_EQ(engine_a.step(dt), Status::Ok);
        ASSERT_EQ(engine_b.step(dt), Status::Ok);
        expect_bit_identical(engine_a, engine_b, "ego");
        lowest_y = std::fmin(lowest_y, engine_a.state("ego")->y);
    }
    // Guard against a dead scenario: the entity really did steer down to the
    // dogleg's low vertex and the trajectory ran to its end.
    EXPECT_LT(lowest_y, -20.0);
    EXPECT_EQ(*engine_a.storyboard_element_state("story/act/group/maneuver/follow"),
              scena::runtime::ElementState::Complete);
}

TEST(DeterminismTest, TrajectorySegmentHeadingsAreHexPinned) {
    // The follower's headings come from det_atan2 and land in EntityState, so
    // they are part of the bit-identity contract. These are the three segment
    // headings of make_trajectory_scenario's dogleg.
    using scena::runtime::det_atan2;
    EXPECT_EQ(hex_bits(det_atan2(11.0 - 0.0, 37.0 - 0.0)), "3fd27e92b7d10a7a");
    EXPECT_EQ(hex_bits(det_atan2(-23.0 - 11.0, 58.0 - 37.0)), "bff047b02dbf07b0");
    EXPECT_EQ(hex_bits(det_atan2(4.25 - -23.0, 95.5 - 58.0)), "3fe41bd9d8b26dd0");
}

TEST(DeterminismTest, GoldenScenario1CruiseBaselineIsBitIdentical) {
    Engine engine_a;
    Engine engine_b;
    ASSERT_EQ(engine_a.init(make_gs1_scenario()), Status::Ok);
    ASSERT_EQ(engine_b.init(make_gs1_scenario()), Status::Ok);

    // A deterministic, non-uniform step sequence spanning the cruise, the ramp
    // and past its completion (~12 s).
    const double pattern[] = {0.05, 0.13, 0.09, 0.07};
    for (int i = 0; i < 120; ++i) {
        const double dt = pattern[i % 4];
        SCOPED_TRACE("step " + std::to_string(i));
        ASSERT_EQ(engine_a.step(dt), Status::Ok);
        ASSERT_EQ(engine_b.step(dt), Status::Ok);
        expect_bit_identical(engine_a, engine_b, "ego");
    }
    // The ramp completed: speed settled exactly on its target, heading constant.
    ASSERT_TRUE(engine_a.state("ego").has_value());
    EXPECT_EQ(engine_a.state("ego")->speed, 20.0);
    EXPECT_EQ(engine_a.state("ego")->heading, 0.0);
}

TEST(DeterminismTest, GoldenScenario11SignalizedIntersectionIsBitIdentical) {
    Engine engine_a;
    Engine engine_b;
    ASSERT_EQ(engine_a.init(make_gs11_scenario()), Status::Ok);
    ASSERT_EQ(engine_b.init(make_gs11_scenario()), Status::Ok);

    bool saw_forced_state = false;
    const double pattern[] = {0.05, 0.13, 0.09, 0.07};
    for (int i = 0; i < 400; ++i) {
        const double dt = pattern[i % 4];
        SCOPED_TRACE("step " + std::to_string(i));
        ASSERT_EQ(engine_a.step(dt), Status::Ok);
        ASSERT_EQ(engine_b.step(dt), Status::Ok);
        expect_bit_identical(engine_a, engine_b, "ego");
        expect_bit_identical(engine_a, engine_b, "cross");
        saw_forced_state = saw_forced_state || engine_a.traffic_signal_state("signal-turning") ==
                                                   std::optional<std::string>("red;green");
        // The signal state is a string, so it is compared as one — the
        // determinism contract covers every observable, not only the doubles.
        ASSERT_EQ(engine_a.traffic_signal_state("signal-straight"),
                  engine_b.traffic_signal_state("signal-straight"));
        ASSERT_EQ(engine_a.traffic_signal_state("signal-turning"),
                  engine_b.traffic_signal_state("signal-turning"));
        ASSERT_EQ(engine_a.traffic_signal_controller_phase("straight"),
                  engine_b.traffic_signal_controller_phase("straight"));
        ASSERT_EQ(engine_a.traffic_signal_controller_phase("turning"),
                  engine_b.traffic_signal_controller_phase("turning"));
    }
    expect_same_diagnostics(engine_a, engine_b);

    // Guard against a dead scenario: every stage of §11.12 really ran.
    ASSERT_TRUE(engine_a.state("cross").has_value());
    EXPECT_GT(engine_a.state("cross")->speed, 0.0); // the cross vehicle moved off
    EXPECT_EQ(engine_a.state("ego")->speed, 6.0);   // the ego slowed after the failure
    EXPECT_TRUE(saw_forced_state);                  // the bulb failure was injected and held
    // …and the cycle eventually reclaimed the signal at its next phase
    // transition, which is the documented end of a forced state.
    EXPECT_NE(engine_a.traffic_signal_state("signal-turning"),
              std::optional<std::string>("red;green"));
    EXPECT_TRUE(engine_a.traffic_signal_controller_phase("turning").has_value());
}

TEST(DeterminismTest, EntityAddDeleteLifecycleIsBitIdentical) {
    Engine engine_a;
    Engine engine_b;
    ASSERT_EQ(engine_a.init(make_lifecycle_scenario()), Status::Ok);
    ASSERT_EQ(engine_b.init(make_lifecycle_scenario()), Status::Ok);

    const double pattern[] = {0.05, 0.13, 0.09, 0.07};
    for (int i = 0; i < 200; ++i) {
        const double dt = pattern[i % 4];
        SCOPED_TRACE("step " + std::to_string(i));
        ASSERT_EQ(engine_a.step(dt), Status::Ok);
        ASSERT_EQ(engine_b.step(dt), Status::Ok);
        // The entities that stay in the scenario are compared every step; the
        // one cycling in and out is compared through its activity flag, and
        // through its state whenever it has one.
        expect_bit_identical(engine_a, engine_b, "alpha");
        expect_bit_identical(engine_a, engine_b, "charlie");
        ASSERT_EQ(engine_a.entity_active("bravo"), engine_b.entity_active("bravo"));
        const auto bravo_a = engine_a.state("bravo");
        const auto bravo_b = engine_b.state("bravo");
        ASSERT_EQ(bravo_a.has_value(), bravo_b.has_value());
        if (bravo_a.has_value()) {
            expect_bit_identical(engine_a, engine_b, "bravo");
        }
    }
    expect_same_diagnostics(engine_a, engine_b);

    // Guard against a dead scenario: bravo really did leave, come back at the
    // action's position, and drive off again from there.
    ASSERT_EQ(engine_a.entity_active("bravo"), std::optional<bool>(true));
    ASSERT_TRUE(engine_a.state("bravo").has_value());
    EXPECT_GT(engine_a.state("bravo")->x, -25.0);
    EXPECT_EQ(engine_a.state("bravo")->speed, 19.0);
    // The entities that never left are unaffected by the lifecycle churn.
    ASSERT_TRUE(engine_a.state("alpha").has_value());
    EXPECT_EQ(engine_a.state("alpha")->speed, 14.0);
}

TEST(DeterminismTest, LongitudinalDynamicsAreBitIdenticalAcrossRuns) {
    Engine engine_a;
    Engine engine_b;
    ASSERT_EQ(engine_a.init(make_dynamics_scenario()), Status::Ok);
    ASSERT_EQ(engine_b.init(make_dynamics_scenario()), Status::Ok);

    // A deterministic, non-uniform step sequence covering both ramps (~10 s).
    const double pattern[] = {0.03, 0.07, 0.11, 0.05};
    for (int i = 0; i < 160; ++i) {
        const double dt = pattern[i % 4];
        SCOPED_TRACE("step " + std::to_string(i));
        ASSERT_EQ(engine_a.step(dt), Status::Ok);
        ASSERT_EQ(engine_b.step(dt), Status::Ok);
        expect_bit_identical(engine_a, engine_b, "ego");
    }
    // The sinusoidal ramp-down has completed by now; the speed settled exactly
    // on its target through the controller's terminal snap.
    ASSERT_TRUE(engine_a.state("ego").has_value());
    EXPECT_EQ(engine_a.state("ego")->speed, 5.0);
}
