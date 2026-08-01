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
// Gateway maturity (p6-s2, #36): the step brackets, the storyboard-transition
// callback and its deterministic order, the host-clock patterns the engine
// promises to support (fixed dt, variable dt, zero-dt query steps), and
// attaching or detaching a gateway after construction.

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "scena/engine.h"
#include "scena/gateway/simulator_gateway.h"
#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/dynamics.h"
#include "scena/ir/entity.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/trigger.h"

namespace {

using scena::Engine;
using scena::EntityState;
using scena::Status;
using scena::ir::ControlMode;
using scena::ir::DynamicsDimension;
using scena::ir::DynamicsShape;
using scena::ir::Entity;
using scena::ir::Scenario;
using scena::ir::SimulationTimeCondition;
using scena::ir::SpeedAction;
using scena::ir::TransitionDynamics;
using scena::runtime::ElementState;
using scena::runtime::TransitionKind;

/// Records the whole callback stream as strings, which is what makes the order
/// itself assertable rather than only the individual calls.
class RecordingGateway final : public scena::gateway::ISimulatorGateway {
public:
    std::vector<std::string> log;
    /// Filled into every host-controlled entity that is polled, when set.
    std::optional<EntityState> polled;

    void publish_state(const std::string& entity_id, const EntityState& /*state*/) override {
        log.push_back("publish:" + entity_id);
    }

    bool poll_state(const std::string& entity_id, EntityState& out) override {
        log.push_back("poll:" + entity_id);
        if (!polled.has_value()) {
            return false;
        }
        out = *polled;
        return true;
    }

    scena::gateway::IRoadQuery* road_query() override { return nullptr; }

    void on_step_begin(double dt) override { log.push_back("begin:" + format(dt)); }
    void on_step_end(double dt) override { log.push_back("end:" + format(dt)); }

    void on_element_transition(const std::string& path, ElementState state,
                               TransitionKind transition) override {
        log.push_back("transition:" + (path.empty() ? std::string("<storyboard>") : path) + ":" +
                      name_of(state) + ":" + name_of(transition));
    }

private:
    static std::string format(double dt) {
        // Enough to tell 0 from 0.5 from 1 without depending on locale.
        return dt == 0.0 ? "0" : (dt == 0.5 ? "0.5" : "1");
    }

    static const char* name_of(ElementState state) {
        switch (state) {
        case ElementState::Standby:
            return "standby";
        case ElementState::Running:
            return "running";
        case ElementState::Complete:
            return "complete";
        }
        return "?";
    }

    static const char* name_of(TransitionKind transition) {
        switch (transition) {
        case TransitionKind::None:
            return "none";
        case TransitionKind::Start:
            return "start";
        case TransitionKind::End:
            return "end";
        case TransitionKind::Stop:
            return "stop";
        case TransitionKind::Skip:
            return "skip";
        }
        return "?";
    }
};

Entity make_entity(std::string id, ControlMode mode) {
    Entity entity;
    entity.id = id;
    entity.name = std::move(id);
    entity.control_mode = mode;
    return entity;
}

/// One entity, one event that ramps its speed to 10 over 2 s starting at t = 1.
Scenario make_scenario(ControlMode mode = ControlMode::EngineControlled) {
    Scenario scenario;
    scenario.name = "gateway";
    scenario.entities.push_back(make_entity("ego", mode));

    scena::ir::Event event;
    event.name = "event";
    event.start_trigger = scena::ir::make_trigger(std::make_shared<SimulationTimeCondition>(1.0));
    event.actions.push_back(std::make_shared<SpeedAction>(
        "ego", 10.0, TransitionDynamics{DynamicsShape::Linear, DynamicsDimension::Time, 2.0}));
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
    return scenario;
}

std::vector<std::string> filter(const std::vector<std::string>& log, std::string_view prefix) {
    std::vector<std::string> selected;
    for (const std::string& entry : log) {
        if (entry.rfind(prefix, 0) == 0) {
            selected.push_back(entry);
        }
    }
    return selected;
}

// --- step brackets ---------------------------------------------------------

TEST(GatewayTest, EveryStepIsBracketed) {
    RecordingGateway gateway;
    Engine engine(&gateway);
    ASSERT_EQ(engine.init(make_scenario()), Status::Ok);
    gateway.log.clear(); // init has no brackets; it is not a step

    ASSERT_EQ(engine.step(1.0), Status::Ok);
    ASSERT_FALSE(gateway.log.empty());
    EXPECT_EQ(gateway.log.front(), "begin:1");
    EXPECT_EQ(gateway.log.back(), "end:1");
}

TEST(GatewayTest, ARejectedStepOpensNoBracket) {
    // Nothing was opened, so nothing must be closed — a host that begins a
    // transaction in on_step_begin must not be left holding one.
    RecordingGateway gateway;
    Engine engine(&gateway);
    ASSERT_EQ(engine.init(make_scenario()), Status::Ok);
    gateway.log.clear();

    EXPECT_EQ(engine.step(-1.0), Status::InvalidArgument);
    EXPECT_TRUE(gateway.log.empty());

    Engine uninitialized(&gateway);
    EXPECT_EQ(uninitialized.step(1.0), Status::NotInitialized);
    EXPECT_TRUE(gateway.log.empty());
}

TEST(GatewayTest, TheStepOrderIsTheOneADR0003Fixes) {
    RecordingGateway gateway;
    Engine engine(&gateway);
    ASSERT_EQ(engine.init(make_scenario(ControlMode::HostControlled)), Status::Ok);
    gateway.log.clear();

    // t = 1: the event starts, so this step exercises every hook.
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    // The story, act, group and maneuver started with the storyboard at init —
    // none of them owns a start trigger — so this step is the event's alone.
    const std::vector<std::string> expected{
        "begin:1",
        "poll:ego", // before evaluation
        "transition:story/act/group/maneuver/event:running:start",
        "end:1",
    };
    // A host-controlled entity is polled and never published, which is what
    // per-entity control ownership means.
    EXPECT_EQ(gateway.log, expected);
}

// --- storyboard transitions ------------------------------------------------

TEST(GatewayTest, TransitionsAreReportedOnceAndInDocumentOrder) {
    RecordingGateway gateway;
    Engine engine(&gateway);
    // init's evaluation at t = 0 is a real evaluation (§8.4.7): the storyboard
    // and the elements that start with it transition there.
    ASSERT_EQ(engine.init(make_scenario()), Status::Ok);
    // Everything without a start trigger enters runningState there, parents
    // first. The event, which has one, waits.
    EXPECT_EQ(filter(gateway.log, "transition:"),
              (std::vector<std::string>{
                  "transition:<storyboard>:running:start",
                  "transition:story:running:start",
                  "transition:story/act:running:start",
                  "transition:story/act/group:running:start",
                  "transition:story/act/group/maneuver:running:start",
              }));

    gateway.log.clear();
    ASSERT_EQ(engine.step(1.0), Status::Ok); // the event starts
    EXPECT_EQ(filter(gateway.log, "transition:"),
              (std::vector<std::string>{
                  "transition:story/act/group/maneuver/event:running:start",
              }));

    // A transition is a one-evaluation pulse: the next step, with nothing
    // happening, reports nothing.
    gateway.log.clear();
    ASSERT_EQ(engine.step(0.5), Status::Ok);
    EXPECT_TRUE(filter(gateway.log, "transition:").empty());

    // The ramp finishes at t = 3, ending the event and everything above it.
    gateway.log.clear();
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok);
    // Every level ends in the same evaluation, and the walk is parents-first —
    // so the stream reads top-down even though the event is what finished.
    EXPECT_EQ(filter(gateway.log, "transition:"),
              (std::vector<std::string>{
                  "transition:story:complete:end",
                  "transition:story/act:complete:end",
                  "transition:story/act/group:complete:end",
                  "transition:story/act/group/maneuver:complete:end",
                  "transition:story/act/group/maneuver/event:complete:end",
              }));
}

TEST(GatewayTest, TheCallbackStreamIsIdenticalAcrossRepeats) {
    // The determinism contract reaches the observer: two identical runs produce
    // element-wise identical callback streams.
    const auto run = []() {
        RecordingGateway gateway;
        Engine engine(&gateway);
        EXPECT_EQ(engine.init(make_scenario()), Status::Ok);
        for (int i = 0; i < 50; ++i) {
            EXPECT_EQ(engine.step(0.1), Status::Ok);
        }
        return gateway.log;
    };
    EXPECT_EQ(run(), run());
}

// --- host clock patterns ---------------------------------------------------

TEST(GatewayTest, AZeroDtStepIsAFullEvaluation) {
    // The query-step pattern: a host that needs the storyboard re-evaluated
    // without advancing time steps with dt = 0. It is a real evaluation — it
    // brackets, polls, publishes and reports transitions — and the clock does
    // not move.
    RecordingGateway gateway;
    Engine engine(&gateway);
    ASSERT_EQ(engine.init(make_scenario()), Status::Ok);
    ASSERT_EQ(engine.step(1.0), Status::Ok); // t = 1, the event starts
    const double time_before = engine.time();
    const double speed_before = engine.state("ego")->speed;

    gateway.log.clear();
    ASSERT_EQ(engine.step(0.0), Status::Ok);
    EXPECT_EQ(engine.time(), time_before);
    EXPECT_EQ(engine.state("ego")->speed, speed_before); // no motion, no ramp
    EXPECT_EQ(gateway.log.front(), "begin:0");
    EXPECT_EQ(gateway.log.back(), "end:0");
    EXPECT_FALSE(filter(gateway.log, "publish:").empty());
}

TEST(GatewayTest, FixedAndVariableStepsReachTheSamePlace) {
    // The engine integrates explicitly, so a fixed-dt host and a variable-dt
    // host do not generally agree mid-ramp — but both reach the target, and a
    // host that replays the same dt sequence gets the same numbers. Only the
    // second property is a contract; this pins it.
    const auto run = [](const std::vector<double>& dts) {
        Engine engine;
        EXPECT_EQ(engine.init(make_scenario()), Status::Ok);
        for (const double dt : dts) {
            EXPECT_EQ(engine.step(dt), Status::Ok);
        }
        return engine.state("ego")->speed;
    };
    const std::vector<double> variable{0.3, 1.1, 0.05, 0.4, 0.7, 0.2, 1.25, 0.5};
    EXPECT_EQ(run(variable), run(variable));

    // Both patterns arrive at the target once the ramp has had its 2 s.
    EXPECT_DOUBLE_EQ(run({1.0, 1.0, 1.0, 1.0}), 10.0);
    EXPECT_DOUBLE_EQ(run({0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5}), 10.0);
}

TEST(GatewayTest, PollingInjectsHostState) {
    RecordingGateway gateway;
    EntityState injected;
    injected.x = 12.5;
    injected.speed = 7.25;
    gateway.polled = injected;

    Engine engine(&gateway);
    ASSERT_EQ(engine.init(make_scenario(ControlMode::HostControlled)), Status::Ok);
    ASSERT_EQ(engine.step(0.5), Status::Ok);
    EXPECT_EQ(engine.state("ego")->x, 12.5);
    EXPECT_EQ(engine.state("ego")->speed, 7.25);
    // A host-controlled entity is not integrated by the engine, so its state is
    // exactly what the host reported.
    ASSERT_EQ(engine.step(0.5), Status::Ok);
    EXPECT_EQ(engine.state("ego")->x, 12.5);
}

// --- attaching and detaching ----------------------------------------------

TEST(GatewayTest, AGatewayCanBeAttachedAndDetachedAfterConstruction) {
    RecordingGateway gateway;
    Engine engine;
    EXPECT_EQ(engine.gateway(), nullptr);
    ASSERT_EQ(engine.init(make_scenario()), Status::Ok);
    ASSERT_EQ(engine.step(0.5), Status::Ok);
    EXPECT_TRUE(gateway.log.empty()); // nothing attached yet

    engine.set_gateway(&gateway);
    EXPECT_EQ(engine.gateway(), &gateway);
    ASSERT_EQ(engine.step(0.5), Status::Ok);
    EXPECT_FALSE(gateway.log.empty());

    gateway.log.clear();
    engine.set_gateway(nullptr);
    EXPECT_EQ(engine.gateway(), nullptr);
    ASSERT_EQ(engine.step(0.5), Status::Ok);
    EXPECT_TRUE(gateway.log.empty());
}

} // namespace
