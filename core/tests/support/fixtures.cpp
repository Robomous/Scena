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

#include "support/fixtures.h"

#include <memory>
#include <utility>

#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/trigger.h"

namespace scena::testsupport {

using ir::Act;
using ir::ControlMode;
using ir::Event;
using ir::Maneuver;
using ir::ManeuverGroup;
using ir::Scenario;
using ir::SimulationTimeCondition;
using ir::SpeedAction;
using ir::Story;

Event make_speed_event(std::string name, double at_time, std::string entity_id,
                       double target_speed) {
    Event event;
    event.name = std::move(name);
    event.start_trigger = ir::make_trigger(std::make_shared<SimulationTimeCondition>(at_time));
    event.actions.push_back(std::make_shared<SpeedAction>(std::move(entity_id), target_speed));
    return event;
}

Scenario make_determinism_scenario() {
    Scenario scenario;
    scenario.name = "determinism-test";
    scenario.entities.push_back(
        {.id = "ego", .name = "ego vehicle", .control_mode = ControlMode::EngineControlled});
    scenario.entities.push_back(
        {.id = "lead", .name = "lead vehicle", .control_mode = ControlMode::EngineControlled});
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
        delayed.start_trigger = ir::make_trigger(std::make_shared<SimulationTimeCondition>(0.7),
                                                 ir::ConditionEdge::Rising, 0.35);
        delayed.actions.push_back(std::make_shared<SpeedAction>("ego", 9.5));
        // The three priority branches of §8.4.2.2 are all walked, even
        // though no action driven through Engine is ever ongoing, so no
        // sibling is ever in runningState when another starts: both of
        // these resolve to a plain start and must do so identically in
        // every run.
        delayed.priority = ir::EventPriority::Skip;
        maneuver.events.push_back(std::move(delayed));
        maneuver.events[1].priority = ir::EventPriority::Override; // "settle"

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
        act.start_trigger = ir::make_trigger(std::make_shared<SimulationTimeCondition>(1.0));
        // Act stop trigger: the stop cascade must run at the same step in
        // both runs (§7.6.1.2).
        act.stop_trigger = ir::make_trigger(std::make_shared<SimulationTimeCondition>(2.5));
        act.groups.push_back(std::move(group));
        Story story;
        story.name = "lead-story";
        story.acts.push_back(std::move(act));
        scenario.storyboard.stories.push_back(std::move(story));
    }
    return scenario;
}

} // namespace scena::testsupport
