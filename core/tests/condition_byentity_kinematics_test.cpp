// SPDX-License-Identifier: MIT
// By-entity conditions (ASAM OpenSCENARIO XML 1.4.0 §7.6.5.1): the
// triggering-entities any/all frame and the six kinematic/position
// conditions. This file covers the shared frame and the pure-context
// per-entity predicates; engine-driven derived-state tests live alongside.
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "scena/entity_state.h"
#include "scena/ir/action.h"
#include "scena/ir/condition.h"
#include "scena/ir/entity_condition.h"
#include "scena/ir/evaluation_context.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/trigger.h"
#include "scena/runtime/scheduler.h"

namespace ir = scena::ir;
using ir::DirectionalDimension;
using ir::EntityKinematics;
using ir::TriggeringEntities;
using ir::TriggeringEntitiesRule;
using scena::EntityState;
using scena::runtime::ActionOutcome;
using scena::runtime::Scheduler;

namespace {

/// A context that answers entity_kinematics from a fixed table — the pure-unit
/// driver for the by-entity predicates (no engine, no storyboard).
class KinematicsContext final : public ir::EvaluationContext {
public:
    explicit KinematicsContext(std::map<std::string, EntityKinematics> entities)
        : entities_(std::move(entities)) {}

    [[nodiscard]] double simulation_time() const override { return 0.0; }

    [[nodiscard]] std::optional<EntityKinematics>
    entity_kinematics(std::string_view id) const override {
        const auto it = entities_.find(std::string(id));
        if (it == entities_.end()) {
            return std::nullopt;
        }
        return it->second;
    }

private:
    std::map<std::string, EntityKinematics> entities_;
};

/// Minimal by-entity predicate for exercising the any/all frame: an entity
/// "holds" when it is observable and moving.
class MovingProbe final : public ir::ByEntityCondition {
public:
    explicit MovingProbe(TriggeringEntities triggering)
        : ir::ByEntityCondition(std::move(triggering)) {}

protected:
    [[nodiscard]] bool evaluate_for_entity(const ir::EvaluationContext& context,
                                           std::string_view entity_id) const override {
        const auto kinematics = context.entity_kinematics(entity_id);
        return kinematics.has_value() && kinematics->state.speed > 0.0;
    }
};

TriggeringEntities triggering(TriggeringEntitiesRule rule, std::vector<std::string> refs) {
    return TriggeringEntities{rule, std::move(refs)};
}

EntityKinematics with_speed(double speed) {
    EntityKinematics kinematics;
    kinematics.state.speed = speed;
    return kinematics;
}

bool holds(const ir::Condition& condition, const ir::EvaluationContext& context) {
    return condition.evaluate(context);
}

} // namespace

// ---------------------------------------------------------------------------
// The entity-kinematics facet (ADR-0007 default-absent contract)
// ---------------------------------------------------------------------------

TEST(EntityKinematicsFacetTest, TimeOnlyContextHasNoEntityFacet) {
    const ir::TimeOnlyEvaluationContext context{0.0};
    EXPECT_FALSE(context.entity_kinematics("ego").has_value());
    // A by-entity condition over a facet-less context is deterministically false.
    const MovingProbe probe{triggering(TriggeringEntitiesRule::Any, {"ego"})};
    EXPECT_FALSE(holds(probe, context));
}

// ---------------------------------------------------------------------------
// TriggeringEntities any/all reduction (§7.6.5.1)
// ---------------------------------------------------------------------------

TEST(TriggeringEntitiesTest, AnyAllReductionMatrix) {
    const KinematicsContext context{{
        {"mover", with_speed(5.0)},
        {"still", with_speed(0.0)},
        {"other", with_speed(3.0)},
    }};

    // Any: at least one entity moving.
    EXPECT_TRUE(holds(MovingProbe{triggering(TriggeringEntitiesRule::Any, {"mover", "still"})},
                      context));
    EXPECT_FALSE(holds(MovingProbe{triggering(TriggeringEntitiesRule::Any, {"still"})}, context));
    // All: every entity moving.
    EXPECT_TRUE(holds(MovingProbe{triggering(TriggeringEntitiesRule::All, {"mover", "other"})},
                      context));
    EXPECT_FALSE(holds(MovingProbe{triggering(TriggeringEntitiesRule::All, {"mover", "still"})},
                       context));
    // Single entity: any and all agree.
    EXPECT_TRUE(holds(MovingProbe{triggering(TriggeringEntitiesRule::Any, {"mover"})}, context));
    EXPECT_TRUE(holds(MovingProbe{triggering(TriggeringEntitiesRule::All, {"mover"})}, context));
}

TEST(TriggeringEntitiesTest, EmptyRefsIsAlwaysFalse) {
    const KinematicsContext context{{{"mover", with_speed(5.0)}}};
    EXPECT_FALSE(holds(MovingProbe{triggering(TriggeringEntitiesRule::Any, {})}, context));
    EXPECT_FALSE(holds(MovingProbe{triggering(TriggeringEntitiesRule::All, {})}, context));
}

TEST(TriggeringEntitiesTest, UnobservableEntityContributesFalsePerEntity) {
    // An entity absent from the context is false for that entity only: Any can
    // still hold through another ref; All cannot (ADR-0007 finest grain).
    const KinematicsContext context{{{"mover", with_speed(5.0)}}};
    EXPECT_TRUE(holds(MovingProbe{triggering(TriggeringEntitiesRule::Any, {"mover", "ghost"})},
                      context));
    EXPECT_FALSE(holds(MovingProbe{triggering(TriggeringEntitiesRule::All, {"mover", "ghost"})},
                       context));
}

// ---------------------------------------------------------------------------
// BoundContext forwards the facet to the host context
// ---------------------------------------------------------------------------

TEST(EntityKinematicsFacetTest, BoundContextForwardsEntityKinematics) {
    // A by-entity trigger inside a bound storyboard must observe the entity
    // state the host context supplies — i.e. Scheduler::BoundContext forwards
    // the facet rather than swallowing it.
    ir::Event event;
    event.name = "event";
    ir::Trigger trigger;
    ir::ConditionGroup group;
    ir::TriggerCondition condition;
    condition.expression =
        std::make_shared<MovingProbe>(triggering(TriggeringEntitiesRule::Any, {"ego"}));
    group.conditions.push_back(std::move(condition));
    trigger.groups.push_back(std::move(group));
    event.start_trigger = std::move(trigger);
    event.actions.push_back(std::make_shared<ir::SpeedAction>("ego", 10.0));

    ir::Maneuver maneuver;
    maneuver.name = "maneuver";
    maneuver.events.push_back(std::move(event));
    ir::ManeuverGroup mgroup;
    mgroup.name = "group";
    mgroup.maneuvers.push_back(std::move(maneuver));
    ir::Act act;
    act.name = "act";
    act.groups.push_back(std::move(mgroup));
    ir::Story story;
    story.name = "story";
    story.acts.push_back(std::move(act));
    ir::Storyboard storyboard;
    storyboard.stories.push_back(std::move(story));

    Scheduler scheduler;
    scheduler.bind(storyboard);

    // Host context where ego is not moving: the event stays in standby.
    const KinematicsContext still{{{"ego", with_speed(0.0)}}};
    bool fired = false;
    const auto fire = [&fired](const ir::Action&) {
        fired = true;
        return ActionOutcome::Complete;
    };
    scheduler.step(still, fire);
    EXPECT_FALSE(fired);

    // Host context where ego is moving: the forwarded facet makes it fire.
    const KinematicsContext moving{{{"ego", with_speed(4.0)}}};
    scheduler.step(moving, fire);
    EXPECT_TRUE(fired);
}
