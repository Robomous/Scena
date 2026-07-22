// SPDX-License-Identifier: MIT
// The minimal entity bounding box (ASAM OpenSCENARIO XML 1.4.0 BoundingBox,
// p5-s3 forward-pull): default value, init-time validation, and the threading
// from ir::Entity through the engine into the entity-kinematics facet the
// interaction conditions read.
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include <gtest/gtest.h>

#include "scena/engine.h"
#include "scena/ir/action.h"
#include "scena/ir/bounding_box.h"
#include "scena/ir/condition.h"
#include "scena/ir/evaluation_context.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"
#include "scena/ir/trigger.h"
#include "scena/status.h"

namespace ir = scena::ir;
using ir::BoundingBox;
using scena::Engine;
using scena::Status;

namespace {

/// A probe condition that holds iff the observed entity carries a bounding box
/// whose length matches `expected_length` exactly — a direct read of the facet
/// the engine threads the geometry through.
class BoxProbe final : public ir::Condition {
public:
    BoxProbe(std::string entity_id, double expected_length)
        : entity_id_(std::move(entity_id)), expected_length_(expected_length) {}

    [[nodiscard]] bool evaluate(const ir::EvaluationContext& context) const override {
        const auto kinematics = context.entity_kinematics(entity_id_);
        return kinematics.has_value() && kinematics->bounding_box.has_value() &&
               kinematics->bounding_box->length == expected_length_;
    }

private:
    std::string entity_id_;
    double expected_length_;
};

/// A scenario with one host entity `ego` (optionally boxed) plus an
/// engine-controlled probe an event drives to speed 99 when `condition` holds.
ir::Scenario probe_scenario(std::optional<BoundingBox> ego_box,
                            std::shared_ptr<ir::Condition> condition) {
    ir::Scenario scenario;
    scenario.name = "bbox";
    ir::Entity ego;
    ego.id = "ego";
    ego.name = "ego";
    ego.control_mode = ir::ControlMode::HostControlled;
    ego.bounding_box = ego_box;
    scenario.entities.push_back(std::move(ego));
    ir::Entity probe;
    probe.id = "probe";
    probe.name = "probe";
    probe.control_mode = ir::ControlMode::EngineControlled;
    scenario.entities.push_back(std::move(probe));

    ir::Event event;
    event.name = "event";
    event.start_trigger = ir::make_trigger(std::move(condition));
    event.actions.push_back(std::make_shared<ir::SpeedAction>("probe", 99.0));
    ir::Maneuver maneuver;
    maneuver.name = "maneuver";
    maneuver.events.push_back(std::move(event));
    ir::ManeuverGroup group;
    group.name = "group";
    group.maneuvers.push_back(std::move(maneuver));
    ir::Act act;
    act.name = "act";
    act.groups.push_back(std::move(group));
    ir::Story story;
    story.name = "story";
    story.acts.push_back(std::move(act));
    scenario.storyboard.stories.push_back(std::move(story));
    return scenario;
}

bool probe_fired(const Engine& engine) {
    const auto state = engine.state("probe");
    return state.has_value() && state->speed != 0.0;
}

BoundingBox vehicle_box() {
    return BoundingBox{/*center_x=*/1.5, /*center_y=*/0.0, /*center_z=*/0.9,
                       /*length=*/4.0,   /*width=*/2.0,    /*height=*/1.8};
}

} // namespace

TEST(BoundingBoxTest, DefaultIsAZeroPointBox) {
    const BoundingBox box;
    EXPECT_EQ(box.center_x, 0.0);
    EXPECT_EQ(box.center_y, 0.0);
    EXPECT_EQ(box.center_z, 0.0);
    EXPECT_EQ(box.length, 0.0);
    EXPECT_EQ(box.width, 0.0);
    EXPECT_EQ(box.height, 0.0);
}

TEST(BoundingBoxTest, EngineThreadsBoxToKinematicsFacet) {
    // The box authored on ego reaches the facet the condition reads, values
    // intact, at the t = 0 evaluation.
    Engine engine;
    ASSERT_EQ(engine.init(probe_scenario(vehicle_box(), std::make_shared<BoxProbe>("ego", 4.0))),
              Status::Ok);
    EXPECT_TRUE(probe_fired(engine));
}

TEST(BoundingBoxTest, AbsentBoxLeavesFacetGeometryEmpty) {
    // No box authored ⇒ the facet reports no geometry, so a box-reading
    // condition is false.
    Engine engine;
    ASSERT_EQ(engine.init(probe_scenario(std::nullopt, std::make_shared<BoxProbe>("ego", 4.0))),
              Status::Ok);
    EXPECT_FALSE(probe_fired(engine));
}

TEST(BoundingBoxTest, ZeroSizeBoxIsValid) {
    // A degenerate point box (all dimensions 0) is schema-valid: it must not
    // fail init.
    Engine engine;
    EXPECT_EQ(engine.init(probe_scenario(BoundingBox{}, std::make_shared<BoxProbe>("ego", 0.0))),
              Status::Ok);
}

TEST(BoundingBoxTest, NegativeDimensionFailsInit) {
    BoundingBox box = vehicle_box();
    box.width = -1.0;
    Engine engine;
    EXPECT_EQ(engine.init(probe_scenario(box, std::make_shared<BoxProbe>("ego", 4.0))),
              Status::ValidationError);
}

TEST(BoundingBoxTest, NanDimensionOrCenterFailsInit) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    {
        BoundingBox box = vehicle_box();
        box.length = nan;
        Engine engine;
        EXPECT_EQ(engine.init(probe_scenario(box, std::make_shared<BoxProbe>("ego", 4.0))),
                  Status::ValidationError);
    }
    {
        BoundingBox box = vehicle_box();
        box.center_y = nan;
        Engine engine;
        EXPECT_EQ(engine.init(probe_scenario(box, std::make_shared<BoxProbe>("ego", 4.0))),
                  Status::ValidationError);
    }
}
