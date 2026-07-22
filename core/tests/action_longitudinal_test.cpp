// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
//
// Private longitudinal actions at the action level (p5-s4): SpeedAction with a
// relative target (§RelativeTargetSpeed) resolved against a reference entity,
// both one-shot and continuous, plus multi-action supersession. The controller
// math itself is covered by longitudinal_test.cpp; here the focus is the IR
// surface and the engine's cross-entity resolution.

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "scena/engine.h"
#include "scena/ir/action.h"
#include "scena/ir/dynamics.h"
#include "scena/ir/entity.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"

namespace {

using scena::Engine;
using scena::Status;
using scena::ir::ControlMode;
using scena::ir::DynamicsDimension;
using scena::ir::DynamicsShape;
using scena::ir::Entity;
using scena::ir::RelativeTargetSpeed;
using scena::ir::Scenario;
using scena::ir::SpeedAction;
using scena::ir::SpeedTargetValueType;
using scena::ir::TransitionDynamics;

constexpr double kTol = 1e-12;

// --- IR surface -----------------------------------------------------------

TEST(SpeedActionIrTest, AbsoluteTargetIsNotRelative) {
    SpeedAction action("ego", 12.0);
    EXPECT_FALSE(action.is_relative());
    EXPECT_DOUBLE_EQ(action.target_speed(), 12.0);
    EXPECT_FALSE(action.relative_target().has_value());
    EXPECT_EQ(action.kind(), "SpeedAction");
    // The default short form is a Step transition.
    EXPECT_EQ(action.dynamics().shape, DynamicsShape::Step);
}

TEST(SpeedActionIrTest, RelativeTargetCarriesReferenceAndValueType) {
    RelativeTargetSpeed target{"lead", 5.0, SpeedTargetValueType::Delta, /*continuous=*/false};
    SpeedAction action("ego", target, {DynamicsShape::Linear, DynamicsDimension::Time, 3.0});
    ASSERT_TRUE(action.is_relative());
    ASSERT_TRUE(action.relative_target().has_value());
    EXPECT_EQ(action.relative_target()->entity_ref, "lead");
    EXPECT_DOUBLE_EQ(action.relative_target()->value, 5.0);
    EXPECT_EQ(action.relative_target()->value_type, SpeedTargetValueType::Delta);
    EXPECT_FALSE(action.relative_target()->continuous);
    EXPECT_EQ(action.dynamics().shape, DynamicsShape::Linear);
}

TEST(SpeedActionIrTest, ContinuousRelativeTargetIsFlagged) {
    RelativeTargetSpeed target{"lead", 1.1, SpeedTargetValueType::Factor, /*continuous=*/true};
    SpeedAction action("ego", target, {});
    ASSERT_TRUE(action.relative_target().has_value());
    EXPECT_TRUE(action.relative_target()->continuous);
    EXPECT_EQ(action.relative_target()->value_type, SpeedTargetValueType::Factor);
    EXPECT_NEAR(action.relative_target()->value, 1.1, kTol);
}

} // namespace
