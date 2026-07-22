// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
//
// TeleportAction (p5-s4): world-frame teleport. A one-shot action that writes an
// entity's world position and completes in the evaluation it fires. Only the
// §WorldPosition target is modeled (the PositionResolver and the other §6.3.8
// variants arrive with p2-s4/p3-s4).

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "scena/engine.h"
#include "scena/ir/action.h"
#include "scena/ir/entity.h"
#include "scena/ir/position.h"
#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"

namespace {

using scena::ir::TeleportAction;
using scena::ir::WorldPosition;

// --- IR surface -----------------------------------------------------------

TEST(TeleportActionIrTest, CarriesEntityAndWorldPosition) {
    TeleportAction action("ego", WorldPosition{10.0, -4.0, 1.5});
    EXPECT_EQ(action.entity_id(), "ego");
    EXPECT_EQ(action.kind(), "TeleportAction");
    EXPECT_DOUBLE_EQ(action.position().x, 10.0);
    EXPECT_DOUBLE_EQ(action.position().y, -4.0);
    EXPECT_DOUBLE_EQ(action.position().z, 1.5);
}

} // namespace
