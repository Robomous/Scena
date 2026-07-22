// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#include "scena/ir/entity.h"

#include <optional>
#include <variant>

#include <gtest/gtest.h>

namespace {

using namespace scena::ir;

Vehicle make_vehicle() {
    Vehicle v;
    v.category = VehicleCategory::Car;
    v.role = Role::Civil;
    v.mass = 1500.0;
    v.bounding_box = BoundingBox{1.4, 0.0, 0.8, 4.6, 2.0, 1.5};
    v.performance = Performance{60.0, 5.0, 9.0, std::nullopt, std::nullopt};
    v.axles.rear = Axle{0.0, 2.8, 0.3, 1.6, 0.6};
    v.axles.front = Axle{0.5, 0.0, 0.3, 1.6, 0.6};
    v.properties = {{"color", "blue"}};
    return v;
}

TEST(EntityModelTest, BareParticipantHasNoObjectAndNoDerivedData) {
    Entity e;
    e.id = "ghost";
    e.name = "ghost";
    e.control_mode = ControlMode::HostControlled;
    EXPECT_FALSE(e.object.has_value());
    EXPECT_FALSE(object_type_of(e).has_value());
    EXPECT_FALSE(bounding_box_of(e).has_value());
    EXPECT_EQ(performance_of(e), nullptr);
}

TEST(EntityModelTest, VehicleRoundTripsThroughTheVariant) {
    Entity e;
    e.id = "ego";
    e.name = "ego";
    e.object = make_vehicle();

    ASSERT_TRUE(e.object.has_value());
    ASSERT_TRUE(std::holds_alternative<Vehicle>(*e.object));
    const Vehicle& v = std::get<Vehicle>(*e.object);
    EXPECT_EQ(v.category, VehicleCategory::Car);
    EXPECT_EQ(v.role, Role::Civil);
    ASSERT_TRUE(v.mass.has_value());
    EXPECT_EQ(*v.mass, 1500.0);
    ASSERT_TRUE(v.axles.front.has_value());
    EXPECT_EQ(v.axles.rear.position_x, 2.8);
    ASSERT_EQ(v.properties.size(), 1u);
    EXPECT_EQ(v.properties[0].value, "blue");
}

TEST(EntityModelTest, ObjectTypeMatchesTheAlternative) {
    Entity vehicle;
    vehicle.object = make_vehicle();
    Entity pedestrian;
    pedestrian.object = Pedestrian{};
    Entity misc;
    misc.object = MiscObject{};

    EXPECT_EQ(object_type_of(vehicle), ObjectType::Vehicle);
    EXPECT_EQ(object_type_of(pedestrian), ObjectType::Pedestrian);
    EXPECT_EQ(object_type_of(misc), ObjectType::MiscObject);
}

TEST(EntityModelTest, BoundingBoxOfReadsAcrossEveryAlternative) {
    Entity pedestrian;
    Pedestrian p;
    p.bounding_box = BoundingBox{0.0, 0.0, 0.9, 0.5, 0.6, 1.8};
    pedestrian.object = p;

    const std::optional<BoundingBox> box = bounding_box_of(pedestrian);
    ASSERT_TRUE(box.has_value());
    EXPECT_EQ(box->height, 1.8);
    EXPECT_EQ(box->length, 0.5);
}

TEST(EntityModelTest, PerformanceOfIsVehicleOnly) {
    Entity vehicle;
    vehicle.object = make_vehicle();
    Entity misc;
    misc.object = MiscObject{};

    const Performance* perf = performance_of(vehicle);
    ASSERT_NE(perf, nullptr);
    EXPECT_EQ(perf->max_speed, 60.0);
    // Only Vehicle defines Performance in the standard.
    EXPECT_EQ(performance_of(misc), nullptr);
}

} // namespace
