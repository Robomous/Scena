// SPDX-License-Identifier: Apache-2.0
#include "scena/ir/entity_types.h"

#include <optional>

#include <gtest/gtest.h>

namespace {

using namespace scena::ir;

TEST(EntityTypesTest, PerformanceDefaultsAreZeroAndRatesAbsent) {
    Performance p;
    EXPECT_EQ(p.max_speed, 0.0);
    EXPECT_EQ(p.max_acceleration, 0.0);
    EXPECT_EQ(p.max_deceleration, 0.0);
    // Absent rate limits mean "unbounded" (§Performance): default to nullopt.
    EXPECT_FALSE(p.max_acceleration_rate.has_value());
    EXPECT_FALSE(p.max_deceleration_rate.has_value());
}

TEST(EntityTypesTest, PerformanceHoldsSuppliedRates) {
    Performance p{50.0, 3.0, 9.0, 2.5, 8.0};
    EXPECT_EQ(p.max_speed, 50.0);
    ASSERT_TRUE(p.max_acceleration_rate.has_value());
    EXPECT_EQ(*p.max_acceleration_rate, 2.5);
    ASSERT_TRUE(p.max_deceleration_rate.has_value());
    EXPECT_EQ(*p.max_deceleration_rate, 8.0);
}

TEST(EntityTypesTest, AxleDefaultsAreZero) {
    Axle a;
    EXPECT_EQ(a.max_steering, 0.0);
    EXPECT_EQ(a.position_x, 0.0);
    EXPECT_EQ(a.position_z, 0.0);
    EXPECT_EQ(a.track_width, 0.0);
    EXPECT_EQ(a.wheel_diameter, 0.0);
}

TEST(EntityTypesTest, AxlesHaveRequiredRearOptionalFrontOrderedAdditional) {
    Axles axles;
    axles.rear = Axle{0.0, 3.8, 0.3, 1.6, 0.6};
    EXPECT_FALSE(axles.front.has_value());
    EXPECT_TRUE(axles.additional.empty());

    axles.front = Axle{0.5, 0.0, 0.3, 1.6, 0.6};
    axles.additional.push_back(Axle{0.0, 5.0, 0.3, 1.6, 0.6});
    axles.additional.push_back(Axle{0.0, 6.0, 0.3, 1.6, 0.6});
    ASSERT_TRUE(axles.front.has_value());
    ASSERT_EQ(axles.additional.size(), 2u);
    // Document order is preserved (a vector, not a set).
    EXPECT_EQ(axles.additional[0].position_x, 5.0);
    EXPECT_EQ(axles.additional[1].position_x, 6.0);
}

TEST(EntityTypesTest, PropertiesPreserveOrderAndDuplicates) {
    std::vector<Property> props{{"color", "red"}, {"color", "blue"}, {"z", "1"}};
    ASSERT_EQ(props.size(), 3u);
    EXPECT_EQ(props[0].name, "color");
    EXPECT_EQ(props[0].value, "red");
    // Duplicate names survive — Properties is an ordered list, not a map.
    EXPECT_EQ(props[1].value, "blue");
}

TEST(EntityTypesTest, EnumeratorsAreDistinct) {
    // A couple of representative spec-order / deprecation invariants.
    EXPECT_NE(VehicleCategory::Motorbike, VehicleCategory::Motorcycle);
    EXPECT_NE(Role::Fire, Role::FireBrigade);
    EXPECT_NE(ObjectType::Vehicle, ObjectType::Pedestrian);
    static_assert(static_cast<int>(Role::None) == 0, "default Role is None (§Vehicle/§Pedestrian)");
}

} // namespace
