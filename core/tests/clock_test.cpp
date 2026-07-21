// SPDX-License-Identifier: MIT
#include "scena/runtime/clock.h"

#include <vector>

#include <gtest/gtest.h>

using scena::runtime::Clock;

TEST(ClockTest, StartsAtZero) {
    const Clock clock;
    EXPECT_EQ(clock.now(), 0.0);
}

TEST(ClockTest, AccumulatesSteps) {
    Clock clock;
    double expected = 0.0;
    const std::vector<double> steps = {0.01, 0.02, 0.005, 1.0, 0.0};
    for (const double dt : steps) {
        clock.advance(dt);
        expected += dt;
        EXPECT_EQ(clock.now(), expected);
    }
}

TEST(ClockTest, DeterministicAcrossInstances) {
    Clock a;
    Clock b;
    for (int i = 0; i < 10000; ++i) {
        const double dt = (i % 3 == 0) ? 0.01 : 0.0025;
        a.advance(dt);
        b.advance(dt);
        ASSERT_EQ(a.now(), b.now()); // bit-identical, not approximately equal
    }
}

TEST(ClockTest, ResetReturnsToZero) {
    Clock clock;
    clock.advance(12.34);
    ASSERT_GT(clock.now(), 0.0);
    clock.reset();
    EXPECT_EQ(clock.now(), 0.0);
}
