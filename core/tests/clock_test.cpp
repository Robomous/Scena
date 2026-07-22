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
