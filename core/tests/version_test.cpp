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

#include "scena/version.h"

#include <regex>
#include <string>

#include <gtest/gtest.h>

TEST(VersionTest, ConstantsComposeTheString) {
    const std::string expected = std::to_string(scena::kVersionMajor) + "." +
                                 std::to_string(scena::kVersionMinor) + "." +
                                 std::to_string(scena::kVersionPatch);
    EXPECT_EQ(scena::version_string(), expected);
}

TEST(VersionTest, StringIsSemanticVersionFormat) {
    const std::regex semver(R"(\d+\.\d+\.\d+)");
    EXPECT_TRUE(std::regex_match(scena::version_string(), semver));
}

TEST(VersionTest, CurrentVersionValues) {
    EXPECT_EQ(scena::kVersionMajor, 0);
    EXPECT_EQ(scena::kVersionMinor, 1);
    EXPECT_EQ(scena::kVersionPatch, 0);
    EXPECT_EQ(scena::version_string(), "0.1.0");
}
