// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
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
