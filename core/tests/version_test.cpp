// SPDX-License-Identifier: MIT
#include "kinema/version.h"

#include <regex>
#include <string>

#include <gtest/gtest.h>

TEST(VersionTest, ConstantsComposeTheString) {
    const std::string expected = std::to_string(kinema::kVersionMajor) + "." +
                                 std::to_string(kinema::kVersionMinor) + "." +
                                 std::to_string(kinema::kVersionPatch);
    EXPECT_EQ(kinema::version_string(), expected);
}

TEST(VersionTest, StringIsSemanticVersionFormat) {
    const std::regex semver(R"(\d+\.\d+\.\d+)");
    EXPECT_TRUE(std::regex_match(kinema::version_string(), semver));
}

TEST(VersionTest, CurrentVersionValues) {
    EXPECT_EQ(kinema::kVersionMajor, 0);
    EXPECT_EQ(kinema::kVersionMinor, 1);
    EXPECT_EQ(kinema::kVersionPatch, 0);
    EXPECT_EQ(kinema::version_string(), "0.1.0");
}
