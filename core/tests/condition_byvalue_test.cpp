// SPDX-License-Identifier: MIT
// By-value conditions and their shared building blocks, per ASAM
// OpenSCENARIO XML 1.4.0: the Rule comparator (§ enum Rule), the dateTime
// value type (preface "Data types"), and the SimulationTime, Parameter,
// Variable, UserDefinedValue, TimeOfDay and StoryboardElementState
// conditions (ByValueCondition group).
#include <cmath>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "scena/ir/date_time.h"
#include "scena/ir/rule.h"

using scena::ir::compare;
using scena::ir::compare_values;
using scena::ir::DateTime;
using scena::ir::parse_scalar;
using scena::ir::Rule;

namespace {

constexpr double kEpoch2000 = 946684800.0; // 2000-01-01T00:00:00Z, seconds.

} // namespace

// ---------------------------------------------------------------------------
// Rule comparator (§ enum Rule)
// ---------------------------------------------------------------------------

TEST(RuleComparatorTest, NumericComparisonsCoverAllSixRules) {
    EXPECT_TRUE(compare(2.0, Rule::EqualTo, 2.0));
    EXPECT_FALSE(compare(2.0, Rule::EqualTo, 3.0));

    EXPECT_TRUE(compare(3.0, Rule::GreaterThan, 2.0));
    EXPECT_FALSE(compare(2.0, Rule::GreaterThan, 2.0));

    EXPECT_TRUE(compare(1.0, Rule::LessThan, 2.0));
    EXPECT_FALSE(compare(2.0, Rule::LessThan, 2.0));

    EXPECT_TRUE(compare(2.0, Rule::GreaterOrEqual, 2.0));
    EXPECT_TRUE(compare(3.0, Rule::GreaterOrEqual, 2.0));
    EXPECT_FALSE(compare(1.0, Rule::GreaterOrEqual, 2.0));

    EXPECT_TRUE(compare(2.0, Rule::LessOrEqual, 2.0));
    EXPECT_TRUE(compare(1.0, Rule::LessOrEqual, 2.0));
    EXPECT_FALSE(compare(3.0, Rule::LessOrEqual, 2.0));

    EXPECT_TRUE(compare(2.0, Rule::NotEqualTo, 3.0));
    EXPECT_FALSE(compare(2.0, Rule::NotEqualTo, 2.0));
}

TEST(RuleComparatorTest, EqualToOnDoublesIsExact) {
    // No tolerance: 0.1 + 0.2 is not 0.3 in IEEE-754, and the comparator must
    // reflect that rather than smoothing it over with an epsilon.
    EXPECT_FALSE(compare(0.1 + 0.2, Rule::EqualTo, 0.3));
    EXPECT_TRUE(compare(0.3, Rule::EqualTo, 0.3));
    // The orientation of the ordering rules is stable regardless of magnitude.
    EXPECT_TRUE(compare(1e308, Rule::GreaterThan, 1.0));
}

TEST(RuleComparatorTest, StringEqualityAndInequality) {
    // Neither operand is a scalar, so only equality/inequality apply, byte
    // for byte (ParameterCondition scalar-convertibility clause).
    EXPECT_TRUE(compare_values("left", Rule::EqualTo, "left"));
    EXPECT_FALSE(compare_values("left", Rule::EqualTo, "right"));
    EXPECT_TRUE(compare_values("left", Rule::NotEqualTo, "right"));
    EXPECT_FALSE(compare_values("left", Rule::NotEqualTo, "left"));
    // No boolean coercion in this phase: "true" and "1" are distinct strings.
    EXPECT_FALSE(compare_values("true", Rule::EqualTo, "1"));
}

TEST(RuleComparatorTest, NumericStringsCompareNumerically) {
    // Both sides parse: the comparison is numeric, so "5" == "5.0" and
    // ordering is meaningful. Left operand is the stored value.
    EXPECT_TRUE(compare_values("5", Rule::EqualTo, "5.0"));
    EXPECT_TRUE(compare_values("16.667", Rule::GreaterThan, "5"));
    EXPECT_TRUE(compare_values("+5", Rule::EqualTo, "5"));
    EXPECT_TRUE(compare_values("5", Rule::LessOrEqual, "5"));
}

TEST(RuleComparatorTest, OrderingOnNonNumericStringsIsFalse) {
    // "Less and greater operators will only be supported if the value ... can
    // unambiguously be converted into a scalar" — otherwise every ordering
    // rule is false, even when the strings differ.
    EXPECT_FALSE(compare_values("apple", Rule::LessThan, "banana"));
    EXPECT_FALSE(compare_values("apple", Rule::GreaterThan, "banana"));
    EXPECT_FALSE(compare_values("apple", Rule::GreaterOrEqual, "apple"));
    EXPECT_FALSE(compare_values("apple", Rule::LessOrEqual, "apple"));
    // One numeric, one not: still not both-scalar, so ordering is false.
    EXPECT_FALSE(compare_values("5", Rule::GreaterThan, "five"));
}

TEST(RuleComparatorTest, PartialAndLocaleLikeTokensAreNotScalars) {
    EXPECT_FALSE(parse_scalar("").has_value());
    EXPECT_FALSE(parse_scalar("1,5").has_value());  // locale decimal comma
    EXPECT_FALSE(parse_scalar("1.5x").has_value()); // trailing remainder
    EXPECT_FALSE(parse_scalar(" 1.5").has_value()); // leading whitespace
    EXPECT_FALSE(parse_scalar("+").has_value());
    EXPECT_FALSE(parse_scalar("+-5").has_value());
    ASSERT_TRUE(parse_scalar("16.667").has_value());
    EXPECT_DOUBLE_EQ(*parse_scalar("16.667"), 16.667);
    ASSERT_TRUE(parse_scalar("+5").has_value());
    EXPECT_DOUBLE_EQ(*parse_scalar("+5"), 5.0);
    ASSERT_TRUE(parse_scalar("-2.5").has_value());
    EXPECT_DOUBLE_EQ(*parse_scalar("-2.5"), -2.5);
}

TEST(RuleComparatorTest, NanOperandsFollowIeeeSemantics) {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    EXPECT_FALSE(compare(nan, Rule::EqualTo, nan));
    EXPECT_TRUE(compare(nan, Rule::NotEqualTo, nan));
    EXPECT_FALSE(compare(nan, Rule::GreaterThan, 0.0));
    EXPECT_FALSE(compare(nan, Rule::LessThan, 0.0));
    EXPECT_FALSE(compare(nan, Rule::GreaterOrEqual, 0.0));
    EXPECT_FALSE(compare(nan, Rule::LessOrEqual, 0.0));
    EXPECT_FALSE(compare(0.0, Rule::LessThan, nan));
}

// ---------------------------------------------------------------------------
// DateTime value type (preface "Data types", Table 5)
// ---------------------------------------------------------------------------

TEST(DateTimeTest, EpochAndFractionAreExact) {
    EXPECT_DOUBLE_EQ(DateTime{}.to_epoch_seconds(), 0.0); // 1970-01-01T00:00:00Z
    const DateTime y2k{2000, 1, 1, 0, 0, 0, 0, 0};
    EXPECT_DOUBLE_EQ(y2k.to_epoch_seconds(), kEpoch2000);
    const DateTime half_second{1970, 1, 1, 0, 0, 0, 500, 0};
    EXPECT_DOUBLE_EQ(half_second.to_epoch_seconds(), 0.5);
}

TEST(DateTimeTest, CanonicalizationHandlesLeapYearsAndOffsets) {
    // 2000 is a leap year (divisible by 400): Feb 29 exists and March 1 is
    // one day after it.
    const DateTime leap_day{2000, 2, 29, 0, 0, 0, 0, 0};
    const DateTime march_first{2000, 3, 1, 0, 0, 0, 0, 0};
    EXPECT_DOUBLE_EQ(march_first.to_epoch_seconds() - leap_day.to_epoch_seconds(), 86400.0);
    EXPECT_DOUBLE_EQ(march_first.to_epoch_seconds(), kEpoch2000 + 60.0 * 86400.0);

    // A +01:00 zone is one hour ahead of UTC, so the same wall reading is an
    // hour earlier in epoch terms.
    const DateTime utc_one_am{1970, 1, 1, 1, 0, 0, 0, 0};
    const DateTime cet_one_am{1970, 1, 1, 1, 0, 0, 0, 60};
    EXPECT_DOUBLE_EQ(utc_one_am.to_epoch_seconds(), 3600.0);
    EXPECT_DOUBLE_EQ(cet_one_am.to_epoch_seconds(), 0.0);
    // A western offset moves the instant later.
    const DateTime west{1970, 1, 1, 0, 0, 0, 0, -300}; // -05:00
    EXPECT_DOUBLE_EQ(west.to_epoch_seconds(), 5.0 * 3600.0);
}

TEST(DateTimeTest, ValidatesFieldRangesAndDayInMonth) {
    EXPECT_TRUE((DateTime{2000, 2, 29, 23, 59, 59, 999, 0}.valid()));
    EXPECT_TRUE((DateTime{}.valid()));
    // 2001 is not a leap year, so Feb 29 does not exist.
    EXPECT_FALSE((DateTime{2001, 2, 29, 0, 0, 0, 0, 0}.valid()));
    EXPECT_FALSE((DateTime{2000, 4, 31, 0, 0, 0, 0, 0}.valid())); // April has 30
    EXPECT_FALSE((DateTime{2000, 0, 1, 0, 0, 0, 0, 0}.valid()));  // month underflow
    EXPECT_FALSE((DateTime{2000, 13, 1, 0, 0, 0, 0, 0}.valid())); // month overflow
    EXPECT_FALSE((DateTime{2000, 1, 1, 24, 0, 0, 0, 0}.valid())); // hour overflow
    EXPECT_FALSE((DateTime{2000, 1, 1, 0, 60, 0, 0, 0}.valid())); // minute overflow
    EXPECT_FALSE((DateTime{2000, 1, 1, 0, 0, 60, 0, 0}.valid())); // second overflow
    EXPECT_FALSE((DateTime{2000, 1, 1, 0, 0, 0, 1000, 0}.valid())); // ms overflow
    EXPECT_FALSE((DateTime{2000, 1, 1, 0, 0, 0, 0, 15 * 60}.valid())); // offset too wide
}
