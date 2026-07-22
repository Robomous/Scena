// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <cstddef>
#include <iterator>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "scena/runtime/detmath.h"
#include "support/detmath_probe.h"
#include "support/trace_recorder.h"

using scena::runtime::det_atan2;
using scena::runtime::det_cos;
using scena::runtime::det_sin;
using scena::runtime::det_sincos;
using scena::runtime::kDetTrigMaxAbsInput;
using scena::runtime::SinCos;
using scena::testsupport::Atan2Input;
using scena::testsupport::detmath_atan2_probe_inputs;
using scena::testsupport::detmath_probe_inputs;
using scena::testsupport::hex_bits;

namespace {

// Expected bit patterns for each detmath_probe_inputs() entry, in the same
// order. Captured once from this implementation (NOT from libm docs or math
// tables): build core, run each probe input through det_sin/det_cos, print
// hex_bits, paste. The 3-OS determinism-trace CI matrix is the cross-platform
// proof these bits are identical everywhere; AccuracyAgainstLibm proves they
// are also numerically correct. Regenerate the same way if the algorithm ever
// changes (a golden-breaking, release-noted event).
struct Expected {
    const char* sin_hex;
    const char* cos_hex;
};

constexpr Expected kExpected[] = {
    {"3e10000000000000", "3ff0000000000000"}, // 0x1p-30
    {"be10000000000000", "3ff0000000000000"}, // -0x1p-30
    {"3fd2e9cd95baba33", "3fee921dd42f09ba"}, // 0.3
    {"3fdeaee8744b05f0", "3fec1528065b7d50"}, // 0.5
    {"bfdeaee8744b05f0", "3fec1528065b7d50"}, // -0.5
    {"3feaed548f090cee", "3fe14a280fb5068c"}, // 1.0
    {"bfeaed548f090cee", "3fe14a280fb5068c"}, // -1.0
    {"3fe6a09e667f3bcc", "3fe6a09e667f3bcd"}, // ~pi/4
    {"3ff0000000000000", "3c91a62633145800"}, // ~pi/2
    {"3ca1a62633145800", "bff0000000000000"}, // ~pi
    {"bcb1a62633145800", "3ff0000000000000"}, // ~2pi
    {"3fe326af0dcfcab0", "bfe9a2f7ef858b7d"}, // 2.5
    {"bfef9f12fcee5458", "3fc3a1c134c1c59e"}, // -7.7
    {"bfe1689ef5f34f53", "bfead9ac890c6b1f"}, // 10.0
    {"bfe03425b78c4db8", "3feb981dbf665fdf"}, // 100.0
    {"bfe68298a1cec146", "3fe6be7c89fe4a8f"}, // 12345.6789
    {"bfd6664b2568d867", "3fedf9df9906d32c"}, // 1.0e6
    {"3fd6664b2568d867", "3fedf9df9906d32c"}, // -1.0e6
};

TEST(DetMathTest, PinnedBitPatterns) {
    const std::vector<double>& inputs = detmath_probe_inputs();
    ASSERT_EQ(inputs.size(), std::size(kExpected));
    for (std::size_t i = 0; i < inputs.size(); ++i) {
        const double x = inputs[i];
        SCOPED_TRACE(testing::Message() << "input=" << x);
        EXPECT_EQ(hex_bits(det_sin(x)), kExpected[i].sin_hex);
        EXPECT_EQ(hex_bits(det_cos(x)), kExpected[i].cos_hex);
    }
}

TEST(DetMathTest, ExactAnchors) {
    // Signed zero flows through unchanged; cosine of zero is exactly one.
    EXPECT_EQ(hex_bits(det_sin(0.0)), "0000000000000000");
    EXPECT_EQ(hex_bits(det_sin(-0.0)), "8000000000000000");
    EXPECT_EQ(det_cos(0.0), 1.0);
    EXPECT_EQ(det_cos(-0.0), 1.0);
    // Paired accessor agrees with the anchors.
    EXPECT_EQ(hex_bits(det_sincos(-0.0).sin), "8000000000000000");
    EXPECT_EQ(det_sincos(0.0).cos, 1.0);
}

TEST(DetMathTest, DomainPolicy) {
    const double qnan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    // Out of domain -> quiet NaN. Assert is-NaN, never specific NaN bits: the
    // payload is out of contract and may differ across platforms.
    for (double x : {qnan, inf, -inf, 2.0e6, -2.0e6}) {
        SCOPED_TRACE(testing::Message() << "input=" << x);
        EXPECT_TRUE(std::isnan(det_sin(x)));
        EXPECT_TRUE(std::isnan(det_cos(x)));
    }
    // The boundary itself is in-domain and finite.
    EXPECT_TRUE(std::isfinite(det_sin(kDetTrigMaxAbsInput)));
    EXPECT_TRUE(std::isfinite(det_cos(kDetTrigMaxAbsInput)));
    EXPECT_TRUE(std::isfinite(det_sin(-kDetTrigMaxAbsInput)));
}

TEST(DetMathTest, AccuracyAgainstLibm) {
    // Guards against a wrong-but-consistent implementation: pinned bits alone
    // would happily lock in a bug. libm is allowed here (this is a test, and
    // the detmath guard scans only core/src + core/include).
    constexpr int kSamples = 2000;
    for (int i = 0; i <= kSamples; ++i) {
        const double x = -20.0 + 40.0 * (static_cast<double>(i) / kSamples);
        SCOPED_TRACE(testing::Message() << "x=" << x);
        EXPECT_NEAR(det_sin(x), std::sin(x), 1e-12);
        EXPECT_NEAR(det_cos(x), std::cos(x), 1e-12);
    }
    for (const double x : detmath_probe_inputs()) {
        SCOPED_TRACE(testing::Message() << "probe=" << x);
        EXPECT_NEAR(det_sin(x), std::sin(x), 1e-12);
        EXPECT_NEAR(det_cos(x), std::cos(x), 1e-12);
    }
}

// Expected bit patterns for each detmath_atan2_probe_inputs() pair, in the
// same order and captured the same way as kExpected above.
constexpr const char* kExpectedAtan2[] = {
    "3fe921fb54442d18", // atan2(1, 1)
    "4002d97c7f3321d2", // atan2(1, -1)
    "bfe921fb54442d18", // atan2(-1, 1)
    "c002d97c7f3321d2", // atan2(-1, -1)
    "0000000000000000", // atan2(0, 1)
    "8000000000000000", // atan2(-0, 1)
    "400921fb54442d18", // atan2(0, -1)
    "c00921fb54442d18", // atan2(-0, -1)
    "3ff921fb54442d18", // atan2(1, 0)
    "bff921fb54442d18", // atan2(-1, 0)
    "0000000000000000", // atan2(0, 0)
    "c00921fb54442d18", // atan2(-0, -0)
    "3fcf5b75f92c80dd", // atan2(0.5, 2)
    "3ff5368c951e9cfc", // atan2(2, 0.5)
    "bfcf5b75f92c80dd", // atan2(-0.5, 2)
    "40072c43f4b1650a", // atan2(0.5, -2)
    "3fe4978fa3269ee2", // atan2(3, 4)
    "bff30243abab7cf6", // atan2(-7.7, 3.1)
    "3fd0c1517765ae7d", // atan2(0.267949, 1)
    "3ff4f1a6f66ac179", // atan2(1, 0.267949)
    "3e45798ee2308c3a", // atan2(1e-8, 1)
    "3ff921fb5194fb3c", // atan2(1, 1e-8)
    "3ff921fb54442d18", // atan2(1e300, 1e-300)
    "40068aea78723686", // atan2(12.5, -37.25)
    "bffd0d6a1369bd34", // atan2(-0.125, -0.03125)
};

TEST(DetMathTest, PinnedAtan2BitPatterns) {
    const std::vector<Atan2Input>& inputs = detmath_atan2_probe_inputs();
    ASSERT_EQ(inputs.size(), std::size(kExpectedAtan2));
    for (std::size_t i = 0; i < inputs.size(); ++i) {
        SCOPED_TRACE(testing::Message() << "atan2(" << inputs[i].y << ", " << inputs[i].x << ")");
        EXPECT_EQ(hex_bits(det_atan2(inputs[i].y, inputs[i].x)), kExpectedAtan2[i]);
    }
}

TEST(DetMathTest, Atan2QuadrantAndSignedZeroAnchors) {
    // The IEEE 754 / C99 atan2 special cases for finite arguments.
    EXPECT_EQ(hex_bits(det_atan2(0.0, 1.0)), "0000000000000000");  // +0
    EXPECT_EQ(hex_bits(det_atan2(-0.0, 1.0)), "8000000000000000"); // -0
    EXPECT_EQ(det_atan2(0.0, -1.0), det_atan2(0.0, -2.0));         // +pi either way
    EXPECT_GT(det_atan2(0.0, -1.0), 3.14);
    EXPECT_LT(det_atan2(-0.0, -1.0), -3.14);
    // A zero x is still an axis, and its own sign decides 0 versus pi.
    EXPECT_EQ(hex_bits(det_atan2(0.0, 0.0)), "0000000000000000");
    EXPECT_LT(det_atan2(-0.0, -0.0), -3.14);
    EXPECT_NEAR(det_atan2(1.0, 0.0), 1.5707963267948966, 1e-15);
    EXPECT_NEAR(det_atan2(-1.0, 0.0), -1.5707963267948966, 1e-15);
}

TEST(DetMathTest, Atan2DomainPolicy) {
    const double qnan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    // Non-finite arguments are out of contract -> quiet NaN (unlike libm,
    // which defines infinity cases); see detmath.h.
    for (const double bad : {qnan, inf, -inf}) {
        SCOPED_TRACE(testing::Message() << "bad=" << bad);
        EXPECT_TRUE(std::isnan(det_atan2(bad, 1.0)));
        EXPECT_TRUE(std::isnan(det_atan2(1.0, bad)));
    }
}

TEST(DetMathTest, Atan2AccuracyAgainstLibm) {
    // Sweep every quadrant on a circle plus a wide range of ratios; libm is
    // the independent reference (the detmath guard scans only core/).
    constexpr int kSamples = 720;
    for (int i = 0; i < kSamples; ++i) {
        const double angle = -3.14159 + 6.28318 * (static_cast<double>(i) / kSamples);
        const double y = 7.5 * std::sin(angle);
        const double x = 7.5 * std::cos(angle);
        SCOPED_TRACE(testing::Message() << "angle=" << angle);
        EXPECT_NEAR(det_atan2(y, x), std::atan2(y, x), 1e-12);
    }
    for (const Atan2Input& probe : detmath_atan2_probe_inputs()) {
        SCOPED_TRACE(testing::Message() << "atan2(" << probe.y << ", " << probe.x << ")");
        EXPECT_NEAR(det_atan2(probe.y, probe.x), std::atan2(probe.y, probe.x), 1e-12);
    }
}

TEST(DetMathTest, Atan2InvertsDetSincos) {
    // The follower composes the two: a segment heading goes through det_atan2
    // and comes back out of det_sincos when the position is integrated.
    for (int i = -180; i < 180; ++i) {
        const double angle = static_cast<double>(i) * 3.14159265358979 / 180.0;
        const SinCos sc = det_sincos(angle);
        SCOPED_TRACE(testing::Message() << "angle=" << angle);
        EXPECT_NEAR(det_atan2(sc.sin, sc.cos), angle, 1e-12);
    }
}

TEST(DetMathTest, SinCosConsistency) {
    for (const double x : detmath_probe_inputs()) {
        SCOPED_TRACE(testing::Message() << "input=" << x);
        const SinCos sc = det_sincos(x);
        // The paired accessor must be bit-identical to the scalar entry points.
        EXPECT_EQ(hex_bits(sc.sin), hex_bits(det_sin(x)));
        EXPECT_EQ(hex_bits(sc.cos), hex_bits(det_cos(x)));
        // Pythagorean identity holds to a few ulp.
        EXPECT_NEAR(sc.sin * sc.sin + sc.cos * sc.cos, 1.0, 1e-12);
    }
}

} // namespace
