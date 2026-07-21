// SPDX-License-Identifier: MIT
#include <cmath>
#include <cstddef>
#include <iterator>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "scena/runtime/detmath.h"
#include "support/detmath_probe.h"
#include "support/trace_recorder.h"

using scena::runtime::det_cos;
using scena::runtime::det_sin;
using scena::runtime::det_sincos;
using scena::runtime::kDetTrigMaxAbsInput;
using scena::runtime::SinCos;
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
