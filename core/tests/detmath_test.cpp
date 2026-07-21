// SPDX-License-Identifier: MIT
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>

#include <gtest/gtest.h>

#include "scena/runtime/detmath.h"

using scena::runtime::det_cos;
using scena::runtime::det_sin;
using scena::runtime::det_sincos;
using scena::runtime::kDetTrigMaxAbsInput;
using scena::runtime::SinCos;

namespace {

// 16 lowercase hex chars of a double's IEEE-754 bit pattern. Locale-immune and
// exact: this is the representation the cross-platform trace diff compares, so
// the tests pin the same bytes CI pins.
std::string hex_bits(double value) {
    const auto bits = std::bit_cast<std::uint64_t>(value);
    static constexpr char kDigits[] = "0123456789abcdef";
    std::string out(16, '0');
    for (int i = 0; i < 16; ++i) {
        out[static_cast<std::size_t>(i)] = kDigits[(bits >> ((15 - i) * 4)) & 0xF];
    }
    return out;
}

struct Probe {
    double input;
    const char* sin_hex;
    const char* cos_hex;
};

// Bit patterns captured once from this implementation (see the honest-capture
// note below), NOT from libm docs or math tables. The 3-OS determinism-trace
// CI matrix is the cross-platform proof that these bits are identical
// everywhere; AccuracyAgainstLibm proves they are also numerically correct.
//
// Capture procedure: build core, run each input through det_sin/det_cos, print
// hex_bits, paste. Regenerate the same way if the algorithm ever changes (a
// golden-breaking, release-noted event).
constexpr Probe kProbes[] = {
    {0x1.0p-30, "3e10000000000000", "3ff0000000000000"},
    {-0x1.0p-30, "be10000000000000", "3ff0000000000000"},
    {0.3, "3fd2e9cd95baba33", "3fee921dd42f09ba"},
    {0.5, "3fdeaee8744b05f0", "3fec1528065b7d50"},
    {-0.5, "bfdeaee8744b05f0", "3fec1528065b7d50"},
    {1.0, "3feaed548f090cee", "3fe14a280fb5068c"},
    {-1.0, "bfeaed548f090cee", "3fe14a280fb5068c"},
    {0x1.921fb54442d18p-1, "3fe6a09e667f3bcc", "3fe6a09e667f3bcd"}, // ~pi/4
    {0x1.921fb54442d18p+0, "3ff0000000000000", "3c91a62633145800"}, // ~pi/2
    {0x1.921fb54442d18p+1, "3ca1a62633145800", "bff0000000000000"}, // ~pi
    {0x1.921fb54442d18p+2, "bcb1a62633145800", "3ff0000000000000"}, // ~2pi
    {2.5, "3fe326af0dcfcab0", "bfe9a2f7ef858b7d"},
    {-7.7, "bfef9f12fcee5458", "3fc3a1c134c1c59e"},
    {10.0, "bfe1689ef5f34f53", "bfead9ac890c6b1f"},
    {100.0, "bfe03425b78c4db8", "3feb981dbf665fdf"},
    {12345.6789, "bfe68298a1cec146", "3fe6be7c89fe4a8f"},
    {1.0e6, "bfd6664b2568d867", "3fedf9df9906d32c"},
    {-1.0e6, "3fd6664b2568d867", "3fedf9df9906d32c"},
};

TEST(DetMathTest, PinnedBitPatterns) {
    for (const Probe& p : kProbes) {
        SCOPED_TRACE(testing::Message() << "input=" << p.input);
        EXPECT_EQ(hex_bits(det_sin(p.input)), p.sin_hex);
        EXPECT_EQ(hex_bits(det_cos(p.input)), p.cos_hex);
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
    for (const Probe& p : kProbes) {
        SCOPED_TRACE(testing::Message() << "probe=" << p.input);
        EXPECT_NEAR(det_sin(p.input), std::sin(p.input), 1e-12);
        EXPECT_NEAR(det_cos(p.input), std::cos(p.input), 1e-12);
    }
}

TEST(DetMathTest, SinCosConsistency) {
    for (const Probe& p : kProbes) {
        SCOPED_TRACE(testing::Message() << "input=" << p.input);
        const SinCos sc = det_sincos(p.input);
        // The paired accessor must be bit-identical to the scalar entry points.
        EXPECT_EQ(hex_bits(sc.sin), hex_bits(det_sin(p.input)));
        EXPECT_EQ(hex_bits(sc.cos), hex_bits(det_cos(p.input)));
        // Pythagorean identity holds to a few ulp.
        EXPECT_NEAR(sc.sin * sc.sin + sc.cos * sc.cos, 1.0, 1e-12);
    }
}

} // namespace
