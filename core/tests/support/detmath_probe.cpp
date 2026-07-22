// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#include "support/detmath_probe.h"

namespace scena::testsupport {

const std::vector<double>& detmath_probe_inputs() {
    // Spans tiny magnitudes, each quadrant near +/-pi/4/2/pi/2pi, signed pairs,
    // and the domain boundary +/-1e6. Hex-float literals so the inputs
    // themselves are exact and platform-independent.
    static const std::vector<double> kInputs = {
        0x1.0p-30,
        -0x1.0p-30,
        0.3,
        0.5,
        -0.5,
        1.0,
        -1.0,
        0x1.921fb54442d18p-1, // ~pi/4
        0x1.921fb54442d18p+0, // ~pi/2
        0x1.921fb54442d18p+1, // ~pi
        0x1.921fb54442d18p+2, // ~2pi
        2.5,
        -7.7,
        10.0,
        100.0,
        12345.6789,
        1.0e6,
        -1.0e6,
    };
    return kInputs;
}

const std::vector<Atan2Input>& detmath_atan2_probe_inputs() {
    // Quadrant diagonals, both axes with both zero signs, the two reduction
    // fold points (|y/x| = 1 and |y/x| = tan(pi/12)), a quotient that
    // overflows to infinity, and a few off-axis pairs.
    static const std::vector<Atan2Input> kInputs = {
        {1.0, 1.0},    {1.0, -1.0},   {-1.0, 1.0},         {-1.0, -1.0},    {0.0, 1.0},
        {-0.0, 1.0},   {0.0, -1.0},   {-0.0, -1.0},        {1.0, 0.0},      {-1.0, 0.0},
        {0.0, 0.0},    {-0.0, -0.0},  {0.5, 2.0},          {2.0, 0.5},      {-0.5, 2.0},
        {0.5, -2.0},   {3.0, 4.0},    {-7.7, 3.1},         {0.267949, 1.0}, {1.0, 0.267949},
        {1.0e-8, 1.0}, {1.0, 1.0e-8}, {1.0e300, 1.0e-300}, {12.5, -37.25},  {-0.125, -0.03125},
    };
    return kInputs;
}

} // namespace scena::testsupport
