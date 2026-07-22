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

} // namespace scena::testsupport
