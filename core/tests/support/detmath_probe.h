// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <vector>

namespace scena::testsupport {

/// The canonical detmath probe inputs (radians), single source of truth for
/// both detmath_test's pinned-bit table and the trace runner's `m` rows. Kept
/// here so the test and the cross-platform trace pin the exact same arguments.
[[nodiscard]] const std::vector<double>& detmath_probe_inputs();

/// One (y, x) argument pair for det_atan2.
struct Atan2Input {
    double y = 0.0;
    double x = 0.0;
};

/// The canonical det_atan2 probe pairs, covering all four quadrants, both
/// axes, the signed zeros, the reduction fold points, and quotients that
/// overflow. Shared by detmath_test's pinned-bit table and the trace runner.
[[nodiscard]] const std::vector<Atan2Input>& detmath_atan2_probe_inputs();

} // namespace scena::testsupport
