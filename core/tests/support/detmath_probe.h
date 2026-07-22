// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <vector>

namespace scena::testsupport {

/// The canonical detmath probe inputs (radians), single source of truth for
/// both detmath_test's pinned-bit table and the trace runner's `m` rows. Kept
/// here so the test and the cross-platform trace pin the exact same arguments.
[[nodiscard]] const std::vector<double>& detmath_probe_inputs();

} // namespace scena::testsupport
