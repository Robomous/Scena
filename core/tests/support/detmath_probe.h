/*
 * Copyright 2026 Robomous
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
