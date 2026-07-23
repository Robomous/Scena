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

namespace scena::runtime {

/// Fixed-step deterministic simulation clock.
///
/// Pure accumulation of the step sizes handed to advance(): no wall-clock
/// access, no drift correction, no hidden state. Two clocks fed the same
/// sequence of steps hold bit-identical times — a cornerstone of the engine's
/// determinism guarantee. The host simulator owns the real clock; this class
/// only mirrors the simulated time it dictates.
class Clock {
public:
    /// Advances simulated time by dt seconds.
    void advance(double dt) noexcept;

    /// Current simulated time in seconds since construction or reset().
    [[nodiscard]] double now() const noexcept;

    /// Returns the clock to t = 0.
    void reset() noexcept;

private:
    double time_ = 0.0;
};

} // namespace scena::runtime
