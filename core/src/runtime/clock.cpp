// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#include "scena/runtime/clock.h"

namespace scena::runtime {

void Clock::advance(double dt) noexcept {
    time_ += dt;
}

double Clock::now() const noexcept {
    return time_;
}

void Clock::reset() noexcept {
    time_ = 0.0;
}

} // namespace scena::runtime
