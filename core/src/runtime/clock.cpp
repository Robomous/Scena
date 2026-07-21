// SPDX-License-Identifier: MIT
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
