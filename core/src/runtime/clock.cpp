// SPDX-License-Identifier: MIT
#include "kinema/runtime/clock.h"

namespace kinema::runtime {

void Clock::advance(double dt) noexcept {
    time_ += dt;
}

double Clock::now() const noexcept {
    return time_;
}

void Clock::reset() noexcept {
    time_ = 0.0;
}

} // namespace kinema::runtime
