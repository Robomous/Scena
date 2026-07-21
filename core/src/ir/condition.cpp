// SPDX-License-Identifier: MIT
#include "kinema/ir/condition.h"

namespace kinema::ir {

SimulationTimeCondition::SimulationTimeCondition(double at_time) : at_time_(at_time) {}

bool SimulationTimeCondition::evaluate(double simulation_time) const {
    return simulation_time >= at_time_;
}

double SimulationTimeCondition::at_time() const {
    return at_time_;
}

} // namespace kinema::ir
