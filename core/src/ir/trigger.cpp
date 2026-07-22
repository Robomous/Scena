// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#include "scena/ir/trigger.h"

#include <utility>

namespace scena::ir {

Trigger make_trigger(std::shared_ptr<Condition> expression, ConditionEdge edge, double delay) {
    TriggerCondition condition;
    condition.delay = delay;
    condition.edge = edge;
    condition.expression = std::move(expression);
    ConditionGroup group;
    group.conditions.push_back(std::move(condition));
    Trigger trigger;
    trigger.groups.push_back(std::move(group));
    return trigger;
}

} // namespace scena::ir
