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
