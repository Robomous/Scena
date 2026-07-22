// SPDX-License-Identifier: MIT
#include "scena/ir/entity_condition.h"

#include <utility>

namespace scena::ir {

ByEntityCondition::ByEntityCondition(TriggeringEntities triggering_entities)
    : triggering_entities_(std::move(triggering_entities)) {}

bool ByEntityCondition::evaluate(const EvaluationContext& context) const {
    const std::vector<std::string>& refs = triggering_entities_.entity_refs;
    // An empty set has no entity to satisfy the expression; validation rejects
    // it at init, so this only guards a hand-built IR.
    if (refs.empty()) {
        return false;
    }
    // Evaluate every entity — no short-circuit — so the outcome never depends
    // on evaluation order and a future per-entity diagnostic stays stable.
    // Then reduce per §7.6.5.1: Any is OR, All is AND.
    bool any = false;
    bool all = true;
    for (const std::string& entity_id : refs) {
        const bool held = evaluate_for_entity(context, entity_id);
        any = any || held;
        all = all && held;
    }
    return triggering_entities_.rule == TriggeringEntitiesRule::All ? all : any;
}

const TriggeringEntities& ByEntityCondition::triggering_entities() const {
    return triggering_entities_;
}

} // namespace scena::ir
