// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "scena/ir/condition.h"

namespace scena::ir {

// The ASAM OpenSCENARIO trigger model (XML 1.4.0 §7.6.1): a Trigger is an
// OR over ConditionGroups, a ConditionGroup an AND over Conditions:
//
//   T(t_d) = OR over n of ( AND over m of Condition_nm(t_d) )
//
// A trigger with no condition groups always evaluates to false (§7.6.1,
// note) — the standard keeps empty triggers only for backward
// compatibility. Absence of a trigger is a different thing entirely and is
// modeled by std::optional in the storyboard hierarchy, not by an empty
// Trigger: an element without a start trigger starts with its parent
// (§7.6.1.1), an element behind an empty one never starts.

/// Edge modifier of a condition, per §7.6.2. The evaluation of a condition
/// depends on the value of its logical expression at the current discrete
/// time t_d and — for every edge but `None` — at the previous evaluation
/// t_{d-1}.
enum class ConditionEdge {
    None,           ///< C_N(t_d) = LE(t_d) (§7.6.2.4).
    Rising,         ///< C_R(t_d) = LE(t_d) AND NOT LE(t_{d-1}) (§7.6.2.1).
    Falling,        ///< C_F(t_d) = NOT LE(t_d) AND LE(t_{d-1}) (§7.6.2.2).
    RisingOrFalling ///< C_RoF(t_d) = C_R(t_d) OR C_F(t_d) (§7.6.2.3).
};

/// The ASAM `Condition` element: a logical expression plus the edge and
/// delay modifiers that turn it into a Boolean the enclosing trigger
/// consumes. `ir::Condition` remains the logical-expression layer beneath
/// it (the condition catalog of §7.6.5 subclasses that one).
///
/// Evaluation pipeline per §7.6.2/§7.6.3, in this order:
/// raw logical expression -> edge detection -> delay. The delay applies to
/// the post-edge value because the class reference for `Condition.delay`
/// states the delay elapses *after* the edge condition is verified; §7.6.3
/// phrases the same rule over the logical expression alone. The class
/// reference wins here — it is the more specific statement.
struct TriggerCondition {
    /// Informational in this phase; the condition catalog uses it for
    /// diagnostics and for storyboard-element condition references.
    std::string name;

    /// Delay in seconds, `>= 0`. A delayed condition evaluates to the value
    /// it had at t_d - delay, and is false while t_d < delay (§7.6.3,
    /// §7.6.4). Validated at Engine::init per rule
    /// asam.net:xosc:1.0.0:data_type.condition_delay_not_negative.
    double delay = 0.0;

    /// Edge modifier (§7.6.2).
    ConditionEdge edge = ConditionEdge::None;

    /// Logical expression; must be non-null (validated at Engine::init).
    std::shared_ptr<Condition> expression;
};

/// AND over its conditions (§7.6.1). Cardinality: conditions 1..* — an
/// empty group would be a vacuously true conjunction, so it is rejected at
/// Engine::init rather than given a meaning the standard does not define.
struct ConditionGroup {
    std::vector<TriggerCondition> conditions;
};

/// OR over its condition groups (§7.6.1). No groups: always false.
struct Trigger {
    std::vector<ConditionGroup> groups;
};

/// Wraps a single logical expression into a one-group, one-condition
/// trigger — the common case, and the shape every trigger had before the
/// full model landed.
[[nodiscard]] Trigger make_trigger(std::shared_ptr<Condition> expression,
                                   ConditionEdge edge = ConditionEdge::None, double delay = 0.0);

} // namespace scena::ir
