// SPDX-License-Identifier: MIT
#pragma once

#include <cstddef>
#include <functional>
#include <vector>

#include "scena/ir/scenario.h"

namespace scena::runtime {

/// Execution state of one storyboard entry's action.
enum class ActionState {
    Pending,  ///< Condition has not held yet; action has not fired.
    Running,  ///< Action is executing. Instantaneous actions pass through this
              ///< state within a single step.
    Complete, ///< Action has finished; it never fires again.
};

/// Skeleton of the storyboard execution state machine.
///
/// Each step, the scheduler evaluates the condition of every Pending entry at
/// the current simulation time. When a condition first holds, the entry moves
/// Pending -> Running, its action is handed to the fire callback exactly once,
/// and — because all actions in this phase are instantaneous — the entry moves
/// Running -> Complete within the same step. Entries are independent: one
/// entry's state never affects another's evaluation.
class Scheduler {
public:
    using FireCallback = std::function<void(const ir::Action&)>;

    /// Binds the scheduler to a storyboard and resets all entries to Pending.
    /// The storyboard must outlive the scheduler binding.
    void bind(const ir::Storyboard& storyboard);

    /// Evaluates pending entries at `simulation_time`, invoking `fire` once
    /// for each action whose condition first holds. No-op before bind().
    void step(double simulation_time, const FireCallback& fire);

    /// State of the entry at `entry_index` (storyboard order).
    [[nodiscard]] ActionState action_state(std::size_t entry_index) const;

    /// Number of bound storyboard entries.
    [[nodiscard]] std::size_t entry_count() const noexcept;

    /// Unbinds the storyboard and clears all entry states.
    void reset() noexcept;

private:
    const ir::Storyboard* storyboard_ = nullptr;
    std::vector<ActionState> states_;
};

} // namespace scena::runtime
