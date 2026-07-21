// SPDX-License-Identifier: MIT
#include "scena/runtime/scheduler.h"

namespace scena::runtime {

void Scheduler::bind(const ir::Storyboard& storyboard) {
    storyboard_ = &storyboard;
    states_.assign(storyboard.entries.size(), ActionState::Pending);
}

void Scheduler::step(double simulation_time, const FireCallback& fire) {
    if (storyboard_ == nullptr) {
        return;
    }
    for (std::size_t i = 0; i < storyboard_->entries.size(); ++i) {
        if (states_[i] != ActionState::Pending) {
            continue;
        }
        const ir::StoryboardEntry& entry = storyboard_->entries[i];
        if (entry.condition->evaluate(simulation_time)) {
            states_[i] = ActionState::Running;
            if (fire) {
                fire(*entry.action);
            }
            // All actions are instantaneous in this phase, so the entry
            // completes within the same step it fires.
            states_[i] = ActionState::Complete;
        }
    }
}

ActionState Scheduler::action_state(std::size_t entry_index) const {
    return states_.at(entry_index);
}

std::size_t Scheduler::entry_count() const noexcept {
    return states_.size();
}

void Scheduler::reset() noexcept {
    storyboard_ = nullptr;
    states_.clear();
}

} // namespace scena::runtime
