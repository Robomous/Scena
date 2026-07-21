// SPDX-License-Identifier: MIT
#include "scena/runtime/scheduler.h"

#include <algorithm>
#include <cstddef>
#include <utility>

namespace scena::runtime {

namespace {

/// Splits a '/'-separated element path into name segments.
std::vector<std::string> split_path(const std::string& path) {
    std::vector<std::string> segments;
    std::string::size_type begin = 0;
    while (begin <= path.size()) {
        const auto end = path.find('/', begin);
        if (end == std::string::npos) {
            segments.push_back(path.substr(begin));
            break;
        }
        segments.push_back(path.substr(begin, end - begin));
        begin = end + 1;
    }
    return segments;
}

} // namespace

Scheduler::TriggerState Scheduler::make_trigger_state(const std::optional<ir::Trigger>& trigger) {
    TriggerState state;
    if (!trigger.has_value()) {
        return state; // absent — not the same as an empty trigger (§7.6.1)
    }
    state.trigger = &trigger.value();
    for (const ir::ConditionGroup& group : state.trigger->groups) {
        for (const ir::TriggerCondition& condition : group.conditions) {
            ConditionState condition_state;
            condition_state.condition = &condition;
            state.conditions.push_back(std::move(condition_state));
        }
    }
    return state;
}

bool Scheduler::evaluate_condition(ConditionState& state, double simulation_time) {
    // Engine::init guarantees a non-null expression and a delay >= 0.
    const ir::TriggerCondition& condition = *state.condition;
    const bool raw = condition.expression->evaluate(simulation_time);

    // Edge detection over the raw logical expression (§7.6.2). Every edge
    // needs LE(t_{d-1}), which does not exist on the first check — §7.6.4
    // makes that check false, which is also why a constantly true
    // expression never produces a rising edge.
    bool value = false;
    switch (condition.edge) {
    case ir::ConditionEdge::None:
        value = raw; // C_N(t_d) = LE(t_d)
        break;
    case ir::ConditionEdge::Rising:
        value = state.has_previous && raw && !state.previous_raw;
        break;
    case ir::ConditionEdge::Falling:
        value = state.has_previous && !raw && state.previous_raw;
        break;
    case ir::ConditionEdge::RisingOrFalling:
        value = state.has_previous && raw != state.previous_raw;
        break;
    }
    state.previous_raw = raw;
    state.has_previous = true;

    if (!(condition.delay > 0.0)) {
        return value;
    }

    // Delay (§7.6.3): C_D(t_d) = C(t_d - delta). The engine is driven by
    // discrete evaluations at host-chosen times, so there is no value at
    // exactly t_d - delta in general; the history is sampled and held —
    // the most recent evaluation at or before t_d - delta is the answer.
    // Comparisons are exact by design: an epsilon would make the result
    // depend on the magnitude of the times and break the bit-identical
    // reproducibility contract.
    state.history.push_back(ConditionState::Sample{simulation_time, value});
    const double target = simulation_time - condition.delay;
    const auto after = std::upper_bound(
        state.history.begin(), state.history.end(), target,
        [](double time, const ConditionState::Sample& sample) { return time < sample.time; });
    if (after == state.history.begin()) {
        // No evaluation at or before t_d - delta: false while t_d < delta
        // (§7.6.4), and equally when the binding simply started later.
        return false;
    }
    // Step back to the most recent sample at or before the target. Several
    // samples may share a time when the host steps with dt = 0; each is a
    // new discrete evaluation t_d, so the latest one wins — which is what
    // upper_bound selects.
    const auto selected = after - 1;
    const bool delayed = selected->value;
    // Older samples can never be selected again (target only grows), so
    // dropping them keeps the history bounded without affecting results.
    state.history.erase(state.history.begin(), selected);
    return delayed;
}

bool Scheduler::evaluate_trigger(TriggerState& state, double simulation_time) {
    if (!state.present()) {
        return false;
    }
    // T(t_d) = OR over condition groups of (AND over their conditions), and
    // an empty trigger is always false (§7.6.1). Deliberately without
    // short-circuiting: every condition owns edge and delay history that
    // must advance on every evaluation, so skipping one would make the
    // outcome depend on the order conditions happen to be written in.
    bool trigger_value = false;
    std::size_t index = 0;
    for (const ir::ConditionGroup& group : state.trigger->groups) {
        bool group_value = true;
        for (std::size_t i = 0; i < group.conditions.size(); ++i, ++index) {
            group_value = evaluate_condition(state.conditions[index], simulation_time) &&
                          group_value; // condition first: it must always run
        }
        trigger_value = group_value || trigger_value;
    }
    return trigger_value;
}

Scheduler::Node Scheduler::build(const ir::Storyboard& storyboard) {
    Node root;
    root.kind = Node::Kind::Storyboard;
    root.stop_trigger = make_trigger_state(storyboard.stop_trigger);
    for (const ir::Story& story : storyboard.stories) {
        Node story_node;
        story_node.kind = Node::Kind::Story;
        story_node.name = story.name;
        for (const ir::Act& act : story.acts) {
            Node act_node;
            act_node.kind = Node::Kind::Act;
            act_node.name = act.name;
            act_node.start_trigger = make_trigger_state(act.start_trigger);
            act_node.stop_trigger = make_trigger_state(act.stop_trigger);
            for (const ir::ManeuverGroup& group : act.groups) {
                Node group_node;
                group_node.kind = Node::Kind::Group;
                group_node.name = group.name;
                for (const ir::Maneuver& maneuver : group.maneuvers) {
                    Node maneuver_node;
                    maneuver_node.kind = Node::Kind::Maneuver;
                    maneuver_node.name = maneuver.name;
                    for (const ir::Event& event : maneuver.events) {
                        Node event_node;
                        event_node.kind = Node::Kind::Event;
                        event_node.name = event.name;
                        event_node.start_trigger = make_trigger_state(event.start_trigger);
                        event_node.event = &event;
                        maneuver_node.children.push_back(std::move(event_node));
                    }
                    group_node.children.push_back(std::move(maneuver_node));
                }
                act_node.children.push_back(std::move(group_node));
            }
            story_node.children.push_back(std::move(act_node));
        }
        root.children.push_back(std::move(story_node));
    }
    return root;
}

void Scheduler::bind(const ir::Storyboard& storyboard) {
    root_ = build(storyboard);
    bound_ = true;
}

void Scheduler::step(double simulation_time, const FireCallback& fire) {
    if (!bound_ || root_.state == ElementState::Complete) {
        return;
    }

    // The storyboard enters runningState when the simulation starts; the
    // engine issues this first evaluation at t = 0 during init (§8.4.7).
    if (root_.state == ElementState::Standby) {
        enter_running(root_, simulation_time, fire);
    }

    // Stop trigger wins over any same-step starts: it moves the storyboard
    // and every still-executing descendant to completeState (§8.4.7). The
    // same stop-before-start ordering applies to act stop triggers in
    // update().
    if (evaluate_trigger(root_.stop_trigger, simulation_time)) {
        stop_cascade(root_);
        return;
    }

    update(root_, simulation_time, fire);
}

void Scheduler::enter_running(Node& node, double simulation_time, const FireCallback& fire) {
    node.state = ElementState::Running;
    node.transition = TransitionKind::Start;

    if (node.kind == Node::Kind::Event) {
        // Actions enter runningState with their parent event (§8.4.1). All
        // actions are instantaneous in this phase, so the event ends
        // regularly within the same evaluation (§8.4.2).
        for (const std::shared_ptr<ir::Action>& action : node.event->actions) {
            if (fire && action != nullptr) {
                fire(*action);
            }
        }
        node.state = ElementState::Complete;
        node.transition = TransitionKind::End;
        return;
    }

    // Child start rule (§8.3): children with a start trigger wait in
    // standbyState; all others enter runningState immediately.
    for (Node& child : node.children) {
        if (!child.start_trigger.present()) {
            enter_running(child, simulation_time, fire);
        }
    }

    // Instantaneous children may already have completed; an element with no
    // executing children ends regularly (e.g. an empty ManeuverGroup
    // completes instantly, §8.4.4). The storyboard itself never completes
    // this way (§8.4.7).
    if (node.kind != Node::Kind::Storyboard && all_children_complete(node)) {
        node.state = ElementState::Complete;
        node.transition = TransitionKind::End;
    }
}

void Scheduler::update(Node& node, double simulation_time, const FireCallback& fire) {
    if (node.state == ElementState::Complete) {
        return;
    }

    // Stop triggers are checked in standbyState and in runningState, and
    // before the start trigger of the same element: §7.6.1.2 lets a stop
    // trigger take an element out of either state, and the storyboard-level
    // ordering in step() would otherwise not hold one level down. An act
    // whose stop and start triggers both hold at the same discrete time is
    // therefore stopped, not started.
    if (evaluate_trigger(node.stop_trigger, simulation_time)) {
        stop_cascade(node);
        return;
    }

    if (node.state == ElementState::Standby) {
        if (!evaluate_trigger(node.start_trigger, simulation_time)) {
            return;
        }
        enter_running(node, simulation_time, fire);
        if (node.state == ElementState::Complete) {
            return;
        }
    }

    // Running, non-event element (running events never persist across
    // evaluations in this phase): advance children in document order, then
    // check for a regular end (§8.4.3–8.4.6).
    for (Node& child : node.children) {
        update(child, simulation_time, fire);
    }
    if (node.kind != Node::Kind::Storyboard && all_children_complete(node)) {
        node.state = ElementState::Complete;
        node.transition = TransitionKind::End;
    }
}

void Scheduler::stop_cascade(Node& node) {
    // §7.6.1.2: every descendant inherits the stop trigger, even one that
    // hosts its own — hence the unconditional recursion. The same rule
    // requires clearing the remaining number of executions of the stopped
    // elements; execution counts arrive with the event lifecycle sprint and
    // this is the place that has to zero them.
    if (node.state != ElementState::Complete) {
        // stopTransition leaves runningState or standbyState (§8.2).
        node.state = ElementState::Complete;
        node.transition = TransitionKind::Stop;
    }
    for (Node& child : node.children) {
        stop_cascade(child);
    }
}

bool Scheduler::all_children_complete(const Node& node) {
    for (const Node& child : node.children) {
        if (child.state != ElementState::Complete) {
            return false;
        }
    }
    return true;
}

const Scheduler::Node* Scheduler::find(const std::string& path) const {
    if (!bound_) {
        return nullptr;
    }
    if (path.empty()) {
        return &root_;
    }
    const Node* node = &root_;
    for (const std::string& segment : split_path(path)) {
        const Node* next = nullptr;
        for (const Node& child : node->children) {
            if (child.name == segment) {
                next = &child;
                break;
            }
        }
        if (next == nullptr) {
            return nullptr;
        }
        node = next;
    }
    return node;
}

std::optional<ElementState> Scheduler::element_state(const std::string& path) const {
    const Node* node = find(path);
    if (node == nullptr) {
        return std::nullopt;
    }
    return node->state;
}

std::optional<TransitionKind> Scheduler::element_transition(const std::string& path) const {
    const Node* node = find(path);
    if (node == nullptr) {
        return std::nullopt;
    }
    return node->transition;
}

bool Scheduler::storyboard_complete() const noexcept {
    return bound_ && root_.state == ElementState::Complete;
}

void Scheduler::reset() noexcept {
    bound_ = false;
    root_ = Node{};
}

} // namespace scena::runtime
