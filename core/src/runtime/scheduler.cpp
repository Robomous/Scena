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

#include "scena/runtime/scheduler.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "scena/ir/condition.h"
#include "scena/runtime/element_ref.h"

namespace scena::runtime {

namespace {

/// Whether an element in `state`/`transition` (stamped at `stamp`) satisfies
/// the queried StoryboardElementState. Level states hold whenever the element
/// is in that state; transitions hold only in the evaluation they occurred
/// (stamp == current evaluation), which is what makes a transition a
/// one-evaluation pulse.
bool element_state_holds(ElementState state, TransitionKind transition, std::uint64_t stamp,
                         std::uint64_t evaluation, ir::StoryboardElementState query) {
    switch (query) {
    case ir::StoryboardElementState::StandbyState:
        return state == ElementState::Standby;
    case ir::StoryboardElementState::RunningState:
        return state == ElementState::Running;
    case ir::StoryboardElementState::CompleteState:
        return state == ElementState::Complete;
    case ir::StoryboardElementState::StartTransition:
        return transition == TransitionKind::Start && stamp == evaluation;
    case ir::StoryboardElementState::EndTransition:
        return transition == TransitionKind::End && stamp == evaluation;
    case ir::StoryboardElementState::StopTransition:
        return transition == TransitionKind::Stop && stamp == evaluation;
    case ir::StoryboardElementState::SkipTransition:
        return transition == TransitionKind::Skip && stamp == evaluation;
    }
    return false;
}

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

bool Scheduler::evaluate_condition(ConditionState& state, const ir::EvaluationContext& context) {
    // Engine::init guarantees a non-null expression and a delay >= 0.
    const ir::TriggerCondition& condition = *state.condition;
    const bool raw = condition.expression->evaluate(context);
    // The edge and delay machinery is keyed on the discrete evaluation time,
    // which the scheduler reads straight from the context.
    const double simulation_time = context.simulation_time();

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

bool Scheduler::evaluate_trigger(TriggerState& state, const ir::EvaluationContext& context) {
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
            group_value = evaluate_condition(state.conditions[index], context) &&
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
                        event_node.priority = event.priority;
                        event_node.max_executions = event.maximum_execution_count;
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
    evaluation_ = 0;
    element_refs_.clear();
    build_element_ref_cache(storyboard);
    bound_ = true;
}

void Scheduler::step(double simulation_time, const FireCallback& fire) {
    const ir::TimeOnlyEvaluationContext context{simulation_time};
    step(context, fire);
}

void Scheduler::step(const ir::EvaluationContext& host_context, const FireCallback& fire) {
    if (!bound_ || root_.state == ElementState::Complete) {
        return;
    }

    // A new discrete evaluation: bump the counter so transitions produced in
    // this step form a one-evaluation pulse. The init evaluation is 1.
    ++evaluation_;

    // Wrap the host context so a StoryboardElementStateCondition sees this
    // storyboard's live states and transitions; every other facet forwards to
    // the host unchanged.
    const BoundContext context{host_context, element_refs_, evaluation_};

    // The storyboard enters runningState when the simulation starts; the
    // engine issues this first evaluation at t = 0 during init (§8.4.7).
    if (root_.state == ElementState::Standby) {
        enter_running(root_, context, fire);
    }

    // Stop trigger wins over any same-step starts: it moves the storyboard
    // and every still-executing descendant to completeState (§8.4.7). The
    // same stop-before-start ordering applies to act stop triggers in
    // update().
    if (evaluate_trigger(root_.stop_trigger, context)) {
        stop_cascade(root_);
        return;
    }

    update(root_, context, fire);
}

void Scheduler::mark_transition(Node& node, TransitionKind transition) {
    node.transition = transition;
    node.transition_evaluation = evaluation_;
}

void Scheduler::enter_running(Node& node, const ir::EvaluationContext& context,
                              const FireCallback& fire) {
    node.state = ElementState::Running;
    mark_transition(node, TransitionKind::Start);

    if (node.kind == Node::Kind::Event) {
        // This startTransition is one of the event's executions (§8.4.2.1).
        ++node.executions;

        // Actions enter runningState with their parent event (§8.4.1). The
        // event "ends regularly when every nested Action is completed"
        // (§8.4.2), so it stays in runningState as soon as one action is
        // still ongoing (never-ending) or running (transition dynamics). Every
        // action is fired regardless — the outcome of an earlier one must not
        // decide whether a later one is applied. Actions reporting Running are
        // recorded so progress_event can re-poll them each step in document
        // order.
        node.running_actions.clear();
        node.has_ongoing = false;
        for (const std::shared_ptr<ir::Action>& action : node.event->actions) {
            if (fire && action != nullptr) {
                switch (fire(*action)) {
                case ActionOutcome::Complete:
                    break;
                case ActionOutcome::Ongoing:
                    node.has_ongoing = true;
                    break;
                case ActionOutcome::Running:
                    node.running_actions.push_back(action.get());
                    break;
                }
            }
        }
        if (node.running_actions.empty() && !node.has_ongoing) {
            end_execution(node);
        }
        return;
    }

    // Child start rule (§8.3): children with a start trigger wait in
    // standbyState; all others enter runningState immediately.
    if (node.kind == Node::Kind::Maneuver) {
        // Events go through the priority resolution of §8.4.2.2 even when
        // they start with their parent: such an event inherits the Act's
        // start trigger (§8.4.2), so it is a triggered event like any other
        // and can equally override or be skipped.
        for (std::size_t i = 0; i < node.children.size(); ++i) {
            if (!node.children[i].start_trigger.present()) {
                start_event(node, i, context, fire);
            } else {
                // An event that is about to wait for its own trigger with no
                // executions left never gets to run (§8.4.2.1).
                apply_standby_exhaustion(node.children[i]);
            }
        }
    } else {
        for (Node& child : node.children) {
            if (!child.start_trigger.present()) {
                enter_running(child, context, fire);
            }
        }
    }

    // Instantaneous children may already have completed; an element with no
    // executing children ends regularly (e.g. an empty ManeuverGroup
    // completes instantly, §8.4.4). The storyboard itself never completes
    // this way (§8.4.7).
    if (node.kind != Node::Kind::Storyboard && all_children_complete(node)) {
        node.state = ElementState::Complete;
        mark_transition(node, TransitionKind::End);
    }
}

bool Scheduler::has_running_sibling(const Node& maneuver, std::size_t index) {
    for (std::size_t i = 0; i < maneuver.children.size(); ++i) {
        if (i != index && maneuver.children[i].state == ElementState::Running) {
            return true;
        }
    }
    return false;
}

void Scheduler::end_execution(Node& event) {
    // §8.4.2.1: when the event is about to transfer out of runningState with
    // an endTransition it completes if its executions reached the maximum,
    // and returns to standbyState while executions remain.
    mark_transition(event, TransitionKind::End);
    event.state =
        event.executions >= event.max_executions ? ElementState::Complete : ElementState::Standby;
}

void Scheduler::apply_standby_exhaustion(Node& event) {
    // §8.4.2.1: "If in standbyState and the number of executions is equal to
    // the number stated by maximumExecutionCount, the Event transfers to
    // completeState with skipTransition." This is also what makes a
    // maximumExecutionCount of zero mean "never executes": such an event is
    // exhausted before it ever starts.
    if (event.state == ElementState::Standby && event.executions >= event.max_executions) {
        event.state = ElementState::Complete;
        mark_transition(event, TransitionKind::Skip);
    }
}

void Scheduler::start_event(Node& maneuver, std::size_t index, const ir::EvaluationContext& context,
                            const FireCallback& fire) {
    Node& event = maneuver.children[index];
    apply_standby_exhaustion(event);
    if (event.state != ElementState::Standby) {
        return;
    }

    switch (event.priority) {
    case ir::EventPriority::Override:
        // §8.4.2.2: "A triggered Event with priority override terminates any
        // running Event in the same scope (Maneuver), when it moves to
        // runningState. A terminated Event moves to completeState with
        // stopTransition, regardless of the number of executions left."
        // Standby siblings are deliberately left alone — both §8.4.2.2 and
        // the Priority class reference say "running". stop_cascade is the
        // same code path a stop trigger takes, so an overridden event and a
        // stopped one are indistinguishable, as §8.4.2 requires.
        for (std::size_t i = 0; i < maneuver.children.size(); ++i) {
            if (i != index && maneuver.children[i].state == ElementState::Running) {
                stop_cascade(maneuver.children[i]);
            }
        }
        break;
    case ir::EventPriority::Skip:
        // Priority class reference: "If a starting event has priority Skip,
        // then it will not be ran if there is any other event in the same
        // scope (maneuver) in the running state." The skipped start is a
        // skipTransition and counts as an execution (§8.4.2.1), which may
        // exhaust the event outright.
        if (has_running_sibling(maneuver, index)) {
            mark_transition(event, TransitionKind::Skip);
            ++event.executions;
            apply_standby_exhaustion(event);
            return;
        }
        break;
    case ir::EventPriority::Parallel:
        // "moves to the runningState regardless of the states of other Event
        // instances in the same scope" (§8.4.2.2).
        break;
    }

    enter_running(event, context, fire);
}

void Scheduler::update_maneuver(Node& maneuver, const ir::EvaluationContext& context,
                                const FireCallback& fire) {
    // Single pass in document order. Each decision is a pure function of the
    // time and of the decisions already taken by strictly earlier siblings,
    // which is what pins the outcome when several events trigger together —
    // the standard gives no ordering rule for that case.
    for (std::size_t i = 0; i < maneuver.children.size(); ++i) {
        Node& event = maneuver.children[i];
        // A running event has no reachable start trigger (§7.3.2), but its
        // transition-dynamics actions are re-polled here so it can end
        // regularly when they finish (§8.4.2). An event re-armed to standby by
        // this same pass keeps its turn to next step — its state was Running
        // when its turn came.
        if (event.state == ElementState::Running) {
            // Re-poll transition-dynamics actions, but not on the same
            // evaluation the event fired them: an action advances starting the
            // step after its startTransition, so its goal is not reached in the
            // evaluation it was applied in (that is the Complete case).
            const bool started_this_step = event.transition == TransitionKind::Start &&
                                           event.transition_evaluation == evaluation_;
            if (!started_this_step) {
                progress_event(event, fire);
            }
            continue;
        }
        // Complete events, and events not yet triggered, are left alone; a
        // standby event never has two simultaneous instantiations.
        if (event.state != ElementState::Standby) {
            continue;
        }
        if (!evaluate_trigger(event.start_trigger, context)) {
            continue;
        }
        start_event(maneuver, i, context, fire);
    }
}

void Scheduler::progress_event(Node& event, const FireCallback& fire) {
    if (event.running_actions.empty()) {
        // Nothing to re-poll: either the event has only never-ending actions
        // (has_ongoing) that a stopTransition must end, or none at all.
        return;
    }
    // Re-poll each still-running action in document order; keep those that
    // remain Running. An action that has completed drops out.
    std::vector<const ir::Action*> still_running;
    still_running.reserve(event.running_actions.size());
    for (const ir::Action* action : event.running_actions) {
        if (fire && action != nullptr && fire(*action) == ActionOutcome::Running) {
            still_running.push_back(action);
        }
    }
    event.running_actions = std::move(still_running);

    // The event ends regularly once every nested action completed (§8.4.2). A
    // never-ending action keeps it running regardless.
    if (event.running_actions.empty() && !event.has_ongoing) {
        end_execution(event);
    }
}

void Scheduler::update(Node& node, const ir::EvaluationContext& context, const FireCallback& fire) {
    if (node.state == ElementState::Complete) {
        return;
    }

    // Events are advanced only by their Maneuver, which is the scope event
    // priority is defined over (§7.3.3). Recursing into an event here would
    // also complete it immediately, since it has no children and the regular
    // end below would find them all vacuously complete.
    if (node.kind == Node::Kind::Event) {
        return;
    }

    // Stop triggers are checked in standbyState and in runningState, and
    // before the start trigger of the same element: §7.6.1.2 lets a stop
    // trigger take an element out of either state, and the storyboard-level
    // ordering in step() would otherwise not hold one level down. An act
    // whose stop and start triggers both hold at the same discrete time is
    // therefore stopped, not started.
    if (evaluate_trigger(node.stop_trigger, context)) {
        stop_cascade(node);
        return;
    }

    if (node.state == ElementState::Standby) {
        if (!evaluate_trigger(node.start_trigger, context)) {
            return;
        }
        enter_running(node, context, fire);
        if (node.state == ElementState::Complete) {
            return;
        }
    }

    // Advance children in document order, then check for a regular end
    // (§8.4.3–8.4.6).
    if (node.kind == Node::Kind::Maneuver) {
        update_maneuver(node, context, fire);
    } else {
        for (Node& child : node.children) {
            update(child, context, fire);
        }
    }
    if (node.kind != Node::Kind::Storyboard && all_children_complete(node)) {
        node.state = ElementState::Complete;
        mark_transition(node, TransitionKind::End);
    }
}

void Scheduler::stop_cascade(Node& node) {
    // §7.6.1.2: every descendant inherits the stop trigger, even one that
    // hosts its own — hence the unconditional recursion.
    //
    // A stopped event completes "regardless of the number of executions
    // left" (§8.4.2.2), so the remaining budget is cleared outside the guard
    // below: an event that already completed regularly this evaluation must
    // not be left with executions it could still spend.
    if (node.kind == Node::Kind::Event) {
        node.executions = node.max_executions;
    }
    if (node.state != ElementState::Complete) {
        // stopTransition leaves runningState or standbyState (§8.2).
        node.state = ElementState::Complete;
        mark_transition(node, TransitionKind::Stop);
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
    evaluation_ = 0;
    element_refs_.clear();
}

std::optional<bool> Scheduler::BoundContext::storyboard_element_state(
    ir::StoryboardElementType type, std::string_view ref, ir::StoryboardElementState state) const {
    const auto it = element_refs_->find(element_ref_key(type, ref));
    if (it == element_refs_->end() || it->second == nullptr) {
        // The reference was not present at bind() (bare-scheduler tests) or did
        // not resolve to a unique element; the condition is deterministically
        // false. Engine::init reports the unresolved reference as an error.
        return std::nullopt;
    }
    const Node& node = *it->second;
    return element_state_holds(node.state, node.transition, node.transition_evaluation, evaluation_,
                               state);
}

std::string Scheduler::element_ref_key(ir::StoryboardElementType type, std::string_view ref) {
    // The type index disambiguates a name shared across element kinds; "|" is
    // not a valid name character in a reference (references split on "::").
    return std::to_string(static_cast<int>(type)) + "|" + std::string(ref);
}

void Scheduler::cache_trigger_refs(const std::optional<ir::Trigger>& trigger) {
    if (!trigger.has_value()) {
        return;
    }
    for (const ir::ConditionGroup& group : trigger->groups) {
        for (const ir::TriggerCondition& condition : group.conditions) {
            const auto* element_condition =
                dynamic_cast<const ir::StoryboardElementStateCondition*>(
                    condition.expression.get());
            if (element_condition == nullptr) {
                continue;
            }
            const std::string key = element_ref_key(element_condition->element_type(),
                                                    element_condition->element_ref());
            if (element_refs_.count(key) != 0) {
                continue; // same reference already resolved
            }
            element_refs_[key] = resolve_element_ref(element_condition->element_type(),
                                                     element_condition->element_ref());
        }
    }
}

void Scheduler::build_element_ref_cache(const ir::Storyboard& storyboard) {
    cache_trigger_refs(storyboard.stop_trigger);
    for (const ir::Story& story : storyboard.stories) {
        for (const ir::Act& act : story.acts) {
            cache_trigger_refs(act.start_trigger);
            cache_trigger_refs(act.stop_trigger);
            for (const ir::ManeuverGroup& group : act.groups) {
                for (const ir::Maneuver& maneuver : group.maneuvers) {
                    for (const ir::Event& event : maneuver.events) {
                        cache_trigger_refs(event.start_trigger);
                    }
                }
            }
        }
    }
}

const Scheduler::Node* Scheduler::resolve_element_ref(ir::StoryboardElementType type,
                                                      std::string_view ref) const {
    // Per-action nodes do not exist yet (p5-s4), so an action reference never
    // resolves; the engine warns about it at init.
    if (type == ir::StoryboardElementType::Action) {
        return nullptr;
    }
    Node::Kind kind = Node::Kind::Story;
    switch (type) {
    case ir::StoryboardElementType::Story:
        kind = Node::Kind::Story;
        break;
    case ir::StoryboardElementType::Act:
        kind = Node::Kind::Act;
        break;
    case ir::StoryboardElementType::ManeuverGroup:
        kind = Node::Kind::Group;
        break;
    case ir::StoryboardElementType::Maneuver:
        kind = Node::Kind::Maneuver;
        break;
    case ir::StoryboardElementType::Event:
        kind = Node::Kind::Event;
        break;
    case ir::StoryboardElementType::Action:
        return nullptr;
    }

    const std::vector<std::string> segments = split_element_ref(ref);
    const Node* match = nullptr;
    int matches = 0;
    // Depth-first walk carrying the root-to-here name path; the self-to-root
    // chain each candidate is matched against is its reverse.
    std::vector<std::string> path;
    const auto visit = [&](const Node& node, auto&& self) -> void {
        path.push_back(node.name);
        if (node.kind == kind) {
            std::vector<std::string> chain(path.rbegin(), path.rend());
            if (element_ref_matches(segments, chain)) {
                ++matches;
                match = &node;
            }
        }
        for (const Node& child : node.children) {
            self(child, self);
        }
        path.pop_back();
    };
    visit(root_, visit);
    // A reference resolves only when exactly one element matches (§ naming).
    return matches == 1 ? match : nullptr;
}

} // namespace scena::runtime
