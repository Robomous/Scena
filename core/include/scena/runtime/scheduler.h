// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "scena/ir/evaluation_context.h"
#include "scena/ir/storyboard.h"

namespace scena::runtime {

/// Monitorable lifecycle states of a storyboard element, per ASAM
/// OpenSCENARIO XML 1.4.0 §8.1 (the implementation-specific initState and
/// endState are not represented).
enum class ElementState {
    Standby,  ///< Waiting for a start trigger.
    Running,  ///< Ongoing, unfinished execution.
    Complete, ///< Reached its goal or was stopped.
};

/// Monitorable transitions of a storyboard element, per §8.2. `None` means
/// no monitorable transition has occurred yet (the element is still in the
/// state its parent put it in).
enum class TransitionKind {
    None,
    Start, ///< Into runningState (§8.2 startTransition).
    End,   ///< Out of runningState, regular end (§8.2 endTransition).
    Stop,  ///< Stopped out of runningState or standbyState (§8.2 stopTransition).
    /// Specific to Event (§8.2 skipTransition). The standard gives it two
    /// distinct meanings and Scena emits it for both, distinguishing them
    /// by the state the element ends up in: an event that could not start
    /// because of its `skip` priority (§8.2, §8.4.2.2), and a standby event
    /// whose executions reached its maximumExecutionCount (§8.4.2.1).
    Skip,
};

/// What applying (or re-polling) one action means for the element state
/// machine (§8.4.1): an event "ends regularly when every nested Action is
/// completed" (§8.4.2), so the applier is what tells the scheduler whether
/// that already happened.
enum class ActionOutcome {
    /// Reached its goal in the evaluation it was applied in. §7.4.1.2: a
    /// LaneChangeAction, SpeedAction or LaneOffsetAction used with the step
    /// dynamic option assigns no control strategy because "the changes are
    /// enacted instantaneously".
    Complete,
    /// Ongoing and unable to end by itself; only a stopTransition ends it
    /// (§7.5.3, never-ending actions). Its event stays in runningState and the
    /// action is not re-polled.
    Ongoing,
    /// In progress and governed by transition dynamics (§7.4.1.2): the action
    /// has not reached its goal yet but will on its own. Its event stays in
    /// runningState and the scheduler re-polls the action each step through the
    /// same fire callback until it reports Complete. Added with p2-s2 (ADR-0005
    /// foresaw this "additional enumerator"; ADR-0011 specifies the drive).
    Running,
};

/// Storyboard executor: walks the Story/Act/ManeuverGroup/Maneuver/Event
/// hierarchy each step and drives the element state machine of §8.1–§8.4.
///
/// Semantics implemented in this phase (all per ASAM OpenSCENARIO XML
/// 1.4.0):
/// - Child start rule (§8.3): when a parent enters runningState, direct
///   children with a start trigger enter standbyState, all others enter
///   runningState immediately.
/// - Events fire their actions on their startTransition; the event ends
///   regularly in that same evaluation when every action reported
///   ActionOutcome::Complete (§8.4.1–8.4.2). An action reporting Running
///   (transition dynamics) is re-polled through the fire callback every step
///   and the event ends regularly once the last such action completes; an
///   action reporting Ongoing (never-ending) keeps the event in runningState
///   until a stopTransition ends it.
/// - Event priority is resolved in the scope of the Maneuver (§7.3.3,
///   §8.4.2.2): `override` stops every *running* sibling of the same
///   Maneuver — standby siblings are untouched — `skip` does not start
///   while a sibling runs, and `parallel` starts regardless. A skipped
///   start performs a skipTransition, which counts as an execution.
/// - Execution counts (§8.3.3.2, §8.4.2.1): an event's executions are the
///   sum of its startTransitions and skipTransitions and are performed
///   sequentially. Leaving runningState with an endTransition returns the
///   event to standbyState while executions remain and completes it once
///   they are exhausted; a standby event whose executions already reached
///   the maximum completes with a skipTransition. A stopTransition — from
///   a stop trigger or from an overriding sibling — completes the event
///   "regardless of the number of executions left" (§8.4.2.2).
/// - Completion propagates child -> parent: a Maneuver completes when all
///   its Events completed (§8.4.3), a ManeuverGroup when all Maneuvers
///   (§8.4.4, an empty group completes instantly), an Act when all groups
///   (§8.4.5), a Story when all Acts (§8.4.6).
/// - The Storyboard never completes on its own; only its stop trigger
///   moves it (and every still-executing descendant) to completeState via
///   stopTransition (§8.4.7). Acts host stop triggers too, and stopping an
///   act stops its whole subtree (§7.6.1.2).
/// - Triggers are OR-of-AND condition groups with edge and delay modifiers
///   (§7.6.1–§7.6.4); each condition carries its own evaluation history,
///   built when the storyboard is bound and reset on rebind.
/// - Stop wins over start within one evaluation, at every level: the
///   storyboard's stop trigger is checked before the walk, and an act
///   checks its own stop trigger before its start trigger. A stop trigger
///   that holds at the same discrete time as a start trigger therefore
///   prevents the start rather than racing it.
/// - Determinism: the walk visits elements in document order; actions of
///   simultaneously started events fire in document order; every condition
///   of every trigger reachable this evaluation is evaluated exactly once,
///   without short-circuiting, so that edge and delay histories advance in
///   lockstep regardless of the trigger's Boolean outcome.
/// - A start trigger is evaluated exactly once per evaluation and only
///   while its element is in standbyState; an element in runningState or
///   completeState has no reachable start trigger, so its condition
///   histories are frozen (§7.3.2: "start triggers only make sense for
///   events in standbyState"). Priority is therefore never expressed by
///   *not* evaluating a trigger — a `skip` event stands by, its trigger is
///   evaluated normally and its histories advance exactly as any other
///   element's; only the transition that follows the evaluation differs.
///   Re-arming does not reset condition history either (only bind() does),
///   so an event with a rising-edge trigger re-executes on the next rise
///   rather than immediately.
/// - The standard gives no ordering rule for two events of one Maneuver
///   whose start triggers hold at the same discrete time. Scena resolves
///   them in a single pass in document order, so every decision is a pure
///   function of the time and of the decisions taken by strictly earlier
///   siblings. Two consequences worth knowing: of two events triggering
///   together where the later one is `override`, the later one wins (it
///   stops the one that just started); and a `skip` event placed before an
///   event that starts in the same evaluation is not skipped, because
///   nothing was running yet when its turn came.
class Scheduler {
public:
    using FireCallback = std::function<ActionOutcome(const ir::Action&)>;

    /// Binds the executor to a storyboard and builds its element tree,
    /// including a fresh evaluation history per trigger condition (any
    /// previous history is discarded). The storyboard must be valid —
    /// Engine::init rejects null logical expressions, negative or NaN
    /// delays, empty condition groups and negative execution counts, and
    /// the scheduler relies on that precondition instead of re-checking it
    /// every step.
    ///
    /// The storyboard enters runningState on the first step() call — simulation
    /// time starts with the execution of the storyboard (§8.4.7), and the
    /// engine issues that first evaluation at t = 0 during init. The
    /// storyboard must outlive the binding.
    void bind(const ir::Storyboard& storyboard);

    /// Evaluates the storyboard against `context`: checks the stop trigger,
    /// starts standby elements whose trigger holds, fires the actions of
    /// started events via `fire`, and propagates completion. No-op before
    /// bind() or after the storyboard completed.
    ///
    /// The scheduler answers storyboard-element-state queries from its own
    /// bound tree: it wraps `context` so that a StoryboardElementStateCondition
    /// sees this storyboard's live states and transitions, while every other
    /// facet (simulation time, named values, time of day) is forwarded to the
    /// host context unchanged.
    void step(const ir::EvaluationContext& context, const FireCallback& fire);

    /// Convenience overload for time-only evaluation: wraps `simulation_time`
    /// in a TimeOnlyEvaluationContext. A condition that reads any other facet
    /// (a named value, time of day) evaluates to false under this overload.
    void step(double simulation_time, const FireCallback& fire);

    /// State of the element addressed by `path`: element names joined with
    /// '/' from the story down (e.g. "story/act/group/maneuver/event");
    /// the empty path addresses the storyboard itself. std::nullopt when
    /// unbound or the path does not name an element.
    [[nodiscard]] std::optional<ElementState> element_state(const std::string& path) const;

    /// Last monitorable transition of the element addressed by `path`
    /// (same addressing as element_state).
    [[nodiscard]] std::optional<TransitionKind> element_transition(const std::string& path) const;

    /// True when the storyboard reached completeState (stop trigger fired).
    [[nodiscard]] bool storyboard_complete() const noexcept;

    /// Unbinds the storyboard and clears all element states.
    void reset() noexcept;

private:
    /// Per-condition evaluation history. The IR stays immutable, so
    /// everything an edge or a delay needs to remember lives here.
    struct ConditionState {
        const ir::TriggerCondition* condition = nullptr;

        /// False until the condition has been evaluated once. §7.6.4: the
        /// first check of a condition defined with an edge always yields
        /// false, which is also what makes a constantly true expression
        /// never produce a rising edge.
        bool has_previous = false;

        /// LogicalExpression(t_{d-1}) — the raw expression value at the
        /// previous evaluation, not the post-edge one.
        bool previous_raw = false;

        /// One post-edge value C(t_d) at the discrete time it was produced.
        struct Sample {
            double time = 0.0;
            bool value = false;
        };

        /// Delay history, populated only when `delay > 0`. Times are
        /// nondecreasing (simulation time never runs backwards within a
        /// binding) and samples older than the one currently selected are
        /// pruned, which bounds the history deterministically.
        std::vector<Sample> history;
    };

    /// Evaluation state of one trigger site. `trigger == nullptr` means the
    /// element hosts no trigger at all, which is different from hosting an
    /// empty one (always false, §7.6.1).
    struct TriggerState {
        const ir::Trigger* trigger = nullptr;
        std::vector<ConditionState> conditions; ///< Document order, flattened.

        [[nodiscard]] bool present() const noexcept { return trigger != nullptr; }
    };

    struct Node {
        enum class Kind { Storyboard, Story, Act, Group, Maneuver, Event };

        Kind kind = Kind::Storyboard;
        std::string name;
        TriggerState start_trigger;       ///< Acts and Events only (§7.6.1.1).
        TriggerState stop_trigger;        ///< Storyboard and Acts only (§7.6.1.2).
        const ir::Event* event = nullptr; ///< Events only.
        ElementState state = ElementState::Standby;
        TransitionKind transition = TransitionKind::None;
        /// The evaluation counter value at which `transition` was last set.
        /// A transition is a one-evaluation pulse: a StoryboardElementState
        /// transition literal holds only when this equals the scheduler's
        /// current evaluation counter, so it stops being observable on the
        /// next step even though `transition` itself is not cleared.
        std::uint64_t transition_evaluation = 0;

        /// Events only (§8.4.2.2).
        ir::EventPriority priority = ir::EventPriority::Parallel;
        /// Events only: the budget of §8.3.3.2 and the running tally of
        /// §8.4.2.1 (startTransitions + skipTransitions).
        int max_executions = 1;
        int executions = 0;

        /// Events only: actions still in progress under transition dynamics
        /// (they reported ActionOutcome::Running at their fire). Re-polled every
        /// step until they complete; document order, populated at
        /// enter_running and drained by progress_event. Borrowed pointers into
        /// the immutable storyboard IR.
        std::vector<const ir::Action*> running_actions;
        /// Events only: at least one action reported Ongoing (never-ending),
        /// which keeps the event in runningState regardless of running_actions
        /// until a stopTransition ends it.
        bool has_ongoing = false;

        std::vector<Node> children;
    };

    /// Read-only context that answers a StoryboardElementStateCondition from
    /// this scheduler's bound tree while forwarding every other facet
    /// (simulation time, named values, time of day) to the host context. It
    /// is constructed per step; the current evaluation counter it captures is
    /// what makes a transition literal a one-evaluation pulse.
    class BoundContext final : public ir::EvaluationContext {
    public:
        BoundContext(const ir::EvaluationContext& host,
                     const std::map<std::string, const Node*>& element_refs,
                     std::uint64_t evaluation)
            : host_(&host), element_refs_(&element_refs), evaluation_(evaluation) {}

        [[nodiscard]] double simulation_time() const override { return host_->simulation_time(); }
        [[nodiscard]] std::optional<std::string_view>
        named_value(ir::NamedValueKind kind, std::string_view name) const override {
            return host_->named_value(kind, name);
        }
        [[nodiscard]] std::optional<double> date_time_seconds() const override {
            return host_->date_time_seconds();
        }
        [[nodiscard]] std::optional<bool>
        storyboard_element_state(ir::StoryboardElementType type, std::string_view ref,
                                 ir::StoryboardElementState state) const override;
        [[nodiscard]] std::optional<ir::EntityKinematics>
        entity_kinematics(std::string_view id) const override {
            return host_->entity_kinematics(id);
        }

    private:
        const ir::EvaluationContext* host_;
        const std::map<std::string, const Node*>* element_refs_;
        std::uint64_t evaluation_;
    };

    static TriggerState make_trigger_state(const std::optional<ir::Trigger>& trigger);
    static bool evaluate_condition(ConditionState& state, const ir::EvaluationContext& context);
    static bool evaluate_trigger(TriggerState& state, const ir::EvaluationContext& context);
    static Node build(const ir::Storyboard& storyboard);

    /// Sets `node.transition` and stamps it with the current evaluation, so
    /// the transition is observable only within the evaluation it occurred.
    void mark_transition(Node& node, TransitionKind transition);

    void enter_running(Node& node, const ir::EvaluationContext& context, const FireCallback& fire);
    void update(Node& node, const ir::EvaluationContext& context, const FireCallback& fire);

    /// Advances the events of a running Maneuver — the scope event priority
    /// is defined over (§7.3.3) and therefore the only place that can see
    /// all the siblings a starting event has to interact with.
    void update_maneuver(Node& maneuver, const ir::EvaluationContext& context,
                         const FireCallback& fire);
    /// Re-polls the transition-dynamics actions of a running event (§8.4.2):
    /// each still-running action is fired again; those reporting Complete drop
    /// out, and once none remain and no never-ending action is present the
    /// event ends regularly with an endTransition.
    void progress_event(Node& event, const FireCallback& fire);
    /// Resolves the priority of the event at `index` and starts it, skips
    /// it or overrides its running siblings accordingly (§8.4.2.2).
    void start_event(Node& maneuver, std::size_t index, const ir::EvaluationContext& context,
                     const FireCallback& fire);
    [[nodiscard]] static bool has_running_sibling(const Node& maneuver, std::size_t index);
    /// Takes an event out of runningState with an endTransition (§8.4.2.1):
    /// back to standbyState while executions remain, complete once they are
    /// exhausted.
    void end_execution(Node& event);
    /// Completes a standby event whose executions already reached its
    /// maximum, with a skipTransition (§8.4.2.1).
    void apply_standby_exhaustion(Node& event);

    void stop_cascade(Node& node);
    static bool all_children_complete(const Node& node);
    [[nodiscard]] const Node* find(const std::string& path) const;

    /// Resolves every StoryboardElementStateCondition reference in the bound
    /// storyboard to a unique element node (or nullptr when zero or several
    /// match), keyed "<type>|<ref>". Built at bind(); read on the hot path.
    void build_element_ref_cache(const ir::Storyboard& storyboard);
    void cache_trigger_refs(const std::optional<ir::Trigger>& trigger);
    [[nodiscard]] const Node* resolve_element_ref(ir::StoryboardElementType type,
                                                  std::string_view ref) const;
    /// Cache key for a StoryboardElementStateCondition reference.
    [[nodiscard]] static std::string element_ref_key(ir::StoryboardElementType type,
                                                     std::string_view ref);

    bool bound_ = false;
    Node root_;
    /// Incremented at the top of every evaluated step; the init evaluation is
    /// 1. Reset by bind()/reset(). Names the discrete time a transition pulse
    /// belongs to.
    std::uint64_t evaluation_ = 0;
    /// Pre-resolved StoryboardElementStateCondition references (see
    /// build_element_ref_cache).
    std::map<std::string, const Node*> element_refs_;
};

} // namespace scena::runtime
