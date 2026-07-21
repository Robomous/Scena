// SPDX-License-Identifier: MIT
#pragma once

#include <functional>
#include <optional>
#include <string>
#include <vector>

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
/// state its parent put it in). `Skip` arrives with event priorities.
enum class TransitionKind {
    None,
    Start, ///< Into runningState (§8.2 startTransition).
    End,   ///< Out of runningState, regular end (§8.2 endTransition).
    Stop,  ///< Stopped out of runningState or standbyState (§8.2 stopTransition).
};

/// Storyboard executor: walks the Story/Act/ManeuverGroup/Maneuver/Event
/// hierarchy each step and drives the element state machine of §8.1–§8.4.
///
/// Semantics implemented in this phase (all per ASAM OpenSCENARIO XML
/// 1.4.0):
/// - Child start rule (§8.3): when a parent enters runningState, direct
///   children with a start trigger enter standbyState, all others enter
///   runningState immediately.
/// - Events fire their actions on their startTransition; every action is
///   instantaneous in this phase, so events complete within the same
///   evaluation (§8.4.1–8.4.2, single execution — maximumExecutionCount
///   and priorities arrive in a later sprint).
/// - Completion propagates child -> parent: a Maneuver completes when all
///   its Events completed (§8.4.3), a ManeuverGroup when all Maneuvers
///   (§8.4.4, an empty group completes instantly), an Act when all groups
///   (§8.4.5), a Story when all Acts (§8.4.6).
/// - The Storyboard never completes on its own; only its stop trigger
///   moves it (and every still-executing descendant) to completeState via
///   stopTransition (§8.4.7).
/// - Determinism: the walk visits elements in document order; actions of
///   simultaneously started events fire in document order.
class Scheduler {
public:
    using FireCallback = std::function<void(const ir::Action&)>;

    /// Binds the executor to a storyboard and builds its element tree. The
    /// storyboard enters runningState on the first step() call — simulation
    /// time starts with the execution of the storyboard (§8.4.7), and the
    /// engine issues that first evaluation at t = 0 during init. The
    /// storyboard must outlive the binding.
    void bind(const ir::Storyboard& storyboard);

    /// Evaluates the storyboard at `simulation_time`: checks the stop
    /// trigger, starts standby elements whose trigger holds, fires the
    /// actions of started events via `fire`, and propagates completion.
    /// No-op before bind() or after the storyboard completed.
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
    struct Node {
        enum class Kind { Storyboard, Story, Act, Group, Maneuver, Event };

        Kind kind = Kind::Storyboard;
        std::string name;
        const ir::Condition* start_trigger = nullptr; ///< Acts and Events only.
        const ir::Event* event = nullptr;             ///< Events only.
        ElementState state = ElementState::Standby;
        TransitionKind transition = TransitionKind::None;
        std::vector<Node> children;
    };

    static Node build(const ir::Storyboard& storyboard);
    static void enter_running(Node& node, double simulation_time, const FireCallback& fire);
    static void update(Node& node, double simulation_time, const FireCallback& fire);
    static void stop_cascade(Node& node);
    static bool all_children_complete(const Node& node);
    [[nodiscard]] const Node* find(const std::string& path) const;

    const ir::Storyboard* storyboard_ = nullptr;
    bool bound_ = false;
    Node root_;
};

} // namespace scena::runtime
