// SPDX-License-Identifier: MIT
#include "scena/runtime/scheduler.h"

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

Scheduler::Node Scheduler::build(const ir::Storyboard& storyboard) {
    Node root;
    root.kind = Node::Kind::Storyboard;
    for (const ir::Story& story : storyboard.stories) {
        Node story_node;
        story_node.kind = Node::Kind::Story;
        story_node.name = story.name;
        for (const ir::Act& act : story.acts) {
            Node act_node;
            act_node.kind = Node::Kind::Act;
            act_node.name = act.name;
            act_node.start_trigger = act.start_trigger.get();
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
                        event_node.start_trigger = event.start_trigger.get();
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
    storyboard_ = &storyboard;
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
    // and every still-executing descendant to completeState (§8.4.7).
    if (storyboard_->stop_trigger != nullptr &&
        storyboard_->stop_trigger->evaluate(simulation_time)) {
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
        if (child.start_trigger == nullptr) {
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

    if (node.state == ElementState::Standby) {
        if (node.start_trigger == nullptr || !node.start_trigger->evaluate(simulation_time)) {
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
    storyboard_ = nullptr;
    bound_ = false;
    root_ = Node{};
}

} // namespace scena::runtime
