// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <optional>
#include <string_view>

#include "scena/entity_state.h"
#include "scena/ir/bounding_box.h"

namespace scena::ir {

/// The kind of storyboard element a StoryboardElementStateCondition refers to,
/// per ASAM OpenSCENARIO XML 1.4.0 StoryboardElementType enumeration. There is
/// no `Storyboard` member: the standard's element types are exactly these six.
enum class StoryboardElementType {
    Story,
    Act,
    ManeuverGroup,
    Maneuver,
    Event,
    Action,
};

/// A state or a transition a StoryboardElementStateCondition can test for, per
/// the StoryboardElementState enumeration. The standard folds runtime states
/// and discrete transitions into one type, and so does this enum: the first
/// three are level-triggered states, the last four are one-evaluation
/// transitions. `SkipTransition` is Event-only (§ StoryboardElementState).
enum class StoryboardElementState {
    StandbyState,
    RunningState,
    CompleteState,
    StartTransition,
    EndTransition,
    StopTransition,
    SkipTransition,
};

/// Which named-value namespace a value lives in. Parameters (§9.1, immutable
/// at runtime), variables (§6.12, mutable), and user-defined external values
/// (UserDefinedValueCondition) are three distinct namespaces that a name is
/// resolved within — a variable and a parameter may share a name.
enum class NamedValueKind {
    Parameter,
    Variable,
    UserDefinedValue,
};

/// Per-entity kinematic observation the by-entity conditions read (§7.6.5.1):
/// the state one evaluation observes plus the derived quantities the engine
/// accumulates. `acceleration` is a finite difference and is absent until two
/// samples exist (ADR-0007 absent ⇒ deterministic false, at the finest grain:
/// AccelerationCondition is false while Speed on the same entity still works).
struct EntityKinematics {
    EntityState state;                  ///< Snapshot the evaluation observes.
    std::optional<double> acceleration; ///< m/s^2; absent until two samples exist.
    double traveled_distance = 0.0;     ///< m, cumulative world-frame path since init.
    double standstill_seconds = 0.0;    ///< s, contiguous time at speed == 0.0.
    /// Optional geometry for the interaction conditions (p5-s3). Absent when
    /// the entity declared no bounding box; freespace metrics then evaluate to
    /// a deterministic false for that entity.
    std::optional<BoundingBox> bounding_box;
};

/// Read-only runtime context a condition is evaluated against.
///
/// This is the seam between the immutable Scenario IR and the mutable runtime:
/// conditions are pure functions of what this interface exposes, which keeps
/// them deterministic and side-effect free. The simulation time is always
/// available; the other facets are optional and default to "absent", so a
/// condition that queries a facet the context does not provide evaluates to a
/// deterministic false rather than failing. Every query must be a pure
/// observation — no state changes, and identical inputs always yield identical
/// results (the determinism contract).
class EvaluationContext {
public:
    virtual ~EvaluationContext() = default;

    /// Simulation time in seconds since the storyboard started (§8.4.7).
    [[nodiscard]] virtual double simulation_time() const = 0;

    /// Current value of a named parameter, variable, or user-defined value,
    /// as a string (the by-value conditions all compare stringly, per their
    /// XSD). std::nullopt when the name is not present in that namespace. The
    /// returned view borrows from the context and is valid only for the
    /// duration of the evaluate() call.
    [[nodiscard]] virtual std::optional<std::string_view>
    named_value(NamedValueKind /*kind*/, std::string_view /*name*/) const {
        return std::nullopt;
    }

    /// The simulated wall-clock instant as seconds since the Unix epoch, or
    /// std::nullopt when no time-of-day anchor has been set. Advances with
    /// simulation time (TimeOfDayCondition).
    [[nodiscard]] virtual std::optional<double> date_time_seconds() const { return std::nullopt; }

    /// Whether the element named `ref` of type `type` currently satisfies the
    /// state or transition `state` (StoryboardElementStateCondition). Level
    /// states hold whenever the element is in that state; transitions hold
    /// only in the evaluation the transition occurs. std::nullopt when the
    /// context cannot resolve storyboard state (e.g. `ref` names no such
    /// element, or the context has no storyboard bound).
    [[nodiscard]] virtual std::optional<bool>
    storyboard_element_state(StoryboardElementType /*type*/, std::string_view /*ref*/,
                             StoryboardElementState /*state*/) const {
        return std::nullopt;
    }

    /// Kinematic observation of the entity `id` (§7.6.5.1, by-entity
    /// conditions). std::nullopt when the entity is unknown or the context
    /// provides no entity facet at all (e.g. TimeOnlyEvaluationContext) — the
    /// per-entity expression then evaluates to a deterministic false. One
    /// coarse facet, not one per quantity: a two-entity condition (p5-s3)
    /// calls it twice and tests override a single method.
    [[nodiscard]] virtual std::optional<EntityKinematics>
    entity_kinematics(std::string_view /*id*/) const {
        return std::nullopt;
    }
};

/// The trivial context: only simulation time, every other facet absent. Used
/// by the scheduler's time-driven convenience overload and by tests that
/// exercise time-only conditions.
class TimeOnlyEvaluationContext final : public EvaluationContext {
public:
    explicit TimeOnlyEvaluationContext(double simulation_time)
        : simulation_time_(simulation_time) {}

    [[nodiscard]] double simulation_time() const override { return simulation_time_; }

private:
    double simulation_time_;
};

} // namespace scena::ir
