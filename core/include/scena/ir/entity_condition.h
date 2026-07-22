// SPDX-License-Identifier: MIT
#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "scena/ir/condition.h"

namespace scena::ir {

/// How the per-entity logical expression of a by-entity condition is reduced
/// over the triggering entities, per ASAM OpenSCENARIO XML 1.4.0
/// TriggeringEntitiesRule (§7.6.5.1): `any` (at least one entity satisfies it)
/// or `all` (every entity does).
enum class TriggeringEntitiesRule {
    Any,
    All,
};

/// The set of entities a by-entity condition observes (§7.6.5.1). The logical
/// expression is evaluated independently per entity, then reduced by `rule`.
/// `entity_refs` has 1..* members (validated at Engine::init).
struct TriggeringEntities {
    TriggeringEntitiesRule rule = TriggeringEntitiesRule::Any;
    std::vector<std::string> entity_refs;
};

/// A direction in the entity's own body frame, per DirectionalDimension
/// (§ enum, ISO 8855:2011): longitudinal is +x forward, lateral +y left,
/// vertical +z up. Used to project a velocity/acceleration onto one axis; when
/// a by-entity condition omits it, the total (magnitude) is compared instead.
enum class DirectionalDimension {
    Longitudinal,
    Lateral,
    Vertical,
};

/// Base class for the ByEntityCondition family (§7.6.5.1): a per-entity
/// logical expression evaluated over a set of triggering entities and reduced
/// with any/all. The reduction is invariant across every by-entity condition
/// (and p5-s3's two-entity set), so it lives here and `evaluate()` is `final`:
/// no subclass can skip it. This mirrors the standard's own ByEntityCondition
/// → EntityCondition hierarchy.
class ByEntityCondition : public Condition {
public:
    /// Evaluates the per-entity predicate for every triggering entity (no
    /// short-circuit, so any future per-entity diagnostic stays order
    /// independent), then reduces: Any = OR, All = AND. Reduction happens here,
    /// before the scheduler's edge/delay machinery, exactly as §7.6.5.1
    /// requires. Empty `entity_refs` ⇒ false (unreachable after validation).
    [[nodiscard]] bool evaluate(const EvaluationContext& context) const final;

    [[nodiscard]] const TriggeringEntities& triggering_entities() const;

protected:
    explicit ByEntityCondition(TriggeringEntities triggering_entities);

    /// The condition-specific predicate for a single entity. Called once per
    /// triggering entity by evaluate(). An entity the context cannot observe
    /// (absent facet) yields false for that entity only.
    [[nodiscard]] virtual bool evaluate_for_entity(const EvaluationContext& context,
                                                   std::string_view entity_id) const = 0;

private:
    TriggeringEntities triggering_entities_;
};

/// Compares a triggering entity's speed to `value` under `rule`, per
/// SpeedCondition. Without `direction` the total speed (magnitude) is
/// compared; with it, the signed projection on that body axis (§7.6.5.1).
class SpeedCondition final : public ByEntityCondition {
public:
    SpeedCondition(TriggeringEntities triggering, double value, Rule rule,
                   std::optional<DirectionalDimension> direction = std::nullopt);

    [[nodiscard]] double value() const;
    [[nodiscard]] Rule rule() const;
    [[nodiscard]] const std::optional<DirectionalDimension>& direction() const;

protected:
    [[nodiscard]] bool evaluate_for_entity(const EvaluationContext& context,
                                           std::string_view entity_id) const override;

private:
    double value_;
    Rule rule_;
    std::optional<DirectionalDimension> direction_;
};

/// Compares a triggering entity's speed relative to a reference entity to
/// `value` under `rule`, per RelativeSpeedCondition. The spec defines
/// speed_rel = speed(triggering) - speed(reference); with `direction` the
/// relative velocity is projected in the *triggering* entity's frame.
class RelativeSpeedCondition final : public ByEntityCondition {
public:
    RelativeSpeedCondition(TriggeringEntities triggering, std::string entity_ref, double value,
                           Rule rule, std::optional<DirectionalDimension> direction = std::nullopt);

    [[nodiscard]] const std::string& entity_ref() const;
    [[nodiscard]] double value() const;
    [[nodiscard]] Rule rule() const;
    [[nodiscard]] const std::optional<DirectionalDimension>& direction() const;

protected:
    [[nodiscard]] bool evaluate_for_entity(const EvaluationContext& context,
                                           std::string_view entity_id) const override;

private:
    std::string entity_ref_;
    double value_;
    Rule rule_;
    std::optional<DirectionalDimension> direction_;
};

/// Compares a triggering entity's acceleration to `value` under `rule`, per
/// AccelerationCondition. Acceleration is a finite difference (see
/// EntityKinematics): until two samples exist it is absent and the condition
/// is deterministically false. Direction handling mirrors SpeedCondition.
class AccelerationCondition final : public ByEntityCondition {
public:
    AccelerationCondition(TriggeringEntities triggering, double value, Rule rule,
                          std::optional<DirectionalDimension> direction = std::nullopt);

    [[nodiscard]] double value() const;
    [[nodiscard]] Rule rule() const;
    [[nodiscard]] const std::optional<DirectionalDimension>& direction() const;

protected:
    [[nodiscard]] bool evaluate_for_entity(const EvaluationContext& context,
                                           std::string_view entity_id) const override;

private:
    double value_;
    Rule rule_;
    std::optional<DirectionalDimension> direction_;
};

} // namespace scena::ir
