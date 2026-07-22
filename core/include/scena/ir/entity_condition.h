// SPDX-License-Identifier: MIT
#pragma once

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

} // namespace scena::ir
