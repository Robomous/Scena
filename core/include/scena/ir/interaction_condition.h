// SPDX-License-Identifier: MIT
#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "scena/ir/entity_condition.h"
#include "scena/ir/position.h"
#include "scena/ir/rule.h"

namespace scena::ir {

/// The referential a distance is measured in, per ASAM OpenSCENARIO XML 1.4.0
/// CoordinateSystem (§6.4). In a condition it relates to the triggering
/// entity. `Entity` is the default. `Lane`, `Road`, and `Trajectory` need a
/// road network (IRoadQuery, p3-s4) and evaluate to a deterministic false with
/// an init-time UnsupportedFeature warning until then (ADR-0009).
enum class CoordinateSystem {
    Entity,
    Lane,
    Road,
    Trajectory,
    World,
};

/// Which dimension(s) a distance uses, per RelativeDistanceType (§6.4).
/// `Longitudinal`/`Lateral` project onto one axis of the effective coordinate
/// system; `EuclidianDistance` (the default) is the coordinate-system-
/// independent 3D magnitude (§6.4.3). `CartesianDistance` is deprecated and is
/// treated as EuclidianDistance with a warning.
enum class RelativeDistanceType {
    Longitudinal,
    Lateral,
    CartesianDistance, ///< Deprecated; treated as EuclidianDistance.
    EuclidianDistance,
};

/// Route-selection algorithm for road/lane distances, per RoutingAlgorithm
/// (§6.4.8.3). Only relevant to the road/lane coordinate systems, which are
/// deferred this phase, so it is stored and exposed but never changes an
/// evaluation (default `Undefined` makes silent-ignore spec-conforming).
enum class RoutingAlgorithm {
    AssignedRoute,
    Fastest,
    LeastIntersections,
    Shortest,
    Undefined,
};

/// A signed range of lanes left/right of the triggering entity, per
/// RelativeLaneRange (RelativeClearanceCondition). 0 is the entity's current
/// lane, positive is left, negative is right. `from` defaults to -inf, `to` to
/// +inf when omitted (std::nullopt).
struct RelativeLaneRange {
    std::optional<int> from;
    std::optional<int> to;
};

/// Base for the interaction subset of ByEntityCondition (§7.6.5.1): the
/// two-entity and entity-to-position metrics (distance, headway, collision).
/// It only bundles the shared distance-parameter accessors used across
/// Distance/RelativeDistance/TimeHeadway/TimeToCollision; the any/all
/// reduction and per-entity dispatch stay in ByEntityCondition.
///
/// coordinateSystem and relativeDistanceType are stored as optionals ("was it
/// authored?") so the alongRoute-ignored-iff-authored rule (§ DistanceCondition
/// "If coordinateSystem or relativeDistanceType are set, alongRoute is
/// ignored") can be implemented exactly.

/// Compares the distance between the triggering entity and a fixed position to
/// `value` under `rule`, per DistanceCondition. `freespace` selects
/// bounding-box vs reference-point distance (§6.4.7). `along_route` is
/// deprecated and ignored when coordinateSystem or relativeDistanceType is set.
class DistanceCondition final : public ByEntityCondition {
public:
    DistanceCondition(TriggeringEntities triggering, WorldPosition position, double value,
                      bool freespace, Rule rule,
                      std::optional<CoordinateSystem> coordinate_system = std::nullopt,
                      std::optional<RelativeDistanceType> relative_distance_type = std::nullopt,
                      std::optional<RoutingAlgorithm> routing_algorithm = std::nullopt,
                      std::optional<bool> along_route = std::nullopt);

    [[nodiscard]] const WorldPosition& position() const;
    [[nodiscard]] double value() const;
    [[nodiscard]] bool freespace() const;
    [[nodiscard]] Rule rule() const;
    [[nodiscard]] const std::optional<CoordinateSystem>& coordinate_system() const;
    [[nodiscard]] const std::optional<RelativeDistanceType>& relative_distance_type() const;
    [[nodiscard]] const std::optional<RoutingAlgorithm>& routing_algorithm() const;
    [[nodiscard]] const std::optional<bool>& along_route() const;

    /// The coordinate system after applying the defaults and the alongRoute
    /// promotion (§ DistanceCondition): authored CS if present; else Road when
    /// neither CS nor RDT is authored and alongRoute is true; else Entity.
    [[nodiscard]] CoordinateSystem effective_coordinate_system() const;
    /// The relative-distance type after applying the default (EuclidianDistance).
    [[nodiscard]] RelativeDistanceType effective_relative_distance_type() const;

protected:
    [[nodiscard]] bool evaluate_for_entity(const EvaluationContext& context,
                                           std::string_view entity_id) const override;

private:
    WorldPosition position_;
    double value_;
    bool freespace_;
    Rule rule_;
    std::optional<CoordinateSystem> coordinate_system_;
    std::optional<RelativeDistanceType> relative_distance_type_;
    std::optional<RoutingAlgorithm> routing_algorithm_;
    std::optional<bool> along_route_;
};

/// Compares the distance between the triggering entity and a reference entity
/// to `value` under `rule`, per RelativeDistanceCondition. Unlike
/// DistanceCondition the relativeDistanceType is required and there is no
/// alongRoute attribute.
class RelativeDistanceCondition final : public ByEntityCondition {
public:
    RelativeDistanceCondition(TriggeringEntities triggering, std::string entity_ref, double value,
                              bool freespace, RelativeDistanceType relative_distance_type,
                              Rule rule,
                              std::optional<CoordinateSystem> coordinate_system = std::nullopt,
                              std::optional<RoutingAlgorithm> routing_algorithm = std::nullopt);

    [[nodiscard]] const std::string& entity_ref() const;
    [[nodiscard]] double value() const;
    [[nodiscard]] bool freespace() const;
    [[nodiscard]] RelativeDistanceType relative_distance_type() const;
    [[nodiscard]] Rule rule() const;
    [[nodiscard]] const std::optional<CoordinateSystem>& coordinate_system() const;
    [[nodiscard]] const std::optional<RoutingAlgorithm>& routing_algorithm() const;

    /// The coordinate system after the default: authored CS if present, else
    /// Entity (there is no alongRoute promotion here).
    [[nodiscard]] CoordinateSystem effective_coordinate_system() const;

protected:
    [[nodiscard]] bool evaluate_for_entity(const EvaluationContext& context,
                                           std::string_view entity_id) const override;

private:
    std::string entity_ref_;
    double value_;
    bool freespace_;
    RelativeDistanceType relative_distance_type_;
    Rule rule_;
    std::optional<CoordinateSystem> coordinate_system_;
    std::optional<RoutingAlgorithm> routing_algorithm_;
};

} // namespace scena::ir
