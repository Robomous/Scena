// SPDX-FileCopyrightText: 2026 Robomous
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <optional>

#include "scena/entity_state.h"
#include "scena/ir/bounding_box.h"
#include "scena/ir/evaluation_context.h"
#include "scena/ir/interaction_condition.h"
#include "scena/ir/position.h"
#include "scena/runtime/obb2.h"

namespace scena::runtime {

/// Shared entity-to-target distance measurement, per ASAM OpenSCENARIO XML
/// 1.4.0 §6.4.
///
/// These are the kernels the interaction conditions (p5-s3) were built on;
/// p5-s5 promoted them out of interaction_condition.cpp because the
/// longitudinal distance-keeping action measures the very same gap and must
/// measure it identically — one implementation, one set of hex-pinned bits.
///
/// Everything here is pure: no engine, gateway, or wall-clock dependency. All
/// arithmetic is IEEE-exact with fixed operand order and the only trigonometry
/// is det_sincos (inside make_obb / projection_axis), so results are
/// bit-identical across platforms (the determinism contract).

/// The effective distance parameters after defaults/deprecations are resolved.
struct DistanceSpec {
    ir::CoordinateSystem cs = ir::CoordinateSystem::Entity; ///< Effective coordinate system.
    /// Effective relative-distance type.
    ir::RelativeDistanceType rdt = ir::RelativeDistanceType::EuclidianDistance;
    bool freespace = false; ///< Bounding-box (true) vs reference-point (false) distance.
};

/// Builds the world-frame 2D oriented box for an entity from its state and its
/// body-frame bounding box. The heading's sine/cosine come from det_sincos —
/// the single trig site for the whole distance calculation — so the geometry
/// is bit-identical across platforms. Fixed operand order; length/width halved
/// by an exact multiply.
[[nodiscard]] Obb2 make_obb(const EntityState& state, const ir::BoundingBox& box);

/// The unit axis a longitudinal/lateral distance projects onto, in world
/// coordinates. Entity CS uses the reference entity's body axes (§6.4.4, "the
/// coordinate system relates to the triggering entity") via det_sincos; World
/// CS uses the world X/Y axes. Only Entity and World reach here — road-based
/// systems are deferred before this is called.
void projection_axis(ir::CoordinateSystem cs, ir::RelativeDistanceType rdt, double heading,
                     double& ux, double& uy);

/// Measures the distance from the triggering entity to a target — another
/// entity (`target_entity` non-null) or a fixed position (`target_point`) —
/// under `spec`, per §6.4. Returns std::nullopt when the measurement cannot be
/// made and the caller must fall back to its deferred behaviour: a road-based
/// coordinate system (deferred to p3-s4) or a freespace request with a missing
/// bounding box (geometry is optional until p2-s1).
[[nodiscard]] std::optional<double> measure_distance(const ir::EntityKinematics& trigger,
                                                     const ir::EntityKinematics* target_entity,
                                                     const ir::WorldPosition& target_point,
                                                     const DistanceSpec& spec);

/// The closing speed (rate at which the separation shrinks, positive when
/// approaching) between the triggering entity and its target, in the effective
/// coordinate system + relative-distance type, per TimeToCollisionCondition.
/// Planar velocity is speed·(cos_h, sin_h) per entity (det_sincos); a position
/// target is stationary. The axis is frozen at the evaluation instant (no
/// yaw-rate term). Returns std::nullopt when the reference points coincide
/// (division would be undefined) — TTC is then false. Reference-point closing
/// speed is used for freespace too (the spec ties the relative speed to the
/// coordinate system, not to the freespace gap; ADR-0009).
[[nodiscard]] std::optional<double> closing_speed(const ir::EntityKinematics& trigger,
                                                  const ir::EntityKinematics* target_entity,
                                                  const ir::WorldPosition& target_point,
                                                  const DistanceSpec& spec);

} // namespace scena::runtime
