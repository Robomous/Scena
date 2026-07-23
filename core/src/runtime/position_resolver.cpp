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

#include "scena/runtime/position_resolver.h"

#include <utility>
#include <variant>

#include "scena/runtime/detmath.h"

namespace scena::runtime {
namespace {

/// Composes the orientation of a resolved pose against the reference frame's
/// orientation (base_*), per §Orientation. A missing Orientation copies the
/// reference orientation; an Absolute one replaces it with world-frame angles;
/// a Relative one is an additive counter-clockwise shift on top of it.
void compose_orientation(double base_h, double base_p, double base_r,
                         const std::optional<ir::Orientation>& orientation, Pose& out) {
    if (!orientation.has_value()) {
        out.heading = base_h;
        out.pitch = base_p;
        out.roll = base_r;
        return;
    }
    if (orientation->type == ir::ReferenceContext::Absolute) {
        out.heading = orientation->h;
        out.pitch = orientation->p;
        out.roll = orientation->r;
        return;
    }
    out.heading = base_h + orientation->h;
    out.pitch = base_p + orientation->p;
    out.roll = base_r + orientation->r;
}

PositionResolution ok() {
    return PositionResolution{Status::Ok, {}, {}};
}

PositionResolution unresolved_reference(const std::string& entity_ref) {
    return PositionResolution{Status::SemanticError,
                              "position references entity '" + entity_ref +
                                  "', which is not an active entity",
                              {}};
}

PositionResolution unsupported(std::string message, std::string rule_id = {}) {
    return PositionResolution{Status::UnsupportedFeature, std::move(message), std::move(rule_id)};
}

} // namespace

PositionResolver::PositionResolver(PoseLookup lookup) noexcept : lookup_(std::move(lookup)) {}

PositionResolution PositionResolver::resolve(const ir::Position& position, Pose& out) const {
    // WorldPosition (§WorldPosition, §6.3.1): the pose is taken directly. Its
    // h/p/r are the world-frame orientation (WorldPosition has no separate
    // Orientation element), so they are inherently absolute.
    if (const auto* world = std::get_if<ir::WorldPosition>(&position)) {
        out.x = world->x;
        out.y = world->y;
        out.z = world->z;
        out.heading = world->h;
        out.pitch = world->p;
        out.roll = world->r;
        return ok();
    }

    // RelativeWorldPosition (§RelativeWorldPosition): deltas along the WORLD
    // axes added to the reference entity's position — not rotated by its
    // heading.
    if (const auto* rel_world = std::get_if<ir::RelativeWorldPosition>(&position)) {
        const EntityState* ref = lookup_(rel_world->entity_ref);
        if (ref == nullptr) {
            return unresolved_reference(rel_world->entity_ref);
        }
        out.x = ref->x + rel_world->dx;
        out.y = ref->y + rel_world->dy;
        out.z = ref->z + rel_world->dz;
        compose_orientation(ref->heading, ref->pitch, ref->roll, rel_world->orientation, out);
        return ok();
    }

    // RelativeObjectPosition (§RelativeObjectPosition): deltas expressed in the
    // reference entity's LOCAL frame, so they rotate with its orientation. The
    // straight-line runtime keeps pitch and roll at 0 (entity_state.h), so the
    // rotation is a yaw about +Z — exact for every state the runtime produces;
    // the full Z→Y→X frame rotation lands when the runtime carries a non-zero
    // pitch/roll. Uses det_sincos for bit-identical results (ADR-0006).
    if (const auto* rel_obj = std::get_if<ir::RelativeObjectPosition>(&position)) {
        const EntityState* ref = lookup_(rel_obj->entity_ref);
        if (ref == nullptr) {
            return unresolved_reference(rel_obj->entity_ref);
        }
        const SinCos rot = det_sincos(ref->heading);
        out.x = ref->x + rel_obj->dx * rot.cos - rel_obj->dy * rot.sin;
        out.y = ref->y + rel_obj->dx * rot.sin + rel_obj->dy * rot.cos;
        out.z = ref->z + rel_obj->dz;
        compose_orientation(ref->heading, ref->pitch, ref->roll, rel_obj->orientation, out);
        return ok();
    }

    // Road-family variants (§RoadPosition, §RelativeRoadPosition, §LanePosition,
    // §RelativeLanePosition, §RoutePosition): resolving the position needs a
    // road network, and the relative-orientation reference context needs the
    // s-axis tangent at the target — neither exists until the road backend
    // (p3-s4). Reported, never silently wrong.
    if (std::holds_alternative<ir::RoadPosition>(position) ||
        std::holds_alternative<ir::RelativeRoadPosition>(position) ||
        std::holds_alternative<ir::LanePosition>(position) ||
        std::holds_alternative<ir::RelativeLanePosition>(position) ||
        std::holds_alternative<ir::RoutePosition>(position)) {
        return unsupported("road-, lane- and route-relative positions require a road network; "
                           "resolution lands with the road backend (p3-s4)");
    }

    // GeoPosition (§GeoPosition): the geographic→world projection is defined by
    // the road network's geodetic datum, which Scena does not yet consume.
    if (std::holds_alternative<ir::GeoPosition>(position)) {
        return unsupported("geographic positions require a geodetic datum in the road network",
                           "asam.net:xosc:1.1.0:positioning.geodetic_datum_defined");
    }

    // TrajectoryPosition (§TrajectoryPosition): needs the trajectory shape
    // evaluation that lands with p2-s5.
    if (std::holds_alternative<ir::TrajectoryPosition>(position)) {
        return unsupported("trajectory-relative positions require trajectory shapes (p2-s5)");
    }

    // Unreachable: the variant has ten alternatives and all are handled above.
    return unsupported("unhandled position variant");
}

} // namespace scena::runtime
