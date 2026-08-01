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

#pragma once

#include <memory>
#include <optional>

#include <pugixml.hpp>

#include "reader_context.h"
#include "scena/ir/dynamics.h"
#include "scena/ir/position.h"
#include "scena/ir/route.h"
#include "scena/ir/trajectory.h"

namespace scena::xml::detail {

/// Reads a `Position` element (§6.3.8) — the ten-alternative choice — into the
/// IR variant. Returns nullopt when the alternative is unreadable or outside
/// the loaded subset, after reporting why.
[[nodiscard]] std::optional<ir::Position> read_position(ReadContext& ctx,
                                                        const pugi::xml_node& node);

/// Reads an `Orientation` element (§Orientation).
[[nodiscard]] ir::Orientation read_orientation(ReadContext& ctx, const pugi::xml_node& node);

/// Reads a `TransitionDynamics` element (§TransitionDynamics): shape,
/// dimension, value, and the 1.2 followingMode.
[[nodiscard]] bool read_transition_dynamics(ReadContext& ctx, const pugi::xml_node& node,
                                            ir::TransitionDynamics& out);

/// Reads a `Trajectory` element (§6.9) with its Shape choice.
[[nodiscard]] std::shared_ptr<ir::Trajectory> read_trajectory(ReadContext& ctx,
                                                              const pugi::xml_node& node);

/// Reads a `Route` element (§6.8.2) with its waypoints.
[[nodiscard]] std::shared_ptr<ir::Route> read_route(ReadContext& ctx, const pugi::xml_node& node);

} // namespace scena::xml::detail
