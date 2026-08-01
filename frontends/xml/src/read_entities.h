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

#include <optional>
#include <vector>

#include <pugixml.hpp>

#include "reader_context.h"
#include "scena/ir/bounding_box.h"
#include "scena/ir/entity.h"

namespace scena::xml::detail {

/// Reads the `Entities` element (§7.2.2) into the IR entity list, in document
/// order.
///
/// `ScenarioObject/@name` is both the IR id and the display name: the
/// standard identifies entities by that single name everywhere (actions,
/// conditions, actors), so splitting it would invent an identity the document
/// does not have.
void read_entities(ReadContext& ctx, const pugi::xml_node& entities, std::vector<ir::Entity>& out);

/// Reads a `BoundingBox` element (§BoundingBox: Center + Dimensions).
[[nodiscard]] bool read_bounding_box(ReadContext& ctx, const pugi::xml_node& node,
                                     ir::BoundingBox& out);

/// Reads a `Properties` element (§Properties) into name/value pairs in
/// document order. Property files (`File` children) are reported as deferred.
void read_properties(ReadContext& ctx, const pugi::xml_node& node, std::vector<ir::Property>& out);

} // namespace scena::xml::detail
