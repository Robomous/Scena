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

#include <pugixml.hpp>

#include "reader_context.h"
#include "scena/ir/scenario.h"

namespace scena::xml::detail {

/// Reads the `Storyboard` element (§8.3): the Init phase into
/// `out.init_actions` and the Story/Act/ManeuverGroup/Maneuver/Event
/// hierarchy plus the storyboard stop trigger into `out.storyboard`.
void read_storyboard(ReadContext& ctx, const pugi::xml_node& node, ir::Scenario& out);

} // namespace scena::xml::detail
