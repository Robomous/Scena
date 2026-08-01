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
#include <string>
#include <vector>

#include <pugixml.hpp>

#include "reader_context.h"
#include "scena/ir/action.h"
#include "scena/ir/traffic_signal.h"

namespace scena::xml::detail {

/// Reads a `PrivateAction` element (§7.4.1) for `entity_id`. Returns nullptr
/// when the action is unreadable or outside the loaded subset, after
/// reporting why.
[[nodiscard]] std::shared_ptr<ir::Action>
read_private_action(ReadContext& ctx, const pugi::xml_node& node, const std::string& entity_id);

/// Reads a `GlobalAction` element (§7.4.2) or a `UserDefinedAction` (§7.4.3).
[[nodiscard]] std::shared_ptr<ir::Action> read_global_action(ReadContext& ctx,
                                                             const pugi::xml_node& node);

/// Reads a `TrafficSignalController` element (§6.11.2), which a scenario
/// declares inside RoadNetwork.
[[nodiscard]] bool read_traffic_signal_controller(ReadContext& ctx, const pugi::xml_node& node,
                                                  ir::TrafficSignalController& out);

} // namespace scena::xml::detail
