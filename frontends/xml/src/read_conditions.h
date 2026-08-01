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

#include <pugixml.hpp>

#include "reader_context.h"
#include "scena/ir/trigger.h"

namespace scena::xml::detail {

/// Reads a `Trigger` element (§7.6.1): OR over ConditionGroups, each an AND
/// over Conditions. Returns nullopt when the trigger has no readable content,
/// after reporting why.
///
/// The distinction the storyboard relies on: an *absent* trigger element and
/// an *empty* one mean different things (§7.6.1) — absent starts with the
/// parent, empty never fires — so this reader is only called when the element
/// exists, and an element with zero groups lowers to an empty Trigger.
[[nodiscard]] std::optional<ir::Trigger> read_trigger(ReadContext& ctx, const pugi::xml_node& node);

} // namespace scena::xml::detail
