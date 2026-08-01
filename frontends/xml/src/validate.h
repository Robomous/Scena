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

#include "reader_context.h"
#include "scena/xml/document.h"

namespace scena::xml::detail {

/// Checks a loaded document for the defects an element reader cannot see on
/// its own — the ones that need the whole file.
///
/// The element readers validate what is in front of them: an attribute's
/// type, a required child, a choice with one alternative. Whether an
/// `entityRef` names a declared entity, or a `StoryboardElementStateCondition`
/// names an element that exists and is of the type it claims, can only be
/// answered once everything has been read. That is this pass.
///
/// It reports:
/// - **Referential integrity** — entity references in actions, actors and
///   triggering entities (rule
///   `reference_control.references_to_scenario_object`); storyboard-element
///   references (`reference_control.resolvable_storyboard_element_ref`);
///   variable references (`reference_control.resolvable_variable_reference`);
///   traffic-signal controller references
///   (`reference_control.traffic_signal_controller_references`).
/// - **Naming** — duplicate names among siblings
///   (`naming.unique_element_names_on_same_level`).
/// - **Unused declarations** — a declared parameter or variable that nothing
///   references, as a warning: harmless, but almost always a typo in the
///   reference rather than a deliberate spare.
///
/// Everything is reported through `ctx`, so a caller gets one diagnostic
/// stream for reading and checking alike.
void validate_document(ReadContext& ctx, const Document& document);

} // namespace scena::xml::detail
