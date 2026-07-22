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

#include <string>

#include "scena/ir/scenario.h"
#include "scena/ir/storyboard.h"

namespace scena::testsupport {

/// A timed speed event: fires at `at_time` and drives `entity_id` to
/// `target_speed`. Shared building block for the determinism fixture.
[[nodiscard]] ir::Event make_speed_event(std::string name, double at_time, std::string entity_id,
                                         double target_speed);

/// Hierarchical determinism fixture: two parallel stories, one act behind a
/// start trigger, several timed events — enough structure to exercise the
/// storyboard walk order, plus an init action. Shared verbatim between the
/// determinism tests and the trace runner so both drive byte-for-byte the same
/// scenario; that is what makes the recorded trace meaningful as a
/// cross-platform golden.
[[nodiscard]] ir::Scenario make_determinism_scenario();

} // namespace scena::testsupport
