// SPDX-License-Identifier: Apache-2.0
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
