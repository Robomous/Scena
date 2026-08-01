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

#include "scena/gateway/road_query.h"

namespace scena::gateway {

/// Null-object `IRoadQuery` for road-free scenarios.
///
/// A world with no road network: every query answers false, which per the
/// interface's unsupported-reporting semantics means "no answer" everywhere.
/// The engine then uses its flat-world fallbacks (ADR-0016) exactly as it
/// does when a gateway returns no road query at all.
///
/// Hosts that sometimes run without a map can hand this object to the engine
/// instead of branching on nullptr; it is also the first backend the
/// `road_query_contract_test` suite runs against, pinning the contract every
/// real backend must satisfy (p3-s2/p3-s3).
class FlatWorldRoadQuery final : public IRoadQuery {
public:
    [[nodiscard]] bool to_lane_position(double /*x*/, double /*y*/, double /*z*/,
                                        LanePosition& /*out*/) const override {
        return false;
    }

    [[nodiscard]] bool to_world_position(const LanePosition& /*position*/, double& /*x*/,
                                         double& /*y*/, double& /*z*/) const override {
        return false;
    }
};

} // namespace scena::gateway
