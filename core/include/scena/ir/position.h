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

namespace scena::ir {

/// A world-frame Cartesian position, meters (§6.3.2). This is the minimal
/// Position variant the kernel needs today; the ten §6.3.8 Position variants
/// (lane, road, relative, …) and the PositionResolver arrive with p2-s4/p3-s4.
/// ReachPositionCondition compares against it directly in world coordinates
/// until then.
struct WorldPosition {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

} // namespace scena::ir
