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
#include <vector>

#include "scena/ir/entity_types.h"

namespace scena::ir {

/// The operational domains a controller acts on, per ASAM OpenSCENARIO XML
/// 1.4.0 §ControllerType (added in 1.3; before that every controller was a
/// movement controller). Default when the attribute is omitted: `Movement`.
enum class ControllerType {
    Lateral,      ///< Lateral movement only.
    Longitudinal, ///< Longitudinal movement only.
    Lighting,     ///< Lighting only.
    Animation,    ///< Animation only.
    Movement,     ///< Lateral and longitudinal movement (the pre-1.3 behavior).
    Appearance,   ///< Lighting and animation.
    All,          ///< Every domain.
};

/// A controller model assigned to an entity, per §Controller: a name, the
/// domains it may act on, and host-defined properties.
///
/// The properties are a name/value contract between the scenario author and
/// the host simulator; Scena preserves them verbatim and in document order —
/// a vector, never an unordered map, so the hand-off to the gateway is
/// deterministic and duplicate names survive round-trip.
struct Controller {
    std::string name;
    ControllerType type = ControllerType::Movement;
    std::vector<Property> properties; ///< Ordered, document order (§Properties).
};

/// Whether this controller type may act on the lateral domain: `Lateral`,
/// `Movement` and `All` may, every other type may not (§ControllerType,
/// enforced by asam.net:xosc:1.2.0:scenario_logic.controller_activation).
[[nodiscard]] constexpr bool controls_lateral(ControllerType type) noexcept {
    return type == ControllerType::Lateral || type == ControllerType::Movement ||
           type == ControllerType::All;
}

/// Whether this controller type may act on the longitudinal domain:
/// `Longitudinal`, `Movement` and `All` may (§ControllerType).
[[nodiscard]] constexpr bool controls_longitudinal(ControllerType type) noexcept {
    return type == ControllerType::Longitudinal || type == ControllerType::Movement ||
           type == ControllerType::All;
}

} // namespace scena::ir
