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

namespace scena {

/// Detectability of an entity, per ASAM OpenSCENARIO XML 1.4.0
/// §VisibilityAction: "the default for entities is that they are visible
/// everywhere", so every flag defaults to true.
///
/// The engine holds this as per-entity runtime state and hands each change to
/// the gateway (`ISimulatorGateway::on_visibility_changed`) — the engine itself
/// has no image generator, sensor model, or traffic participants, so the flags
/// are a contract with the host rather than something the kernel enacts.
///
/// The 1.2 `sensorReferenceSet` (per-sensor visibility) is not modeled; the
/// flags apply to all sensors.
struct EntityVisibility {
    bool graphics = true; ///< Visible in the host's image generator(s).
    bool sensors = true;  ///< Visible to the host's sensor model(s).
    bool traffic = true;  ///< Visible to other traffic participants.
};

} // namespace scena
