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

#include "scena/ir/entity.h"

#include <optional>
#include <type_traits>
#include <variant>

namespace scena::ir {

std::optional<ObjectType> object_type_of(const Entity& entity) {
    if (!entity.object.has_value()) {
        return std::nullopt;
    }
    // Alternative order matches ObjectType (Vehicle, Pedestrian, MiscObject).
    return std::visit(
        [](const auto& obj) -> ObjectType {
            using T = std::decay_t<decltype(obj)>;
            if constexpr (std::is_same_v<T, Vehicle>) {
                return ObjectType::Vehicle;
            } else if constexpr (std::is_same_v<T, Pedestrian>) {
                return ObjectType::Pedestrian;
            } else {
                return ObjectType::MiscObject;
            }
        },
        *entity.object);
}

std::optional<BoundingBox> bounding_box_of(const Entity& entity) {
    if (!entity.object.has_value()) {
        return std::nullopt;
    }
    // Every concrete object carries a bounding box (§Vehicle/§Pedestrian/
    // §MiscObject all require BoundingBox).
    return std::visit([](const auto& obj) { return obj.bounding_box; }, *entity.object);
}

const Performance* performance_of(const Entity& entity) {
    if (!entity.object.has_value()) {
        return nullptr;
    }
    // Only Vehicle defines Performance in the standard.
    if (const auto* vehicle = std::get_if<Vehicle>(&*entity.object)) {
        return &vehicle->performance;
    }
    return nullptr;
}

} // namespace scena::ir
