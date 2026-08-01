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

#include <cstdint>

namespace scena::ir {

class Action;

/// The entity control domains an action may assign a *control strategy* to,
/// per ASAM OpenSCENARIO XML 1.4.0 §7.4.1.1. Every entity has a control
/// strategy on the longitudinal and lateral domains at all times; the default
/// controller supplies one where no action does.
///
/// A bitmask, because an action may own more than one domain at once — a
/// FollowTrajectoryAction with `timeReference = timing` steers *and* paces, so
/// it owns both motion domains (Annex A Table 10).
///
/// Lighting and Animation are declared because §7.4.1.1 names them and Table 10
/// classifies actions against them; Scena's v0.0.1 subset ships no action that
/// claims either, so `control_domains` never returns them today. They are here
/// so that the conflict rule is written once against the standard's domain set
/// rather than against the subset that happens to exist.
enum class ActionDomain : std::uint8_t {
    /// Assigns no control strategy: the action sets a state and completes
    /// immediately (§7.4.1.2 "non-motion control actions"), so it can never
    /// conflict with anything.
    None = 0,
    /// Speed along the entity's own axis (§7.4.1.1 "controlling the length of
    /// the vehicle's speed vector").
    Longitudinal = 1u << 0u,
    /// Lane keeping, lane offset and steering (§7.4.1.1).
    Lateral = 1u << 1u,
    /// Lights (§7.4.1.1). No v0.0.1 action claims it.
    Lighting = 1u << 2u,
    /// Animations (§7.4.1.1). No v0.0.1 action claims it.
    Animation = 1u << 3u,
};

[[nodiscard]] constexpr ActionDomain operator|(ActionDomain lhs, ActionDomain rhs) noexcept {
    return static_cast<ActionDomain>(static_cast<std::uint8_t>(lhs) |
                                     static_cast<std::uint8_t>(rhs));
}

[[nodiscard]] constexpr ActionDomain operator&(ActionDomain lhs, ActionDomain rhs) noexcept {
    return static_cast<ActionDomain>(static_cast<std::uint8_t>(lhs) &
                                     static_cast<std::uint8_t>(rhs));
}

/// True when `domains` includes every domain in `wanted` (and `wanted` is not
/// empty).
[[nodiscard]] constexpr bool holds(ActionDomain domains, ActionDomain wanted) noexcept {
    return wanted != ActionDomain::None && (domains & wanted) == wanted;
}

/// True when the two domain sets overlap — the §7.5.1 test for whether two
/// actions on the same entity conflict: "actions are treated as conflicting if
/// they are competing for control of the same domain in the same resource".
[[nodiscard]] constexpr bool conflicts(ActionDomain lhs, ActionDomain rhs) noexcept {
    return (lhs & rhs) != ActionDomain::None;
}

/// The domains `action` assigns a control strategy to, per §7.4.1.2 and Annex A
/// Table 10.
///
/// The classification is *settings-dependent*, not type-dependent, which is why
/// this takes an action rather than a kind string. §7.4.1.2: "LaneChangeActions,
/// SpeedAction, LaneOffsetAction may be used to set a state, if used with the
/// step dynamic option. In this particular use case, these actions do not assign
/// a control strategy as the changes are enacted instantaneously." A Step-shaped
/// SpeedAction therefore returns None and never overrides a running ramp — it
/// writes a speed and is done.
///
/// Actions outside Scena's implemented subset return None: an action the runtime
/// does not execute holds no domain, so it can neither override nor be
/// overridden.
[[nodiscard]] ActionDomain control_domains(const Action& action) noexcept;

} // namespace scena::ir
