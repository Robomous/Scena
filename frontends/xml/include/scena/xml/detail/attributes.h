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
#include <string_view>

namespace scena::xml::detail {

/// Locale-independent conversions for XML attribute text.
///
/// Every numeric conversion in the frontend goes through this header. The
/// project rule behind it: `std::stod`/`strtod`/`atoi` and the iostream
/// extractors all honor the C locale, so a scenario file parsed under a
/// comma-decimal locale would read "1.5" as 1 (or fail) on one machine and as
/// 1.5 on another. That breaks the bit-identical determinism contract before
/// the engine ever runs, and it is the single most common cross-platform bug
/// in XML loading. `std::from_chars` is locale-independent by definition, and
/// it is the only conversion used here.
///
/// All three parsers are whole-token: trailing characters make the token
/// invalid rather than being ignored, so "1.5x", "1,5" and " 1.5" are
/// rejected. Attribute values in OpenSCENARIO carry no units and no
/// thousands separators, so there is nothing legitimate to skip.

/// Parses a double attribute, e.g. `@length`, `@value`.
///
/// Delegates to `ir::parse_scalar`, the kernel's single implementation of
/// locale-safe scalar reading, so the frontend and the runtime agree on
/// exactly which strings are numbers (including "+5", "inf" and "nan", which
/// `parse_scalar` documents).
[[nodiscard]] std::optional<double> parse_double(std::string_view text);

/// Parses a signed integer attribute, e.g. `@revMajor`, `@laneId`.
///
/// Returns nullopt for anything that is not an integer token, including
/// otherwise-valid doubles ("1.0"): the XSD integer types are not spelled
/// with a fractional part, and silently truncating would hide a defect in the
/// document. A single leading '+' is accepted, mirroring parse_double.
[[nodiscard]] std::optional<long long> parse_integer(std::string_view text);

/// Parses a boolean attribute.
///
/// ASAM OpenSCENARIO XML allows boolean literals to be given as `0`, `1`,
/// `true` and `false` (rule asam.net:xosc:1.1.0:expressions.type_of_boolean).
/// Case is significant — XSD boolean spells them lower case — so "True" is
/// not a boolean and returns nullopt rather than silently reading as true.
[[nodiscard]] std::optional<bool> parse_boolean(std::string_view text);

} // namespace scena::xml::detail
