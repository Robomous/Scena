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

/// A civil date-time, the value type behind TimeOfDayCondition and the
/// Environment TimeOfDay, per ASAM OpenSCENARIO XML 1.4.0 dateTime data type
/// (preface "Data types", Table 5): ISO 8601 basic notation
/// "yyyy-MM-ddTHH:mm:ss.FFFZ" with an optional RFC-822 UTC offset.
///
/// This is the runtime representation only — no string parsing lives in the
/// core. Lowering the ISO-8601 text into these fields (locale-independently,
/// via std::from_chars) is the XML frontend's job (P4). The default value is
/// the Unix epoch, 1970-01-01T00:00:00.000 UTC.
struct DateTime {
    int year = 1970;
    int month = 1;              ///< 1..12.
    int day = 1;                ///< 1..days-in-month (leap years honored).
    int hour = 0;               ///< 0..23.
    int minute = 0;             ///< 0..59.
    int second = 0;             ///< 0..59.
    int millisecond = 0;        ///< 0..999.
    int utc_offset_minutes = 0; ///< RFC-822 zone as minutes east of UTC; ±14:00.

    /// True when every field is in range and the day exists in its month
    /// (28/29/30/31 per the Gregorian calendar, leap years included). The
    /// frontend and the engine reject an invalid DateTime before it can reach
    /// to_epoch_seconds().
    [[nodiscard]] bool valid() const;

    /// Seconds since the Unix epoch (1970-01-01T00:00:00 UTC), including the
    /// millisecond fraction and after normalizing the UTC offset. Pure civil
    /// arithmetic over C++20 std::chrono day counts — no locale, no wall
    /// clock, and a fixed IEEE expression so the result is bit-identical
    /// across platforms. Only meaningful when valid() is true.
    [[nodiscard]] double to_epoch_seconds() const;
};

} // namespace scena::ir
