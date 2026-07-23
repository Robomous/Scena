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

#include "scena/ir/date_time.h"

#include <chrono>

namespace scena::ir {

bool DateTime::valid() const {
    if (month < 1 || month > 12) {
        return false;
    }
    if (day < 1 || day > 31) {
        return false;
    }
    if (hour < 0 || hour > 23) {
        return false;
    }
    if (minute < 0 || minute > 59) {
        return false;
    }
    if (second < 0 || second > 59) {
        return false;
    }
    if (millisecond < 0 || millisecond > 999) {
        return false;
    }
    // RFC-822 zones span ±14:00 in practice; anything wider is malformed.
    if (utc_offset_minutes < -14 * 60 || utc_offset_minutes > 14 * 60) {
        return false;
    }
    // std::chrono::year_month_day::ok() is the authority on day-in-month,
    // including the Gregorian leap-year rule, so Feb 29 is accepted only in
    // leap years and Apr 31 never is.
    const std::chrono::year_month_day date{std::chrono::year{year},
                                           std::chrono::month{static_cast<unsigned>(month)},
                                           std::chrono::day{static_cast<unsigned>(day)}};
    return date.ok();
}

double DateTime::to_epoch_seconds() const {
    // Civil-day integer math: year_month_day -> sys_days gives the number of
    // days since 1970-01-01 as an exact integer (constexpr, no locale, no
    // wall clock). Seconds are accumulated in integers and only the
    // millisecond fraction touches floating point, in a fixed expression, so
    // the value is bit-identical everywhere.
    const std::chrono::year_month_day date{std::chrono::year{year},
                                           std::chrono::month{static_cast<unsigned>(month)},
                                           std::chrono::day{static_cast<unsigned>(day)}};
    const std::chrono::sys_days days{date};
    const long long day_count = days.time_since_epoch().count();
    const long long seconds_of_day =
        static_cast<long long>(hour) * 3600 + static_cast<long long>(minute) * 60 + second;
    // A local time east of UTC (positive offset) is ahead of UTC, so the
    // offset is subtracted to reach the epoch (UTC) instant.
    const long long utc_seconds =
        day_count * 86400 + seconds_of_day - static_cast<long long>(utc_offset_minutes) * 60;
    return static_cast<double>(utc_seconds) + static_cast<double>(millisecond) / 1000.0;
}

} // namespace scena::ir
