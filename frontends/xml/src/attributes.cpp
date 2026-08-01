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

#include "scena/xml/detail/attributes.h"

#include <charconv>
#include <system_error>

#include "scena/ir/rule.h" // ir::parse_scalar: the kernel's from_chars reader

namespace scena::xml::detail {

std::optional<double> parse_double(std::string_view text) {
    return ir::parse_scalar(text);
}

std::optional<long long> parse_integer(std::string_view text) {
    // A single leading '+' is legal in the XSD lexical space but rejected by
    // std::from_chars, so it is consumed here — the same accommodation
    // ir::parse_scalar makes for doubles.
    if (!text.empty() && text.front() == '+') {
        text.remove_prefix(1);
    }
    if (text.empty()) {
        return std::nullopt;
    }
    long long value = 0;
    const char* const begin = text.data();
    const char* const end = begin + text.size();
    const std::from_chars_result result = std::from_chars(begin, end, value);
    if (result.ec != std::errc{} || result.ptr != end) {
        return std::nullopt;
    }
    return value;
}

std::optional<bool> parse_boolean(std::string_view text) {
    if (text == "true" || text == "1") {
        return true;
    }
    if (text == "false" || text == "0") {
        return false;
    }
    return std::nullopt;
}

} // namespace scena::xml::detail
