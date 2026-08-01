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

#include "read_common.h"

#include <cmath>
#include <cstddef>
#include <string>

namespace scena::xml::detail {

namespace {

/// Reads a double attribute that is known to be present.
bool read_present_double(ReadContext& ctx, const pugi::xml_node& node, const char* name,
                         const pugi::xml_attribute& attr, double& out) {
    const std::optional<double> value = parse_double(attr.value());
    if (!value.has_value()) {
        ctx.report_at(node, Severity::Error, Status::ParseError, attribute_path(node, name),
                      std::string("attribute '") + name + "' is not a number: '" + attr.value() +
                          "'");
        return false;
    }
    out = *value;
    return true;
}

} // namespace

bool require_double(ReadContext& ctx, const pugi::xml_node& node, const char* name, double& out) {
    const pugi::xml_attribute attr = node.attribute(name);
    if (!attr) {
        ctx.report_at(node, Severity::Error, Status::ValidationError, attribute_path(node, name),
                      std::string("missing required attribute '") + name + "'");
        return false;
    }
    return read_present_double(ctx, node, name, attr, out);
}

bool optional_double(ReadContext& ctx, const pugi::xml_node& node, const char* name, double& out) {
    const pugi::xml_attribute attr = node.attribute(name);
    if (!attr) {
        return true;
    }
    return read_present_double(ctx, node, name, attr, out);
}

bool optional_double(ReadContext& ctx, const pugi::xml_node& node, const char* name,
                     std::optional<double>& out) {
    const pugi::xml_attribute attr = node.attribute(name);
    if (!attr) {
        return true;
    }
    double value = 0.0;
    if (!read_present_double(ctx, node, name, attr, value)) {
        return false;
    }
    out = value;
    return true;
}

bool require_int(ReadContext& ctx, const pugi::xml_node& node, const char* name, int& out) {
    const pugi::xml_attribute attr = node.attribute(name);
    if (!attr) {
        ctx.report_at(node, Severity::Error, Status::ValidationError, attribute_path(node, name),
                      std::string("missing required attribute '") + name + "'");
        return false;
    }
    return optional_int(ctx, node, name, out);
}

bool optional_int(ReadContext& ctx, const pugi::xml_node& node, const char* name, int& out) {
    const pugi::xml_attribute attr = node.attribute(name);
    if (!attr) {
        return true;
    }
    const std::optional<long long> value = parse_integer(attr.value());
    if (!value.has_value()) {
        ctx.report_at(node, Severity::Error, Status::ParseError, attribute_path(node, name),
                      std::string("attribute '") + name + "' is not an integer: '" + attr.value() +
                          "'");
        return false;
    }
    // The IR stores counts as int so that an out-of-range value arriving from
    // a document is reported rather than wrapped around silently.
    if (*value < -2147483648LL || *value > 2147483647LL) {
        ctx.report_at(node, Severity::Error, Status::ValidationError, attribute_path(node, name),
                      std::string("attribute '") + name + "' is out of range");
        return false;
    }
    out = static_cast<int>(*value);
    return true;
}

bool optional_bool(ReadContext& ctx, const pugi::xml_node& node, const char* name, bool& out) {
    const pugi::xml_attribute attr = node.attribute(name);
    if (!attr) {
        return true;
    }
    const std::optional<bool> value = parse_boolean(attr.value());
    if (!value.has_value()) {
        ctx.report_at(node, Severity::Error, Status::ParseError, attribute_path(node, name),
                      std::string("attribute '") + name + "' is not a boolean: '" + attr.value() +
                          "'");
        return false;
    }
    out = *value;
    return true;
}

bool require_string(ReadContext& ctx, const pugi::xml_node& node, const char* name,
                    std::string& out) {
    const pugi::xml_attribute attr = node.attribute(name);
    if (!attr) {
        ctx.report_at(node, Severity::Error, Status::ValidationError, attribute_path(node, name),
                      std::string("missing required attribute '") + name + "'");
        return false;
    }
    out = attr.value();
    return true;
}

void optional_string(const pugi::xml_node& node, const char* name, std::string& out) {
    if (const pugi::xml_attribute attr = node.attribute(name)) {
        out = attr.value();
    }
}

void warn_unconsumed_children(ReadContext& ctx, const pugi::xml_node& node,
                              const char* const consumed[]) {
    for (pugi::xml_node child : node.children()) {
        if (child.type() != pugi::node_element) {
            continue;
        }
        bool known = false;
        for (const char* const* name = consumed; *name != nullptr; ++name) {
            if (child.name() == std::string_view(*name)) {
                known = true;
                break;
            }
        }
        if (!known) {
            ctx.report_at(child, Severity::Warning, Status::UnsupportedFeature, element_path(child),
                          std::string("element '") + child.name() +
                              "' is outside the loaded subset and is ignored");
        }
    }
}

void warn_deferred(ReadContext& ctx, const pugi::xml_node& node, std::string_view owner) {
    std::string message = std::string("element '") + node.name() + "' is not loaded yet (";
    message.append(owner);
    message += ")";
    ctx.report_at(node, Severity::Warning, Status::UnsupportedFeature, element_path(node),
                  std::move(message));
}

void warn_out_of_scope(ReadContext& ctx, const pugi::xml_node& node, std::string_view reason) {
    std::string message = std::string("element '") + node.name() + "' is not executed: ";
    message.append(reason);
    ctx.report_at(node, Severity::Warning, Status::UnsupportedFeature, element_path(node),
                  std::move(message));
}

void warn_deprecated(ReadContext& ctx, const pugi::xml_node& node, std::string_view successor) {
    std::string message = std::string("element '") + node.name() + "' is deprecated: ";
    message.append(successor);
    ctx.report_at(node, Severity::Warning, Status::DeprecatedFeature, element_path(node),
                  std::move(message));
}

pugi::xml_node read_choice(ReadContext& ctx, const pugi::xml_node& node,
                           const char* const candidates[]) {
    pugi::xml_node chosen;
    int matches = 0;
    for (pugi::xml_node child : node.children()) {
        if (child.type() != pugi::node_element) {
            continue;
        }
        for (const char* const* name = candidates; *name != nullptr; ++name) {
            if (child.name() == std::string_view(*name)) {
                ++matches;
                if (!chosen) {
                    chosen = child;
                }
                break;
            }
        }
    }
    if (matches == 0) {
        ctx.report_at(node, Severity::Error, Status::ValidationError, element_path(node),
                      std::string("element '") + node.name() +
                          "' declares none of its alternatives");
        return {};
    }
    if (matches > 1) {
        // An XSD choice admits exactly one alternative; keeping the first
        // keeps the rest of the document readable instead of cascading.
        ctx.report_at(node, Severity::Error, Status::ValidationError, element_path(node),
                      std::string("element '") + node.name() +
                          "' declares more than one alternative of a choice");
    }
    return chosen;
}

/// Parses a `dateTime` attribute (§DateTime, ISO 8601 basic notation) into the
/// kernel's civil DateTime. Locale-independent by construction: every field is
/// read with from_chars through parse_integer.
bool read_date_time(ReadContext& ctx, const pugi::xml_node& node, const char* name,
                    ir::DateTime& out) {
    std::string text;
    if (!require_string(ctx, node, name, text)) {
        return false;
    }
    const auto field = [&](std::size_t at, std::size_t count, int& target) {
        if (at + count > text.size()) {
            return false;
        }
        const std::optional<long long> value =
            parse_integer(std::string_view(text).substr(at, count));
        if (!value.has_value()) {
            return false;
        }
        target = static_cast<int>(*value);
        return true;
    };

    bool ok = text.size() >= 19 && text[4] == '-' && text[7] == '-' && text[10] == 'T' &&
              text[13] == ':' && text[16] == ':';
    ok = ok && field(0, 4, out.year) && field(5, 2, out.month) && field(8, 2, out.day);
    ok = ok && field(11, 2, out.hour) && field(14, 2, out.minute) && field(17, 2, out.second);

    std::size_t at = 19;
    if (ok && at < text.size() && text[at] == '.') {
        // Fractional seconds: the standard's pattern writes milliseconds
        // (FFF); more digits are truncated, fewer are scaled up, so ".5" is
        // 500 ms rather than 5.
        ++at;
        const std::size_t first = at;
        while (at < text.size() && text[at] >= '0' && text[at] <= '9') {
            ++at;
        }
        int digits = static_cast<int>(at - first);
        if (digits == 0) {
            ok = false;
        } else {
            int milliseconds = 0;
            ok =
                field(first, static_cast<std::size_t>(digits > 3 ? 3 : digits), milliseconds) && ok;
            for (; digits < 3; ++digits) {
                milliseconds *= 10;
            }
            out.millisecond = milliseconds;
        }
    }
    if (ok && at < text.size()) {
        if (text[at] == 'Z') {
            out.utc_offset_minutes = 0;
            ok = at + 1 == text.size();
        } else if (text[at] == '+' || text[at] == '-') {
            const int sign = text[at] == '-' ? -1 : 1;
            int hours = 0;
            int minutes = 0;
            const bool colon = at + 3 < text.size() && text[at + 3] == ':';
            ok = field(at + 1, 2, hours) && field(at + (colon ? 4 : 3), 2, minutes) && ok;
            out.utc_offset_minutes = sign * (hours * 60 + minutes);
        } else {
            ok = false;
        }
    }
    if (!ok || !out.valid()) {
        ctx.report_at(node, Severity::Error, Status::ValidationError, attribute_path(node, name),
                      std::string("attribute '") + name +
                          "' is not an ISO 8601 date and time in the pattern "
                          "yyyy-MM-ddThh:mm:ss[.fff][Z|±hh:mm]",
                      "asam.net:xosc:1.0.0:data_type.time_format");
        return false;
    }
    return true;
}

pugi::xml_node require_child(ReadContext& ctx, const pugi::xml_node& node, const char* name) {
    const pugi::xml_node child = node.child(name);
    if (!child) {
        ctx.report_at(node, Severity::Error, Status::ValidationError, element_path(node),
                      std::string("element '") + node.name() + "' has no required '" + name +
                          "' child");
    }
    return child;
}

} // namespace scena::xml::detail
