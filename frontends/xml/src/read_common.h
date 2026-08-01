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

#include <initializer_list>
#include <optional>
#include <string>
#include <string_view>

#include <pugixml.hpp>

#include "reader_context.h"
#include "scena/ir/date_time.h"
#include "scena/xml/detail/attributes.h"

// Internal to the XML frontend: the attribute, enumeration and child-element
// helpers every element reader is written against. Keeping them in one place
// is what makes the readers uniform — same diagnostics, same paths, same
// never-silent behavior, whichever element is being read.
namespace scena::xml::detail {

/// Required double attribute. Reports a ParseError when missing or not a
/// number and returns false; `out` is untouched then.
[[nodiscard]] bool require_double(ReadContext& ctx, const pugi::xml_node& node, const char* name,
                                  double& out);

/// Optional double attribute: absent leaves `out` alone and succeeds;
/// present-but-not-a-number is a ParseError.
[[nodiscard]] bool optional_double(ReadContext& ctx, const pugi::xml_node& node, const char* name,
                                   double& out);

/// Optional double attribute into an optional: absent clears nothing and
/// succeeds.
[[nodiscard]] bool optional_double(ReadContext& ctx, const pugi::xml_node& node, const char* name,
                                   std::optional<double>& out);

/// Required integer attribute (XSD int/unsignedInt/unsignedShort).
[[nodiscard]] bool require_int(ReadContext& ctx, const pugi::xml_node& node, const char* name,
                               int& out);

/// Optional integer attribute.
[[nodiscard]] bool optional_int(ReadContext& ctx, const pugi::xml_node& node, const char* name,
                                int& out);

/// Optional boolean attribute (0/1/true/false).
[[nodiscard]] bool optional_bool(ReadContext& ctx, const pugi::xml_node& node, const char* name,
                                 bool& out);

/// Required string attribute; an empty value is accepted (the XSD allows it),
/// a missing one is a ValidationError.
[[nodiscard]] bool require_string(ReadContext& ctx, const pugi::xml_node& node, const char* name,
                                  std::string& out);

/// Optional string attribute: absent leaves `out` alone.
void optional_string(ReadContext& ctx, const pugi::xml_node& node, const char* name,
                     std::string& out);

/// The text of `name` on `node` after parameter resolution: a whole-token
/// `$reference` is replaced by the parameter's value and a `${expression}`
/// by its result (§9.1, §9.2), a literal passes through unchanged.
///
/// Returns nullopt when the attribute is absent, and reports plus returns
/// nullopt when it is present but its reference or expression is broken —
/// callers distinguish the two through `node.attribute(name)`.
[[nodiscard]] std::optional<std::string>
attribute_text(ReadContext& ctx, const pugi::xml_node& node, const char* name);

/// One enumeration literal and the value it maps to.
template <typename T> struct EnumEntry {
    const char* literal;
    T value;
};

/// Maps an enumeration attribute onto `out`.
///
/// Absent attribute: leaves `out` at its default and succeeds — every
/// enumeration attribute Scena reads either has an XSD default or a
/// documented Scena default. Unknown literal: ValidationError naming the
/// attribute and the offending text, and false.
template <typename T>
[[nodiscard]] bool read_enum(ReadContext& ctx, const pugi::xml_node& node, const char* name,
                             std::initializer_list<EnumEntry<T>> entries, T& out) {
    if (!node.attribute(name)) {
        return true;
    }
    const std::optional<std::string> text = attribute_text(ctx, node, name);
    if (!text.has_value()) {
        return false; // already reported: a broken reference or expression
    }
    for (const EnumEntry<T>& entry : entries) {
        if (*text == entry.literal) {
            out = entry.value;
            return true;
        }
    }
    ctx.report_at(node, Severity::Error, Status::ValidationError, attribute_path(node, name),
                  std::string("attribute '") + name + "' has the unknown value '" + *text + "'");
    return false;
}

/// Reports every child element of `node` that is not in `consumed` as a
/// Severity::Warning / Status::UnsupportedFeature finding — the never-silent
/// rule. `consumed` is a null-terminated array of element names this reader
/// understands at this level.
void warn_unconsumed_children(ReadContext& ctx, const pugi::xml_node& node,
                              const char* const consumed[]);

/// Reports one element as recognized but not implemented, naming the sprint
/// or issue that owns it. `owner` is free text such as "p4-s4" — every
/// deferral in the frontend names where it went.
void warn_deferred(ReadContext& ctx, const pugi::xml_node& node, std::string_view owner);

/// Reports a construct the coverage matrix marks Post-v0.0.1 or Excluded.
void warn_out_of_scope(ReadContext& ctx, const pugi::xml_node& node, std::string_view reason);

/// Reports a deprecated construct that is still loaded (Status
/// DeprecatedFeature, Severity Warning — the kernel's contract for
/// "executed, but the standard deprecated it").
void warn_deprecated(ReadContext& ctx, const pugi::xml_node& node, std::string_view successor);

/// Same, but only for a document that declares revision `major.minor` or
/// later — the revision that deprecated the construct.
///
/// A construct is not deprecated in a document written before its successor
/// existed: telling a 1.0 file to use a 1.2 element is noise, not guidance.
/// Everything stays accepted and executed either way; the version decides
/// only whether the author is told (§5 and the coverage matrix's
/// "accepted, deprecated" rows).
void warn_deprecated_since(ReadContext& ctx, const pugi::xml_node& node, int major, int minor,
                           std::string_view successor);

/// The single element child of `node` among `candidates` (a null-terminated
/// array), i.e. an XSD choice. Reports a ValidationError and returns an empty
/// node when there is none; reports and keeps the first when there are
/// several, so a malformed choice still yields a usable reading.
[[nodiscard]] pugi::xml_node read_choice(ReadContext& ctx, const pugi::xml_node& node,
                                         const char* const candidates[]);

/// Parses a `dateTime` attribute (§DateTime) into the kernel's civil
/// DateTime, in the ISO 8601 basic notation the standard prescribes
/// (yyyy-MM-ddThh:mm:ss[.fff][Z|±hh:mm], rule
/// asam.net:xosc:1.0.0:data_type.time_format). Every field is read with
/// from_chars, never with a locale-sensitive conversion.
[[nodiscard]] bool read_date_time(ReadContext& ctx, const pugi::xml_node& node, const char* name,
                                  ir::DateTime& out);

/// Required child element; reports a ValidationError when missing.
[[nodiscard]] pugi::xml_node require_child(ReadContext& ctx, const pugi::xml_node& node,
                                           const char* name);

} // namespace scena::xml::detail
