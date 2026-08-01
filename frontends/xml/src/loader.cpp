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

#include "scena/xml/loader.h"

#include <cstddef>
#include <fstream>
#include <iterator>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include <pugixml.hpp>

#include "reader_context.h"
#include "scena/xml/detail/attributes.h"

namespace scena::xml {

namespace {

using detail::attribute_path;
using detail::element_path;
using detail::ReadContext;

// ASAM checker rules this layer enforces (XML 1.4.0 annex C).
constexpr const char* kRuleValidSchema = "asam.net:xosc:1.0.0:xml.valid_schema";
constexpr const char* kRuleFileEnding = "asam.net:xosc:1.0.0:general.file_ending";
constexpr const char* kRuleTimeFormat = "asam.net:xosc:1.0.0:data_type.time_format";

constexpr const char* kRootElement = "OpenSCENARIO";

/// The children of OpenSCENARIO that select the document form (§9): the
/// ScenarioDefinition group members, plus the single elements of the
/// CatalogDefinition and ParameterValueDistributionDefinition groups.
struct DefinitionElement {
    const char* name;
    DocumentKind kind;
};
constexpr DefinitionElement kDefinitionElements[] = {
    {"ParameterDeclarations", DocumentKind::Scenario},
    {"VariableDeclarations", DocumentKind::Scenario},
    {"MonitorDeclarations", DocumentKind::Scenario},
    {"CatalogLocations", DocumentKind::Scenario},
    {"RoadNetwork", DocumentKind::Scenario},
    {"Entities", DocumentKind::Scenario},
    {"Storyboard", DocumentKind::Scenario},
    {"Catalog", DocumentKind::Catalog},
    {"ParameterValueDistribution", DocumentKind::ParameterValueDistribution},
};

/// True when `text` matches the ISO 8601 basic notation the standard
/// prescribes for FileHeader/@date: yyyy-MM-ddTHH:mm:ss, optionally followed
/// by fractional seconds and/or a zone designator
/// (rule asam.net:xosc:1.0.0:data_type.time_format).
///
/// The check is a shape check, not a calendar check: it rejects text that is
/// plainly not a timestamp without pretending to validate day-of-month
/// ranges, which the standard does not ask a loader to do.
bool is_iso8601_date_time(std::string_view text) {
    const auto digits = [&text](std::size_t at, std::size_t count) {
        if (at + count > text.size()) {
            return false;
        }
        for (std::size_t i = at; i < at + count; ++i) {
            if (text[i] < '0' || text[i] > '9') {
                return false;
            }
        }
        return true;
    };
    const auto literal = [&text](std::size_t at, char expected) {
        return at < text.size() && text[at] == expected;
    };

    if (text.size() < 19) {
        return false;
    }
    if (!digits(0, 4) || !literal(4, '-') || !digits(5, 2) || !literal(7, '-') || !digits(8, 2)) {
        return false;
    }
    if (!literal(10, 'T')) {
        return false;
    }
    if (!digits(11, 2) || !literal(13, ':') || !digits(14, 2) || !literal(16, ':') ||
        !digits(17, 2)) {
        return false;
    }

    std::size_t at = 19;
    if (literal(at, '.')) {
        ++at;
        const std::size_t first = at;
        while (at < text.size() && text[at] >= '0' && text[at] <= '9') {
            ++at;
        }
        if (at == first) {
            return false; // a '.' with no fractional digits
        }
    }
    if (at == text.size()) {
        return true; // local time, no zone designator
    }
    if (text[at] == 'Z') {
        return at + 1 == text.size();
    }
    if (text[at] == '+' || text[at] == '-') {
        // ±hh:mm or ±hhmm
        if (digits(at + 1, 2) && literal(at + 3, ':') && digits(at + 4, 2)) {
            return at + 6 == text.size();
        }
        return digits(at + 1, 4) && at + 5 == text.size();
    }
    return false;
}

/// Reads a required unsignedShort attribute of FileHeader.
bool read_revision(ReadContext& ctx, const pugi::xml_node& header, const char* name, int& out) {
    const pugi::xml_attribute attr = header.attribute(name);
    const std::string path = attribute_path(header, name);
    if (!attr) {
        ctx.report_at(header, Severity::Error, Status::ParseError, path,
                      std::string("missing required attribute '") + name + "' on FileHeader",
                      kRuleValidSchema);
        return false;
    }
    const std::optional<long long> value = detail::parse_integer(attr.value());
    if (!value.has_value()) {
        ctx.report_at(header, Severity::Error, Status::ParseError, path,
                      std::string("attribute '") + name + "' is not an integer revision number: '" +
                          attr.value() + "'",
                      kRuleValidSchema);
        return false;
    }
    // xsd:unsignedShort, documented range [0..inf[ in the FileHeader class.
    if (*value < 0 || *value > 65535) {
        ctx.report_at(header, Severity::Error, Status::ValidationError, path,
                      std::string("attribute '") + name + "' is outside the unsignedShort range",
                      kRuleValidSchema);
        return false;
    }
    out = static_cast<int>(*value);
    return true;
}

/// Required string attribute of FileHeader (author, date, description). The
/// XSD marks all three `use="required"`; an empty value is legal.
bool require_string(ReadContext& ctx, const pugi::xml_node& header, const char* name) {
    if (header.attribute(name)) {
        return true;
    }
    ctx.report_at(header, Severity::Error, Status::ValidationError, attribute_path(header, name),
                  std::string("missing required attribute '") + name + "' on FileHeader",
                  kRuleValidSchema);
    return false;
}

/// Reads FileHeader: the version, the required descriptive attributes, and a
/// never-silent warning for the children this layer does not consume yet.
bool read_file_header(ReadContext& ctx, const pugi::xml_node& root, Document& out) {
    const pugi::xml_node header = root.child("FileHeader");
    if (!header) {
        ctx.report_at(root, Severity::Error, Status::ValidationError, element_path(root),
                      "document has no FileHeader element", kRuleValidSchema);
        return false;
    }

    int rev_major = 0;
    int rev_minor = 0;
    const bool major_ok = read_revision(ctx, header, "revMajor", rev_major);
    const bool minor_ok = read_revision(ctx, header, "revMinor", rev_minor);
    if (!major_ok || !minor_ok) {
        return false;
    }
    out.version = DocumentVersion{rev_major, rev_minor};

    if (!out.version.is_supported()) {
        // 1.4 exists and is the version of the local reference copy, but
        // Scena targets 1.0-1.3: a 1.4 document may use constructs whose
        // semantics this engine never implemented, so it is rejected rather
        // than executed on a guess. A different major revision is not a
        // revision of this standard that the loader knows at all.
        const Status code = rev_major == 1 ? Status::UnsupportedFeature : Status::ValidationError;
        const std::string reason =
            rev_major == 1
                ? "is newer than the targeted range 1.0-1.3 and is not executed"
                : "is not a known OpenSCENARIO XML revision; this loader targets 1.0-1.3";
        ctx.report_at(header, Severity::Error, code, element_path(header),
                      "OpenSCENARIO version " + out.version.to_string() + " " + reason,
                      kRuleValidSchema);
        return false;
    }

    bool ok = require_string(ctx, header, "author");
    ok = require_string(ctx, header, "date") && ok;
    ok = require_string(ctx, header, "description") && ok;

    if (const pugi::xml_attribute date = header.attribute("date");
        date && !is_iso8601_date_time(date.value())) {
        // A warning, not an error: a malformed timestamp cannot change how
        // the scenario executes, and rejecting real-world files over their
        // header date would be hostile.
        ctx.report_at(header, Severity::Warning, Status::ValidationError,
                      attribute_path(header, "date"),
                      "FileHeader date is not in the ISO 8601 basic notation "
                      "yyyy-MM-ddThh:mm:ss the standard prescribes",
                      kRuleTimeFormat);
    }

    for (pugi::xml_node child : header.children()) {
        if (child.type() != pugi::node_element) {
            continue;
        }
        // License and Properties are both optional FileHeader children that
        // carry no execution semantics; they are named explicitly so the
        // message says "not consumed" rather than "unknown".
        const std::string_view name = child.name();
        const bool known = name == "License" || name == "Properties";
        ctx.report_at(child, Severity::Warning, Status::UnsupportedFeature, element_path(child),
                      known ? std::string("FileHeader element '") + child.name() +
                                  "' is not consumed by this loader"
                            : std::string("element '") + child.name() +
                                  "' is not a FileHeader child of OpenSCENARIO XML 1.0-1.3 and "
                                  "is ignored");
    }
    return ok;
}

/// Classifies the document by its definition children and warns about every
/// element the loader does not consume yet — the never-silent rule. The
/// entity and storyboard lowering that consumes them arrives with p4-s2.
void read_definition(ReadContext& ctx, const pugi::xml_node& root, Document& out) {
    for (pugi::xml_node child : root.children()) {
        if (child.type() != pugi::node_element) {
            continue;
        }
        const std::string_view name = child.name();
        if (name == "FileHeader") {
            continue;
        }
        DocumentKind kind = DocumentKind::Unknown;
        for (const DefinitionElement& element : kDefinitionElements) {
            if (name == element.name) {
                kind = element.kind;
                break;
            }
        }
        if (kind == DocumentKind::Unknown) {
            ctx.report_at(child, Severity::Warning, Status::UnsupportedFeature, element_path(child),
                          std::string("element '") + child.name() +
                              "' is not a child of OpenSCENARIO XML 1.0-1.3 and is ignored");
            continue;
        }
        if (out.kind == DocumentKind::Unknown) {
            out.kind = kind;
        } else if (out.kind != kind) {
            // OpenScenarioCategory is an XSD choice: a file is a scenario, a
            // catalog, or a parameter-value distribution, never two of them.
            ctx.report_at(child, Severity::Error, Status::ValidationError, element_path(child),
                          std::string("element '") + child.name() +
                              "' mixes two OpenSCENARIO document categories in one file",
                          kRuleValidSchema);
            continue;
        }
        if (kind == DocumentKind::ParameterValueDistribution) {
            // Stochastic parameter-value distribution files are outside the
            // v0.0.1 scope declared by the roadmap (P4 scope-out list).
            ctx.report_at(child, Severity::Error, Status::UnsupportedFeature, element_path(child),
                          "parameter value distribution documents are outside the supported scope");
            continue;
        }
        ctx.report_at(child, Severity::Warning, Status::UnsupportedFeature, element_path(child),
                      std::string("element '") + child.name() +
                          "' is not loaded into the scenario yet");
    }

    if (out.kind == DocumentKind::Unknown) {
        ctx.report_at(root, Severity::Error, Status::ValidationError, element_path(root),
                      "document declares neither a scenario, a catalog, nor a parameter value "
                      "distribution",
                      kRuleValidSchema);
    }
}

Status load_buffer(std::string_view xml, std::string file, Document& out, DiagnosticSink& sink) {
    out = Document{};

    pugi::xml_document doc;
    // encoding_auto honors a byte-order mark and the XML declaration, so
    // UTF-8 (with or without BOM) and UTF-16 documents all load. pugixml
    // transcodes non-UTF-8 input into its own buffer, and the node offsets it
    // then reports index that transcoded text rather than the caller's bytes;
    // in that case the source is withheld from the ReadContext so positions
    // read "unknown" instead of pointing at the wrong place.
    const pugi::xml_parse_result result =
        doc.load_buffer(xml.data(), xml.size(), pugi::parse_default, pugi::encoding_auto);
    const bool offsets_index_input = result.encoding == pugi::encoding_utf8;
    ReadContext ctx(sink, offsets_index_input ? xml : std::string_view{}, std::move(file));

    if (!result) {
        ctx.report_at_offset(result.offset, Severity::Error, Status::ParseError, "/",
                             std::string("document is not well-formed XML: ") +
                                 result.description());
        return ctx.first_error();
    }

    const pugi::xml_node root = doc.document_element();
    if (!root) {
        ctx.report(Severity::Error, Status::ParseError, "/", "document has no root element",
                   kRuleValidSchema);
        return ctx.first_error();
    }
    if (std::string_view(root.name()) != kRootElement) {
        ctx.report_at(root, Severity::Error, Status::ValidationError, element_path(root),
                      std::string("root element is '") + root.name() + "', expected '" +
                          kRootElement + "'",
                      kRuleValidSchema);
        return ctx.first_error();
    }

    if (!read_file_header(ctx, root, out)) {
        return ctx.first_error();
    }
    read_definition(ctx, root, out);
    return ctx.first_error();
}

} // namespace

Status load_string(std::string_view xml, Document& out, DiagnosticSink& sink) {
    return load_buffer(xml, std::string{}, out, sink);
}

Status load_file(const std::filesystem::path& path, Document& out, DiagnosticSink& sink) {
    out = Document{};
    const std::string file = path.string();

    // Binary mode: no newline translation, so a CRLF document reads as the
    // same bytes on every platform and the reported offsets describe the file
    // as it is stored.
    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
        Diagnostic diagnostic;
        diagnostic.severity = Severity::Error;
        diagnostic.code = Status::ParseError;
        diagnostic.message = "cannot open scenario file";
        diagnostic.path = "/";
        diagnostic.location.file = file;
        sink.report(std::move(diagnostic));
        return Status::ParseError;
    }
    const std::string text((std::istreambuf_iterator<char>(stream)),
                           std::istreambuf_iterator<char>());

    if (path.extension() != ".xosc") {
        Diagnostic diagnostic;
        diagnostic.severity = Severity::Warning;
        diagnostic.code = Status::ValidationError;
        diagnostic.message = "scenario file does not use the '.xosc' extension";
        diagnostic.path = "/";
        diagnostic.location.file = file;
        diagnostic.rule_id = kRuleFileEnding;
        sink.report(std::move(diagnostic));
    }

    return load_buffer(text, file, out, sink);
}

} // namespace scena::xml
