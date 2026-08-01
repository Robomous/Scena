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

#include <cstddef>
#include <string>
#include <string_view>

#include <pugixml.hpp>

#include "scena/diagnostic.h"
#include "scena/status.h"

// Internal to the XML frontend: pugixml is a private dependency of the
// frontend target and must never appear in a public scena/xml header.
namespace scena::xml::detail {

/// Builds the xpath-ish element path a diagnostic is anchored to, e.g.
/// "/OpenSCENARIO/Storyboard/Story[2]/Act".
///
/// The path is absolute, '/'-joined and positional in the XPath sense: an
/// element gets a 1-based `[i]` predicate exactly when it has at least one
/// same-named sibling, so unambiguous elements stay readable. Building it
/// from the document tree (rather than from what the reader happens to know)
/// keeps the addressing identical for every diagnostic, whichever code path
/// emits it.
[[nodiscard]] std::string element_path(const pugi::xml_node& node);

/// Same path with a trailing "/@name" attribute step, the XPath spelling for
/// "this attribute of this element".
[[nodiscard]] std::string attribute_path(const pugi::xml_node& node, std::string_view attribute);

/// Maps a byte offset in the source buffer to a 1-based line and column.
///
/// Offsets come from pugixml (`xml_node::offset_debug`, `xml_parse_result::
/// offset`); both can be unavailable, in which case the offset is negative
/// and the location stays at the "unknown" 0/0 of scena::SourceLocation.
/// Columns count bytes, not code points: a byte column is what an editor's
/// "go to offset" reproduces without assuming an encoding.
struct LineColumn {
    int line = 0;
    int column = 0;
};
[[nodiscard]] LineColumn line_column_at(std::string_view source, std::ptrdiff_t offset);

/// Collects diagnostics for one document read and remembers the first error,
/// which becomes the Status the loader returns.
///
/// The kernel invariant this upholds: an Error diagnostic implies a non-Ok
/// Status, and warnings leave the status Ok. Findings accumulate in document
/// order and are never reordered or deduplicated, so two reads of the same
/// bytes produce element-wise identical diagnostics.
class ReadContext {
public:
    ReadContext(DiagnosticSink& sink, std::string_view source, std::string file)
        : sink_(sink), source_(source), file_(std::move(file)) {}

    /// Reports a finding anchored at `path` with no source position.
    void report(Severity severity, Status code, std::string path, std::string message,
                std::string rule_id = {});

    /// Reports a finding positioned at `node` (its start tag) and anchored at
    /// `path`, which is usually `element_path(node)` or an attribute path
    /// under it.
    void report_at(const pugi::xml_node& node, Severity severity, Status code, std::string path,
                   std::string message, std::string rule_id = {});

    /// Reports a finding at a raw source offset, for failures that happen
    /// before there is a tree to point at (malformed XML).
    void report_at_offset(std::ptrdiff_t offset, Severity severity, Status code, std::string path,
                          std::string message, std::string rule_id = {});

    /// Status of the first Error reported, or Status::Ok when none was.
    [[nodiscard]] Status first_error() const noexcept { return first_error_; }

    [[nodiscard]] const std::string& file() const noexcept { return file_; }

private:
    void emit(Severity severity, Status code, std::string path, std::string message,
              std::string rule_id, LineColumn position);

    DiagnosticSink& sink_;
    std::string_view source_;
    std::string file_;
    Status first_error_ = Status::Ok;
};

} // namespace scena::xml::detail
