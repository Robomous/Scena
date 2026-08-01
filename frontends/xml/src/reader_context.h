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
#include <filesystem>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <pugixml.hpp>

#include "scena/diagnostic.h"
#include "scena/status.h"
#include "scena/xml/document.h"

namespace scena::xml::detail {
class CatalogCache;
class ParameterScope;
} // namespace scena::xml::detail

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
    ReadContext(DiagnosticSink& sink, std::string_view source, std::string file);
    ~ReadContext();
    ReadContext(const ReadContext&) = delete;
    ReadContext& operator=(const ReadContext&) = delete;

    /// Records the revision the document declared, once FileHeader has been
    /// read. Element readers consult it for the constructs whose availability
    /// depends on the version (`version_at_least`).
    void set_version(DocumentVersion version) noexcept { version_ = version; }

    [[nodiscard]] DocumentVersion version() const noexcept { return version_; }

    /// True when the document declares revision `major.minor` or later.
    ///
    /// A document whose FileHeader has not been read yet (version 0.0)
    /// answers false for every real revision, so a version-gated check never
    /// fires before the version is known.
    [[nodiscard]] bool version_at_least(int major, int minor) const noexcept {
        return version_.rev_major > major ||
               (version_.rev_major == major && version_.rev_minor >= minor);
    }

    /// The next `ir::Action::bulk_group` id, counting from 1 (0 means "no bulk
    /// group"). One id per authored PrivateAction; every actor instance of that
    /// action shares it (§8.3.3.3). Ids depend only on document order, so a
    /// scenario loads to the same IR on every platform.
    [[nodiscard]] std::size_t next_bulk_group() noexcept { return ++bulk_group_; }

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

    /// The parameter declarations in scope at the element being read.
    ///
    /// The context owns the scope because attribute reading itself consults
    /// it: every attribute of every element may be a `$reference` or a
    /// `${expression}` (§9.2), so resolution belongs in the one place all
    /// readers funnel through rather than in each of them.
    [[nodiscard]] ParameterScope& parameters() noexcept { return *parameters_; }
    [[nodiscard]] const ParameterScope& parameters() const noexcept { return *parameters_; }

    /// The catalog directories the scenario declared and the entries loaded
    /// from them (§9.6). Lives on the context for the same reason the
    /// parameter scope does: a catalog reference can appear under almost any
    /// element, so every reader needs the same resolver.
    [[nodiscard]] CatalogCache& catalogs() noexcept { return *catalogs_; }

    /// Sets the directory relative catalog paths resolve against — the
    /// directory of the scenario file. Empty for a document loaded from
    /// memory, which then cannot resolve a relative catalog path.
    void set_base_directory(const std::filesystem::path& base);

    /// Marks the next `ParameterDeclarations` element as already applied.
    ///
    /// A catalog entry's declarations are read by `CatalogEntryScope`, which
    /// then overrides them with the reference's assignments. The element
    /// reader that runs next would otherwise read the same declarations into
    /// its own frame and shadow those assignments with the defaults, so the
    /// scope sets this flag and the declaration reader consumes it once.
    void set_declarations_applied() noexcept { declarations_applied_ = true; }
    [[nodiscard]] bool take_declarations_applied() noexcept {
        const bool applied = declarations_applied_;
        declarations_applied_ = false;
        return applied;
    }

    /// The entity ids of a named EntitySelection (§7.2.2.2), or nullptr when
    /// no selection has that name.
    [[nodiscard]] const std::vector<std::string>* entity_selection(std::string_view name) const;

    /// Records a selection's expanded members, in document order.
    void add_entity_selection(std::string name, std::vector<std::string> members);

    /// The names of every declared EntitySelection, for the validation pass
    /// (a selection is a legal target wherever an entity is).
    [[nodiscard]] std::vector<std::string> entity_selection_names() const;

    /// Records that an attribute referenced a parameter by name; the
    /// validation pass uses it to find declarations nothing refers to.
    void note_parameter_reference(std::string name);
    [[nodiscard]] const std::set<std::string, std::less<>>& referenced_parameters() const noexcept {
        return referenced_parameters_;
    }

private:
    void emit(Severity severity, Status code, std::string path, std::string message,
              std::string rule_id, LineColumn position);

    DiagnosticSink& sink_;
    std::unique_ptr<ParameterScope> parameters_;
    std::unique_ptr<CatalogCache> catalogs_;
    // Ordered map: selections are walked when expanding actors, and the
    // resulting IR order must not depend on a hash seed.
    std::map<std::string, std::vector<std::string>, std::less<>> selections_;
    std::set<std::string, std::less<>> referenced_parameters_;
    std::string_view source_;
    std::string file_;
    DocumentVersion version_;
    std::size_t bulk_group_ = 0;
    bool declarations_applied_ = false;
    Status first_error_ = Status::Ok;
};

} // namespace scena::xml::detail
