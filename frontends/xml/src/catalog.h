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

#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <pugixml.hpp>

#include "reader_context.h"

namespace scena::xml::detail {

/// The catalog kinds a scenario may reference (§9.6). Each has its own
/// directory declaration in `CatalogLocations` and its own entry element.
enum class CatalogKind {
    Vehicle,
    Controller,
    Pedestrian,
    MiscObject,
    Environment,
    Maneuver,
    Trajectory,
    Route,
};

/// Resolves `CatalogReference`s against the directories a scenario declares.
///
/// Loading is lazy and cached: a directory is scanned the first time a
/// reference needs it, every catalog file in it is parsed once, and the
/// parsed documents stay alive for as long as the cache does — a resolved
/// entry is a node inside one of them.
///
/// **Determinism.** `std::filesystem::directory_iterator` yields entries in
/// an unspecified order that differs between filesystems, so the scan sorts
/// file names before parsing. Two loads of the same tree therefore see the
/// same files in the same order, and a duplicate `(catalog, entry)` name
/// resolves to the same one every time — the first in sorted order, with a
/// warning naming the duplicate.
class CatalogCache {
public:
    /// `base` is the directory relative paths are resolved against: the
    /// directory of the scenario file, or empty when the document was loaded
    /// from memory and has no location on disk.
    explicit CatalogCache(std::filesystem::path base) : base_(std::move(base)) {}

    /// Records a `<...CatalogLocation><Directory path="..."/>` declaration.
    void set_directory(CatalogKind kind, std::string path);

    /// True when the scenario declared a directory for `kind`.
    [[nodiscard]] bool has_directory(CatalogKind kind) const;

    /// The entry element named by a `CatalogReference`, or an empty node
    /// after reporting why it could not be resolved (no directory declared,
    /// no such catalog, no such entry, a relative path with no base).
    ///
    /// `element_name` is the element the entry must be, e.g. "Vehicle" —
    /// a catalog of the right kind holding the wrong element is a defect,
    /// not a silent miss.
    [[nodiscard]] pugi::xml_node resolve(ReadContext& ctx, const pugi::xml_node& reference,
                                         CatalogKind kind, const char* element_name);

    /// Same, but silent when the entry simply is not in this catalog kind.
    /// A ScenarioObject's catalog reference does not say whether it names a
    /// vehicle, a pedestrian or a misc object, so the kinds are tried in
    /// turn and only the last failure is worth a diagnostic.
    [[nodiscard]] pugi::xml_node try_resolve(ReadContext& ctx, const pugi::xml_node& reference,
                                             CatalogKind kind, const char* element_name);

private:
    struct Directory {
        std::string path;
        bool scanned = false;
        /// Parsed catalog files, kept alive because resolved entries point
        /// into them. Sorted by file name (see the determinism note above).
        std::vector<std::unique_ptr<pugi::xml_document>> documents;
        /// (catalog name, entry name) to the entry element.
        std::map<std::pair<std::string, std::string>, pugi::xml_node> entries;
    };

    void scan(ReadContext& ctx, const pugi::xml_node& reference, CatalogKind kind,
              Directory& directory);

    std::filesystem::path base_;
    std::map<CatalogKind, Directory> directories_;
};

/// Reads a `CatalogLocations` element into `out` (§9.6). Unknown or
/// out-of-scope catalog locations are reported, never dropped.
void read_catalog_locations(ReadContext& ctx, const pugi::xml_node& node, CatalogCache& out);

/// Pushes the parameter frame a catalog entry is read in (§9.5): the entry's
/// own `ParameterDeclarations` provide the defaults and the reference's
/// `ParameterAssignments` override them.
///
/// The frame is *isolated*: "no other parameters may be referenced from
/// within the catalog", so the scenario's own declarations are deliberately
/// invisible inside an entry. Assignment values are evaluated in the
/// *referencing* scope, before the isolation begins — the standard notes a
/// ParameterAssignment value may itself reference a parameter.
class CatalogEntryScope {
public:
    CatalogEntryScope(ReadContext& ctx, const pugi::xml_node& reference,
                      const pugi::xml_node& entry);
    ~CatalogEntryScope();
    CatalogEntryScope(const CatalogEntryScope&) = delete;
    CatalogEntryScope& operator=(const CatalogEntryScope&) = delete;

private:
    ReadContext& ctx_;
};

} // namespace scena::xml::detail
