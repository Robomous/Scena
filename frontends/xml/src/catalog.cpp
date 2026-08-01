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

#include "catalog.h"

#include <algorithm>
#include <fstream>
#include <iterator>
#include <system_error>
#include <utility>

#include "parameters.h"
#include "read_common.h"

namespace scena::xml::detail {

namespace {

constexpr const char* kRuleResolvability =
    "asam.net:xosc:1.0.0:reference_control.catalog_reference_resolvability";
constexpr const char* kRuleByDirectory =
    "asam.net:xosc:1.0.0:reference_control.catalogs_referenced_by_directory";

struct CatalogLocationElement {
    const char* element;
    CatalogKind kind;
};

/// The catalog location elements of §9.6, in the XSD's own order.
constexpr CatalogLocationElement kLocations[] = {
    {"VehicleCatalog", CatalogKind::Vehicle},
    {"ControllerCatalog", CatalogKind::Controller},
    {"PedestrianCatalog", CatalogKind::Pedestrian},
    {"MiscObjectCatalog", CatalogKind::MiscObject},
    {"EnvironmentCatalog", CatalogKind::Environment},
    {"ManeuverCatalog", CatalogKind::Maneuver},
    {"TrajectoryCatalog", CatalogKind::Trajectory},
    {"RouteCatalog", CatalogKind::Route},
};

} // namespace

void CatalogCache::set_directory(CatalogKind kind, std::string path) {
    directories_[kind].path = std::move(path);
}

bool CatalogCache::has_directory(CatalogKind kind) const {
    const auto found = directories_.find(kind);
    return found != directories_.end() && !found->second.path.empty();
}

void CatalogCache::scan(ReadContext& ctx, const pugi::xml_node& reference, CatalogKind kind,
                        Directory& directory) {
    directory.scanned = true;
    (void)kind;

    std::filesystem::path root(directory.path);
    if (root.is_relative()) {
        if (base_.empty()) {
            // A relative catalog directory is relative to the scenario file;
            // a document loaded from memory has no location to resolve it
            // against, so the reference cannot be resolved at all.
            ctx.report_at(reference, Severity::Error, Status::SemanticError,
                          element_path(reference),
                          "catalog directory '" + directory.path +
                              "' is relative, and the scenario has no file location to resolve "
                              "it against",
                          kRuleByDirectory);
            return;
        }
        root = base_ / root;
    }

    std::error_code error;
    if (!std::filesystem::is_directory(root, error)) {
        ctx.report_at(reference, Severity::Error, Status::SemanticError, element_path(reference),
                      "catalog directory '" + root.string() + "' does not exist", kRuleByDirectory);
        return;
    }

    // Enumeration order is unspecified and differs between filesystems, so
    // the file names are sorted before anything is parsed: two loads of the
    // same tree must see the same catalogs in the same order.
    std::vector<std::filesystem::path> files;
    for (const std::filesystem::directory_entry& entry :
         std::filesystem::directory_iterator(root, error)) {
        if (entry.is_regular_file(error)) {
            files.push_back(entry.path());
        }
    }
    std::sort(files.begin(), files.end(),
              [](const std::filesystem::path& lhs, const std::filesystem::path& rhs) {
                  return lhs.filename().string() < rhs.filename().string();
              });

    for (const std::filesystem::path& file : files) {
        std::ifstream stream(file, std::ios::binary);
        if (!stream) {
            continue;
        }
        const std::string text((std::istreambuf_iterator<char>(stream)),
                               std::istreambuf_iterator<char>());
        auto document = std::make_unique<pugi::xml_document>();
        const pugi::xml_parse_result result = document->load_buffer(
            text.data(), text.size(), pugi::parse_default, pugi::encoding_auto);
        if (!result) {
            ctx.report_at(reference, Severity::Warning, Status::ParseError, element_path(reference),
                          "catalog file '" + file.filename().string() +
                              "' is not well-formed XML and is skipped");
            continue;
        }
        const pugi::xml_node root_element = document->document_element();
        const pugi::xml_node catalog = root_element.child("Catalog");
        if (!catalog) {
            // Not a catalog file — a scenario may sit in the same directory.
            continue;
        }
        const std::string catalog_name = catalog.attribute("name").value();
        for (pugi::xml_node entry : catalog.children()) {
            if (entry.type() != pugi::node_element) {
                continue;
            }
            const std::string entry_name = entry.attribute("name").value();
            if (entry_name.empty()) {
                continue;
            }
            const auto key = std::make_pair(catalog_name, entry_name);
            const auto [position, inserted] = directory.entries.emplace(key, entry);
            if (!inserted) {
                // Deterministic by construction: the first file in sorted
                // order wins, and the duplicate is named.
                ctx.report_at(reference, Severity::Warning, Status::ValidationError,
                              element_path(reference),
                              "catalog '" + catalog_name + "' declares entry '" + entry_name +
                                  "' more than once; the first one in file-name order is used");
            }
        }
        directory.documents.push_back(std::move(document));
    }
}

pugi::xml_node CatalogCache::try_resolve(ReadContext& ctx, const pugi::xml_node& reference,
                                         CatalogKind kind, const char* element_name) {
    const auto found = directories_.find(kind);
    if (found == directories_.end() || found->second.path.empty()) {
        return {};
    }
    const pugi::xml_attribute catalog_name = reference.attribute("catalogName");
    const pugi::xml_attribute entry_name = reference.attribute("entryName");
    if (!catalog_name || !entry_name) {
        return {};
    }
    Directory& directory = found->second;
    if (!directory.scanned) {
        scan(ctx, reference, kind, directory);
    }
    const auto entry =
        directory.entries.find(std::make_pair(catalog_name.value(), entry_name.value()));
    if (entry == directory.entries.end() ||
        std::string_view(entry->second.name()) != element_name) {
        return {};
    }
    return entry->second;
}

pugi::xml_node CatalogCache::resolve(ReadContext& ctx, const pugi::xml_node& reference,
                                     CatalogKind kind, const char* element_name) {
    std::string catalog_name;
    std::string entry_name;
    if (!require_string(ctx, reference, "catalogName", catalog_name) ||
        !require_string(ctx, reference, "entryName", entry_name)) {
        return {};
    }

    const auto found = directories_.find(kind);
    if (found == directories_.end() || found->second.path.empty()) {
        ctx.report_at(reference, Severity::Error, Status::SemanticError, element_path(reference),
                      "the scenario declares no catalog directory for '" + catalog_name + "'",
                      kRuleByDirectory);
        return {};
    }
    Directory& directory = found->second;
    if (!directory.scanned) {
        scan(ctx, reference, kind, directory);
    }

    const auto entry = directory.entries.find(std::make_pair(catalog_name, entry_name));
    if (entry == directory.entries.end()) {
        ctx.report_at(reference, Severity::Error, Status::SemanticError, element_path(reference),
                      "catalog '" + catalog_name + "' has no entry '" + entry_name + "'",
                      kRuleResolvability);
        return {};
    }
    if (std::string_view(entry->second.name()) != element_name) {
        ctx.report_at(reference, Severity::Error, Status::ValidationError, element_path(reference),
                      "catalog entry '" + entry_name + "' is a '" + entry->second.name() +
                          "', not a '" + element_name + "'",
                      kRuleResolvability);
        return {};
    }
    return entry->second;
}

void read_catalog_locations(ReadContext& ctx, const pugi::xml_node& node, CatalogCache& out) {
    static const char* const kConsumed[] = {"VehicleCatalog",
                                            "ControllerCatalog",
                                            "PedestrianCatalog",
                                            "MiscObjectCatalog",
                                            "EnvironmentCatalog",
                                            "ManeuverCatalog",
                                            "TrajectoryCatalog",
                                            "RouteCatalog",
                                            "TrafficDistributionEntryCatalog",
                                            nullptr};
    warn_unconsumed_children(ctx, node, kConsumed);
    if (const pugi::xml_node traffic = node.child("TrafficDistributionEntryCatalog")) {
        // The traffic family is Post-v0.0.1 (coverage matrix), so its
        // catalog has nothing to instantiate.
        warn_out_of_scope(ctx, traffic, "the traffic family is Post-v0.0.1");
    }

    for (const CatalogLocationElement& location : kLocations) {
        const pugi::xml_node element = node.child(location.element);
        if (!element) {
            continue;
        }
        const pugi::xml_node directory = require_child(ctx, element, "Directory");
        if (!directory) {
            continue;
        }
        std::string path;
        if (require_string(ctx, directory, "path", path)) {
            out.set_directory(location.kind, std::move(path));
        }
    }
}

CatalogEntryScope::CatalogEntryScope(ReadContext& ctx, const pugi::xml_node& reference,
                                     const pugi::xml_node& entry)
    : ctx_(ctx) {
    // Assignment values are read in the *referencing* scope — the standard
    // notes that a ParameterAssignment value may itself reference a
    // parameter — so they are collected before the isolated frame is pushed.
    std::vector<std::pair<std::string, std::string>> assignments;
    if (const pugi::xml_node list = reference.child("ParameterAssignments")) {
        static const char* const kConsumed[] = {"ParameterAssignment", nullptr};
        warn_unconsumed_children(ctx, list, kConsumed);
        for (pugi::xml_node assignment : list.children("ParameterAssignment")) {
            std::string name;
            std::string value;
            if (require_string(ctx, assignment, "parameterRef", name) &&
                require_string(ctx, assignment, "value", value)) {
                assignments.emplace_back(std::move(name), std::move(value));
            }
        }
    }

    ctx_.parameters().push(/*isolated=*/true);
    // The entry's own declarations set the defaults (§9.5) ...
    if (const pugi::xml_node declarations = entry.child("ParameterDeclarations")) {
        read_parameter_declarations(ctx_, declarations);
    }
    // ... and the reference's assignments override them.
    for (auto& [name, value] : assignments) {
        const std::optional<Value> declared = ctx_.parameters().find(name);
        if (!declared.has_value()) {
            ctx_.report_at(reference, Severity::Warning, Status::SemanticError,
                           element_path(reference),
                           "catalog entry declares no parameter '" + name + "' to assign");
            continue;
        }
        // The assignment is text; it must be a value of the declared type,
        // exactly like the declaration's own value.
        const std::optional<Value> assigned = parse_typed(value, declared->type);
        if (!assigned.has_value()) {
            ctx_.report_at(
                reference, Severity::Error, Status::ValidationError, element_path(reference),
                "assigned value '" + value + "' does not match the declared type of '" + name + "'",
                "asam.net:xosc:1.0.0:parameters.parameter_declaration_parameter_type_"
                "inference");
            continue;
        }
        ctx_.parameters().declare(name, *assigned);
    }
    // The element reader about to run must not re-read the same
    // declarations into a nested frame.
    ctx_.set_declarations_applied();
}

CatalogEntryScope::~CatalogEntryScope() {
    ctx_.parameters().pop();
}

} // namespace scena::xml::detail
