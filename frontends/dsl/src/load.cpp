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

#include "scena/dsl/load.h"

#include <fstream>
#include <set>
#include <sstream>
#include <utility>

#include "scena/dsl/parser.h"
#include "scena/dsl/resolve.h"
#include "scena/dsl/stdlib.h"

namespace scena::dsl {
namespace {

void report(DiagnosticSink& sink, Severity severity, Status code, const std::string& path,
            const SourceRange& at, std::string message) {
    Diagnostic diagnostic;
    diagnostic.severity = severity;
    diagnostic.code = code;
    diagnostic.message = std::move(message);
    diagnostic.location.file = path;
    diagnostic.location.line = at.line;
    diagnostic.location.column = at.column;
    sink.report(std::move(diagnostic));
}

/// Strips the `file` URI scheme from a string-literal import (§7.7.5.1.1).
///
/// Only `file` is supported. `file:///x/y` and `file:/x/y` both denote the path
/// `/x/y`; a `file://host/...` form with a non-empty authority is not a local
/// path and is rejected by the caller, which has the range to report it.
[[nodiscard]] bool strip_file_scheme(std::string& reference) {
    constexpr std::string_view kScheme = "file:";
    if (reference.rfind(kScheme, 0) != 0) {
        return true; // not a URI with a scheme; a bare relative or absolute path
    }
    reference.erase(0, kScheme.size());
    if (reference.rfind("//", 0) != 0) {
        return true; // file:/path
    }
    reference.erase(0, 2);
    // What is left starts with the authority. An empty authority (the
    // `file:///path` form) leaves a leading '/' — anything else is a host.
    return !reference.empty() && reference.front() == '/';
}

/// `a.b.c` → `a/b/c.osc`, the implementation-specific mapping §7.7.5.1.2 leaves
/// open. Returns false when a component is empty (`a..b`).
[[nodiscard]] bool module_to_relative_path(std::string_view reference, std::filesystem::path& out) {
    std::filesystem::path relative;
    std::size_t start = 0;
    while (start <= reference.size()) {
        const std::size_t dot = reference.find('.', start);
        const std::string_view component = reference.substr(
            start, dot == std::string_view::npos ? std::string_view::npos : dot - start);
        if (component.empty()) {
            return false;
        }
        relative /= std::filesystem::path(component);
        if (dot == std::string_view::npos) {
            break;
        }
        start = dot + 1;
    }
    out = relative.replace_extension(".osc");
    return true;
}

[[nodiscard]] bool read_file(const std::filesystem::path& path, std::string& out) {
    std::ifstream stream(path, std::ios::binary);
    if (!stream) {
        return false;
    }
    std::ostringstream buffer;
    buffer << stream.rdbuf();
    out = buffer.str();
    return true;
}

/// Resolves imports depth first, loading every referenced file before the file
/// that references it (§7.7.5.1).
class Loader {
public:
    Loader(const LoadOptions& options, LoadResult& out, DiagnosticSink& sink)
        : options_(options), out_(out), sink_(sink) {}

    /// Loads the bundled library, when the caller asked for it. Runs before the
    /// root so that its declarations precede every user declaration.
    void load_implicit_standard_library() {
        if (!options_.implicit_standard_library) {
            return;
        }
        for (const std::string_view sub : standard_submodules(kStandardTypesModule)) {
            load_standard_module(sub);
        }
    }

    const File* load(std::string_view source, const std::filesystem::path& origin) {
        const std::string key = "file:" + origin.generic_string();
        if (!seen_.insert(key).second) {
            return nullptr;
        }
        return parse_and_recurse(source, origin.generic_string(), origin.parent_path(), false);
    }

    [[nodiscard]] bool failed() const noexcept { return failed_; }

private:
    const File* parse_and_recurse(std::string_view source, const std::string& path,
                                  const std::filesystem::path& directory, bool is_standard) {
        auto file = std::make_unique<File>();
        file->path = path;
        file->is_standard_library = is_standard;
        if (parse_source(source, path, *file, sink_) != Status::Ok) {
            failed_ = true;
        }
        // The imports are followed before the file is adopted, so that a
        // referenced file lands ahead of the file that references it.
        for (const Declaration& declaration : file->declarations) {
            if (declaration.kind == Declaration::Kind::Import) {
                follow(declaration.import, path, directory);
            }
        }
        return out_.adopt(std::move(file));
    }

    void follow(const ImportDecl& import, const std::string& path,
                const std::filesystem::path& directory) {
        if (import.is_path) {
            follow_path(import, path, directory);
            return;
        }
        if (is_standard_module(import.reference)) {
            for (const std::string_view sub : standard_submodules(import.reference)) {
                load_standard_module(sub);
            }
            // §7.7.5.2.3's legacy form additionally auto-uses the library's
            // namespaces in the null namespace. Scena already places
            // `stdtypes` there for every file, so the legacy form asks for
            // nothing this release does not already do.
            (void)standard_module_auto_uses(import.reference);
            return;
        }
        if (is_reserved_module(import.reference)) {
            report(sink_, Severity::Error, Status::SemanticError, path, import.range,
                   "'" + import.reference +
                       "' is not a module of this standard; §7.7.5.1.2 reserves every reference "
                       "starting with 'osc' for ASAM OpenSCENARIO");
            failed_ = true;
            return;
        }
        std::filesystem::path relative;
        if (!module_to_relative_path(import.reference, relative)) {
            report(sink_, Severity::Error, Status::ValidationError, path, import.range,
                   "'" + import.reference + "' is not a well-formed module reference (§7.7.5.1.2)");
            failed_ = true;
            return;
        }
        for (const std::filesystem::path& search : options_.search_paths) {
            const std::filesystem::path candidate = search / relative;
            std::error_code error;
            if (std::filesystem::is_regular_file(candidate, error)) {
                load_from_disk(candidate, import, path);
                return;
            }
        }
        report(sink_, Severity::Error, Status::SemanticError, path, import.range,
               "module '" + import.reference + "' was not found on the search path (§7.7.5.1.2); " +
                   (options_.search_paths.empty()
                        ? "no search path is configured"
                        : "looked for '" + relative.generic_string() + "'"));
        failed_ = true;
    }

    void follow_path(const ImportDecl& import, const std::string& path,
                     const std::filesystem::path& directory) {
        std::string reference = import.reference;
        if (!strip_file_scheme(reference)) {
            report(sink_, Severity::Error, Status::InvalidArgument, path, import.range,
                   "'" + import.reference +
                       "' names a host; only local 'file' URIs are supported (§7.7.5.1.1)");
            failed_ = true;
            return;
        }
        std::filesystem::path target(reference);
        if (target.is_relative()) {
            // §7.7.5.1.1: "Relative URIs are resolved relative to the location
            // of the referencing file."
            target = directory / target;
        }
        load_from_disk(target, import, path);
    }

    void load_from_disk(const std::filesystem::path& target, const ImportDecl& import,
                        const std::string& path) {
        std::error_code error;
        std::filesystem::path canonical = std::filesystem::weakly_canonical(target, error);
        if (error) {
            canonical = target;
        }
        const std::string key = "file:" + canonical.generic_string();
        if (!seen_.insert(key).second) {
            return; // §7.7.5.1: a file referenced more than once is imported once
        }
        std::string source;
        if (!read_file(target, source)) {
            report(sink_, Severity::Error, Status::SemanticError, path, import.range,
                   "cannot read '" + target.generic_string() + "' (§7.7.5.1.1)");
            failed_ = true;
            return;
        }
        parse_and_recurse(source, canonical.generic_string(), canonical.parent_path(), false);
    }

    void load_standard_module(std::string_view module_reference) {
        const std::string key = "module:" + std::string(module_reference);
        if (!seen_.insert(key).second) {
            return;
        }
        const std::string_view source = standard_module_source(module_reference);
        if (source.empty()) {
            return;
        }
        parse_and_recurse(source, std::string(module_reference), std::filesystem::path(), true);
    }

    const LoadOptions& options_;
    LoadResult& out_;
    DiagnosticSink& sink_;
    std::set<std::string> seen_;
    bool failed_ = false;
};

} // namespace

const File* LoadResult::adopt(std::unique_ptr<File> file) {
    const File* view = file.get();
    owned_.push_back(std::move(file));
    order_.push_back(view);
    return view;
}

Status load_source(std::string_view source, const std::filesystem::path& origin,
                   const LoadOptions& options, LoadResult& out, DiagnosticSink& sink) {
    Loader loader(options, out, sink);
    loader.load_implicit_standard_library();
    const File* root = loader.load(source, origin);
    if (root != nullptr) {
        out.set_root(root);
    }
    return loader.failed() ? Status::ValidationError : Status::Ok;
}

Status load_file(const std::filesystem::path& path, const LoadOptions& options, LoadResult& out,
                 DiagnosticSink& sink) {
    std::string source;
    if (!read_file(path, source)) {
        Diagnostic diagnostic;
        diagnostic.code = Status::InvalidArgument;
        diagnostic.message = "cannot read '" + path.generic_string() + "'";
        diagnostic.location.file = path.generic_string();
        sink.report(std::move(diagnostic));
        return Status::InvalidArgument;
    }
    std::error_code error;
    std::filesystem::path canonical = std::filesystem::weakly_canonical(path, error);
    if (error) {
        canonical = path;
    }
    return load_source(source, canonical, options, out, sink);
}

Status check_source(std::string_view source, const std::filesystem::path& origin,
                    const LoadOptions& options, LoadResult& loaded, Program& out,
                    DiagnosticSink& sink) {
    const Status loading = load_source(source, origin, options, loaded, sink);
    const Status resolving = resolve(loaded.files(), out, sink);
    return loading != Status::Ok ? loading : resolving;
}

Status check_file(const std::filesystem::path& path, const LoadOptions& options, LoadResult& loaded,
                  Program& out, DiagnosticSink& sink) {
    const Status loading = load_file(path, options, loaded, sink);
    if (loading == Status::InvalidArgument) {
        return loading;
    }
    const Status resolving = resolve(loaded.files(), out, sink);
    return loading != Status::Ok ? loading : resolving;
}

} // namespace scena::dsl
