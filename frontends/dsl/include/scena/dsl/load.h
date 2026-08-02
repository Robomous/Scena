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
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "scena/diagnostic.h"
#include "scena/dsl/ast.h"
#include "scena/dsl/types.h"
#include "scena/status.h"

namespace scena::dsl {

/// How a load resolves imports.
struct LoadOptions {
    /// Directories searched for a module reference (§7.7.5.1.2). A reference
    /// `a.b.c` is looked up as `<path>/a/b/c.osc`, in order; the first hit
    /// wins. Standard-library references never reach the search paths.
    std::vector<std::filesystem::path> search_paths;

    /// Make the bundled standard library's types sub-module available without
    /// an import, and place `stdtypes` on the use list of the null namespace.
    ///
    /// §7.7.5.2 leaves it to the implementation whether the library arrives by
    /// importing files "or through any other means (for example, by providing
    /// access to built-in definitions)". Scena provides it built in, because
    /// the physical types of §8.14 are what give a literal like `30kph` a type
    /// at all. Turn this off to check a file in isolation.
    bool implicit_standard_library = true;
};

/// A parsed source file together with everything it imports.
///
/// Owns the ASTs, because a `Program` holds pointers into them: the result must
/// outlive every `Program` resolved from it.
class LoadResult {
public:
    /// The files in import order — a referenced file precedes the file that
    /// references it (§7.7.5.1), and the implicit standard library precedes
    /// them all.
    [[nodiscard]] const std::vector<const File*>& files() const noexcept { return order_; }

    /// The file the load was asked for, or nullptr when it did not parse.
    [[nodiscard]] const File* root() const noexcept { return root_; }

    /// Appends `file` to the load, taking ownership.
    const File* adopt(std::unique_ptr<File> file);

    /// Marks the most recently adopted file as the root.
    void set_root(const File* file) noexcept { root_ = file; }

private:
    std::vector<std::unique_ptr<File>> owned_;
    std::vector<const File*> order_;
    const File* root_ = nullptr;
};

/// Parses `source` and every file it imports, transitively (§7.7.5).
///
/// `origin` names the source for diagnostics and anchors relative imports; it
/// need not exist on disk (pass e.g. `<string>` for a source that does not).
///
/// A file referenced more than once, directly or transitively, is loaded once,
/// at the first place a depth-first traversal reaches it (§7.7.5.1). That rule
/// also makes an import cycle terminate rather than fail.
///
/// Like the parser, one call reports as much as it can: an import that cannot
/// be resolved costs that import, not the load.
///
/// Returns Status::Ok when nothing was reported as an error, and
/// Status::ValidationError otherwise.
[[nodiscard]] Status load_source(std::string_view source, const std::filesystem::path& origin,
                                 const LoadOptions& options, LoadResult& out, DiagnosticSink& sink);

/// Reads `path` and loads it as `load_source` does.
///
/// Returns Status::NotFound when the file cannot be read.
[[nodiscard]] Status load_file(const std::filesystem::path& path, const LoadOptions& options,
                               LoadResult& out, DiagnosticSink& sink);

/// Loads and resolves in one call — the entry point `scena-check`, the C ABI
/// and the Python bindings all sit on.
///
/// `out` is only usable while `loaded` is alive.
[[nodiscard]] Status check_source(std::string_view source, const std::filesystem::path& origin,
                                  const LoadOptions& options, LoadResult& loaded, Program& out,
                                  DiagnosticSink& sink);

/// Reads `path`, then checks it as `check_source` does.
[[nodiscard]] Status check_file(const std::filesystem::path& path, const LoadOptions& options,
                                LoadResult& loaded, Program& out, DiagnosticSink& sink);

} // namespace scena::dsl
