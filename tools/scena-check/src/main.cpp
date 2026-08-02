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

//
// scena-check — load an OpenSCENARIO DSL file, resolve its imports and report
// what the checker finds. A thin consumer of `scena::dsl::check_file`, with no
// language semantics of its own (p7-s5, #43).
//
// It does not execute anything: DSL execution is P8. A file that checks clean
// here is one the frontend understood, not one that will necessarily run.
//

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "scena/diagnostic.h"
#include "scena/dsl/load.h"
#include "scena/dsl/types.h"
#include "scena/status.h"

namespace {

/// Exit codes. Distinct per failure so a script can branch on them, and split
/// along the same line the Status model draws: a content defect in the input is
/// not the same thing as the host handing us something unusable.
enum ExitCode : int {
    kOk = 0,          ///< the file checked clean
    kUsage = 2,       ///< bad command line
    kCheckFailed = 3, ///< the DSL source has errors (or warnings under --strict)
    kInputFailed = 4, ///< the file could not be read, or an import went missing
};

struct Options {
    std::filesystem::path source;
    std::vector<std::filesystem::path> search_paths;
    bool no_standard_library = false;
    bool strict = false; ///< treat warnings as failures
    bool quiet = false;
};

void print_usage() {
    std::cout << R"(scena-check — check an OpenSCENARIO DSL file

usage: scena-check <source> [options]

  <source>                   an OpenSCENARIO DSL file (.osc)

options:
  -I, --search-path <dir>    directory to resolve module imports against;
                             repeatable, searched in the order given
  --no-standard-library      do not load the bundled osc.standard library
  --strict                   exit non-zero when there are warnings, not just
                             errors
  --quiet                    do not print diagnostics to stderr
  -h, --help                 this text

exit codes:
  0 ok   2 usage   3 the source did not check   4 the input could not be read

scena-check does not execute the scenario; it loads it, follows its imports and
reports what the checker finds.
)";
}

const char* severity_name(scena::Severity severity) {
    switch (severity) {
    case scena::Severity::Info:
        return "info";
    case scena::Severity::Warning:
        return "warning";
    case scena::Severity::Error:
        break;
    }
    return "error";
}

/// Same one-diagnostic-per-line shape scena-run prints, so a script that
/// already parses one tool's output can parse the other's.
void print_diagnostics(const std::vector<scena::Diagnostic>& diagnostics, bool quiet) {
    if (quiet) {
        return;
    }
    for (const scena::Diagnostic& diagnostic : diagnostics) {
        std::cerr << severity_name(diagnostic.severity) << ": ";
        if (!diagnostic.location.file.empty()) {
            std::cerr << diagnostic.location.file;
            if (diagnostic.location.line > 0) {
                std::cerr << ':' << diagnostic.location.line << ':' << diagnostic.location.column;
            }
            std::cerr << ": ";
        }
        if (!diagnostic.path.empty()) {
            std::cerr << diagnostic.path << ": ";
        }
        std::cerr << diagnostic.message;
        // The DSL standard defines no `asam.net:` rule identifiers, so a DSL
        // diagnostic cites a section in its message and leaves this empty. The
        // branch is here because the same Diagnostic type carries XML rule ids.
        if (!diagnostic.rule_id.empty()) {
            std::cerr << " [" << diagnostic.rule_id << ']';
        }
        std::cerr << '\n';
    }
}

bool parse_options(int argc, char** argv, Options& options, int& exit_code) {
    for (int index = 1; index < argc; ++index) {
        const std::string_view argument = argv[index];
        if (argument == "-h" || argument == "--help") {
            print_usage();
            exit_code = kOk;
            return false;
        }
        if (argument == "-I" || argument == "--search-path") {
            if (index + 1 >= argc) {
                std::cerr << "error: " << argument << " needs a directory\n";
                exit_code = kUsage;
                return false;
            }
            options.search_paths.emplace_back(argv[++index]);
            continue;
        }
        if (argument == "--no-standard-library") {
            options.no_standard_library = true;
            continue;
        }
        if (argument == "--strict") {
            options.strict = true;
            continue;
        }
        if (argument == "--quiet") {
            options.quiet = true;
            continue;
        }
        if (!argument.empty() && argument.front() == '-') {
            std::cerr << "error: unknown option '" << argument << "'\n";
            exit_code = kUsage;
            return false;
        }
        if (!options.source.empty()) {
            std::cerr << "error: more than one source file given\n";
            exit_code = kUsage;
            return false;
        }
        options.source = argument;
    }
    if (options.source.empty()) {
        print_usage();
        exit_code = kUsage;
        return false;
    }
    return true;
}

} // namespace

int main(int argc, char** argv) {
    Options options;
    int exit_code = kOk;
    if (!parse_options(argc, argv, options, exit_code)) {
        return exit_code;
    }

    scena::dsl::LoadOptions load_options;
    load_options.search_paths = options.search_paths;
    load_options.implicit_standard_library = !options.no_standard_library;

    scena::dsl::LoadResult loaded;
    scena::dsl::Program program;
    scena::DiagnosticSink sink;
    const scena::Status status =
        scena::dsl::check_file(options.source, load_options, loaded, program, sink);

    print_diagnostics(sink.diagnostics(), options.quiet);

    // `InvalidArgument` is the Status model's host-misuse code: the path was
    // not something we could read at all. Everything else that fails is a
    // defect in the content, which is the other kind of exit.
    if (status == scena::Status::InvalidArgument) {
        return kInputFailed;
    }
    if (status != scena::Status::Ok) {
        return kCheckFailed;
    }
    if (options.strict) {
        for (const scena::Diagnostic& diagnostic : sink.diagnostics()) {
            if (diagnostic.severity == scena::Severity::Warning) {
                return kCheckFailed;
            }
        }
    }
    if (!options.quiet) {
        std::cout << options.source.string() << ": ok, " << program.types.size() << " types across "
                  << loaded.files().size() << " files\n";
    }
    return kOk;
}
