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
// The import mechanism (p7-s5, #43): §7.7.5's file and module imports, the
// import-once rule, and the standard-library references of §7.7.5.2.
//
// Every source fragment is written from the specification text (ADR-0002).
//

#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include <gtest/gtest.h>

#include "scena/diagnostic.h"
#include "scena/dsl/load.h"
#include "scena/dsl/stdlib.h"
#include "scena/dsl/types.h"
#include "scena/status.h"

namespace {

using scena::DiagnosticSink;
using scena::Severity;
using scena::Status;
using scena::dsl::LoadOptions;
using scena::dsl::LoadResult;
using scena::dsl::Program;

[[nodiscard]] bool mentions(const std::vector<scena::Diagnostic>& diagnostics,
                            std::string_view needle) {
    for (const scena::Diagnostic& diagnostic : diagnostics) {
        if (diagnostic.message.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

/// A directory of `.osc` files that removes itself.
class Tree {
public:
    Tree() {
        std::error_code error;
        root_ = std::filesystem::temp_directory_path(error) /
                std::filesystem::path("scena-dsl-import-test");
        std::filesystem::remove_all(root_, error);
        std::filesystem::create_directories(root_, error);
    }

    Tree(const Tree&) = delete;
    Tree& operator=(const Tree&) = delete;

    ~Tree() {
        std::error_code error;
        std::filesystem::remove_all(root_, error);
    }

    [[nodiscard]] const std::filesystem::path& root() const noexcept { return root_; }

    std::filesystem::path write(std::string_view relative, std::string_view contents) const {
        const std::filesystem::path target = root_ / std::filesystem::path(relative);
        std::error_code error;
        std::filesystem::create_directories(target.parent_path(), error);
        std::ofstream stream(target, std::ios::binary);
        stream << contents;
        return target;
    }

private:
    std::filesystem::path root_;
};

// --- the bundled library is available without an import ---------------------

TEST(DslImportTest, TheStandardTypesAreAvailableWithoutAnImport) {
    // §7.7.5.2 lets an implementation provide the library "by providing access
    // to built-in definitions"; Scena provides the types sub-module that way.
    DiagnosticSink sink;
    LoadResult loaded;
    Program program;
    ASSERT_EQ(scena::dsl::check_source("struct s:\n    v: speed = 30kph\n", "<test>", LoadOptions{},
                                       loaded, program, sink),
              Status::Ok);
    EXPECT_NE(program.find("stdtypes::speed"), nullptr);
    EXPECT_NE(program.units.find("kph"), program.units.end());
}

TEST(DslImportTest, TheStandardTypesCanBeSwitchedOff) {
    LoadOptions options;
    options.implicit_standard_library = false;
    DiagnosticSink sink;
    LoadResult loaded;
    Program program;
    EXPECT_EQ(scena::dsl::check_source("struct s:\n    v: speed = 30kph\n", "<test>", options,
                                       loaded, program, sink),
              Status::ValidationError);
    EXPECT_EQ(program.find("stdtypes::speed"), nullptr);
}

TEST(DslImportTest, AQualifiedStandardNameResolves) {
    // §7.7.4.1: an explicitly qualified identifier resolves in exactly one place.
    DiagnosticSink sink;
    LoadResult loaded;
    Program program;
    ASSERT_EQ(scena::dsl::check_source("struct s:\n    v: stdtypes::speed = 30kph\n", "<test>",
                                       LoadOptions{}, loaded, program, sink),
              Status::Ok);
    EXPECT_NE(program.find("stdtypes::speed"), nullptr);
}

TEST(DslImportTest, AUserDeclarationShadowsTheStandardOne) {
    // §7.7.4.2 rule 2: the current namespace wins over the use list.
    DiagnosticSink sink;
    LoadResult loaded;
    Program program;
    ASSERT_EQ(scena::dsl::check_source("struct speed:\n    v: float\n", "<test>", LoadOptions{},
                                       loaded, program, sink),
              Status::Ok);
    const scena::dsl::TypeInfo* shadowing = program.find("::speed");
    ASSERT_NE(shadowing, nullptr);
    EXPECT_EQ(shadowing->kind, scena::dsl::TypeKind::Struct);
}

// --- standard-library module references (§7.7.5.2) --------------------------

TEST(DslImportTest, EveryStandardModuleReferenceIsAccepted) {
    for (const std::string_view reference :
         {scena::dsl::kStandardTypesModule, scena::dsl::kStandardDomainModule,
          scena::dsl::kStandardAllModule, scena::dsl::kStandardLegacyModule}) {
        DiagnosticSink sink;
        LoadResult loaded;
        Program program;
        const std::string source = "import " + std::string(reference) + "\nstruct s\n";
        EXPECT_EQ(scena::dsl::check_source(source, "<test>", LoadOptions{}, loaded, program, sink),
                  Status::Ok)
            << reference;
        EXPECT_FALSE(sink.has_errors()) << reference;
    }
}

TEST(DslImportTest, ImportingTheStandardLibraryTwiceLoadsItOnce) {
    // §7.7.5.1: "a file that is referenced multiple times ... is only imported
    // once". The implicit types sub-module counts as the first reference, so
    // naming it again costs nothing and `osc.standard.all` adds only the
    // domain sub-module: two library files, however many references.
    DiagnosticSink sink;
    LoadResult loaded;
    Program program;
    ASSERT_EQ(scena::dsl::check_source("import osc.standard.types\nimport osc.standard.all\n"
                                       "import osc.standard.domain\nimport osc.standard\n"
                                       "struct s:\n    v: speed = 1mps\n",
                                       "<test>", LoadOptions{}, loaded, program, sink),
              Status::Ok);
    std::size_t standard_files = 0;
    for (const scena::dsl::File* file : loaded.files()) {
        standard_files += file->is_standard_library ? 1U : 0U;
    }
    EXPECT_EQ(standard_files, 2U);
}

TEST(DslImportTest, AnUnknownReservedModuleIsReported) {
    // §7.7.5.1.2 reserves every `osc`-prefixed reference for the standard, so
    // one this release does not know cannot be a user module.
    DiagnosticSink sink;
    LoadResult loaded;
    Program program;
    EXPECT_EQ(scena::dsl::check_source("import osc.standard.solver\nstruct s\n", "<test>",
                                       LoadOptions{}, loaded, program, sink),
              Status::ValidationError);
    EXPECT_TRUE(mentions(sink.diagnostics(), "is not a module of this standard"));
    EXPECT_TRUE(mentions(sink.diagnostics(), "§7.7.5.1.2"));
}

TEST(DslImportTest, AModuleThatIsNotOnTheSearchPathIsReported) {
    DiagnosticSink sink;
    LoadResult loaded;
    Program program;
    EXPECT_EQ(scena::dsl::check_source("import company.library\nstruct s\n", "<test>",
                                       LoadOptions{}, loaded, program, sink),
              Status::ValidationError);
    EXPECT_TRUE(mentions(sink.diagnostics(), "was not found on the search path"));
}

// --- file imports (§7.7.5.1.1) ----------------------------------------------

TEST(DslImportTest, ARelativeImportResolvesAgainstTheReferencingFile) {
    // §7.7.5.1.1: "Relative URIs are resolved relative to the location of the
    // referencing file."
    const Tree tree;
    tree.write("lib/units.osc", "struct wheel:\n    radius: length\n");
    const std::filesystem::path root = tree.write("scenario.osc", "import \"lib/units.osc\"\n"
                                                                  "struct car:\n"
                                                                  "    w: wheel\n");
    DiagnosticSink sink;
    LoadResult loaded;
    Program program;
    ASSERT_EQ(scena::dsl::check_file(root, LoadOptions{}, loaded, program, sink), Status::Ok)
        << (sink.diagnostics().empty() ? std::string() : sink.diagnostics().front().message);
    EXPECT_NE(program.find("::wheel"), nullptr);
    EXPECT_NE(program.find("::car"), nullptr);
}

TEST(DslImportTest, AReferencedFilePrecedesTheReferencingFile) {
    // §7.7.5.1: the referenced file's statements are treated as appearing
    // before the referencing file's.
    const Tree tree;
    tree.write("base.osc", "struct base\n");
    const std::filesystem::path root =
        tree.write("top.osc", "import \"base.osc\"\nstruct top inherits base\n");
    DiagnosticSink sink;
    LoadResult loaded;
    Program program;
    ASSERT_EQ(scena::dsl::check_file(root, LoadOptions{}, loaded, program, sink), Status::Ok);
    ASSERT_GE(loaded.files().size(), 3U); // library, base, top
    EXPECT_NE(loaded.files()[loaded.files().size() - 2]->path.find("base.osc"), std::string::npos);
    EXPECT_EQ(loaded.root(), loaded.files().back());
}

/// `p` as the path part of a `file` URI. §7.7.5.1.1 spells a Windows path
/// `/c:/Users/...`, so the drive letter is preceded by a slash; a POSIX path
/// already starts with one.
std::string uri_path(const std::filesystem::path& p) {
    const std::string text = p.generic_string();
    return text.empty() || text.front() == '/' ? text : "/" + text;
}

TEST(DslImportTest, AFileUriImportIsAccepted) {
    // §7.7.5.1.1 lists `file:///p`, `file:/p` and a bare `/p` as three
    // spellings of the same local path; all three must reach the same file.
    const Tree tree;
    const std::filesystem::path base = tree.write("uri-base.osc", "struct uri_base\n");
    const std::string path = uri_path(base);
    for (const std::string& reference : {"file://" + path, "file:" + path, path}) {
        const std::filesystem::path root =
            tree.write("uri-top.osc", "import \"" + reference + "\"\nstruct t\n");
        DiagnosticSink sink;
        LoadResult loaded;
        Program program;
        ASSERT_EQ(scena::dsl::check_file(root, LoadOptions{}, loaded, program, sink), Status::Ok)
            << reference << ": "
            << (sink.diagnostics().empty() ? std::string() : sink.diagnostics().front().message);
        EXPECT_NE(program.find("::uri_base"), nullptr) << reference;
    }
}

TEST(DslImportTest, AUriNamingAHostIsRejected) {
    // §7.7.5.1.1 requires only the `file` scheme, and a `file` URI with a
    // non-empty authority does not name a local path.
    const Tree tree;
    const std::filesystem::path root =
        tree.write("host.osc", "import \"file://example.invalid/shared.osc\"\nstruct t\n");
    DiagnosticSink sink;
    LoadResult loaded;
    Program program;
    EXPECT_EQ(scena::dsl::check_file(root, LoadOptions{}, loaded, program, sink),
              Status::ValidationError);
    EXPECT_TRUE(mentions(sink.diagnostics(), "names a host"));
}

TEST(DslImportTest, ADiamondImportsTheSharedFileOnce) {
    // §7.7.5.1: the import happens at the first place a depth-first traversal
    // reaches the file; later references import nothing further. Without that
    // rule `shared` would be declared twice and the resolve would fail.
    const Tree tree;
    tree.write("shared.osc", "struct shared\n");
    tree.write("left.osc", "import \"shared.osc\"\nstruct left\n");
    tree.write("right.osc", "import \"shared.osc\"\nstruct right_side\n");
    const std::filesystem::path root =
        tree.write("diamond.osc", "import \"left.osc\"\nimport \"right.osc\"\nstruct diamond\n");
    DiagnosticSink sink;
    LoadResult loaded;
    Program program;
    ASSERT_EQ(scena::dsl::check_file(root, LoadOptions{}, loaded, program, sink), Status::Ok)
        << (sink.diagnostics().empty() ? std::string() : sink.diagnostics().front().message);
    EXPECT_NE(program.find("::shared"), nullptr);
    EXPECT_EQ(loaded.files().size(), 5U); // library, shared, left, right, diamond
}

TEST(DslImportTest, AnImportCycleTerminates) {
    // Nothing special is needed: the import-once rule of §7.7.5.1 makes the
    // second reference a no-op, so a cycle is not an error.
    const Tree tree;
    tree.write("a.osc", "import \"b.osc\"\nstruct a_type\n");
    tree.write("b.osc", "import \"a.osc\"\nstruct b_type\n");
    DiagnosticSink sink;
    LoadResult loaded;
    Program program;
    ASSERT_EQ(scena::dsl::check_file(tree.root() / "a.osc", LoadOptions{}, loaded, program, sink),
              Status::Ok)
        << (sink.diagnostics().empty() ? std::string() : sink.diagnostics().front().message);
    EXPECT_NE(program.find("::a_type"), nullptr);
    EXPECT_NE(program.find("::b_type"), nullptr);
}

TEST(DslImportTest, AMissingImportedFileIsReported) {
    const Tree tree;
    const std::filesystem::path root = tree.write("missing.osc", "import \"absent.osc\"\n");
    DiagnosticSink sink;
    LoadResult loaded;
    Program program;
    EXPECT_EQ(scena::dsl::check_file(root, LoadOptions{}, loaded, program, sink),
              Status::ValidationError);
    EXPECT_TRUE(mentions(sink.diagnostics(), "cannot read"));
    EXPECT_TRUE(mentions(sink.diagnostics(), "§7.7.5.1.1"));
}

TEST(DslImportTest, AnUnreadableRootIsHostMisuse) {
    DiagnosticSink sink;
    LoadResult loaded;
    Program program;
    EXPECT_EQ(
        scena::dsl::check_file("does-not-exist-anywhere.osc", LoadOptions{}, loaded, program, sink),
        Status::InvalidArgument);
}

// --- module imports resolved on the search path (§7.7.5.1.2) ----------------

TEST(DslImportTest, AModuleReferenceMapsToAPathOnTheSearchPath) {
    const Tree tree;
    tree.write("company/library.osc", "struct company_type\n");
    LoadOptions options;
    options.search_paths.push_back(tree.root());
    DiagnosticSink sink;
    LoadResult loaded;
    Program program;
    ASSERT_EQ(scena::dsl::check_source("import company.library\nstruct s:\n    c: company_type\n",
                                       "<test>", options, loaded, program, sink),
              Status::Ok)
        << (sink.diagnostics().empty() ? std::string() : sink.diagnostics().front().message);
    EXPECT_NE(program.find("::company_type"), nullptr);
}

TEST(DslImportTest, AMalformedModuleReferenceIsReported) {
    DiagnosticSink sink;
    LoadResult loaded;
    Program program;
    EXPECT_EQ(scena::dsl::check_source("import company..library\nstruct s\n", "<test>",
                                       LoadOptions{}, loaded, program, sink),
              Status::ValidationError);
    EXPECT_TRUE(sink.has_errors());
}

// --- diagnostics ------------------------------------------------------------

TEST(DslImportTest, ImportDiagnosticsCiteSectionsAndCarryTheFile) {
    DiagnosticSink sink;
    LoadResult loaded;
    Program program;
    EXPECT_EQ(scena::dsl::check_source("import osc.nowhere\nstruct s\n", "scenario.osc",
                                       LoadOptions{}, loaded, program, sink),
              Status::ValidationError);
    ASSERT_FALSE(sink.diagnostics().empty());
    const scena::Diagnostic& first = sink.diagnostics().front();
    EXPECT_EQ(first.severity, Severity::Error);
    // The DSL standard defines no rule ids; diagnostics cite sections.
    EXPECT_TRUE(first.rule_id.empty());
    EXPECT_EQ(first.location.file, "scenario.osc");
    EXPECT_EQ(first.location.line, 1);
}

TEST(DslImportTest, LoadingIsDeterministic) {
    const Tree tree;
    tree.write("d1.osc", "struct one\n");
    tree.write("d2.osc", "struct two\n");
    const std::filesystem::path root =
        tree.write("det.osc", "import \"d1.osc\"\nimport \"d2.osc\"\nstruct three\n");
    std::vector<std::string> first;
    std::vector<std::string> second;
    for (std::vector<std::string>* order : {&first, &second}) {
        DiagnosticSink sink;
        LoadResult loaded;
        Program program;
        ASSERT_EQ(scena::dsl::check_file(root, LoadOptions{}, loaded, program, sink), Status::Ok);
        for (const scena::dsl::File* file : loaded.files()) {
            order->push_back(file->path);
        }
    }
    EXPECT_EQ(first, second);
}

TEST(DslImportTest, ADiagnosticNamesTheFileItCameFrom) {
    // A Program spans every file its root imported, so a line number on its own
    // does not locate anything. Both the resolver's diagnostics and the ones
    // expression typing reports must carry the file they came from — without it
    // `scena-check` can print a line number but not say which file it is a line
    // of (p7-s5, #43).
    const Tree tree;
    tree.write("helper.osc", "struct helper:\n"
                             "    v: no_such_type\n"
                             "export *\n");
    const auto root = tree.write("main.osc", "import \"helper.osc\"\n"
                                             "struct probe:\n"
                                             "    w: another_missing_type\n"
                                             "    keep(it.w == unknown_name)\n");
    DiagnosticSink sink;
    LoadResult loaded;
    Program program;
    EXPECT_EQ(scena::dsl::check_file(root, LoadOptions{}, loaded, program, sink),
              Status::ValidationError);
    bool named_helper = false;
    bool named_root = false;
    for (const scena::Diagnostic& diagnostic : sink.diagnostics()) {
        EXPECT_FALSE(diagnostic.location.file.empty()) << diagnostic.message;
        named_helper =
            named_helper || diagnostic.location.file.find("helper.osc") != std::string::npos;
        named_root = named_root || diagnostic.location.file.find("main.osc") != std::string::npos;
    }
    EXPECT_TRUE(named_helper) << "the imported file's error must name the imported file";
    EXPECT_TRUE(named_root) << "the root's error must name the root";
}

} // namespace
