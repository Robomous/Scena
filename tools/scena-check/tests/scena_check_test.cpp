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
// scena-check's contract is what a script depends on — the exit code and the
// shape of a diagnostic line — so these drive the built binary as a subprocess
// rather than linking its internals (p7-s5, #43).
//

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include <gtest/gtest.h>

namespace {

namespace fs = std::filesystem;

/// Runs scena-check with `args`, returning its exit code. Output goes to a file
/// so a failing test can show what the tool said.
int run(const std::string& args, std::string& output) {
    const fs::path log = fs::temp_directory_path() / "scena_check_test.log";
    // Quoting the binary path: a build directory may contain spaces.
    std::string command =
        std::string("\"") + SCENA_CHECK_BINARY + "\" " + args + " > \"" + log.string() + "\" 2>&1";
#ifdef _WIN32
    // cmd.exe strips the outer pair of quotes from the command it is given, so
    // a command that *starts* with a quoted path loses it. Wrapping the whole
    // thing in one more pair is the documented way to keep both.
    command = "\"" + command + "\"";
#endif
    const int status = std::system(command.c_str());
    std::ifstream file(log);
    std::stringstream buffer;
    buffer << file.rdbuf();
    output = buffer.str();
#ifdef _WIN32
    return status;
#else
    // POSIX packs the exit status; WEXITSTATUS without <sys/wait.h>.
    return (status & 0x7f) == 0 ? ((status >> 8) & 0xff) : -1;
#endif
}

/// A self-cleaning temporary directory to write .osc files into.
class Tree {
public:
    Tree() {
        root_ = fs::temp_directory_path() /
                ("scena_check_" + std::to_string(reinterpret_cast<std::uintptr_t>(this)));
        std::error_code ignored;
        fs::remove_all(root_, ignored);
        fs::create_directories(root_);
    }
    ~Tree() {
        std::error_code ignored;
        fs::remove_all(root_, ignored);
    }
    Tree(const Tree&) = delete;
    Tree& operator=(const Tree&) = delete;

    fs::path write(const std::string& name, const std::string& contents) const {
        const fs::path path = root_ / name;
        fs::create_directories(path.parent_path());
        std::ofstream file(path, std::ios::binary);
        file << contents;
        return path;
    }

    [[nodiscard]] const fs::path& root() const { return root_; }

private:
    fs::path root_;
};

/// Wraps a path for the command line; a temp directory may contain spaces.
std::string quoted(const fs::path& path) {
    return "\"" + path.string() + "\"";
}

constexpr int kOk = 0;
constexpr int kUsage = 2;
constexpr int kCheckFailed = 3;
constexpr int kInputFailed = 4;

TEST(ScenaCheckTest, AValidFileChecksClean) {
    const Tree tree;
    const fs::path source = tree.write("ok.osc", "import osc.standard.all\n"
                                                 "namespace demo use std, stdtypes\n"
                                                 "scenario cut_in:\n"
                                                 "    ego: vehicle\n"
                                                 "    target: lane\n"
                                                 "    keep(target.lane_type == lane_type!driving)\n"
                                                 "    keep(target.width == 3.5m)\n");
    std::string output;
    EXPECT_EQ(run(quoted(source), output), kOk) << output;
    EXPECT_NE(output.find("ok,"), std::string::npos) << output;
}

TEST(ScenaCheckTest, TheStandardLibraryIsAvailableWithoutAnImport) {
    // §8.14's physical types are what give `30kph` a type at all, so the types
    // sub-module is loaded for every check (ADR-0029).
    const Tree tree;
    const fs::path source = tree.write("implicit.osc", "struct probe:\n"
                                                       "    v: speed\n"
                                                       "    keep(it.v == 30kph)\n");
    std::string output;
    EXPECT_EQ(run(quoted(source), output), kOk) << output;
}

TEST(ScenaCheckTest, NoStandardLibrarySuppressesIt) {
    const Tree tree;
    const fs::path source = tree.write("implicit.osc", "struct probe:\n"
                                                       "    v: speed\n");
    std::string output;
    EXPECT_EQ(run(quoted(source) + " --no-standard-library", output), kCheckFailed) << output;
    EXPECT_NE(output.find("speed"), std::string::npos) << output;
}

TEST(ScenaCheckTest, AnErrorExitsThreeAndCitesItsSection) {
    // DSL diagnostics cite a § in the message: the DSL standard defines no
    // `asam.net:` rule identifiers, so there is no rule id to print.
    const Tree tree;
    const fs::path source = tree.write("bad.osc", "struct probe:\n"
                                                  "    v: no_such_type\n");
    std::string output;
    EXPECT_EQ(run(quoted(source), output), kCheckFailed) << output;
    EXPECT_NE(output.find("error: "), std::string::npos) << output;
    EXPECT_NE(output.find('\xc2'), std::string::npos) << "expected a § reference: " << output;
    EXPECT_EQ(output.find('['), std::string::npos) << "no rule id belongs here: " << output;
}

TEST(ScenaCheckTest, ADiagnosticNamesTheFileAndPosition) {
    const Tree tree;
    const fs::path source = tree.write("located.osc", "struct probe:\n"
                                                      "    v: no_such_type\n");
    std::string output;
    EXPECT_EQ(run(quoted(source), output), kCheckFailed) << output;
    EXPECT_NE(output.find("located.osc:2:"), std::string::npos) << output;
}

TEST(ScenaCheckTest, AMissingFileExitsFourNotThree) {
    // The Status model draws the line between a defect in the content and the
    // host handing us something unusable; the exit codes follow it.
    const Tree tree;
    std::string output;
    EXPECT_EQ(run(quoted(tree.root() / "absent.osc"), output), kInputFailed) << output;
}

TEST(ScenaCheckTest, QuietPrintsNothingButStillExitsNonZero) {
    const Tree tree;
    const fs::path source = tree.write("bad.osc", "struct probe:\n"
                                                  "    v: no_such_type\n");
    std::string output;
    EXPECT_EQ(run(quoted(source) + " --quiet", output), kCheckFailed) << output;
    EXPECT_TRUE(output.empty()) << output;
}

TEST(ScenaCheckTest, StrictTurnsAWarningIntoAFailure) {
    // §7.7.4 reserves the `std`-prefixed namespaces for the standard, so this
    // warns. Without --strict a warning is still a clean exit.
    const Tree tree;
    const fs::path source = tree.write("warn.osc", "namespace stdthing\n"
                                                   "struct probe:\n"
                                                   "    v: int\n");
    std::string output;
    EXPECT_EQ(run(quoted(source), output), kOk) << output;
    EXPECT_NE(output.find("warning: "), std::string::npos) << output;

    std::string strict_output;
    EXPECT_EQ(run(quoted(source) + " --strict", strict_output), kCheckFailed) << strict_output;
}

TEST(ScenaCheckTest, ASearchPathResolvesAModuleImport) {
    // §7.7.5.1.2 maps `a.b.c` to `a/b/c.osc` under the configured search paths.
    const Tree tree;
    tree.write("lib/shared/types.osc", "struct helper:\n"
                                       "    v: int\n"
                                       "export *\n");
    const fs::path source = tree.write("main.osc", "import shared.types\n"
                                                   "struct probe:\n"
                                                   "    h: helper\n");
    std::string output;
    EXPECT_EQ(run(quoted(source), output), kCheckFailed)
        << "without -I the module must not resolve: " << output;

    std::string found;
    EXPECT_EQ(run(quoted(source) + " -I " + quoted(tree.root() / "lib"), found), kOk) << found;
}

TEST(ScenaCheckTest, SearchPathsAreTriedInTheOrderGiven) {
    const Tree tree;
    tree.write("first/shared/types.osc", "struct helper:\n"
                                         "    marker_first: int\n"
                                         "export *\n");
    tree.write("second/shared/types.osc", "struct helper:\n"
                                          "    marker_second: int\n"
                                          "export *\n");
    const fs::path source = tree.write("main.osc", "import shared.types\n"
                                                   "struct probe:\n"
                                                   "    h: helper\n"
                                                   "    keep(it.h.marker_first == 1)\n");
    std::string output;
    EXPECT_EQ(run(quoted(source) + " -I " + quoted(tree.root() / "first") + " -I " +
                      quoted(tree.root() / "second"),
                  output),
              kOk)
        << output;

    std::string reversed;
    EXPECT_EQ(run(quoted(source) + " -I " + quoted(tree.root() / "second") + " -I " +
                      quoted(tree.root() / "first"),
                  reversed),
              kCheckFailed)
        << "the second copy has no marker_first: " << reversed;
}

TEST(ScenaCheckTest, AMissingImportExitsThree) {
    // An unresolvable import is a defect in the content, not host misuse: the
    // file we were handed is readable, it just names something that is not
    // there.
    const Tree tree;
    const fs::path source = tree.write("main.osc", "import shared.absent\n");
    std::string output;
    EXPECT_EQ(run(quoted(source), output), kCheckFailed) << output;
}

TEST(ScenaCheckTest, HelpExitsZeroAndDescribesTheExitCodes) {
    std::string output;
    EXPECT_EQ(run("--help", output), kOk) << output;
    EXPECT_NE(output.find("exit codes:"), std::string::npos) << output;
    EXPECT_NE(output.find("--search-path"), std::string::npos) << output;
}

TEST(ScenaCheckTest, NoArgumentsExitsUsage) {
    std::string output;
    EXPECT_EQ(run("", output), kUsage) << output;
    EXPECT_NE(output.find("usage:"), std::string::npos) << output;
}

TEST(ScenaCheckTest, AnUnknownOptionExitsUsage) {
    const Tree tree;
    const fs::path source = tree.write("ok.osc", "struct probe:\n"
                                                 "    v: int\n");
    std::string output;
    EXPECT_EQ(run(quoted(source) + " --nonsense", output), kUsage) << output;
}

TEST(ScenaCheckTest, TwoSourceFilesExitUsage) {
    const Tree tree;
    const fs::path first = tree.write("a.osc", "struct a:\n    v: int\n");
    const fs::path second = tree.write("b.osc", "struct b:\n    v: int\n");
    std::string output;
    EXPECT_EQ(run(quoted(first) + " " + quoted(second), output), kUsage) << output;
}

TEST(ScenaCheckTest, ASearchPathWithoutADirectoryExitsUsage) {
    const Tree tree;
    const fs::path source = tree.write("ok.osc", "struct probe:\n    v: int\n");
    std::string output;
    EXPECT_EQ(run(quoted(source) + " -I", output), kUsage) << output;
}

TEST(ScenaCheckTest, EveryDiagnosticIsReportedNotJustTheFirst) {
    // Error recovery is contractual: a malformed member never costs its
    // siblings, so one run reports many diagnostics.
    const Tree tree;
    const fs::path source = tree.write("many.osc", "struct probe:\n"
                                                   "    a: no_such_type\n"
                                                   "    b: another_missing_type\n");
    std::string output;
    EXPECT_EQ(run(quoted(source), output), kCheckFailed) << output;
    EXPECT_NE(output.find("no_such_type"), std::string::npos) << output;
    EXPECT_NE(output.find("another_missing_type"), std::string::npos) << output;
}

TEST(ScenaCheckTest, CheckingIsDeterministic) {
    // Load time is inside the determinism contract, and that includes the
    // order diagnostics come out in.
    const Tree tree;
    const fs::path source = tree.write("many.osc", "struct probe:\n"
                                                   "    a: no_such_type\n"
                                                   "    b: another_missing_type\n");
    std::string first;
    std::string second;
    EXPECT_EQ(run(quoted(source), first), kCheckFailed);
    EXPECT_EQ(run(quoted(source), second), kCheckFailed);
    EXPECT_EQ(first, second);
}

} // namespace
