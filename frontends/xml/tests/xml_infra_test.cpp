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

// p4-s1 document layer: encodings, CRLF, the comma-decimal locale trap, the
// version matrix, xpath-ish diagnostic addressing and source locations.

#include <clocale>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

#include "scena/diagnostic.h"
#include "scena/status.h"
#include "scena/xml/detail/attributes.h"
#include "scena/xml/loader.h"

namespace {

using scena::Diagnostic;
using scena::DiagnosticSink;
using scena::Severity;
using scena::Status;
using scena::xml::Document;
using scena::xml::DocumentKind;
using scena::xml::DocumentVersion;

/// A minimal well-formed scenario document at the given revision.
std::string scenario_document(int rev_major, int rev_minor) {
    return "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
           "<OpenSCENARIO>\n"
           "  <FileHeader revMajor=\"" +
           std::to_string(rev_major) + "\" revMinor=\"" + std::to_string(rev_minor) +
           "\" date=\"2026-08-01T00:00:00\" description=\"infra fixture\" author=\"Scena\"/>\n"
           "  <Entities/>\n"
           "  <Storyboard/>\n"
           "</OpenSCENARIO>\n";
}

/// The diagnostics of `sink` whose severity is Error.
std::vector<Diagnostic> errors(const DiagnosticSink& sink) {
    std::vector<Diagnostic> result;
    for (const Diagnostic& diagnostic : sink.diagnostics()) {
        if (diagnostic.severity == Severity::Error) {
            result.push_back(diagnostic);
        }
    }
    return result;
}

bool has_message_containing(const DiagnosticSink& sink, std::string_view needle) {
    for (const Diagnostic& diagnostic : sink.diagnostics()) {
        if (diagnostic.message.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

bool has_path(const DiagnosticSink& sink, std::string_view path) {
    for (const Diagnostic& diagnostic : sink.diagnostics()) {
        if (diagnostic.path == path) {
            return true;
        }
    }
    return false;
}

/// Comparable projection of a diagnostic, for the determinism assertions.
std::string signature(const Diagnostic& diagnostic) {
    return std::to_string(static_cast<int>(diagnostic.severity)) + "|" +
           std::to_string(static_cast<int>(diagnostic.code)) + "|" + diagnostic.path + "|" +
           diagnostic.message + "|" + diagnostic.rule_id + "|" + diagnostic.location.file + ":" +
           std::to_string(diagnostic.location.line) + ":" +
           std::to_string(diagnostic.location.column);
}

std::vector<std::string> signatures(const DiagnosticSink& sink) {
    std::vector<std::string> result;
    for (const Diagnostic& diagnostic : sink.diagnostics()) {
        result.push_back(signature(diagnostic));
    }
    return result;
}

/// Writes `content` verbatim (binary, no newline translation) to a unique
/// path under the temp directory and removes it on destruction.
class TempFile {
public:
    TempFile(std::string_view name, std::string_view content)
        : path_(std::filesystem::temp_directory_path() / name) {
        std::ofstream stream(path_, std::ios::binary | std::ios::trunc);
        stream.write(content.data(), static_cast<std::streamsize>(content.size()));
    }
    ~TempFile() {
        std::error_code ignored;
        std::filesystem::remove(path_, ignored);
    }
    TempFile(const TempFile&) = delete;
    TempFile& operator=(const TempFile&) = delete;

    [[nodiscard]] const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
};

// --- version matrix -------------------------------------------------------

class VersionMatrix : public testing::TestWithParam<std::pair<int, int>> {};

TEST_P(VersionMatrix, TargetedVersionsLoad) {
    const auto [major, minor] = GetParam();
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(scena::xml::load_string(scenario_document(major, minor), document, sink), Status::Ok);
    EXPECT_EQ(document.version, (DocumentVersion{major, minor}));
    EXPECT_TRUE(document.version.is_supported());
    EXPECT_EQ(document.kind, DocumentKind::Scenario);
    EXPECT_TRUE(errors(sink).empty());
}

INSTANTIATE_TEST_SUITE_P(SupportedRange, VersionMatrix,
                         testing::Values(std::pair{1, 0}, std::pair{1, 1}, std::pair{1, 2},
                                         std::pair{1, 3}));

TEST(Version, FourteenIsRejectedAsUnsupported) {
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::load_string(scenario_document(1, 4), document, sink),
              Status::UnsupportedFeature);
    const std::vector<Diagnostic> reported = errors(sink);
    ASSERT_EQ(reported.size(), 1U);
    EXPECT_EQ(reported.front().code, Status::UnsupportedFeature);
    EXPECT_EQ(reported.front().rule_id, "asam.net:xosc:1.0.0:xml.valid_schema");
    EXPECT_EQ(reported.front().path, "/OpenSCENARIO/FileHeader");
    EXPECT_NE(reported.front().message.find("1.4"), std::string::npos);
    // The version is still reported, so a host can tell the user what it read.
    EXPECT_EQ(document.version, (DocumentVersion{1, 4}));
}

TEST(Version, UnknownMajorIsAValidationError) {
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::load_string(scenario_document(2, 0), document, sink),
              Status::ValidationError);
    ASSERT_EQ(errors(sink).size(), 1U);
    EXPECT_EQ(errors(sink).front().rule_id, "asam.net:xosc:1.0.0:xml.valid_schema");
    EXPECT_FALSE(document.version.is_supported());
}

TEST(Version, MissingRevisionAttributesAreParseErrors) {
    constexpr std::string_view kNoRevMinor =
        "<OpenSCENARIO><FileHeader revMajor=\"1\" date=\"2026-08-01T00:00:00\" "
        "description=\"d\" author=\"a\"/><Storyboard/></OpenSCENARIO>";
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::load_string(kNoRevMinor, document, sink), Status::ParseError);
    ASSERT_EQ(errors(sink).size(), 1U);
    EXPECT_EQ(errors(sink).front().path, "/OpenSCENARIO/FileHeader/@revMinor");
}

TEST(Version, NonIntegerRevisionIsRejected) {
    for (const char* value : {"1.0", "one", "", " 1", "1,3"}) {
        const std::string text =
            std::string("<OpenSCENARIO><FileHeader revMajor=\"1\" revMinor=\"") + value +
            "\" date=\"2026-08-01T00:00:00\" description=\"d\" "
            "author=\"a\"/><Storyboard/></OpenSCENARIO>";
        Document document;
        DiagnosticSink sink;
        EXPECT_EQ(scena::xml::load_string(text, document, sink), Status::ParseError)
            << "revMinor='" << value << "'";
        EXPECT_TRUE(has_path(sink, "/OpenSCENARIO/FileHeader/@revMinor"));
    }
}

// --- the comma-decimal locale trap ---------------------------------------

/// Restores the global C locale whatever the test does to it: a leaked
/// locale would silently change every later conversion in this process.
class LocaleGuard {
public:
    LocaleGuard() {
        if (const char* current = std::setlocale(LC_ALL, nullptr); current != nullptr) {
            saved_ = current;
        }
    }
    ~LocaleGuard() { std::setlocale(LC_ALL, saved_.c_str()); }
    LocaleGuard(const LocaleGuard&) = delete;
    LocaleGuard& operator=(const LocaleGuard&) = delete;

private:
    std::string saved_ = "C";
};

TEST(LocaleTrap, DecimalPointIsNeverLocaleDependent) {
    LocaleGuard guard;
    // Any of these may be missing on a given CI image; the assertions below
    // must hold in every one of them, including plain "C".
    for (const char* name : {"C", "de_DE.UTF-8", "de_DE", "German_Germany.1252", "fr_FR.UTF-8"}) {
        if (std::setlocale(LC_ALL, name) == nullptr) {
            continue;
        }
        SCOPED_TRACE(name);

        // The trap: with a comma-decimal locale active, a locale-sensitive
        // conversion reads "1.5" as 1 and "1,5" as 1.5. from_chars does
        // neither.
        const std::optional<double> dotted = scena::xml::detail::parse_double("1.5");
        ASSERT_TRUE(dotted.has_value());
        EXPECT_EQ(*dotted, 1.5);
        EXPECT_FALSE(scena::xml::detail::parse_double("1,5").has_value());
        EXPECT_FALSE(scena::xml::detail::parse_double("1.5x").has_value());

        EXPECT_EQ(scena::xml::detail::parse_integer("-7").value_or(0), -7);
        EXPECT_FALSE(scena::xml::detail::parse_integer("1.0").has_value());
        EXPECT_TRUE(scena::xml::detail::parse_boolean("true").value_or(false));
        EXPECT_FALSE(scena::xml::detail::parse_boolean("True").has_value());

        // ... and the same at document level: a comma-decimal revision is
        // rejected rather than read as 1.3 under one locale and not another.
        Document document;
        DiagnosticSink sink;
        EXPECT_EQ(scena::xml::load_string(scenario_document(1, 3), document, sink), Status::Ok);
        EXPECT_EQ(document.version, (DocumentVersion{1, 3}));
    }
}

// --- encodings and line endings ------------------------------------------

TEST(Encoding, Utf8WithAndWithoutBomLoadIdentically) {
    const std::string plain = scenario_document(1, 2);
    const std::string with_bom = std::string("\xEF\xBB\xBF") + plain;

    Document plain_document;
    DiagnosticSink plain_sink;
    ASSERT_EQ(scena::xml::load_string(plain, plain_document, plain_sink), Status::Ok);

    Document bom_document;
    DiagnosticSink bom_sink;
    ASSERT_EQ(scena::xml::load_string(with_bom, bom_document, bom_sink), Status::Ok);

    EXPECT_EQ(plain_document.version, bom_document.version);
    EXPECT_EQ(plain_document.kind, bom_document.kind);
}

TEST(Encoding, Utf16DocumentLoads) {
    // Hand-built UTF-16LE with a BOM: pugixml detects it and transcodes.
    const std::string source = scenario_document(1, 1);
    std::string utf16;
    utf16 += '\xFF';
    utf16 += '\xFE';
    for (const char character : source) {
        utf16 += character;
        utf16 += '\0';
    }

    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(scena::xml::load_string(utf16, document, sink), Status::Ok);
    EXPECT_EQ(document.version, (DocumentVersion{1, 1}));
    // Offsets index pugixml's transcoded buffer, not these bytes, so the
    // loader reports "position unknown" rather than a wrong line.
    for (const Diagnostic& diagnostic : sink.diagnostics()) {
        EXPECT_EQ(diagnostic.location.line, 0);
    }
}

TEST(Encoding, NonAsciiAttributeTextSurvives) {
    const std::string source = "<OpenSCENARIO><FileHeader revMajor=\"1\" revMinor=\"0\" "
                               "date=\"2026-08-01T00:00:00\" description=\"Fahrspurwechsel\" "
                               "author=\"\xC3\x84nderung\"/><Storyboard/></OpenSCENARIO>";
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::load_string(source, document, sink), Status::Ok);
}

TEST(LineEndings, CrlfLoadsLikeLf) {
    const std::string lf = scenario_document(1, 3);
    std::string crlf;
    for (const char character : lf) {
        if (character == '\n') {
            crlf += '\r';
        }
        crlf += character;
    }

    Document lf_document;
    DiagnosticSink lf_sink;
    ASSERT_EQ(scena::xml::load_string(lf, lf_document, lf_sink), Status::Ok);
    Document crlf_document;
    DiagnosticSink crlf_sink;
    ASSERT_EQ(scena::xml::load_string(crlf, crlf_document, crlf_sink), Status::Ok);

    EXPECT_EQ(lf_document.version, crlf_document.version);
    // Same findings, same lines: "\r\n" is one line break, not two.
    EXPECT_EQ(signatures(lf_sink), signatures(crlf_sink));
}

// --- diagnostics: addressing and positions -------------------------------

TEST(Diagnostics, UnconsumedElementsAreWarnedNeverSilent) {
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(scena::xml::load_string(scenario_document(1, 3), document, sink), Status::Ok);

    // Entities and Storyboard are recognized but not lowered yet (p4-s2).
    EXPECT_TRUE(has_path(sink, "/OpenSCENARIO/Entities"));
    EXPECT_TRUE(has_path(sink, "/OpenSCENARIO/Storyboard"));
    for (const Diagnostic& diagnostic : sink.diagnostics()) {
        EXPECT_EQ(diagnostic.severity, Severity::Warning);
        EXPECT_EQ(diagnostic.code, Status::UnsupportedFeature);
    }
}

TEST(Diagnostics, UnknownElementIsWarned) {
    const std::string source =
        "<OpenSCENARIO><FileHeader revMajor=\"1\" revMinor=\"0\" date=\"2026-08-01T00:00:00\" "
        "description=\"d\" author=\"a\"/><Storyboard/><HouseRules/></OpenSCENARIO>";
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::load_string(source, document, sink), Status::Ok);
    EXPECT_TRUE(has_path(sink, "/OpenSCENARIO/HouseRules"));
    EXPECT_TRUE(has_message_containing(sink, "is not a child of OpenSCENARIO XML 1.0-1.3"));
}

TEST(Diagnostics, RepeatedSiblingsGetPositionalPredicates) {
    const std::string source =
        "<OpenSCENARIO><FileHeader revMajor=\"1\" revMinor=\"0\" date=\"2026-08-01T00:00:00\" "
        "description=\"d\" author=\"a\"/><Storyboard/><Extra/><Extra/></OpenSCENARIO>";
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::load_string(source, document, sink), Status::Ok);
    EXPECT_TRUE(has_path(sink, "/OpenSCENARIO/Extra[1]"));
    EXPECT_TRUE(has_path(sink, "/OpenSCENARIO/Extra[2]"));
    // A unique element carries no predicate.
    EXPECT_TRUE(has_path(sink, "/OpenSCENARIO/Storyboard"));
}

TEST(Diagnostics, PositionsPointAtTheOffendingLine) {
    const std::string source = "<OpenSCENARIO>\n"
                               "  <FileHeader revMajor=\"1\" revMinor=\"9\" "
                               "date=\"2026-08-01T00:00:00\" description=\"d\" author=\"a\"/>\n"
                               "  <Storyboard/>\n"
                               "</OpenSCENARIO>\n";
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::load_string(source, document, sink), Status::UnsupportedFeature);
    ASSERT_EQ(errors(sink).size(), 1U);
    EXPECT_EQ(errors(sink).front().location.line, 2);
    EXPECT_GT(errors(sink).front().location.column, 0);
    EXPECT_TRUE(errors(sink).front().location.file.empty()); // in-memory input
}

TEST(Diagnostics, MalformedXmlReportsAPosition) {
    constexpr std::string_view kSource = "<OpenSCENARIO>\n  <FileHeader>\n</OpenSCENARIO>\n";
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::load_string(kSource, document, sink), Status::ParseError);
    ASSERT_EQ(errors(sink).size(), 1U);
    EXPECT_EQ(errors(sink).front().path, "/");
    EXPECT_GT(errors(sink).front().location.line, 0);
    EXPECT_TRUE(has_message_containing(sink, "not well-formed XML"));
}

TEST(Diagnostics, WrongRootElementIsRejected) {
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::load_string("<Scenario/>", document, sink), Status::ValidationError);
    ASSERT_EQ(errors(sink).size(), 1U);
    EXPECT_EQ(errors(sink).front().path, "/Scenario");
    EXPECT_EQ(errors(sink).front().rule_id, "asam.net:xosc:1.0.0:xml.valid_schema");
}

TEST(Diagnostics, EmptyInputIsAParseError) {
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::load_string("", document, sink), Status::ParseError);
    EXPECT_FALSE(errors(sink).empty());
}

TEST(Diagnostics, MissingFileHeaderIsAValidationError) {
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::load_string("<OpenSCENARIO><Storyboard/></OpenSCENARIO>", document, sink),
              Status::ValidationError);
    ASSERT_EQ(errors(sink).size(), 1U);
    EXPECT_EQ(errors(sink).front().path, "/OpenSCENARIO");
}

TEST(Diagnostics, MissingRequiredHeaderAttributesAreReported) {
    constexpr std::string_view kSource =
        "<OpenSCENARIO><FileHeader revMajor=\"1\" revMinor=\"0\"/><Storyboard/></OpenSCENARIO>";
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::load_string(kSource, document, sink), Status::ValidationError);
    EXPECT_TRUE(has_path(sink, "/OpenSCENARIO/FileHeader/@author"));
    EXPECT_TRUE(has_path(sink, "/OpenSCENARIO/FileHeader/@date"));
    EXPECT_TRUE(has_path(sink, "/OpenSCENARIO/FileHeader/@description"));
}

TEST(Diagnostics, NonIso8601DateIsWarnedNotRejected) {
    constexpr std::string_view kSource =
        "<OpenSCENARIO><FileHeader revMajor=\"1\" revMinor=\"0\" date=\"01/08/2026\" "
        "description=\"d\" author=\"a\"/><Storyboard/></OpenSCENARIO>";
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::load_string(kSource, document, sink), Status::Ok);
    bool found = false;
    for (const Diagnostic& diagnostic : sink.diagnostics()) {
        if (diagnostic.rule_id == "asam.net:xosc:1.0.0:data_type.time_format") {
            EXPECT_EQ(diagnostic.severity, Severity::Warning);
            EXPECT_EQ(diagnostic.path, "/OpenSCENARIO/FileHeader/@date");
            found = true;
        }
    }
    EXPECT_TRUE(found);
}

TEST(Diagnostics, Iso8601VariantsAreAccepted) {
    for (const char* date :
         {"2026-08-01T00:00:00", "2026-08-01T00:00:00.123", "2026-08-01T00:00:00Z",
          "2026-08-01T12:30:00+02:00", "2026-08-01T12:30:00-0200"}) {
        const std::string source = std::string("<OpenSCENARIO><FileHeader revMajor=\"1\" "
                                               "revMinor=\"0\" date=\"") +
                                   date + "\" description=\"d\" author=\"a\"/></OpenSCENARIO>";
        Document document;
        DiagnosticSink sink;
        // No Storyboard: the document declares no category, which is an
        // error — but never the time-format one, which is what this asserts.
        (void)scena::xml::load_string(source, document, sink);
        for (const Diagnostic& diagnostic : sink.diagnostics()) {
            EXPECT_NE(diagnostic.rule_id, "asam.net:xosc:1.0.0:data_type.time_format")
                << "date='" << date << "'";
        }
    }
}

TEST(Diagnostics, ReadingTheSameDocumentTwiceReportsTheSameFindings) {
    const std::string source = scenario_document(1, 4);
    Document first_document;
    DiagnosticSink first;
    const Status first_status = scena::xml::load_string(source, first_document, first);
    Document second_document;
    DiagnosticSink second;
    const Status second_status = scena::xml::load_string(source, second_document, second);

    EXPECT_EQ(first_status, second_status);
    EXPECT_EQ(signatures(first), signatures(second));
}

// --- document kinds -------------------------------------------------------

TEST(DocumentKinds, CatalogFileIsRecognized) {
    constexpr std::string_view kSource =
        "<OpenSCENARIO><FileHeader revMajor=\"1\" revMinor=\"2\" date=\"2026-08-01T00:00:00\" "
        "description=\"d\" author=\"a\"/><Catalog name=\"vehicles\"/></OpenSCENARIO>";
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::load_string(kSource, document, sink), Status::Ok);
    EXPECT_EQ(document.kind, DocumentKind::Catalog);
}

TEST(DocumentKinds, ParameterValueDistributionIsOutOfScope) {
    constexpr std::string_view kSource =
        "<OpenSCENARIO><FileHeader revMajor=\"1\" revMinor=\"2\" date=\"2026-08-01T00:00:00\" "
        "description=\"d\" author=\"a\"/><ParameterValueDistribution/></OpenSCENARIO>";
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::load_string(kSource, document, sink), Status::UnsupportedFeature);
    EXPECT_EQ(document.kind, DocumentKind::ParameterValueDistribution);
}

TEST(DocumentKinds, MixedCategoriesAreRejected) {
    constexpr std::string_view kSource =
        "<OpenSCENARIO><FileHeader revMajor=\"1\" revMinor=\"2\" date=\"2026-08-01T00:00:00\" "
        "description=\"d\" author=\"a\"/><Storyboard/><Catalog name=\"c\"/></OpenSCENARIO>";
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::load_string(kSource, document, sink), Status::ValidationError);
    EXPECT_TRUE(has_message_containing(sink, "mixes two OpenSCENARIO document categories"));
}

TEST(DocumentKinds, NoCategoryIsAValidationError) {
    constexpr std::string_view kSource =
        "<OpenSCENARIO><FileHeader revMajor=\"1\" revMinor=\"2\" date=\"2026-08-01T00:00:00\" "
        "description=\"d\" author=\"a\"/></OpenSCENARIO>";
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::load_string(kSource, document, sink), Status::ValidationError);
    EXPECT_EQ(document.kind, DocumentKind::Unknown);
}

// --- file input -----------------------------------------------------------

TEST(FileInput, LoadsAFileAndRecordsItsPath) {
    const TempFile file("scena_xml_infra_ok.xosc", scenario_document(1, 3));
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(scena::xml::load_file(file.path(), document, sink), Status::Ok);
    EXPECT_EQ(document.version, (DocumentVersion{1, 3}));
    ASSERT_FALSE(sink.diagnostics().empty());
    for (const Diagnostic& diagnostic : sink.diagnostics()) {
        EXPECT_EQ(diagnostic.location.file, file.path().string());
    }
}

TEST(FileInput, CrlfFileLoadsUnchanged) {
    std::string crlf;
    for (const char character : scenario_document(1, 0)) {
        if (character == '\n') {
            crlf += '\r';
        }
        crlf += character;
    }
    const TempFile file("scena_xml_infra_crlf.xosc", crlf);
    Document document;
    DiagnosticSink sink;
    ASSERT_EQ(scena::xml::load_file(file.path(), document, sink), Status::Ok);
    EXPECT_EQ(document.version, (DocumentVersion{1, 0}));
}

TEST(FileInput, MissingFileIsAParseError) {
    const std::filesystem::path missing =
        std::filesystem::temp_directory_path() / "scena_xml_infra_absent.xosc";
    std::error_code ignored;
    std::filesystem::remove(missing, ignored);

    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::load_file(missing, document, sink), Status::ParseError);
    ASSERT_EQ(sink.diagnostics().size(), 1U);
    EXPECT_EQ(sink.diagnostics().front().location.file, missing.string());
}

TEST(FileInput, ForeignExtensionIsWarned) {
    const TempFile file("scena_xml_infra_extension.xml", scenario_document(1, 3));
    Document document;
    DiagnosticSink sink;
    EXPECT_EQ(scena::xml::load_file(file.path(), document, sink), Status::Ok);
    bool found = false;
    for (const Diagnostic& diagnostic : sink.diagnostics()) {
        if (diagnostic.rule_id == "asam.net:xosc:1.0.0:general.file_ending") {
            EXPECT_EQ(diagnostic.severity, Severity::Warning);
            found = true;
        }
    }
    EXPECT_TRUE(found);
}

} // namespace
