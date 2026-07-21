// SPDX-License-Identifier: MIT
#include "scena/diagnostic.h"

#include <gtest/gtest.h>

#include "scena/engine.h"
#include "scena/ir/scenario.h"
#include "scena/status.h"

using scena::Diagnostic;
using scena::DiagnosticSink;
using scena::Engine;
using scena::Severity;
using scena::SourceLocation;
using scena::Status;

namespace {

Diagnostic make_diagnostic(Severity severity, std::string message) {
    Diagnostic diagnostic;
    diagnostic.severity = severity;
    diagnostic.code = Status::ValidationError;
    diagnostic.message = std::move(message);
    return diagnostic;
}

} // namespace

TEST(DiagnosticSinkTest, StartsEmpty) {
    const DiagnosticSink sink;
    EXPECT_TRUE(sink.diagnostics().empty());
    EXPECT_FALSE(sink.has_errors());
}

TEST(DiagnosticSinkTest, PreservesReportOrder) {
    DiagnosticSink sink;
    sink.report(make_diagnostic(Severity::Info, "first"));
    sink.report(make_diagnostic(Severity::Warning, "second"));
    sink.report(make_diagnostic(Severity::Error, "third"));

    ASSERT_EQ(sink.diagnostics().size(), 3U);
    EXPECT_EQ(sink.diagnostics()[0].message, "first");
    EXPECT_EQ(sink.diagnostics()[1].message, "second");
    EXPECT_EQ(sink.diagnostics()[2].message, "third");
}

TEST(DiagnosticSinkTest, HasErrorsOnlyForErrorSeverity) {
    DiagnosticSink sink;
    sink.report(make_diagnostic(Severity::Info, "info"));
    sink.report(make_diagnostic(Severity::Warning, "warning"));
    EXPECT_FALSE(sink.has_errors());
    sink.report(make_diagnostic(Severity::Error, "error"));
    EXPECT_TRUE(sink.has_errors());
}

TEST(DiagnosticSinkTest, ClearDropsEverything) {
    DiagnosticSink sink;
    sink.report(make_diagnostic(Severity::Error, "error"));
    ASSERT_FALSE(sink.diagnostics().empty());
    sink.clear();
    EXPECT_TRUE(sink.diagnostics().empty());
    EXPECT_FALSE(sink.has_errors());
}

TEST(DiagnosticSinkTest, DefaultsAreUnknownLocationAndNoRule) {
    const Diagnostic diagnostic;
    EXPECT_EQ(diagnostic.severity, Severity::Error);
    EXPECT_EQ(diagnostic.code, Status::Ok);
    EXPECT_TRUE(diagnostic.path.empty());
    EXPECT_TRUE(diagnostic.rule_id.empty());

    const SourceLocation& location = diagnostic.location;
    EXPECT_TRUE(location.file.empty());
    EXPECT_EQ(location.line, 0);
    EXPECT_EQ(location.column, 0);
}

TEST(DiagnosticsTest, EngineStartsWithNoDiagnostics) {
    const Engine engine;
    EXPECT_TRUE(engine.diagnostics().empty());
}

TEST(DiagnosticsTest, ValidScenarioProducesNoDiagnostics) {
    Engine engine;
    scena::ir::Scenario scenario;
    scenario.name = "empty";
    ASSERT_EQ(engine.init(std::move(scenario)), Status::Ok);
    EXPECT_TRUE(engine.diagnostics().empty());
    EXPECT_EQ(engine.close(), Status::Ok);
    EXPECT_TRUE(engine.diagnostics().empty());
}

TEST(DiagnosticsTest, ClearDiagnosticsIsAvailableToHosts) {
    Engine engine;
    engine.clear_diagnostics(); // no-op on a fresh engine, never a failure
    EXPECT_TRUE(engine.diagnostics().empty());
}
