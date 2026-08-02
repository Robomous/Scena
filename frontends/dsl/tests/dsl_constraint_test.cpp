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
// Constraints and coverage (p7-s4, #42): §7.3.11's keep/remove_default with
// concrete-value satisfiability only, and §7.5's coverage constructs checked
// but not collected. Anything that would need search is diagnosed, not solved
// (ADR-0004).
//
// Every source fragment is written from the specification text (ADR-0002).
//

#include <string>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>

#include "scena/diagnostic.h"
#include "scena/dsl/parser.h"
#include "scena/dsl/resolve.h"
#include "scena/dsl/stdlib.h"
#include "scena/dsl/types.h"
#include "scena/status.h"

namespace {

using scena::DiagnosticSink;
using scena::Severity;
using scena::Status;
using scena::dsl::File;
using scena::dsl::Program;

/// The bundled types sub-module, parsed once, as a file of its own. The
/// standard library is a separate translation unit (§7.7.5.2), never text
/// pasted in front of the source under test.
const File& standard_types() {
    static const File parsed = [] {
        File file;
        DiagnosticSink sink;
        (void)scena::dsl::parse_source(
            scena::dsl::standard_module_source(scena::dsl::kStandardTypesModule),
            std::string(scena::dsl::kStandardTypesModule), file, sink);
        file.is_standard_library = true;
        return file;
    }();
    return parsed;
}

/// Resolves `file` together with the bundled types sub-module.
Status resolve_with_std(const File& file, Program& program, DiagnosticSink& sink) {
    const std::vector<const File*> files{&standard_types(), &file};
    return scena::dsl::resolve(files, program, sink);
}

constexpr std::string_view kPreamble = "enum side: [left, right]\n"
                                       "actor vehicle:\n"
                                       "    length: float\n"
                                       "    s: side\n"
                                       "scenario host:\n";

/// Resolves a `host` scenario carrying `members` and returns everything
/// reported.
std::vector<scena::Diagnostic> check(std::string_view members) {
    const std::string source = std::string(kPreamble) + std::string(members);
    DiagnosticSink sink;
    File file;
    (void)scena::dsl::parse_source(source, file, sink);
    Program program;
    (void)resolve_with_std(file, program, sink);
    return sink.diagnostics();
}

bool mentions(const std::vector<scena::Diagnostic>& diagnostics, std::string_view needle) {
    for (const scena::Diagnostic& diagnostic : diagnostics) {
        if (diagnostic.message.find(needle) != std::string::npos) {
            return true;
        }
    }
    return false;
}

bool has_error(const std::vector<scena::Diagnostic>& diagnostics) {
    for (const scena::Diagnostic& diagnostic : diagnostics) {
        if (diagnostic.severity == Severity::Error) {
            return true;
        }
    }
    return false;
}

/// True when something was reported as an unsupported feature rather than as an
/// error — the shape ADR-0004 asks for.
bool notes_unsupported(const std::vector<scena::Diagnostic>& diagnostics) {
    for (const scena::Diagnostic& diagnostic : diagnostics) {
        if (diagnostic.code == Status::UnsupportedFeature) {
            EXPECT_EQ(diagnostic.severity, Severity::Warning) << diagnostic.message;
            return true;
        }
    }
    return false;
}

// --- constraint typing (§7.3.11.1) -----------------------------------------

TEST(DslConstraintTest, AKeepIsABooleanExpression) {
    const std::vector<scena::Diagnostic> errors = check("    a: int\n    keep(a)\n");
    EXPECT_TRUE(mentions(errors, "§7.3.11.1"));
    EXPECT_TRUE(has_error(errors));
}

TEST(DslConstraintTest, AWellTypedKeepIsAccepted) {
    const std::vector<scena::Diagnostic> diagnostics = check("    a: int\n    keep(a == 5)\n");
    EXPECT_FALSE(has_error(diagnostics));
}

TEST(DslConstraintTest, AKeepOverAnUnknownNameIsReported) {
    const std::vector<scena::Diagnostic> errors = check("    keep(nowhere == 1)\n");
    EXPECT_TRUE(mentions(errors, "unknown name 'nowhere'"));
}

TEST(DslConstraintTest, EveryQualifierIsAccepted) {
    // §7.3.11.3's strengths, and §7.3.11's default form.
    const std::vector<scena::Diagnostic> diagnostics =
        check("    a: int\n    keep(hard a == 1)\n    b: int\n    keep(default b == 2)\n");
    EXPECT_FALSE(has_error(diagnostics));
}

TEST(DslConstraintTest, AParameterWithBlockCarriesConstraints) {
    const std::vector<scena::Diagnostic> diagnostics = check("    v: speed with:\n"
                                                             "        keep(it == 30.0kph)\n");
    EXPECT_FALSE(has_error(diagnostics));
}

// --- remove_default (§7.3.11) ----------------------------------------------

TEST(DslConstraintTest, RemoveDefaultNamesAParameter) {
    const std::vector<scena::Diagnostic> diagnostics =
        check("    a: int = 1\n    remove_default(a)\n");
    EXPECT_FALSE(has_error(diagnostics));
}

TEST(DslConstraintTest, RemoveDefaultOverSomethingElseIsReported) {
    const std::vector<scena::Diagnostic> errors = check("    remove_default(nowhere)\n");
    EXPECT_TRUE(mentions(errors, "§7.3.11"));
    EXPECT_TRUE(has_error(errors));
}

// --- concrete-value satisfiability (§7.3.11.3, ADR-0004) -------------------

TEST(DslConstraintTest, AConstantFalseConstraintIsAnError) {
    // §7.3.11.3: a hard constraint that cannot hold is an error, and an
    // unqualified keep is hard.
    const std::vector<scena::Diagnostic> errors = check("    keep(1 > 2)\n");
    EXPECT_TRUE(mentions(errors, "cannot be satisfied"));
    EXPECT_TRUE(has_error(errors));
}

TEST(DslConstraintTest, AnExplicitlyHardFalseConstraintIsAnError) {
    const std::vector<scena::Diagnostic> errors = check("    keep(hard 30.0kph < 10.0kph)\n");
    EXPECT_TRUE(mentions(errors, "§7.3.11.3"));
    EXPECT_TRUE(has_error(errors));
}

TEST(DslConstraintTest, AFalseDefaultConstraintIsOnlyAWarning) {
    // A `default` keep proposes a value rather than requiring one, so an
    // unsatisfiable one loses its effect instead of failing the file.
    const std::vector<scena::Diagnostic> diagnostics = check("    keep(default 1 > 2)\n");
    EXPECT_FALSE(has_error(diagnostics));
    EXPECT_TRUE(mentions(diagnostics, "will have no effect"));
}

TEST(DslConstraintTest, AConstantTrueConstraintIsAccepted) {
    const std::vector<scena::Diagnostic> diagnostics =
        check("    keep(2 > 1)\n    keep(36.0kph > 5.0mps)\n");
    EXPECT_FALSE(has_error(diagnostics));
    EXPECT_FALSE(notes_unsupported(diagnostics));
}

TEST(DslConstraintTest, AConcreteEqualityBindingIsAccepted) {
    // The one shape v0.0.1 resolves without search: parameter == constant.
    const std::vector<scena::Diagnostic> diagnostics =
        check("    v: speed\n    keep(v == 30.0kph)\n");
    EXPECT_FALSE(has_error(diagnostics));
    EXPECT_FALSE(notes_unsupported(diagnostics));
}

TEST(DslConstraintTest, AConjunctionOfBindingsIsAccepted) {
    const std::vector<scena::Diagnostic> diagnostics =
        check("    a: int\n    b: int\n    keep(a == 1 and b == 2)\n");
    EXPECT_FALSE(has_error(diagnostics));
    EXPECT_FALSE(notes_unsupported(diagnostics));
}

TEST(DslConstraintTest, AMembershipBindingIsAccepted) {
    const std::vector<scena::Diagnostic> diagnostics =
        check("    v: speed\n    keep(v in [10.0kph..30.0kph])\n");
    EXPECT_FALSE(has_error(diagnostics));
    EXPECT_FALSE(notes_unsupported(diagnostics));
}

TEST(DslConstraintTest, AnInequalityOverAParameterNeedsASolver) {
    // ADR-0004: anything that has to search a space is diagnosed, not solved.
    const std::vector<scena::Diagnostic> diagnostics = check("    a: int\n    keep(a > 5)\n");
    EXPECT_FALSE(has_error(diagnostics));
    EXPECT_TRUE(notes_unsupported(diagnostics));
    EXPECT_TRUE(mentions(diagnostics, "ADR-0004"));
}

TEST(DslConstraintTest, ARelationBetweenTwoParametersNeedsASolver) {
    const std::vector<scena::Diagnostic> diagnostics =
        check("    a: int\n    b: int\n    keep(a == b)\n");
    EXPECT_FALSE(has_error(diagnostics));
    EXPECT_TRUE(notes_unsupported(diagnostics));
}

TEST(DslConstraintTest, ADisjunctionNeedsASolver) {
    const std::vector<scena::Diagnostic> diagnostics =
        check("    a: int\n    keep(a == 1 or a == 2)\n");
    EXPECT_FALSE(has_error(diagnostics));
    EXPECT_TRUE(notes_unsupported(diagnostics));
}

TEST(DslConstraintTest, AnUnsupportedConstraintNamesItsSectionAndDecision) {
    const std::vector<scena::Diagnostic> diagnostics = check("    a: int\n    keep(a > 5)\n");
    EXPECT_TRUE(mentions(diagnostics, "§7.3.11"));
    for (const scena::Diagnostic& diagnostic : diagnostics) {
        // The DSL standard defines no rule ids; diagnostics cite sections.
        EXPECT_TRUE(diagnostic.rule_id.empty());
    }
}

TEST(DslConstraintTest, ConstraintsInsideAnInvocationAreChecked) {
    // §7.3.11.4: constraints inside a scenario invocation.
    const std::vector<scena::Diagnostic> errors = check("    do serial:\n"
                                                        "        drive() with:\n"
                                                        "            keep(1 > 2)\n");
    EXPECT_TRUE(mentions(errors, "cannot be satisfied"));
}

// --- coverage (§7.5) -------------------------------------------------------

TEST(DslConstraintTest, CoverageIsCheckedButNotCollected) {
    const std::vector<scena::Diagnostic> diagnostics =
        check("    a: int\n    cover(a)\n    record(a)\n");
    EXPECT_FALSE(has_error(diagnostics));
    EXPECT_TRUE(mentions(diagnostics, "not collected in v0.0.1"));
    EXPECT_TRUE(notes_unsupported(diagnostics));
}

TEST(DslConstraintTest, CoverageArgumentsAreTypeChecked) {
    const std::vector<scena::Diagnostic> errors = check("    a: int\n    cover(nowhere)\n");
    EXPECT_TRUE(mentions(errors, "unknown name 'nowhere'"));
    EXPECT_TRUE(has_error(errors));
}

TEST(DslConstraintTest, BothCoverAndRecordAreNamedInTheirNote) {
    const std::vector<scena::Diagnostic> cover = check("    a: int\n    cover(a)\n");
    EXPECT_TRUE(mentions(cover, "cover is checked"));
    const std::vector<scena::Diagnostic> record = check("    a: int\n    record(a)\n");
    EXPECT_TRUE(mentions(record, "record is checked"));
}

// --- sample() and every() (§7.3.10.4) --------------------------------------

TEST(DslConstraintTest, SampleIsCheckedButNotExecuted) {
    const std::vector<scena::Diagnostic> diagnostics =
        check("    n: float\n    event tick\n    var v: float = sample(n, @tick, 0.0)\n");
    EXPECT_TRUE(mentions(diagnostics, "sample() is checked"));
    EXPECT_TRUE(notes_unsupported(diagnostics));
}

TEST(DslConstraintTest, EveryIsCheckedButNotExecuted) {
    const std::vector<scena::Diagnostic> diagnostics = check("    event periodic is every(1.0s)\n");
    EXPECT_TRUE(mentions(diagnostics, "'every' is checked"));
    EXPECT_TRUE(notes_unsupported(diagnostics));
}

// --- event conditions (§7.3.10.4) ------------------------------------------

TEST(DslConstraintTest, AnEventConditionIsABooleanExpression) {
    const std::vector<scena::Diagnostic> errors = check("    n: float\n    event bad is n\n");
    EXPECT_TRUE(mentions(errors, "§7.3.10.4"));
    EXPECT_TRUE(has_error(errors));
}

TEST(DslConstraintTest, RiseAndFallTakeBooleanExpressions) {
    const std::vector<scena::Diagnostic> diagnostics =
        check("    n: float\n"
              "    event started is rise(n > 1.0)\n"
              "    event stopped is fall(n > 1.0)\n");
    EXPECT_FALSE(has_error(diagnostics));
}

TEST(DslConstraintTest, AnEventFormulaSeesItsParameters) {
    const std::vector<scena::Diagnostic> diagnostics =
        check("    event fast(at: float) is rise(at > 1.0)\n");
    EXPECT_FALSE(has_error(diagnostics));
}

// --- unsupported paths never crash ----------------------------------------

TEST(DslConstraintTest, EverySpecialFormIsReportedNotCrashed) {
    // The exit criterion: unsupported paths diagnose, never crash. Each of
    // these reaches a different not-implemented corner in one run.
    const std::vector<scena::Diagnostic> diagnostics =
        check("    a: int\n"
              "    event tick\n"
              "    var v: float = sample(n, @tick, 0.0)\n"
              "    event periodic is every(1.0s)\n"
              "    cover(a)\n"
              "    record(a)\n"
              "    keep(a > 5)\n");
    EXPECT_FALSE(has_error(diagnostics));
    int unsupported = 0;
    for (const scena::Diagnostic& diagnostic : diagnostics) {
        if (diagnostic.code == Status::UnsupportedFeature) {
            ++unsupported;
            EXPECT_GT(diagnostic.location.line, 0) << diagnostic.message;
        }
    }
    EXPECT_GE(unsupported, 5);
}

TEST(DslConstraintTest, CheckingIsDeterministicAcrossRepeats) {
    constexpr std::string_view kMembers = "    a: int\n"
                                          "    v: speed\n"
                                          "    keep(v == 30.0kph)\n"
                                          "    keep(a > 5)\n"
                                          "    cover(a)\n";
    const std::vector<scena::Diagnostic> first = check(kMembers);
    const std::vector<scena::Diagnostic> second = check(kMembers);
    ASSERT_EQ(first.size(), second.size());
    for (std::size_t i = 0; i < first.size(); ++i) {
        EXPECT_EQ(first[i].message, second[i].message);
        EXPECT_EQ(first[i].location.line, second[i].location.line);
        EXPECT_EQ(first[i].severity, second[i].severity);
    }
}

} // namespace
