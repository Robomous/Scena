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

#include <string>
#include <vector>

#include "scena/diagnostic.h"
#include "scena/dsl/load.h"
#include "scena/dsl/types.h"
#include "scena/ir/scenario.h"
#include "scena/status.h"

namespace scena::dsl {

/// How a checked DSL program becomes a Scenario IR (ADR-0030).
struct LowerOptions {
    /// The scenario to instantiate, by qualified name (`demo::overtake`) or by
    /// the name as written (`overtake`).
    ///
    /// §7.7.2 leaves entry-point selection entirely to the implementation.
    /// Empty means "the only top-level scenario the root file declares" — a
    /// file with exactly one is the common case and naming it again adds
    /// nothing; a file with several is reported rather than guessed at, because
    /// picking one silently would make the run depend on declaration order.
    std::string entry_point;

    /// The `one_of` alternative to run, by its label (§7.6.2.1.3).
    ///
    /// The standard says at least one alternative must hold and says nothing
    /// about which, so an executor picks. Picking at random would put a hidden
    /// input in the run, which is what the determinism contract exists to
    /// prevent — so the choice is an explicit input, and empty means the first
    /// alternative in declaration order.
    std::string alternative;
};

/// What lowering produced: the IR plus the host-side references that are not
/// kernel state.
///
/// The shape mirrors `xml::Document` deliberately. A road-network file is an
/// input to the *host*, not to the engine — the engine reaches roads only
/// through `IRoadQuery` (ADR-0003) — so it travels beside the IR rather than
/// inside it, exactly as `xml::RoadNetwork` does.
struct LowerResult {
    ir::Scenario scenario;
    /// §8.5.4's `map_file`, verbatim and unresolved: the string the scenario
    /// wrote, whether through `map.set_map_file("m.xodr")` (Code 61) or through
    /// a `keep` on a declared `map` field (Code 62). Interpreting it relative to
    /// the scenario file is the host's decision, the same rule the XML side
    /// states for `LogicFile`.
    std::string map_file;
};

/// The scenarios a root file offers as entry points, in declaration order.
///
/// What a CLI prints when the choice is ambiguous, and what an editor would
/// offer. Qualified names, so each one can be passed back as
/// `LowerOptions::entry_point` unambiguously.
[[nodiscard]] std::vector<std::string> entry_points(const Program& program,
                                                    const LoadResult& loaded);

/// Lowers a checked program to the Scenario IR.
///
/// Only **attribute-level concrete** scenarios lower (§6.3.1.2.1): every value
/// the IR needs must be fixed by an equality constraint or a field default.
/// Anything that would need search is reported as an
/// `UnsupportedFeature` warning and left at the IR's default — reporting beats
/// approximating where determinism is at stake (ADR-0004).
///
/// `program` must have been checked without errors; lowering an unchecked
/// program is host misuse, not a content defect, and is rejected as such.
///
/// Returns Status::Ok when nothing was reported as an error.
[[nodiscard]] Status lower(const Program& program, const LoadResult& loaded,
                           const LowerOptions& options, LowerResult& out, DiagnosticSink& sink);

} // namespace scena::dsl
