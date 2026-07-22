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

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "scena/engine.h"

namespace scena::testsupport {

/// 16 lowercase hex chars of a double's IEEE-754 bit pattern. Locale-immune
/// and exact — never uses printf `%f` or an ostream, which would round and
/// respect the C locale. This is the on-the-wire form the cross-platform trace
/// diff compares byte for byte.
[[nodiscard]] std::string hex_bits(double value);

/// Accumulates a "scena-trace v1" text image: a deterministic, ASCII-only,
/// LF-only record of engine state and detmath outputs, designed so two runs on
/// different platforms produce byte-identical files iff the runtime is
/// bit-identical. `cmp`/`diff -u` on the result is the cross-platform proof.
///
/// Line grammar (also documented in docs/user-guide/determinism.md):
///   # scena-trace v1 fixture=<name>
///   t <step> <time-hex16>
///   e <step> <entity-id> <x> <y> <z> <heading> <speed>   (each hex16)
///   m <tag> <in-hex16> <out-hex16>
/// A polled entity that is absent yields `e <step> <id> MISSING` so the diff
/// catches a vanished entity instead of silently skipping it.
class TraceRecorder {
public:
    /// Emits the format header. Call once, first.
    void header(const std::string& fixture_name);

    /// Appends a free-form comment line, prefixed `# `.
    void note(const std::string& line);

    /// Records the engine clock (`t`) and one `e` line per entity id, in the
    /// order given, from the engine's current state.
    void record_step(int step, const Engine& engine, const std::vector<std::string>& entity_ids);

    /// Records one detmath probe result as an `m` line. A NaN output is written
    /// as the literal `nan` (its bits are out of contract and may differ).
    void record_value(const std::string& tag, double input, double output);

    /// The accumulated trace text (LF line endings, trailing newline).
    [[nodiscard]] const std::string& text() const { return text_; }

    /// Writes text() to `path` in binary mode (no CRLF translation on Windows).
    /// Returns false if the file could not be opened or written.
    [[nodiscard]] bool write_file(const std::filesystem::path& path) const;

private:
    std::string text_;
};

} // namespace scena::testsupport
