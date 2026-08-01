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
#include <string_view>

#include "scena/diagnostic.h"
#include "scena/opendrive/map.h"
#include "scena/status.h"

namespace scena::opendrive {

/// Parses an OpenDRIVE document from memory into the p3-s2 `Map` subset.
///
/// Reading follows the kernel's diagnostics contract: content defects are
/// reported through `sink` (errors cite the `asam.net:xodr` rule id where the
/// standard defines one) and the function returns the Status of the first
/// error; map features outside the consumed subset are *never* dropped
/// silently — each one is reported as a Severity::Warning /
/// Status::UnsupportedFeature diagnostic (Status::DeprecatedFeature for
/// `<poly3>`, which is still evaluated). Warnings leave the return Ok.
///
/// Numeric attributes are parsed with the locale-independent
/// `ir::parse_scalar` (std::from_chars) — never strtod-family, whose locale
/// sensitivity is the classic cross-platform map-loading bug.
///
/// On error `out` is left in a valid but unspecified state; use it only when
/// the return is Status::Ok.
[[nodiscard]] Status load_string(std::string_view xml, Map& out, DiagnosticSink& sink);

/// Reads `path` in binary mode (CRLF-safe on every platform) and parses it
/// with `load_string` semantics; diagnostics carry the file path in their
/// source location.
[[nodiscard]] Status load_file(const std::filesystem::path& path, Map& out, DiagnosticSink& sink);

} // namespace scena::opendrive
