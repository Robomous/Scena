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
#include "scena/status.h"
#include "scena/xml/document.h"

namespace scena::xml {

/// Loads an ASAM OpenSCENARIO XML document (versions 1.0-1.3) from memory.
///
/// Reporting follows the kernel's diagnostics contract, the same one the
/// OpenDRIVE reader uses: content defects go to `sink` — errors cite the
/// `asam.net:xosc` rule id where the standard defines one — and the function
/// returns the Status of the first error. Constructs outside the consumed
/// subset are *never* dropped silently: each one is reported as a
/// Severity::Warning / Status::UnsupportedFeature diagnostic, and warnings
/// leave the return Status::Ok.
///
/// Every diagnostic is addressed by an xpath-ish element path
/// ("/OpenSCENARIO/FileHeader/@revMinor") and carries a 1-based line and
/// column into the source text.
///
/// Version policy (§5, and the coverage matrix): `FileHeader/@revMajor` and
/// `@revMinor` are required; 1.0-1.3 are accepted; 1.4 and any later 1.x are
/// rejected with Status::UnsupportedFeature; any other major revision is
/// rejected with Status::ValidationError.
///
/// Numeric attributes are read exclusively with std::from_chars, through
/// `xml::detail`, never with a locale-sensitive conversion.
///
/// The document layer landed in p4-s1; the entity and storyboard lowering
/// that fills `out.scenario` arrives with p4-s2, so a document that parses
/// cleanly today still reports one UnsupportedFeature warning per element
/// that is not yet consumed.
///
/// On error `out` is left in a valid but unspecified state; use it only when
/// the return is Status::Ok.
[[nodiscard]] Status load_string(std::string_view xml, Document& out, DiagnosticSink& sink);

/// Reads `path` in binary mode and loads it with `load_string` semantics.
///
/// Binary mode is what makes CRLF documents behave identically on every
/// platform: no newline translation happens, so byte offsets — and therefore
/// the reported line and column — describe the file as it is stored. A file
/// whose extension is not `.xosc` is loaded anyway, with a warning citing
/// asam.net:xosc:1.0.0:general.file_ending. Diagnostics carry the path in
/// their source location.
[[nodiscard]] Status load_file(const std::filesystem::path& path, Document& out,
                               DiagnosticSink& sink);

} // namespace scena::xml
