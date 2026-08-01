# Loading scenarios

Scena reads ASAM OpenSCENARIO XML documents through the XML frontend
(`scena::frontend-xml`). This chapter covers the document layer: how a file
becomes a parsed document, which versions are accepted, and how the loader
reports what it found. The lowering of entities and the storyboard into the
Scenario IR arrives in later sprints; until then a document that loads
cleanly reports one warning per element that is not consumed yet.

## The loader API

```cpp
#include "scena/xml/loader.h"

scena::xml::Document document;
scena::DiagnosticSink sink;

const scena::Status status = scena::xml::load_file("cut_in.xosc", document, sink);
if (status != scena::Status::Ok) {
    for (const scena::Diagnostic& d : sink.diagnostics()) {
        // d.severity, d.code, d.message, d.path, d.location, d.rule_id
    }
    return;
}
// document.version — what the file declared
// document.kind    — scenario, catalog, or parameter value distribution
// document.scenario — the Scenario IR
```

`load_string(std::string_view, Document&, DiagnosticSink&)` does the same for
a document already in memory.

Both follow the kernel's [error model](error-handling.md): findings go to the
sink, the return value is the status of the *first* error, and warnings leave
the return `Status::Ok`. An `Error` diagnostic always means a non-`Ok`
status — a loaded document with warnings is a usable document.

`load_file` reads in binary mode. That is what makes a CRLF file behave the
same on every platform: no newline translation happens, so the bytes — and
the line and column a diagnostic reports — describe the file as it is stored.

## Versions

| Declared `revMajor.revMinor` | Result |
|---|---|
| 1.0, 1.1, 1.2, 1.3 | Accepted |
| 1.4 and later 1.x | Rejected, `Status::UnsupportedFeature` |
| any other major | Rejected, `Status::ValidationError` |
| missing / not an integer | Rejected, `Status::ParseError` |

Scena targets 1.0–1.3, so 1.4 documents are rejected rather than executed on
a guess about constructs the engine never implemented. The detected version
is still reported in `Document::version`, so a host can tell its user what
the file claimed.

Within the supported range the runtime does not branch on the minor
revision. Version 1.3 corrected the calculation specifications of the
`Position` sub-classes, and per ASAM OpenSCENARIO XML §5 those corrected
semantics are the ones an implementation should apply to every version —
Scena runs one set of position semantics for the whole range.

## Document kinds

The root `OpenSCENARIO` element holds one of three document forms (§9), and
the loader resolves which at load time:

- `DocumentKind::Scenario` — a scenario definition (`Storyboard`,
  `Entities`, `RoadNetwork`, …).
- `DocumentKind::Catalog` — a catalog file (`Catalog`); catalog loading and
  reference resolution arrive with p4-s4.
- `DocumentKind::ParameterValueDistribution` — rejected: stochastic
  parameter-value-distribution files are outside the v0.0.1 scope.

Mixing two categories in one file is a `ValidationError`.

## Diagnostics

Every finding carries:

- **a path** — an xpath-ish address built from the document tree, with a
  1-based predicate exactly when an element has same-named siblings:
  `/OpenSCENARIO/Storyboard/Story[2]/Act`, `/OpenSCENARIO/FileHeader/@revMinor`.
- **a source location** — the file (for `load_file`) plus a 1-based line and
  column. `"\r\n"` counts as one line break. Where the input is not UTF-8 and
  the parser transcodes it, offsets would index the transcoded text, so the
  position is reported as unknown (0) rather than pointing at the wrong
  place.
- **a rule id** where the standard defines one, e.g.
  `asam.net:xosc:1.0.0:xml.valid_schema` for version and structure
  rejections, `asam.net:xosc:1.0.0:general.file_ending` for a file that is
  not named `.xosc`, `asam.net:xosc:1.0.0:data_type.time_format` for a
  `FileHeader` date outside the ISO 8601 basic notation.

Nothing is dropped silently. Elements the loader does not implement yet are
reported as `Warning` / `Status::UnsupportedFeature`, so the warning list of
a real file is an honest statement of what Scena did and did not read.

Two loads of the same bytes produce element-wise identical diagnostics: the
sink never reorders, deduplicates, or timestamps, and messages never contain
a formatted floating-point value.

## Encodings and numbers

UTF-8 (with or without a byte-order mark) and UTF-16 documents all load; the
encoding is detected from the byte-order mark and the XML declaration.

Every numeric attribute is converted with `std::from_chars`, never with a
locale-sensitive function. This matters more than it looks: under a
comma-decimal locale, `std::stod("1.5")` returns `1`, so the same file would
load differently on two machines and break bit-identical determinism before
the engine ever ran. Conversions are whole-token, so `"1,5"`, `"1.5x"` and
`" 1.5"` are not numbers and are reported as defects in the document.

## See also

- [Error handling](error-handling.md) — the status codes, the
  severity/status invariant, and the diagnostic path grammar.
- [Determinism](determinism.md) — why locale-dependent parsing is a
  determinism bug, not a formatting preference.
- [ADR-0020](../architecture/ADR-0020-xml-document-layer.md) — the document
  layer's design decisions.
