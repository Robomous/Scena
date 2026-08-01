# ADR-0020: The OpenSCENARIO XML document layer (p4-s1)

## Status

Accepted (p4-s1, #24).

## Context

P4 builds the OpenSCENARIO XML frontend. Before any storyboard or entity
lowering can land, the frontend needs a document layer: something that turns
bytes on disk into a parsed tree, decides whether the document is a version
Scena executes, and reports what it finds in the kernel's structured
diagnostics vocabulary. Everything P4 does afterwards inherits those
decisions, so they are worth fixing once.

Two project rules dominate the design. First, determinism: identical input
must produce identical output, which rules out every locale-sensitive
numeric conversion (`std::stod`, `atoi`, iostream extraction) — under a
comma-decimal locale they read `"1.5"` as `1`. Second, the never-silent
rule: a construct the loader does not implement is reported, never dropped.

## Decision

- **Parser: pugixml v1.14 (MIT).** Already approved, pinned in
  `cmake/Dependencies.cmake` and recorded in `THIRD_PARTY_LICENSES.md` for
  the OpenDRIVE backend (p3-s2); the XML frontend reuses it rather than
  adding a second XML dependency. It stays a **private** link dependency of
  `scena::frontend-xml` — no pugixml type appears in a public `scena/xml`
  header, so the frontend's surface is Scena's own.
- **API shape mirrors the OpenDRIVE reader.**
  `load_string(std::string_view, Document&, DiagnosticSink&) → Status` and
  `load_file(std::filesystem::path, …)`. Diagnostics go to the sink, the
  return is the status of the first error, warnings leave it `Ok`. The
  earlier `load_xosc` stub, which carried its own private `Severity` and
  `Diagnostic` types, is removed: the frontend uses the kernel's diagnostic
  model, so a host has exactly one diagnostic vocabulary to handle.
- **Version policy.** `FileHeader/@revMajor` and `@revMinor` are required.
  1.0–1.3 are accepted; 1.4 and any later 1.x are rejected as
  `UnsupportedFeature`; any other major revision is a `ValidationError`.
  Both cite `asam.net:xosc:1.0.0:xml.valid_schema`, and the detected version
  is reported in `Document::version` even when the document is rejected, so
  a host can tell its user what the file claimed. Per §5 the 1.3 corrections
  to the `Position` calculation specifications are the semantics an
  implementation should apply to *every* version, so the runtime does not
  branch on the minor revision — the version gates acceptance, not
  behavior.
- **Locale-safe conversions only.** `scena::xml::detail::parse_double` /
  `parse_integer` / `parse_boolean` are the frontend's only numeric readers;
  `parse_double` delegates to the kernel's `ir::parse_scalar`
  (`std::from_chars`) so frontend and runtime agree on what a number is.
  All three are whole-token: `"1,5"`, `"1.5x"` and `" 1.5"` are not numbers.
- **Binary-mode reading.** Files are read with `std::ios::binary`, so no
  newline translation happens and a CRLF document is the same bytes — and
  reports the same lines — on every platform. `"\r\n"` counts as one line
  break.
- **xpath-ish addressing.** Every diagnostic is anchored to an absolute
  element path built from the tree, with a 1-based `[i]` predicate exactly
  when an element has same-named siblings
  (`/OpenSCENARIO/Storyboard/Story[2]/Act`), and attributes addressed as
  `…/FileHeader/@revMinor`. Line and column are byte positions into the
  source. Where pugixml transcodes the input (UTF-16), its offsets index the
  transcoded text, so positions are reported as unknown (0) rather than
  pointing at the wrong place.
- **Never silent.** Every element the layer does not consume — including
  `Entities` and `Storyboard`, which p4-s2 will consume — is reported as a
  `Warning` / `UnsupportedFeature` diagnostic. Sprints remove those warnings
  as they implement the elements.
- **Document kind up front.** The `OpenScenarioCategory` choice is resolved
  at load: scenario, catalog, or parameter-value distribution. Mixing two
  categories in one file is an error; parameter-value-distribution documents
  are rejected as out of scope (roadmap P4 scope-out).

## Consequences

- p4-s2..s5 write element readers against a settled context: report through
  `ReadContext`, address with `element_path`, convert with `detail::parse_*`.
- A document that parses cleanly today still yields an empty
  `Document::scenario` plus one warning per unconsumed element; that is the
  honest state of the frontend until p4-s2 lands, and the warnings are the
  work list.
- The frontend's public API changed (`load_xosc` removed) with no C ABI or
  Python impact: the loader was a stub referenced nowhere else, and the XML
  frontend is not exposed through the C ABI or the bindings yet — that
  surface arrives with P6.
- Hosts get one diagnostic model for road networks and scenarios alike:
  same `Diagnostic`, same severity/status invariant, same rule-id citations.
