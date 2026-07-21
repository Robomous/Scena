# ADR-0002: Licensing and clean-room policy

- **Status:** accepted
- **Date:** 2026-07-21

## Context

Kinema must be safely embeddable in commercial simulation products, and its
implementation must be legally unencumbered with respect to both the ASAM
standard texts and existing scenario engines.

## Decision

### License

Kinema is licensed under **MIT**. Every dependency must be MIT, BSD, or
Apache-2.0 compatible:

- **No GPL or LGPL anywhere** in the dependency tree, including statically
  linked LGPL components.
- **No MPL-licensed code in the runtime dependency tree.**
- No proprietary SDKs.
- Dependencies are fetched with CMake FetchContent, pinned to exact tags in
  `cmake/Dependencies.cmake`, and each one is recorded with its license in
  `THIRD_PARTY_LICENSES.md`.
- The `kinema-core` runtime library keeps zero third-party runtime
  dependencies; test frameworks and binding layers are isolated to their own
  targets.

### Clean-room implementation

Standard behavior is implemented **exclusively from the ASAM OpenSCENARIO
specifications**. Contributors must never copy, translate, or paraphrase
source code from any existing scenario engine, regardless of its license.
Studying other engines' code to derive Kinema's implementation is not
permitted; the specification text is the only normative input.

### Local-only reference texts

The maintainer copies the ASAM reference texts into `docs/reference/` for
local use. That directory is **gitignored from the first commit**: the texts
are never committed and never quoted verbatim in committed files, because the
standard documents are not redistributable under the project license. Cite the
standard by section number instead (e.g. "per ASAM OpenSCENARIO XML 1.3
§x.y").

## Consequences

- License review is part of dependency review; adding a dependency without a
  `THIRD_PARTY_LICENSES.md` entry fails review.
- Contributors familiar with other engines' internals must implement from the
  specification, not from memory of that code; PR review watches for
  structural similarity.
- Citations by section number keep the implementation auditable against the
  standard without redistributing it.
