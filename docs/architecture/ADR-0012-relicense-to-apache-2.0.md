# ADR-0012: Relicense from MIT to Apache-2.0

- **Status:** accepted
- **Date:** 2026-07-22
- **Supersedes:** the *License* section of
  [ADR-0002](ADR-0002-licensing-and-clean-room.md) (its dependency rules and
  its entire clean-room policy stay in force)

## Context

ADR-0002 licensed Scena under MIT to make it safely embeddable in commercial
simulation products. MIT achieves that, but it is silent on two things that
matter for a standards-implementing engine intended for industrial adoption:

- **Patents.** MIT grants no express patent licence. Scena implements ASAM
  OpenSCENARIO semantics — an area where adopters and their legal reviewers
  expect an explicit, irrevocable patent grant from contributors before
  embedding an engine in a shipped product.
- **Contribution provenance.** MIT defines no contribution terms, so the
  inbound licence of a pull request is a matter of custom rather than of the
  licence text.

Apache-2.0 addresses both directly: §3 is an express patent grant with a
defensive termination clause, and §5 makes every contribution inbound under
the same terms unless stated otherwise. It remains a permissive,
OSI-approved, non-copyleft licence, so the embeddability property that
motivated the original MIT choice is preserved.

Relicensing is cheap right now and gets more expensive with every external
contributor: at the time of this decision all copyright in the tree is held
by Robomous, so no third-party consent is required.

## Decision

Scena is licensed under **Apache License, Version 2.0**.

- `LICENSE` holds the verbatim Apache-2.0 text, including its unmodified
  appendix (the `[yyyy] [name of copyright owner]` placeholders are part of
  the canonical text and are deliberately left intact).
- A top-level `NOTICE` file carries the project's copyright attribution, as
  contemplated by Apache-2.0 §4(d). Redistributions must carry it forward.
- Every source file starts with `// SPDX-License-Identifier: Apache-2.0`
  (comment syntax per file format). The short SPDX identifier remains the
  per-file convention — Scena does not use the full Apache boilerplate header
  in every file.
- The Python distribution metadata declares `Apache-2.0` and the
  `License :: OSI Approved :: Apache Software License` classifier.

### What does not change

- **Dependency policy.** MIT / BSD / Apache-2.0 / zlib / BSL-1.0 only; no GPL
  or LGPL anywhere, no MPL in the runtime tree. Inbound permissive licences
  are all compatible with an Apache-2.0 outbound project, so the existing
  entries in `THIRD_PARTY_LICENSES.md` (googletest BSD-3-Clause, pugixml MIT,
  nanobind BSD-3-Clause) need no action.
- **Clean-room policy.** ADR-0002's prohibition on copying, translating, or
  paraphrasing any other scenario engine's source is unaffected, as is the
  local-only treatment of the ASAM reference texts.
- **The code.** This is a licence change only; no runtime behavior, API, or
  ABI is touched.

## Consequences

- Contributors grant a patent licence covering their contributions, and
  anyone asserting patents against the project over it loses that grant
  (Apache-2.0 §3). This is the substantive gain over MIT.
- Downstream redistributors now have obligations MIT did not impose: retain
  the licence and `NOTICE`, and state significant changes to modified files
  (§4). Purely internal embedding is unaffected.
- Apache-2.0 is one-way compatible with GPLv3 but **not** with GPLv2. This
  does not affect Scena — the dependency policy already excludes GPL — but it
  narrows how GPLv2 projects may consume Scena.
- Future relicensing gets harder as external contributors accumulate. Any
  further change requires either their consent or a CLA, which the project
  does not have. Treat Apache-2.0 as the terminal choice.
