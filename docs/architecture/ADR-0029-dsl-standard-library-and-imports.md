<!--
SPDX-FileCopyrightText: 2026 Robomous
SPDX-License-Identifier: Apache-2.0
-->

# ADR-0029 — The bundled DSL standard library and the import model

- **Status:** Accepted
- **Date:** 2026-08-01
- **Sprint:** p7-s5 (#43)
- **Supersedes:** nothing. Builds on ADR-0027 (hand-written frontend) and
  ADR-0028 (symbols and the type model).

## Context

ASAM OpenSCENARIO DSL 2.2.0 §8.16 describes a standard library — the
`stdtypes` and `std` namespaces — and ships `types.osc`, `domain.osc` and
`standard.osc` alongside the specification. It is explicit that **those files
are not normative**: "The `types.osc`, `domain.osc`, and `standard.osc` files
are non-normative. The normative part is defined in the document." §7.7.5.2 is
equally explicit that how an implementation makes the library reachable is its
own choice: "Whether an implementation implements access to the standard
library by importing some variant of these files, or through any other means
(for example, by providing access to built-in definitions) is left to the
implementation."

Until this sprint the frontend carried a hand-written prelude of the §7.3.4
physical types, injected by test helpers concatenating it in front of the
source. That was enough to type `30kph`; it is not a standard library, it is
not reachable the way a scenario author would reach it, and concatenation is
not what an import does.

Separately, `import` was parsed and then ignored: the resolver had no file
loader, so a multi-file scenario could not be checked at all.

## Decision

### 1. The library is authored from the normative §8 text, as DSL source

Every type, unit, struct and method is transcribed from the §8 chapters of the
specification document, which is the normative part, and each block cites the
subsection it comes from. The shipped `.osc` files are never read, copied or
adapted — consistent with ADR-0002's clean-room rule, and in this case the
standard itself says the document, not the files, is what binds.

The library is DSL **source**, not hand-built `TypeInfo` values. It therefore
travels through the same lexer, parser and resolver as user code, and a defect
in any of the three shows up in the library first. `DslStdlibTest` asserts the
library checks with **zero diagnostics of any severity** — not merely zero
errors, because a warning in the library would appear in every user's output.

The source lives in `frontends/dsl/src/stdlib.cpp` as raw string literals
rather than as installed files. Nothing then depends on an install prefix, a
working directory or a packaging step, which matters most for the C ABI and
the Python wheel. The literals are split into several chunks joined once at
first use, because MSVC caps a single string literal at 16380 bytes.

### 2. Conversion factors are the ones the standard prints

§8.14.1 prints, for instance, the kilometre-per-hour factor as `0.277777778`
rather than as 1/3.6. The library carries what the standard prints. The
observable consequence is that `36kph == 10mps` is **false**: 36 kph folds to
10.000000008 m/s.

The alternative — substituting mathematically exact factors — would make Scena
disagree with a conforming implementation that used the printed numbers, and
the project's stated moat is standards correctness. Rounding differences of
this size are also invisible against the physical quantities scenarios
describe. `DslStdlibTest.TheRoundedFactorsOfTheStandardAreCarriedVerbatim`
pins the decision so it stays deliberate.

### 3. The types sub-module is built in, and auto-used in the null namespace

`osc.standard.types` is loaded for every check unless the caller turns it off
(`LoadOptions::implicit_standard_library`), and `stdtypes` starts on the use
list of the null namespace.

This is the "built-in definitions" route §7.7.5.2 offers, and it is the same
auto-use §7.7.5.2.3 defines for the legacy `import osc.standard` form. It is
not a convenience: §8.14's physical types are what give a literal like `30kph`
a type at all, so a file that has not reached its first `import` yet would
otherwise be untypeable.

Ordinary rules resume the moment a `namespace` statement appears — §7.7.4 says
a namespace statement's use list takes effect normally — so a file that
declares a namespace and wants the library says `use stdtypes`, exactly as the
standard describes. A user declaration of the same name shadows the library
one, per §7.7.4.2 rule 2.

`import osc.standard.types`, `.domain`, `.all` and the legacy `osc.standard`
are all accepted and all resolve to bundled sources; the import-once rule
(§7.7.5.1) makes the already-built-in module a no-op rather than a redeclaration.

### 4. Module references map to paths; only `osc` is reserved

§7.7.5.1.2 deliberately leaves module resolution "non-specific for this
release". Scena maps `a.b.c` to `a/b/c.osc` and searches
`LoadOptions::search_paths` in order, first hit wins. A reference whose first
component is `osc` never reaches the search path: the standard reserves it, so
an unknown `osc.*` reference is reported as "not a module of this standard"
rather than silently looked up as a user module.

String-literal imports are URIs (§7.7.5.1.1). The `file` scheme is supported,
in the `file:///path`, `file:/path` and bare-path forms; relative references
resolve against the directory of the referencing file. A `file://host/...` form
names a host and is rejected.

### 5. Import-once, and therefore cycles terminate

§7.7.5.1 requires a file referenced more than once to be imported once, at the
first place a depth-first traversal reaches it. Scena keys that on the weakly
canonical path. The rule also means an import cycle is not an error and needs
no separate detection: the second reference does nothing. A diamond of imports
declares the shared file's types once.

Referenced files are placed ahead of the file that references them, which is
what §7.7.5.1 asks for. Resolution itself is order-independent (§7.3.15,
ADR-0028), so the order is about diagnostics reading sensibly, not about
correctness.

## Consequences

- `builtin_prelude()` is gone. Tests that used to concatenate it now pass the
  library as its own file, which is what an import does and what
  `check_source` does in production.
- Physical types are now named `stdtypes::speed`, not `::speed`. Unqualified
  `speed` still resolves, through the use list.
- `check_source` / `check_file` in `scena/dsl/load.h` are the single entry
  point for "load a `.osc` and everything it needs, then check it". The
  `scena-check` CLI, the C ABI and the Python bindings all sit on them rather
  than re-implementing import resolution.
- A `Program` holds pointers into the ASTs a `LoadResult` owns, so the
  `LoadResult` must outlive it. Both are returned together by every entry
  point for that reason.
- The domain sub-module (`std`) is not bundled yet; it lands with the second
  half of #43. Until it does, `osc.standard.all` and `osc.standard` resolve to
  the types sub-module alone — accepted, not diagnosed, because the reference
  is valid and the shortfall is ours.

## Alternatives considered

- **Install the `.osc` files and read them at runtime.** Rejected: it makes
  every consumer — the CLI, the C ABI, the Python wheel — depend on an install
  layout, and it makes "did the library load" a deployment question instead of
  a compile-time fact.
- **Generate the embedded source from `.osc` files at build time.** Rejected
  for now: it buys reviewability that raw literals already have (they are the
  DSL text, verbatim) at the cost of a build-time generation step on three
  platforms.
- **Keep the prelude in the null namespace and add `stdtypes` beside it.**
  Rejected: two `speed` types with the same dimension but different identities
  would fail to type against each other, which is worse than either option
  alone.
- **Require an explicit `import osc.standard.types` in every file.** Closer to
  the letter of §7.7.5.2.1, but §7.7.5.2 explicitly permits built-in access,
  and requiring the import would make the physical-literal syntax of §7.2.1
  unusable in a file without one.
