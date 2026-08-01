# ADR-0027: A hand-written lexer and parser for the DSL frontend (OQ-2)

- **Status:** accepted
- **Date:** 2026-08-01

## Context

The roadmap named ANTLR4 for the OpenSCENARIO DSL frontend and left the wiring
as an open question:

> ANTLR4 wiring needs maintainer dependency approval before p7-s1 (open question
> OQ-2: build-time generation vs vendored generated sources — recommendation:
> vendor the generated parser sources so the build needs no Java toolchain).

Both variants of that recommendation still add a third-party runtime: ANTLR's
C++ runtime is a library, not a header, whichever way the parser sources are
produced. `CLAUDE.md` requires maintainer approval before any new dependency,
and Scena's dependency policy admits MIT/BSD/Apache-2.0/zlib/BSL-1.0 only.

The pillar's exit criteria are what actually decide this:

> the DSL standard library type-checks with zero errors; spec-annex-derived
> example files parse and check with **precise diagnostics (section-number
> citations — the DSL spec defines no rule IDs)**; `scena-check` exits nonzero
> with **actionable messages** on seeded error corpora.

So the frontend is judged on the quality of its diagnostics and its error
recovery, not on the speed of writing its grammar.

## Decision

**No parser generator. The lexer and parser are hand-written from the ASAM
OpenSCENARIO DSL specification text**, clean room per ADR-0002, in
`frontends/dsl/`.

Four reasons, in the order they mattered:

1. **Diagnostics are the deliverable.** A generated parser reports what its
   grammar could not match; the exit criteria ask for messages that cite the
   section they enforce and say what was expected where. Getting there through a
   generator means fighting its error strategy. Writing the descent by hand
   means every error site is a place where the expectation is already known,
   which is exactly what a §-citation needs.
2. **Error recovery is a design choice, not a fallback.** "Emit more useful
   diagnostics rather than stop at the first error" is a decision about
   synchronisation points in the grammar — statement starts, block boundaries,
   NEWLINE. A hand-written parser makes those explicit and testable.
3. **No new dependency.** No approval to obtain, no license row, no CMake
   fetch, no Java toolchain at any point in the build, nothing new for a
   downstream packager to satisfy. The build stays what it is.
4. **It matches the repo.** The XML frontend's §9.2.1 expression parser is
   already hand-written recursive descent (ADR-0022), and the OpenDRIVE reader
   is hand-written over pugixml. A second parsing idiom would be the outlier.

The DSL's grammar suits this: §7.2.2 is an LL grammar with an offside rule, and
the one place needing care — expression precedence (§7.4) — is a precedence
climb, which is a well-understood thirty lines.

### The lexer's shape (p7-s1)

`lex(source, file, tokens, sink)` tokenizes a whole buffer in one pass and
returns the tokens together. Not an incremental `next_token()`, because the
layout rules of §7.2.1.2 and §7.2.1.4 are not local: whether a newline ends a
logical line depends on bracket depth and on whether the previous physical line
ended in a `\`, and a DEDENT is only known once the *next* line's indentation
has been measured.

Two decisions inside it are worth recording:

- **There is no `Keyword` token kind.** §7.2.1.5.1 says reserved words "are only
  recognized as keywords in the places identified in the grammar. This means
  that these identifiers should be treated as normal identifier tokens in all
  other places." A lexer that returned `Keyword` for `scenario` would make a
  field named `type` unparseable. So every word is an `Identifier` that records
  *whether* it is reserved, and the parser — which knows the grammar position —
  decides. The `|...|` form is never a keyword: delimiting is precisely how a
  file names something otherwise reserved.
- **`inf` and `nan` lex as float literals**, although both appear in Table 4.
  §7.2.1.5.2's `float-literal` production lists them, and they can denote
  nothing else.

Identifier start characters are approximated as "ASCII letter, underscore, or
any byte ≥ 0x80" rather than by carrying a Unicode category table. The spec
mandates UTF-8 (§7.2.1.1), so every byte ≥ 0x80 belongs to a multi-byte code
point; the approximation accepts a few non-letter code points the spec would
reject, and the `|...|` form exists for exactly those characters anyway.
Rejecting valid non-Latin identifiers would be the worse error.

Numbers go through `std::from_chars`, never `strtod` — the locale must not
decide what `1.5` means. This is the same rule the XML frontend follows, and
for the same reason: a load-time value reaches the IR and is therefore inside
the bit-identity contract.

## Consequences

- `frontends/dsl/` is a new library, `scena::frontend-dsl`, alongside
  `scena::frontend-xml`. Both compile into the shared Scenario IR; runtime
  semantics stay in the runtime. `capi/` and `python/` do not depend on it yet —
  p7-s5 adds the `check_dsl` entry points.
- The dependency list is unchanged. `THIRD_PARTY_LICENSES.md` needs no row.
- More code to maintain than a grammar file: roughly a lexer, a parser, and an
  AST rather than one `.g4`. That is the cost, and it buys the diagnostics the
  exit criteria are written in terms of.
- **Reversible.** The AST is the frontend's interface to everything downstream;
  a future ANTLR-based parser can be dropped in behind it without touching the
  symbol table, the type system, or the lowering. Nothing above the parser knows
  how the tokens were produced.

## Alternatives considered

**Vendor ANTLR-generated sources** (the roadmap's own recommendation). Removes
the Java toolchain from the build but still adds the ANTLR C++ runtime as a
dependency, and adds generated sources to the tree that must be regenerated —
by a tool nobody has installed — whenever the grammar changes. It also leaves
the diagnostics problem exactly where it was.

**Generate at build time.** Puts Java on the critical path of every build and
every CI runner, for a language whose grammar changes about once a year.

**A parser-combinator or PEG header-only library.** Header-only sidesteps the
runtime-library objection but is still a new dependency, and the error messages
of a combinator library are typically worse than a generator's, not better.
