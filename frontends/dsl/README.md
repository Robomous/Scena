# `frontends/dsl` — the OpenSCENARIO DSL frontend

Compiles ASAM OpenSCENARIO DSL 2.x (`.osc`) into the shared Scenario IR, the
same IR the XML frontend produces. Two frontends, one runtime: behaviour lives
in `core/`, never here.

## Provenance

Everything in this directory is written **from the ASAM OpenSCENARIO DSL
specification text** — the local reference copy under
`docs/reference/asam/openscenario-dsl-2.2.0/`. No grammar, lexer, parser or
test corpus is taken, translated or adapted from any other implementation of
the language, whatever its license (ADR-0002). Section numbers in the code
comments are the citations for that: each one names the clause the code
implements, so a reader can check the behaviour against the standard rather
than against another project.

The spec defines **no `asam.net:` rule ids** for the DSL — unlike the XML
standard's checker rules — so diagnostics cite section numbers instead. A
`Diagnostic::rule_id` from this frontend is always empty, deliberately.

## No parser generator

The lexer and parser are hand-written recursive descent (ADR-0027). The
frontend's exit criteria are about diagnostic quality and error recovery, and
those are easier to get right when every error site already knows what it
expected. It also keeps the dependency list unchanged.

## Layout

| Path | What |
|---|---|
| `include/scena/dsl/token.h` | token kinds (§7.2.1.5) and the `Token` type |
| `include/scena/dsl/lexer.h` | `lex()` — source to token stream |
| `src/lexer.cpp` | line structure, the offside rule, literals, operators |
| `tests/dsl_lexer_test.cpp` | §7.2.1 token goldens |

## Lexing notes

- **One pass over the whole buffer.** The layout rules are not local: whether a
  newline ends a logical line depends on bracket depth and on line continuation
  (§7.2.1.2), and a DEDENT is only known once the next line's indentation has
  been measured (§7.2.1.4).
- **No `Keyword` token kind.** §7.2.1.5.1 makes reserved words keywords "only
  ... in the places identified in the grammar", so every word lexes as an
  identifier that records whether it is reserved; the parser decides. The
  `|...|` form is never a keyword — delimiting is how a file names something
  otherwise reserved.
- **Errors do not stop lexing.** An unterminated string or a stray character is
  reported and the scan continues, so one run can report more than one problem.
  The token stream stays well formed either way: one `EndOfFile`, and every
  `Indent` matched by a `Dedent`.
- **`std::from_chars`, never `strtod`.** A load-time value reaches the IR, so
  it is inside the determinism contract; the locale must not decide what `1.5`
  means.
