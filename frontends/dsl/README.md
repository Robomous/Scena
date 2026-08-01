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
| `include/scena/dsl/ast.h` | the §7.2.2 syntax tree |
| `include/scena/dsl/parser.h` | `parse()` / `parse_source()` — tokens to AST |
| `src/parser.cpp` | recursive descent over the §7.2.2 productions |
| `tests/dsl_lexer_test.cpp` | §7.2.1 token goldens |
| `tests/dsl_parser_test.cpp` | §7.2.2 grammar, expression precedence, error recovery |

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

## Parsing notes

- **Left recursion is rewritten as iteration.** §7.2.2's grammar is written
  "for explanatory and normative purposes" and says implementers "might want to
  eliminate left recursions"; `relation`, `sum`, `term` and the postfix chain
  are loops here, which recognises the same language and yields the same
  left-associative tree.
- **The AST is one node type per family, not a class hierarchy.** An `Expr`
  carries a kind tag, because the tree is walked far more often than it is
  built — by name resolution, the type checker, the constant evaluator and
  lowering — and a switch over a tag keeps each of those in one readable place.
- **Errors do not stop parsing.** A parse error is reported and the parser
  resynchronises at the next end of logical line, or skips the current block,
  and carries on. A malformed member does not cost its siblings, and a
  malformed declaration does not cost the rest of the file. This is what makes
  a `scena-check` run useful on a file with several mistakes in it.
- **A reserved word is a keyword only where the grammar says so.** `type: int`
  declares a field named `type`; the parser decides, exactly as §7.2.1.5.1
  requires, which is why the token stream has no `Keyword` kind.
- **The `-` sign lives in the literal.** §7.2.1.5.2 puts the sign inside
  `int-literal`, so the lexer hands `-2` over as one token; where a value has
  already been parsed, `parse_sum` splits it back into a subtraction.
