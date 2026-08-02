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
| `include/scena/dsl/types.h` | the §7.3 type model and the resolved program |
| `include/scena/dsl/resolve.h` | `resolve()` — AST to symbol table, and the prelude |
| `src/types.cpp` | queries over a resolved program |
| `src/resolve.cpp` | the resolution passes and the §7.3/§7.3.11/§7.5 rules |
| `include/scena/dsl/expression.h` | expression typing and constant evaluation |
| `src/expression.cpp` | the §7.4 operator rules and the constant folder |
| `tests/dsl_lexer_test.cpp` | §7.2.1 token goldens |
| `tests/dsl_parser_test.cpp` | §7.2.2 grammar, expression precedence, error recovery |
| `tests/dsl_types_test.cpp` | §7.3 type rules, units, inheritance, namespaces |
| `tests/dsl_expression_test.cpp` | §7.4 typing, conversion rules, constant folding |
| `tests/dsl_constraint_test.cpp` | §7.3.11 constraints and §7.5 coverage |

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

## Resolution notes (ADR-0028)

- **Three passes, because the language declares anywhere.** §7.3.15 puts no
  ordering restriction on declaration and use, and §7.3.9 lets an `extend` reach
  a type declared later or in another file. Declare, then link, then members —
  and anything needing a supertype's members (a conditional-inheritance
  determinant) waits for the end of the third pass.
- **Types are indices, not pointers.** The table grows while it is being built —
  `list of int` becomes a type the moment someone writes it — so helpers take a
  `TypeId` and re-index after any call that can resolve a declarator. A
  `TypeInfo&` held across such a call is a dangling reference.
- **Ordered containers everywhere.** `std::map` plus a declaration-order vector
  for the two places order is meaning: positional arguments and enumeration
  value succession. Load time is inside the determinism contract.
- **Physical types are compared by dimension.** A unit must carry its type's
  SI exponents exactly (§7.3.4), which is what makes
  `base = value * factor + offset` a legal direct conversion.
- **The standard library is source, not a hand-built table.** It travels
  through the same lexer, parser and resolver as user code, so a bug in any of
  them shows up there first (ADR-0029).

## Expression and constraint notes

- **Typing may create a type.** A list or range constructor names a structural
  aggregate no declarator had to mention, so `type_of` takes the program by
  reference and interns it — that is what makes `[c, c]` and a declared
  `list of car` the same `TypeId`. As in resolution, nothing may hold a
  `TypeInfo&`, a `FieldInfo*` or a `MethodInfo*` across a call that types a
  subexpression.
- **One mistake, one message.** An operand that failed to type makes its parent
  untyped without a second complaint, so a misspelled name does not cascade into
  a paragraph.
- **A physical literal folds to its base unit.** `36kph` and `10mps` are the
  same constant, which is what lets a constraint over two units of one type be
  decided without a conversion step at every comparison.
- **Constant folding is how satisfiability is decided.** A `keep` that folds to
  `false` is an error when hard and a warning when `default` (§7.3.11.3). One
  that does not fold is accepted only in the shapes v0.0.1 can resolve without
  search — `f == const`, `f in <const>`, and conjunctions of those. Everything
  else gets an `UnsupportedFeature` note rather than a wrong answer (ADR-0004).
- **Checked-but-not-executed says so.** `cover`, `record`, `sample()` and
  `every` are typed and then reported as unsupported, at Warning severity with
  `Status::UnsupportedFeature` — the file is legal DSL, the engine just does not
  run that part in v0.0.1.

## Standard library and import notes

- **The library is the §8 document text, not the shipped files.** §8.16 says
  `types.osc` / `domain.osc` / `standard.osc` are non-normative and the document
  is the normative part, so every declaration is transcribed from §8 with the
  subsection cited beside it (ADR-0002, ADR-0029).
- **It is embedded, not installed.** `src/stdlib.cpp` carries the DSL text as
  raw literals, split into chunks because MSVC caps one literal at 16380 bytes.
  Nothing depends on an install prefix, so the CLI, the C ABI and the Python
  wheel all get the same library.
- **The types sub-module is built in and auto-used.** `stdtypes` is loaded for
  every check and starts on the use list of the null namespace — the
  "built-in definitions" route §7.7.5.2 offers, and what makes `30kph` typeable
  in a file that has not imported anything. A `namespace` statement replaces the
  use list, and from there the ordinary §7.7.4 rules hold.
- **Conversion factors are the printed ones.** §8.14.1 prints kph as
  `0.277777778`, so `36kph` is 10.000000008 m/s and `36kph == 10mps` is false.
  Matching a conforming implementation beats matching arithmetic.
- **Import-once makes cycles a non-event.** §7.7.5.1 imports a file at the
  first place a depth-first traversal reaches it; a second reference does
  nothing, so a cycle terminates and a diamond declares its shared types once.
- **`osc` is reserved.** A module reference whose first component is `osc` never
  reaches the search path: an unknown one is reported rather than looked up
  (§7.7.5.1.2). Everything else maps `a.b.c` to `a/b/c.osc` under the configured
  search paths.
- **`check_source` / `check_file` are the entry points.** Loading and resolving
  in one call, with imports followed; the CLI and the bindings sit on them. The
  `LoadResult` owns the ASTs a `Program` points into, so it must outlive it.
