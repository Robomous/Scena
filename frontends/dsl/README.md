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
| `include/scena/dsl/load.h` | `load_*` / `check_*` — imports (§7.7.5) and the one-call check |
| `src/load.cpp` | import resolution, both reference forms, import-once |
| `include/scena/dsl/stdlib.h` | the bundled `osc.standard` library (ADR-0029) |
| `src/stdlib.cpp` | its source, as chunked raw literals |
| `tests/dsl_import_test.cpp` | §7.7.5 imports, search paths, diagnostics carrying their file |
| `tests/dsl_stdlib_test.cpp` | the §8 library, pinned declaration by declaration |
| `include/scena/dsl/lower.h` | `lower()` / `entry_points()` — checked program to Scenario IR |
| `src/lower.cpp` | the §7.7.2 entry point, §8.7 actors as entities, §7.3.11 concrete values |
| `tests/dsl_lowering_test.cpp` | the DSL→IR mapping, and that lowering is deterministic |

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
- **A table row is a field.** §8.7 onward prints the domain model as parameter
  tables rather than as code. A row is declared as a field whether the standard
  calls it a parameter or a state variable, and whether or not it is marked
  mandatory: the language has no optional-field marker, and §7.3.11 already lets
  a scenario leave a field unconstrained.
- **An actor's modifier uses the prefixed form.** §8.7.4.1.1 writes
  `stationary_object.location()`, which is `modifier stationary_object.location`
  — not `modifier location of stationary_object`, because §7.3.12.2's `of`
  names a scenario or an action.
- **An actor-associated modifier is reached through its receiver, not through
  the namespace.** §7.3.12.2 puts its name "in the actor scope", so
  `modifier vehicle.keep_lane` interns under the actor's qualified name and
  ordinary lookup will never find `keep_lane` on its own. Application
  (§7.3.12.4.1) resolves the actor expression's type and walks its inheritance
  chain, probing the name table once per step — an exact lookup, not a scan.
  With the actor omitted the receiver is what the site already implies: the
  enclosing declaration in a member position, the invoked behavior's actor
  inside a `with:` block. This was issue #100; before it, such a modifier could
  not be applied at all, and a `with:` block accepted any name whatsoever.
- **Association is what makes §8.9 declarable.** §7.3.12.3's own example is
  `modifier vehicle.keep_lane()`, annotated as being §8.9.16's, so the movement
  modifiers are actor-associated. That is not decoration: an unassociated
  `modifier lane` would collide with §8.12.10's `struct lane`, and an
  unassociated `modifier speed` would shadow `stdtypes::speed` so thoroughly
  that `range of speed` stops naming a physical type. Three modifiers must
  still be unassociated — `change_speed`, `keep_speed`, `change_lane` — because
  §8.8 declares *actions* of those names on those actors, and a qualified
  behavior name identifies exactly one declaration.
- **Every declaration name is a lookup space of its own.** The standard reuses
  words freely: a field `driving_rule` of type `driving_rule` (§8.12.2), a
  struct `air` and an action `environment.air` (§8.10.4 / §8.11.2), an enum
  member `left` in both `side_left_right` and `junction_direction`. Only the
  last of those needs qualifying, and §7.3.3 says how — `side_left_right!left`.
  At library scale that is the normal case, not the exception.
- **Each chapter's translation has a worksheet.** `docs/dev/stdlib-worksheets/`
  holds one file per translated chapter: every declaration with its resulting
  DSL spelling and a note wherever the printed text needed a judgement call.
  Reading a worksheet is much cheaper than re-reading the chapter.
- **`check_source` / `check_file` are the entry points.** Loading and resolving
  in one call, with imports followed; the CLI and the bindings sit on them. The
  `LoadResult` owns the ASTs a `Program` points into, so it must outlive it.
- **All three surfaces reach them.** `scena-check`, the C ABI
  (`scn_check_dsl_file` / `scn_check_dsl_string`, results in an opaque
  `scn_dsl_check`) and Python (`scena.check_dsl_file` / `check_dsl_string`,
  returning a `DslCheck`). None of them constructs an engine — checking is a
  frontend service, and that is why the C result is its own handle rather than
  an engine's diagnostic list. `scripts/parity_audit.py` now audits these entry
  points alongside the `Engine` methods, so a frontend function added to one
  surface and forgotten in the others is a CI failure.

## Lowering notes (ADR-0030)

- **Lowering decides denotation, not semantics.** Both frontends compile into
  one IR; runtime behaviour lives in the runtime. If the DSL side needs
  behaviour the XML side would also need, it belongs in `core/`.
- **The entry point is named, and a lone scenario names itself.** §7.7.2 leaves
  the choice to the implementation. A file declaring several scenarios is an
  error listing them, because picking one would make the run depend on
  declaration order.
- **A participant is a field deriving from `std::physical_object`.** §8.7 roots
  its hierarchy there, so that is the test; `animal` has no taxonomy
  counterpart and stays an unclassified participant rather than being misfiled
  as a pedestrian.
- **Concrete means `keep(field == constant)`.** Either operand order, constant
  side folding without a solver. Everything else is diagnosed (ADR-0004).
  Values arrive already folded to base units, so lowering never converts —
  re-applying §8.14.1.3's printed factors a second time is exactly the bug
  ADR-0029 exists to prevent.
- **An enum literal resolves through the use list.** `vehicle_category!bus`
  written in `namespace demo use std` names a type in `std`, so constant
  evaluation searches the use list after the current namespace (§7.7.4.2).
  Before p8-s1 it searched only the current namespace, which silently made every
  enum-valued `keep` look like one that needs a solver.
- **A movement action denotes an IR action, and the `do` directive a
  storyboard.** Seven §8.8 actions have an unambiguous counterpart and lower;
  the generic `move`/`drive`/`walk` carry no target of their own and lower to
  nothing, because they exist to be shaped by §8.9 modifiers (p8-s3). Anything
  with no counterpart is reported by name — never silently dropped. `serial`
  chains a phase on its predecessor reaching completeState, `parallel` leaves
  the triggers absent; the rest of composition is p8-s2.
- **A struct-valued argument names a declaration.** §7.2.2.6.7 declares list and
  range constructors and no struct constructor, so `assign_position(position:
  start)` reaches its numbers through the same `keep`s that make an actor
  concrete.
- **Composition decides when a phase starts, and nothing else** (ADR-0031). A
  concrete `duration` is arithmetic done at load — the storyboard starts at
  t = 0, so a phase's start time is the sum of the durations before it — and a
  parallel join is one ConditionGroup, which is already an AND. `one_of` picks
  by label from `LowerOptions::alternative`, defaulting to the first: the engine
  has no seed machinery, and a hidden input is what determinism forbids.
  `wait elapsed(d)` lowers to nothing but the offset.
- **`..` has to beat a float that starts with a dot.** §7.2.2.6.7 spells the
  range constructor `[a '..' b]` while §7.2.1.5.2 makes a float's leading digits
  optional, so `[2..4]` is a race the operator must win. It did not until p8-s2;
  the lexer emitted no `..` and the parser looked for `...`, and because the two
  agreed nothing noticed.
- **The map file travels beside the IR.** §8.5.4's `map_file` is the DSL's
  `RoadNetwork/LogicFile`, and like it, a road-network path is a host input
  rather than kernel state — the engine reaches roads only through
  `IRoadQuery`. `map.set_map_file("m.xodr")` needs one resolution rule the
  checker did not have: a bare **actor type name** is a receiver in its own
  right, because the map is a singleton no scenario declares a field for.
