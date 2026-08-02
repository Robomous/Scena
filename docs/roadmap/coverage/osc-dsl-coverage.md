# ASAM OpenSCENARIO DSL 2.x — v0.0.1 coverage matrix

Normative scope declaration for the DSL frontend (P7) and DSL execution
(P8). Section numbers follow the ASAM OpenSCENARIO DSL 2.2.0 text (the
local reference copy; 2.2.0 is the current published 2.x version). The DSL
spec defines no `asam.net:` rule IDs, so diagnostics cite section numbers.

Two capability columns, because the DSL pillar split makes them distinct:

- **Check** (P7): parsed, resolved, and type-checked — `scena-check`
  accepts/rejects correctly with §-cited diagnostics.
- **Exec** (P8): executable for **attribute-level concrete scenarios**
  (§6.3.1.2.1 — all attribute values fixed; the engine synthesizes
  motion). The spec's semantics are trace-acceptance based (§7.6.1); a
  deterministic executor is one conformant operational choice.

Statuses: **In** (v0.0.1, tested), **Post** (post-v0.0.1; structured
`UnsupportedFeature` diagnostic when encountered at that layer), **Excl**
(excluded with reason), **n/a** (nothing to execute — a static-only
construct).

Headline honesty rule: **"concrete scenarios only" excludes** — range and
distribution *selection*, `keep()` constraints requiring search (anything
but fixed-value resolution), coverage-driven generation, and external
methods. All of these still **Check** cleanly; execution diagnoses them.

## Checking surfaces

Everything below is reachable from all three of Scena's surfaces. Checking
needs no engine — it is a frontend service, so none of these entry points
construct one.

| Surface | Check | Sprint(s) | Notes |
|---|---|---|---|
| C++ — `scena::dsl::check_file` / `check_source` | In | p7-s5 | Load and resolve in one call; the `LoadResult` owns the ASTs the `Program` points into (`dsl_import_test.cpp`, `dsl_stdlib_test.cpp`) |
| CLI — `scena-check <file.osc>` | In | p7-s5 | `-I` search paths, `--no-standard-library`, `--strict`, `--quiet`; exit codes 0 ok / 2 usage / 3 the source did not check / 4 the input could not be read (`scena_check_test.cpp`) |
| C ABI — `scn_check_dsl_file` / `scn_check_dsl_string` | In | p7-s5 | Opaque `scn_dsl_check` handle carrying the diagnostics and the two counts; a failing check still produces one, because that is the case whose findings you want (`c_consumer.c`) |
| Python — `scena.check_dsl_file` / `check_dsl_string` | In | p7-s5 | Returns a `DslCheck` — status, diagnostics, `type_count`, `file_count` (`test_dsl_check.py`, `python/examples/check_dsl.py`) |

## Lowering to the IR (P8)

The DSL and the XML frontend compile into one Scenario IR, so nothing in this
table decides runtime semantics — only which DSL construct denotes which IR
construct (ADR-0030). Lowering is inside the determinism contract, because load
time is.

| Feature | Section | Check | Exec | Sprint(s) | Notes |
|---|---|---|---|---|---|
| Actor field → IR entity | §8.7 | In | In | p8-s1 | **Landed** (`dsl_lowering_test.cpp`): every entry-scenario field whose type derives from `std::physical_object` becomes one entity, in declaration order, engine-controlled. `vehicle`/`person`/`stationary_object` classify onto the p2-s1 taxonomy; anything else deriving from `physical_object` (§8.7.10's `animal`) stays an unclassified participant rather than being misfiled |
| Concrete value binding | §7.3.11 | In | In | p8-s1 | **Landed**: `keep(<field-path> == <constant>)` in either operand order, where the constant folds without a solver — that is what §6.3.1.2.1's "attribute-level concrete" means. §7.3.8.2 conditional inheritance is read the same way. Anything needing search is diagnosed, never approximated (ADR-0004) |
| Physical values in the IR | §7.3.4 | In | In | p8-s1 | **Landed**: values arrive already folded to their base unit, so lowering never converts and never re-applies the standard's printed factors a second time (ADR-0029) |
| Performance limits | §8.7 | n/a | Excl | p8-s1 | §8.7 declares no performance limits at all — the domain model has no counterpart to XML's `Performance`. The IR's zeros are the faithful lowering: the runtime reads a non-positive limit as unconstrained. No numbers are invented |
| §8.8 movement actions → IR actions | §8.8.2–§8.8.4 | In | In | p8-s1 | **Landed** (`dsl_lowering_test.cpp`): `assign_speed`, `change_speed`, `remain_stationary`, `assign_position`, `change_lane`, `change_space_gap`, `change_time_gap`. `dynamic_profile` names the shape and `rate_peak` the magnitude; with no peak rate there is no number to ramp over, so the change is a Step (§7.4.1.2). Every action with no counterpart is reported by name, never silently dropped |
| Generic actions (`move`, `drive`, `walk`) | §8.8.2.3, §8.8.3.1, §8.8.4.1 | In | In | p8-s1, p8-s3 | **Landed**: they carry no target of their own and exist to be shaped by §8.9 modifiers, so on their own they lower to nothing — "keep doing what you are doing" is what the runtime already does |
| Struct-valued action arguments | §7.2.2.6.7 | In | In | p8-s1 | **Landed**: the DSL declares list and range constructors and no struct constructor, so a struct-valued argument names a declaration and the `keep`s on it are the value (ADR-0030 §7). A coordinate nothing constrains is reported, not assumed |
| `do` directive → storyboard | §7.6.2.1 | In | In | p8-s1, p8-s2 | **Landed** (`dsl_lowering_test.cpp`): one Story, one Act, one ManeuverGroup per phase, nesting to any depth. A composition contributes exactly one thing — the start trigger on a phase's Event — so the runtime is unchanged (ADR-0031) |
| Serial composition | §7.6.2.1.2 | In | In | p8-s2 | **Landed**: a member starts when its predecessor ends. With concrete durations that is an absolute `SimulationTimeCondition` computed at load; otherwise the predecessor group's completeState |
| Parallel composition | §7.6.2.1.4 | In | In (default overlap) | p8-s2 | **Landed**: the default `overlap: start` is what absent start triggers already give, and the join is one ConditionGroup (an AND) over the members' ends. The other seven overlap kinds and `start_to_start`/`end_to_end` are reported, not realised |
| One-of composition | §7.6.2.1.3 | In | In | p8-s2 | **Landed**: the alternative is an *input* (`LowerOptions::alternative`, fed by `scena-run --select`), defaulting to the first in declaration order. Picking at random would put a hidden input in the run, and the engine has no seed machinery (ADR-0031) |
| `duration` on an invocation or composition | §7.6.2.4, §7.6.2.4.1 | In | Concrete only | p8-s2 | **Landed**: a constant duration becomes absolute phase boundaries; a range (`[10s..30s]`) constrains accepted traces rather than fixing a time, so it is reported and needs a solver (ADR-0004) |
| `wait elapsed(<duration>)` | §7.6.2.4.2 | In | In | p8-s2 | **Landed**: a phase in which nothing is specified lowers to nothing — no group, no event — and only advances the offset the next phase starts from. The clock is already running |
| `emit` / `wait @event` / `on` / `until` | §7.6.2.5 | In | Post | p8-s2 | Reported: §7.6.2.5's events are abstract control objects with no runtime carrier in v0.0.1. The trigger system has no event namespace, and inventing one would be runtime machinery only the DSL frontend can reach |
| Range constructor `[a..b]` | §7.2.2.6.7 | In | In | p7-s1, p8-s2 | **Fixed in p8-s2** (`dsl_lexer_test.cpp`): `..` has to beat `float-literal ::= digit* '.' digit+`, whose leading digits are optional, or `[2..4]` lexes as `2`, `.`, `.4`. The lexer emitted no `..` at all and the parser looked for `...`; they agreed with each other, so nothing noticed until a duration range was written |
| `set_map_file` → road backend | §8.5.4, §8.12.2 | In | In | p8-s1 | **Landed** (`dsl_lowering_test.cpp`, `scena_run_test.cpp`): both spellings the standard prints — `map.set_map_file("m.xodr")` (Code 61) and `keep(my_map.map_file == "m.xodr")` (Code 62). The reference travels beside the IR, as `RoadNetwork/LogicFile` does, because a road-network path is a host input not kernel state (ADR-0003) |
| Actor type name as a modifier receiver | §7.3.12.4.1, §8.5.4 | In | n/a | p8-s1 | **Landed** (`dsl_types_test.cpp`): `map` in Code 61 names the actor *type* — the road network is a singleton no scenario declares a field for — so a bare actor type name is a receiver in its own right. A declared field of the same name still wins |
| XML↔DSL trace parity | — | n/a | In | p8-s4 | **Landed**: `scripts/golden.py compare-pair` runs a declared pair — one scenario authored in both languages — and asserts the traces are equal byte for byte. Part of `check-all`, so CI verifies it on three platforms |
| DSL golden scenarios | — | n/a | In | p8-s4 | **Landed**: GS-12 (`gs12-dsl-cruise.osc`, with its XML twin) and GS-13 (`gs13-dsl-alternatives.osc` — a parallel nested in a serial, a placement modifier, a relative-speed modifier and a `one_of` chosen by input) |
| `scena-run` runs `.osc` | — | n/a | In | p8-s1 | **Landed** (`scena_run_test.cpp`): the extension picks the frontend, `--entry` names the §7.7.2 entry point, `-I` adds an import search path. Same options, exit codes and trace format as XML |

## Language core (§7.2, §7.3)

| Feature | Section | Check | Exec | Sprint(s) | Notes |
|---|---|---|---|---|---|
| Lexical structure (indentation blocks, comments, continuation, escaped identifiers) | §7.2.1 | In | n/a | p7-s1 | **Landed** (`dsl_lexer_test.cpp`): logical lines with explicit `\` and implicit bracket joining, CR/LF/CRLF, offside rule → INDENT/DEDENT with 8-column tab stops, `#` comments, both identifier forms. No `Keyword` token kind — §7.2.1.5.1 makes reserved words positional, so the parser decides (ADR-0027) |
| Literals (int/uint/hex/float/bool/string/physical) | §7.2.1.5.2 | In | In | p7-s1, p8-s1 | **Lexing landed** (`dsl_lexer_test.cpp`): uint/int/hex with exact 64-bit values, every float form incl. `inf`/`nan`, short and long strings with escapes, physical literal = number joined to a unit name with no intervening whitespace. `std::from_chars`, never locale-dependent parsing |
| Grammar (declarations, structured-type members, behavior specification, expression ladder) | §7.2.2 | In | n/a | p7-s2 | **Landed** (`dsl_parser_test.cpp`): recursive-descent parser over the §7.2.2 productions → AST (`ast.h`), left recursions rewritten as loops per the spec's own note. Error recovery is contractual — a parse error resynchronises at the end of the logical line or block and the parse continues, so one run reports many §-cited diagnostics (ADR-0027) |
| Physical types + units, SI dimensions, conversion factors/offsets | §7.2.2.2.1, §7.3.4 | In | In | p7-s3, p8-s1 | **Checking landed** (`dsl_types_test.cpp`): SI exponent vectors, a unit must carry its type's dimension, `value * factor + offset` conversion, unit names in their own global namespace |
| Enums (incl. `enum!member`, extension) | §7.3.3 | In | In | p7-s3, p8-s1 | **Checking landed**: implicit value succession, the `= other_member` reference form with cycle detection, extension continuing the numbering, overloaded literals enumerated in name order |
| Primitive types (bool, int, uint, float, string) | §7.3.2 | In | In | p7-s3 | **Checking landed**: always present, never namespaced (§7.3.2); int/uint→float implicit in a literal default. IEEE 754 float; 64-bit int/uint |
| Structs | §7.3.5.1.1 | In | In | p7-s3, p8-s1 | **Checking landed**; a struct carries no `do` directive |
| Actors | §7.3.5.1.2 | In | In | p7-s3, p8-s1 | **Checking landed** |
| Scenarios (assoc. actor, ≤1 `do`, n `on`) | §7.3.5.1.3 | In | In | p7-s3, p8-s1 | **Checking landed**: the actor prefix resolves to an actor; more than one `do` is reported, including one arriving by extension |
| Actions (atomic behaviors) | §7.3.5.1.4, §7.6.3 | In | In (matrix subset) | p7-s3, p8-s1 | Action internals are implementation-defined per §7.6.3; see §8.8 table |
| Modifier declarations + application | §7.3.12 | In | In (matrix subset) | p7-s3, p8-s3 | **Checking landed**: all three association types (§7.3.12.3), `of` names a scenario or action, applications resolve and their named arguments must name parameters. Argument = equality constraint (§7.3.12.4) |
| `override()` atomic modifier | §7.3.12.1.1 | In | Post | p7-s3 | Maps to XML override/skip machinery; wiring deferred with a documented diagnostic |
| Lists (+ list operators) | §7.3.5.2.1, §7.4.2.7 | In | In | p7-s3, p7-s4 | **Declarator checking landed** (p7-s3): structural and shared, and a list of lists is reported (§7.3.1). Operators in p7-s4 |
| Ranges (`[a..b]`, range operators) | §7.3.5.2.2, §7.4.2.8 | In | Partial → see keep/ranges row | p7-s3, p7-s4 | **Declarator checking landed** (p7-s3): `range of` needs a numeric base type (§7.3.1). Operators in p7-s4 |
| Fields: parameters vs `var` variables, `with:` blocks | §7.3.6 | In | In | p7-s3, p8-s1 | **Checking landed**: typed, ordered, unique, non-shadowing; a literal default is checked against the declared type and unit. Parameters fixed at init; variables runtime-mutable |
| `sample()` variable initializer | §7.3.10.4 | In | Post | p7-s4 | **Checking landed** (`dsl_constraint_test.cpp`): the sampled expression and event spec are typed, and an `UnsupportedFeature` note says it is not executed. Event-sampled variables deferred (needs event-valuation plumbing beyond v0.0.1 set) |
| `keep(hard/default)` constraints | §7.3.11 | In | **Concrete-value only** | p7-s4, p8-s1 | **Checking landed** (`dsl_constraint_test.cpp`): typed as Boolean (§7.3.11.1); a constant-false hard keep is an error and a constant-false `default` keep a warning (§7.3.11.3); `f == const`, `f in <const>` and conjunctions of those resolve; everything else gets an `UnsupportedFeature` note (ADR-0004) |
| `remove_default()` | §7.3.11 | In | In (within concrete resolution) | p7-s4 | **Checking landed**: must name a parameter of the enclosing type |
| Methods (expression, `undefined`, override rules) | §7.3.7 | In | In (expression methods in constant contexts) | p7-s3 | **Signature checking landed**: parameters and return type resolved; `is only` requires a supertype method and keeps its return type (§7.3.7.2). Bodies type-check in p7-s4 |
| External methods (`is external ...`) | §7.3.7.4 | In (parse/check) | Post | p7-s3, p7-s4 | **Signature checking landed**; the body is not typed because there is none. Host-binding FFI is post-release; invocation diagnosed |
| Inheritance (single; conditional `inherits X(f == v)`) | §7.3.8 | In | In | p7-s3 | **Checking landed**: single, same-kind, cycle-broken; guards need a bool or enum determinant that the base declares; Rule 1 (§7.3.8.2.3) enforced; actor-behavior inheritance restricted per §7.3.8.1. Latent subtypes via `is()`/`as()` (p7-s4) |
| Extension (`extend`) | §7.3.9 | In | In | p7-s3 | **Checking landed**: members merge into the extended type, in any textual order, and cannot shadow existing ones. Compile-time composition |
| Events (`event`, predefined start/end/fail) | §7.3.10 | In | In | p7-s3, p8-s2 | Predefined events map to storyboard element transitions |
| Event specifications (`@`, `rise`, `fall`, `elapsed`, `every`, `if`, `as` binding) | §7.3.10.4 | In | In except `every` | p7-s2, p8-s2 | **Parsing landed** (`dsl_parser_test.cpp`): all five condition forms plus the `as` binding and `if` guard. `every` (periodic) Post: no v0.0.1 scenario needs it; deterministic period machinery deferred |
| Global parameters | §7.3.14 | In | In | p7-s3 | **Checking landed**: typed, ordered, unique; `it` has no instance to bind to in one (§7.4.1.3) |
| Type resolution order (declare-anywhere) | §7.3.15 | In | n/a | p7-s3 | **Landed**: three passes — declare, link, members (ADR-0028). Use may precede declaration for types, units and extensions alike |
| Namespaces + `::`, export rules | §7.7.4 | In | n/a | p7-s3 | **Landed**: `namespace ... use`, explicit `ns::name`, export lists and wildcards, the current namespace shadowing the use list (§7.7.4.2), ambiguity across two used namespaces reported, `std`-prefixed namespaces warned, each file starting in the null namespace |
| Import (URI + identifier forms; `osc.standard.all/types/domain`, legacy `osc.standard`) | §7.7.5 | In | n/a | p7-s2, p7-s5 | **Landed** (`dsl_import_test.cpp`): both reference forms resolved, `file` URIs (`file:///p`, `file:/p`, bare) with relative references anchored to the referencing file, module references mapped `a.b.c` → `a/b/c.osc` over configured search paths, import-once by canonical path so a diamond declares once and a cycle terminates (§7.7.5.1), referenced files ordered before the referencing file, `osc`-prefixed references reserved (§7.7.5.1.2) |
| Standard-library access (built-in definitions; auto-use) | §7.7.5.2 | In | n/a | p7-s5 | **Landed** (`dsl_import_test.cpp`, `dsl_stdlib_test.cpp`): the types sub-module is provided as built-in definitions, the route §7.7.5.2 permits, with `stdtypes` auto-used in the null namespace (§7.7.5.2.3) so a physical literal types before any import; all four module references accepted; a namespace statement restores the ordinary §7.7.4 use-list rules (ADR-0029) |
| Scenario entry-point selection | §7.7.2 | n/a | In | p8-s1 | **Landed** (`dsl_lowering_test.cpp`): §7.7.2 leaves the choice to the implementation. `LowerOptions::entry_point` takes a scenario by qualified name or as written; empty means the root file's only scenario, and a file declaring several is an error that lists them rather than guessing (ADR-0030). `entry_points()` returns the same list in declaration order |

## Expressions (§7.4)

| Feature | Section | Check | Exec | Sprint(s) | Notes |
|---|---|---|---|---|---|
| Atomic (identifiers, literals, `it`) | §7.4.1 | In | In | p7-s4 | **Landed** (`dsl_expression_test.cpp`): fields, method/event parameters, globals and enum literals; `it` is the enclosing instance, the list member inside a member-evaluation operator, or the parameter inside its `with:` block |
| Logical (short-circuit), arithmetic (+ numeric conversion rules), relational + `in`, ternary, `=>` | §7.4.2.2–.4, .9 | In | In | p7-s4 | **Landed**: §7.4.2.3.1's conversion ladder (float wins, int+uint→int, unary minus on uint→int), ordering restricted to numerics (§7.4.2.4.2), ternary arms need a common type. Physical types never implicitly converted — a physical literal folds to its base unit so units compare directly |
| `is()` / `as()` | §7.4.2.5–.6 | In | In | p7-s4 | **Landed**: `is()` yields bool, `as()` the target type, and a conversion between unrelated types is rejected statically. Cast failure = error |
| Method application | §7.4.2.1 | In | In (constant contexts + In-scope library methods) | p7-s4 | **Landed**: resolved against the receiver's type and its supertypes, arity checked, arguments typed; an expression body must return its declared type |
| List/range operators | §7.4.2.7–.8 | In | In | p7-s4 | **Landed**: `size`/index, the five member-evaluation operators with their `it` binding, list construction with §7.4.2.7.4's common type and flattening, both range constructor forms with value-ordered bounds, `lower`/`upper`, and `in` over both |

## Coverage constructs (§7.5)

| Feature | Section | Check | Exec | Sprint(s) | Notes |
|---|---|---|---|---|---|
| `cover()` / `record()` / cross / args / override / grading | §7.5 | In | Post | p7-s4 | **Checking landed** (`dsl_constraint_test.cpp`): arguments are typed and every declaration carries an explicit `UnsupportedFeature` note. Collection not performed in v0.0.1. `target`/grading exist to steer generation — post-release with the solver family |

## Behavior composition & semantics (§7.6)

| Feature | Section | Check | Exec | Sprint(s) | Notes |
|---|---|---|---|---|---|
| Direct invocation in `do` | §7.6.2.1.1 | In | In | p7-s2, p8-s2 | **Parsing landed** (`dsl_parser_test.cpp`): actor-qualified invocation, arguments, and the `with:` block (modifiers, constraints, `until`) |
| `serial` | §7.6.2.1.2 | In | In | p8-s2 | Member starts when predecessor ends |
| `parallel` (default `start` overlap; duration) | §7.6.2.1.4 | In | In | p8-s2 | All-members-complete join |
| `parallel` non-default overlap kinds + `start_to_start`/`end_to_end` offsets | §7.6.2.1.4 | In | Post | p8-s2 | Declared honestly: only default overlap executes in v0.0.1 |
| `one_of` | §7.6.2.1.3 | In | In | p8-s2 | Deterministic host-selected alternative (default: first) — any single branch is a valid acceptance; randomness barred by the determinism contract. The exact operator set in 2.2.0 is serial/parallel/one_of — there is no `first_of` or `mix` |
| Labels on do-members | §7.2.2.4.7 | In | In | p7-s2, p8-s2 | **Parsing landed** (`dsl_parser_test.cpp`) on compositions and invocations alike |
| `duration` bounds on compositions/invocations | §7.6.2.4 | In | In | p8-s2 | |
| `until` | §7.6.2.5.4 | In | In | p8-s2 | Terminates the annotated invocation exactly at the event; first-of-any for multiples |
| `wait` | §7.6.2.5.3 | In | In | p8-s2 | Pure synchronization |
| `emit` | §7.6.2.5.2 | In | In | p8-s2 | Zero-time; eager parameter evaluation |
| `on` directives (call/emit reactions) | §7.6.2.5.5 | In | In | p8-s2 | |
| `call` | §7.2.2.4.7 | In | In (In-scope methods) | p8-s2 | |
| Actor-binding rules for scenario parameters | §7.6.2.2 | In | In | p8-s1 | Constant binding over invocation lifetime |
| Constraint sets over invocation lifetime | §7.6.2.3 | In | Concrete-value only | p8-s1 | See keep row |

## Standard library (§8) — checking

The entire §8 library (namespaces `stdtypes` + `std`) **type-checks** in
v0.0.1 — that is the P7 pillar gate (p7-s5). The library is authored from the
normative §8 *document* text, not from the `types.osc` / `domain.osc` /
`standard.osc` files, which §8.16 itself declares non-normative (ADR-0002,
ADR-0029). Its conversion factors are the ones §8.14.1 prints, rounded decimals
included, so a conforming implementation and Scena agree.

| Sub-module | Section | Check | Sprint | Notes |
|---|---|---|---|---|
| `stdtypes` — scalar physical types and units | §8.14.1 | In | p7-s5 | **Landed** (`dsl_stdlib_test.cpp`): all 16 scalar types with their SI dimensions and every printed unit spelling |
| `stdtypes` — compound structs | §8.14.2 | In | p7-s5 | **Landed**: the position/orientation/velocity/acceleration compounds, `position` inheritance, `norm()` on the three translational compounds |
| `stdtypes` — string methods | §8.13 | In | p7-s5 | **Landed**: declared on the `string` primitive. §8.13 heads them under the types sub-module and then names the `std` namespace; declared under the section they appear in, and nothing observable turns on it — a method on a primitive is reached through a value, never through a namespace |
| `std` — physical-object actors, structs and enums | §8.7 | In | p7-s5 | **Landed** (`dsl_stdlib_test.cpp`): the `osc_actor` → `physical_object` → `movable_object` → `traffic_participant` → `vehicle` → `trailer` chain with `stationary_object`, `person` and `animal`; `bounding_box`, `axle`, `hitch_receiver`, `hitch_coupler`; all 11 enums including the backward-compatibility spellings (`truck = heavy_truck`, `fire = fire_brigade`, …) sharing their replacement's value; the §8.7.6.1 measurement methods. §8.7.26 (traffic-participant groups) is non-normative and excluded |
| `std` — road abstraction classes | §8.12.3–§8.12.41 | In | p7-s5 | **Landed** (`dsl_stdlib_test.cpp`): the `route` → `route_element` hierarchy with `road`, `lane_section`, `lane`, `crossing`, the four point structs, `path`/`trajectory` and their relative variants, `compound_route`/`compound_lane`, `junction`, and 15 enums. The road-dependent `physical_object` and `traffic_participant` methods arrive here as `extend` blocks, which is how the standard prints their prototypes |
| `std` — the `map` actor | §8.12.2 | In | p7-s5 | **Landed** (`dsl_stdlib_test.cpp`): the top-level road-network actor with its six fields, its 18 conversion and creation methods, and its 12 search-space modifiers. Applying them works as of the #100 fix: the modifier name lives in the actor scope (§7.3.12.2) and is reached through the receiver. Worksheet: `docs/dev/stdlib-worksheets/08-12-02-map.md` |
| `std` — action hierarchy and the environment | §8.8.1, §8.10, §8.11 | In | p7-s5 | **Landed** (`dsl_stdlib_test.cpp`): `osc_action` with `action_for_environment` and `action_for_movable_object`; the `environment` actor with `local_to_unix_time`; the `weather`/`air`/`precipitation`/`wind`/`fog`/`clouds`/`celestial_light_source` structs; the seven §8.11 environment actions. Their *execution* stays Post (see the actor table) — the DSL environment actions are checked, not run, in v0.0.1 |
| `std` — traffic lights | §8.15 | In | p7-s5 | **Landed** (`dsl_stdlib_test.cpp`): `traffic_light_bulb`, `traffic_light` and `traffic_light_group` with their state methods, `traffic_light_stop_line` (a `route_element`), `traffic_light_phase`/`traffic_light_cycle`, the `traffic_light_controller` actor and its seven §8.15.9 actions, and 5 enums. Their *execution* stays Post (see the actor table). Worksheet: `docs/dev/stdlib-worksheets/08-15-traffic-lights.md` |
| `std` — movement actions | §8.8.2–§8.8.4 | In | p7-s5 | **Landed** (`dsl_stdlib_test.cpp`): 15 actions for `movable_object`, 13 for `vehicle` and `walk` for `person`, plus `dynamic_profile`, `lane_change_side`, `gap_direction` and `headway_direction`. `action_for_vehicle` and `action_for_person` are declared here — §8.8.3/§8.8.4 name them as parents but only §8.8.1's prose defines them. Their *execution* stays Post except where the roadmap's action table says otherwise. Worksheet: `docs/dev/stdlib-worksheets/08-08-movement-actions.md` |
| `std` — movement modifiers | §8.9 | In | p7-s5 | **Landed** (`dsl_stdlib_test.cpp`): the `any_shape`/`common_*_shape` hierarchy, the seven §8.9.19–§8.9.25 enums, and 17 modifiers each carrying §8.9.1.1's four common parameters (`at`, `movement_mode`, `track`, `shape`). Fourteen are actor-associated per §7.3.12.3; `change_speed`, `keep_speed` and `change_lane` are unassociated because §8.8 already declares actions of those names on those actors and a qualified behavior name identifies one declaration — the collision is the standard's. Applying them works as of the #100 fix, in both of §7.3.12.4.1's positions. Worksheet: `docs/dev/stdlib-worksheets/08-09-movement-modifiers.md` |

Coverage of the individual §8 declarations: all physical types and units
(§8.14.1), compound structs (§8.14.2), string methods (§8.13), all actors
(§8.7: osc_actor, physical_object, stationary_object, movable_object,
traffic_participant, vehicle, trailer, person, animal, vehicle_group,
environment, map, traffic_light_controller), all actions (§8.8, §8.11,
§8.15.9), all modifiers (§8.9), and the map/route/environment/traffic-light
type families (§8.10, §8.12, §8.15). "Check = In" is therefore implicit for
every §8 row below; the Exec column is the declared execution scope.

### Actors — execution

| Actor | Section | Exec | Sprint | Notes |
|---|---|---|---|---|
| vehicle (+ vehicle_category) | §8.7.7 | In | p8-s1 | → IR vehicle taxonomy |
| person | §8.7.9 | In | p8-s1 | → IR pedestrian |
| stationary_object / movable_object base data | §8.7.4–.5 | In | p8-s1 | → IR misc object / base fields |
| traffic_participant metric methods (time_to_collision, time_headway, time_gap, space_gap, space_headway) | §8.7.6.1 | In | p8-s2 | Backed by the P5 measurement machinery |
| physical_object distance methods (object_distance, road_distance, get_s_coord, get_t_coord, …) | §8.7.3.1 | In | p8-s2 | Backed by P3 `IRoadQuery` |
| animal | §8.7.10 | Post | — | No dedicated motion model in v0.0.1 |
| trailer (+ hitch structs) | §8.7.8 | Post | — | Trailer model out of entity scope (matches XML decision) |
| vehicle_group | §8.7.26.1 | Post | — | Group orchestration deferred |
| environment (weather/air/precipitation/wind/fog/clouds structs) | §8.10 | Post | — | State-only in XML path; DSL environment actions deferred |
| map actor: set_map_file, create_route/point/path/trajectory, conversions | §8.12.2 | In (listed methods) | p8-s1 | `set_map_file` binds the P3 backend; creation/conversion methods over `IRoadQuery` |
| map search-space modifiers (number_of_lanes, routes_are_in_sequence, …) | §8.12.2.2 | Post | — | Abstract map-matching (search-space binding) belongs to the solver family; `set_map_file` + concrete routes cover concrete execution |
| traffic_light_controller | §8.15.8.1 | Post | — | DSL signal control deferred; XML signal actions cover v0.0.1 (kernel phase model is shared groundwork) |

### Actions — execution (§8.8, §8.11, §8.15.9)

| Action | Section | Exec | Sprint | Notes |
|---|---|---|---|---|
| move (generic, modifier-shaped) | §8.8.2.3 | In | p8-s1/p8-s3 | The modifier carrier |
| assign_position / assign_orientation / assign_speed | §8.8.2.4–.6 | In | p8-s1 | Teleport-family → IR |
| change_position / change_speed / keep_speed | §8.8.2.11–.13 | In | p8-s1 | → P2/P5 longitudinal machinery |
| assign_acceleration / change_acceleration / keep_acceleration | §8.8.2.7, .14–.15 | Post | — | Direct acceleration-target actions deferred (speed-target machinery covers the golden set) |
| follow_path / follow_trajectory | §8.8.2.16–.17 | In | p8-s1 | → P2-s5 trajectory machinery |
| replay_path / replay_trajectory (+ relative variants) | §8.8.2.8–.9 | Post | — | Timestamped replay deferred; overlaps host-controlled replay capability |
| remain_stationary | §8.8.2.10 | In | p8-s1 | Trivial on the runtime |
| drive (generic vehicle action) | §8.8.3.1 | In | p8-s1/p8-s3 | The GS-12 workhorse |
| follow_lane / change_lane | §8.8.3.2–.3 | In | p8-s1 | → lane machinery |
| change/keep_time_gap, change/keep_space_gap | §8.8.3.4–.7 | In | p8-s1 | → LongitudinalDistanceAction machinery |
| change/keep_time_headway, change/keep_space_headway | §8.8.3.8–.11 | Post | — | Headway-hold variants deferred; gap family covers the golden set |
| connect_trailer / disconnect_trailer | §8.8.3.12–.13 | Post | — | With trailers |
| walk | §8.8.4.1 | In | p8-s1 | → pedestrian motion |
| environment actions (air, rain, snow, wind, fog, clouds, assign_celestial_position) | §8.11 | Post | — | With the environment family |
| traffic-light actions (set_bulb_state … play_cycles) | §8.15.9 | Post | — | With DSL signal control |

### Movement modifiers — execution (§8.9)

Common machinery: `at` phase anchoring (§8.9.19) In; `movement_mode`,
`track`, shape structs (§8.9.1.2) Post (default profiles only —
documented); scalar params In, `*_range` params Post (range-valued
envelopes belong to the logical level).

| Modifier | Section | Exec | Sprint | Notes |
|---|---|---|---|---|
| position() | §8.9.2 | In | p8-s3 | **Landed** (`dsl_lowering_test.cpp`): `ahead_of`/`behind` with a concrete distance. The anchor changes the *kind* of action — at the start a `TeleportAction` placement, over the phase a `LongitudinalDistanceAction` gap (ADR-0032). Point forms and `*_range` Post |
| keep_position() | §8.9.3 | In | p8-s3 | **Landed**: lowers to *no* action. The runtime already holds relative position between actions, so one that set the current value would be a no-op that still occupies the action domain (§7.5) |
| speed() / change_speed() | §8.9.4–.5 | In | p8-s3 | **Landed**: absolute, and the `faster_than`/`slower_than`/`same_as` relative forms with `factor` → `RelativeTargetSpeed`. `change_speed` is relative to the actor's own speed. Ranges are reported |
| keep_speed() | §8.9.6 | In | p8-s3 | **Landed**: lowers to no action, same reason as `keep_position` |
| acceleration() | §8.9.7 | Post | — | Reported: it shapes an acceleration the phase is already performing, and the IR has no acceleration-target action — the same reason §8.8's are deferred |
| lateral() | §8.9.8 | In | p8-s3 | **Landed**: `side_of` → `LateralDistanceAction`, otherwise a `LaneOffsetAction` from the lane centre. Positive offsets are to the left (§7.4.1.4), so `side: right` is negative |
| lane() / change_lane() / keep_lane() | §8.9.14–.16 | In | p8-s3 | **Landed**: a lane number → an absolute lane target, `side_of`+`side` → a relative one, `keep_lane` → a continuous zero `LaneOffsetAction`. `change_lane` without an explicit side is diagnosed, not chosen (determinism) |
| along() / along_trajectory() | §8.9.11–.12 | Post | — | Reported: both need a concrete route or trajectory value, and the DSL has no struct constructor (§7.2.2.6.7) — one can only come from §8.12.2's map methods, which the standard says may be external (§7.3.7.4) |
| distance() | §8.9.13 | Post | — | Reported: it bounds a phase by distance travelled, and phase sequencing is by time (ADR-0031) |
| `at` phase anchoring | §8.9.1.1.1, §8.9.19 | In | p8-s3 | **Landed**: absent/`all`/`start` set the value when the phase begins (a Step); `end` reaches it over the phase, which needs the phase length p8-s2's duration fixes. Without one it is reported, never invented |
| Overloaded enum literal resolved by context | §7.3.3 | **Gap** | — | §7.3.3 says the literal "will depend on the type requirements of the place it is used in"; Scena reports the ambiguity first, so the standard's own `at: start` is rejected while `at: at!start` works. Tracked as #110; blocks nothing |
| yaw() / orientation() | §8.9.9–.10 | Post | — | Orientation-target modifiers deferred (teleport orientation covers placements) |
| physical_movement() | §8.9.17 | Post | — | Single documented default profile in v0.0.1 |
| avoid_collisions() | §8.9.18 | Post | — | No collision-avoidance controller in v0.0.1 (engine executes what the scenario says; Collision condition detects) |

## What "concrete scenarios only" excludes (summary)

Checked but not executed in v0.0.1, all with structured diagnostics:
range/distribution *selection* (logical scenarios, §6.3.1.2.2), `keep()`
requiring search, coverage collection and coverage-driven generation
(§7.5), abstract map-matching modifiers (§8.12.2.2), external methods
(§7.3.7.4 — note Annex C shows all distributions are built on external
methods, so distribution workflows are inherently post-release),
`sample()`/`every` event machinery, non-default parallel overlap kinds.
Constraint solving and abstract-scenario generation are post-v0.0.1 and
feasibility-gated (ADR-0004).

## Declared coverage summary (DSL 2.2.0)

- **Check (P7): 100 %** of the language (§7) and standard library (§8) —
  the full-library type-check is the p7-s5 pillar gate.
- Exec, language/composition constructs: **serial, parallel (default
  overlap), one_of, until, wait, emit, on, call, duration, labels, events**
  In — 11 of 14 behavior constructs (79%); `every`, non-default overlaps,
  `override()` modifier Post.
- Exec, §8.8 movement actions: **13 of 24** In (54%); the deferred 11 are
  acceleration-target, replay, headway-hold, trailer, environment, and
  signal families.
- Exec, §8.9 movement modifiers: **13 of 18** In (72%); scalar forms only.
