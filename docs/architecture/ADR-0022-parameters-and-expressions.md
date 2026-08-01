# ADR-0022: Parameters, expressions and load-time determinism (p4-s3)

## Status

Accepted (p4-s3, #26).

## Context

OpenSCENARIO files are written to be reused: §9.1 parameters give a document
its knobs, and §9.2 expressions compute from them. Both are resolved **at
load time**, which puts them on the wrong side of an easily missed line — an
expression result is not a display value, it is a number that lands in the
Scenario IR and drives the simulation. Whatever computes it is therefore
part of the determinism contract, exactly like the runtime is.

## Decision

- **Resolution lives in the attribute readers.** `ReadContext` owns a
  `ParameterScope`, and every attribute read — double, integer, boolean,
  string, enumeration — resolves a whole-token `$reference` or a
  `${expression}` before parsing. Every attribute of every element therefore
  supports parameters, which is what the standard says ("every attribute of a
  language element may contain either the actual attribute as a literal, a
  reference to a parameter, or an expression"), and no reader can forget to.
- **Scope is a stack of frames.** Push on entering an element that declares
  parameters (`Story`, `Maneuver`, `Trajectory`, `Route`, `Environment`, an
  entity object), pop on leaving, look up innermost first. That is §9.1's
  "smallest scope that subsumes the location" verbatim, and it makes
  shadowing fall out rather than needing a rule.
- **Declarations are typed and checked.** `parameterType` is parsed, the
  value must be of that type (rule
  `parameters.parameter_declaration_parameter_type_inference`), names must
  match `[A-Za-z_][A-Za-z0-9_]*`, the reserved `OSC` prefix warns, and
  `ConstraintGroup`s are enforced as a disjunction of conjunctions.
- **Expressions are evaluated deterministically or not at all.** The
  evaluator is a recursive-descent parser over §9.2.1's precedence ladder
  with exactly the whitelisted operators. Its transcendentals go through
  `runtime::detmath` — `sin`/`cos` directly, `tan` as their quotient,
  `atan`/`asin`/`acos` derived from `det_atan2` — never libm. `sqrt`, the
  remainder and integer arithmetic are IEEE-exact.
  - **`pow` accepts integer exponents only**, evaluated as exact repeated
    multiplication. A general power needs a platform `exp`/`log` pair, which
    is not bit-identical across platforms; Scena reports it rather than
    approximating it and quietly making two machines disagree.
  - `scripts/check_detmath.sh` now covers `frontends/**` for the same
    reason: the guard follows the values that reach the IR, not the
    directory they were computed in.
- **`pi` is rejected.** The constant is a 1.4 addition and Scena targets
  1.0–1.3; the version gate (p5-s1 follow-up) decides, so it becomes
  available for free if the targeted range ever moves.
- **Integers stay integers.** The evaluator carries an integer value
  separately from a double one, so integer arithmetic never round-trips
  through a double. The single implicit conversion is the widening the
  standard allows (rule `type_casting`); narrowing is a type error.
- **Variables are not expression inputs.** A `VariableDeclaration`'s initial
  value is resolved at load time like any attribute, but a `$reference` to a
  variable is an undeclared-parameter error. Variables change during the run
  (§6.12), so reading one into an attribute would freeze a moving value at
  load time and read differently than the same reference at runtime would.

## Consequences

- A document may compute anything the standard allows, and two loads of it
  produce the same IR bit for bit, on every platform — the property the
  expression suite asserts directly.
- The two constructs Scena rejects (`pi`, non-integer `pow`) are reported
  with the rule they violate, so a user sees what to change rather than a
  silently different result. Both are recorded in the coverage matrix.
- p4-s4's catalogs get the mechanism they need: a catalog entry's
  `ParameterDeclarations` push a frame, and a `CatalogReference`'s
  `ParameterAssignments` will fill it before the entry is read.
- The p4-s2 workaround — deferring any declaration whose value contained a
  `$` — is gone; those declarations now resolve.
