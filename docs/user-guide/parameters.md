# Parameters, expressions and variables

Real scenario files are written once and reused many times, which is what
parameters are for. This chapter covers the three related mechanisms:
**parameters** (§9.1, fixed when the file loads), **expressions** (§9.2,
evaluated at load time), and **variables** (§6.12, scenario state that
changes while the run proceeds).

## Parameters

A `ParameterDeclaration` names a typed value:

```xml
<ParameterDeclarations>
  <ParameterDeclaration name="ego_speed" parameterType="double" value="30.0"/>
  <ParameterDeclaration name="lane"      parameterType="int"    value="-1"/>
</ParameterDeclarations>
```

Any attribute may then reference it with a `$` prefix:

```xml
<AbsoluteTargetSpeed value="$ego_speed"/>
```

Parameter values are evaluated and set **at load time and cannot change
afterwards** (§9.1). A `ParameterCondition` therefore has the same result in
every step of a run — parameters configure a scenario, they do not drive it.

### Scope

"The scope of a parameter is the subtree rooted in the element where the
`ParameterDeclaration` is located", and where scopes overlap "only the
parameter with the smallest scope that subsumes the location is accessible".
Scena implements exactly that: declarations at the root of the document are
global, and declarations on a `Story`, `Maneuver`, `Trajectory`, `Route`,
`Environment` or an entity object are visible only inside that element,
shadowing an outer declaration of the same name.

A reference to a name that is not in scope is a `Status::SemanticError`
citing `asam.net:xosc:1.1.0:parameters.parameter_declaration_parameter_scope`.

### Names and types

Names must match `[A-Za-z_][A-Za-z0-9_]*` (rule
`naming.parameter_declaration_parameter_name`), and names beginning with
`OSC` are reserved by the standard — Scena warns rather than rejecting, since
the standard phrases it as a *should*.

The declared `parameterType` is checked against the value: a value that is
not of its type is rejected citing
`parameters.parameter_declaration_parameter_type_inference`. The types are
`int` (spelled `integer` before 1.2), `unsignedInt`, `unsignedShort`,
`double`, `boolean`, `string` and `dateTime`.

### Value constraints

A declaration may carry `ConstraintGroup`s (§9.1). A group holds
`ValueConstraint`s that must **all** hold; several groups are **alternatives**,
so the declared value must satisfy at least one of them:

```xml
<ParameterDeclaration name="speed" parameterType="double" value="30.0">
  <ConstraintGroup>
    <ValueConstraint rule="greaterThan" value="0"/>
    <ValueConstraint rule="lessOrEqual" value="50"/>
  </ConstraintGroup>
</ParameterDeclaration>
```

## Expressions

`${...}` computes a value at load time (§9.2):

```xml
<AbsoluteTargetSpeed value="${$ego_speed * 1.2}"/>
```

The supported operators are exactly the whitelist of rule
`asam.net:xosc:1.1.0:expressions.allowed_operators`, with the precedence of
§9.2.1:

| Precedence | Operators |
|---|---|
| Highest | unary `-`, `round`, `floor`, `ceil`, `sqrt`, `sin`, `cos`, `tan`, `asin`, `acos`, `atan`, `sign`, `abs`, `max`, `min` |
| Middle | `pow`, `*`, `/`, `%` |
| Lowest | `+`, `-` |

and, for booleans, `not` binding tighter than `and`, which binds tighter than
`or`. Named operators take their arguments in parentheses after the name
(rule `arguments_of_operators`), and brackets nest as usual.

Typing follows §9.2.3:

- Integer arithmetic stays integral (`${7 % 2 + 3 * 2}` is the integer `7`);
  `/` is the double division, so it always yields a double.
- An integer widens to a double implicitly; a double never narrows to an
  integer (rule `type_casting`).
- Mixing an arithmetic value into a boolean position, or the other way
  round, is a type error citing `type_of_boolean`.
- A result that is not finite is rejected citing
  `evaluation_of_expressions_possible`, as are division and remainder by
  zero and `sqrt` of a negative number.

Two constructs are deliberately not available:

- **The `pi` constant** is a 1.4 addition. Scena targets 1.0–1.3, so an
  expression using it is rejected.
- **`pow` with a non-integer exponent.** An integer exponent is exact
  repeated multiplication; a general power needs a platform `exp`/`log`
  pair, whose results differ between platforms and would break
  [bit-identical determinism](determinism.md) before the engine ever runs.
  Scena reports it instead of approximating it.

For the same reason, `sin`, `cos`, `tan`, `asin`, `acos` and `atan` are
evaluated through the deterministic math layer, not libm: an expression
result reaches the Scenario IR, and everything derived from it must be
identical on every platform.

## Variables

A `VariableDeclaration` (§6.12, 1.2 and later) declares **runtime state**:

```xml
<VariableDeclarations>
  <VariableDeclaration name="phase" variableType="string" value="idle"/>
</VariableDeclarations>
```

The initial value is read at load time — so it may itself be a parameter
reference or an expression — but a variable is **not** an expression input:
its value changes while the scenario runs, so referencing one from an
attribute is an undeclared-parameter error rather than a silent read of the
initial value. Variables are read and written by `VariableCondition`,
`VariableSetAction`, `VariableModifyAction`, and by the host through
`Engine::set_variable`.

The difference in one line: a **parameter** configures a scenario before it
starts; a **variable** carries state through it.

## See also

- [Loading scenarios](loading-scenarios.md) — the loader, versions and
  diagnostics.
- [Conditions](conditions.md) — `ParameterCondition` and `VariableCondition`.
- [ADR-0022](../architecture/ADR-0022-parameters-and-expressions.md) — why
  expression evaluation is deterministic and what it excludes.
