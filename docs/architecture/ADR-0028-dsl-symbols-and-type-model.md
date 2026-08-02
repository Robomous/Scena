# ADR-0028: The DSL symbol table and type model

- **Status:** accepted
- **Date:** 2026-08-01

## Context

The DSL frontend needs a type model before it can check anything: p7-s3 resolves
names and applies the §7.3 rules, p7-s4 types expressions and constraints, p7-s5
checks the whole standard library, and P8 lowers what survives into the shared
Scenario IR. Four passes over the same data, so its shape is a decision worth
recording rather than discovering four times.

Three properties of the language force the design:

- **Declare-anywhere.** §7.3.15: "there are no restrictions on the ordering of
  type use and type declaration in terms of textual ordering", and the same for
  units. A single pass cannot resolve names.
- **Extension modifies a type.** §7.3.9: `extend car:` adds members to `car`
  itself, everywhere, including from another file and before `car` is declared.
  So a type is not one declaration — it is the union of all of them.
- **Load time is inside the determinism contract.** The XML frontend already
  established this (ADR-0021): values resolved at load reach the IR, so two runs
  over the same sources must produce the same tables in the same order on every
  platform.

## Decision

**Types are values in a vector, addressed by a `TypeId` index.** Not pointers,
not `shared_ptr`. The table grows across passes — resolving `list of int`
creates a type — and an index survives that where a reference does not. It also
removes address-dependent ordering from the frontend by construction, which is
what the determinism contract asks for.

**Members live in `std::map`, with a parallel `*_order` vector.** The map gives
ordered iteration and lookup; the order vector keeps declaration order for the
two places where order *is* meaning: positional arguments (§7.2.2.5.2) and
enumeration value succession (§7.3.3). No `unordered_map` anywhere in the
frontend.

**A type accumulates its declarations.** `TypeInfo::declarations` holds the
original declaration and every `extend` that contributed, in textual order —
which is exactly the order §7.3.15 prescribes for resolving overrides among
extensions with the same inheritance relationship.

**Resolution is three passes.** *Declare* enters every name into its namespace
and attaches extensions to what they extend. *Link* resolves supertypes,
conditional guards, unit dimensions, enum values, modifier association and
behavior actors. *Members* types fields, methods and events, and checks them
against what a supertype already declares. Anything that needs a supertype's
members — the conditional-inheritance determinant — runs at the end of the third
pass, not the second.

**Aggregates are structural.** `list of int` is one type wherever it is written,
created on demand and shared, so two fields declared with the same declarator
compare equal by `TypeId`.

**Physical types are checked by dimension, not by name.** A `SiDimension` is the
exponent vector of §7.3.4's seven SI base units plus the radian. A unit must
carry its physical type's exponents exactly — the specification says every unit
of a type has identical exponents, and that is what makes
`base_value = value * factor + offset` a legal direct conversion.

**Diagnostics cite sections, never rule ids.** The DSL standard defines no
`asam.net:` rule ids at all, so `Diagnostic::rule_id` is always empty from this
frontend, and every message names the clause it enforces.

**The prelude is DSL source.** The built-in physical types and units are a
string of `.osc` that goes through the same lexer, parser and resolver as user
code, rather than hand-built `TypeInfo`s. A bug in any of the three shows up in
the prelude first, and p7-s5 grows the same mechanism into `osc.standard`.

## Consequences

- p7-s4 can type expressions against `TypeId`s without rebuilding anything, and
  P8 can lower from a table it does not have to re-derive.
- A `TypeId` is stable only within one `Program`; nothing serialises it.
- Checking a default value is split: p7-s3 checks a *literal* default, where a
  wrong unit or a wrong enum name already shows, and leaves general expression
  typing to p7-s4. The seam is deliberate and is documented on `resolve()`.
- Reference invalidation is a real hazard with a growing vector: helpers take a
  `TypeId` and re-index after any call that can resolve a declarator. This is
  written down because it is the kind of bug that only shows under a sanitizer.
- Import resolution is not here. The parser records the reference; p7-s5 wires a
  file loader and the standard library to it.
