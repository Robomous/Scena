<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p4-s3] Parameters, expressions & variables

**Pillar:** P4 — OpenSCENARIO XML Frontend · **Roadmap:** `docs/roadmap/roadmap.md` § p4-s3

## Goal
The reuse machinery that makes real-world files load.

## Deliverables
- Frontend: `ParameterDeclaration` scoping and `ValueConstraint` groups per XML §9.1; `$`-references.
- Frontend: `${...}` expression evaluation per §9.2 — operator whitelist, constants, typing, NaN/overflow rejection per the `asam.net:xosc:1.1.0:expressions.*` rules; the 1.4-only constant `pi` rejected for 1.0–1.3.
- Frontend: parameter type checking; `ParameterAssignments` at reference sites.
- Kernel: runtime variable store (`VariableDeclaration`, §6.12 — variables are runtime state, not expression inputs).

## Tests
- `xml_expression_test.cpp` (operator/precedence matrix, error diagnostics citing rule IDs).
- `variable_store_test.cpp` (kernel).

## Docs
- Parameters page in the user guide.

## Exit criteria
- [ ] Expression matrix green
- [ ] Diagnostics carry rule IDs
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p4-s2
