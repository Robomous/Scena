<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p7-s2] Parser & AST

**Pillar:** P7 — OpenSCENARIO DSL Frontend · **Roadmap:** `docs/roadmap/roadmap.md` § p7-s2

## Goal
Full-syntax parse to a typed AST.

## Deliverables
- Frontend: parser for DSL §7.2.2 — type declarations (physical types/units, enums, structs, actors, scenarios, actions, modifiers, `extend`); global parameters; structured-type members (events, fields with `with:` blocks, `keep`/`remove_default`, methods, coverage, modifier applications, behavior `on`/`do` with serial/parallel/one_of, labels, invocation `with` blocks, `wait`/`emit`/`call`/`until`); argument lists; the expression grammar.
- AST value types with stable ordering + source ranges.
- Error recovery at sync points.

## Tests
- `dsl_parser_test.cpp` (AST snapshots for self-authored spec-derived files, seeded-error corpus with location assertions).

## Docs
- Coverage matrix rows updated with spec citations (`docs/roadmap/coverage/`).

## Exit criteria
- [ ] AST snapshot suite green
- [ ] Seeded-error corpus suite green
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p7-s1
