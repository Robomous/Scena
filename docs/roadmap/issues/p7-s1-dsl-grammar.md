<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p7-s1] DSL grammar & lexer (ANTLR4)

**Pillar:** P7 — OpenSCENARIO DSL Frontend · **Roadmap:** `docs/roadmap/roadmap.md` § p7-s1

## Goal
Tokens and layout exactly per DSL §7.2.1.

## Deliverables
- Frontend (`frontends/dsl/`): approved ANTLR4 dependency wiring.
- Lexer: identifiers incl. escaped `|...|` form, physical literals with units, int/uint/hex/float/bool/string literals, `#` comments, line continuation.
- Indentation preprocessor (offside rule → INDENT/DEDENT, CRLF-safe, per §7.2.1.4).
- `std::from_chars` numeric conversion.

## Tests
- `dsl_lexer_test.cpp` (token goldens incl. pathological indentation and unit literals).

## Docs
- Frontend README (grammar written from spec §7.2 — clean-room provenance note).

## Exit criteria
- [ ] Lexer goldens green
- [ ] ANTLR4 dependency pinned + licensed
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p1-s1, p1-s4; maintainer ANTLR4 dependency approval before start (open question OQ-2)
