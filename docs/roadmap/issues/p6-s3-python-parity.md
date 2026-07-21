<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p6-s3] Python parity & examples

**Pillar:** P6 — Embedding: Gateway, C API & Python · **Roadmap:** `docs/roadmap/roadmap.md` § p6-s3

## Goal
Python at parity with the C++ surface.

## Deliverables
- Bindings (`python/`): bindings for every p6-s1/p6-s2 capability (callbacks as Python callables, GIL/reentrancy policy documented); typing stubs.
- Examples: `load_and_run.py`, `host_controlled.py`, `storyboard_events.py`.
- `scripts/parity_audit.py` generating the C++/C/Python parity table.

## Tests
- pytest `test_loader.py`, `test_callbacks.py`, `test_parity.py` (audit table has no gaps).

## Docs
- Python quickstart rewrite.

## Exit criteria
- [ ] Parity audit gap-free
- [ ] Examples run in CI on all platforms
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p6-s2
