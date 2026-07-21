<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p6-s1] C ABI expansion to the full engine surface

**Pillar:** P6 — Embedding: Gateway, C API & Python · **Roadmap:** `docs/roadmap/roadmap.md` § p6-s1

## Goal
Everything reachable from C.

## Deliverables
- Bindings (`capi/`): scenario loading (`scn_load_xml_file/_string`); diagnostics iteration; storyboard-element state query by path; entity enumeration + metadata; extended state struct (full pose); variable get/set; signal state get/set; `scn_abi_version()`.
- Opaque handles + `SCN_*` status codes throughout; header stays C-clean.

## Tests
- `capi_test.cpp` grown per function group, incl. a null-argument rejection sweep.
- A pure-C consumer compile check.

## Docs
- C API reference page regenerated.

## Exit criteria
- [ ] Null-safety sweep green
- [ ] Pure-C consumer builds on all platforms
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p1-s4 (grows with P2–P5 per the pillar dependency note)
