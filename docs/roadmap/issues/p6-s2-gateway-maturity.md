<!-- Issue seed. Created on GitHub only by the maintainer via scripts/roadmap-gh-seed.sh. -->
# [p6-s2] Gateway maturity: callbacks & state injection

**Pillar:** P6 — Embedding: Gateway, C API & Python · **Roadmap:** `docs/roadmap/roadmap.md` § p6-s2

## Goal
Host integration beyond polling.

## Deliverables
- Kernel: `ISimulatorGateway` v1 — batched publish/poll (step order guarantees unchanged per ADR-0003); storyboard event callbacks (element state transitions, deterministic callback order); custom-command callback (p5-s6 hook); road-query injection point formalized; time-sourcing contract documented (fixed dt, variable dt, zero-dt query steps).
- Bindings (C ABI): function-pointer callback registration.

## Tests
- `gateway_test.cpp` (callback ordering determinism, host-clock patterns incl. dt=0).
- capi callback round-trip.

## Docs
- ADR-0003 amendment note.
- Embedding guide page.

## Exit criteria
- [ ] `gateway_test` and capi callback round-trip suites green
- [ ] Embedding guide reviewed
- [ ] Single focused PR merged with CI green (macOS/Linux/Windows, sanitizers, format)

**Depends on:** p6-s1
