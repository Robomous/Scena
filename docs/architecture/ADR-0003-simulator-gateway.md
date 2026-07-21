# ADR-0003: Simulator gateway — step API, control ownership, road queries

- **Status:** accepted
- **Date:** 2026-07-21

## Context

Kinema must embed into host simulators with very different runtime models
(fixed-step, variable-step, co-simulation). The engine therefore cannot own
the clock, spawn threads, or assume how entities are moved: some are driven by
scenario logic, others by the host's own vehicle models or traffic.

## Decision

### Step-based API contract

The engine exposes exactly this lifecycle (`kinema::Engine`, mirrored by the
C ABI):

```
init(scenario) → step(dt) … step(dt) → state()/report_state() between steps → close()
```

- **The host owns the clock.** `step(dt)` advances simulated time by exactly
  `dt`; the engine never reads a wall clock and never blocks.
- **No internal threads, no imposed main loop.** All engine work happens
  synchronously inside `step()`.
- **No exceptions cross the public API.** All fallible operations return
  status codes.
- Within one step the order is fixed: clock advance → poll host-controlled
  states → evaluate storyboard conditions and fire due actions → integrate
  engine-controlled entities → publish engine-controlled states.

### Determinism guarantee

Identical scenario + identical step sequence ⇒ **bit-identical entity
states**. This is what makes scenario runs reproducible and testable across
hosts. It is enforced by construction: pure accumulation clock, no wall-clock
or randomness in the runtime, deterministic iteration order for entity
updates. A dedicated determinism test suite guards the property.

### Per-entity control ownership

Each entity is either:

- **Engine-controlled** (default): the engine integrates its motion from
  scenario actions each step and pushes the result to the host.
- **Host-controlled**: the host reports the entity's authoritative state each
  step (via `report_state()` or the gateway); the engine never integrates it,
  but scenario conditions still observe it.

Ownership is declared per entity in the scenario IR and honored by both the
engine loop and the gateway exchange.

### Gateway and road queries

`gateway::ISimulatorGateway` is the integration boundary: `publish_state()`
pushes engine-controlled states to the host, `poll_state()` pulls
host-controlled states from it. The gateway is optional — without one the
engine runs self-contained and the host uses the engine API directly.

Road-network access is abstracted behind `gateway::IRoadQuery`
(lane-relative positioning in ASAM OpenDRIVE s/t coordinates). The runtime
programs against this interface only; no concrete road implementation exists
in this phase, and none will live inside the core.

## Consequences

- Any host that can call C functions at a fixed cadence can embed the engine;
  co-simulation and lockstep testing come for free with determinism.
- Placeholder physics (straight-line kinematics) lives behind the same step
  contract, so replacing it in later phases does not change the API.
- The gateway keeps the core free of platform dependencies, at the cost of one
  indirection per entity per step — acceptable at scenario entity counts.
- Host-controlled entities make the host part of the determinism equation: a
  host replaying identical reported states reproduces identical runs.
