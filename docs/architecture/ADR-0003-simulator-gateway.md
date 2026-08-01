# ADR-0003: Simulator gateway — step API, control ownership, road queries

- **Status:** accepted
- **Date:** 2026-07-21

## Context

Scena must embed into host simulators with very different runtime models
(fixed-step, variable-step, co-simulation). The engine therefore cannot own
the clock, spawn threads, or assume how entities are moved: some are driven by
scenario logic, others by the host's own vehicle models or traffic.

## Decision

### Step-based API contract

The engine exposes exactly this lifecycle (`scena::Engine`, mirrored by the
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

## Amendments

- **p5-s5 (ADR-0014):** `on_controller_assigned` / `on_visibility_changed`
  defaulted callbacks; **p5-s6 (ADR-0015):** `on_custom_command`. Documented
  in those ADRs; listed here for completeness.
- **p3-s1 — IRoadQuery v1 frozen.** `gateway::IRoadQuery` is extended to its
  v1 surface and frozen: lane-relative ↔ world conversions plus road heading,
  lane queries (existence, width, centre offset, type, relative-lane
  arithmetic), s-range queries (road length, lane s-range), and the route
  interface (`RouteSpan`, `build_route`, `position_along_route`). Every query
  reports through a `bool` return where `false` uniformly means "no answer"
  — off-network input, unknown id, non-finite input, or an unsupported query;
  backends never throw across the boundary and must answer deterministically.
  New-in-v1 queries are defaulted to `false` so pre-freeze host
  implementations keep compiling; the two conversions stay pure.
  `gateway::FlatWorldRoadQuery` is the null-object backend for road-free
  scenarios. The runtime consumes road data only through this header;
  changing the surface from here on is an ADR-level decision. The executable
  contract every backend must pass lives in
  `core/tests/support/road_query_contract.h`.
- **p6-s2 — `ISimulatorGateway` v1: step brackets, storyboard observation, and
  the C ABI's gateway.** Three additions, all defaulted to no-ops so every
  existing implementation keeps compiling and behaving identically.

  - **`on_step_begin(dt)` / `on_step_end(dt)`** bracket each step. They are the
    *batching* hook: a host that writes into a scene graph, a shared buffer or a
    network frame opens it at the start and commits at the end, receiving the
    step's `poll_state` / `publish_state` calls in between. The per-entity
    contract and the step order above are unchanged — the brackets only enclose
    them. A rejected step (invalid dt, uninitialized engine) opens no bracket, so
    a host can rely on open/close pairing. A zero-dt step *does* bracket: it is a
    real evaluation.
  - **`on_element_transition(path, state, transition)`** reports every storyboard
    element that transitioned in the step's evaluation, so a host observing the
    storyboard does not have to poll every element it cares about. The order is
    part of the deterministic run: document order, depth first, **parents before
    their children**, reported after the evaluation completes and before entity
    motion is integrated. A transition is a one-evaluation pulse, so a host that
    steps once sees each exactly once. Parents-before-children means an act's
    `endTransition` is reported before the event's that caused it; the order is
    a stable traversal, not a causal one, and the state each callback carries is
    the state after the transition, so nothing is ambiguous.
  - **`Engine::set_gateway` / `gateway()`** make attachment a runtime operation
    rather than a constructor-only one. The engine reads the pointer only at the
    points this ADR fixes, so a swap between steps cannot be observed
    half-applied.

  **Time sourcing.** The host owns the clock, and three patterns are supported
  and tested: a **fixed dt** loop; a **variable dt** loop, where the explicit
  integrator means two different dt sequences do not generally agree mid-ramp
  but a replayed sequence reproduces exactly (that, not dt-independence, is the
  determinism contract); and **zero-dt query steps**, where a host re-evaluates
  the storyboard without advancing time — the clock does not move, no motion is
  integrated, the derived observations are deliberately not updated (no 0/0),
  and the brackets, polls, publishes and transitions all still happen.

  **The C ABI's gateway is function pointers.** `scn_callbacks` mirrors this
  interface as a struct of optional function pointers plus a `void* user_data`,
  installed with `scn_engine_set_callbacks`; the C API owns an adapter that
  implements `ISimulatorGateway` and forwards. Every member may be NULL, which
  behaves exactly like no gateway for that hook. One thing deliberately does not
  cross: **`road_query()` stays C++-only**. `IRoadQuery` is a geometry service
  with a wide surface and no natural C representation, and a C host that has a
  road network is better served by implementing the C++ interface directly than
  by marshalling every query through function pointers.
