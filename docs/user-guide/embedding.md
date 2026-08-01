# Embedding Scena

Scena is a library, not an application. The host owns the clock, the entities it
already simulates, and the road network; Scena owns the scenario. This page is
about the seam between them.

## The step contract

```
init  →  step(dt)  →  query / report state  →  …  →  close
```

The engine spawns no threads, imposes no main loop, and never reads a wall
clock. `step(dt)` is the only thing that advances time, and the host decides
when and by how much.

Every step runs the same phases in the same order (ADR-0003):

1. `on_step_begin(dt)`
2. the clock advances to `t + dt`
3. host-controlled entities are **polled** from the gateway
4. derived observations are refreshed (skipped when `dt == 0`)
5. the storyboard is evaluated — triggers, actions, completion
6. storyboard transitions are reported through `on_element_transition`
7. engine-controlled entities integrate their motion
8. engine-controlled entities are **published** to the gateway
9. `on_step_end(dt)`

The order is a contract, not an implementation detail. Changing it is an
ADR-level decision.

## Control ownership

Each entity is driven either by the engine or by the host, declared per entity:

| Mode | Who moves it | Gateway traffic |
|---|---|---|
| `EngineControlled` | Scena integrates its motion from the scenario | `publish_state` each step |
| `HostControlled` | the host does | `poll_state` each step |

An entity is never both. A host that tries to teleport an entity it does not own
gets `InvalidControlMode` rather than a silently ignored write (ADR-0017).

Without a gateway the same exchange happens through `Engine::state()` and
`Engine::report_state()` directly — the gateway is a convenience for hosts that
prefer to be called, not a requirement.

## The gateway

```cpp
class MyGateway final : public scena::gateway::ISimulatorGateway {
    void publish_state(const std::string& id, const scena::EntityState& s) override;
    bool poll_state(const std::string& id, scena::EntityState& out) override;
    scena::gateway::IRoadQuery* road_query() override;   // nullptr if you have no map

    // All of these are optional — they default to no-ops.
    void on_step_begin(double dt) override;
    void on_step_end(double dt) override;
    void on_element_transition(const std::string& path, ElementState, TransitionKind) override;
    void on_custom_command(const std::string& type, const std::string& content) override;
    void on_controller_assigned(const std::string& id, const scena::ir::Controller&) override;
    void on_visibility_changed(const std::string& id, const scena::EntityVisibility&) override;
};

MyGateway gateway;
scena::Engine engine(&gateway);      // or engine.set_gateway(&gateway) later
```

**Batching.** `on_step_begin` / `on_step_end` bracket everything the gateway
sees in a step. A host that writes into a scene graph, a shared buffer or a
network frame opens it at the start and commits at the end. A step the engine
rejects (an invalid `dt`, an uninitialized engine) opens no bracket, so
open/close always pair.

**Observing the storyboard.** `on_element_transition` reports every element that
transitioned this evaluation, so a host does not have to poll each element it
cares about. `path` is the element's name path from the story down joined with
`/` (the empty string is the storyboard); `state` is the state *after* the
transition. The order is document order, depth first, **parents before their
children** — a stable traversal, not a causal one, so an act's end is reported
before the event's that caused it.

**Do not call back into the engine from a callback.** They run synchronously
inside `step()`, and a reentrant call would observe a half-built step. React
between steps, through the setters.

## Time sourcing

Three host-clock patterns are supported and tested.

**Fixed dt.** The simplest and the one the golden scenarios use. Bit-identical
across platforms.

**Variable dt.** Legal and supported. The engine integrates explicitly, so two
*different* dt sequences do not generally agree mid-ramp — the same is true of
any explicit integrator. What is guaranteed is that a host replaying the *same*
dt sequence reproduces the same numbers exactly. Determinism is reproducibility
of a run, not independence from the step size.

**Zero-dt query steps.** `step(0.0)` is a real evaluation: triggers are
re-checked, actions can fire, states are polled and published, transitions are
reported, and the brackets run. What does *not* happen: the clock does not
advance, no motion is integrated, and the derived observations (acceleration,
odometer, standstill timer) are deliberately left untouched — differencing over
a zero interval would be a division by zero, and the standstill timer would
accrue time that did not pass. Use it to let the storyboard react to something
the host just reported.

## Embedding from C

The same surface, as a struct of optional function pointers:

```c
static void on_transition(void* user, const char* path,
                          scn_element_state state, scn_element_transition t) { /* … */ }

scn_callbacks cb = {0};          /* zero-init: fields added later stay NULL */
cb.user_data = my_host;
cb.publish_state = &on_publish;
cb.poll_state = &on_poll;
cb.on_element_transition = &on_transition;
scn_engine_set_callbacks(engine, &cb);
```

Every member may be NULL, which behaves exactly like no gateway for that hook.
The struct is copied, so the caller's copy need not outlive the call; whatever
`user_data` points at must outlive the engine. Pass NULL to remove the
callbacks.

One thing deliberately does not cross into C: **`road_query()`**. `IRoadQuery`
is a geometry service with a wide surface and no natural C representation, and a
C host that has a road network is better served by implementing the C++
interface than by marshalling every query through function pointers.

See [The C API](c-api.md) for the rest of the ABI and
`capi/tests/c_consumer.c` for a complete pure-C embedding.

## Determinism and the host

The host is part of the determinism equation. Scena guarantees that an identical
scenario plus an identical step sequence plus identical host inputs produce
bit-identical results, on every platform. The parts the host owns:

- **the dt sequence** — replay it to reproduce a run;
- **reported states** for host-controlled entities — a host replaying identical
  states reproduces identical runs;
- **`IRoadQuery` answers**, which must be deterministic themselves (the contract
  suite in `core/tests/support/road_query_contract.h` checks a backend for this);
- **not calling back into the engine** from a callback.

See [Determinism](determinism.md) for the engine's half of the contract.

## Further reading

- [ADR-0003](../architecture/ADR-0003-simulator-gateway.md) — the gateway
  decision and its amendments, including the p6-s2 additions on this page.
- [Positions and control ownership](positions.md) — the report_state round-trip
  and what a mode violation looks like.
- [Roads](roads.md) — the `IRoadQuery` surface a host implements to give the
  scenario a map.
