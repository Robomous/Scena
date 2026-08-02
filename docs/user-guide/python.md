# Python quickstart

The Python package wraps the same engine the C++ and C surfaces drive. Anything
you can do from C++ you can do from Python; a
[parity audit](#parity-with-c-and-c) enforces that in CI.

```sh
python -m pip install ./python
# or, against the build tree:
cmake -B build && cmake --build build --parallel
export PYTHONPATH=build/python
```

## Load a scenario and run it

```python
import scena as scn

status, scenario, diagnostics = scn.load_file_with_diagnostics("scenario.xosc")
assert status == scn.Status.Ok
for d in diagnostics:            # a clean load can still have told you something
    print(f"{d.severity.name}: {d.path}: {d.message}")

engine = scn.Engine()
assert engine.init(scenario) == scn.Status.Ok

for _ in range(500):             # the host owns the clock
    assert engine.step(0.02) == scn.Status.Ok

print(engine.state("ego").speed)
engine.close()
```

Four loaders, two shapes:

| | returns |
|---|---|
| `load_file(path)`, `load_string(xml)` | `(status, scenario)` |
| `load_file_with_diagnostics(path)`, `load_string_with_diagnostics(xml)` | `(status, scenario, diagnostics)` |

Prefer the `_with_diagnostics` forms. A document that loads with `Status.Ok` may
still carry warnings — a deprecated spelling, a construct outside the implemented
subset — and those are exactly the things worth seeing early.

Nothing raises: a malformed document is reported through the status and the
diagnostics, not through an exception. See [Error handling](error-handling.md).

Full example: `python/examples/load_and_run.py`.

## Build a scenario in memory

```python
scenario = scn.Scenario("cut-in")
scenario.add_entity(scn.Entity("ego", "ego", scn.ControlMode.EngineControlled))

ramp = scn.TransitionDynamics(shape=scn.DynamicsShape.Cubic,
                              dimension=scn.DynamicsDimension.Time, value=4.0)
event = scn.Event("go", start_trigger=scn.make_trigger(scn.SimulationTimeCondition(1.0)))
event.add_action(scn.SpeedAction("ego", 20.0, ramp))

maneuver = scn.Maneuver("maneuver"); maneuver.add_event(event)
group = scn.ManeuverGroup("group"); group.add_actor("ego"); group.add_maneuver(maneuver)
act = scn.Act("act"); act.add_group(group)
story = scn.Story("story"); story.add_act(act)
scenario.add_story(story)
```

The whole Scenario IR is bound, so a scenario can be generated, mutated and run
without ever touching XML. `scenario.entities` reads back what was declared,
however the scenario was built.

## Observe a running scenario

```python
for entity_id in engine.entity_ids():          # ascending id order
    state = engine.state(entity_id)
    active = engine.entity_active(entity_id)

engine.storyboard_element_state("story/act/group/maneuver/go")   # Standby/Running/Complete
engine.storyboard_element_transition("story/act/group/maneuver/go")

engine.set_traffic_signal_state("signal_17", "green")
engine.traffic_signal_state("signal_17")

engine.diagnostics()      # a list of copies, safe across close/re-init
engine.time               # simulation seconds since init
```

Full example: `python/examples/host_queries.py`.

## Drive entities from the host

Subclass `scn.SimulatorGateway` and implement only the hooks you need:

```python
class MySimulator(scn.SimulatorGateway):
    def poll_state(self, entity_id):
        # Called for every host-controlled entity, before evaluation.
        # Return an EntityState to update it, or None to leave it alone.
        state = scn.EntityState(); state.x = self.x; state.speed = self.v
        return state

    def publish_state(self, entity_id, state):
        # Called for every engine-controlled entity, after integration.
        self.scene[entity_id] = (state.x, state.y, state.heading)

    def on_step_begin(self, dt): ...   # open a write
    def on_step_end(self, dt): ...     # commit it

engine.set_gateway(MySimulator())
```

Unlike the C++ signature, `poll_state` **returns** a state (or `None`) rather
than filling an out parameter — Python has no out parameters. Everything else
mirrors the C++ interface exactly.

Full example: `python/examples/host_controlled.py`.

## Watch the storyboard

```python
class Events(scn.SimulatorGateway):
    def on_element_transition(self, path, state, transition):
        print(f"{transition.name}: {path or '<storyboard>'} -> {state.name}")
```

Reported in document order, depth first, parents before children, once per
transition. Full example: `python/examples/storyboard_events.py`.

### The GIL and reentrancy

Callbacks run **synchronously inside `init()` and `step()`**, on the calling
thread. The bindings reacquire the GIL for each override, so a Python gateway
works whether or not the host released it around `step()`.

**Do not call back into the engine from a callback** — not `step()`, not a
setter. They would observe a half-built step. React between steps.

An exception raised in a callback propagates out of `step()`. That is
deliberate: the host is part of the determinism equation, and a broken host
should stop the run rather than silently produce a different one.

One thing deliberately does not reach Python: **`road_query()`**. `IRoadQuery` is
queried per entity per step, and routing it through the interpreter would put
Python on the runtime's hot path. Implement the C++ interface for that.

## Check an OpenSCENARIO DSL file

```python
result = scn.check_dsl_file("overtake.osc", search_paths=["lib"])
print(result.status, result.type_count, result.file_count)
for diagnostic in result.diagnostics:
    print(diagnostic.location.file, diagnostic.location.line, diagnostic.message)
```

Checking is not running: DSL execution is P8, so a file that checks clean is one
the frontend understood. `check_dsl_string(source, origin)` does the same for a
source in memory. Both return a `DslCheck`, which is falsy unless the status is
`Ok`. `python/examples/check_dsl.py` is the runnable version, and
[`scena-check`](scena-check.md) documents the options and what "checked clean"
covers.

## Parity with C++ and C

`scripts/parity_audit.py` extracts the public `scena::Engine` methods and the
frontend entry points (`scena::xml`, `scena::dsl`) and checks each against the C
ABI and the Python bindings. Anything missing from either must be listed in the
script's exclusion tables with a reason; anything else is a gap and the script
exits non-zero. `python/tests/test_parity.py` runs it, and CI runs it again on
every platform — including a test that doctors a header to prove the audit still
detects a real gap.

```sh
python scripts/parity_audit.py              # ok/GAP per entry point
python scripts/parity_audit.py --markdown   # the table, for docs or release notes
```

## Further reading

- [Embedding Scena](embedding.md) — the step contract, control ownership and
  the host-clock patterns, in language-neutral terms.
- [The C API](c-api.md) — the same surface from C.
- [Error handling](error-handling.md) — statuses, diagnostics and severities.
