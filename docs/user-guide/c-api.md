# The C API

`capi/include/scena/capi.h` is Scena's stable ABI: opaque handles, `scn_`
prefixed symbols, no C++ types in the header, and a `scn_status` from every
function. A pure-C consumer (`capi/tests/c_consumer.c`) compiles and runs
against it on macOS, Linux and Windows in CI, which is what keeps it C-clean.

Everything the C++ engine exposes is reachable from C.

## Versioning

Two different things have versions, and they move independently.

| Symbol | Meaning |
|---|---|
| `scn_version()` | the product version, `"major.minor.patch"` |
| `scn_abi_version()` | the ABI version, `major * 10000 + minor` |
| `SCN_ABI_VERSION` | the ABI version this header declares |

The ABI **minor** increases when symbols, enumerators or trailing struct fields
are *added* — none of which breaks a consumer built against an older minor. The
**major** increases only if an existing symbol's meaning, signature or layout
changes, which is the thing this ABI exists to avoid.

A host that loads the library dynamically checks the major before calling
anything else:

```c
if (scn_abi_version() / 10000u != SCN_ABI_VERSION / 10000u) {
    /* built against an incompatible ABI */
}
```

## Lifecycle

```c
scn_engine* engine = scn_engine_create();          /* NULL on allocation failure */
scn_engine_load_xml_file(engine, "scenario.xosc"); /* or build it with scn_engine_add_* */
for (int i = 0; i < 1000; ++i) {
    scn_engine_step(engine, 0.01);
}
scn_engine_close(engine);
scn_engine_destroy(engine);                        /* NULL is a no-op */
```

`scn_engine_destroy(NULL)` is a no-op, and every other function rejects a NULL
handle with `SCN_ERROR_INVALID_ARGUMENT` rather than crashing. So does every
NULL out-parameter: an out parameter is never written unless the call returns
`SCN_OK`.

## Loading a scenario

```c
scn_status status = scn_engine_load_xml_file(engine, path);   /* from disk    */
scn_status status = scn_engine_load_xml_string(engine, text); /* from memory  */
```

Both load an OpenSCENARIO XML document **and** initialize the engine with it —
the C equivalent of `scena::xml::load_file` followed by `Engine::init`. Findings
from both halves land in the same diagnostic list, in the order they happened,
whatever the return value: from C there is one call, so there is one place to
look afterwards. A load that fails leaves the engine uninitialized.

An in-memory document has no file to resolve relative catalog and road-network
paths against, so a document that needs them reports rather than guessing.

A scenario can also be built entirely from C with the `scn_engine_add_*`
builders and then started with `scn_engine_init`; the two routes are
interchangeable, and the entity metadata accessors read either.

## Querying the running scenario

```c
size_t count;
scn_engine_entity_count(engine, &count);
for (size_t i = 0; i < count; ++i) {
    const char* id;
    scn_engine_entity_id_at(engine, i, &id);   /* borrowed — copy before the next call */
    scn_entity_state state;
    scn_engine_get_state(engine, id, &state);
}
```

Entity ids come back in ascending order, so an index names the same entity on
every platform and across steps. The list includes entities a
`DeleteEntityAction` has removed; `scn_engine_entity_active` tells them apart.

Storyboard elements are addressed by their name path from the story down, joined
with `/`; the empty string addresses the storyboard itself:

```c
scn_element_state state;
scn_engine_element_state(engine, "story/act/group/maneuver/event", &state);

scn_element_transition transition;
scn_engine_element_transition(engine, "story/act/group/maneuver/event", &transition);
```

A transition is a **one-evaluation pulse**: it reports what happened in the most
recent evaluation, not a lasting state. A path that names no element returns
`SCN_ERROR_UNKNOWN_NAME`.

`scn_engine_get_time` reads the simulation time and `scn_engine_initialized`
reports whether the engine is between a successful init and close.

## Borrowed strings

Every accessor that hands back a `const char*` — `scn_engine_get_variable`,
`scn_engine_get_user_defined_value`, `scn_engine_entity_id_at`,
`scn_engine_traffic_signal_state`, `scn_engine_entity_controller_name`,
`scn_diagnostic`'s fields — returns a pointer into a buffer owned by the engine.
It is valid **until the next such call on this engine**, or any mutating call
(`step`, `init`, `close`, `destroy`). Copy it out before then. Walking the entity
list means copying each id as you go.

See [Error handling](error-handling.md) for the full lifetime rule and the
`scn_diagnostic` layout.

## Traffic signals

The engine writes signal states from controller phases and
`TrafficSignalStateAction`s. A host that owns the real signals can publish into
the same store:

```c
scn_engine_set_traffic_signal_state(engine, "signal_17", "green");
const char* state;
scn_engine_traffic_signal_state(engine, "signal_17", &state);
```

Signal ids are free-form road-network references, so any name is accepted. A
controller phase naming the same signal overwrites it on its next transition,
exactly as it overwrites an action's write.

## Checking OpenSCENARIO DSL

`scn_check_dsl_file` and `scn_check_dsl_string` check a DSL source and its
imports without constructing an engine — checking is a frontend service, which
is why the result is its own `scn_dsl_check` handle rather than an engine's
diagnostic list. Release it with `scn_dsl_check_destroy`, whatever the status: a
failing check still produces one. See
[`scena-check`](scena-check.md#checking-from-c-and-python) for the worked
example, and note that a zero-initialized `scn_dsl_check_options` turns the
standard library *off* — pass `NULL` to get the defaults instead.

## What is not in C

The C ABI carries data, not C++ objects. A `Controller`'s property list, an
`ir::Route`'s full geometry and the scenario IR itself stay in C++ and Python;
C sees the queries a host loop actually needs. Nothing in `capi.h` includes a
C++ header, and nothing in it names a C++ type.

## Further reading

- [Error handling](error-handling.md) — status codes, diagnostics and the
  borrowed-string lifetime.
- `capi/tests/c_consumer.c` — a complete, runnable pure-C embedding.
- `core/tests/capi_test.cpp` — the behavioural suite, including the
  null-argument sweep every entry point passes.
