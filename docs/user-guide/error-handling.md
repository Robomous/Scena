# Error handling

Scena reports failures in two complementary ways:

- a **`Status` code** returned by every engine operation — the immediate
  yes/no answer the caller checks; and
- **structured diagnostics** — an ordered list of findings the engine
  collected, each with a severity, a machine-readable code, a
  human-readable message, an element path, and (for standards violations)
  the ASAM checker rule id.

No exceptions cross the C++ API, the C ABI, or the Python boundary; this is
a hard contract (ADR-0001).

## Status codes

`scena::Status` (`scn_status` in C, `scena.Status` in Python) is the result
of every fallible call:

| Status | Meaning |
|---|---|
| `Ok` | Success. |
| `AlreadyInitialized` | `init()` called on an initialized engine. |
| `NotInitialized` | `step`/`close`/`report_state` before `init()`. |
| `UnknownEntity` | A queried or reported entity id does not exist. |
| `InvalidControlMode` | `report_state()` on an engine-controlled entity. |
| `InvalidArgument` | **Host** API misuse: a null argument, a negative or NaN `dt`, an out-of-range enumerator. Never used for defects in scenario content. |
| `ParseError` | A frontend could not parse the source. Reserved — no frontend exists yet. |
| `ValidationError` | Scenario **content** violates a structural rule (an empty or duplicate name, a missing required child, a negative delay). |
| `SemanticError` | Scenario content references something that does not exist (an action or actor targeting an undeclared entity). |
| `UnsupportedFeature` | A construct the engine does not implement yet. Emitted at runtime as a warning; never returned by `init()`. |

The split matters: `InvalidArgument` is *your* mistake as a host,
`ValidationError`/`SemanticError` are defects in the *scenario*. A tool that
loads user scenarios routes the two differently — the first is a bug, the
second is feedback for the scenario author.

## Diagnostics

`init()` walks the whole scenario in **document order** and records a
diagnostic for **every** defect it finds — it does not stop at the first.
`init()` then returns the code of the first error diagnostic. Read the full
list afterwards:

```python
engine = scn.Engine()
status = engine.init(scenario)
if status != scn.Status.Ok:
    for d in engine.diagnostics():
        print(f"{d.severity.name} [{d.code.name}] {d.path}: {d.message}"
              + (f"  ({d.rule_id})" if d.rule_id else ""))
```

Each `Diagnostic` carries:

| Field | Meaning |
|---|---|
| `severity` | `Info`, `Warning`, or `Error`. |
| `code` | The `Status` category (reused, not a parallel enum). |
| `message` | A human-readable description. Deterministic by design: names, ids, and indices only — never a floating-point value. |
| `path` | The element the finding is anchored to (grammar below); empty addresses the whole scenario. |
| `location` | Source `file`/`line`/`column`. Unknown (empty/0) for scenarios built in code; a frontend fills it in (F1). |
| `rule_id` | The ASAM checker rule UID, when the standard defines one; empty otherwise. |

### The severity / status invariant

An operation that emitted an **Error** diagnostic returned a non-`Ok`
status; an operation that emitted only **Warning** or **Info** returned
`Ok`. A degraded scenario is not a broken engine.

### Runtime diagnostics never halt a run

`step()` always returns `Ok`. If, at runtime, an action targets a missing
entity or names a kind the engine does not implement yet, the engine emits
a **Warning** and continues — it never silently drops the action, and never
stops the simulation. Runtime warnings accumulate in the same list as the
init-time findings, in the order the steps produced them.

### Path grammar

Paths extend the `/`-joined addressing used by
`storyboard_element_state`:

| Element | Path |
|---|---|
| Storyboard element | `story/act/group/maneuver/event` |
| Entity | `entities/<id>` — or `entities[<index>]` when the id is empty |
| Init action | `init/action[<index>]` |
| Trigger condition | `<owner>/startTrigger` or `stopTrigger` `/group[<i>]/condition[<j>]`; the condition's `name` replaces `condition[<j>]` when it has one; the storyboard's own stop trigger is just `stopTrigger` |
| Event action | `<event>/action[<index>]` |

## Lifetime and clearing

- `init()` clears the diagnostics **after** the `AlreadyInitialized` guard:
  a rejected re-init preserves the previous record; a successful init starts
  clean.
- `close()` does **not** clear — a finished or failed run stays inspectable.
- `clear_diagnostics()` empties the list on demand.
- The Python `Engine.diagnostics()` returns a **list of copies**, so it is
  safe to hold across `close()` and re-init.

### C ABI note (borrowed strings)

Through the C ABI the diagnostic is read back into a transparent
`scn_diagnostic` whose string members are **borrowed** from the engine —
never `NULL` (absent fields are `""`). They stay valid until the next
`scn_engine_init`, `scn_engine_step`, `scn_engine_clear_diagnostics`, or
`scn_engine_destroy` on that engine. Copy them out before calling any of
those.

```c
size_t count = 0;
scn_engine_diagnostic_count(engine, &count);
for (size_t i = 0; i < count; ++i) {
    scn_diagnostic d;
    if (scn_engine_diagnostic_at(engine, i, &d) == SCN_OK) {
        printf("%s [%d] %s: %s\n", d.rule_id, d.code, d.path, d.message);
    }
}
```
